// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Bluetooth HCI transport over the Google AOC coprocessor.
 *
 * On Tensor phones from zuma on (Pixel 9 / komodo) the Bluetooth controller of
 * the BCM4390 combo chip is not wired to the AP at all: its HCI UART lands on
 * the AOC (Always-On Compute) coprocessor, whose firmware runs a Pigweed
 * "btproxy" HCI stack and relays H4 traffic to the host over AOC IPC.  The
 * firmware image names the pieces plainly -- platform_bt_uart_driver.cc,
 * uart_[UART_MAP_BT], bt_hci_command_encoder.cc, and
 *
 *	"Bluetooth Proxy reserved %d ACL data credits. Passed %d on to host."
 *	"Resetting proxy on HCI_Reset Command from host."
 *
 * -- and platform_bt_offload_manager.cc sits next to the service names used
 * here.  So this driver is an ordinary HCI transport whose "wire" is a pair of
 * AOC ring services, with controller setup delegated to the shared
 * Broadcom helpers exactly as btusb does.
 *
 * Note the service names say "chre_bt_offload": on this firmware that is the
 * only Bluetooth pipe (the string "com.google.bt" does not appear in the image
 * at all -- that name belongs to other Tensor products).
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/gpio/consumer.h>
#include <linux/ktime.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/soc/google/aoc.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btbcm.h"

/* idx 22 queue, idx 89 and 90 rings in the AOC's debugfs "services". */
#define AOC_BT_CTL_SERVICE	"chre_bt_offload_ctl"
#define AOC_BT_TX_SERVICE	"chre_bt_offload_data_tx"
#define AOC_BT_RX_SERVICE	"chre_bt_offload_data_rx"

#define AOC_BT_RX_CHUNK		512
#define AOC_BT_TX_RETRIES	100

/*
 * EFW framing, recovered from the stock HAL's EfwApTransport
 * (android.hardware.bluetooth-service.bcmbtlinux).  Neither ring is a byte
 * stream: every message is a header plus payload padded out to a 32-byte slot,
 * and the control queue carries a separate fixed-size message with its own type
 * byte.  Bare H4 written into the ring is silently dropped, which is exactly
 * how a controller that never answers presents.
 *
 * Bring-up is a handshake, not an open: drain the control queue, send a pause
 * message, and the AOC answers pause then sync.  The sync carries the geometry
 * (ring size, its own header size and version), and until it arrives the HAL
 * refuses to transmit at all.
 */
#define AOC_BT_AP_HDR_LEN	17	/* ApTxHdrV1: ts u64, len u32, seq u32, type u8 */
#define AOC_BT_SLOT_ALIGN	32
#define AOC_BT_CTL_LEN		48	/* AP -> AOC control message */
#define AOC_BT_CTL_RX_LEN	56	/* read buffer for AOC -> AP messages */
#define AOC_BT_SYNC_TIMEOUT_MS	1000
#define AOC_BT_SYNC_TRIES	6
#define AOC_BT_RX_BUF		8192

/* Control-queue message types (byte 0). */
#define AOC_BT_CTL_PAUSE	0	/* AP -> AOC, 48 zero bytes */
#define AOC_BT_CTL_AOC_PAUSE	1
#define AOC_BT_CTL_AP_READY	2	/* AP -> AOC, answers AOC_PAUSE */
#define AOC_BT_CTL_SYNC		3
#define AOC_BT_CTL_CREDIT	4	/* both ways: u32 byte delta at +1 */
#define AOC_BT_CTL_FATAL	5	/* AOC -> AP: error code at +1, then state */

/* Data-ring message types (ApTxHdrV1 byte 16, AocTxHdr byte 0x1c). */
#define AOC_BT_MSG_DATA		0
#define AOC_BT_MSG_TRANSACT	1	/* request expecting a reply */
#define AOC_BT_MSG_PASSTHROUGH	2	/* data too, AOC proxy bypassed */

/*
 * Every data-ring message opens with a u32 id.  HCI goes out as id 8 followed
 * by a flag byte and the H4 type; it comes back as id 8..11 for event, ACL,
 * SCO and ISO, with no type byte.  Requests to the AOC's BT HAL server are id
 * 39 followed by a FlatBuffer: root { payload_type: ubyte; payload: table },
 * PowerControl being payload type 3 with { on: bool; ?: bool }.
 */
#define AOC_BT_ID_HCI		8
#define AOC_BT_ID_HAL		39
#define AOC_BT_XACT_TIMEOUT_MS	800

static const u8 aoc_bt_power_on[] = {
	0x27, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0e, 0x00,
	0x07, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
	0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x07, 0x00,
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
};

static const u8 aoc_bt_power_off[] = {
	0x27, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00,
	0x07, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
	0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00,
};

/* Sync-message fields, all unaligned. */
#define AOC_BT_SYNC_RING	1	/* u32 data-ring size */
#define AOC_BT_SYNC_HDR		9	/* u32 AocTxHdr size */
#define AOC_BT_SYNC_VER		14	/* u8 AocTxHdr version */

/* AocTxHdr field offsets.  Version 0 has a 24-byte header and no type byte, so
 * every field past +0x10 has to be checked against the reported size. */
#define AOC_BT_FW_SIZE		0x10	/* u32 payload length */
#define AOC_BT_FW_SEQ		0x14	/* u32 */
#define AOC_BT_FW_XACT		0x18	/* u32, the request's seq on a reply */
#define AOC_BT_FW_TYPE		0x1c	/* u8 */
#define AOC_BT_FW_STATUS	0x1d	/* u8, pw::Status of a reply */

enum aoc_bt_state {
	AOC_BT_ST_CREATED,		/* nothing exchanged yet */
	AOC_BT_ST_PAUSED,		/* our pause was answered */
	AOC_BT_ST_READY,		/* sync received, data may flow */
};

struct aoc_bt {
	struct device *dev;
	struct device *aoc_dev;
	struct hci_dev *hdev;
	struct aoc_service *ctl;
	struct aoc_service *tx;
	struct aoc_service *rx;
	struct gpio_desc *reg_on;	/* BT_REG_ON */
	struct gpio_desc *reg_on_latch;	/* clocks BT_REG_ON into the chip */
	struct gpio_desc *dev_wake;
	struct work_struct rx_work;
	struct mutex ctl_lock;		/* the ctl queue has no writer locking */

	/* EFW transport state; the geometry is all learned from the sync. */
	enum aoc_bt_state state;
	struct completion synced;
	u32 ring_size;
	u32 fw_hdr_len;			/* AocTxHdr size the AOC reported */
	u8 fw_hdr_ver;
	u32 tx_write;			/* our offset into the data_tx ring */
	u32 tx_acked;			/* the AOC's, advanced by CREDIT */
	bool tx_full;			/* write caught the ack: ring is full */
	u32 rx_done;			/* bytes consumed, owed back as credit */
	u32 rx_ack_at;			/* return credit at ring_size / 4 */
	u32 seq;
	u8 *rx_buf;			/* de-framing buffer */
	size_t rx_len;

	struct completion xact_done;
	u32 xact_id;
	u8 xact_status;
};

static void aoc_bt_rx_msg(struct aoc_bt *bt, const u8 *data, u32 len)
{
	static const u8 types[] = {
		HCI_EVENT_PKT, HCI_ACLDATA_PKT, HCI_SCODATA_PKT, HCI_ISODATA_PKT,
	};
	struct sk_buff *skb;
	u32 id;

	if (len < 4)
		return;
	id = get_unaligned_le32(data);
	if (id < AOC_BT_ID_HCI || id >= AOC_BT_ID_HCI + ARRAY_SIZE(types)) {
		bt_dev_dbg(bt->hdev, "rx message id %u, %u bytes", id, len);
		return;
	}
	skb = bt_skb_alloc(len - 4, GFP_KERNEL);
	if (!skb)
		return;
	hci_skb_pkt_type(skb) = types[id - AOC_BT_ID_HCI];
	skb_put_data(skb, data + 4, len - 4);
	hci_recv_frame(bt->hdev, skb);
}

/* Ring writes can be short when the AOC is behind; finish the packet. */
static int aoc_bt_write_all(struct aoc_bt *bt, struct aoc_service *svc,
			    const u8 *buf, size_t len)
{
	unsigned int tries = 0;

	while (len) {
		int n = aoc_service_write(svc, buf, len);

		if (n < 0)
			return n;
		if (!n) {
			if (++tries > AOC_BT_TX_RETRIES)
				return -ETIMEDOUT;
			usleep_range(200, 400);
			continue;
		}
		tries = 0;
		buf += n;
		len -= n;
	}
	return 0;
}

static u32 aoc_bt_slot(u32 payload, u32 hdr)
{
	return ALIGN(payload + hdr, AOC_BT_SLOT_ALIGN);
}

static int aoc_bt_ctl_write(struct aoc_bt *bt, const u8 *msg)
{
	int ret;

	mutex_lock(&bt->ctl_lock);
	ret = aoc_bt_write_all(bt, bt->ctl, msg, AOC_BT_CTL_LEN);
	mutex_unlock(&bt->ctl_lock);
	return ret;
}

/* One fixed-size control message.  PAUSE is all zeroes, so val is ignored. */
static int aoc_bt_ctl_send(struct aoc_bt *bt, u8 type, u32 val)
{
	u8 msg[AOC_BT_CTL_LEN] = { };

	if (!bt->ctl)
		return -ENODEV;

	msg[0] = type;
	put_unaligned_le32(val, &msg[1]);
	return aoc_bt_ctl_write(bt, msg);
}

/*
 * The AOC's pause asks the AP to drop whatever data_rx holds and describe its
 * own header; only then does the AOC send the sync.  Bytes 5..7 are what the
 * HAL sends: ApTxHdr version 1, then 3 (meaning unknown), then 0 for "no
 * second rx channel".
 */
static void aoc_bt_ap_ready(struct aoc_bt *bt)
{
	u8 msg[AOC_BT_CTL_LEN] = { AOC_BT_CTL_AP_READY };

	while (aoc_service_can_read(bt->rx) &&
	       aoc_service_read(bt->rx, bt->rx_buf, AOC_BT_RX_BUF) > 0)
		;
	put_unaligned_le32(AOC_BT_AP_HDR_LEN, &msg[1]);
	msg[5] = 1;
	msg[6] = 3;
	if (aoc_bt_ctl_write(bt, msg))
		bt_dev_warn(bt->hdev, "cannot answer the AOC's pause");
}

/* Hand the AOC back credit for the data_rx bytes we consumed, or it fills the
 * ring and stops sending.  The HAL does this once a quarter ring is owed. */
static void aoc_bt_rx_credit(struct aoc_bt *bt)
{
	if (!bt->rx_ack_at || bt->rx_done < bt->rx_ack_at)
		return;

	if (aoc_bt_ctl_send(bt, AOC_BT_CTL_CREDIT, bt->rx_done))
		bt_dev_warn_ratelimited(bt->hdev, "cannot return rx credit");
	bt->rx_done = 0;
}

static void aoc_bt_ctl_msg(struct aoc_bt *bt, const u8 *msg, int len)
{
	if (len < 1)
		return;

	switch (msg[0]) {
	case AOC_BT_CTL_AOC_PAUSE:
		if (bt->state == AOC_BT_ST_CREATED) {
			bt->state = AOC_BT_ST_PAUSED;
			aoc_bt_ap_ready(bt);
		} else
			bt_dev_info(bt->hdev, "unsolicited pause from the AOC");
		break;

	case AOC_BT_CTL_SYNC:
		if (len <= AOC_BT_SYNC_VER) {
			bt_dev_err(bt->hdev, "short sync message (%d bytes)",
				   len);
			break;
		}
		bt->ring_size = get_unaligned_le32(msg + AOC_BT_SYNC_RING);
		bt->fw_hdr_len = get_unaligned_le32(msg + AOC_BT_SYNC_HDR);
		bt->fw_hdr_ver = msg[AOC_BT_SYNC_VER];
		if (bt->fw_hdr_len < AOC_BT_FW_SIZE + 4 ||
		    bt->fw_hdr_len > 64 || !bt->ring_size ||
		    bt->ring_size > SZ_1M) {
			bt_dev_err(bt->hdev,
				   "implausible sync: hdr %u ver %u ring %u",
				   bt->fw_hdr_len, bt->fw_hdr_ver,
				   bt->ring_size);
			bt->fw_hdr_len = 0;
			break;
		}
		bt->tx_write = 0;
		bt->tx_acked = 0;
		bt->tx_full = false;
		bt->rx_done = 0;
		bt->rx_ack_at = bt->ring_size / 4;
		bt->seq = 0;		/* the AOC restarts its count too */
		bt->state = AOC_BT_ST_READY;
		bt_dev_info(bt->hdev,
		    "EFW ready: AocTxHdrV%u %u bytes, ring %u, ApTxHdrV1 %u",
		    bt->fw_hdr_ver, bt->fw_hdr_len, bt->ring_size,
		    AOC_BT_AP_HDR_LEN);
		complete(&bt->synced);
		break;

	case AOC_BT_CTL_CREDIT:
		/* A byte delta against our write pointer, not an absolute. */
		if (len >= 5 && bt->ring_size) {
			bt->tx_acked = (bt->tx_acked +
					get_unaligned_le32(msg + 1)) %
				       bt->ring_size;
			bt->tx_full = false;
		}
		break;

	case AOC_BT_CTL_FATAL:
		bt_dev_err(bt->hdev, "AOC fatal error %u: %*ph", msg[1],
			   min(len, 48), msg);
		break;

	default:
		bt_dev_info(bt->hdev, "ctl type %u (%d bytes): %*ph", msg[0],
			    len, min(len, 32), msg);
		break;
	}
}

/* Peel whole messages off the de-framing buffer.  A read can end mid-slot, so
 * whatever is left over stays for the next round. */
static void aoc_bt_rx_frames(struct aoc_bt *bt)
{
	if (!bt->fw_hdr_len)
		return;

	while (bt->rx_len >= bt->fw_hdr_len) {
		u32 payload = get_unaligned_le32(bt->rx_buf + AOC_BT_FW_SIZE);
		u8 type = bt->fw_hdr_len > AOC_BT_FW_TYPE ?
				bt->rx_buf[AOC_BT_FW_TYPE] : AOC_BT_MSG_DATA;
		u32 slot = aoc_bt_slot(payload, bt->fw_hdr_len);

		if (slot > AOC_BT_RX_BUF) {
			bt_dev_err(bt->hdev,
				   "bad frame: type %u payload %u, resyncing",
				   type, payload);
			bt->rx_len = 0;
			return;
		}
		if (bt->rx_len < slot)
			return;		/* slot not all here yet */

		if (type == AOC_BT_MSG_DATA || type == AOC_BT_MSG_PASSTHROUGH) {
			aoc_bt_rx_msg(bt, bt->rx_buf + bt->fw_hdr_len, payload);
		} else if (type == AOC_BT_MSG_TRANSACT &&
			   bt->fw_hdr_len > AOC_BT_FW_STATUS &&
			   get_unaligned_le32(bt->rx_buf + AOC_BT_FW_XACT) ==
			   bt->xact_id) {
			bt->xact_status = bt->rx_buf[AOC_BT_FW_STATUS];
			complete(&bt->xact_done);
		} else {
			bt_dev_info(bt->hdev, "rx message type %u, %u bytes",
				    type, payload);
		}

		memmove(bt->rx_buf, bt->rx_buf + slot, bt->rx_len - slot);
		bt->rx_len -= slot;
		bt->rx_done += slot;
		aoc_bt_rx_credit(bt);
	}
}

/* Doorbell callback -- interrupt context, so only kick the worker. */
static void aoc_bt_service_irq(struct aoc_service *svc, void *priv)
{
	struct aoc_bt *bt = priv;

	schedule_work(&bt->rx_work);
}

static void aoc_bt_rx_work(struct work_struct *work)
{
	struct aoc_bt *bt = container_of(work, struct aoc_bt, rx_work);
	u8 *buf;

	buf = kmalloc(AOC_BT_RX_CHUNK, GFP_KERNEL);
	if (!buf)
		return;

	while (bt->ctl && aoc_service_can_read(bt->ctl)) {
		int len = aoc_service_read(bt->ctl, buf, AOC_BT_CTL_RX_LEN);

		if (len <= 0)
			break;
		aoc_bt_ctl_msg(bt, buf, len);
	}

	while (bt->rx && bt->fw_hdr_len && aoc_service_can_read(bt->rx)) {
		size_t room = AOC_BT_RX_BUF - bt->rx_len;
		int len;

		if (!room)
			break;
		len = aoc_service_read(bt->rx, bt->rx_buf + bt->rx_len,
				       min_t(size_t, room, AOC_BT_RX_CHUNK));
		if (len <= 0)
			break;
		bt->rx_len += len;
		aoc_bt_rx_frames(bt);
	}
	kfree(buf);
}

/* One data-ring message: ApTxHdrV1, then pre and data back to back. */
static int aoc_bt_tx(struct aoc_bt *bt, u8 type, const u8 *pre, size_t pre_len,
		     const u8 *data, size_t len)
{
	unsigned int tries = 0;
	u32 slot, avail;
	u8 *msg;
	int ret;

	if (!bt->tx)
		return -ENODEV;
	if (bt->state != AOC_BT_ST_READY)
		return -EHOSTDOWN;

	slot = aoc_bt_slot(pre_len + len, AOC_BT_AP_HDR_LEN);
	if (slot > bt->ring_size)
		return -EMSGSIZE;

	/* Wait for the AOC to ack enough of the ring.  Equal pointers mean
	 * empty, not full -- tx_full tells those apart, as it does in the HAL.
	 */
	for (;;) {
		avail = bt->tx_full ? 0 :
			bt->ring_size - (bt->tx_write + bt->ring_size -
					 bt->tx_acked) % bt->ring_size;
		if (avail >= slot)
			break;
		if (++tries > AOC_BT_TX_RETRIES)
			return -ENOBUFS;
		usleep_range(200, 400);
	}

	msg = kzalloc(slot, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	put_unaligned_le64(ktime_get_boottime_ns(), msg);
	put_unaligned_le32(pre_len + len, msg + 8);
	put_unaligned_le32(bt->seq++, msg + 12);
	msg[16] = type;
	memcpy(msg + AOC_BT_AP_HDR_LEN, pre, pre_len);
	memcpy(msg + AOC_BT_AP_HDR_LEN + pre_len, data, len);

	ret = aoc_bt_write_all(bt, bt->tx, msg, slot);
	kfree(msg);
	if (ret)
		return ret;
	bt->tx_write = (bt->tx_write + slot) % bt->ring_size;
	if (bt->tx_write == bt->tx_acked)
		bt->tx_full = true;
	return 0;
}

/* A request to the AOC's BT HAL server; returns its pw::Status or -errno. */
static int aoc_bt_transact(struct aoc_bt *bt, const u8 *req, size_t len)
{
	int ret;

	reinit_completion(&bt->xact_done);
	bt->xact_id = bt->seq;		/* the reply echoes our sequence */
	ret = aoc_bt_tx(bt, AOC_BT_MSG_TRANSACT, NULL, 0, req, len);
	if (ret)
		return ret;
	if (!wait_for_completion_timeout(&bt->xact_done,
			msecs_to_jiffies(AOC_BT_XACT_TIMEOUT_MS)))
		return -ETIMEDOUT;
	return bt->xact_status;
}

static int aoc_bt_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);
	u8 pre[6] = { AOC_BT_ID_HCI, 0, 0, 0, 0, hci_skb_pkt_type(skb) };
	int ret;

	ret = aoc_bt_tx(bt, AOC_BT_MSG_DATA, pre, sizeof(pre), skb->data,
			skb->len);
	if (ret)
		return ret;

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}

	kfree_skb(skb);
	return 0;
}

static int aoc_bt_close(struct hci_dev *hdev);

/*
 * BT_REG_ON reaches the chip through a flip-flop: a level change only takes
 * effect once the latch line is pulsed.  This is the vendor nitrous driver's
 * sequence; without the pulses the core stays off and HCI goes unanswered.
 */
static void aoc_bt_latch(struct aoc_bt *bt)
{
	gpiod_set_value_cansleep(bt->reg_on_latch, 1);
	udelay(1);
	gpiod_set_value_cansleep(bt->reg_on_latch, 0);
}

static void aoc_bt_power(struct aoc_bt *bt, bool on)
{
	if (on) {
		gpiod_set_value_cansleep(bt->reg_on, 0);
		aoc_bt_latch(bt);
		msleep(30);
		gpiod_set_value_cansleep(bt->reg_on, 1);
		aoc_bt_latch(bt);
		gpiod_set_value_cansleep(bt->dev_wake, 1);
		/* The vendor HAL's reg_on_delay_ms; 10-20 ms leaves HCI mute. */
		msleep(100);
	} else {
		gpiod_set_value_cansleep(bt->dev_wake, 0);
		gpiod_set_value_cansleep(bt->reg_on, 0);
		aoc_bt_latch(bt);
	}
	usleep_range(10000, 20000);
}

static int aoc_bt_open(struct hci_dev *hdev)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);
	int tries, ret;

	aoc_bt_power(bt, true);

	bt->ctl = aoc_service_find(bt->aoc_dev, AOC_BT_CTL_SERVICE);
	bt->tx = aoc_service_find(bt->aoc_dev, AOC_BT_TX_SERVICE);
	bt->rx = aoc_service_find(bt->aoc_dev, AOC_BT_RX_SERVICE);
	if (!bt->ctl || !bt->tx || !bt->rx) {
		dev_err(bt->dev, "AOC has no chre_bt_offload services (is the AOC up?)\n");
		bt->ctl = bt->tx = bt->rx = NULL;
		aoc_bt_power(bt, false);
		return -ENODEV;
	}

	bt->rx_len = 0;
	bt->fw_hdr_len = 0;
	bt->state = AOC_BT_ST_CREATED;
	reinit_completion(&bt->synced);
	aoc_service_set_handler(bt->rx, aoc_bt_service_irq, bt);
	aoc_service_set_handler(bt->ctl, aoc_bt_service_irq, bt);

	/*
	 * Handshake: rx_work drains whatever the control queue already holds,
	 * then we send the pause.  The AOC answers pause and then sync, and the
	 * sync carries the ring geometry -- until it lands, transmit is refused
	 * and the data ring is not read.
	 */
	schedule_work(&bt->rx_work);
	flush_work(&bt->rx_work);

	/* Re-send the pause only while unanswered: each one restarts the AOC's
	 * side of the handshake. */
	for (tries = 0; tries < AOC_BT_SYNC_TRIES; tries++) {
		if (bt->state == AOC_BT_ST_CREATED &&
		    aoc_bt_ctl_send(bt, AOC_BT_CTL_PAUSE, 0))
			bt_dev_warn(bt->hdev, "cannot send pause");
		if (wait_for_completion_timeout(&bt->synced,
				msecs_to_jiffies(AOC_BT_SYNC_TIMEOUT_MS)))
			break;
	}
	if (bt->state != AOC_BT_ST_READY) {
		bt_dev_err(bt->hdev,
			   "no EFW sync from the AOC after %d pauses (state %d)",
			   AOC_BT_SYNC_TRIES, bt->state);
		aoc_bt_close(hdev);
		return -ETIMEDOUT;
	}

	/* The AOC owns BT power on this board; ask it to bring the core up. */
	ret = aoc_bt_transact(bt, aoc_bt_power_on, sizeof(aoc_bt_power_on));
	if (ret) {
		bt_dev_err(bt->hdev, "AOC PowerControl(on) failed: %d", ret);
		aoc_bt_close(hdev);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int aoc_bt_close(struct hci_dev *hdev)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);

	/* Before the handlers go: the reply comes back through rx_work. */
	if (bt->state == AOC_BT_ST_READY)
		aoc_bt_transact(bt, aoc_bt_power_off, sizeof(aoc_bt_power_off));

	if (bt->rx)
		aoc_service_set_handler(bt->rx, NULL, NULL);
	if (bt->ctl)
		aoc_service_set_handler(bt->ctl, NULL, NULL);
	cancel_work_sync(&bt->rx_work);
	bt->rx_len = 0;
	bt->fw_hdr_len = 0;
	bt->state = AOC_BT_ST_CREATED;
	bt->ctl = bt->tx = bt->rx = NULL;
	aoc_bt_power(bt, false);
	return 0;
}

/*
 * Controller bring-up is the ordinary Broadcom sequence (chip name, matching
 * .hcd patchram, re-read local version).  Nothing here is AOC-specific -- the
 * transport is just a pipe -- which is why btbcm is reused rather than vendor
 * commands being open-coded.
 */
static int aoc_bt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *aoc_pdev;
	struct device_node *np;
	struct aoc_bt *bt;
	struct hci_dev *hdev;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	bt->dev = dev;

	np = of_parse_phandle(dev->of_node, "aoc", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no aoc phandle\n");
	aoc_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!aoc_pdev)
		return dev_err_probe(dev, -EPROBE_DEFER, "AOC not ready\n");
	bt->aoc_dev = &aoc_pdev->dev;

	bt->reg_on = devm_gpiod_get(dev, "shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(bt->reg_on)) {
		ret = dev_err_probe(dev, PTR_ERR(bt->reg_on), "no BT_REG_ON\n");
		goto err_put;
	}
	bt->reg_on_latch = devm_gpiod_get_optional(dev, "reg-on-latch",
						   GPIOD_OUT_LOW);
	if (IS_ERR(bt->reg_on_latch)) {
		ret = dev_err_probe(dev, PTR_ERR(bt->reg_on_latch),
				    "bad reg-on-latch GPIO\n");
		goto err_put;
	}
	bt->dev_wake = devm_gpiod_get_optional(dev, "device-wakeup",
					       GPIOD_OUT_LOW);
	if (IS_ERR(bt->dev_wake)) {
		ret = dev_err_probe(dev, PTR_ERR(bt->dev_wake),
				    "bad device-wakeup GPIO\n");
		goto err_put;
	}

	INIT_WORK(&bt->rx_work, aoc_bt_rx_work);
	init_completion(&bt->synced);
	init_completion(&bt->xact_done);
	mutex_init(&bt->ctl_lock);

	bt->rx_buf = devm_kmalloc(dev, AOC_BT_RX_BUF, GFP_KERNEL);
	if (!bt->rx_buf) {
		ret = -ENOMEM;
		goto err_put;
	}

	hdev = hci_alloc_dev();
	if (!hdev) {
		ret = -ENOMEM;
		goto err_put;
	}
	bt->hdev = hdev;

	/*
	 * There is no "AOC" bus type, and the choice is not cosmetic: btbcm
	 * selects its subver -> chip-name table on hdev->bus, with HCI_USB the
	 * odd one out and everything else using the serial table.  What is on
	 * the far side of the AOC here is a UART-attached BCM4390 speaking H4,
	 * so HCI_UART picks the right table and .hcd naming.
	 */
	hdev->bus = HCI_UART;
	hci_set_drvdata(hdev, bt);
	SET_HCIDEV_DEV(hdev, dev);

	hdev->open = aoc_bt_open;
	hdev->close = aoc_bt_close;
	hdev->send = aoc_bt_send_frame;
	hdev->setup = btbcm_setup_patchram;
	/* Every close cuts power, and with it the patchram. */
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	hdev->set_bdaddr = btbcm_set_bdaddr;

	platform_set_drvdata(pdev, bt);

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "cannot register hci device\n");
		goto err_free;
	}

	return 0;

err_free:
	hci_free_dev(hdev);
err_put:
	put_device(bt->aoc_dev);
	return ret;
}

static void aoc_bt_remove(struct platform_device *pdev)
{
	struct aoc_bt *bt = platform_get_drvdata(pdev);

	hci_unregister_dev(bt->hdev);	/* runs close(): drops the services */
	cancel_work_sync(&bt->rx_work);
	hci_free_dev(bt->hdev);
	put_device(bt->aoc_dev);
}

static const struct of_device_id aoc_bt_of_match[] = {
	{ .compatible = "google,aoc-bluetooth" },
	{}
};
MODULE_DEVICE_TABLE(of, aoc_bt_of_match);

static struct platform_driver aoc_bt_driver = {
	.probe = aoc_bt_probe,
	.remove = aoc_bt_remove,
	.driver = {
		.name = "hci_aoc",
		.of_match_table = aoc_bt_of_match,
	},
};
module_platform_driver(aoc_bt_driver);

MODULE_DESCRIPTION("Bluetooth HCI transport over the Google AOC coprocessor");
MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_LICENSE("GPL");
