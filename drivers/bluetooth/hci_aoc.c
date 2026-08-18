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
 * AOC ring services carrying H4, with controller setup delegated to the shared
 * Broadcom helpers exactly as btusb does.
 *
 * Note the service names say "chre_bt_offload": on this firmware that is the
 * only Bluetooth pipe (the string "com.google.bt" does not appear in the image
 * at all -- that name belongs to other Tensor products).  Whether the AOC's
 * proxy will serve a general HCI host or only context-hub clients is the open
 * question this driver exists to answer, so the service names, the ring
 * direction and the control handshake are all overridable at runtime.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hex.h>
#include <linux/module.h>
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

/* Live service table (see the AOC's debugfs "services"): idx 22 queue,
 * idx 89 ring, idx 90 ring on komodo's AOC build 1050. */
static char *ctl_service = "chre_bt_offload_ctl";
module_param(ctl_service, charp, 0444);
MODULE_PARM_DESC(ctl_service, "AOC control service for the BT offload proxy");

static char *tx_service = "chre_bt_offload_data_tx";
module_param(tx_service, charp, 0444);
MODULE_PARM_DESC(tx_service, "AOC service carrying host->controller H4");

static char *rx_service = "chre_bt_offload_data_rx";
module_param(rx_service, charp, 0444);
MODULE_PARM_DESC(rx_service, "AOC service carrying controller->host H4");

/*
 * The tx/rx names are the AOC's, and it is not established whose point of view
 * they take.  Each AOC service has both an AP->AOC and an AOC->AP region, so if
 * the sense is inverted this simply swaps which service is written and which is
 * read, with no other change.
 */
static bool swap_rings;
module_param(swap_rings, bool, 0644);
MODULE_PARM_DESC(swap_rings, "write to rx_service and read from tx_service");

/*
 * Some proxies need to be told to start.  The control service's message format
 * is not known, so rather than invent one, allow a byte sequence to be handed
 * in at runtime; anything the control service sends back is logged, which is
 * how that format gets learned.  Empty (the default) sends nothing.
 */
static char *ctl_enable;
module_param(ctl_enable, charp, 0644);
MODULE_PARM_DESC(ctl_enable, "hex bytes to send on the control service at open");

static bool setup_patchram = true;
module_param(setup_patchram, bool, 0644);
MODULE_PARM_DESC(setup_patchram,
		 "run the Broadcom patchram download (clear if the AOC already did it)");

#define AOC_BT_RX_CHUNK		512
#define AOC_BT_TX_RETRIES	100

struct aoc_bt {
	struct device *dev;
	struct device *aoc_dev;
	struct hci_dev *hdev;
	struct aoc_service *ctl;
	struct aoc_service *tx;
	struct aoc_service *rx;
	struct gpio_desc *reg_on;	/* BT_REG_ON: powers the BT core */
	struct gpio_desc *dev_wake;	/* host -> device wake */
	struct work_struct rx_work;

	/* H4 reassembly: the services are byte rings, not message queues, so a
	 * read returns whatever happens to be there -- packets split across
	 * reads and several packets per read are both normal. */
	u8 rx_type;			/* current H4 packet type, 0 = idle */
	u8 rx_hdr[4];
	unsigned int rx_hdr_len;
	unsigned int rx_have;
	unsigned int rx_remaining;
	struct sk_buff *rx_skb;
};

/* H4 header length per packet type, 0 for a type we do not accept. */
static unsigned int aoc_bt_hdr_len(u8 type)
{
	switch (type) {
	case HCI_EVENT_PKT:
		return HCI_EVENT_HDR_SIZE;
	case HCI_ACLDATA_PKT:
		return HCI_ACL_HDR_SIZE;
	case HCI_SCODATA_PKT:
		return HCI_SCO_HDR_SIZE;
	case HCI_ISODATA_PKT:
		return HCI_ISO_HDR_SIZE;
	default:
		return 0;
	}
}

/* Payload length carried by a complete H4 header. */
static unsigned int aoc_bt_payload_len(u8 type, const u8 *hdr)
{
	switch (type) {
	case HCI_EVENT_PKT:
		return ((const struct hci_event_hdr *)hdr)->plen;
	case HCI_ACLDATA_PKT:
		return __le16_to_cpu(((const struct hci_acl_hdr *)hdr)->dlen);
	case HCI_SCODATA_PKT:
		return ((const struct hci_sco_hdr *)hdr)->dlen;
	case HCI_ISODATA_PKT:
		return __le16_to_cpu(((const struct hci_iso_hdr *)hdr)->dlen);
	default:
		return 0;
	}
}

static void aoc_bt_rx_reset(struct aoc_bt *bt)
{
	kfree_skb(bt->rx_skb);
	bt->rx_skb = NULL;
	bt->rx_type = 0;
	bt->rx_hdr_len = 0;
	bt->rx_have = 0;
	bt->rx_remaining = 0;
}

static void aoc_bt_rx_deliver(struct aoc_bt *bt)
{
	struct sk_buff *skb = bt->rx_skb;

	bt->rx_skb = NULL;
	bt->rx_type = 0;
	bt->rx_have = 0;
	bt->rx_hdr_len = 0;
	hci_recv_frame(bt->hdev, skb);		/* consumes skb */
}

/* Feed a chunk of the ring into the H4 reassembler. */
static void aoc_bt_rx_stream(struct aoc_bt *bt, const u8 *data, size_t len)
{
	while (len) {
		unsigned int n;

		if (!bt->rx_type) {
			u8 type = *data++;

			len--;
			bt->rx_hdr_len = aoc_bt_hdr_len(type);
			if (!bt->rx_hdr_len) {
				/* Not H4, or we lost sync.  Say so once and
				 * drop the byte rather than build garbage. */
				bt_dev_warn_ratelimited(bt->hdev,
					"unexpected H4 type %#x, resyncing",
					type);
				continue;
			}
			bt->rx_type = type;
			bt->rx_have = 0;
			continue;
		}

		if (bt->rx_have < bt->rx_hdr_len) {
			n = min_t(size_t, len, bt->rx_hdr_len - bt->rx_have);
			memcpy(bt->rx_hdr + bt->rx_have, data, n);
			bt->rx_have += n;
			data += n;
			len -= n;
			if (bt->rx_have < bt->rx_hdr_len)
				return;		/* header still incomplete */

			bt->rx_remaining = aoc_bt_payload_len(bt->rx_type,
							      bt->rx_hdr);
			bt->rx_skb = bt_skb_alloc(bt->rx_hdr_len +
						  bt->rx_remaining, GFP_KERNEL);
			if (!bt->rx_skb) {
				aoc_bt_rx_reset(bt);
				return;
			}
			hci_skb_pkt_type(bt->rx_skb) = bt->rx_type;
			skb_put_data(bt->rx_skb, bt->rx_hdr, bt->rx_hdr_len);
			if (!bt->rx_remaining)
				aoc_bt_rx_deliver(bt);
			continue;
		}

		n = min_t(size_t, len, bt->rx_remaining);
		skb_put_data(bt->rx_skb, data, n);
		bt->rx_remaining -= n;
		data += n;
		len -= n;
		if (!bt->rx_remaining)
			aoc_bt_rx_deliver(bt);
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

	/* Anything the control service says is unmapped protocol; log it, as
	 * that is how the handshake gets identified. */
	while (bt->ctl && aoc_service_can_read(bt->ctl)) {
		int len = aoc_service_read(bt->ctl, buf, AOC_BT_RX_CHUNK);

		if (len <= 0)
			break;
		bt_dev_info(bt->hdev, "ctl message (%d bytes): %*ph",
			    len, min(len, 32), buf);
	}

	while (bt->rx && aoc_service_can_read(bt->rx)) {
		int len = aoc_service_read(bt->rx, buf, AOC_BT_RX_CHUNK);

		if (len <= 0)
			break;
		aoc_bt_rx_stream(bt, buf, len);
	}
	kfree(buf);
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

static int aoc_bt_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);
	int ret;

	if (!bt->tx)
		return -ENODEV;

	/* H4: one type byte ahead of the packet. */
	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);

	ret = aoc_bt_write_all(bt, bt->tx, skb->data, skb->len);
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

/* Optional, runtime-supplied kick for the control service (see ctl_enable). */
static void aoc_bt_send_ctl_enable(struct aoc_bt *bt)
{
	u8 buf[32];
	size_t n;

	if (!bt->ctl || !ctl_enable || !*ctl_enable)
		return;

	n = min(strlen(ctl_enable) / 2, sizeof(buf));
	if (!n)
		return;
	if (hex2bin(buf, ctl_enable, n)) {
		dev_warn(bt->dev, "ctl_enable is not valid hex\n");
		return;
	}
	dev_info(bt->dev, "sending %zu ctl byte(s): %*ph\n", n, (int)n, buf);
	aoc_service_write(bt->ctl, buf, n);
}

static int aoc_bt_open(struct hci_dev *hdev)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);
	const char *want_tx = swap_rings ? rx_service : tx_service;
	const char *want_rx = swap_rings ? tx_service : rx_service;

	gpiod_set_value_cansleep(bt->dev_wake, 1);
	gpiod_set_value_cansleep(bt->reg_on, 1);
	usleep_range(10000, 12000);	/* BT_REG_ON -> core out of reset */

	bt->ctl = aoc_service_find(bt->aoc_dev, ctl_service);
	bt->tx = aoc_service_find(bt->aoc_dev, want_tx);
	bt->rx = aoc_service_find(bt->aoc_dev, want_rx);
	if (!bt->tx || !bt->rx) {
		dev_err(bt->dev, "AOC has no '%s'/'%s' (is the AOC up?)\n",
			want_tx, want_rx);
		gpiod_set_value_cansleep(bt->reg_on, 0);
		gpiod_set_value_cansleep(bt->dev_wake, 0);
		return -ENODEV;
	}
	if (!bt->ctl)
		dev_warn(bt->dev, "no '%s'; continuing without it\n",
			 ctl_service);

	aoc_bt_rx_reset(bt);
	aoc_service_set_handler(bt->rx, aoc_bt_service_irq, bt);
	if (bt->ctl)
		aoc_service_set_handler(bt->ctl, aoc_bt_service_irq, bt);

	aoc_bt_send_ctl_enable(bt);

	/* Drain anything queued while we were attaching. */
	schedule_work(&bt->rx_work);
	return 0;
}

static int aoc_bt_close(struct hci_dev *hdev)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);

	if (bt->rx)
		aoc_service_set_handler(bt->rx, NULL, NULL);
	if (bt->ctl)
		aoc_service_set_handler(bt->ctl, NULL, NULL);
	cancel_work_sync(&bt->rx_work);
	aoc_bt_rx_reset(bt);
	bt->ctl = bt->tx = bt->rx = NULL;

	gpiod_set_value_cansleep(bt->reg_on, 0);
	gpiod_set_value_cansleep(bt->dev_wake, 0);
	return 0;
}

static int aoc_bt_flush(struct hci_dev *hdev)
{
	struct aoc_bt *bt = hci_get_drvdata(hdev);

	cancel_work_sync(&bt->rx_work);
	aoc_bt_rx_reset(bt);
	return 0;
}

/*
 * Controller bring-up is the ordinary Broadcom sequence (chip name, matching
 * .hcd patchram, re-read local version).  Nothing here is AOC-specific -- the
 * transport is just a pipe -- which is why btbcm is reused rather than vendor
 * commands being open-coded.  If the AOC's proxy has already patched the
 * controller, clear setup_patchram and let the core do a plain HCI reset.
 */
static int aoc_bt_setup(struct hci_dev *hdev)
{
	if (!setup_patchram)
		return 0;
	return btbcm_setup_patchram(hdev);
}

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

	/* BT_REG_ON (gpn8-0 on komodo); the AOC owns the HCI UART itself. */
	bt->reg_on = devm_gpiod_get(dev, "shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(bt->reg_on)) {
		ret = dev_err_probe(dev, PTR_ERR(bt->reg_on),
				    "no shutdown (BT_REG_ON) GPIO\n");
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
	hdev->flush = aoc_bt_flush;
	hdev->send = aoc_bt_send_frame;
	hdev->setup = aoc_bt_setup;
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
