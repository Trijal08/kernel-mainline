// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm (TCM) protocol version 1 SPI touchscreen driver.
 *
 * The TouchComm message layer implemented here follows the Synaptics
 * reference implementation shipped in Google Pixel downstream kernels
 * (syna_tcm2 / synaptics_touchcom_core_v1.c):
 *
 *  - every device message is led by a 4-byte header [0xa5 marker, code,
 *    length_le16]; the header read announces the payload length and the
 *    payload/trailer are then fetched with [0xa5, STATUS_CONTINUED_READ]
 *    continuation transactions,
 *  - a message tail carries one 0x5a end-of-message pad byte and, when
 *    the firmware has the features enabled, a CRC-16 (CCITT-FALSE,
 *    little-endian) plus an RC byte and one further pad.  Presence of
 *    the optional trailer is sniffed once at startup: the device pads
 *    reads past the end of message with 0x5a, so trailer bytes that
 *    read back as padding mean the feature is absent,
 *  - host commands are bare [code, length_le16, payload] writes, with
 *    the CRC-16 over the whole packet appended when enabled,
 *  - there are no sequence bits and no fetch/ACK commands: the device
 *    streams its pending message on every read, reports (codes >= 0x10)
 *    and command responses (status codes < 0x10) over the same
 *    transport, with ATTN asserted while a message is pending.
 *
 * Touch reports are parsed with the device-provided touch report config
 * descriptor: a list of (entity code, bit length) pairs with FOREACH
 * loop markers, fields packed LSB-first in the report bitstream.
 *
 * The controller runs its firmware from internal flash; no firmware
 * download is required for operation.
 */

#include <linux/bitops.h>
#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/string_choices.h>
#include <linux/unaligned.h>

#include <drm/drm_panel.h>

#include "touch_bus_negotiator.h"

#define SYNA_TCM_HEADER_SIZE		4
#define SYNA_TCM_CRC_SIZE		2
#define SYNA_TCM_HEADER_BITS		(SYNA_TCM_HEADER_SIZE * 8)

/* Largest message this driver drains (touch reports/configs are tiny). */
#define SYNA_TCM_MSG_MAX		1024
#define SYNA_TCM_OUT_MAX		32
#define SYNA_TCM_CONFIG_MAX		256

/* Worst-case end-of-message trailer: pad + CRC-16 + RC + final pad. */
#define SYNA_TCM_TRAILER_MAX		5
/* Largest payload a message read accepts. */
#define SYNA_TCM_PAYLOAD_MAX		512
/*
 * Keep SPI transfers below the s3c64xx polling FIFO depth.  Continuation
 * packets spend two bytes on marker/status, leaving the rest for data.
 */
#define SYNA_TCM_XFER_MAX		63
#define SYNA_TCM_CONT_HEADER_SIZE	2
#define SYNA_TCM_CONT_DATA_MAX		(SYNA_TCM_XFER_MAX - \
					 SYNA_TCM_CONT_HEADER_SIZE)

#define SYNA_TCM_RETRIES		5

/* Power sequencing, from the downstream tegu DT. */
#define SYNA_TCM_POWER_DELAY_MS		200
#define SYNA_TCM_RESET_ACTIVE_MS	2
#define SYNA_TCM_RESET_DELAY_MS		50
/* Bus turnaround between transactions of one message exchange. */
#define SYNA_TCM_TAT_DELAY_US		50
/* Pace and budget for polling a command response out of the device. */
#define SYNA_TCM_RESP_POLL_MS		2
#define SYNA_TCM_RESP_TIMEOUT_MS	3000

/* How long an AoC tap-to-wake holds the system awake for userspace to react. */
#define SYNA_TCM_WAKE_HOLD_MS		3000

/* TouchComm commands */
#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_RESET			0x04
#define TCM_CMD_GET_APPLICATION_INFO	0x20
#define TCM_CMD_GET_TOUCH_REPORT_CONFIG	0x25
#define TCM_CMD_SET_DYNAMIC_CONFIG	0x24
#define TCM_CMD_ENTER_DEEP_SLEEP	0x2c
#define TCM_CMD_EXIT_DEEP_SLEEP		0x2d

/*
 * Dynamic-config field ids (payload of TCM_CMD_SET_DYNAMIC_CONFIG, a
 * [field, value_le16] triple).  Field 0x09 arms the controller's low-power
 * wake-on-gesture ("LPWG") scanning: instead of deep sleep the firmware keeps
 * a low-power scan running so a tap can be detected.  On tegu the bus is then
 * handed to AoC, which does the gesture recognition itself.  Values from the
 * Synaptics reference (synaptics_touchcom_core_dev.h: DC_ENABLE_WAKEUP_GESTURE_
 * MODE = 0x09; syna_tcm_set_dynamic_config()).
 */
#define TCM_DC_ENABLE_WAKEUP_GESTURE_MODE	0x09

/* Status codes (< 0x10) and report codes (>= 0x10) */
#define TCM_STATUS_IDLE			0x00
#define TCM_STATUS_OK			0x01
#define TCM_STATUS_CONTINUED_READ	0x03
#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH		0x11

/* IDENTIFY payload */
#define TCM_MODE_APPLICATION_FIRMWARE	0x01
#define TCM_MODE_ROMBOOTLOADER		0x04
#define TCM_MODE_BOOTLOADER		0x0b

#define TCM_V1_MARKER			0xa5
#define TCM_V1_PADDING			0x5a

/* Touch report config entity codes */
#define TCM_TOUCH_END			0x00
#define TCM_TOUCH_FOREACH_ACTIVE_OBJECT	0x01
#define TCM_TOUCH_FOREACH_OBJECT	0x02
#define TCM_TOUCH_FOREACH_END		0x03
#define TCM_TOUCH_PAD_TO_NEXT_BYTE	0x04
#define TCM_TOUCH_OBJECT_N_INDEX	0x06
#define TCM_TOUCH_OBJECT_N_CLASSIFICATION 0x07
#define TCM_TOUCH_OBJECT_N_X_POSITION	0x08
#define TCM_TOUCH_OBJECT_N_Y_POSITION	0x09
#define TCM_TOUCH_OBJECT_N_Z		0x0a
#define TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS	0x18

/* Object classifications */
#define TCM_OBJ_LIFT			0
#define TCM_OBJ_FINGER			1
#define TCM_OBJ_GLOVED_OBJECT		2

#define SYNA_TCM_MAX_SLOTS		10

struct syna_tcm_object {
	u8 status;
	u16 x;
	u16 y;
	u8 z;
};

struct syna_tcm {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];

	/* Serializes all bus message exchanges. */
	struct mutex io_lock;

	/* Optional message trailer features, sniffed during detect. */
	bool has_crc;
	bool has_extra_rc;
	u16 max_wr_size;

	u16 max_x;
	u16 max_y;
	u16 max_objects;

	/* Last received message: header + reassembled payload. */
	u8 *msg;
	u32 payload_len;
	u8 code;

	/* Raw bus transfer buffers (DMA-safe). */
	u8 *rx;
	u8 *tx;

	u8 config[SYNA_TCM_CONFIG_MAX];
	u32 config_len;

	struct syna_tcm_object objects[SYNA_TCM_MAX_SLOTS];
	unsigned long slots_seen;

	/* Touch Bus Negotiator (AoC bus handoff for wake gestures). */
	bool tbn_enabled;	/* opted in via goog,tbn-enabled and registered */
	u32 tbn_mask;		/* device bit from register_tbn() */
	bool tbn_suspended;	/* bus was released to AoC across this suspend */

	/*
	 * Low-power entry is driven by the panel when we can follow one, and by
	 * system sleep otherwise -- and by both, in either order, when the
	 * system suspends with the display still on. @pm_lock serialises them;
	 * the two "off because" flags decide when it is safe to come back.
	 */
	struct drm_panel_follower follower;
	bool following;
	struct mutex pm_lock;
	bool powered_off;
	bool off_by_panel;
	bool off_by_pm;
};

/*
 * CRC-6 over the leading 'bits' of the buffer, MSB first, as used for the
 * TouchComm v2 header.  Transcribed from the Synaptics reference
 * implementation (synaptics_touchcom_core_dev.h); a fully received v2
 * header including its CRC field computes to 0.  Only used to recognize
 * (and reject) TouchComm v2 parts during protocol detection.
 */
static u8 syna_tcm_crc6(const u8 *p, unsigned int bits)
{
	static const u16 crc6_table[16] = {
		0,  268,  536,  788, 1072, 1340, 1576, 1828,
		2144, 2412, 2680, 2932, 3152, 3420, 3656, 3908
	};
	u16 r = 0x003f << 2;
	u16 x;

	for (; bits > 8; bits -= 8) {
		r ^= *p++;
		r = (r << 4) ^ crc6_table[r >> 4];
		r = (r << 4) ^ crc6_table[r >> 4];
	}

	if (bits > 0) {
		x = *p;
		while (bits--) {
			if (x & 0x80)
				r ^= 0x80;

			x <<= 1;
			r <<= 1;
			if (r & 0x100)
				r ^= (0x03 << 2);
		}
	}

	return (r >> 2) & 0x3f;
}

static int syna_tcm_spi_read(struct syna_tcm *ts, unsigned int len)
{
	struct spi_transfer xfer = {
		.rx_buf	= ts->rx,
		.len	= len,
	};

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

static int syna_tcm_spi_write(struct syna_tcm *ts, unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf	= ts->tx,
		.len	= len,
	};

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

/*
 * Send one command packet: [code, length_le16, payload], plus the CRC-16
 * (little-endian, over the whole packet) when the firmware has the
 * feature enabled.
 */
static int syna_tcm_send(struct syna_tcm *ts, u8 command, const u8 *payload,
			 u16 len)
{
	unsigned int total = 3 + len;
	u16 crc;

	if (total + SYNA_TCM_CRC_SIZE > SYNA_TCM_OUT_MAX)
		return -EINVAL;
	if (ts->max_wr_size && total + SYNA_TCM_CRC_SIZE > ts->max_wr_size)
		return -EINVAL;

	ts->tx[0] = command;
	put_unaligned_le16(len, &ts->tx[1]);
	if (len)
		memcpy(&ts->tx[3], payload, len);

	if (ts->has_crc) {
		crc = crc_itu_t(0xffff, ts->tx, total);
		ts->tx[total] = crc & 0xff;
		ts->tx[total + 1] = crc >> 8;
		total += SYNA_TCM_CRC_SIZE;
	}

	return syna_tcm_spi_write(ts, total);
}

static int syna_tcm_send_startup_identify(struct syna_tcm *ts)
{
	ts->tx[0] = TCM_CMD_IDENTIFY;

	return syna_tcm_spi_write(ts, 1);
}

/*
 * Read one bus transaction into ts->rx and verify the leading v1 marker,
 * retrying on garbage like the reference syna_tcm_v1_read().
 */
static int syna_tcm_read_packet(struct syna_tcm *ts, unsigned int len)
{
	int retry;
	int error;

	for (retry = 0; retry < SYNA_TCM_RETRIES; retry++) {
		if (retry)
			usleep_range(5000, 10000);

		error = syna_tcm_spi_read(ts, len);
		if (error)
			return error;

		if (ts->rx[0] == TCM_V1_MARKER)
			return 0;
	}

	dev_warn_ratelimited(&ts->spi->dev, "no marker, read leads %*ph\n",
			     (int)umin(len, 4), ts->rx);

	return -EBADMSG;
}

/*
 * End-of-message trailer length: one pad byte, plus CRC-16/RC and one
 * further pad when the firmware has the features enabled.
 */
static unsigned int syna_tcm_trailer_len(struct syna_tcm *ts)
{
	unsigned int trailer = 1;

	if (ts->has_crc)
		trailer += SYNA_TCM_CRC_SIZE;
	if (ts->has_extra_rc)
		trailer += 1;
	if (ts->has_crc || ts->has_extra_rc)
		trailer += 1;

	return trailer;
}

/*
 * Drain the payload and trailer of the current message.  After a v1 header,
 * the remaining bytes are supplied by continuation transactions led by
 * [0xa5, STATUS_CONTINUED_READ].
 */
static int syna_tcm_read_continued(struct syna_tcm *ts)
{
	unsigned int total;
	unsigned int remaining;
	unsigned int offset;
	unsigned int chunk;
	int error;

	if (!ts->payload_len)
		return 0;

	total = ts->payload_len + syna_tcm_trailer_len(ts);
	if (SYNA_TCM_HEADER_SIZE + total > SYNA_TCM_MSG_MAX)
		return -EMSGSIZE;

	remaining = total;
	offset = SYNA_TCM_HEADER_SIZE;

	while (remaining) {
		chunk = min_t(unsigned int, remaining, SYNA_TCM_CONT_DATA_MAX);

		if (chunk == 1) {
			ts->msg[offset++] = TCM_V1_PADDING;
			remaining--;
			continue;
		}

		usleep_range(SYNA_TCM_TAT_DELAY_US,
			     2 * SYNA_TCM_TAT_DELAY_US);

		error = syna_tcm_read_packet(ts,
					     chunk + SYNA_TCM_CONT_HEADER_SIZE);
		if (error)
			return error;

		if (ts->rx[1] != TCM_STATUS_CONTINUED_READ) {
			dev_warn_ratelimited(&ts->spi->dev,
					     "bad continuation %*ph\n",
					     (int)umin(chunk + SYNA_TCM_CONT_HEADER_SIZE,
						       4),
					     ts->rx);
			return -EBADMSG;
		}

		memcpy(&ts->msg[offset], &ts->rx[SYNA_TCM_CONT_HEADER_SIZE],
		       chunk);
		offset += chunk;
		remaining -= chunk;
	}

	return 0;
}

/*
 * Read the device's pending message into ts->msg.  A header-only read
 * announces code and payload length; a message with payload is then fetched
 * through continued-read chunks.  Protocol-level corruption is reported as
 * -EBADMSG so callers can retry.
 */
static int syna_tcm_get_response(struct syna_tcm *ts)
{
	unsigned int total;
	int error;

	error = syna_tcm_read_packet(ts, SYNA_TCM_HEADER_SIZE);
	if (error)
		return error;

	ts->code = ts->rx[1];
	total = get_unaligned_le16(&ts->rx[2]);
	ts->payload_len = total;

	if (ts->code == TCM_STATUS_CONTINUED_READ)
		return -EBADMSG;

	memcpy(ts->msg, ts->rx, SYNA_TCM_HEADER_SIZE);

	if (!total)
		return 0;

	if (total > SYNA_TCM_PAYLOAD_MAX) {
		dev_warn_ratelimited(&ts->spi->dev,
				     "implausible message %*ph\n",
				     SYNA_TCM_HEADER_SIZE, ts->msg);
		return -EBADMSG;
	}

	if (SYNA_TCM_HEADER_SIZE + total + syna_tcm_trailer_len(ts) >
	    SYNA_TCM_MSG_MAX) {
		dev_warn_ratelimited(&ts->spi->dev,
				     "oversize message %*ph\n",
				     SYNA_TCM_HEADER_SIZE, ts->msg);
		return -EBADMSG;
	}

	return syna_tcm_read_continued(ts);
}

/*
 * Send a command and poll for the message answering it.  Asynchronous
 * reports share the same stream; drain them and keep waiting, except that
 * IDENTIFY is itself returned as an identify report.
 */
static int syna_tcm_exchange(struct syna_tcm *ts, u8 command,
			     const u8 *payload, u16 len)
{
	unsigned int waited = 0;
	int error;

	error = syna_tcm_send(ts, command, payload, len);
	if (error)
		return error;

	usleep_range(SYNA_TCM_TAT_DELAY_US, 2 * SYNA_TCM_TAT_DELAY_US);

	for (;;) {
		error = syna_tcm_get_response(ts);
		if (error && error != -EBADMSG)
			return error;

		if (!error) {
			switch (ts->code) {
			case TCM_STATUS_OK:
				return 0;
			case TCM_STATUS_IDLE:
				/* Response not ready yet, keep polling. */
				break;
			default:
				if (ts->code >= TCM_REPORT_IDENTIFY) {
					if (command == TCM_CMD_IDENTIFY &&
					    ts->code == TCM_REPORT_IDENTIFY)
						return 0;
					break;
				}
				dev_dbg(&ts->spi->dev,
					"command %#x failed, status %#x\n",
					command, ts->code);
				return -EPROTO;
			}
		}

		if (waited >= SYNA_TCM_RESP_TIMEOUT_MS)
			return -ETIMEDOUT;
		msleep(SYNA_TCM_RESP_POLL_MS);
		waited += SYNA_TCM_RESP_POLL_MS;
	}
}

/* Write one [field, value_le16] dynamic-config entry and wait for its ack. */
static int syna_tcm_set_dynamic_config(struct syna_tcm *ts, u8 field, u16 value)
{
	u8 payload[3];

	payload[0] = field;
	put_unaligned_le16(value, &payload[1]);

	return syna_tcm_exchange(ts, TCM_CMD_SET_DYNAMIC_CONFIG, payload,
				 sizeof(payload));
}

/*
 * Enter or leave the controller's low-power wake-on-gesture mode.  This is
 * the alternative to deep sleep used when AoC takes the bus in suspend: the
 * firmware keeps a low-power scan alive so a tap can wake the system.
 */
static int syna_tcm_set_gesture_mode(struct syna_tcm *ts, bool enable)
{
	return syna_tcm_set_dynamic_config(ts, TCM_DC_ENABLE_WAKEUP_GESTURE_MODE,
					   enable ? 1 : 0);
}

/* LSB-first bit field extraction; reads beyond the report yield zero. */
static u32 syna_tcm_get_bits(const u8 *report, unsigned int report_bits,
			     unsigned int offset, unsigned int bits)
{
	u32 value = 0;
	unsigned int i;

	if (offset + bits > report_bits)
		return 0;

	for (i = 0; i < bits; i++) {
		unsigned int pos = offset + i;

		value |= ((report[pos / 8] >> (pos % 8)) & 1) << i;
	}

	return value;
}

/*
 * Skip config entries up to and including the next FOREACH_END.  Entity
 * codes carry a bit-length byte; flow-control codes do not.
 */
static unsigned int syna_tcm_skip_foreach(struct syna_tcm *ts,
					  unsigned int idx)
{
	while (idx < ts->config_len) {
		u8 code = ts->config[idx++];

		switch (code) {
		case TCM_TOUCH_FOREACH_END:
			return idx;
		case TCM_TOUCH_END:
			return idx - 1;
		case TCM_TOUCH_FOREACH_ACTIVE_OBJECT:
		case TCM_TOUCH_FOREACH_OBJECT:
		case TCM_TOUCH_PAD_TO_NEXT_BYTE:
			break;
		default:
			idx++;	/* skip the bit-length byte */
			break;
		}
	}

	return idx;
}

static void syna_tcm_report_frame(struct syna_tcm *ts)
{
	struct input_dev *input = ts->input;
	unsigned int i;

	for_each_set_bit(i, &ts->slots_seen, SYNA_TCM_MAX_SLOTS) {
		struct syna_tcm_object *obj = &ts->objects[i];
		bool active = obj->status == TCM_OBJ_FINGER ||
			      obj->status == TCM_OBJ_GLOVED_OBJECT;

		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, active);
		if (active) {
			input_report_abs(input, ABS_MT_POSITION_X, obj->x);
			input_report_abs(input, ABS_MT_POSITION_Y, obj->y);
		}
	}

	input_mt_sync_frame(input);
	input_sync(input);
}

/*
 * Walk the touch report against the device's report config descriptor.
 * Follows the reference syna_tcm_parse_touch_report() loop semantics.
 */
static void syna_tcm_parse_touch(struct syna_tcm *ts, const u8 *report,
				 unsigned int len)
{
	unsigned int report_bits = len * 8;
	unsigned int offset = 0;
	unsigned int idx = 0;
	unsigned int loop_start = 0;
	unsigned int active_objects = 0;
	unsigned int objects = 0;
	unsigned int obj = 0;
	bool have_active_count = false;
	bool active_only = false;
	unsigned int budget = 8 * SYNA_TCM_CONFIG_MAX;
	u8 code, bits;
	u32 data;

	ts->slots_seen = 0;

	while (idx < ts->config_len && budget--) {
		code = ts->config[idx++];

		switch (code) {
		case TCM_TOUCH_END:
			goto done;
		case TCM_TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			loop_start = idx;
			active_only = true;
			break;
		case TCM_TOUCH_FOREACH_OBJECT:
			obj = 0;
			loop_start = idx;
			active_only = false;
			break;
		case TCM_TOUCH_FOREACH_END:
			if (offset >= report_bits)
				break;
			if (active_only) {
				if (have_active_count) {
					objects++;
					obj++;
					if (objects < active_objects)
						idx = loop_start;
				} else {
					obj++;
					idx = loop_start;
				}
			} else {
				obj++;
				if (obj < ts->max_objects)
					idx = loop_start;
			}
			break;
		case TCM_TOUCH_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			break;
		case TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = ts->config[idx++];
			active_objects = syna_tcm_get_bits(report, report_bits,
							   offset, bits);
			have_active_count = true;
			offset += bits;
			if (!active_objects)
				idx = syna_tcm_skip_foreach(ts, idx);
			break;
		case TCM_TOUCH_OBJECT_N_INDEX:
			bits = ts->config[idx++];
			obj = syna_tcm_get_bits(report, report_bits,
						offset, bits);
			offset += bits;
			break;
		case TCM_TOUCH_OBJECT_N_CLASSIFICATION:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS) {
				ts->objects[obj].status = data;
				ts->slots_seen |= BIT(obj);
			}
			break;
		case TCM_TOUCH_OBJECT_N_X_POSITION:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS)
				ts->objects[obj].x = data;
			break;
		case TCM_TOUCH_OBJECT_N_Y_POSITION:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS)
				ts->objects[obj].y = data;
			break;
		case TCM_TOUCH_OBJECT_N_Z:
			bits = ts->config[idx++];
			data = syna_tcm_get_bits(report, report_bits,
						 offset, bits);
			offset += bits;
			if (obj < SYNA_TCM_MAX_SLOTS)
				ts->objects[obj].z = data;
			break;
		default:
			/* Unknown data entity: skip its bit field. */
			bits = ts->config[idx++];
			offset += bits;
			break;
		}
	}

done:
	syna_tcm_report_frame(ts);
}

static irqreturn_t syna_tcm_irq_thread(int irq, void *data)
{
	struct syna_tcm *ts = data;
	int error;

	mutex_lock(&ts->io_lock);

	error = syna_tcm_get_response(ts);
	if (error)
		goto out;

	switch (ts->code) {
	case TCM_REPORT_TOUCH:
		syna_tcm_parse_touch(ts, &ts->msg[SYNA_TCM_HEADER_SIZE],
				     ts->payload_len);
		break;
	case TCM_REPORT_IDENTIFY:
		/* Unsolicited identify means the controller reset itself. */
		dev_warn(&ts->spi->dev, "unexpected device reset\n");
		break;
	default:
		break;
	}

out:
	mutex_unlock(&ts->io_lock);

	return IRQ_HANDLED;
}

static void syna_tcm_power_off(void *data)
{
	struct syna_tcm *ts = data;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
}

static int syna_tcm_power_on(struct syna_tcm *ts)
{
	int error;

	error = regulator_bulk_enable(ARRAY_SIZE(ts->supplies), ts->supplies);
	if (error)
		return error;

	msleep(SYNA_TCM_POWER_DELAY_MS);

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(SYNA_TCM_RESET_ACTIVE_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(SYNA_TCM_RESET_DELAY_MS);

	return 0;
}

/*
 * After reset, send the raw v1 startup identify byte and read the pending
 * IDENTIFY report.  The first header read tells the protocol generation
 * apart: a TouchComm v1 header leads with the 0xa5 marker, a v2 header
 * CRC-6s to zero.  Drain the identify message, sniff the optional CRC/RC
 * trailer and check that the part came up in application firmware mode.
 */
static int syna_tcm_detect(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	const u8 *trailer;
	int tries;
	int error;

	for (tries = 0; tries < SYNA_TCM_RETRIES; tries++) {
		error = syna_tcm_send_startup_identify(ts);
		if (error)
			return error;

		usleep_range(SYNA_TCM_TAT_DELAY_US,
			     2 * SYNA_TCM_TAT_DELAY_US);

		error = syna_tcm_spi_read(ts, SYNA_TCM_HEADER_SIZE);
		if (error)
			return error;

		if (ts->rx[0] == TCM_V1_MARKER)
			break;

		if (syna_tcm_crc6(ts->rx, SYNA_TCM_HEADER_BITS) == 0 &&
		    ts->rx[0] == TCM_REPORT_IDENTIFY) {
			dev_err(dev, "TouchComm v2 device (%*ph), not supported\n",
				SYNA_TCM_HEADER_SIZE, ts->rx);
			return -ENODEV;
		}

		msleep(20);
	}

	if (tries == SYNA_TCM_RETRIES) {
		dev_err(dev, "no TouchComm v1 header (%*ph)\n",
			SYNA_TCM_HEADER_SIZE, ts->rx);
		return -ENODEV;
	}

	ts->code = ts->rx[1];
	ts->payload_len = get_unaligned_le16(&ts->rx[2]);

	if (ts->code == TCM_REPORT_IDENTIFY && ts->payload_len >= 24 &&
	    ts->payload_len <= SYNA_TCM_PAYLOAD_MAX) {
		memcpy(ts->msg, ts->rx, SYNA_TCM_HEADER_SIZE);
		error = syna_tcm_read_continued(ts);
		if (error) {
			dev_err(dev, "startup identify read failed: %d\n",
				error);
			return error;
		}
	} else {
		/*
		 * No usable startup packet: log the raw stream and ask for
		 * the identification explicitly, mirroring the reference
		 * syna_tcm_v1_detect() fallback.
		 */
		dev_err(dev, "no startup identify, read %*ph\n",
			SYNA_TCM_HEADER_SIZE, ts->rx);
		error = syna_tcm_exchange(ts, TCM_CMD_IDENTIFY, NULL, 0);
		if (error) {
			dev_err(dev, "identify failed, last read %*ph: %d\n",
				SYNA_TCM_HEADER_SIZE, ts->rx, error);
			return error;
		}
		if ((ts->code != TCM_STATUS_OK &&
		     ts->code != TCM_REPORT_IDENTIFY) || ts->payload_len < 24) {
			dev_err(dev, "unexpected identify response %*ph\n",
				SYNA_TCM_HEADER_SIZE, ts->msg);
			return -EPROTO;
		}
	}

	/*
	 * Both trailer features were assumed present for the reads above;
	 * over-reading is safe because the device pads past the end of
	 * message with 0x5a.  Assumed-trailer bytes that read back as
	 * padding mean the feature is absent (reference
	 * syna_tcm_v1_detect()).
	 */
	trailer = &ts->msg[SYNA_TCM_HEADER_SIZE + ts->payload_len];
	if (get_unaligned_le16(&trailer[1]) == 0x5a5a)
		ts->has_crc = false;
	if (trailer[3] == TCM_V1_PADDING)
		ts->has_extra_rc = false;

	if (ts->msg[SYNA_TCM_HEADER_SIZE + 1] != TCM_MODE_APPLICATION_FIRMWARE) {
		dev_err(dev, "device not in application fw mode (mode %#x)\n",
			ts->msg[SYNA_TCM_HEADER_SIZE + 1]);
		return -ENODEV;
	}

	ts->max_wr_size = get_unaligned_le16(&ts->msg[SYNA_TCM_HEADER_SIZE + 22]);

	dev_info(dev, "TouchComm v1, part '%.16s', build %u, crc %s, rc %s\n",
		 (char *)&ts->msg[SYNA_TCM_HEADER_SIZE + 2],
		 get_unaligned_le32(&ts->msg[SYNA_TCM_HEADER_SIZE + 18]),
		 str_yes_no(ts->has_crc), str_yes_no(ts->has_extra_rc));

	return 0;
}

static int syna_tcm_setup_app(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	const u8 *info;
	int error;

	error = syna_tcm_exchange(ts, TCM_CMD_GET_APPLICATION_INFO, NULL, 0);
	if (error)
		return error;
	if (ts->code != TCM_STATUS_OK || ts->payload_len < 38)
		return -EPROTO;

	info = &ts->msg[SYNA_TCM_HEADER_SIZE];
	ts->max_x = get_unaligned_le16(&info[32]);
	ts->max_y = get_unaligned_le16(&info[34]);
	ts->max_objects = get_unaligned_le16(&info[36]);

	if (!ts->max_x || !ts->max_y || !ts->max_objects)
		return -ENODEV;

	ts->max_objects = min_t(u16, ts->max_objects, SYNA_TCM_MAX_SLOTS);

	error = syna_tcm_exchange(ts, TCM_CMD_GET_TOUCH_REPORT_CONFIG,
				  NULL, 0);
	if (error)
		return error;
	if (ts->code != TCM_STATUS_OK || !ts->payload_len ||
	    ts->payload_len > SYNA_TCM_CONFIG_MAX)
		return -EPROTO;

	memcpy(ts->config, &ts->msg[SYNA_TCM_HEADER_SIZE], ts->payload_len);
	ts->config_len = ts->payload_len;

	dev_dbg(dev, "%ux%u, %u objects, %u-byte report config\n",
		ts->max_x, ts->max_y, ts->max_objects, ts->config_len);

	return 0;
}

static int syna_tcm_setup_input(struct syna_tcm *ts)
{
	struct input_dev *input;
	int error;

	input = devm_input_allocate_device(&ts->spi->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Synaptics TouchComm Touchscreen";
	input->id.bustype = BUS_SPI;
	input->phys = "syna_tcm/input0";

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);

	/* AoC tap-to-wake (TBN LPTW) is delivered as a KEY_WAKEUP press. */
	input_set_capability(input, EV_KEY, KEY_WAKEUP);

	error = input_mt_init_slots(input, ts->max_objects,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		return error;

	ts->input = input;

	return 0;
}

/*
 * AoC detected a low-power touch-wake gesture while it owned the bus and woke
 * the AP over its mailbox (the touch IRQ stays silent).  This runs in the TBN
 * read-thread context once the AP is already awake: report a wakeup event so
 * the system does not immediately re-suspend, and inject a KEY_WAKEUP press so
 * userspace turns the display on.
 */
static void syna_tcm_lptw_report(struct TbnLptwEvent *lptw, void *data)
{
	struct syna_tcm *ts = data;

	pm_wakeup_event(&ts->spi->dev, SYNA_TCM_WAKE_HOLD_MS);

	input_report_key(ts->input, KEY_WAKEUP, 1);
	input_sync(ts->input);
	input_report_key(ts->input, KEY_WAKEUP, 0);
	input_sync(ts->input);
}

static void syna_tcm_tbn_unregister(void *data)
{
	struct syna_tcm *ts = data;

	register_tbn_lptw_callback(NULL, NULL);
	unregister_tbn(&ts->tbn_mask);
}

/*
 * Opt in to the Touch Bus Negotiator when the DT asks for it.  Registration
 * only reserves a device bit in the (global) negotiator; if the negotiator has
 * not probed yet register_tbn() yields mask 0 and the driver quietly keeps the
 * deep-sleep suspend path.  The feature must never be able to break touch, so a
 * missing/late TBN is not fatal.
 */
static void syna_tcm_setup_tbn(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;

	ts->tbn_enabled = device_property_read_bool(dev, "goog,tbn-enabled");
	if (!ts->tbn_enabled)
		return;

	register_tbn(&ts->tbn_mask);
	if (!ts->tbn_mask) {
		dev_warn(dev, "TBN unavailable, using deep-sleep suspend\n");
		ts->tbn_enabled = false;
		return;
	}

	if (devm_add_action_or_reset(dev, syna_tcm_tbn_unregister, ts)) {
		ts->tbn_enabled = false;
		return;
	}

	register_tbn_lptw_callback(syna_tcm_lptw_report, ts);
	devm_device_init_wakeup(dev);

	dev_info(dev, "TBN bus handoff enabled, mask %#x\n", ts->tbn_mask);
}

/*
 * Enter low power: when the AoC bus handoff is available, put the controller
 * into low-power gesture mode and release the SPI bus to AoC so it can watch
 * for a gesture.  Otherwise -- or on any failure -- fall back to deep sleep
 * with the AP still owning the bus.  The IRQ is disabled either way: once AoC
 * owns the bus the AP must not drive SPI, and AoC signals a wake over its own
 * mailbox, not this line.
 *
 * Idempotent; caller holds pm_lock.
 */
static void syna_tcm_enter_low_power(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	bool use_tbn = ts->tbn_enabled && tbn_ready();
	int error;

	if (ts->powered_off)
		return;
	ts->powered_off = true;

	disable_irq(ts->spi->irq);

	mutex_lock(&ts->io_lock);
	if (use_tbn) {
		error = syna_tcm_set_gesture_mode(ts, true);
		if (error) {
			/*
			 * Gesture mode did not arm: do not hand a bus AoC
			 * cannot use to it.  Fall back to deep sleep, AP keeps
			 * the bus.
			 */
			dev_warn(dev,
				 "gesture mode failed (%d), using deep sleep\n",
				 error);
			use_tbn = false;
			error = syna_tcm_exchange(ts, TCM_CMD_ENTER_DEEP_SLEEP,
						  NULL, 0);
		}
	} else {
		error = syna_tcm_exchange(ts, TCM_CMD_ENTER_DEEP_SLEEP, NULL, 0);
	}
	mutex_unlock(&ts->io_lock);
	if (error)
		dev_warn(dev, "failed to enter low power: %d\n", error);

	if (use_tbn) {
		/*
		 * Hand the bus to AoC.  tbn_suspended is set unconditionally so
		 * resume always reclaims it -- even if the release times out we
		 * must never leave touch stranded with AoC owning the bus.
		 */
		ts->tbn_suspended = true;
		error = tbn_release_bus(ts->tbn_mask);
		if (error)
			dev_warn(dev, "tbn_release_bus failed: %d\n", error);
	}
}

/*
 * Leave low power: mirror syna_tcm_enter_low_power().  If the bus went to AoC,
 * reclaim it before any SPI traffic and leave gesture mode; otherwise just exit
 * deep sleep.
 *
 * Idempotent; caller holds pm_lock.
 */
static void syna_tcm_exit_low_power(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	int error;

	if (!ts->powered_off)
		return;
	ts->powered_off = false;

	if (ts->tbn_suspended) {
		error = tbn_request_bus(ts->tbn_mask);
		if (error)
			dev_warn(dev, "tbn_request_bus failed: %d\n", error);
		ts->tbn_suspended = false;

		/*
		 * Whatever state the controller is in now is AoC's doing, not
		 * ours: it puts the part into deep sleep when it takes the bus
		 * and reprograms it to watch for a gesture, and it may have
		 * reset it. So do not just undo the one bit we set -- wake it,
		 * leave gesture mode, and re-read what it says about itself.
		 */
		mutex_lock(&ts->io_lock);
		error = syna_tcm_exchange(ts, TCM_CMD_EXIT_DEEP_SLEEP, NULL, 0);
		if (error)
			dev_warn(dev, "failed to exit deep sleep: %d\n", error);

		error = syna_tcm_set_gesture_mode(ts, false);
		if (error)
			dev_warn(dev, "failed to exit gesture mode: %d\n", error);

		error = syna_tcm_setup_app(ts);
		if (error)
			dev_warn(dev, "failed to re-read app info: %d\n", error);
		mutex_unlock(&ts->io_lock);
	} else {
		mutex_lock(&ts->io_lock);
		error = syna_tcm_exchange(ts, TCM_CMD_EXIT_DEEP_SLEEP, NULL, 0);
		mutex_unlock(&ts->io_lock);
		if (error)
			dev_warn(dev, "failed to exit deep sleep: %d\n", error);
	}

	enable_irq(ts->spi->irq);
}

/*
 * The panel going dark is what should hand the bus to AoC: AoC can only watch
 * for a gesture while it owns the bus, and the screen can be off long before
 * (or without) the AP suspending.  Tying the handoff to system sleep alone
 * would mean no gesture is seen while the AP is awake with the display off.
 */
/*
 * Called on the way out, before the follower is removed. Removing a follower
 * from a live panel replays "unpreparing", which would release the bus to AoC
 * just as the driver that reclaims it disappears.
 */
static void syna_tcm_stop_following(void *data)
{
	struct syna_tcm *ts = data;

	mutex_lock(&ts->pm_lock);
	ts->following = false;
	mutex_unlock(&ts->pm_lock);
}

static int syna_tcm_panel_prepared(struct drm_panel_follower *follower)
{
	struct syna_tcm *ts = container_of(follower, struct syna_tcm, follower);

	mutex_lock(&ts->pm_lock);
	ts->off_by_panel = false;
	if (!ts->off_by_pm)
		syna_tcm_exit_low_power(ts);
	mutex_unlock(&ts->pm_lock);
	return 0;
}

static int syna_tcm_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct syna_tcm *ts = container_of(follower, struct syna_tcm, follower);

	mutex_lock(&ts->pm_lock);
	if (ts->following) {
		ts->off_by_panel = true;
		syna_tcm_enter_low_power(ts);
	}
	mutex_unlock(&ts->pm_lock);
	return 0;
}

static const struct drm_panel_follower_funcs syna_tcm_follower_funcs = {
	.panel_prepared = syna_tcm_panel_prepared,
	.panel_unpreparing = syna_tcm_panel_unpreparing,
};

static int syna_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct syna_tcm *ts;
	int error;

	if (!spi->irq) {
		dev_err(dev, "no irq specified\n");
		return -EINVAL;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->msg = devm_kzalloc(dev, SYNA_TCM_MSG_MAX, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, SYNA_TCM_MSG_MAX, GFP_KERNEL);
	ts->tx = devm_kzalloc(dev, SYNA_TCM_OUT_MAX, GFP_KERNEL);
	if (!ts->msg || !ts->rx || !ts->tx)
		return -ENOMEM;

	ts->spi = spi;
	mutex_init(&ts->io_lock);
	mutex_init(&ts->pm_lock);
	spi_set_drvdata(spi, ts);

	/* Assumed present until the detect-time trailer sniff says no. */
	ts->has_crc = true;
	ts->has_extra_rc = true;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	error = spi_setup(spi);
	if (error)
		return error;

	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "avdd";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->supplies),
					ts->supplies);
	if (error)
		return dev_err_probe(dev, error, "failed to get supplies\n");

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	error = syna_tcm_power_on(ts);
	if (error)
		return dev_err_probe(dev, error, "failed to power on\n");

	error = devm_add_action_or_reset(dev, syna_tcm_power_off, ts);
	if (error)
		return error;

	error = syna_tcm_detect(ts);
	if (error)
		return error;

	error = syna_tcm_setup_app(ts);
	if (error)
		return dev_err_probe(dev, error, "application setup failed\n");

	error = syna_tcm_setup_input(ts);
	if (error)
		return error;

	syna_tcm_setup_tbn(ts);

	error = devm_request_threaded_irq(dev, spi->irq, NULL,
					  syna_tcm_irq_thread,
					  IRQF_ONESHOT, "syna_tcm", ts);
	if (error)
		return dev_err_probe(dev, error, "failed to request irq\n");

	/*
	 * Follow the panel if the DT says which one. Without it the driver
	 * keeps its previous behaviour and enters low power on system sleep
	 * only, which costs wake gestures while the AP is awake with the
	 * display off, but works.
	 */
	ts->follower.funcs = &syna_tcm_follower_funcs;
	error = drm_panel_add_follower(dev, &ts->follower);
	if (error == -EPROBE_DEFER)
		return error;
	if (error) {
		dev_info(dev, "not following a panel (%d), using system sleep\n",
			 error);
	} else {
		ts->following = true;
		/*
		 * Registered after the follower so it unwinds first: the
		 * removal path would otherwise hand the bus to AoC on the way
		 * out, with nothing left to reclaim it.
		 */
		error = devm_add_action_or_reset(dev, syna_tcm_stop_following,
						 ts);
		if (error)
			return error;
	}

	return 0;
}

/*
 * System sleep still powers the controller down, even when the panel drives it:
 * the system can suspend with the display on, and the part must not be left
 * scanning.  Coming back is conditional -- if the panel is dark, it stays down
 * until the panel says otherwise.
 */
static int syna_tcm_suspend(struct device *dev)
{
	struct syna_tcm *ts = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&ts->pm_lock);
	ts->off_by_pm = true;
	syna_tcm_enter_low_power(ts);
	mutex_unlock(&ts->pm_lock);

	return 0;
}

static int syna_tcm_resume(struct device *dev)
{
	struct syna_tcm *ts = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&ts->pm_lock);
	ts->off_by_pm = false;
	if (!ts->off_by_panel)
		syna_tcm_exit_low_power(ts);
	mutex_unlock(&ts->pm_lock);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(syna_tcm_pm_ops,
				syna_tcm_suspend, syna_tcm_resume);

static const struct of_device_id syna_tcm_of_match[] = {
	{ .compatible = "syna,tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, syna_tcm_of_match);

static const struct spi_device_id syna_tcm_spi_id[] = {
	{ "tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, syna_tcm_spi_id);

static struct spi_driver syna_tcm_driver = {
	.driver = {
		.name		= "syna_tcm",
		.of_match_table	= syna_tcm_of_match,
		.pm		= pm_sleep_ptr(&syna_tcm_pm_ops),
	},
	.id_table	= syna_tcm_spi_id,
	.probe		= syna_tcm_probe,
};
module_spi_driver(syna_tcm_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("Synaptics TouchComm v1 SPI touchscreen driver");
MODULE_LICENSE("GPL");
