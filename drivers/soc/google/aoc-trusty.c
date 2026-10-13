// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google AOC - starting the core through the Trusty GSA hardware manager.
 *
 * The GSA authenticates the AOC image over its mailbox, but only this Trusty
 * application programs the AOC's secure SysMMU context and releases the core
 * from reset; the mailbox has no command for it.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/types.h>

#include <linux/trusty/trusty_ipc.h>

#include "aoc-boot.h"

#define AOC_HWMGR_PORT		"com.android.trusty.gsa.hwmgr.aoc"

/* Sub-commands of the hwmgr STATE command (enum gsa_aoc_cmd). */
#define AOC_TZ_GET_STATE	0
#define AOC_TZ_START		1

/* GSA hwmgr tipc protocol (from the vendor hwmgr-ipc.h). */
#define HWMGR_CMD_RESP		(1U << 31)
#define HWMGR_CMD_STATE_CMD	1

struct hwmgr_state_req {
	u32 hdr_cmd;			/* HWMGR_CMD_STATE_CMD */
	u32 state_cmd;			/* AOC_TZ_START / AOC_TZ_GET_STATE */
};

struct hwmgr_state_rsp {
	u32 hdr_cmd;			/* HWMGR_CMD_STATE_CMD | HWMGR_CMD_RESP */
	u32 hdr_err;
	u32 state;
};

#define AOC_TZ_CONNECT_TIMEOUT_MS	5000
#define AOC_TZ_MSG_TIMEOUT_MS		10000
#define AOC_TZ_BUF_TIMEOUT_MS		10000

/**
 * struct aoc_trusty - this backend's state
 * @dev:	the AOC device, for logging
 * @chan:	the tipc channel, opened on first use
 * @reply:	completed by a reply or a channel event
 * @req_lock:	serialises requests
 * @rsp_lock:	protects @rsp, @rsp_size and @rsp_res against the callback
 * @rsp:	where the callback copies a reply, or NULL if none is awaited
 * @rsp_size:	capacity of @rsp
 * @rsp_res:	bytes copied, or a negative errno
 */
struct aoc_trusty {
	struct device *dev;
	struct tipc_chan *chan;
	struct completion reply;
	struct mutex req_lock;
	struct mutex rsp_lock;
	void *rsp;
	size_t rsp_size;
	int rsp_res;
};

/* A reply arrived - copy it out and wake the waiter. */
static struct tipc_msg_buf *aoc_trusty_handle_msg(void *data,
						  struct tipc_msg_buf *rxbuf)
{
	struct aoc_trusty *tz = data;

	mutex_lock(&tz->rsp_lock);
	if (tz->rsp) {
		size_t len = mb_avail_data(rxbuf);

		if (len <= tz->rsp_size) {
			memcpy(tz->rsp, mb_get_data(rxbuf, len), len);
			tz->rsp_res = len;
		} else {
			tz->rsp_res = -EMSGSIZE;
		}
	}
	mutex_unlock(&tz->rsp_lock);
	complete(&tz->reply);

	return rxbuf;
}

/* Connect/disconnect events also wake the waiter. */
static void aoc_trusty_handle_event(void *data, int event)
{
	struct aoc_trusty *tz = data;

	complete(&tz->reply);
}

static const struct tipc_chan_ops aoc_trusty_chan_ops = {
	.handle_msg = aoc_trusty_handle_msg,
	.handle_event = aoc_trusty_handle_event,
};

/* Open the tipc channel to the GSA AOC hardware manager and wait for it. */
static int aoc_trusty_connect(struct aoc_trusty *tz)
{
	struct tipc_chan *chan;
	long left;
	int ret;

	chan = tipc_create_channel(NULL, &aoc_trusty_chan_ops, tz);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	reinit_completion(&tz->reply);
	ret = tipc_chan_connect(chan, AOC_HWMGR_PORT);
	if (ret) {
		tipc_chan_destroy(chan);
		return ret;
	}

	left = wait_for_completion_timeout(&tz->reply,
					   msecs_to_jiffies(AOC_TZ_CONNECT_TIMEOUT_MS));
	if (left <= 0) {
		tipc_chan_shutdown(chan);
		tipc_chan_destroy(chan);
		return -ETIMEDOUT;
	}

	tz->chan = chan;
	return 0;
}

/*
 * Send a hardware-manager state command and return the AOC state the GSA
 * reports.  The channel is opened on first use: Trusty may not be ready when
 * the AOC probes.
 */
static int aoc_trusty_state_cmd(struct aoc_trusty *tz, u32 cmd)
{
	struct hwmgr_state_req req = {
		.hdr_cmd = HWMGR_CMD_STATE_CMD,
		.state_cmd = cmd,
	};
	struct hwmgr_state_rsp rsp = { };
	struct tipc_msg_buf *txbuf;
	long left;
	int ret;

	mutex_lock(&tz->req_lock);

	if (!tz->chan) {
		ret = aoc_trusty_connect(tz);
		if (ret) {
			dev_err(tz->dev, "cannot reach %s: %d\n",
				AOC_HWMGR_PORT, ret);
			goto out;
		}
		dev_info(tz->dev, "connected to %s\n", AOC_HWMGR_PORT);
	}

	txbuf = tipc_chan_get_txbuf_timeout(tz->chan, AOC_TZ_BUF_TIMEOUT_MS);
	if (IS_ERR(txbuf)) {
		ret = PTR_ERR(txbuf);
		goto out;
	}
	memcpy(mb_put_data(txbuf, sizeof(req)), &req, sizeof(req));

	mutex_lock(&tz->rsp_lock);
	tz->rsp = &rsp;
	tz->rsp_size = sizeof(rsp);
	tz->rsp_res = -ENOMSG;
	mutex_unlock(&tz->rsp_lock);

	reinit_completion(&tz->reply);
	ret = tipc_chan_queue_msg(tz->chan, txbuf);
	if (ret) {
		tipc_chan_put_txbuf(tz->chan, txbuf);
		goto out_detach;
	}

	left = wait_for_completion_timeout(&tz->reply,
					   msecs_to_jiffies(AOC_TZ_MSG_TIMEOUT_MS));
	ret = (left <= 0) ? -ETIMEDOUT : tz->rsp_res;

out_detach:
	mutex_lock(&tz->rsp_lock);
	tz->rsp = NULL;
	tz->rsp_size = 0;
	mutex_unlock(&tz->rsp_lock);
out:
	mutex_unlock(&tz->req_lock);

	if (ret < 0)
		return ret;
	if (ret < (int)sizeof(rsp) ||
	    rsp.hdr_cmd != (HWMGR_CMD_STATE_CMD | HWMGR_CMD_RESP) || rsp.hdr_err)
		return -EIO;

	return rsp.state;
}

static void *aoc_trusty_attach(struct device *dev)
{
	struct aoc_trusty *tz;

	tz = devm_kzalloc(dev, sizeof(*tz), GFP_KERNEL);
	if (!tz)
		return ERR_PTR(-ENOMEM);

	tz->dev = dev;
	mutex_init(&tz->req_lock);
	mutex_init(&tz->rsp_lock);
	init_completion(&tz->reply);

	return tz;
}

static void aoc_trusty_detach(void *priv)
{
	struct aoc_trusty *tz = priv;

	if (tz->chan) {
		tipc_chan_shutdown(tz->chan);
		tipc_chan_destroy(tz->chan);
		tz->chan = NULL;
	}
}

static int aoc_trusty_get_state(void *priv)
{
	return aoc_trusty_state_cmd(priv, AOC_TZ_GET_STATE);
}

static int aoc_trusty_start(void *priv)
{
	return aoc_trusty_state_cmd(priv, AOC_TZ_START);
}

const struct aoc_boot_backend aoc_boot_trusty = {
	.name		= "Trusty",
	.attach		= aoc_trusty_attach,
	.detach		= aoc_trusty_detach,
	.get_state	= aoc_trusty_get_state,
	.start		= aoc_trusty_start,
};
