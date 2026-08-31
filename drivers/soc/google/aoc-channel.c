// SPDX-License-Identifier: GPL-2.0
/*
 * AoC Channelized Comms (AOCC) - in-kernel client.
 *
 * A single AOC service carries many logical channels.  Each message begins
 * with a 4-byte header holding the channel index in its low 31 bits and a
 * non-wake flag in the top bit; channel 0 carries control messages that open,
 * close and throttle the others.
 *
 * The index space is global rather than per-service, and the AOC answers on
 * whichever service matches a sensor's wakeup class -- a stream started on
 * com.google.usf can arrive on com.google.usf.non_wake_up carrying the index
 * it was opened with.  Demux therefore matches on the index alone and ignores
 * which service a message came in on, which is also what the vendor driver
 * does.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#define pr_fmt(fmt) "aoc_chan: " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <linux/soc/google/aoc.h>
#include <linux/soc/google/aoc_channel.h>

/*
 * Stop one short of the 31-bit field so two racing openers cannot both pass
 * the bound check and push the counter past it.
 */
#define AOCC_MAX_CHANNEL_INDEX	((1 << 30) - 1)

struct aocc_msg {
	u32 channel_index : 31;
	u32 non_wake_up : 1;
	char payload[AOCC_MAX_MSG_SIZE - sizeof(u32)];
} __packed;

/* Control message; channel_index is always 0. */
struct aocc_cmd {
	s32 channel_index;
	s32 command_code;
	s32 channel_to_modify;
} __packed;

enum aocc_cmd_code {
	AOCC_CMD_OPEN_CHANNEL = 0,
	AOCC_CMD_CLOSE_CHANNEL,
	AOCC_CMD_BLOCK_CHANNEL,
	AOCC_CMD_UNBLOCK_CHANNEL,
	AOCC_CMD_SUSPEND_PREPARE,
	AOCC_CMD_WAKEUP_COMPLETE,
};

/* One per AOC service we have attached the demux to. */
struct aocc_transport {
	struct list_head node;
	struct device *aoc_dev;
	const char *name;
	struct aoc_service *svc;
	struct work_struct rx_work;
	struct mutex tx_lock;		/* serialises writes into the ring */
	unsigned int users;
};

struct aocc_channel {
	struct list_head node;
	struct aocc_transport *tr;
	u32 channel_index;
	void (*rx)(void *ctx, const void *payload, size_t len);
	void *ctx;
};

static LIST_HEAD(aocc_transports);
static DEFINE_MUTEX(aocc_transports_lock);

static LIST_HEAD(aocc_channels);
static DEFINE_MUTEX(aocc_channels_lock);

static u32 aocc_next_index = 1;

static int aocc_send_cmd(struct aocc_transport *tr, enum aocc_cmd_code code,
			 u32 channel)
{
	struct aocc_cmd cmd = {
		.channel_index = 0,
		.command_code = code,
		.channel_to_modify = channel,
	};
	int ret;

	mutex_lock(&tr->tx_lock);
	ret = aoc_service_write(tr->svc, &cmd, sizeof(cmd));
	mutex_unlock(&tr->tx_lock);

	return ret;
}

/*
 * Drain the service and hand each message to the channel that owns it.  Runs
 * in a worker rather than the doorbell handler because the callbacks and the
 * close-on-unknown-channel reply both need process context.
 */
static void aocc_rx_work(struct work_struct *work)
{
	struct aocc_transport *tr =
		container_of(work, struct aocc_transport, rx_work);
	struct aocc_msg *msg;

	msg = kmalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return;

	while (aoc_service_can_read(tr->svc)) {
		struct aocc_channel *chan;
		bool handled = false;
		size_t payload_len;
		int ret;

		ret = aoc_service_read(tr->svc, msg, sizeof(*msg));
		if (ret < 0)
			break;
		if (ret < (int)sizeof(u32)) {
			pr_err_ratelimited("%s: runt message (%d bytes)\n",
					   tr->name, ret);
			continue;
		}
		payload_len = ret - sizeof(u32);

		mutex_lock(&aocc_channels_lock);
		list_for_each_entry(chan, &aocc_channels, node) {
			if (chan->channel_index != msg->channel_index)
				continue;
			chan->rx(chan->ctx, msg->payload, payload_len);
			handled = true;
			break;
		}
		mutex_unlock(&aocc_channels_lock);

		if (!handled) {
			pr_warn_ratelimited("%s: no handler for channel %u\n",
					    tr->name, msg->channel_index);
			/* Tell the AOC to stop producing on a dead channel. */
			aocc_send_cmd(tr, AOCC_CMD_CLOSE_CHANNEL,
				      msg->channel_index);
		}
	}

	kfree(msg);
}

static void aocc_doorbell(struct aoc_service *svc, void *priv)
{
	struct aocc_transport *tr = priv;

	schedule_work(&tr->rx_work);
}

/* Callers hold aocc_transports_lock. */
static struct aocc_transport *aocc_transport_get(struct device *aoc_dev,
						 const char *name)
{
	struct aocc_transport *tr;

	list_for_each_entry(tr, &aocc_transports, node) {
		if (tr->aoc_dev == aoc_dev && !strcmp(tr->name, name)) {
			tr->users++;
			return tr;
		}
	}

	tr = kzalloc(sizeof(*tr), GFP_KERNEL);
	if (!tr)
		return ERR_PTR(-ENOMEM);

	tr->name = kstrdup(name, GFP_KERNEL);
	if (!tr->name) {
		kfree(tr);
		return ERR_PTR(-ENOMEM);
	}

	tr->svc = aoc_service_find(aoc_dev, name);
	if (!tr->svc) {
		kfree(tr->name);
		kfree(tr);
		return ERR_PTR(-ENODEV);
	}

	tr->aoc_dev = aoc_dev;
	tr->users = 1;
	mutex_init(&tr->tx_lock);
	INIT_WORK(&tr->rx_work, aocc_rx_work);
	list_add(&tr->node, &aocc_transports);

	aoc_service_set_handler(tr->svc, aocc_doorbell, tr);

	/* Anything the AOC queued before we attached is still waiting. */
	schedule_work(&tr->rx_work);

	return tr;
}

/* Callers hold aocc_transports_lock. */
static void aocc_transport_put(struct aocc_transport *tr)
{
	if (--tr->users)
		return;

	aoc_service_set_handler(tr->svc, NULL, NULL);
	list_del(&tr->node);

	/*
	 * Safe under aocc_transports_lock: the demux worker only ever takes
	 * aocc_channels_lock and the transport's own tx_lock.
	 */
	cancel_work_sync(&tr->rx_work);

	mutex_destroy(&tr->tx_lock);
	kfree(tr->name);
	kfree(tr);
}

int aocc_kernel_listen(struct device *aoc_dev, const char *service_name)
{
	struct aocc_transport *tr;

	if (!aoc_dev || !service_name)
		return -EINVAL;

	mutex_lock(&aocc_transports_lock);
	tr = aocc_transport_get(aoc_dev, service_name);
	mutex_unlock(&aocc_transports_lock);

	return PTR_ERR_OR_ZERO(tr);
}
EXPORT_SYMBOL_GPL(aocc_kernel_listen);

void aocc_kernel_unlisten(struct device *aoc_dev, const char *service_name)
{
	struct aocc_transport *tr;

	if (!aoc_dev || !service_name)
		return;

	mutex_lock(&aocc_transports_lock);
	list_for_each_entry(tr, &aocc_transports, node) {
		if (tr->aoc_dev == aoc_dev && !strcmp(tr->name, service_name)) {
			aocc_transport_put(tr);
			break;
		}
	}
	mutex_unlock(&aocc_transports_lock);
}
EXPORT_SYMBOL_GPL(aocc_kernel_unlisten);

struct aocc_channel *
aocc_kernel_open_channel(struct device *aoc_dev, const char *service_name,
			 void (*rx)(void *ctx, const void *payload, size_t len),
			 void *ctx)
{
	struct aocc_transport *tr;
	struct aocc_channel *chan;
	int ret;

	if (!aoc_dev || !service_name || !rx)
		return ERR_PTR(-EINVAL);

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&aocc_transports_lock);
	tr = aocc_transport_get(aoc_dev, service_name);
	mutex_unlock(&aocc_transports_lock);
	if (IS_ERR(tr)) {
		kfree(chan);
		return ERR_CAST(tr);
	}

	chan->tr = tr;
	chan->rx = rx;
	chan->ctx = ctx;

	mutex_lock(&aocc_channels_lock);
	if (aocc_next_index >= AOCC_MAX_CHANNEL_INDEX) {
		mutex_unlock(&aocc_channels_lock);
		ret = -EMFILE;
		goto err_put;
	}
	chan->channel_index = aocc_next_index++;
	/* Publish before opening, so an immediate reply already routes. */
	list_add(&chan->node, &aocc_channels);
	mutex_unlock(&aocc_channels_lock);

	ret = aocc_send_cmd(tr, AOCC_CMD_OPEN_CHANNEL, chan->channel_index);
	if (ret < 0) {
		mutex_lock(&aocc_channels_lock);
		list_del(&chan->node);
		mutex_unlock(&aocc_channels_lock);
		goto err_put;
	}

	return chan;

err_put:
	mutex_lock(&aocc_transports_lock);
	aocc_transport_put(tr);
	mutex_unlock(&aocc_transports_lock);
	kfree(chan);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(aocc_kernel_open_channel);

int aocc_kernel_write(struct aocc_channel *chan, const void *payload,
		      size_t len)
{
	struct aocc_transport *tr;
	struct aocc_msg *msg;
	int ret;

	if (IS_ERR_OR_NULL(chan))
		return -EINVAL;
	if (len > AOCC_MAX_PAYLOAD)
		return -EMSGSIZE;

	tr = chan->tr;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->channel_index = chan->channel_index;
	msg->non_wake_up = 0;
	memcpy(msg->payload, payload, len);

	mutex_lock(&tr->tx_lock);
	ret = aoc_service_write(tr->svc, msg, sizeof(u32) + len);
	mutex_unlock(&tr->tx_lock);

	kfree(msg);

	return ret < 0 ? ret : (int)len;
}
EXPORT_SYMBOL_GPL(aocc_kernel_write);

void aocc_kernel_close_channel(struct aocc_channel *chan)
{
	struct aocc_transport *tr;

	if (IS_ERR_OR_NULL(chan))
		return;

	tr = chan->tr;

	mutex_lock(&aocc_channels_lock);
	list_del(&chan->node);
	mutex_unlock(&aocc_channels_lock);

	aocc_send_cmd(tr, AOCC_CMD_CLOSE_CHANNEL, chan->channel_index);

	mutex_lock(&aocc_transports_lock);
	aocc_transport_put(tr);
	mutex_unlock(&aocc_transports_lock);

	kfree(chan);
}
EXPORT_SYMBOL_GPL(aocc_kernel_close_channel);

MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_DESCRIPTION("Google AoC channelized comms, in-kernel client");
MODULE_LICENSE("GPL");
