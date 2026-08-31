// SPDX-License-Identifier: GPL-2.0-only
/*
 * Client session for the AoC USF (Unified Sensor Framework) firmware.
 *
 * The transport half of talking to USF: AOCC channels, serialised
 * request/response transactions, server-handle resolution, sensor enumeration
 * and sampling control. Split out of the USF->IIO bridge so the input side
 * (gesture wake) can speak the same protocol without duplicating the
 * transaction machinery.
 *
 * Copyright 2026 Steffen Deusch
 */

#define pr_fmt(fmt) "usf: " fmt

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#include <linux/soc/google/aoc_channel.h>
#include <linux/soc/google/usf.h>

#define USF_WAKE_SERVICE	"com.google.usf"
#define USF_NONWAKE_SERVICE	"com.google.usf.non_wake_up"

#define USF_CTL_TIMEOUT_MS	5000	/* per request/response round-trip */
#define USF_RESP_MAX		1024	/* one AOCC MTU */
#define USF_SETTLE_TRIES	25	/* GetSensorList settle: 25 * 200 ms */
#define USF_SETTLE_MS		200

struct usf_session {
	struct device *dev;
	struct device *aoc_dev;

	void (*sample)(void *ctx, const u8 *pay, u32 plen);
	void *sample_ctx;

	/* AOCC transport: wake-class traffic on @wake, the rest on @nonwake. */
	struct aocc_channel *wake;
	struct aocc_channel *nonwake;

	/* Control request/response, serialised by @ctl_lock. */
	struct mutex ctl_lock;
	struct usf_fbb fbb;		/* request builder scratch */
	u32 txn;			/* monotonic transaction id */

	/* Response slot, filled by the rx callback, protected by @resp_lock. */
	spinlock_t resp_lock;
	struct completion resp_done;
	u32 resp_txn;			/* awaited txn, or U32_MAX for none */
	u32 resp_ready;			/* txn whose payload is in resp[] */
	u32 resp_len;
	u8 resp[USF_RESP_MAX];

	/* Server handles, resolved at runtime -- never hardcoded. */
	u32 sensor_mgr;
	u32 sample_chan;
};

static void usf_session_rx(void *ctx, const void *payload, size_t len)
{
	struct usf_session *s = ctx;
	const u8 *pay;
	u32 type, plen, rtxn;
	unsigned long flags;
	bool matched = false;

	if (!usf_parse_outer(payload, len, &type, &pay, &plen) || !pay)
		return;
	if (type == USF_T_SAMPLE) {
		if (s->sample)
			s->sample(s->sample_ctx, pay, plen);
		return;
	}
	if (type != USF_T_RESPONSE)
		return;

	rtxn = usf_resp_txn(pay, plen);

	spin_lock_irqsave(&s->resp_lock, flags);
	if (s->resp_txn != U32_MAX && rtxn == s->resp_txn) {
		s->resp_len = min_t(u32, plen, USF_RESP_MAX);
		memcpy(s->resp, pay, s->resp_len);
		s->resp_ready = rtxn;	/* mark which txn's payload resp[] holds */
		s->resp_txn = U32_MAX;
		matched = true;
	}
	spin_unlock_irqrestore(&s->resp_lock, flags);

	if (matched)
		complete(&s->resp_done);
}

/*
 * Send one request on the wake channel and wait for its response. Caller holds
 * ctl_lock (which also owns @fbb, so @req must stay valid across the write).
 * On success @resp/@resp_len point at s->resp (valid until the next call).
 */
static int usf_ctl(struct usf_session *s, u32 txn, const u8 *req, size_t reqlen,
		   const u8 **resp, u32 *resp_len)
{
	unsigned long flags;
	long left;
	int ret;

	reinit_completion(&s->resp_done);
	spin_lock_irqsave(&s->resp_lock, flags);
	s->resp_txn = txn;
	s->resp_ready = U32_MAX;	/* discard any stale payload */
	spin_unlock_irqrestore(&s->resp_lock, flags);

	ret = aocc_kernel_write(s->wake, req, reqlen);
	if (ret < 0)
		goto clear;

	left = wait_for_completion_timeout(&s->resp_done,
					   msecs_to_jiffies(USF_CTL_TIMEOUT_MS));

	/*
	 * Trust resp[] only if it actually holds this txn's payload. The
	 * completion token alone is not enough: a stalled rx could complete()
	 * for a prior txn after that call already timed out, leaking a wakeup
	 * into this one. resp_ready closes that window deterministically.
	 */
	spin_lock_irqsave(&s->resp_lock, flags);
	s->resp_txn = U32_MAX;
	if (s->resp_ready == txn) {
		*resp = s->resp;
		*resp_len = s->resp_len;
		ret = 0;
	} else {
		ret = left ? -EIO : -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&s->resp_lock, flags);
	return ret;

clear:
	spin_lock_irqsave(&s->resp_lock, flags);
	s->resp_txn = U32_MAX;
	spin_unlock_irqrestore(&s->resp_lock, flags);
	return ret;
}

/* GetServer(uuid) -> server handle (0 = not found / error). */
static u32 usf_get_server(struct usf_session *s, const u8 uuid[16],
			  const char *label)
{
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn, handle = 0;
	int ret;

	mutex_lock(&s->ctl_lock);
	txn = s->txn++;
	ret = usf_build_get_server(&s->fbb, txn, uuid, &req, &reqlen);
	if (!ret)
		ret = usf_ctl(s, txn, req, reqlen, &resp, &rlen);
	if (!ret) {
		body = usf_resp_body(resp, rlen, &blen);
		if (body)
			handle = usf_fb_u32(body, blen, 0, 0);
	}
	mutex_unlock(&s->ctl_lock);

	if (ret)
		dev_warn(s->dev, "GetServer(%s) failed: %d\n", label, ret);
	else
		dev_info(s->dev, "GetServer(%s) -> handle %u%s\n", label,
			 handle, handle ? "" : " (NOT FOUND)");
	return handle;
}

struct usf_session *usf_session_alloc(struct device *dev,
				      struct device *aoc_dev,
				      void (*sample)(void *ctx, const u8 *pay,
						     u32 plen),
				      void *ctx)
{
	struct usf_session *s;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return ERR_PTR(-ENOMEM);

	s->dev = dev;
	s->aoc_dev = aoc_dev;
	s->sample = sample;
	s->sample_ctx = ctx;
	s->txn = 1;
	s->resp_txn = U32_MAX;
	s->resp_ready = U32_MAX;
	mutex_init(&s->ctl_lock);
	spin_lock_init(&s->resp_lock);
	init_completion(&s->resp_done);

	return s;
}
EXPORT_SYMBOL_GPL(usf_session_alloc);

int usf_session_open(struct usf_session *s)
{
	int ret;

	if (s->wake)
		return 0;

	s->wake = aocc_kernel_open_channel(s->aoc_dev, USF_WAKE_SERVICE,
					   usf_session_rx, s);
	if (IS_ERR(s->wake)) {
		ret = PTR_ERR(s->wake);
		s->wake = NULL;
		return ret;
	}

	s->nonwake = aocc_kernel_open_channel(s->aoc_dev, USF_NONWAKE_SERVICE,
					      usf_session_rx, s);
	if (IS_ERR(s->nonwake)) {
		ret = PTR_ERR(s->nonwake);
		s->nonwake = NULL;
		aocc_kernel_close_channel(s->wake);
		s->wake = NULL;
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(usf_session_open);

void usf_session_close(struct usf_session *s)
{
	if (s->nonwake) {
		aocc_kernel_close_channel(s->nonwake);
		s->nonwake = NULL;
	}
	if (s->wake) {
		aocc_kernel_close_channel(s->wake);
		s->wake = NULL;
	}
	s->sensor_mgr = 0;
	s->sample_chan = 0;
}
EXPORT_SYMBOL_GPL(usf_session_close);

int usf_session_bootstrap(struct usf_session *s)
{
	u32 h_reg, h_sensor, h_sample;

	if (!s->wake)
		return -ENODEV;

	h_reg = usf_get_server(s, usf_uuid_registry, "Registry");
	h_sensor = usf_get_server(s, usf_uuid_sensor_mgr, "SensorMgr");
	h_sample = usf_get_server(s, usf_uuid_sample_channel, "SampleChannel");

	/* Dormant servers mean the registry has not been loaded yet. */
	if (!h_sensor || !h_sample)
		return -EAGAIN;

	s->sensor_mgr = h_sensor;
	s->sample_chan = h_sample;
	dev_info(s->dev,
		 "USF bootstrap OK (registry=%u sensor_mgr=%u sample_chan=%u)\n",
		 h_reg, h_sensor, h_sample);
	return 0;
}
EXPORT_SYMBOL_GPL(usf_session_bootstrap);

int usf_session_sensor_list(struct usf_session *s, u32 *handles, int max)
{
	const u8 *req, *resp, *body, *vec;
	size_t reqlen;
	u32 rlen, blen, txn, n;
	int ret, tries, i, count;

	for (tries = 0; tries < USF_SETTLE_TRIES; tries++) {
		count = 0;
		mutex_lock(&s->ctl_lock);
		txn = s->txn++;
		ret = usf_build_no_body(&s->fbb, USF_MSG_GET_SENSOR_LIST, txn,
					s->sensor_mgr, &req, &reqlen);
		if (!ret)
			ret = usf_ctl(s, txn, req, reqlen, &resp, &rlen);
		if (!ret) {
			body = usf_resp_body(resp, rlen, &blen);
			vec = body ? usf_fb_vec(body, blen, 0, &n) : NULL;
			if (vec) {
				count = min_t(u32, n, (u32)max);
				for (i = 0; i < count; i++)
					handles[i] = get_unaligned_le32(vec + i * 4);
			}
		}
		mutex_unlock(&s->ctl_lock);

		if (ret)
			return ret;
		if (count > 0)
			return count;
		/* Registry loaded, but AoC's async chip probe hasn't settled. */
		msleep(USF_SETTLE_MS);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usf_session_sensor_list);

int usf_session_sensor_info(struct usf_session *s, u32 handle, char *name,
			    size_t namesz, u32 *res_bits)
{
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn;
	int ret;

	if (res_bits)
		*res_bits = 0;
	mutex_lock(&s->ctl_lock);
	txn = s->txn++;
	ret = usf_build_no_body(&s->fbb, USF_MSG_GET_SENSOR_INFO, txn, handle,
				&req, &reqlen);
	if (!ret)
		ret = usf_ctl(s, txn, req, reqlen, &resp, &rlen);
	if (!ret) {
		body = usf_resp_body(resp, rlen, &blen);
		if (!body || usf_fb_string(body, blen, 0, name, namesz) < 0) {
			name[0] = '\0';
			ret = -ENODATA;
		} else if (res_bits) {
			*res_bits = usf_fb_u32(body, blen, 9, 0);
		}
	}
	mutex_unlock(&s->ctl_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(usf_session_sensor_info);

int usf_session_start_sampling(struct usf_session *s, u32 handle,
			       enum usf_sampling_mode mode, s64 period_ns,
			       u32 client_id, u32 *sampling_id)
{
	const u8 *req, *resp, *body;
	size_t reqlen;
	u32 rlen, blen, txn, sid = 0;
	int ret;

	mutex_lock(&s->ctl_lock);
	txn = s->txn++;
	ret = usf_build_create_sampling(&s->fbb, txn, handle, mode, period_ns,
					client_id, &req, &reqlen);
	if (!ret)
		ret = usf_ctl(s, txn, req, reqlen, &resp, &rlen);
	if (!ret) {
		body = usf_resp_body(resp, rlen, &blen);
		if (body)
			sid = usf_fb_u32(body, blen, 0, 0);
	}
	mutex_unlock(&s->ctl_lock);

	if (ret) {
		dev_err(s->dev, "CreateSampling(handle=0x%x) failed: %d\n",
			handle, ret);
		return ret;
	}
	/*
	 * The firmware answers a rejected request (an unsupported mode, say)
	 * with sampling_id 0 rather than an error, and nothing would ever
	 * arrive on a stream that was never created.
	 */
	if (!sid) {
		dev_err(s->dev,
			"CreateSampling(handle=0x%x mode=%d) refused by firmware\n",
			handle, mode);
		return -EINVAL;
	}

	*sampling_id = sid;
	return 0;
}
EXPORT_SYMBOL_GPL(usf_session_start_sampling);

int usf_session_reconfig(struct usf_session *s, u32 handle, u32 sampling_id,
			 s64 period_ns, s64 max_latency_ns, bool enable)
{
	const u8 *req;
	size_t reqlen;
	u32 txn;
	int ret;

	mutex_lock(&s->ctl_lock);
	txn = s->txn++;
	ret = usf_build_reconfig(&s->fbb, txn, handle, sampling_id, period_ns,
				 max_latency_ns, enable, &req, &reqlen);
	if (!ret)
		ret = aocc_kernel_write(s->wake, req, reqlen);
	mutex_unlock(&s->ctl_lock);

	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(usf_session_reconfig);

int usf_session_stop_sampling(struct usf_session *s, u32 handle,
			      u32 sampling_id)
{
	const u8 *req, *resp;
	size_t reqlen;
	u32 txn, rlen;
	int ret;

	if (!sampling_id)
		return 0;

	mutex_lock(&s->ctl_lock);
	txn = s->txn++;
	ret = usf_build_stop_sampling(&s->fbb, txn, handle, sampling_id,
				      &req, &reqlen);
	if (!ret)
		ret = usf_ctl(s, txn, req, reqlen, &resp, &rlen);
	mutex_unlock(&s->ctl_lock);

	if (ret)
		dev_warn(s->dev,
			 "StopSampling(handle=0x%x id=%u) unacknowledged (%d), session may leak\n",
			 handle, sampling_id, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(usf_session_stop_sampling);

MODULE_DESCRIPTION("AoC USF client session");
MODULE_LICENSE("GPL");
