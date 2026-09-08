// SPDX-License-Identifier: GPL-2.0-or-later
/* Workload policy derived from the Samsung/Google MFC QoS driver. */

#include <linux/clk.h>
#include <linux/math64.h>
#include <linux/interconnect.h>

#include <dt-bindings/interconnect/google,zumapro.h>

#include "s5p_mfc_common.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_qos.h"

struct s5p_mfc_qos_step {
	u32 threshold;
	u32 rate;
	u32 firmware_us;
};

/* Weighted 16x16 macroblocks/s, Hz, and firmware overhead per frame. */
static const struct s5p_mfc_qos_step zumapro_dec_steps[] = {
	{       0, 100000000, 2058 },
	{  265147, 310000000, 1379 },
	{  551811, 310000000, 1046 },
	{ 1202239, 400000000,  732 },
	{ 1887335, 465000000,  655 },
	{ 2088750, 664000000,  479 },
	{ 3197754, 664000000,  445 },
};

static const struct s5p_mfc_qos_step zumapro_enc_steps[] = {
	{       0, 100000000, 2058 },
	{  265147, 310000000, 1379 },
	{  551811, 310000000, 1046 },
	{ 1202239, 400000000,  732 },
	{ 1887335, 465000000,  655 },
	{ 2088750, 465000000,  559 },
	{ 2625764, 465000000,  479 },
	{ 3185240, 664000000,  445 },
};

static const struct {
	u32 rate;
	u32 kbps;
} zumapro_bitrate_steps[] = {
	{ 100000000,  36864 },
	{ 310000000, 113049 },
	{ 400000000, 147456 },
	{ 465000000, 172032 },
	{ 664000000, 245760 },
};

static u32 s5p_mfc_qos_fps(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_qos *qos = &ctx->qos;
	u64 interval = U64_MAX;
	unsigned int i, j;

	if (ctx->type == MFCINST_ENCODER)
		return clamp_t(u64, DIV_ROUND_UP_ULL(ctx->enc_params.rc_framerate_num,
				max(ctx->enc_params.rc_framerate_denom, 1U)), 1, 480);

	/* Nearest distinct presentation timestamps also handle reordered input. */
	for (i = 0; i < qos->count; i++) {
		for (j = 0; j < i; j++) {
			u64 a = qos->timestamps[i], b = qos->timestamps[j];
			u64 delta = a > b ? a - b : b - a;

			if (delta)
				interval = min(interval, delta);
		}
	}
	/*
	 * Some clients use microsecond-spaced tracking IDs instead of PTS.
	 * Treat intervals beyond 480 fps as unknown, as with absent timestamps.
	 */
	if (interval == U64_MAX || interval < 2000000)
		return 60;
	/* Downstream's cadence buckets tolerate timestamp quantisation/jitter. */
	if (interval > 40000000)
		return 24;
	if (interval > 25000000)
		return 30;
	if (interval > 12500000)
		return 60;
	if (interval > 6940000)
		return 120;
	if (interval > 4860000)
		return 180;
	if (interval > 3125000)
		return 240;
	return 480;
}

static bool s5p_mfc_qos_high_perf(struct s5p_mfc_ctx *ctx)
{
	return ctx->codec_mode == S5P_MFC_CODEC_H264_DEC ||
	       ctx->codec_mode == S5P_MFC_CODEC_H264_MVC_DEC ||
	       ctx->codec_mode == S5P_MFC_CODEC_HEVC_DEC ||
	       ctx->codec_mode == S5P_MFC_CODEC_H264_ENC ||
	       ctx->codec_mode == S5P_MFC_CODEC_HEVC_ENC;
}

static u32 s5p_mfc_qos_weight(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_enc_params *p = &ctx->enc_params;
	const struct s5p_mfc_fmt *raw = ctx->type == MFCINST_DECODER ?
		ctx->dst_fmt : ctx->src_fmt;
	u32 weight = 1000;
	bool refs = false;

	if (!s5p_mfc_qos_high_perf(ctx) &&
	    ctx->codec_mode != S5P_MFC_CODEC_VP8_DEC &&
	    ctx->codec_mode != S5P_MFC_CODEC_VP9_DEC &&
	    ctx->codec_mode != S5P_MFC_CODEC_VP8_ENC &&
	    ctx->codec_mode != S5P_MFC_CODEC_VP9_ENC)
		weight = weight * 100 / 25;
	else if (raw->num_planes == 3)
		weight = weight * 100 / 80;

	if (ctx->type == MFCINST_DECODER) {
		if (READ_ONCE(ctx->qos.tiled))
			weight = weight * 100 / 75;
		if (READ_ONCE(ctx->qos.mbaff))
			weight = weight * 100 / 60;
	} else {
		switch (ctx->codec_mode) {
		case S5P_MFC_CODEC_H264_ENC:
			refs = p->codec.h264.num_ref_pic_4p >= 2;
			break;
		case S5P_MFC_CODEC_HEVC_ENC:
			refs = p->codec.hevc.num_refs_for_p >= 2;
			break;
		case S5P_MFC_CODEC_VP8_ENC:
		case S5P_MFC_CODEC_VP9_ENC:
			refs = p->codec.vpx.num_ref >= V4L2_CID_MPEG_VIDEO_VPX_2_REF_FRAME;
			break;
		}
		if (p->num_b_frame)
			weight = weight * 100 / 50;
		else if (refs)
			weight = weight * 100 / 60;
		else if (ctx->codec_mode == S5P_MFC_CODEC_HEVC_ENC &&
			 p->codec.hevc.general_pb_enable)
			weight = weight * 100 / 50;
	}
	return weight;
}

static u64 s5p_mfc_qos_mbs(struct s5p_mfc_ctx *ctx, u32 fps)
{
	u32 width, height;

	if (ctx->type == MFCINST_DECODER) {
		width = READ_ONCE(ctx->img_width);
		height = READ_ONCE(ctx->img_height);
	} else {
		width = ctx->enc_params.width;
		height = ctx->enc_params.height;
	}
	return (u64)DIV_ROUND_UP(width, 16) * DIV_ROUND_UP(height, 16) * fps;
}

static u64 s5p_mfc_qos_kbps(struct s5p_mfc_ctx *ctx, u32 fps)
{
	u64 bytes = 0, kbps;
	unsigned int i;

	if (ctx->type == MFCINST_ENCODER) {
		/* Fixed-QP output has no configured bitrate limit. */
		kbps = ctx->enc_params.rc_frame ? ctx->enc_params.rc_bitrate / 1024 : 0;
	} else {
		for (i = 0; i < ctx->qos.count; i++)
			bytes += ctx->qos.bytes[i];
		kbps = div_u64(bytes * 8 * fps, max(ctx->qos.count, 1U) * 1024);
	}
	return s5p_mfc_qos_high_perf(ctx) ? kbps : kbps * 3;
}

struct s5p_mfc_qos_bw {
	u32 peak;
	u32 read;
	u32 write;
};

/* KB/s per UHD frame/s, for uncompressed 8-bit 4:2:0 buffers. */
static const struct s5p_mfc_qos_bw zumapro_dec_bw[] = {
	{ 32605, 34381, 21263 }, /* H.264 */
	{ 29973, 28851, 17538 }, /* HEVC */
	{ 28672, 30468, 22324 }, /* VP8 */
	{ 18351, 18947, 16877 }, /* VP9 */
	{ 31540, 25368, 15770 }, /* MPEG-4 and other legacy codecs */
};

static const struct s5p_mfc_qos_bw zumapro_enc_bw[] = {
	{ 45456, 56112, 11170 }, /* H.264 */
	{ 46756, 52766,  9763 }, /* HEVC */
	{ 64000, 67318, 22518 }, /* VP8 */
	{ 72326, 59726, 16530 }, /* VP9 */
	{ 44647, 55324,  9531 }, /* MPEG-4 and H.263 */
};

static void s5p_mfc_qos_bw(struct s5p_mfc_ctx *ctx, u64 mbs,
			   u64 *avg, u64 *peak)
{
	const struct s5p_mfc_qos_bw *bw;
	unsigned int i;

	switch (ctx->codec_mode) {
	case S5P_MFC_CODEC_H264_DEC:
	case S5P_MFC_CODEC_H264_MVC_DEC:
	case S5P_MFC_CODEC_H264_ENC:
		i = 0;
		break;
	case S5P_MFC_CODEC_HEVC_DEC:
	case S5P_MFC_CODEC_HEVC_ENC:
		i = 1;
		break;
	case S5P_MFC_CODEC_VP8_DEC:
	case S5P_MFC_CODEC_VP8_ENC:
		i = 2;
		break;
	case S5P_MFC_CODEC_VP9_DEC:
	case S5P_MFC_CODEC_VP9_ENC:
		i = 3;
		break;
	default:
		i = 4;
		break;
	}
	bw = ctx->type == MFCINST_DECODER ? &zumapro_dec_bw[i] : &zumapro_enc_bw[i];
	mbs = min_t(u64, mbs, 32400 * 120);
	*avg += max_t(u64, div_u64(bw->read * mbs, 32400), 1) +
		max_t(u64, div_u64(bw->write * mbs, 32400), 1);
	*peak += max_t(u64, div_u64(bw->peak * mbs, 32400), 1);
}

static u32 s5p_mfc_qos_tag(u32 rate)
{
	switch (rate) {
	case 664000000:
		return ZUMAPRO_BTS_MFC_664;
	case 465000000:
		return ZUMAPRO_BTS_MFC_465;
	case 400000000:
		return ZUMAPRO_BTS_MFC_400;
	case 310000000:
		return ZUMAPRO_BTS_MFC_310;
	default:
		return 0;
	}
}

static int s5p_mfc_qos_set_rate(struct s5p_mfc_dev *dev, unsigned long rate)
{
	struct clk *clk = dev->pm.rate_clock;
	int ret;

	/* Refresh CCF before its same-rate shortcut, and verify firmware readback. */
	if (!clk_get_rate(clk))
		return -EIO;
	ret = clk_set_rate(clk, rate);
	if (!ret && clk_get_rate(clk) != rate)
		ret = -EIO;
	if (ret)
		dev_err(dev->pm.device, "cannot set MFC to %lu Hz: %d\n", rate, ret);
	return ret;
}

static int s5p_mfc_qos_set_bw(struct s5p_mfc_dev *dev, u32 avg, u32 peak, u32 tag)
{
	icc_set_tag(dev->pm.memory_path, tag);
	return icc_set_bw(dev->pm.memory_path, avg, peak);
}

static int s5p_mfc_qos_apply(struct s5p_mfc_dev *dev, u32 rate,
			    u32 avg, u32 peak, u32 tag)
{
	struct s5p_mfc_pm *pm = &dev->pm;
	u32 guard_avg = max(avg, pm->qos_avg_bw);
	u32 guard_peak = max(peak, pm->qos_peak_bw);
	u32 guard_tag = tag | pm->qos_tag;
	unsigned long old_rate = pm->qos_rate ?: 100000000;
	int ret, rollback_ret;

	if (rate == pm->qos_rate && avg == pm->qos_avg_bw &&
	    peak == pm->qos_peak_bw && tag == pm->qos_tag && !pm->qos_dirty)
		return 0;

	/* Cover both workloads while the clock moves, including failed retries. */
	if (pm->qos_dirty)
		guard_tag |= ZUMAPRO_BTS_MFC_664;
	ret = s5p_mfc_qos_set_bw(dev, guard_avg, guard_peak, guard_tag);
	if (ret)
		goto rollback;
	ret = s5p_mfc_qos_set_rate(dev, rate);
	if (ret)
		goto rollback;
	/* Only release bandwidth after the new clock has been confirmed. */
	ret = s5p_mfc_qos_set_bw(dev, avg, peak, tag);
	if (ret)
		goto rollback;

	pm->qos_rate = rate;
	pm->qos_avg_bw = avg;
	pm->qos_peak_bw = peak;
	pm->qos_tag = tag;
	pm->qos_dirty = false;
	return 0;

rollback:
	/* ICC rolls back bandwidth on error, but does not roll back its tag. */
	rollback_ret = s5p_mfc_qos_set_bw(dev, guard_avg, guard_peak, guard_tag);
	if (!rollback_ret)
		rollback_ret = s5p_mfc_qos_set_rate(dev, old_rate);
	if (!rollback_ret)
		rollback_ret = s5p_mfc_qos_set_bw(dev, pm->qos_avg_bw,
					       pm->qos_peak_bw, pm->qos_tag);
	if (rollback_ret)
		dev_err(pm->device, "cannot restore MFC workload vote: %d\n", rollback_ret);
	/* Retain only the last complete transaction as the resume/retry target. */
	pm->qos_dirty = true;
	return ret;
}

static int s5p_mfc_qos_update(struct s5p_mfc_dev *dev)
{
	const struct s5p_mfc_qos_step *steps;
	struct s5p_mfc_ctx *ctx;
	u64 mbs = 0, kbps = 0, load, avg = 0, peak = 0;
	u32 fps = 0, rate = 100000000;
	bool decoder = false, uhd60 = false;
	unsigned int count, i;

	lockdep_assert_held(&dev->mfc_mutex);
	list_for_each_entry(ctx, &dev->open_ctxs, node) {
		u32 ctx_fps;
		u64 ctx_mbs;

		if (!ctx->qos.active)
			continue;
		ctx_fps = s5p_mfc_qos_fps(ctx);
		ctx_mbs = s5p_mfc_qos_mbs(ctx, ctx_fps);
		s5p_mfc_qos_bw(ctx, ctx_mbs, &avg, &peak);
		mbs += div_u64(ctx_mbs * s5p_mfc_qos_weight(ctx), 1000);
		fps += ctx_fps;
		kbps += s5p_mfc_qos_kbps(ctx, ctx_fps);
		if (ctx->type == MFCINST_DECODER) {
			decoder = true;
			uhd60 |= ctx_mbs >= 32400 * 60;
		}
	}
	steps = decoder ? zumapro_dec_steps : zumapro_enc_steps;
	count = decoder ? ARRAY_SIZE(zumapro_dec_steps) : ARRAY_SIZE(zumapro_enc_steps);
	for (i = count; i-- > 0;) {
		u64 overhead = (u64)fps * (500 + steps[i].firmware_us);

		load = overhead >= 1000000 ? U64_MAX :
			div_u64(mbs * 1000000, 1000000 - overhead);
		if (load > steps[i].threshold || !i) {
			rate = steps[i].rate;
			break;
		}
	}
	for (i = 0; i < ARRAY_SIZE(zumapro_bitrate_steps); i++) {
		if (kbps <= zumapro_bitrate_steps[i].kbps ||
		    i == ARRAY_SIZE(zumapro_bitrate_steps) - 1) {
			rate = max(rate, zumapro_bitrate_steps[i].rate);
			break;
		}
	}
	/* Budget at least 664 MHz for uncompressed 4K60 decoding. */
	if (uhd60)
		rate = max(rate, 664000000U);
	if (rate != dev->pm.qos_rate || avg != dev->pm.qos_avg_bw ||
	    peak != dev->pm.qos_peak_bw || dev->pm.qos_dirty)
		dev_dbg(dev->pm.device,
			"MFC workload %llu MB/s, %u fps -> %u Hz, BW %llu/%llu KB/s\n",
			mbs, fps, rate, avg, peak);
	return s5p_mfc_qos_apply(dev, rate, min_t(u64, avg, U32_MAX),
				 min_t(u64, peak, U32_MAX), s5p_mfc_qos_tag(rate));
}

int s5p_mfc_qos_queue(struct vb2_buffer *vb)
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct s5p_mfc_qos *qos = &ctx->qos;
	bool active = qos->active;
	int ret;

	if (!ctx->dev->pm.rate_clock)
		return 0;
	if (V4L2_TYPE_IS_OUTPUT(vb->type) && ctx->type == MFCINST_DECODER &&
	    vb2_get_plane_payload(vb, 0)) {
		qos->timestamps[qos->next] = vb->timestamp;
		qos->bytes[qos->next] = vb2_get_plane_payload(vb, 0);
		qos->next = (qos->next + 1) % S5P_MFC_QOS_SAMPLES;
		qos->count = min(qos->count + 1, S5P_MFC_QOS_SAMPLES);
	}
	qos->active = true;
	ret = s5p_mfc_qos_update(ctx->dev);
	if (ret) {
		qos->active = active;
		vb2_queue_error(&ctx->vq_src);
		vb2_queue_error(&ctx->vq_dst);
	}
	return ret;
}

void s5p_mfc_qos_release(struct s5p_mfc_ctx *ctx)
{
	if (!ctx->dev || !ctx->dev->pm.rate_clock)
		return;
	ctx->qos.active = false;
	ctx->qos.count = 0;
	ctx->qos.next = 0;
	if (s5p_mfc_qos_update(ctx->dev))
		dev_warn(ctx->dev->pm.device, "cannot release MFC workload vote\n");
}

void s5p_mfc_qos_stop(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_queue *other = V4L2_TYPE_IS_OUTPUT(q->type) ? &ctx->vq_dst : &ctx->vq_src;

	if (!vb2_is_streaming(other))
		s5p_mfc_qos_release(ctx);
}

int s5p_mfc_qos_restore(struct s5p_mfc_dev *dev)
{
	if (!dev->pm.rate_clock)
		return 0;
	dev->pm.qos_dirty = true;
	return s5p_mfc_qos_apply(dev, dev->pm.qos_rate ?: 100000000,
				 dev->pm.qos_avg_bw, dev->pm.qos_peak_bw, dev->pm.qos_tag);
}

void s5p_mfc_qos_cleanup(struct s5p_mfc_dev *dev)
{
	if (!IS_ERR_OR_NULL(dev->pm.rate_clock) &&
	    (dev->pm.qos_rate || dev->pm.qos_dirty) &&
	    s5p_mfc_qos_apply(dev, 100000000, 0, 0, 0))
		dev_warn(dev->pm.device, "cannot release MFC workload vote\n");
}
