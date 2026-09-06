// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * linux/drivers/media/platform/samsung/s5p-mfc/s5p_mfc_dec.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 * Kamil Debski, <k.debski@samsung.com>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_ctrl.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_dec.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_pm.h"

static const struct s5p_mfc_fmt formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_NV12MT_16X16,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6_BIT | MFC_V7_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12MT,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V5_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6PLUS_BITS | MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 1,
		.versions	= MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV21M,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6PLUS_BITS,
	},
	{
		.fourcc         = V4L2_PIX_FMT_YUV420M,
		.codec_mode     = S5P_MFC_CODEC_NONE,
		.type           = MFC_FMT_RAW,
		.num_planes     = 3,
		.versions       = MFC_V12_BIT,
	},
	{
		.fourcc         = V4L2_PIX_FMT_YVU420M,
		.codec_mode     = S5P_MFC_CODEC_NONE,
		.type           = MFC_FMT_RAW,
		.num_planes     = 3,
		.versions       = MFC_V12_BIT
	},
	{
		.fourcc		= V4L2_PIX_FMT_H264,
		.codec_mode	= S5P_MFC_CODEC_H264_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H264_MVC,
		.codec_mode	= S5P_MFC_CODEC_H264_MVC_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V6PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H263,
		.codec_mode	= S5P_MFC_CODEC_H263_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG1,
		.codec_mode	= S5P_MFC_CODEC_MPEG2_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG2,
		.codec_mode	= S5P_MFC_CODEC_MPEG2_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG4,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_XVID,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VC1_ANNEX_G,
		.codec_mode	= S5P_MFC_CODEC_VC1_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VC1_ANNEX_L,
		.codec_mode	= S5P_MFC_CODEC_VC1RCV_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP8,
		.codec_mode	= S5P_MFC_CODEC_VP8_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V6PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_HEVC,
		.codec_mode	= S5P_FIMV_CODEC_HEVC_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V10PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP9,
		.codec_mode	= S5P_FIMV_CODEC_VP9_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V10PLUS_BITS | MFC_V16_BIT,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* Find selected format description */
static const struct s5p_mfc_fmt *find_format(struct v4l2_format *f, unsigned int t)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    formats[i].type == t)
			return &formats[i];
	}
	return NULL;
}

static struct mfc_control controls[] = {
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H264 Display Delay",
		.minimum = 0,
		.maximum = 16383,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 16383,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Display Delay Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Mpeg4 Loop Filter Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Slice Interface Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Minimum number of cap bufs",
		.minimum = 1,
		.maximum = 32,
		.step = 1,
		.default_value = 1,
		.is_volatile = 1,
	},
};

#define NUM_CTRLS ARRAY_SIZE(controls)

bool s5p_mfc_dec_copy_not_coded(struct s5p_mfc_ctx *ctx)
{
	return IS_MFCV16_PLUS(ctx->dev) &&
	       (ctx->codec_mode == S5P_MFC_CODEC_VP9_DEC ||
		ctx->codec_mode == S5P_MFC_CODEC_MPEG4_DEC);
}

/* MMAP keeps each DPB's DMA mapping stable until the queue is freed. */
int s5p_mfc_dec_dpb_index(struct s5p_mfc_ctx *ctx)
{
	u32 queued = READ_ONCE(ctx->dec_dst_flag);
	u32 used = READ_ONCE(ctx->dec_dpb_used);
	u32 available = queued & ~used;

	if (available)
		return __ffs(available);
	/* A full reference set needs a display-only command to release DPBs. */
	if (queued && hweight32(used) == ctx->pb_count + MFC_MAX_EXTRA_DPB)
		return __ffs(queued);
	return -EAGAIN;
}

/* Check whether a context should be run on hardware */
static int s5p_mfc_ctx_ready(struct s5p_mfc_ctx *ctx)
{
	bool dpb_ready = IS_MFCV16_PLUS(ctx->dev) ?
		s5p_mfc_dec_dpb_index(ctx) >= 0 :
		ctx->dst_queue_cnt >= ctx->pb_count;

	/* Context is to parse header */
	if (ctx->src_queue_cnt >= 1 && ctx->state == MFCINST_GOT_INST)
		return 1;
	/* Context is to decode a frame */
	if (ctx->src_queue_cnt >= 1 &&
	    ctx->state == MFCINST_RUNNING &&
	    dpb_ready)
		return 1;
	/* Context is to return last frame */
	if (ctx->state == MFCINST_FINISHING &&
	    dpb_ready)
		return 1;
	/* Context is to set buffers */
	if (ctx->src_queue_cnt >= 1 &&
	    ctx->state == MFCINST_HEAD_PARSED &&
	    ctx->capture_state == QUEUE_BUFS_MMAPED)
		return 1;
	/* Resolution change */
	if ((ctx->state == MFCINST_RES_CHANGE_INIT ||
		ctx->state == MFCINST_RES_CHANGE_FLUSH) &&
		dpb_ready)
		return 1;
	if (ctx->state == MFCINST_RES_CHANGE_END &&
		ctx->src_queue_cnt >= 1)
		return 1;
	mfc_debug(2, "ctx is not ready\n");
	return 0;
}

static const struct s5p_mfc_codec_ops decoder_codec_ops = {
	.pre_seq_start		= NULL,
	.post_seq_start		= NULL,
	.pre_frame_start	= NULL,
	.post_frame_start	= NULL,
};

/* Query capabilities of the device */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);

	strscpy(cap->driver, S5P_MFC_NAME, sizeof(cap->driver));
	strscpy(cap->card, dev->vfd_dec->name, sizeof(cap->card));
	return 0;
}

/* Enumerate format */
static int vidioc_enum_fmt(struct file *file, struct v4l2_fmtdesc *f,
							bool out)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	int i, j = 0;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (out && formats[i].type != MFC_FMT_DEC)
			continue;
		else if (!out && formats[i].type != MFC_FMT_RAW)
			continue;
		else if ((dev->variant->version_bit & formats[i].versions) == 0)
			continue;

		if (j == f->index)
			break;
		++j;
	}
	if (i == ARRAY_SIZE(formats))
		return -EINVAL;
	f->pixelformat = formats[i].fourcc;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, false);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, true);
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct v4l2_format f = { .fmt.pix_mp.pixelformat = fsize->pixel_format };
	const struct s5p_mfc_fmt *fmt;

	if (!dev->variant->dec_frmsize)
		return -ENOTTY;
	if (fsize->index)
		return -EINVAL;

	fmt = find_format(&f, MFC_FMT_DEC);
	if (!fmt)
		fmt = find_format(&f, MFC_FMT_RAW);
	if (!fmt || (dev->variant->version_bit & fmt->versions) == 0)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = *dev->variant->dec_frmsize;
	return 0;
}

/* A coded size only bounds the placeholder; the parsed header replaces it. */
static const struct v4l2_frmsize_stepwise dec_frmsize_placeholder = {
	.min_width = 16, .max_width = 8192, .step_width = 1,
	.min_height = 16, .max_height = 8192, .step_height = 1,
};

static void s5p_mfc_dec_bound_coded_size(struct s5p_mfc_ctx *ctx,
					 u32 *width, u32 *height)
{
	const struct v4l2_frmsize_stepwise *fs = ctx->dev->variant->dec_frmsize;

	if (!fs)
		fs = &dec_frmsize_placeholder;
	if (!*width || !*height) {
		*width = ctx->img_width;
		*height = ctx->img_height;
	}
	v4l_bound_align_image(width, fs->min_width, fs->max_width, 0,
			      height, fs->min_height, fs->max_height, 0, 0);
}

static u32 s5p_mfc_dec_bound_src_size(struct s5p_mfc_ctx *ctx, u32 sizeimage)
{
	if (!sizeimage)
		return DEF_CPB_SIZE;
	return min_t(u32, sizeimage, ctx->dev->variant->buf_size->cpb);
}

static void s5p_mfc_dec_fill_output(struct v4l2_pix_format_mplane *pix_mp,
				    const struct s5p_mfc_fmt *fmt,
				    u32 width, u32 height, u32 sizeimage)
{
	pix_mp->pixelformat = fmt->fourcc;
	pix_mp->width = width;
	pix_mp->height = height;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->flags = 0;
	pix_mp->num_planes = 1;
	memset(pix_mp->plane_fmt, 0, sizeof(pix_mp->plane_fmt));
	pix_mp->plane_fmt[0].sizeimage = sizeimage;
}

/*
 * The frame buffer format is known once the header has been parsed; until
 * then the coded-size placeholder describes it, as the stateful decoder
 * interface requires.
 */
static void s5p_mfc_dec_fill_capture(struct s5p_mfc_ctx *ctx,
				     struct v4l2_pix_format_mplane *pix_mp,
				     const struct s5p_mfc_fmt *fmt)
{
	int luma_size = ctx->luma_size, chroma_size = ctx->chroma_size;

	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->flags = 0;
	pix_mp->colorspace = ctx->colorspace;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_mp->quantization = ctx->quantization;
	memset(pix_mp->plane_fmt, 0, sizeof(pix_mp->plane_fmt));

	/*
	 * Before the header only the placeholder is known; afterwards the
	 * parsed layout describes the current format, and on v16 any raw
	 * format, whose planes differ only by the firmware's minimum.
	 */
	if (ctx->state < MFCINST_HEAD_PARSED || ctx->state >= MFCINST_ABORT ||
	    (fmt != ctx->dst_fmt && !IS_MFCV16_PLUS(ctx->dev))) {
		v4l2_fill_pixfmt_mp(pix_mp, fmt->fourcc, ctx->img_width,
				    ctx->img_height);
		return;
	}
	if (fmt != ctx->dst_fmt)
		s5p_mfc_dpb_plane_sizes_v16(ctx, fmt, &luma_size, &chroma_size);

	/* Width and height are set to the dimensions
	   of the movie, the buffer is bigger and
	   further processing stages should crop to this
	   rectangle. */
	pix_mp->pixelformat = fmt->fourcc;
	pix_mp->width = ctx->buf_width;
	pix_mp->height = ctx->buf_height;
	pix_mp->num_planes = fmt->num_planes;
	pix_mp->plane_fmt[0].bytesperline = ctx->stride[0];
	if (fmt->num_planes == 1) {
		/* Luma, then chroma at bytesperline * height. */
		pix_mp->plane_fmt[0].sizeimage = luma_size + chroma_size;
		return;
	}
	pix_mp->plane_fmt[0].sizeimage = luma_size;
	pix_mp->plane_fmt[1].bytesperline = ctx->stride[1];
	pix_mp->plane_fmt[1].sizeimage = chroma_size;
	if (fmt->fourcc == V4L2_PIX_FMT_YUV420M ||
	    fmt->fourcc == V4L2_PIX_FMT_YVU420M) {
		pix_mp->plane_fmt[2].bytesperline = ctx->stride[2];
		pix_mp->plane_fmt[2].sizeimage = ctx->chroma_size_1;
	}
}

/* Get format */
static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;

	mfc_debug_enter();
	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		s5p_mfc_dec_fill_output(pix_mp, ctx->src_fmt, ctx->img_width,
					ctx->img_height, ctx->dec_src_buf_size);
		pix_mp->colorspace = ctx->colorspace;
		pix_mp->xfer_func = ctx->xfer_func;
		pix_mp->ycbcr_enc = ctx->ycbcr_enc;
		pix_mp->quantization = ctx->quantization;
		mfc_debug_leave();
		return 0;
	}
	/* Legacy clients read the CAPTURE format to wait for the header. */
	if ((ctx->state == MFCINST_GOT_INST ||
	     ctx->state == MFCINST_RES_CHANGE_END) &&
	    vb2_is_streaming(&ctx->vq_src))
		s5p_mfc_wait_for_done_ctx(ctx, S5P_MFC_R2H_CMD_SEQ_DONE_RET,
									0);
	s5p_mfc_dec_fill_capture(ctx, pix_mp, ctx->dst_fmt);
	mfc_debug_leave();
	return 0;
}

/* Try format */
static int vidioc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct s5p_mfc_fmt *fmt;
	u32 width, height;

	mfc_debug(2, "Type is %d\n", f->type);
	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmt = find_format(f, MFC_FMT_DEC);
		if (!fmt) {
			mfc_err("Unsupported format for source.\n");
			return -EINVAL;
		}
		if (fmt->codec_mode == S5P_FIMV_CODEC_NONE) {
			mfc_err("Unknown codec\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_err("Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
		width = pix_mp->width;
		height = pix_mp->height;
		s5p_mfc_dec_bound_coded_size(ctx, &width, &height);
		/* Colorimetry is the client's to set on OUTPUT. */
		s5p_mfc_dec_fill_output(pix_mp, fmt, width, height,
					s5p_mfc_dec_bound_src_size(ctx,
						pix_mp->plane_fmt[0].sizeimage));
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt = find_format(f, MFC_FMT_RAW);
		if (!fmt) {
			mfc_err("Unsupported format for destination.\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_err("Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
		s5p_mfc_dec_fill_capture(ctx, pix_mp, fmt);
	}

	return 0;
}

/* Set format */
static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int ret;

	mfc_debug_enter();
	ret = vidioc_try_fmt(file, priv, f);
	if (ret)
		goto out;
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (vb2_is_busy(&ctx->vq_dst)) {
			v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
			ret = -EBUSY;
			goto out;
		}
		/* dst_fmt is validated by call to vidioc_try_fmt */
		ctx->dst_fmt = find_format(f, MFC_FMT_RAW);
		/* Once the header is parsed the DPB layout follows the raw format. */
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			s5p_mfc_hw_call(dev->mfc_ops, dec_calc_dpb_size, ctx);
			/* TRY_FMT answered for the previous format's layout. */
			s5p_mfc_dec_fill_capture(ctx, pix_mp, ctx->dst_fmt);
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (vb2_is_busy(&ctx->vq_src) || vb2_is_busy(&ctx->vq_dst)) {
			v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
			ret = -EBUSY;
			goto out;
		}
		/* An instance left by freed buffers belongs to the old codec. */
		if (ctx->inst_no != MFC_NO_INSTANCE_SET) {
			s5p_mfc_clock_on(dev);
			s5p_mfc_close_mfc_inst(dev, ctx);
			s5p_mfc_clock_off(dev);
		}
		/* src_fmt is validated by call to vidioc_try_fmt */
		ctx->src_fmt = find_format(f, MFC_FMT_DEC);
		ctx->codec_mode = ctx->src_fmt->codec_mode;
		mfc_debug(2, "The codec number is: %d\n", ctx->codec_mode);
		ctx->img_width = pix_mp->width;
		ctx->img_height = pix_mp->height;
		ctx->dec_src_buf_size = pix_mp->plane_fmt[0].sizeimage;
		ctx->colorspace = pix_mp->colorspace;
		ctx->xfer_func = pix_mp->xfer_func;
		ctx->ycbcr_enc = pix_mp->ycbcr_enc;
		ctx->quantization = pix_mp->quantization;
		ctx->state = MFCINST_INIT;
	} else {
		mfc_err("Wrong type error for S_FMT : %d", f->type);
		ret = -EINVAL;
	}

out:
	mfc_debug_leave();
	return ret;
}

static int reqbufs_output(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx,
				struct v4l2_requestbuffers *reqbufs)
{
	int ret = 0;

	s5p_mfc_clock_on(dev);

	if (reqbufs->count == 0) {
		mfc_debug(2, "Freeing buffers\n");
		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret)
			goto out;
		ctx->src_bufs_cnt = 0;
		ctx->output_state = QUEUE_FREE;
		goto out;
	}

	mfc_debug(2, "Allocating %d buffers for OUTPUT queue\n",
			reqbufs->count);
	/* vb2 frees an earlier set itself; count its replacement afresh. */
	ctx->src_bufs_cnt = 0;
	ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
	if (ret)
		goto out;

	/* The first buffers bind the context to a firmware instance. */
	if (ctx->inst_no == MFC_NO_INSTANCE_SET) {
		if (ctx->state != MFCINST_INIT) {
			mfc_err("Reqbufs called in an invalid state\n");
			ret = -EINVAL;
		} else {
			ret = s5p_mfc_open_mfc_inst(dev, ctx);
		}
		if (ret) {
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_src, reqbufs);
			ctx->src_bufs_cnt = 0;
			goto out;
		}
	}

	ctx->output_state = QUEUE_BUFS_REQUESTED;
out:
	s5p_mfc_clock_off(dev);
	if (ret)
		mfc_err("Failed allocating buffers for OUTPUT queue\n");
	return ret;
}

static int reqbufs_capture(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx,
				struct v4l2_requestbuffers *reqbufs)
{
	int ret = 0;

	s5p_mfc_clock_on(dev);

	if (reqbufs->count == 0) {
		mfc_debug(2, "Freeing buffers\n");
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;
		s5p_mfc_hw_call(dev->mfc_ops, release_codec_buffers, ctx);
		ctx->dst_bufs_cnt = 0;
		ctx->capture_state = QUEUE_FREE;
	} else if (ctx->capture_state == QUEUE_FREE) {
		WARN_ON(ctx->dst_bufs_cnt != 0);
		mfc_debug(2, "Allocating %d buffers for CAPTURE queue\n",
				reqbufs->count);
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;

		ctx->capture_state = QUEUE_BUFS_REQUESTED;
		ctx->total_dpb_count = reqbufs->count;

		ret = s5p_mfc_hw_call(dev->mfc_ops, alloc_codec_buffers, ctx);
		if (ret) {
			mfc_err("Failed to allocate decoding buffers\n");
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_dst, reqbufs);
			ret = -ENOMEM;
			ctx->capture_state = QUEUE_FREE;
			goto out;
		}

		WARN_ON(ctx->dst_bufs_cnt != ctx->total_dpb_count);
		ctx->capture_state = QUEUE_BUFS_MMAPED;

		if (s5p_mfc_ctx_ready(ctx))
			set_work_bit_irqsave(ctx);
		s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
		s5p_mfc_wait_for_done_ctx(ctx, S5P_MFC_R2H_CMD_INIT_BUFFERS_RET,
					  0);
	} else {
		mfc_err("Buffers have already been requested\n");
		ret = -EINVAL;
	}
out:
	s5p_mfc_clock_off(dev);
	if (ret)
		mfc_err("Failed allocating buffers for CAPTURE queue\n");
	return ret;
}

/* Request buffers */
static int vidioc_reqbufs(struct file *file, void *priv,
					  struct v4l2_requestbuffers *reqbufs)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (reqbufs->memory != V4L2_MEMORY_MMAP) {
		mfc_debug(2, "Only V4L2_MEMORY_MMAP is supported\n");
		return -EINVAL;
	}

	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		return reqbufs_output(dev, ctx, reqbufs);
	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return reqbufs_capture(dev, ctx, reqbufs);
	} else {
		mfc_err("Invalid type requested\n");
		return -EINVAL;
	}
}

/* Query buffer */
static int vidioc_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret;
	int i;

	mfc_debug(2, "State: %d, buf->type: %d\n", ctx->state, buf->type);
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_src, buf);
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		/* Both queues share the mmap space; CAPTURE sits above. */
		if (!ret && buf->memory == V4L2_MEMORY_MMAP)
			for (i = 0; i < buf->length; i++)
				buf->m.planes[i].m.mem_offset += DST_QUEUE_OFF_BASE;
	} else {
		mfc_err("invalid buf type\n");
		ret = -EINVAL;
	}
	mfc_debug_leave();
	return ret;
}

/* Queue a buffer */
static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (ctx->state == MFCINST_ERROR) {
		mfc_err("Call on QBUF after unrecoverable error\n");
		return -EIO;
	}
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_qbuf(&ctx->vq_src, NULL, buf);
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_qbuf(&ctx->vq_dst, NULL, buf);
	return -EINVAL;
}

/* Dequeue a buffer */
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS
	};
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret;

	if (ctx->state == MFCINST_ERROR) {
		mfc_err_limited("Call on DQBUF after unrecoverable error\n");
		return -EIO;
	}

	switch (buf->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
		if (ret)
			return ret;

		if (ctx->state == MFCINST_FINISHED &&
		    (ctx->dst_bufs[buf->index].flags & MFC_BUF_FLAG_EOS))
			v4l2_event_queue_fh(&ctx->fh, &ev);
		return 0;
	default:
		return -EINVAL;
	}
}

/* Export DMA buffer */
static int vidioc_expbuf(struct file *file, void *priv,
	struct v4l2_exportbuffer *eb)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (eb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_expbuf(&ctx->vq_src, eb);
	if (eb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_expbuf(&ctx->vq_dst, eb);
	return -EINVAL;
}

/* Stream on */
static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret = -EINVAL;

	mfc_debug_enter();
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = vb2_streamon(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = vb2_streamon(&ctx->vq_dst, type);
	mfc_debug_leave();
	return ret;
}

/* Stream off, which equals to a pause */
static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_streamoff(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_streamoff(&ctx->vq_dst, type);
	return -EINVAL;
}

/* Set controls - v4l2 control framework */
static int s5p_mfc_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY:
	case V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY:
		ctx->display_delay = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE:
	case V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE:
		ctx->display_delay_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		ctx->loop_filter_mpeg4 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE:
		ctx->slice_interface = ctrl->val;
		break;
	default:
		mfc_err("Invalid control 0x%08x\n", ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_dec_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		/*
		 * A header can only be parsed from a queued buffer while the
		 * OUTPUT queue streams; legacy clients read this control to
		 * wait for it.
		 */
		if ((ctx->state == MFCINST_INIT ||
		     ctx->state == MFCINST_GOT_INST ||
		     ctx->state == MFCINST_RES_CHANGE_END) &&
		    vb2_is_streaming(&ctx->vq_src) && ctx->src_queue_cnt)
			s5p_mfc_wait_for_done_ctx(ctx,
					S5P_MFC_R2H_CMD_SEQ_DONE_RET, 0);
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT)
			ctrl->val = ctx->pb_count;
		else
			ctrl->val = ctrl->default_value;
		break;
	}
	return 0;
}


static const struct v4l2_ctrl_ops s5p_mfc_dec_ctrl_ops = {
	.s_ctrl = s5p_mfc_dec_s_ctrl,
	.g_volatile_ctrl = s5p_mfc_dec_g_v_ctrl,
};

/* Get compose information */
static int vidioc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = ctx->dev;
	u32 left, right, top, bottom;
	u32 width, height;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (ctx->state != MFCINST_HEAD_PARSED &&
	    ctx->state != MFCINST_RUNNING &&
	    ctx->state != MFCINST_FINISHING &&
	    ctx->state != MFCINST_FINISHED) {
		mfc_err("Can not get compose information\n");
		return -EINVAL;
	}
	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_H264 ||
	    (IS_MFCV16_PLUS(dev) && ctx->src_fmt->fourcc == V4L2_PIX_FMT_HEVC)) {
		left = s5p_mfc_hw_call(dev->mfc_ops, get_crop_info_h, ctx);
		right = left >> S5P_FIMV_SHARED_CROP_RIGHT_SHIFT;
		left = left & S5P_FIMV_SHARED_CROP_LEFT_MASK;
		top = s5p_mfc_hw_call(dev->mfc_ops, get_crop_info_v, ctx);
		bottom = top >> S5P_FIMV_SHARED_CROP_BOTTOM_SHIFT;
		top = top & S5P_FIMV_SHARED_CROP_TOP_MASK;
		width = ctx->img_width - left - right;
		height = ctx->img_height - top - bottom;
		mfc_debug(2, "Composing info [h264]: l=%d t=%d w=%d h=%d (r=%d b=%d fw=%d fh=%d\n",
			  left, top, s->r.width, s->r.height, right, bottom,
			  ctx->buf_width, ctx->buf_height);
	} else {
		left = 0;
		top = 0;
		width = ctx->img_width;
		height = ctx->img_height;
		mfc_debug(2, "Composing info: w=%d h=%d fw=%d fh=%d\n",
			  s->r.width, s->r.height, ctx->buf_width,
			  ctx->buf_height);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = left;
		s->r.top = top;
		s->r.width = width;
		s->r.height = height;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_try_decoder_cmd(struct file *file, void *priv,
				  struct v4l2_decoder_cmd *cmd)
{
	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		cmd->flags = 0;
		cmd->stop.pts = 0;
		return 0;
	case V4L2_DEC_CMD_START:
		cmd->flags = 0;
		cmd->start.speed = 0;
		cmd->start.format = V4L2_DEC_START_FMT_NONE;
		return 0;
	default:
		return -EINVAL;
	}
}

static int vidioc_decoder_cmd(struct file *file, void *priv,
			      struct v4l2_decoder_cmd *cmd)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *buf;
	unsigned long flags;
	int ret;

	ret = vidioc_try_decoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	/* A drain needs both queues; until then the command is a no-op. */
	if (!vb2_is_streaming(&ctx->vq_src) || !vb2_is_streaming(&ctx->vq_dst))
		return 0;

	spin_lock_irqsave(&dev->irqlock, flags);
	if (ctx->draining) {
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return -EBUSY;
	}
	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		ctx->draining = true;
		if (list_empty(&ctx->src_queue)) {
			mfc_debug(2, "EOS: empty src queue, entering finishing state\n");
			ctx->state = MFCINST_FINISHING;
			if (s5p_mfc_ctx_ready(ctx))
				set_work_bit_irqsave(ctx);
			spin_unlock_irqrestore(&dev->irqlock, flags);
			s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
		} else {
			mfc_debug(2, "EOS: marking last buffer of stream\n");
			buf = list_entry(ctx->src_queue.prev,
						struct s5p_mfc_buf, list);
			if (buf->flags & MFC_BUF_FLAG_USED)
				ctx->state = MFCINST_FINISHING;
			else
				buf->flags |= MFC_BUF_FLAG_EOS;
			spin_unlock_irqrestore(&dev->irqlock, flags);
		}
		break;
	case V4L2_DEC_CMD_START:
		if (ctx->state == MFCINST_FINISHED)
			ctx->state = MFCINST_RUNNING;
		spin_unlock_irqrestore(&dev->irqlock, flags);
		vb2_clear_last_buffer_dequeued(&ctx->vq_dst);
		if (s5p_mfc_ctx_ready(ctx))
			set_work_bit_irqsave(ctx);
		s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
		break;
	default:
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return -EINVAL;
	}
	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				const struct  v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}


/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops s5p_mfc_dec_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_enum_framesizes = vidioc_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_expbuf = vidioc_expbuf,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
	.vidioc_g_selection = vidioc_g_selection,
	.vidioc_decoder_cmd = vidioc_decoder_cmd,
	.vidioc_try_decoder_cmd = vidioc_try_decoder_cmd,
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int s5p_mfc_queue_setup(struct vb2_queue *vq,
			unsigned int *buf_count,
			unsigned int *plane_count, unsigned int psize[],
			struct device *alloc_devs[])
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	struct s5p_mfc_dev *dev = ctx->dev;

	/* Video output for decoding (source)
	 * this can be set after getting an instance */
	if (ctx->state != MFCINST_ERROR &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* A single plane is required for input */
		*plane_count = 1;
		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	/* Video capture for decoding (destination)
	 * this can be set after the header was parsed */
	} else if (ctx->state == MFCINST_HEAD_PARSED &&
		   vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		*plane_count = ctx->dst_fmt->num_planes;

		/* Setup buffer count */
		if (*buf_count < ctx->pb_count)
			*buf_count = ctx->pb_count;
		if (*buf_count > ctx->pb_count + MFC_MAX_EXTRA_DPB)
			*buf_count = ctx->pb_count + MFC_MAX_EXTRA_DPB;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	} else {
		mfc_err("State seems invalid. State = %d, vq->type = %d\n",
							ctx->state, vq->type);
		return -EINVAL;
	}
	mfc_debug(2, "Buffer count=%d, plane count=%d\n",
						*buf_count, *plane_count);
	if (ctx->state == MFCINST_HEAD_PARSED &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->dst_fmt->num_planes == 1)
			psize[0] = ctx->luma_size + ctx->chroma_size;
		else
			psize[0] = ctx->luma_size;
		psize[1] = ctx->chroma_size;
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M)
			psize[2] = ctx->chroma_size_1;
		if (IS_MFCV6_PLUS(dev))
			alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
		else
			alloc_devs[0] = ctx->dev->mem_dev[BANK_R_CTX];
		alloc_devs[1] = ctx->dev->mem_dev[BANK_L_CTX];
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M)
			alloc_devs[2] = ctx->dev->mem_dev[BANK_L_CTX];
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
		   ctx->state != MFCINST_ERROR) {
		psize[0] = ctx->dec_src_buf_size;
		alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
	} else {
		mfc_err("This video node is dedicated to decoding. Decoding not initialized\n");
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int i;

	for (i = 0; i < vb->num_planes; i++) {
		if (!s5p_mfc_dma_addr_valid(ctx->dev,
					  vb2_dma_contig_plane_dma_addr(vb, i))) {
			mfc_err("Plane buffer is below the firmware allocation\n");
			return -EINVAL;
		}
	}

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->capture_state == QUEUE_BUFS_MMAPED)
			return 0;
		for (i = 0; i < ctx->dst_fmt->num_planes; i++) {
			if (IS_ERR_OR_NULL(ERR_PTR(
					vb2_dma_contig_plane_dma_addr(vb, i)))) {
				mfc_err("Plane mem not allocated\n");
				return -EINVAL;
			}
		}
		if (ctx->dst_fmt->num_planes == 1 ?
		    vb2_plane_size(vb, 0) < ctx->luma_size + ctx->chroma_size :
		    (vb2_plane_size(vb, 0) < ctx->luma_size ||
		     vb2_plane_size(vb, 1) < ctx->chroma_size)) {
			mfc_err("Plane buffer (CAPTURE) is too small\n");
			return -EINVAL;
		}
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M) {
			if (vb2_plane_size(vb, 2) < ctx->chroma_size_1) {
				mfc_err("Plane buffer (CAPTURE) is too small\n");
				return -EINVAL;
			}
		}
		i = vb->index;
		ctx->dst_bufs[i].b = vbuf;
		s5p_mfc_raw_plane_addrs(ctx, ctx->dst_fmt, vb,
					&ctx->dst_bufs[i].cookie.raw.luma,
					&ctx->dst_bufs[i].cookie.raw.chroma,
					&ctx->dst_bufs[i].cookie.raw.chroma_1);
		ctx->dst_bufs_cnt++;
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (IS_ERR_OR_NULL(ERR_PTR(
					vb2_dma_contig_plane_dma_addr(vb, 0)))) {
			mfc_err("Plane memory not allocated\n");
			return -EINVAL;
		}
		if (vb2_plane_size(vb, 0) < ctx->dec_src_buf_size) {
			mfc_err("Plane buffer (OUTPUT) is too small\n");
			return -EINVAL;
		}

		i = vb->index;
		ctx->src_bufs[i].b = vbuf;
		ctx->src_bufs[i].cookie.stream =
					vb2_dma_contig_plane_dma_addr(vb, 0);
		ctx->src_bufs_cnt++;
	} else {
		mfc_err("s5p_mfc_buf_init: unknown queue type\n");
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct s5p_mfc_dev *dev = ctx->dev;

	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (ctx->state == MFCINST_FINISHING ||
		ctx->state == MFCINST_FINISHED)
		ctx->state = MFCINST_RUNNING;
	/* If context is ready then dev = work->data;schedule it to run */
	if (s5p_mfc_ctx_ready(ctx))
		set_work_bit_irqsave(ctx);
	s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
	return 0;
}

/* Called with irqlock held, before vb2 can release any DMA mappings. */
static void s5p_mfc_dec_stop_failed(struct s5p_mfc_dev *dev)
{
	if (READ_ONCE(dev->fw_failed))
		return;

	s5p_mfc_clock_on(dev);
	s5p_mfc_abort_firmware(dev);
	/* Drop the outstanding command's clock reference, if it timed out. */
	if (test_and_clear_bit(0, &dev->hw_lock))
		s5p_mfc_clock_off(dev);
	s5p_mfc_clock_off(dev);
}

static void s5p_mfc_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct s5p_mfc_dev *dev = ctx->dev;
	int aborted = 0;
	int ret;

	ctx->draining = false;
	spin_lock_irqsave(&dev->irqlock, flags);
	if (IS_MFCV16_PLUS(dev)) {
		clear_work_bit(ctx);
		/* Sequence parsing and resolution-change drains also access DMA. */
		if (ctx->state == MFCINST_RES_CHANGE_INIT ||
		    ctx->state == MFCINST_RES_CHANGE_FLUSH ||
		    (dev->curr_ctx == ctx->num && dev->hw_lock &&
		     ctx->state != MFCINST_RUNNING && ctx->state != MFCINST_FINISHING))
			s5p_mfc_dec_stop_failed(dev);
	}
	if ((ctx->state == MFCINST_FINISHING ||
		ctx->state ==  MFCINST_RUNNING) &&
		dev->curr_ctx == ctx->num && dev->hw_lock) {
		ctx->state = MFCINST_ABORT;
		spin_unlock_irqrestore(&dev->irqlock, flags);
		ret = s5p_mfc_wait_for_done_ctx(ctx,
					S5P_MFC_R2H_CMD_FRAME_DONE_RET, 0);
		aborted = 1;
		spin_lock_irqsave(&dev->irqlock, flags);
		if (IS_MFCV16_PLUS(dev)) {
			/* Frame completion can itself discover a resolution change. */
			if (ret || ctx->state == MFCINST_RES_CHANGE_INIT ||
			    ctx->state == MFCINST_RES_CHANGE_FLUSH ||
			    (dev->curr_ctx == ctx->num && dev->hw_lock))
				s5p_mfc_dec_stop_failed(dev);
			else if (ctx->state == MFCINST_ABORT)
				ctx->state = MFCINST_RUNNING;
			aborted = 0;
		}
	}
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		s5p_mfc_cleanup_queue(&ctx->dst_queue, &ctx->vq_dst);
		INIT_LIST_HEAD(&ctx->dst_queue);
		ctx->dst_queue_cnt = 0;
		ctx->dpb_flush_flag = 1;
		ctx->dec_dst_flag = 0;
		if (IS_MFCV6_PLUS(dev) &&
		    (ctx->state == MFCINST_RUNNING ||
		     (IS_MFCV16_PLUS(dev) &&
		      (ctx->state == MFCINST_FINISHING || ctx->state == MFCINST_FINISHED)))) {
			enum s5p_mfc_inst_state prev_state = ctx->state;

			ctx->state = MFCINST_FLUSH;
			s5p_mfc_clean_ctx_int_flags(ctx);
			set_work_bit_irqsave(ctx);
			s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
			spin_unlock_irqrestore(&dev->irqlock, flags);
			ret = s5p_mfc_wait_for_done_ctx(ctx,
				S5P_MFC_R2H_CMD_DPB_FLUSH_RET, 0);
			if (ret)
				mfc_err("Err flushing buffers\n");
			spin_lock_irqsave(&dev->irqlock, flags);
			if (IS_MFCV16_PLUS(dev)) {
				if (ret)
					s5p_mfc_dec_stop_failed(dev);
				else
					ctx->state = prev_state;
			}
		}
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		s5p_mfc_cleanup_queue(&ctx->src_queue, &ctx->vq_src);
		INIT_LIST_HEAD(&ctx->src_queue);
		ctx->src_queue_cnt = 0;
	}
	if (aborted)
		ctx->state = MFCINST_RUNNING;
	spin_unlock_irqrestore(&dev->irqlock, flags);
}


static void s5p_mfc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;
	struct s5p_mfc_buf *mfc_buf;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_buf = &ctx->src_bufs[vb->index];
		/* Unused, and no longer the end of the stream STOP marked it as. */
		mfc_buf->flags = 0;
		spin_lock_irqsave(&dev->irqlock, flags);
		list_add_tail(&mfc_buf->list, &ctx->src_queue);
		ctx->src_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_buf = &ctx->dst_bufs[vb->index];
		mfc_buf->flags = 0;
		/* Mark destination as available for use by MFC */
		spin_lock_irqsave(&dev->irqlock, flags);
		set_bit(vb->index, &ctx->dec_dst_flag);
		list_add_tail(&mfc_buf->list, &ctx->dst_queue);
		ctx->dst_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else {
		mfc_err("Unsupported buffer type (%d)\n", vq->type);
	}
	if (s5p_mfc_ctx_ready(ctx))
		set_work_bit_irqsave(ctx);
	s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
}

static const struct vb2_ops s5p_mfc_dec_qops = {
	.queue_setup		= s5p_mfc_queue_setup,
	.buf_init		= s5p_mfc_buf_init,
	.start_streaming	= s5p_mfc_start_streaming,
	.stop_streaming		= s5p_mfc_stop_streaming,
	.buf_queue		= s5p_mfc_buf_queue,
};

const struct s5p_mfc_codec_ops *get_dec_codec_ops(void)
{
	return &decoder_codec_ops;
}

const struct vb2_ops *get_dec_queue_ops(void)
{
	return &s5p_mfc_dec_qops;
}

const struct v4l2_ioctl_ops *get_dec_v4l2_ioctl_ops(void)
{
	return &s5p_mfc_dec_ioctl_ops;
}

#define IS_MFC51_PRIV(x) ((V4L2_CTRL_ID2WHICH(x) == V4L2_CTRL_CLASS_CODEC) \
						&& V4L2_CTRL_DRIVER_PRIV(x))

int s5p_mfc_dec_ctrls_setup(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_ctrl_config cfg;
	int i;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, NUM_CTRLS);
	if (ctx->ctrl_handler.error) {
		mfc_err("v4l2_ctrl_handler_init failed\n");
		return ctx->ctrl_handler.error;
	}

	for (i = 0; i < NUM_CTRLS; i++) {
		/* v16 post-filtering needs additional DPBs, not implemented yet. */
		if (IS_MFCV16_PLUS(ctx->dev) &&
		    controls[i].id == V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER)
			continue;
		if (IS_MFC51_PRIV(controls[i].id)) {
			memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
			cfg.ops = &s5p_mfc_dec_ctrl_ops;
			cfg.id = controls[i].id;
			cfg.min = controls[i].minimum;
			cfg.max = controls[i].maximum;
			cfg.def = controls[i].default_value;
			cfg.name = controls[i].name;
			cfg.type = controls[i].type;

			cfg.step = controls[i].step;
			cfg.menu_skip_mask = 0;

			ctx->ctrls[i] = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
					&cfg, NULL);
		} else {
			ctx->ctrls[i] = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					&s5p_mfc_dec_ctrl_ops,
					controls[i].id, controls[i].minimum,
					controls[i].maximum, controls[i].step,
					controls[i].default_value);
		}
		if (ctx->ctrl_handler.error) {
			mfc_err("Adding control (%d) failed\n", i);
			return ctx->ctrl_handler.error;
		}
		if (controls[i].is_volatile && ctx->ctrls[i])
			ctx->ctrls[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}
	return 0;
}

void s5p_mfc_dec_ctrls_delete(struct s5p_mfc_ctx *ctx)
{
	int i;

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	for (i = 0; i < NUM_CTRLS; i++)
		ctx->ctrls[i] = NULL;
}

void s5p_mfc_dec_init(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_format f;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	ctx->src_fmt = find_format(&f, MFC_FMT_DEC);
	if (IS_MFCV8_PLUS(ctx->dev))
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
	else if (IS_MFCV6_PLUS(ctx->dev))
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT_16X16;
	else
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT;
	ctx->dst_fmt = find_format(&f, MFC_FMT_RAW);
	mfc_debug(2, "Default src_fmt is %p, dest_fmt is %p\n",
			ctx->src_fmt, ctx->dst_fmt);
	/* A complete default format lets a client stream without S_FMT. */
	ctx->codec_mode = ctx->src_fmt->codec_mode;
	ctx->img_width = DEF_WIDTH;
	ctx->img_height = DEF_HEIGHT;
	ctx->dec_src_buf_size = DEF_CPB_SIZE;
	ctx->state = MFCINST_INIT;
}
