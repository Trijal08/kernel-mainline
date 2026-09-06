// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * linux/drivers/media/platform/samsung/s5p-mfc/s5p_mfc_enc.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Jeongtae Park	<jtp.park@samsung.com>
 * Kamil Debski		<k.debski@samsung.com>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rational.h>
#include <linux/sched.h>
#include <linux/videodev2.h>
#include <media/v4l2-event.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_ctrl.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_enc.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_pm.h"

#define DEF_SRC_FMT_ENC	V4L2_PIX_FMT_NV12M
#define DEF_DST_FMT_ENC	V4L2_PIX_FMT_H264

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
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
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
		.versions       = MFC_V12_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H264,
		.codec_mode	= S5P_MFC_CODEC_H264_ENC,
		.type		= MFC_FMT_ENC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG4,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_ENC,
		.type		= MFC_FMT_ENC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H263,
		.codec_mode	= S5P_MFC_CODEC_H263_ENC,
		.type		= MFC_FMT_ENC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS | MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP8,
		.codec_mode	= S5P_MFC_CODEC_VP8_ENC,
		.type		= MFC_FMT_ENC,
		.num_planes	= 1,
		.versions	= MFC_V7PLUS_BITS | MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_HEVC,
		.codec_mode	= S5P_FIMV_CODEC_HEVC_ENC,
		.type		= MFC_FMT_ENC,
		.num_planes	= 1,
		.versions	= MFC_V10PLUS_BITS | MFC_V16_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP9,
		.codec_mode	= S5P_MFC_CODEC_VP9_ENC,
		.type		= MFC_FMT_ENC,
		.num_planes	= 1,
		.versions	= MFC_V16_BIT,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)
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
		.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 12,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.maximum = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES,
		.default_value = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1900,
		.maximum = (1 << 30) - 1,
		.step = 1,
		.default_value = 1900,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_PADDING,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Padding Control Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_PADDING_YUV,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Padding Color YUV Value",
		.minimum = 0,
		.maximum = (1 << 25) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = (1 << 30) - 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_RC_REACTION_COEFF,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Rate Control Reaction Coeff.",
		.minimum = 1,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Force frame type",
		.minimum = V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_DISABLED,
		.maximum = V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_NOT_CODED,
		.default_value = V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_DISABLED,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.minimum = 0,
		.maximum = 0,
		.step = 0,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VBV_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MV_H_SEARCH_RANGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Horizontal MV Search Range",
		.minimum = 16,
		.maximum = 128,
		.step = 16,
		.default_value = 32,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MV_V_SEARCH_RANGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Vertical MV Search Range",
		.minimum = 16,
		.maximum = 128,
		.step = 16,
		.default_value = 32,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE,
		.maximum = V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		.default_value = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Frame Skip Enable",
		.minimum = V4L2_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE_DISABLED,
		.maximum = V4L2_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE_BUF_LIMIT,
		.menu_skip_mask = 0,
		.default_value = V4L2_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE_DISABLED,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_FRAME_SKIP_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.maximum = V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_BUF_LIMIT,
		.default_value = V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_DISABLED,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_RC_FIXED_TARGET_BIT,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Fixed Target Bit Enable",
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
		.step = 1,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_B_FRAMES,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.maximum = V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH,
		.default_value = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.menu_skip_mask = ~(
				(1 << V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
				(1 << V4L2_MPEG_VIDEO_H264_PROFILE_MAIN) |
				(1 << V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)
				),
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.maximum = V4L2_MPEG_VIDEO_H264_LEVEL_4_0,
		.default_value = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.maximum = V4L2_MPEG_VIDEO_MPEG4_LEVEL_5,
		.default_value = V4L2_MPEG_VIDEO_MPEG4_LEVEL_0,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED,
		.maximum = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY,
		.default_value = V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = -6,
		.maximum = 6,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = -6,
		.maximum = 6,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.maximum = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC,
		.default_value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_H264_NUM_REF_PIC_FOR_P,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "The Number of Ref. Pic for P",
		.minimum = 1,
		.maximum = 2,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 51,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H263_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H263 I-Frame QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H263_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H263 Minimum QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H263_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H263 Maximum QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 31,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H263 P frame QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H263_B_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H263 B frame QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "MPEG4 I-Frame QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "MPEG4 Minimum QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "MPEG4 Maximum QP value",
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 51,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "MPEG4 P frame QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_B_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "MPEG4 B frame QP value",
		.minimum = 1,
		.maximum = 31,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_DARK,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Dark Reg Adaptive RC",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_SMOOTH,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Smooth Reg Adaptive RC",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_STATIC,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Static Reg Adaptive RC",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_ACTIVITY,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Activity Reg Adaptive RC",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.maximum = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED,
		.default_value = V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_GOP_CLOSURE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.maximum = V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE,
		.default_value = V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MPEG4_QPEL,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_NUM_PARTITIONS,
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.maximum = V4L2_CID_MPEG_VIDEO_VPX_8_PARTITIONS,
		.default_value = V4L2_CID_MPEG_VIDEO_VPX_1_PARTITION,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_IMD_DISABLE_4X4,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_NUM_REF_FRAMES,
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.maximum = V4L2_CID_MPEG_VIDEO_VPX_2_REF_FRAME,
		.default_value = V4L2_CID_MPEG_VIDEO_VPX_1_REF_FRAME,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_FILTER_LEVEL,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 63,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_FILTER_SHARPNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 7,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_REF_PERIOD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_SEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_USE_PREV,
		.maximum = V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_USE_REF_PERIOD,
		.default_value = V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_USE_PREV,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 127,
		.step = 1,
		.default_value = 127,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 11,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 127,
		.step = 1,
		.default_value = 10,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 127,
		.step = 1,
		.default_value = 10,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VP8_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_VP8_PROFILE_0,
		.maximum = V4L2_MPEG_VIDEO_VP8_PROFILE_3,
		.default_value = V4L2_MPEG_VIDEO_VP8_PROFILE_0,
		.menu_skip_mask = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "HEVC I Frame QP Value",
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "HEVC P Frame QP Value",
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.maximum = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.maximum = V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_TIER,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_TIER_MAIN,
		.maximum = V4L2_MPEG_VIDEO_HEVC_TIER_HIGH,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_TIER_MAIN,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_FRAME_RATE_RESOLUTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_MAX_PARTITION_DEPTH,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_REF_NUMBER_FOR_PFRAMES,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 2,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_TYPE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_REFRESH_NONE,
		.maximum = V4L2_MPEG_VIDEO_HEVC_REFRESH_IDR,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_REFRESH_NONE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_CONST_INTRA_PRED,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LOSSLESS_CU,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_WAVEFRONT,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED,
		.maximum = V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_QP,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_TYPE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_B,
		.maximum = V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_P,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_B,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_LAYER,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 6,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L4_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L5_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L6_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 51,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L4_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L5_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L6_BR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = INT_MIN,
		.maximum = INT_MAX,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_GENERAL_PB,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_TEMPORAL_ID,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_STRONG_SMOOTHING,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_INTRA_PU_SPLIT,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_TMV_PREDICTION,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_WITHOUT_STARTCODE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = (1 << 16) - 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = -6,
		.maximum = 6,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = -6,
		.maximum = 6,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_SIZE_OF_LENGTH_FIELD,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_HEVC_SIZE_0,
		.maximum = V4L2_MPEG_VIDEO_HEVC_SIZE_4,
		.step = 1,
		.default_value = V4L2_MPEG_VIDEO_HEVC_SIZE_0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Minimum number of output bufs",
		.minimum = 1,
		.maximum = 32,
		.step = 1,
		.default_value = 1,
		.is_volatile = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VP9_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.maximum = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.default_value = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VP9_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.minimum = V4L2_MPEG_VIDEO_VP9_LEVEL_1_0,
		.maximum = V4L2_MPEG_VIDEO_VP9_LEVEL_6_2,
		.default_value = V4L2_MPEG_VIDEO_VP9_LEVEL_4_0,
	},
};

#define NUM_CTRLS ARRAY_SIZE(controls)
static const char * const *mfc51_get_menu(u32 id)
{
	static const char * const mfc51_video_frame_skip[] = {
		"Disabled",
		"Level Limit",
		"VBV/CPB Limit",
		NULL,
	};
	static const char * const mfc51_video_force_frame[] = {
		"Disabled",
		"I Frame",
		"Not Coded",
		NULL,
	};
	switch (id) {
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE:
		return mfc51_video_frame_skip;
	case V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE:
		return mfc51_video_force_frame;
	}
	return NULL;
}

static int s5p_mfc_ctx_ready(struct s5p_mfc_ctx *ctx)
{
	mfc_debug(2, "src=%d, dst=%d, state=%d\n",
		  ctx->src_queue_cnt, ctx->dst_queue_cnt, ctx->state);
	/* context is ready to make header */
	if (ctx->state == MFCINST_GOT_INST && ctx->dst_queue_cnt >= 1)
		return 1;
	/* context is ready to encode a frame */
	if ((ctx->state == MFCINST_RUNNING ||
		ctx->state == MFCINST_HEAD_PRODUCED) &&
		ctx->src_queue_cnt >= 1 && ctx->dst_queue_cnt >= 1)
		return 1;
	/* context is ready to encode remaining frames */
	if (ctx->state == MFCINST_FINISHING &&
		ctx->dst_queue_cnt >= 1)
		return 1;
	mfc_debug(2, "ctx is not ready\n");
	return 0;
}

static void cleanup_ref_queue(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_buf *mb_entry;

	/* move buffers in ref queue to src queue */
	while (!list_empty(&ctx->ref_queue)) {
		mb_entry = list_entry((&ctx->ref_queue)->next,
						struct s5p_mfc_buf, list);
		list_del(&mb_entry->list);
		ctx->ref_queue_cnt--;
		list_add_tail(&mb_entry->list, &ctx->src_queue);
		ctx->src_queue_cnt++;
	}
	mfc_debug(2, "enc src count: %d, enc ref count: %d\n",
		  ctx->src_queue_cnt, ctx->ref_queue_cnt);
	INIT_LIST_HEAD(&ctx->ref_queue);
	ctx->ref_queue_cnt = 0;
}

static int enc_pre_seq_start(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *dst_mb;
	unsigned long dst_addr;
	unsigned int dst_size;

	dst_mb = list_entry(ctx->dst_queue.next, struct s5p_mfc_buf, list);
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_mb->b->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst_mb->b->vb2_buf, 0);
	s5p_mfc_hw_call(dev->mfc_ops, set_enc_stream_buffer, ctx, dst_addr,
			dst_size);
	return 0;
}

static int enc_post_seq_start(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_enc_params *p = &ctx->enc_params;
	struct s5p_mfc_buf *dst_mb;

	if (p->seq_hdr_mode == V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE &&
	    !(IS_MFCV16_PLUS(dev) &&
	      (ctx->codec_mode == S5P_MFC_CODEC_VP8_ENC ||
	       ctx->codec_mode == S5P_MFC_CODEC_VP9_ENC ||
	       ctx->codec_mode == S5P_MFC_CODEC_H263_ENC))) {
		if (!list_empty(&ctx->dst_queue)) {
			dst_mb = list_entry(ctx->dst_queue.next,
					struct s5p_mfc_buf, list);
			list_del(&dst_mb->list);
			ctx->dst_queue_cnt--;
			vb2_set_plane_payload(&dst_mb->b->vb2_buf, 0,
				s5p_mfc_hw_call(dev->mfc_ops, get_enc_strm_size,
						dev));
			vb2_buffer_done(&dst_mb->b->vb2_buf,
					VB2_BUF_STATE_DONE);
		}
	}

	if (!IS_MFCV6_PLUS(dev)) {
		ctx->state = MFCINST_RUNNING;
		if (s5p_mfc_ctx_ready(ctx))
			set_work_bit_irqsave(ctx);
		s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
	} else {
		ctx->pb_count = s5p_mfc_hw_call(dev->mfc_ops, get_enc_dpb_count, dev);
		if (FW_HAS_E_MIN_SCRATCH_BUF(dev))
			ctx->scratch_buf_size = s5p_mfc_hw_call(dev->mfc_ops,
					get_e_min_scratch_buf_size, dev);
		ctx->state = MFCINST_HEAD_PRODUCED;
	}

	return 0;
}

static int enc_pre_frame_start(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *dst_mb;
	struct s5p_mfc_buf *src_mb;
	unsigned long src_y_addr, src_c_addr, src_c_1_addr, dst_addr;
	unsigned int dst_size;

	src_mb = list_entry(ctx->src_queue.next, struct s5p_mfc_buf, list);
	s5p_mfc_raw_plane_addrs(ctx, ctx->src_fmt, &src_mb->b->vb2_buf,
				&src_y_addr, &src_c_addr, &src_c_1_addr);
	s5p_mfc_hw_call(dev->mfc_ops, set_enc_frame_buffer, ctx,
					src_y_addr, src_c_addr, src_c_1_addr);

	dst_mb = list_entry(ctx->dst_queue.next, struct s5p_mfc_buf, list);
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_mb->b->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst_mb->b->vb2_buf, 0);
	s5p_mfc_hw_call(dev->mfc_ops, set_enc_stream_buffer, ctx, dst_addr,
			dst_size);

	return 0;
}

static void enc_copy_metadata(struct vb2_v4l2_buffer *dst,
			      const struct vb2_v4l2_buffer *src)
{
	u32 mask = V4L2_BUF_FLAG_TIMECODE | V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

	if (!dst)
		return;

	dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
	dst->vb2_buf.copied_timestamp = 1;
	dst->timecode = src->timecode;
	dst->field = src->field;
	dst->flags = (dst->flags & ~mask) | (src->flags & mask);
}

static int enc_post_frame_start(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *mb_entry;
	struct vb2_v4l2_buffer *dst = NULL;
	unsigned long enc_y_addr = 0, enc_c_addr = 0, enc_c_1_addr = 0;
	unsigned long mb_y_addr, mb_c_addr, mb_c_1_addr;
	int slice_type;
	unsigned int strm_size;
	bool src_ready;

	slice_type = s5p_mfc_hw_call(dev->mfc_ops, get_enc_slice_type, dev);
	strm_size = s5p_mfc_hw_call(dev->mfc_ops, get_enc_strm_size, dev);
	if (strm_size && !list_empty(&ctx->dst_queue))
		dst = list_first_entry(&ctx->dst_queue, struct s5p_mfc_buf, list)->b;
	mfc_debug(2, "Encoded slice type: %d\n", slice_type);
	mfc_debug(2, "Encoded stream size: %d\n", strm_size);
	mfc_debug(2, "Display order: %d\n",
		  mfc_read(dev, S5P_FIMV_ENC_SI_PIC_CNT));
	if (slice_type >= 0) {
		s5p_mfc_hw_call(dev->mfc_ops, get_enc_frame_buffer, ctx,
				&enc_y_addr, &enc_c_addr, &enc_c_1_addr);
		list_for_each_entry(mb_entry, &ctx->src_queue, list) {
			s5p_mfc_raw_plane_addrs(ctx, ctx->src_fmt,
						&mb_entry->b->vb2_buf, &mb_y_addr,
						&mb_c_addr, &mb_c_1_addr);
			if (enc_y_addr == mb_y_addr && enc_c_addr == mb_c_addr && enc_c_1_addr
					== mb_c_1_addr) {
				list_del(&mb_entry->list);
				ctx->src_queue_cnt--;
				enc_copy_metadata(dst, mb_entry->b);
				vb2_buffer_done(&mb_entry->b->vb2_buf,
						VB2_BUF_STATE_DONE);
				break;
			}
		}
		list_for_each_entry(mb_entry, &ctx->ref_queue, list) {
			s5p_mfc_raw_plane_addrs(ctx, ctx->src_fmt,
						&mb_entry->b->vb2_buf, &mb_y_addr,
						&mb_c_addr, &mb_c_1_addr);
			if (enc_y_addr == mb_y_addr && enc_c_addr == mb_c_addr && enc_c_1_addr
					== mb_c_1_addr) {
				list_del(&mb_entry->list);
				ctx->ref_queue_cnt--;
				enc_copy_metadata(dst, mb_entry->b);
				vb2_buffer_done(&mb_entry->b->vb2_buf,
						VB2_BUF_STATE_DONE);
				break;
			}
		}
	}
	if (ctx->src_queue_cnt > 0 && (ctx->state == MFCINST_RUNNING ||
				ctx->state == MFCINST_FINISHING)) {
		mb_entry = list_entry(ctx->src_queue.next, struct s5p_mfc_buf,
				list);
		if (mb_entry->flags & MFC_BUF_FLAG_USED) {
			list_del(&mb_entry->list);
			ctx->src_queue_cnt--;
			list_add_tail(&mb_entry->list, &ctx->ref_queue);
			ctx->ref_queue_cnt++;
		}
	}
	mfc_debug(2, "enc src count: %d, enc ref count: %d\n",
		  ctx->src_queue_cnt, ctx->ref_queue_cnt);
	if ((ctx->dst_queue_cnt > 0) && (strm_size > 0)) {
		mb_entry = list_entry(ctx->dst_queue.next, struct s5p_mfc_buf,
									list);
		list_del(&mb_entry->list);
		ctx->dst_queue_cnt--;
		switch (slice_type) {
		case S5P_FIMV_ENC_SI_SLICE_TYPE_I:
			mb_entry->b->flags |= V4L2_BUF_FLAG_KEYFRAME;
			break;
		case S5P_FIMV_ENC_SI_SLICE_TYPE_P:
			mb_entry->b->flags |= V4L2_BUF_FLAG_PFRAME;
			break;
		case S5P_FIMV_ENC_SI_SLICE_TYPE_B:
			mb_entry->b->flags |= V4L2_BUF_FLAG_BFRAME;
			break;
		}
		vb2_set_plane_payload(&mb_entry->b->vb2_buf, 0, strm_size);
		vb2_buffer_done(&mb_entry->b->vb2_buf, VB2_BUF_STATE_DONE);
	}

	src_ready = true;
	if (ctx->state == MFCINST_RUNNING && ctx->src_queue_cnt == 0)
		src_ready = false;
	/* v16 keeps sending LAST_FRAME until the firmware completes the sequence. */
	if (ctx->state == MFCINST_FINISHING && ctx->ref_queue_cnt == 0 &&
	    !IS_MFCV16_PLUS(dev))
		src_ready = false;
	if (!src_ready || ctx->dst_queue_cnt == 0)
		clear_work_bit_irqsave(ctx);

	return 0;
}

static const struct s5p_mfc_codec_ops encoder_codec_ops = {
	.pre_seq_start		= enc_pre_seq_start,
	.post_seq_start		= enc_post_seq_start,
	.pre_frame_start	= enc_pre_frame_start,
	.post_frame_start	= enc_post_frame_start,
};

/* Query capabilities of the device */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);

	strscpy(cap->driver, S5P_MFC_NAME, sizeof(cap->driver));
	strscpy(cap->card, dev->vfd_enc->name, sizeof(cap->card));
	return 0;
}

static int vidioc_enum_fmt(struct file *file, struct v4l2_fmtdesc *f,
							bool out)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	int i, j = 0;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (out && formats[i].type != MFC_FMT_RAW)
			continue;
		else if (!out && formats[i].type != MFC_FMT_ENC)
			continue;
		else if ((dev->variant->version_bit & formats[i].versions) == 0)
			continue;

		if (j == f->index) {
			f->pixelformat = formats[i].fourcc;
			return 0;
		}
		++j;
	}
	return -EINVAL;
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

/* The v16 firmware bounds some codecs more tightly than the block itself. */
static const struct v4l2_frmsize_stepwise *
s5p_mfc_enc_frmsize(struct s5p_mfc_dev *dev, u32 codec_mode)
{
	static const struct v4l2_frmsize_stepwise hevc = {
		.min_width = 64, .max_width = 8192, .step_width = 2,
		.min_height = 64, .max_height = 8192, .step_height = 2,
	};
	static const struct v4l2_frmsize_stepwise vp9 = {
		.min_width = 64, .max_width = 4096, .step_width = 2,
		.min_height = 64, .max_height = 8192, .step_height = 2,
	};
	static const struct v4l2_frmsize_stepwise mpeg4 = {
		.min_width = 32, .max_width = 2048, .step_width = 2,
		.min_height = 32, .max_height = 2048, .step_height = 2,
	};
	static const struct v4l2_frmsize_stepwise h263 = {
		.min_width = 32, .max_width = 2048, .step_width = 2,
		.min_height = 32, .max_height = 1152, .step_height = 2,
	};

	if (!IS_MFCV16_PLUS(dev))
		return dev->variant->enc_frmsize;

	switch (codec_mode) {
	case S5P_MFC_CODEC_HEVC_ENC:
		return &hevc;
	case S5P_MFC_CODEC_VP9_ENC:
		return &vp9;
	case S5P_MFC_CODEC_MPEG4_ENC:
		return &mpeg4;
	case S5P_MFC_CODEC_H263_ENC:
		return &h263;
	default:
		return dev->variant->enc_frmsize;
	}
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct v4l2_format f = { .fmt.pix_mp.pixelformat = fsize->pixel_format };
	const struct s5p_mfc_fmt *fmt;
	u32 codec_mode;

	if (fsize->index)
		return -EINVAL;

	fmt = find_format(&f, MFC_FMT_ENC);
	if (fmt) {
		codec_mode = fmt->codec_mode;
	} else {
		/* A raw format is bounded by the coded format it feeds. */
		fmt = find_format(&f, MFC_FMT_RAW);
		if (!fmt)
			return -EINVAL;
		codec_mode = ctx->dst_fmt->codec_mode;
	}
	if ((dev->variant->version_bit & fmt->versions) == 0)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = *s5p_mfc_enc_frmsize(dev, codec_mode);
	return 0;
}

static void s5p_mfc_enc_apply_layout(struct s5p_mfc_ctx *ctx,
				     const struct s5p_mfc_raw_layout *l)
{
	ctx->frame_width = l->frame_width;
	ctx->buf_width = l->buf_width;
	ctx->buf_height = l->buf_height;
	ctx->stride[0] = l->stride[0];
	ctx->stride[1] = l->stride[1];
	ctx->stride[2] = l->stride[2];
	ctx->luma_size = l->luma_size;
	ctx->chroma_size = l->chroma_size;
	ctx->chroma_size_1 = l->chroma_size_1;
}

static void s5p_mfc_enc_ctx_layout(const struct s5p_mfc_ctx *ctx,
				   struct s5p_mfc_raw_layout *l)
{
	l->frame_width = ctx->frame_width;
	l->buf_width = ctx->buf_width;
	l->buf_height = ctx->buf_height;
	l->stride[0] = ctx->stride[0];
	l->stride[1] = ctx->stride[1];
	l->stride[2] = ctx->stride[2];
	l->luma_size = ctx->luma_size;
	l->chroma_size = ctx->chroma_size;
	l->chroma_size_1 = ctx->chroma_size_1;
}

/* Room for a coded frame: never below the CPB default, at most a raw frame. */
static u32 s5p_mfc_enc_bound_dst_size(struct s5p_mfc_ctx *ctx,
				      const struct s5p_mfc_raw_layout *l,
				      u32 sizeimage)
{
	u32 raw = l->luma_size + l->chroma_size + l->chroma_size_1;
	u32 cap = max_t(u32, ctx->dev->variant->buf_size->cpb, raw);

	if (!sizeimage)
		sizeimage = raw / 2;
	return clamp_t(u32, sizeimage, DEF_CPB_SIZE, cap);
}

/* The coded format's range applies to the source it will encode. */
static void s5p_mfc_enc_bound_source(struct s5p_mfc_dev *dev, u32 codec_mode,
				     u32 *width, u32 *height)
{
	const struct v4l2_frmsize_stepwise *fs;

	fs = s5p_mfc_enc_frmsize(dev, codec_mode);
	v4l_bound_align_image(width, fs->min_width, fs->max_width, 1,
			      height, fs->min_height, fs->max_height, 1, 0);
}

/* The format describes the buffer; the visible frame is the CROP selection. */
static void s5p_mfc_enc_fill_output(struct v4l2_pix_format_mplane *pix_mp,
				    const struct s5p_mfc_fmt *fmt,
				    const struct s5p_mfc_raw_layout *l)
{
	pix_mp->pixelformat = fmt->fourcc;
	pix_mp->width = l->frame_width;
	pix_mp->height = l->buf_height;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->flags = 0;
	pix_mp->num_planes = fmt->num_planes;
	memset(pix_mp->plane_fmt, 0, sizeof(pix_mp->plane_fmt));
	pix_mp->plane_fmt[0].bytesperline = l->stride[0];
	if (fmt->num_planes == 1) {
		/* Luma, then chroma at bytesperline * height. */
		pix_mp->plane_fmt[0].sizeimage = l->luma_size + l->chroma_size;
		return;
	}
	pix_mp->plane_fmt[0].sizeimage = l->luma_size;
	pix_mp->plane_fmt[1].bytesperline = l->stride[1];
	pix_mp->plane_fmt[1].sizeimage = l->chroma_size;
	if (fmt->fourcc == V4L2_PIX_FMT_YUV420M ||
	    fmt->fourcc == V4L2_PIX_FMT_YVU420M) {
		pix_mp->plane_fmt[2].bytesperline = l->stride[2];
		pix_mp->plane_fmt[2].sizeimage = l->chroma_size_1;
	}
}

/* The coded stream: the visible frame in whole macroblocks, its colorimetry. */
static void s5p_mfc_enc_fill_capture(struct s5p_mfc_ctx *ctx,
				     struct v4l2_pix_format_mplane *pix_mp,
				     const struct s5p_mfc_fmt *fmt,
				     u32 width, u32 height, u32 sizeimage)
{
	pix_mp->pixelformat = fmt->fourcc;
	pix_mp->width = ALIGN(width, 16);
	pix_mp->height = ALIGN(height, 16);
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->flags = 0;
	pix_mp->colorspace = ctx->colorspace;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_mp->quantization = ctx->quantization;
	pix_mp->num_planes = fmt->num_planes;
	memset(pix_mp->plane_fmt, 0, sizeof(pix_mp->plane_fmt));
	pix_mp->plane_fmt[0].sizeimage = sizeimage;
}

static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_raw_layout l;

	mfc_debug(2, "f->type = %d ctx->state = %d\n", f->type, ctx->state);
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* This is run on output (encoder dest) */
		s5p_mfc_enc_fill_capture(ctx, pix_fmt_mp, ctx->dst_fmt,
					 ctx->img_width, ctx->img_height,
					 ctx->enc_dst_buf_size);
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* This is run on capture (encoder src) */
		s5p_mfc_enc_ctx_layout(ctx, &l);
		s5p_mfc_enc_fill_output(pix_fmt_mp, ctx->src_fmt, &l);
		pix_fmt_mp->colorspace = ctx->colorspace;
		pix_fmt_mp->xfer_func = ctx->xfer_func;
		pix_fmt_mp->ycbcr_enc = ctx->ycbcr_enc;
		pix_fmt_mp->quantization = ctx->quantization;
	} else {
		mfc_err("invalid buf type\n");
		return -EINVAL;
	}
	return 0;
}

static int vidioc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	const struct s5p_mfc_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct s5p_mfc_raw_layout l;
	u32 width, height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt = find_format(f, MFC_FMT_ENC);
		if (!fmt) {
			mfc_debug(2, "failed to try output format\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_debug(2, "Unsupported format by this MFC version.\n");
			return -EINVAL;
		}

		/* The coded stream carries a source this codec can take. */
		width = ctx->img_width;
		height = ctx->img_height;
		s5p_mfc_enc_bound_source(dev, fmt->codec_mode, &width, &height);
		s5p_mfc_hw_call(dev->mfc_ops, enc_calc_src_size, dev,
				ctx->src_fmt, width, height, &l);
		s5p_mfc_enc_fill_capture(ctx, pix_fmt_mp, fmt, width, height,
			s5p_mfc_enc_bound_dst_size(ctx, &l,
				pix_fmt_mp->plane_fmt[0].sizeimage));
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmt = find_format(f, MFC_FMT_RAW);
		if (!fmt) {
			mfc_debug(2, "failed to try output format\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_debug(2, "Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
		s5p_mfc_enc_bound_source(dev, ctx->dst_fmt->codec_mode,
					 &pix_fmt_mp->width, &pix_fmt_mp->height);
		s5p_mfc_hw_call(dev->mfc_ops, enc_calc_src_size, dev, fmt,
				pix_fmt_mp->width, pix_fmt_mp->height, &l);
		/* Colorimetry is the client's to set on OUTPUT. */
		s5p_mfc_enc_fill_output(pix_fmt_mp, fmt, &l);
	} else {
		mfc_err("invalid buf type\n");
		return -EINVAL;
	}
	return 0;
}

static void s5p_mfc_enc_adjust_framerate(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_enc_params *p = &ctx->enc_params;
	unsigned long num, denom;
	unsigned int max = ctx->codec_mode == S5P_MFC_CODEC_H263_ENC ? 255 : 65535;

	if (!p->rc_framerate_num || !p->rc_framerate_denom) {
		p->rc_framerate_num = 30;
		p->rc_framerate_denom = 1;
	}
	rational_best_approximation(p->rc_framerate_num, p->rc_framerate_denom,
				    max, 65535, &num, &denom);
	p->rc_framerate_num = max_t(unsigned long, num, 1);
	p->rc_framerate_denom = max_t(unsigned long, denom, 1);
}

static void s5p_mfc_enc_update_controls(struct s5p_mfc_ctx *ctx)
{
	static const u32 qp_ids[] = {
		V4L2_CID_MPEG_VIDEO_VPX_MIN_QP,
		V4L2_CID_MPEG_VIDEO_VPX_MAX_QP,
		V4L2_CID_MPEG_VIDEO_VPX_I_FRAME_QP,
		V4L2_CID_MPEG_VIDEO_VPX_P_FRAME_QP,
	};
	bool vp9 = ctx->codec_mode == S5P_MFC_CODEC_VP9_ENC;
	bool bframes = ctx->codec_mode == S5P_MFC_CODEC_H264_ENC ||
		       ctx->codec_mode == S5P_MFC_CODEC_HEVC_ENC ||
		       ctx->codec_mode == S5P_MFC_CODEC_MPEG4_ENC;
	struct v4l2_ctrl *ctrl;
	int i, max = vp9 ? 255 : 127;

	s5p_mfc_enc_adjust_framerate(ctx);
	for (i = 0; i < ARRAY_SIZE(qp_ids); i++) {
		ctrl = v4l2_ctrl_find(&ctx->ctrl_handler, qp_ids[i]);
		v4l2_ctrl_modify_range(ctrl, 0, max, 1,
				       i == 1 ? max : i == 0 ? 0 : 10);
	}
	ctrl = v4l2_ctrl_find(&ctx->ctrl_handler, V4L2_CID_MPEG_VIDEO_B_FRAMES);
	v4l2_ctrl_modify_range(ctrl, 0, bframes ? 2 : 0, 1, 0);
}

static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	u32 sizeimage_req = pix_fmt_mp->plane_fmt[0].sizeimage;
	u32 width = pix_fmt_mp->width, height = pix_fmt_mp->height;
	struct s5p_mfc_raw_layout l;
	int ret = 0;

	ret = vidioc_try_fmt(file, priv, f);
	if (ret)
		return ret;
	/* Both formats precede any buffer; the instance is sized from them. */
	if (vb2_is_busy(&ctx->vq_src) || vb2_is_busy(&ctx->vq_dst)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* An instance left by freed buffers belongs to the old codec. */
		if (ctx->inst_no != MFC_NO_INSTANCE_SET) {
			s5p_mfc_clock_on(dev);
			s5p_mfc_close_mfc_inst(dev, ctx);
			s5p_mfc_clock_off(dev);
		}
		/* dst_fmt is validated by call to vidioc_try_fmt */
		ctx->dst_fmt = find_format(f, MFC_FMT_ENC);
		ctx->state = MFCINST_INIT;
		ctx->codec_mode = ctx->dst_fmt->codec_mode;
		if (IS_MFCV16_PLUS(dev))
			s5p_mfc_enc_update_controls(ctx);
		/* The source frame, bounded to this codec's range as TRY_FMT did. */
		width = ctx->img_width;
		height = ctx->img_height;
		s5p_mfc_enc_bound_source(dev, ctx->dst_fmt->codec_mode,
					 &width, &height);
		ctx->img_width = width;
		ctx->img_height = height;
		s5p_mfc_hw_call(dev->mfc_ops, enc_calc_src_size, dev,
				ctx->src_fmt, ctx->img_width, ctx->img_height,
				&l);
		s5p_mfc_enc_apply_layout(ctx, &l);
		ctx->enc_dst_buf_size_req = sizeimage_req;
		ctx->enc_dst_buf_size =	pix_fmt_mp->plane_fmt[0].sizeimage;
		ctx->dst_bufs_cnt = 0;
		ctx->capture_state = QUEUE_FREE;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* src_fmt is validated by call to vidioc_try_fmt */
		ctx->src_fmt = find_format(f, MFC_FMT_RAW);
		/* The frame asked for is the visible one; TRY_FMT padded the buffer. */
		s5p_mfc_enc_bound_source(dev, ctx->dst_fmt->codec_mode,
					 &width, &height);
		ctx->img_width = width;
		ctx->img_height = height;
		mfc_debug(2, "codec number: %d\n", ctx->src_fmt->codec_mode);
		mfc_debug(2, "fmt - w: %d, h: %d, ctx - w: %d, h: %d\n",
			pix_fmt_mp->width, pix_fmt_mp->height,
			ctx->img_width, ctx->img_height);

		s5p_mfc_hw_call(dev->mfc_ops, enc_calc_src_size, dev,
				ctx->src_fmt, ctx->img_width, ctx->img_height,
				&l);
		s5p_mfc_enc_apply_layout(ctx, &l);
		/* A defaulted coded buffer follows the source frame. */
		ctx->enc_dst_buf_size = s5p_mfc_enc_bound_dst_size(ctx, &l,
						ctx->enc_dst_buf_size_req);
		ctx->colorspace = pix_fmt_mp->colorspace;
		ctx->xfer_func = pix_fmt_mp->xfer_func;
		ctx->ycbcr_enc = pix_fmt_mp->ycbcr_enc;
		ctx->quantization = pix_fmt_mp->quantization;

		ctx->src_bufs_cnt = 0;
		ctx->output_state = QUEUE_FREE;
	} else {
		mfc_err("invalid buf type\n");
		ret = -EINVAL;
	}
out:
	mfc_debug_leave();
	return ret;
}

static int vidioc_reqbufs(struct file *file, void *priv,
					  struct v4l2_requestbuffers *reqbufs)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret = 0;

	/* if memory is not mmp or userptr or dmabuf return error */
	if ((reqbufs->memory != V4L2_MEMORY_MMAP) &&
		(reqbufs->memory != V4L2_MEMORY_USERPTR) &&
		(reqbufs->memory != V4L2_MEMORY_DMABUF))
		return -EINVAL;
	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (reqbufs->count == 0) {
			mfc_debug(2, "Freeing buffers\n");
			ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
			s5p_mfc_hw_call(dev->mfc_ops, release_codec_buffers,
					ctx);
			ctx->capture_state = QUEUE_FREE;
			return ret;
		}
		ctx->dst_bufs_cnt = 0;
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret != 0) {
			mfc_err("error in vb2_reqbufs() for E(D)\n");
			return ret;
		}
		ctx->capture_state = QUEUE_BUFS_REQUESTED;

	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (reqbufs->count == 0) {
			mfc_debug(2, "Freeing buffers\n");
			ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
			s5p_mfc_hw_call(dev->mfc_ops, release_codec_buffers,
					ctx);
			ctx->output_state = QUEUE_FREE;
			return ret;
		}
		ctx->src_bufs_cnt = 0;

		if (IS_MFCV6_PLUS(dev) && (!IS_MFCV12(dev))) {
			/* Check for min encoder buffers */
			if (ctx->pb_count &&
				(reqbufs->count < ctx->pb_count)) {
				reqbufs->count = ctx->pb_count;
				mfc_debug(2, "Minimum %d output buffers needed\n",
						ctx->pb_count);
			}
		}

		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret != 0) {
			mfc_err("error in vb2_reqbufs() for E(S)\n");
			return ret;
		}
		ctx->output_state = QUEUE_BUFS_REQUESTED;
	} else {
		mfc_err("invalid buf type\n");
		return -EINVAL;
	}
	return ret;
}

static int vidioc_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret = 0;
	int i;

	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		if (ret != 0) {
			mfc_err("error in vb2_querybuf() for E(D)\n");
			return ret;
		}
		/* Both queues share the mmap space; CAPTURE sits above. */
		if (buf->memory == V4L2_MEMORY_MMAP)
			for (i = 0; i < buf->length; i++)
				buf->m.planes[i].m.mem_offset += DST_QUEUE_OFF_BASE;
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_src, buf);
		if (ret != 0) {
			mfc_err("error in vb2_querybuf() for E(S)\n");
			return ret;
		}
	} else {
		mfc_err("invalid buf type\n");
		return -EINVAL;
	}
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
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (ctx->state == MFCINST_FINISHING) {
			mfc_err("Call on QBUF after EOS command\n");
			return -EIO;
		}
		return vb2_qbuf(&ctx->vq_src, NULL, buf);
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return vb2_qbuf(&ctx->vq_dst, NULL, buf);
	}
	return -EINVAL;
}

/* Dequeue a buffer */
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret;

	if (ctx->state == MFCINST_ERROR) {
		mfc_err_limited("Call on DQBUF after unrecoverable error\n");
		return -EIO;
	}
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
	} else {
		ret = -EINVAL;
	}

	return ret;
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

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_streamon(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_streamon(&ctx->vq_dst, type);
	return -EINVAL;
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

static inline int h264_level(enum v4l2_mpeg_video_h264_level lvl)
{
	static unsigned int t[V4L2_MPEG_VIDEO_H264_LEVEL_4_0 + 1] = {
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_0   */ 10,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1B    */ 9,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_1   */ 11,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_2   */ 12,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_1_3   */ 13,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_2_0   */ 20,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_2_1   */ 21,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_2_2   */ 22,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_3_0   */ 30,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_3_1   */ 31,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_3_2   */ 32,
		/* V4L2_MPEG_VIDEO_H264_LEVEL_4_0   */ 40,
	};
	return t[lvl];
}

static inline int mpeg4_level(enum v4l2_mpeg_video_mpeg4_level lvl)
{
	static unsigned int t[V4L2_MPEG_VIDEO_MPEG4_LEVEL_5 + 1] = {
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_0    */ 0,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B   */ 9,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_1    */ 1,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_2    */ 2,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_3    */ 3,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_3B   */ 7,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_4    */ 4,
		/* V4L2_MPEG_VIDEO_MPEG4_LEVEL_5    */ 5,
	};
	return t[lvl];
}

static inline int hevc_level(enum v4l2_mpeg_video_hevc_level lvl)
{
	static unsigned int t[] = {
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_1    */ 10,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_2    */ 20,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1  */ 21,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_3    */ 30,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1  */ 31,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_4    */ 40,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1  */ 41,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_5    */ 50,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1  */ 51,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2  */ 52,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_6    */ 60,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1  */ 61,
		/* V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2  */ 62,
	};
	return t[lvl];
}

static inline int vui_sar_idc(enum v4l2_mpeg_video_h264_vui_sar_idc sar)
{
	static unsigned int t[V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED + 1] = {
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED     */ 0,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_1x1             */ 1,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_12x11           */ 2,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_10x11           */ 3,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_16x11           */ 4,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_40x33           */ 5,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_24x11           */ 6,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_20x11           */ 7,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_32x11           */ 8,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_80x33           */ 9,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_18x11           */ 10,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_15x11           */ 11,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_64x33           */ 12,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_160x99          */ 13,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_4x3             */ 14,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_3x2             */ 15,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_2x1             */ 16,
		/* V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED        */ 255,
	};
	return t[sar];
}

/*
 * Update range of all HEVC quantization parameter controls that depend on the
 * V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP controls.
 */
static void __enc_update_hevc_qp_ctrls_range(struct s5p_mfc_ctx *ctx,
					     int min, int max)
{
	static const int __hevc_qp_ctrls[] = {
		V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L4_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L5_QP,
		V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L6_QP,
	};
	struct v4l2_ctrl *ctrl = NULL;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(__hevc_qp_ctrls); i++) {
		for (j = 0; j < ARRAY_SIZE(ctx->ctrls); j++) {
			if (ctx->ctrls[j] &&
			    ctx->ctrls[j]->id == __hevc_qp_ctrls[i]) {
				ctrl = ctx->ctrls[j];
				break;
			}
		}
		if (WARN_ON(!ctrl))
			break;

		__v4l2_ctrl_modify_range(ctrl, min, max, ctrl->step, min);
	}
}

static int s5p_mfc_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_enc_params *p = &ctx->enc_params;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		p->gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		p->slice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		p->slice_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES:
		p->slice_bit = ctrl->val * 8;
		break;
	case V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB:
		p->intra_refresh_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_PADDING:
		p->pad = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_PADDING_YUV:
		p->pad_luma = (ctrl->val >> 16) & 0xff;
		p->pad_cb = (ctrl->val >> 8) & 0xff;
		p->pad_cr = (ctrl->val >> 0) & 0xff;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		p->rc_frame = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		p->rc_bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_RC_REACTION_COEFF:
		p->rc_reaction_coeff = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE:
		ctx->force_frame_type = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_frame_type =
			V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_I_FRAME;
		break;
	case V4L2_CID_MPEG_VIDEO_VBV_SIZE:
		p->vbv_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_H_SEARCH_RANGE:
		p->mv_h_range = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_V_SEARCH_RANGE:
		p->mv_v_range = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE:
		p->codec.h264.cpb_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		p->seq_hdr_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE:
	case V4L2_CID_MPEG_VIDEO_FRAME_SKIP_MODE:
		p->frame_skip_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_RC_FIXED_TARGET_BIT:
		p->fixed_target_bit = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		p->num_b_frame = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
			p->codec.h264.profile = IS_MFCV16_PLUS(dev) ?
				S5P_FIMV_ENC_PROFILE_H264_MAIN_V16 :
				S5P_FIMV_ENC_PROFILE_H264_MAIN;
			break;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
			p->codec.h264.profile = IS_MFCV16_PLUS(dev) ?
				S5P_FIMV_ENC_PROFILE_H264_HIGH_V16 :
				S5P_FIMV_ENC_PROFILE_H264_HIGH;
			break;
		case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
			p->codec.h264.profile = IS_MFCV16_PLUS(dev) ?
				S5P_FIMV_ENC_PROFILE_H264_BASELINE_V16 :
				S5P_FIMV_ENC_PROFILE_H264_BASELINE;
			break;
		case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
			if (IS_MFCV6_PLUS(dev))
				p->codec.h264.profile =
				S5P_FIMV_ENC_PROFILE_H264_CONSTRAINED_BASELINE;
			else
				ret = -EINVAL;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		p->codec.h264.level_v4l2 = ctrl->val;
		p->codec.h264.level = h264_level(ctrl->val);
		if (p->codec.h264.level < 0) {
			mfc_err("Level number is wrong\n");
			ret = p->codec.h264.level;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL:
		p->codec.mpeg4.level_v4l2 = ctrl->val;
		p->codec.mpeg4.level = mpeg4_level(ctrl->val);
		if (p->codec.mpeg4.level < 0) {
			mfc_err("Level number is wrong\n");
			ret = p->codec.mpeg4.level;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		p->codec.h264.loop_filter_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA:
		p->codec.h264.loop_filter_alpha = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA:
		p->codec.h264.loop_filter_beta = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		p->codec.h264.entropy_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_NUM_REF_PIC_FOR_P:
		p->codec.h264.num_ref_pic_4p = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM:
		p->codec.h264._8x8_transform = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		p->rc_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		p->codec.h264.rc_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		p->codec.h264.rc_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		p->codec.h264.rc_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		p->codec.h264.rc_p_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP:
		p->codec.h264.rc_b_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_H263_I_FRAME_QP:
		p->codec.mpeg4.rc_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_H263_MIN_QP:
		p->codec.mpeg4.rc_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_H263_MAX_QP:
		p->codec.mpeg4.rc_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP:
		p->codec.mpeg4.rc_p_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_B_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_H263_B_FRAME_QP:
		p->codec.mpeg4.rc_b_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_DARK:
		p->codec.h264.rc_mb_dark = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_SMOOTH:
		p->codec.h264.rc_mb_smooth = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_STATIC:
		p->codec.h264.rc_mb_static = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_ACTIVITY:
		p->codec.h264.rc_mb_activity = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE:
		p->codec.h264.vui_sar = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC:
		p->codec.h264.vui_sar_idc = vui_sar_idc(ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH:
		p->codec.h264.vui_ext_sar_width = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT:
		p->codec.h264.vui_ext_sar_height = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
		p->codec.h264.open_gop = !ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		p->codec.h264.open_gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE:
			p->codec.mpeg4.profile =
				S5P_FIMV_ENC_PROFILE_MPEG4_SIMPLE;
			break;
		case V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE:
			p->codec.mpeg4.profile =
			S5P_FIMV_ENC_PROFILE_MPEG4_ADVANCED_SIMPLE;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_QPEL:
		p->codec.mpeg4.quarter_pixel = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_NUM_PARTITIONS:
		p->codec.vpx.num_partitions = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_IMD_DISABLE_4X4:
		p->codec.vpx.imd_4x4 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_NUM_REF_FRAMES:
		p->codec.vpx.num_ref = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_FILTER_LEVEL:
		p->codec.vpx.filter_level = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_FILTER_SHARPNESS:
		p->codec.vpx.filter_sharpness = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_REF_PERIOD:
		p->codec.vpx.golden_frame_ref_period = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_SEL:
		p->codec.vpx.golden_frame_sel = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_MIN_QP:
		p->codec.vpx.rc_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_MAX_QP:
		p->codec.vpx.rc_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_I_FRAME_QP:
		p->codec.vpx.rc_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_P_FRAME_QP:
		p->codec.vpx.rc_p_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_PROFILE:
		p->codec.vpx.profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
		/* Only profile 0 is exposed. */
		break;
	case V4L2_CID_MPEG_VIDEO_VP9_LEVEL: {
		static const u8 levels[] = {
			10, 11, 20, 21, 30, 31, 40, 41, 50, 51, 52, 60, 61, 62,
		};

		p->codec.vpx.vp9_level = levels[ctrl->val];
		break;
	}
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		p->codec.hevc.rc_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		p->codec.hevc.rc_p_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP:
		p->codec.hevc.rc_b_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_FRAME_RATE_RESOLUTION:
		p->codec.hevc.rc_framerate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		p->codec.hevc.rc_min_qp = ctrl->val;
		__enc_update_hevc_qp_ctrls_range(ctx, ctrl->val,
						 p->codec.hevc.rc_max_qp);
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		p->codec.hevc.rc_max_qp = ctrl->val;
		__enc_update_hevc_qp_ctrls_range(ctx, p->codec.hevc.rc_min_qp,
						 ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		p->codec.hevc.level_v4l2 = ctrl->val;
		p->codec.hevc.level = hevc_level(ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
			p->codec.hevc.profile =
				V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
			break;
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
			p->codec.hevc.profile =
			V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_TIER:
		p->codec.hevc.tier = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_PARTITION_DEPTH:
		p->codec.hevc.max_partition_depth = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_REF_NUMBER_FOR_PFRAMES:
		p->codec.hevc.num_refs_for_p = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_TYPE:
		p->codec.hevc.refreshtype = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_CONST_INTRA_PRED:
		p->codec.hevc.const_intra_period_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LOSSLESS_CU:
		p->codec.hevc.lossless_cu_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_WAVEFRONT:
		p->codec.hevc.wavefront_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE:
		p->codec.hevc.loopfilter = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_QP:
		p->codec.hevc.hier_qp_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_TYPE:
		p->codec.hevc.hier_qp_type = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_LAYER:
		p->codec.hevc.num_hier_layer = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_QP:
		p->codec.hevc.hier_qp_layer[0] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_QP:
		p->codec.hevc.hier_qp_layer[1] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_QP:
		p->codec.hevc.hier_qp_layer[2] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_QP:
		p->codec.hevc.hier_qp_layer[3] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L4_QP:
		p->codec.hevc.hier_qp_layer[4] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L5_QP:
		p->codec.hevc.hier_qp_layer[5] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L6_QP:
		p->codec.hevc.hier_qp_layer[6] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_BR:
		p->codec.hevc.hier_bit_layer[0] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_BR:
		p->codec.hevc.hier_bit_layer[1] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_BR:
		p->codec.hevc.hier_bit_layer[2] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_BR:
		p->codec.hevc.hier_bit_layer[3] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L4_BR:
		p->codec.hevc.hier_bit_layer[4] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L5_BR:
		p->codec.hevc.hier_bit_layer[5] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L6_BR:
		p->codec.hevc.hier_bit_layer[6] = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_GENERAL_PB:
		p->codec.hevc.general_pb_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_TEMPORAL_ID:
		p->codec.hevc.temporal_id_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_STRONG_SMOOTHING:
		p->codec.hevc.strong_intra_smooth = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_INTRA_PU_SPLIT:
		p->codec.hevc.intra_pu_split_disable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_TMV_PREDICTION:
		p->codec.hevc.tmv_prediction_disable = !ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1:
		p->codec.hevc.max_num_merge_mv = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_WITHOUT_STARTCODE:
		p->codec.hevc.encoding_nostartcode_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD:
		p->codec.hevc.refreshperiod = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2:
		p->codec.hevc.lf_beta_offset_div2 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2:
		p->codec.hevc.lf_tc_offset_div2 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_SIZE_OF_LENGTH_FIELD:
		p->codec.hevc.size_of_length_field = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR:
		p->codec.hevc.prepend_sps_pps_to_idr = ctrl->val;
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid control, id=%d, val=%d\n",
							ctrl->id, ctrl->val);
		ret = -EINVAL;
	}
	return ret;
}

static int s5p_mfc_enc_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);
	struct s5p_mfc_dev *dev = ctx->dev;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_OUTPUT:
		/* Wait only for a header the hardware is already producing. */
		if (ctx->state == MFCINST_GOT_INST &&
		    dev->curr_ctx == ctx->num && dev->hw_lock)
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

static const struct v4l2_ctrl_ops s5p_mfc_enc_ctrl_ops = {
	.s_ctrl = s5p_mfc_enc_s_ctrl,
	.g_volatile_ctrl = s5p_mfc_enc_g_v_ctrl,
};

static int vidioc_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *a)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	memset(&a->parm, 0, sizeof(a->parm));
	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe.denominator =
				ctx->enc_params.rc_framerate_num;
	a->parm.output.timeperframe.numerator =
				ctx->enc_params.rc_framerate_denom;
	return 0;
}

static int vidioc_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *a)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct v4l2_fract *tpf = &a->parm.output.timeperframe;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_err("Setting FPS is only possible for the output queue\n");
		return -EINVAL;
	}

	ctx->enc_params.rc_framerate_num = tpf->denominator;
	ctx->enc_params.rc_framerate_denom = tpf->numerator;
	s5p_mfc_enc_adjust_framerate(ctx);
	return vidioc_g_parm(file, priv, a);
}

static int vidioc_try_encoder_cmd(struct file *file, void *priv,
				  struct v4l2_encoder_cmd *cmd)
{
	if (cmd->cmd != V4L2_ENC_CMD_STOP && cmd->cmd != V4L2_ENC_CMD_START)
		return -EINVAL;
	cmd->flags = 0;
	return 0;
}

static int vidioc_encoder_cmd(struct file *file, void *priv,
			      struct v4l2_encoder_cmd *cmd)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *buf;
	unsigned long flags;
	int ret;

	ret = vidioc_try_encoder_cmd(file, priv, cmd);
	if (ret)
		return ret;

	/* A drain needs both queues; until then the command is a no-op. */
	if (!vb2_is_streaming(&ctx->vq_src) || !vb2_is_streaming(&ctx->vq_dst))
		return 0;

	spin_lock_irqsave(&dev->irqlock, flags);
	/* The drain owns the context until its terminal buffer is out. */
	if (ctx->draining) {
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return -EBUSY;
	}
	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
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
	case V4L2_ENC_CMD_START:
		if (ctx->state == MFCINST_FINISHED)
			ctx->state = ctx->enc_seq_complete ?
				     MFCINST_GOT_INST : MFCINST_RUNNING;
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
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

/* The visible frame inside a padded source buffer. */
static int vidioc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	s->r.left = 0;
	s->r.top = 0;
	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.width = ctx->frame_width;
		s->r.height = ctx->buf_height;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		s->r.width = ctx->img_width;
		s->r.height = ctx->img_height;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_s_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = ctx->dev;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	/* The coded size is fixed with the source buffers. */
	if (vb2_is_busy(&ctx->vq_src) || vb2_is_busy(&ctx->vq_dst))
		return -EBUSY;
	/* The firmware takes its source from the buffer's origin. */
	s->r.left = 0;
	s->r.top = 0;
	s->r.width = clamp_t(u32, s->r.width, 1, ctx->frame_width);
	s->r.height = clamp_t(u32, s->r.height, 1, ctx->buf_height);
	s5p_mfc_enc_bound_source(dev, ctx->dst_fmt->codec_mode,
				 &s->r.width, &s->r.height);
	/* Before v7 the firmware derives the source pitch from the width. */
	if (!IS_MFCV7_PLUS(dev))
		s->r.width = ctx->frame_width;
	ctx->img_width = s->r.width;
	ctx->img_height = s->r.height;
	return 0;
}

static const struct v4l2_ioctl_ops s5p_mfc_enc_ioctl_ops = {
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
	.vidioc_g_selection = vidioc_g_selection,
	.vidioc_s_selection = vidioc_s_selection,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_expbuf = vidioc_expbuf,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
	.vidioc_s_parm = vidioc_s_parm,
	.vidioc_g_parm = vidioc_g_parm,
	.vidioc_encoder_cmd = vidioc_encoder_cmd,
	.vidioc_try_encoder_cmd = vidioc_try_encoder_cmd,
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int check_vb_with_fmt(const struct s5p_mfc_fmt *fmt, struct vb2_buffer *vb)
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int i;

	if (!fmt)
		return -EINVAL;
	if (fmt->num_planes != vb->num_planes) {
		mfc_err("invalid plane number for the format\n");
		return -EINVAL;
	}
	for (i = 0; i < fmt->num_planes; i++) {
		dma_addr_t dma = vb2_dma_contig_plane_dma_addr(vb, i);
		if (!dma) {
			mfc_err("failed to get plane cookie\n");
			return -EINVAL;
		}
		if (!s5p_mfc_dma_addr_valid(ctx->dev, dma)) {
			mfc_err("Plane buffer is below the firmware allocation\n");
			return -EINVAL;
		}
		mfc_debug(2, "index: %d, plane[%d] cookie: %pad\n",
			  vb->index, i, &dma);
	}
	return 0;
}

static int s5p_mfc_queue_setup(struct vb2_queue *vq,
			unsigned int *buf_count, unsigned int *plane_count,
			unsigned int psize[], struct device *alloc_devs[])
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	struct s5p_mfc_dev *dev = ctx->dev;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->state == MFCINST_ERROR) {
			mfc_err("invalid state: %d\n", ctx->state);
			return -EINVAL;
		}

		if (ctx->dst_fmt)
			*plane_count = ctx->dst_fmt->num_planes;
		else
			*plane_count = MFC_ENC_CAP_PLANE_COUNT;
		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
		psize[0] = ctx->enc_dst_buf_size;
		alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (ctx->src_fmt)
			*plane_count = ctx->src_fmt->num_planes;
		else
			*plane_count = MFC_ENC_OUT_PLANE_COUNT;

		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;

		if (ctx->src_fmt && ctx->src_fmt->num_planes == 1)
			psize[0] = ctx->luma_size + ctx->chroma_size;
		else
			psize[0] = ctx->luma_size;
		psize[1] = ctx->chroma_size;
		if (ctx->src_fmt && (ctx->src_fmt->fourcc ==
					V4L2_PIX_FMT_YUV420M || ctx->src_fmt->fourcc ==
					V4L2_PIX_FMT_YVU420M))
			psize[2] = ctx->chroma_size_1;

		if (IS_MFCV6_PLUS(dev)) {
			alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
			alloc_devs[1] = ctx->dev->mem_dev[BANK_L_CTX];
			if (ctx->src_fmt && (ctx->src_fmt->fourcc ==
						V4L2_PIX_FMT_YUV420M || ctx->src_fmt->fourcc ==
						V4L2_PIX_FMT_YVU420M))
				alloc_devs[2] = ctx->dev->mem_dev[BANK_L_CTX];
		} else {
			alloc_devs[0] = ctx->dev->mem_dev[BANK_R_CTX];
			alloc_devs[1] = ctx->dev->mem_dev[BANK_R_CTX];
		}
	} else {
		mfc_err("invalid queue type: %d\n", vq->type);
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
	int ret;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = check_vb_with_fmt(ctx->dst_fmt, vb);
		if (ret < 0)
			return ret;
		i = vb->index;
		ctx->dst_bufs[i].b = vbuf;
		ctx->dst_bufs[i].cookie.stream =
					vb2_dma_contig_plane_dma_addr(vb, 0);
		ctx->dst_bufs_cnt++;
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = check_vb_with_fmt(ctx->src_fmt, vb);
		if (ret < 0)
			return ret;
		i = vb->index;
		ctx->src_bufs[i].b = vbuf;
		s5p_mfc_raw_plane_addrs(ctx, ctx->src_fmt, vb,
					&ctx->src_bufs[i].cookie.raw.luma,
					&ctx->src_bufs[i].cookie.raw.chroma,
					&ctx->src_bufs[i].cookie.raw.chroma_1);
		ctx->src_bufs_cnt++;
	} else {
		mfc_err("invalid queue type: %d\n", vq->type);
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	int ret;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = check_vb_with_fmt(ctx->dst_fmt, vb);
		if (ret < 0)
			return ret;
		mfc_debug(2, "plane size: %ld, dst size: %zu\n",
			vb2_plane_size(vb, 0), ctx->enc_dst_buf_size);
		if (vb2_plane_size(vb, 0) < ctx->enc_dst_buf_size) {
			mfc_err("plane size is too small for capture\n");
			return -EINVAL;
		}
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = check_vb_with_fmt(ctx->src_fmt, vb);
		if (ret < 0)
			return ret;
		mfc_debug(2, "plane size: %ld, luma size: %d\n",
			vb2_plane_size(vb, 0), ctx->luma_size);
		mfc_debug(2, "plane size: %ld, chroma size: %d\n",
			vb2_plane_size(vb, 1), ctx->chroma_size);
		if (ctx->src_fmt->num_planes == 1 ?
		    vb2_plane_size(vb, 0) < ctx->luma_size + ctx->chroma_size :
		    (vb2_plane_size(vb, 0) < ctx->luma_size ||
		     vb2_plane_size(vb, 1) < ctx->chroma_size)) {
			mfc_err("plane size is too small for output\n");
			return -EINVAL;
		}
		if ((ctx->src_fmt->fourcc == V4L2_PIX_FMT_YUV420M ||
		     ctx->src_fmt->fourcc == V4L2_PIX_FMT_YVU420M) &&
		    (vb2_plane_size(vb, 2) < ctx->chroma_size_1)) {
			mfc_err("plane size is too small for output\n");
			return -EINVAL;
		}
	} else {
		mfc_err("invalid queue type: %d\n", vq->type);
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct s5p_mfc_dev *dev = ctx->dev;
	int ret;

	/* A stopped stream resumes where it was; an ended sequence restarts. */
	if (ctx->state == MFCINST_FINISHING || ctx->state == MFCINST_FINISHED)
		ctx->state = ctx->enc_seq_complete ?
			     MFCINST_GOT_INST : MFCINST_RUNNING;

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* Each CAPTURE streaming session is one firmware instance. */
		if (ctx->inst_no == MFC_NO_INSTANCE_SET) {
			if (ctx->state != MFCINST_INIT) {
				mfc_err("invalid state: %d\n", ctx->state);
				return -EINVAL;
			}
			ret = s5p_mfc_open_mfc_inst(dev, ctx);
			if (ret)
				return ret;
		}
		if (IS_MFCV6_PLUS(dev)) {
			/*
			 * The header fixes how many OUTPUT buffers are needed;
			 * produce it now, even if another context runs first.
			 */
			if (ctx->state == MFCINST_GOT_INST &&
			    s5p_mfc_ctx_ready(ctx)) {
				set_work_bit_irqsave(ctx);
				s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
				s5p_mfc_wait_for_done_ctx(ctx,
							S5P_MFC_R2H_CMD_SEQ_DONE_RET,
							0);
			}
			if (q->memory != V4L2_MEMORY_DMABUF) {
				if (ctx->src_bufs_cnt < ctx->pb_count) {
					mfc_err("Need minimum %d OUTPUT buffers\n", ctx->pb_count);
					return -ENOBUFS;
				}
			}
		}
	}

	/* If context is ready then dev = work->data;schedule it to run */
	if (s5p_mfc_ctx_ready(ctx))
		set_work_bit_irqsave(ctx);
	s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);

	return 0;
}

/* Caller holds irqlock, including when the last buffer arrives after drain. */
static void enc_return_last_buffer(struct s5p_mfc_ctx *ctx)
{
	const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };
	struct s5p_mfc_buf *buf;

	if (!ctx->enc_eos_pending || list_empty(&ctx->dst_queue))
		return;

	buf = list_first_entry(&ctx->dst_queue, struct s5p_mfc_buf, list);
	list_del(&buf->list);
	ctx->dst_queue_cnt--;
	ctx->enc_eos_pending = false;
	ctx->draining = false;
	buf->b->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			   V4L2_BUF_FLAG_BFRAME);
	buf->b->flags |= V4L2_BUF_FLAG_LAST;
	vb2_set_plane_payload(&buf->b->vb2_buf, 0, 0);
	vb2_buffer_done(&buf->b->vb2_buf, VB2_BUF_STATE_DONE);
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

void s5p_mfc_enc_stream_complete(struct s5p_mfc_ctx *ctx)
{
	ctx->enc_eos_pending = true;
	enc_return_last_buffer(ctx);
}

static void s5p_mfc_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *mb_entry;

	ctx->draining = false;
	if ((ctx->state == MFCINST_FINISHING ||
		ctx->state == MFCINST_RUNNING) &&
		dev->curr_ctx == ctx->num && dev->hw_lock) {
		ctx->state = MFCINST_ABORT;
		s5p_mfc_wait_for_done_ctx(ctx, S5P_MFC_R2H_CMD_FRAME_DONE_RET,
					  0);
	}
	/* Only a started stream is finished; an unstarted one keeps its step. */
	if (ctx->state == MFCINST_RUNNING || ctx->state == MFCINST_FINISHING ||
	    ctx->state == MFCINST_ABORT)
		ctx->state = MFCINST_FINISHED;
	spin_lock_irqsave(&dev->irqlock, flags);
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ctx->enc_eos_pending = false;
		s5p_mfc_cleanup_queue(&ctx->dst_queue, &ctx->vq_dst);
		INIT_LIST_HEAD(&ctx->dst_queue);
		ctx->dst_queue_cnt = 0;
		/* The instance that referenced these frames closes below. */
		while (!list_empty(&ctx->ref_queue)) {
			mb_entry = list_first_entry(&ctx->ref_queue,
						    struct s5p_mfc_buf, list);
			list_del(&mb_entry->list);
			ctx->ref_queue_cnt--;
			vb2_buffer_done(&mb_entry->b->vb2_buf,
					VB2_BUF_STATE_DONE);
		}
	}
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		cleanup_ref_queue(ctx);
		s5p_mfc_cleanup_queue(&ctx->src_queue, &ctx->vq_src);
		INIT_LIST_HEAD(&ctx->src_queue);
		ctx->src_queue_cnt = 0;
	}
	spin_unlock_irqrestore(&dev->irqlock, flags);
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    ctx->inst_no != MFC_NO_INSTANCE_SET) {
		/* The coded stream ends with its instance; STREAMON opens another. */
		s5p_mfc_clock_on(dev);
		s5p_mfc_close_mfc_inst(dev, ctx);
		s5p_mfc_clock_off(dev);
		ctx->state = MFCINST_INIT;
	}
}

static void s5p_mfc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;
	struct s5p_mfc_buf *mfc_buf;

	if (ctx->state == MFCINST_ERROR) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		cleanup_ref_queue(ctx);
		return;
	}
	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_buf = &ctx->dst_bufs[vb->index];
		mfc_buf->flags = 0;
		/* Mark destination as available for use by MFC */
		spin_lock_irqsave(&dev->irqlock, flags);
		list_add_tail(&mfc_buf->list, &ctx->dst_queue);
		ctx->dst_queue_cnt++;
		enc_return_last_buffer(ctx);
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_buf = &ctx->src_bufs[vb->index];
		/* Unused, and no longer the end of the stream STOP marked it as. */
		mfc_buf->flags = 0;
		spin_lock_irqsave(&dev->irqlock, flags);
		list_add_tail(&mfc_buf->list, &ctx->src_queue);
		ctx->src_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else {
		mfc_err("unsupported buffer type (%d)\n", vq->type);
	}
	if (s5p_mfc_ctx_ready(ctx))
		set_work_bit_irqsave(ctx);
	s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
}

static const struct vb2_ops s5p_mfc_enc_qops = {
	.queue_setup		= s5p_mfc_queue_setup,
	.buf_init		= s5p_mfc_buf_init,
	.buf_prepare		= s5p_mfc_buf_prepare,
	.start_streaming	= s5p_mfc_start_streaming,
	.stop_streaming		= s5p_mfc_stop_streaming,
	.buf_queue		= s5p_mfc_buf_queue,
};

const struct s5p_mfc_codec_ops *get_enc_codec_ops(void)
{
	return &encoder_codec_ops;
}

const struct vb2_ops *get_enc_queue_ops(void)
{
	return &s5p_mfc_enc_qops;
}

const struct v4l2_ioctl_ops *get_enc_v4l2_ioctl_ops(void)
{
	return &s5p_mfc_enc_ioctl_ops;
}

#define IS_MFC51_PRIV(x) ((V4L2_CTRL_ID2WHICH(x) == V4L2_CTRL_CLASS_CODEC) \
						&& V4L2_CTRL_DRIVER_PRIV(x))

int s5p_mfc_enc_ctrls_setup(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_ctrl_config cfg;
	int i;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, NUM_CTRLS);
	if (ctx->ctrl_handler.error) {
		mfc_err("v4l2_ctrl_handler_init failed\n");
		return ctx->ctrl_handler.error;
	}
	for (i = 0; i < NUM_CTRLS; i++) {
		if (!IS_MFCV16_PLUS(ctx->dev) &&
		    (controls[i].id == V4L2_CID_MPEG_VIDEO_VP9_PROFILE ||
		     controls[i].id == V4L2_CID_MPEG_VIDEO_VP9_LEVEL))
			continue;
		if (IS_MFC51_PRIV(controls[i].id)) {
			memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
			cfg.ops = &s5p_mfc_enc_ctrl_ops;
			cfg.id = controls[i].id;
			cfg.min = controls[i].minimum;
			cfg.max = controls[i].maximum;
			cfg.def = controls[i].default_value;
			cfg.name = controls[i].name;
			cfg.type = controls[i].type;
			cfg.flags = 0;

			if (cfg.type == V4L2_CTRL_TYPE_MENU) {
				cfg.step = 0;
				cfg.menu_skip_mask = controls[i].menu_skip_mask;
				cfg.qmenu = mfc51_get_menu(cfg.id);
			} else {
				cfg.step = controls[i].step;
				cfg.menu_skip_mask = 0;
			}
			ctx->ctrls[i] = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
					&cfg, NULL);
		} else {
			if ((controls[i].type == V4L2_CTRL_TYPE_MENU) ||
				(controls[i].type ==
					V4L2_CTRL_TYPE_INTEGER_MENU)) {
				int maximum = controls[i].maximum;
				u64 skip = controls[i].menu_skip_mask;

				/* v16's profile value 2 selects a 4:2:2 10-bit mode. */
				if (IS_MFCV16_PLUS(ctx->dev) &&
				    controls[i].id == V4L2_CID_MPEG_VIDEO_HEVC_PROFILE)
					maximum = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
				/* Constrained baseline needs v6 firmware. */
				if (IS_MFCV6_PLUS(ctx->dev) &&
				    controls[i].id == V4L2_CID_MPEG_VIDEO_H264_PROFILE)
					skip &= ~BIT_ULL(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE);
				ctx->ctrls[i] = v4l2_ctrl_new_std_menu(
					&ctx->ctrl_handler,
					&s5p_mfc_enc_ctrl_ops, controls[i].id,
					maximum, skip,
					controls[i].default_value);
			} else {
				int min = controls[i].minimum;
				int max = controls[i].maximum;
				int def = controls[i].default_value;

				if (IS_MFCV16_PLUS(ctx->dev) &&
				    controls[i].id == V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP) {
					min = 1;
					max = def = 31;
				}
				ctx->ctrls[i] = v4l2_ctrl_new_std(
					&ctx->ctrl_handler,
					&s5p_mfc_enc_ctrl_ops, controls[i].id,
					min, max, controls[i].step, def);
			}
		}
		if (ctx->ctrl_handler.error) {
			mfc_err("Adding control (%d) failed\n", i);
			return ctx->ctrl_handler.error;
		}
		if (controls[i].is_volatile && ctx->ctrls[i])
			ctx->ctrls[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	return 0;
}

void s5p_mfc_enc_ctrls_delete(struct s5p_mfc_ctx *ctx)
{
	int i;

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	for (i = 0; i < NUM_CTRLS; i++)
		ctx->ctrls[i] = NULL;
}

void s5p_mfc_enc_init(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_raw_layout l;
	struct v4l2_format f;
	f.fmt.pix_mp.pixelformat = DEF_SRC_FMT_ENC;
	ctx->src_fmt = find_format(&f, MFC_FMT_RAW);
	f.fmt.pix_mp.pixelformat = DEF_DST_FMT_ENC;
	ctx->dst_fmt = find_format(&f, MFC_FMT_ENC);
	ctx->enc_params.rc_framerate_num = 30;
	ctx->enc_params.rc_framerate_denom = 1;
	/* A complete default format lets a client stream without S_FMT. */
	ctx->codec_mode = ctx->dst_fmt->codec_mode;
	ctx->img_width = DEF_WIDTH;
	ctx->img_height = DEF_HEIGHT;
	s5p_mfc_hw_call(ctx->dev->mfc_ops, enc_calc_src_size, ctx->dev,
			ctx->src_fmt, DEF_WIDTH, DEF_HEIGHT, &l);
	s5p_mfc_enc_apply_layout(ctx, &l);
	ctx->enc_dst_buf_size_req = 0;
	ctx->enc_dst_buf_size = s5p_mfc_enc_bound_dst_size(ctx, &l, 0);
	ctx->state = MFCINST_INIT;
}
