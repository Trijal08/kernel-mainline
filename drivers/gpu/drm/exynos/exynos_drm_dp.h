/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Samsung ExynosAuto DRM Display Port driver Header
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 */

#ifndef __EXYNOS_DRM_DP_H__
#define __EXYNOS_DRM_DP_H__

#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dp_mst_helper.h>

#include <drm/drm_bridge.h>
#include <drm/drm_print.h>
#include <video/videomode.h>

#include "exynos_drm_drv.h"

#include <linux/types.h>
#include <linux/kernel.h> /* DIV_ROUND_CLOSEST */
#include <linux/printk.h> /* pr_xxx */
#include <linux/io.h> /* for use 'readl()', 'writel()' functions */

#define CHECK_RANGE(param, min, max) ((min) <= (param) && (param) <= (max))

enum dpu_pixel_format {
	/* RGB 8bit display */
	/* 4byte */
	DPU_PIXEL_FORMAT_ARGB_8888 = 0,
	DPU_PIXEL_FORMAT_ABGR_8888,
	DPU_PIXEL_FORMAT_RGBA_8888,
	DPU_PIXEL_FORMAT_BGRA_8888,
	DPU_PIXEL_FORMAT_XRGB_8888,
	DPU_PIXEL_FORMAT_XBGR_8888,
	DPU_PIXEL_FORMAT_RGBX_8888,
	DPU_PIXEL_FORMAT_BGRX_8888,
	/* 2byte */
	DPU_PIXEL_FORMAT_RGBA_5551,
	DPU_PIXEL_FORMAT_BGRA_5551,
	DPU_PIXEL_FORMAT_ABGR_4444,
	DPU_PIXEL_FORMAT_RGBA_4444,
	DPU_PIXEL_FORMAT_BGRA_4444,
	DPU_PIXEL_FORMAT_RGB_565,
	DPU_PIXEL_FORMAT_BGR_565,

	/* RGB 10bit display */
	/* 4byte */
	DPU_PIXEL_FORMAT_ARGB_2101010,
	DPU_PIXEL_FORMAT_ABGR_2101010,
	DPU_PIXEL_FORMAT_RGBA_1010102,
	DPU_PIXEL_FORMAT_BGRA_1010102,

	/* YUV 8bit display */
	/* YUV422 2P */
	DPU_PIXEL_FORMAT_NV16,
	DPU_PIXEL_FORMAT_NV61,
	/* YUV422 3P */
	DPU_PIXEL_FORMAT_YVU422_3P,
	/* YUV420 2P */
	DPU_PIXEL_FORMAT_NV12,
	DPU_PIXEL_FORMAT_NV21,
	DPU_PIXEL_FORMAT_NV12M,
	DPU_PIXEL_FORMAT_NV21M,
	/* YUV420 3P */
	DPU_PIXEL_FORMAT_YUV420,
	DPU_PIXEL_FORMAT_YVU420,
	DPU_PIXEL_FORMAT_YUV420M,
	DPU_PIXEL_FORMAT_YVU420M,
	/* YUV - 2 planes but 1 buffer */
	DPU_PIXEL_FORMAT_NV12N,
	DPU_PIXEL_FORMAT_NV12N_10B,

	/* YUV 10bit display */
	/* YUV420 2P */
	DPU_PIXEL_FORMAT_NV12M_P010,
	DPU_PIXEL_FORMAT_NV21M_P010,

	/* YUV420(P8+2) 4P */
	DPU_PIXEL_FORMAT_NV12M_S10B,
	DPU_PIXEL_FORMAT_NV21M_S10B,

	/* YUV422 2P */
	DPU_PIXEL_FORMAT_NV16M_P210,
	DPU_PIXEL_FORMAT_NV61M_P210,

	/* YUV422(P8+2) 4P */
	DPU_PIXEL_FORMAT_NV16M_S10B,
	DPU_PIXEL_FORMAT_NV61M_S10B,

	DPU_PIXEL_FORMAT_NV12_P010,

	DPU_PIXEL_FORMAT_MAX,
};

enum chip_version {
	/* EXYNOS chip version */
	V910,
	V920,
};

struct dpu_panel_timing {
	unsigned long pclk;
	unsigned int vactive;
	unsigned int vfp;
	unsigned int vsa;
	unsigned int vbp;

	unsigned int hactive;
	unsigned int hfp;
	unsigned int hsa;
	unsigned int hbp;
	unsigned int fps;
};

struct dpu_format {
	const char *name;
	enum dpu_pixel_format dpu_fmt; /* user-interface dpu color format */
	u32 dma_fmt_afbc; /* afbc color format to DPU_DMA */
	u32 dma_fmt; /* color format to DPU_DMA */
	u32 dpp_fmt; /* color format to DPP */
};

struct exynos_dsc {
	bool enabled;
	u32 dsc_count;
	u32 slice_count;
	u32 slice_width;
	u32 slice_height;
};

/**
 * DPU register types to be controlled
 */
typedef enum cal_regs_type {
	REGS_NONE = 0, /* This is default type register */

	REGS_IDMA, /* DPU_DMA Read/Write Layer + DPU_DMA Global(v920) */
	REGS_ODMA, /* unused */
	REGS_DMA_GLB, /* DPU_DMA Global(v910) */
	REGS_DPP, /* DPP Layer(common CSC, SCL) */
	REGS_SCL_COEF, /* SCL COEFx */
	REGS_DPP_SRAMC, /* SRAM_CON Layer */
	REGS_VOTF, /* unused */
	REGS_HDR_COMM, /* HDR Common */

	REGS_DECON,
	REGS_DECON_CON,
	REGS_DECON_SHD,
	REGS_DECON_GLB,
	REGS_DECON_DSC,
	REGS_DECON_SRAMC_D,
	REGS_DECON_SRAMC_G,
	REGS_WIN,
	REGS_WIN_CTRL,

	REGS_DSI,
	REGS_DSI_SYSR, /* system register */
	REGS_DPHY,
	REGS_DPHY_BIAS,

	REGS_DP,
	REGS_DPPHY,

	REGS_TYPE_MAX
} regs_type_t;

/* This is for register dump descriptor */
struct cal_dump_desc {
	u32 offs;
	u32 size;
	const char *name; /* subpart has no name(=NULL) */
	u32 shdw; /* shadow offset if device has */
};

struct cal_regs_desc {
	const char *name; /* device name */
	void __iomem *regs;
	regs_type_t type;
	u32 idx; /* device id */

	const struct cal_dump_desc *dps; /* dumps */
	u32 nr_dps; /* number of dumps array */
};

#define SFRDUMP_DEF(_start, _end, _shadow, _name)  \
	{                                          \
		.offs = _start,                    \
		.size = ((_end) - (_start) + 0x4), \
		.shdw = _shadow,                   \
		.name = _name,                     \
	}

/* return compressed DSC slice width(unit: pixel cnt) */
static inline u32 get_comp_dsc_width(const struct exynos_dsc *dsc, u32 bpc)
{
	unsigned int slice_width_pixels =
		DIV_ROUND_UP(dsc->slice_width * bpc, 8);

	return ALIGN(DIV_ROUND_UP(slice_width_pixels, 3), 4);
}

/* common function macro for register control file */
/* to get cal_regs_desc */
#define cal_regs_desc_check(id, max, name)                                     \
	({                                                                     \
		if (id >= max) {                                               \
			cal_log_err(id, "dev(%s) is bigger than max_id(%d)\n", \
				    name, max);                                \
			BUG();                                                 \
		}                                                              \
	})

#define cal_regs_desc_save(desc_name, _regs, _name, _type, id)               \
	({                                                                   \
		regs_##desc_name[id].regs = _regs;                           \
		regs_##desc_name[id].name = _name;                           \
		regs_##desc_name[id].type = _type;                           \
		regs_##desc_name[id].idx = id;                               \
		cal_log_debug(id, "%s: type(%d) regs(0x%p)\n", _name, _type, \
			      &_regs);                                       \
	})

#define cal_dump_desc_save(desc_name, id)                                    \
	({                                                                   \
		regs_##desc_name[id].dps = desc_name##_dumps;                \
		regs_##desc_name[id].nr_dps = ARRAY_SIZE(desc_name##_dumps); \
	})

/* SFR read/write */
static inline uint32_t cal_read(struct cal_regs_desc *regs_desc,
				uint32_t offset)
{
	uint32_t val = 0;
	val = readl(regs_desc->regs + offset);
	return val;
}

static inline void cal_write(struct cal_regs_desc *regs_desc, uint32_t offset,
			     uint32_t val)
{
	writel(val, regs_desc->regs + offset);
}

static inline uint32_t cal_read_mask(struct cal_regs_desc *regs_desc,
				     uint32_t offset, uint32_t mask)
{
	uint32_t val = cal_read(regs_desc, offset);
	val &= (mask);
	return val;
}

static inline void cal_write_mask(struct cal_regs_desc *regs_desc,
				  uint32_t offset, uint32_t val, uint32_t mask)
{
	uint32_t old = cal_read(regs_desc, offset);
	val = (val & mask) | (old & ~mask);
	cal_write(regs_desc, offset, val);
}

#define DEFINE_CAL_REGS_FUNCS(name, size)                                      \
	static struct cal_regs_desc regs_##name[size];                         \
	static inline uint32_t name##_read(u32 idx, u32 offset)                \
	{                                                                      \
		return cal_read(&regs_##name[idx], offset);                    \
	}                                                                      \
	static inline uint32_t name##_read_mask(u32 idx, u32 offset, u32 mask) \
	{                                                                      \
		return cal_read_mask(&regs_##name[idx], offset, mask);         \
	}                                                                      \
	static inline void name##_write(u32 idx, u32 offset, u32 val)          \
	{                                                                      \
		return cal_write(&regs_##name[idx], offset, val);              \
	}                                                                      \
	static inline void name##_write_mask(u32 idx, u32 offset, u32 val,     \
					     u32 mask)                         \
	{                                                                      \
		return cal_write_mask(&regs_##name[idx], offset, val, mask);   \
	}

/* log messages */
#define cal_msg(func, _id, fmt, ...) \
	func("[drm:%s(#%d)] " fmt, __func__, _id, ##__VA_ARGS__)

#define cal_log_enter(id) cal_msg(pr_debug, id, "%s", "+")
#define cal_log_exit(id) cal_msg(pr_debug, id, "%s", "-")

#define cal_log_debug(id, fmt, ...) cal_msg(pr_debug, id, fmt, ##__VA_ARGS__)
#define cal_log_warn(id, fmt, ...) cal_msg(pr_warn, id, fmt, ##__VA_ARGS__)
#define cal_log_info(id, fmt, ...) cal_msg(pr_info, id, fmt, ##__VA_ARGS__)
#define cal_log_err(id, fmt, ...) \
	cal_msg(pr_err_ratelimited, id, fmt, ##__VA_ARGS__)

#define cal_ops(ctx, op, args...)                                              \
	(((ctx)->cal_ops && (ctx)->cal_ops->op) ? ((ctx)->cal_ops->op(args)) : \
						  0)

/* register dumps message */
#define DUMP_PREFIX "[drm:DUMP] "
#define print_dump_info(string, ...) \
	pr_info("%s===== " string " =====\n", DUMP_PREFIX, ##__VA_ARGS__)

static inline void __cal_dump_regs(struct cal_regs_desc *desc, bool shadow)
{
	int i;
	u32 shdw_offs;

	for (i = 0; i < desc->nr_dps; i++) {
		const struct cal_dump_desc *dps = &desc->dps[i];

		if (!dps)
			continue;

		if (shadow && !dps->shdw)
			continue;

		shdw_offs = shadow ? dps->shdw : 0x0;
		if (dps->name)
			print_dump_info("%s: %s %s(+0x%X)", desc->name,
					dps->name, shadow ? "SHADOW " : "",
					shdw_offs);
		// print_hexdump(desc->regs + dps->offs + shdw_offs, dps->size);
	}
}

static inline void cal_dump_regs(struct cal_regs_desc *desc)
{
	const struct cal_dump_desc *dps;

	if (!desc || !desc->regs || !desc->dps)
		return;

	__cal_dump_regs(desc, false);

	dps = &desc->dps[0];
	if (dps->shdw)
		__cal_dump_regs(desc, true);
}


#define MAX_DP_CNT 2
#define MAX_SST_CNT 4
#define MAX_VC_PAYLOAD_TIMESLOT 63
#define NUM_VC_PAYLOAD_SLOT 8

#define LINK_RATE_1_62Gbps 0x06
#define LINK_RATE_2_7Gbps 0x0A
#define LINK_RATE_5_4Gbps 0x14
#define LINK_RATE_8_1Gbps 0x1E

#define AUX_DATA_BUF_COUNT 16
#define AUX_RETRY_COUNT 3
#define AUX_TIMEOUT_1800us 0x03

#define SYNC_POSITIVE 0
#define SYNC_NEGATIVE 1

typedef enum {
	NORAMAL_DATA = 0,
	TRAINING_PATTERN_1 = 1,
	TRAINING_PATTERN_2 = 2,
	TRAINING_PATTERN_3 = 3,
	TRAINING_PATTERN_4 = 5,
} dp_training_pattern;

typedef enum {
	DISABLE_PATTEN = 0,
	D10_2_PATTERN = 1,
	SERP_PATTERN = 2,
	PRBS7 = 3,
	CUSTOM_80BIT = 4,
	HBR2_COMPLIANCE = 5,
} dp_qual_pattern;

enum aux_ch_command_type {
	I2C_WRITE = 0x4,
	I2C_READ = 0x5,
	DPCD_WRITE = 0x8,
	DPCD_READ = 0x9,
};

enum phy_tune_info {
	AMP = 0,
	POST_EMP = 1,
	PRE_EMP = 2,
	IDRV = 3,
};

enum test_pattern {
	COLOR_BAR = 0,
	WGB_BAR,
	MW_BAR,
	CTS_COLOR_RAMP,
	CTS_BLACK_WHITE,
	CTS_COLOR_SQUARE_VESA,
	CTS_COLOR_SQUARE_CEA,
};

typedef enum {
	ENABLE_SCRAM = 0,
	DISABLE_SCRAM = 1,
} dp_scrambling;

enum dp_interrupt_mask {
	PLL_LOCK_CHG_INT_MASK,
	HOTPLUG_CHG_INT_MASK,
	HPD_LOST_INT_MASK,
	PLUG_INT_MASK,
	HPD_IRQ_INT_MASK,
	RPLY_RECEIV_INT_MASK,
	AUX_ERR_INT_MASK,
	HDCP_LINK_CHECK_INT_MASK,
	HDCP_LINK_FAIL_INT_MASK,
	HDCP_R0_READY_INT_MASK,
	VIDEO_FIFO_UNDER_FLOW_MASK,
	VSYNC_DET_INT_MASK,
	AUDIO_FIFO_UNDER_RUN_INT_MASK,
	AUDIO_FIFO_OVER_RUN_INT_MASK,

	ALL_INT_MASK
};

enum dynamic_range_type {
	VESA_RANGE = 0, /* (0 ~ 255) */
	CEA_RANGE = 1, /* (16 ~ 235) */
};

enum bit_depth {
	BPC_6 = 0,
	BPC_8,
	BPC_10,
};

typedef enum {
	V640X480P60,
	V640X480P30,
	V720X480P60,
	V720X576P50,
	V1280X800P60RB,
	V1280X720P60,
	V1366X768P60,
	V1280X1024P60,
	V1920X1080P24,
	V1920X1080P25,
	V1920X1080P30,
	V1600X900P60RB,
	V1920X1080P60,
	V1920X1200P60,
	V1920X1200P60P,
	V1920X1200P30,
	V1920X1200P30P,
	V3840X2160P24,
	V3840X2160P25,
	V3840X2160P30,
	V4096X2160P24,
	V4096X2160P25,
	V4096X2160P30,
	V3840X2160P50,
	V3840X2160P60,
	V4096X2160P50,
	V4096X2160P60,
	V1560X700P60,
	V800X400P60,
	V1120X780P60,
	/* Used for AR HUD */
	V1440X2560P60,
	V1440X720P60,
	V1800X900P60,
	VIDEO_FORMAT_MAX,
} videoformat;

struct dp_support_video {
	videoformat video_format;
	u32 hactive;
	u32 hfront_porch;
	u32 hsync_len;
	u32 hback_porch;
	u32 vactive;
	u32 vfront_porch;
	u32 vsync_len;
	u32 vback_porch;
	u32 pixelclock;
	u32 vsync_pol;
	u32 hsync_pol;
	u8 vic;
	char *name;
};

#define MAX_PPS_NUM 96 /* PPS96 through PPS127 are reverved until DSC v1.2a */
struct dp_dsc {
	bool enable;
	u8 slice_count;
	u16 chunk_size;
	u8 pps[MAX_PPS_NUM]; /* DSC Picture Paremeter Set */
};

struct dp_video_info {
	struct dp_support_video vm;
	unsigned int bpc; /* bits per component */

	u32 sst_id; /* dp_sst_idx_t - 1 */
	enum dynamic_range_type dyn_range;

	/* DSC */
	struct dp_dsc dsc;
};

extern const struct dp_support_video support_videos[];

/*************** CP CAL APIs exposed to DP driver ***************/
struct dp_regs {
	void __iomem *link_addr;
	void __iomem *phy_addr;
};

/* DP0, DP1 */
typedef enum dp_regs_id {
	REGS_DP0_ID = 0,
	REGS_DP1_ID,
	REGS_DP_ID_MAX
} dp_regs_id_t;

typedef enum dp_regs_type {
	REGS_DP_LINK = 0,
	REGS_DP_PHY,
	REGS_DP_TYPE_MAX
} dp_regs_type_t;

typedef enum dp_irq_type {
	DP_IRQ_HPD_IRQ_FLAG = (1 << 11),
	DP_IRQ_HPD_CHG = (1 << 10),
	DP_IRQ_HPD_LOST = (1 << 9),
	DP_IRQ_HPD_PLUG_INT = (1 << 8),
	DP_IRQ_MAPI_FIFO_UNDER_FLOW = (1 << 8),
} dp_irq_type_t;

typedef enum dp_irq_reg_type {
	DP_IRQ_REG_SYSTEM,

	DP_IRQ_REG_SST1_SET0,
	DP_IRQ_REG_SST2_SET0,
	DP_IRQ_REG_SST3_SET0,
	DP_IRQ_REG_SST4_SET0,

	DP_IRQ_REG_SST1_SET1,
	DP_IRQ_REG_SST2_SET1,
	DP_IRQ_REG_SST3_SET1,
	DP_IRQ_REG_SST4_SET1,
} dp_irq_reg_type_t;

u32 dp_reg_read_vcpi_timeslot(u32 id, u32 index);

void dp_reg_sw_reset(u32 id);
void dp_reg_phy_reset(u32 id, u32 en);
void dp_reg_phy_init_setting(u32 id);
u32 dp_reg_phy_get_link_bw(u32 id);
void dp_reg_phy_set_link_bw(u32 id, u8 link_rate);
void dp_reg_phy_mode_setting(u32 id);
void dp_reg_wait_phy_pll_lock(u32 id);
void dp_reg_phy_disable(u32 id);
void dp_reg_set_lane_count(u32 id, u8 lane_cnt);
u32 dp_reg_get_lane_count(u32 id);
void dp_reg_set_enhanced_mode(u32 id, u32 en);
void dp_reg_set_training_pattern(u32 id, dp_training_pattern pattern);
void dp_reg_scrambling_enable(u32 id, bool status);
void dp_reg_set_voltage_and_pre_emphasis(u32 id, u8 *voltage, u8 *pre_emphasis);
void dp_reg_set_phy_tune(u32 id, u32 phy_lane_num, u32 amp_lvl,
			 u32 pre_emp_lvl);
void dp_reg_init(u32 id);
void dp_reg_deinit(u32 id);
void dp_reg_set_hpd_interrupt(u32 id, u32 en);
void dp_reg_set_plug_interrupt(u32 id, u32 en);
u32 dp_reg_get_hpd_status(u32 id);
u32 dp_reg_get_int_and_clear(u32 id, u32 irq_reg);
void dp_reg_set_video_config(u32 id, struct dp_video_info dp_video_info);
void dp_reg_set_bist_video_config(u32 id, struct dp_video_info dp_video_info,
				  u8 type);
void dp_reg_start(u32 id, u32 sst_id);
void dp_reg_stop(u32 id, u32 sst_id);
int dp_reg_aux_write(u32 id, u32 comm, u32 address, u32 length, u8 *data);
int dp_reg_aux_read(u32 id, u32 comm, u32 address, u32 length, u8 *data);
int dp_reg_dpcd_write_burst(u32 id, u32 address, u32 length, u8 *data);
int dp_reg_dpcd_read_burst(u32 id, u32 address, u32 length, u8 *data);

int dp_regs_desc_init(u32 dp_id, struct dp_regs *regs);

void dp_reg_set_mst_en(u32 id, u32 en);
void dp_reg_set_strm_x_y(u32 id, u32 sst_id, u32 x_val, u32 y_val);
void dp_reg_set_vcpi_timeslot(u32 id, u32 sst_id, u32 start, u32 size);
void dp_reg_remove_vcpi_timeslot(u32 id, u32 sst_id);
int dp_reg_wait_for_vcpi_update(u32 id);
void dp_reg_set_mst_always_sent_act(u32 id, u32 en);
int dp_reg_get_link_clock(u32 id);
bool dp_reg_get_sst_pstate(u32 id);
void dp_reg_set_dsc_fec(u32 id, u32 en);

/* log messages */
#define dp_msg(func, dev, fmt, ...)	\
	func(dev, fmt, ##__VA_ARGS__)

#define dp_log_enter(dev)	dp_msg(DRM_DEV_DEBUG, dev, "%s", "+")
#define dp_log_exit(dev)	dp_msg(DRM_DEV_DEBUG, dev, "%s", "-")

#define dp_log_dbg(dev, f, ...)  dp_msg(DRM_DEV_DEBUG_DRIVER, dev, f, ##__VA_ARGS__)
#define dp_log_kms(dev, f, ...)  dp_msg(DRM_DEV_DEBUG_KMS, dev, f, ##__VA_ARGS__)
#define dp_log_info(dev, f, ...) dp_msg(DRM_DEV_INFO, dev, f, ##__VA_ARGS__)
#define dp_log_err(dev, f, ...)  dp_msg(DRM_DEV_ERROR, dev, f, ##__VA_ARGS__)
#define dp_log_errl(dev, f, ...) dp_msg(DRM_DEV_ERROR_RATELIMITED, dev, f, ##__VA_ARGS__)

#define MAX_LANE_CNT 4

enum dp_state {
	DP_STATE_OFF,
	DP_STATE_LINKED,
	DP_STATE_ON,
};

enum hotplug_state {
	HPD_UNPLUG	= 0,
	HPD_PLUG	= BIT(0),
	HPD_IRQ		= BIT(1),
	HPD_CHECK	= BIT(2),
	HPD_LT_FAILED 	= BIT(3),
};

typedef enum exynos_dp_sst_index {
	DP_SST_UNKNOWN = 0,
	DP_SST_1 = 1,
	DP_SST_2 = 2,
	DP_SST_3 = 3,
	DP_SST_4 = 4,
	DP_SST_MAX = DP_SST_4,
} dp_sst_idx_t;

typedef enum exynos_dp_debug_lt {
	DEBUG_LT_NORMAL,
	DEBUG_LT_DPCD_READ_FAIL,
	DEBUG_LT_FAIL_FIXED_BW,
	DEBUG_LT_FAIL_TRY_BW_DOWN,
	DEBUG_LT_BW_LOWER,
	DEBUG_LT_BW_NO_STEPDOWN,
} dp_debug_lt;

struct exynos_dp_video_info {
	struct videomode vm;	/* clock & resolution & porch */
	bool hsync_pol;		/* polarity */
	bool vsync_pol;		/* polarity */
	unsigned int bpc;	/* bits per component */
	unsigned int vrefresh; /* vrefresh freq */

	u32 sst_id;	/* dp_sst_idx_t - 1 */
	enum dynamic_range_type dyn_range;

	/* DSC */
	struct {
		bool enable;
		u8 slice_count;
		u16 chunk_size;
		struct drm_dsc_picture_parameter_set pps;
	} dsc;
};
struct exynos_dp_lt_info {
	int link_rate;
	u8 max_link_lane;
	u8 lane_cnt;
	u8 enhanced_frame_cap;

	u8 voltage_swing[MAX_LANE_CNT];
	u8 pre_emphasis[MAX_LANE_CNT];
};

struct exynos_dp_subdev {
	struct device			*dev;

	enum chip_version		version;

	/* Filled by me: drm related */
	struct drm_device		*drm_dev;
	/* filled from super-dev: drm related */
	struct drm_dp_aux		aux;
	void (*detect)(struct device *);
	void (*mst_config)(struct device *, bool is_mst);
	void (*mst_irq)(struct device *);
	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE];
	u8 fec_capable;
	bool force_dsc_dis;		/* dsc disable forcibly */
	bool mst_dsc_en;		/* dsc enable w/ mst_hub over DP-SST */
	u8 downstream_ports[DP_RECEIVER_CAP_SIZE];
	struct list_head mst_list;

	u32 id;
	int irq;
	int hpd_gpio;
	int hpd_gpio_irq;

	struct phy *phy;
	spinlock_t slock;

	struct workqueue_struct *dp_wq;
	struct delayed_work hpd_plug_work;
	struct delayed_work hpd_unplug_work;
	struct delayed_work hpd_irq_work;

	struct dp_regs regs;

	enum dp_state state;
	struct exynos_dp_lt_info lt_info;

	int hpd_state;
	bool training_state;
	u32 bist_use;
	enum test_pattern bist_type;
	struct exynos_dp_video_info vi[DP_SST_MAX];

	struct mutex	lock;
	struct mutex	pwlock;

	/* DEBUG */
	struct {
		bool debug_hpd_irq;
		dp_debug_lt debug_lt;
	} dp_debug;
};

#define MST_SLOT_INIT_NUM 1
struct dp_mst_config {
	struct list_head list;
	int sst_id;
	int num_slots;

	u32 x_val;
	u32 y_val;
};


enum msg_aux_client_type {
	MSG_AUX_CLIENT,			/* supported messaging aux client */
	SKIP_MSG_AUX_CLIENT,		/* not supported messaing aux client */
	FIX_TOPOLOGY_WITH_MST_HUB,
	/* This is the mode for testing
	 * not support messaing aux client with DP MST HUB.
	 * The MST topology contsructs by reading Device-tree,
	 * not using Messaing aux client.
	 * After then the sequence works as a device that
	 * supports Messaing aux client. it's just for test.
	 */
};

struct dp_encoder {
	struct drm_encoder		base;
	struct drm_encoder		*remote_base;
	struct dp_connector		*dp_connector;
	struct exynos_drm_dp		*dp;
	dp_sst_idx_t			sst_idx;
	enum exynos_drm_output_type	output_type;

	int				mst_pbn;
	int				mst_slots;
};

struct dp_connector {
	struct drm_connector		base;
	struct drm_connector		*remote_base;
	struct drm_dp_mst_port		*port;
	struct dp_encoder		*dp_encoder;
	struct exynos_drm_dp		*dp;
	enum drm_connector_status	status;

	int				native_mode;
	bool				native_only;
};

struct drm_remote {
	struct drm_device		*drm_dev;
	int (*detect)(struct dp_connector *rdp_con,
			     enum drm_connector_status status,
			     struct drm_device *drm_dev);
};

struct exynos_drm_dp {
	struct drm_encoder		*encoder;
	struct drm_connector		*connector;
	struct drm_device		*drm_dev;
	struct device			*dev;

	u32				id;
	enum exynos_drm_output_type	output_type;
	struct display_timings		*timings;
	struct drm_display_mode         cur_mode;
	/* mst */
	bool				is_mst;
	unsigned long			used_sst;
	struct drm_dp_mst_topology_mgr mst_mgr;
	enum msg_aux_client_type	skip_messaging_aux_client;

	struct exynos_dp_subdev		*subdev;

	/* vmst */
	struct drm_bridge		bridge;
	struct drm_remote		remote[DP_SST_MAX];
};

#if IS_ENABLED(CONFIG_DRM_EXYNOS9_DP)
int exynos_drm_dp_dump_sfr(struct exynos_dp_subdev *subdev);
void exynos_drm_dp_stream_enable(struct exynos_dp_subdev *dp, dp_sst_idx_t sst_idx);
void exynos_drm_dp_stream_disable(struct exynos_dp_subdev *dp, dp_sst_idx_t sst_idx);
bool exynos_drm_dp_is_hpd_connected(struct exynos_dp_subdev *dp);
void exynos_drm_dp_videomode_set(struct exynos_dp_subdev *dp,
		struct exynos_dp_video_info *vi, dp_sst_idx_t sst_idx);
void exynos_drm_dp_hpd_en(struct exynos_dp_subdev *dp);
int exynos_drm_dp_link_training(struct exynos_dp_subdev *dp);
void exynos_drm_dp_set_normal_data(struct exynos_dp_subdev *dp);
void exynos_drm_dp_dpcd_status_dump(struct exynos_dp_subdev *dp);

uint32_t exynos_drm_dp_find_possible_crtc(struct exynos_drm_dp *dp, int sst_idx);
dp_sst_idx_t exynos_drm_dp_get_sst_idx(struct drm_encoder *encoder);
int exynos_drm_dp_add_timings(struct exynos_drm_dp *dp,
			struct drm_connector *connector);
struct drm_connector *exynos_drm_dp_find_connector
				(struct drm_encoder *encoder);
void exynos_drm_dp_to_videoinfo(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct exynos_dp_video_info *vi);
bool exynos_drm_dp_dsc_enable(struct drm_encoder *encoder,
		struct exynos_dp_video_info *vi,
		dp_sst_idx_t sst_idx);
void exynos_drm_dp_dsc_disable(struct drm_encoder *encoder,
		dp_sst_idx_t sst_idx);
#else
static inline int exynos_drm_dp_dump_sfr(struct exynos_dp_subdev *subdev)
{
	return 0;
}
static inline void exynos_drm_dp_stream_enable(struct exynos_dp_subdev *dp, dp_sst_idx_t sst_idx)
{
}
static inline void exynos_drm_dp_stream_disable(struct exynos_dp_subdev *dp, dp_sst_idx_t sst_idx)
{
}
static inline bool exynos_drm_dp_is_hpd_connected(struct exynos_dp_subdev *dp)
{
	return false;
}
static inline void exynos_drm_dp_videomode_set(struct exynos_dp_subdev *dp,
		struct exynos_dp_video_info *vi, dp_sst_idx_t sst_idx)
{
}
static inline void exynos_drm_dp_hpd_en(struct exynos_dp_subdev *dp)
{
}
static inline int exynos_drm_dp_link_training(struct exynos_dp_subdev *dp)
{
	return 0;
}
static inline void exynos_drm_dp_set_normal_data(struct exynos_dp_subdev *dp)
{
}
static inline void exynos_drm_dp_dpcd_status_dump(struct exynos_dp_subdev *dp)
{
}
static inline uint32_t exynos_drm_dp_find_possible_crtc(struct exynos_drm_dp *dp, int sst_idx)
{
	return 0;
}
static inline dp_sst_idx_t exynos_drm_dp_get_sst_idx(struct drm_encoder *encoder)
{
	return DP_SST_UNKNOWN;
}
static inline int exynos_drm_dp_add_timings(struct exynos_drm_dp *dp,
			struct drm_connector *connector)
{
	return 0;
}
static inline struct drm_connector *exynos_drm_dp_find_connector
				(struct drm_encoder *encoder)
{
	return NULL;
}
static inline void exynos_drm_dp_to_videoinfo(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct exynos_dp_video_info *vi)
{
}
static inline bool exynos_drm_dp_dsc_enable(struct drm_encoder *encoder,
		struct exynos_dp_video_info *vi,
		dp_sst_idx_t sst_idx)
{
	return false;
}
static inline void exynos_drm_dp_dsc_disable(struct drm_encoder *encoder,
		dp_sst_idx_t sst_idx)
{
}

#endif

#if IS_ENABLED(CONFIG_DRM_EXYNOS9_DP_MST)
extern int exynos_drm_dp_mst_init(struct dp_connector *dp_connector);
extern void exynos_drm_dp_mst_dump_topology(struct seq_file *m,
			      struct drm_dp_mst_topology_mgr *mgr);
extern bool exynos_dp_mst_cap(struct exynos_dp_subdev *dp);
#else
static inline int exynos_drm_dp_mst_init(struct dp_connector *dp_connector)
{
	return 0;
}

static inline void exynos_drm_dp_mst_dump_topology(struct seq_file *m,
			      struct drm_dp_mst_topology_mgr *mgr)
{
}

static inline bool exynos_dp_mst_cap(struct exynos_dp_subdev *dp)
{
	return false;
}
#endif

#if IS_ENABLED(CONFIG_DRM_EXYNOS9_DP_MST_TOPOLOGY)
extern int exynos_drm_dp_mst_topology_mgr_init(struct drm_dp_mst_topology_mgr *mgr,
				 struct drm_device *dev, struct drm_dp_aux *aux,
				 int max_dpcd_transaction_bytes, int max_payloads,
				 int max_lane_count, int max_link_rate,
				 int conn_base_id);
extern void exynos_drm_dp_mst_mapping_sst_id_to_vcpi(struct drm_dp_mst_topology_mgr *mgr,
		struct drm_dp_mst_port *port, int sst_idx);
extern int exynos_drm_dp_update_payload_part1(struct drm_dp_mst_topology_mgr *mgr);
extern int exynos_drm_dp_clear_update_payload(struct drm_dp_mst_topology_mgr *mgr);
#else
static inline int exynos_drm_dp_mst_topology_mgr_init(struct drm_dp_mst_topology_mgr *mgr,
				 struct drm_device *dev, struct drm_dp_aux *aux,
				 int max_dpcd_transaction_bytes, int max_payloads,
				 int max_lane_count, int max_link_rate,
				 int conn_base_id)

{
	return 0;
}
static inline void exynos_drm_dp_mst_mapping_sst_id_to_vcpi(struct drm_dp_mst_topology_mgr *mgr,
		struct drm_dp_mst_port *port, int sst_idx)
{
}
static inline int exynos_drm_dp_update_payload_part1(struct drm_dp_mst_topology_mgr *mgr)
{
	return 0;
}
static inline int exynos_drm_dp_clear_update_payload(struct drm_dp_mst_topology_mgr *mgr)
{
	return 0;
}
#endif

#define to_subdev(nm)		container_of(nm, struct exynos_dp_subdev, nm)
#define to_encoder(nm)		container_of(nm, struct dp_encoder, base)
#define to_connector(nm)	container_of(nm, struct dp_connector, base)
#define encoder_to_dp(nm)	to_encoder(nm)->dp
#define connector_to_dp(nm)	to_connector(nm)->dp
#define to_dp_sst(id, sst)	((id * DP_SST_4) + (sst - 1))

#endif
