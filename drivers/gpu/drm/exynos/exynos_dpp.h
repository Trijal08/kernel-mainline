/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Samsung ExynosAuto DRM DPP(Display Pre-Processor) driver Header
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 */

#ifndef _EXYNOS_DRM_DPP_H_
#define _EXYNOS_DRM_DPP_H_

#include <linux/clk.h>
#include "exynos_drm_drv.h"

/*
 * DPUM  DPP base: 0x18C2_0000
 * - DPP GF0 offset: 0x1000
 * - DPP GF1 offset: 0x2000
 * - DPP GF2 offset: 0x3000
 * - DPP GF3 offset: 0x4000
 * - DPP GF4 offset: 0x5000
 * - DPP GF5 offset: 0x6000
 * - DPP VG0 offset: 0x7000
 * - DPP VG1 offset: 0x8000
 */
#define DPP_ENABLE 0x0000
#define DPP_SRSET (1 << 24)
#define DPP_HDR_SEL (1 << 11)
#define DPP_SFR_CLOCK_GATE_EN (1 << 10)
#define DPP_SRAM_CLOCK_GATE_EN (1 << 9)
#define DPP_INT_CLOCK_GATE_EN (1 << 8)
#define DPP_ALL_CLOCK_GATE_EN_MASK (0x7 << 8)
#define DPP_PSLVERR_EN (1 << 5)
#define DPP_SFR_UPDATE_FORCE (1 << 4)
#define DPP_QCHANNEL_EN (1 << 3)
#define DPP_OP_STATUS (1 << 2)
#define DPP_TZPC_FLAG (1 << 0)

#define DPP_IRQ 0x0004
#define DPP_CONFIG_ERROR (1 << 21)
#define DPP_STATUS_FRAMEDONE_IRQ (1 << 16)
#define DPP_ALL_IRQ_CLEAR (0x21 << 16)
#define DPP_CONFIG_ERROR_MASK (1 << 6)
#define DPP_IRQ_FRAMEDONE_MASK (1 << 1)
#define DPP_ALL_IRQ_MASK (0x21 << 1)
#define DPP_IRQ_ENABLE (1 << 0)
#define DPP_ALL_IRQ (DPP_CONFIG_ERROR | DPP_STATUS_FRAMEDONE_IRQ)

#define DPP_IN_CON 0x0008
#define DPP_CSC_TYPE(_v) ((_v) << 18)
#define DPP_CSC_TYPE_MASK (3 << 18)
#define DPP_CSC_RANGE(_v) ((_v) << 17)
#define DPP_CSC_RANGE_MASK (1 << 17)
#define DPP_CSC_MODE(_v) ((_v) << 16)
#define DPP_CSC_MODE_MASK (1 << 16)
#define DPP_DITH_MASK_SEL (1 << 5)
#define DPP_DITH_MASK_SPIN (1 << 4)
// #define DPP_ALPHA_SEL(_v)			((_v) << 3)
// #define DPP_ALPHA_SEL_MASK			(1 << 3)
#define DPP_IMG_FORMAT(_v) ((_v) << 0)
#define DPP_IMG_FORMAT_MASK (0x7 << 0)
#define DPP_IMG_FORMAT_ARGB8888 (0 << 0)
#define DPP_IMG_FORMAT_ARGB8101010 (1 << 0)
/* VG only */
#define DPP_IMG_FORMAT_YUV420_8P (2 << 0)
#define DPP_IMG_FORMAT_YUV420_P010 (3 << 0)
#define DPP_IMG_FORMAT_YUV420_8P2 (4 << 0)
#define DPP_IMG_FORMAT_YUV422_8P (5 << 0)
#define DPP_IMG_FORMAT_YUV422_P210 (6 << 0)
#define DPP_IMG_FORMAT_YUV422_8P2 (7 << 0)

#define DPP_IMG_SIZE 0x0018
#define DPP_IMG_HEIGHT(_v) ((_v) << 16)
#define DPP_IMG_HEIGHT_MASK (0x1FFF << 16)
#define DPP_IMG_WIDTH(_v) ((_v) << 0)
#define DPP_IMG_WIDTH_MASK (0x1FFF << 0)
enum dpp_attr {
	DPP_ATTR_AFBC = 0,
	DPP_ATTR_SAJC = 0,
	DPP_ATTR_BLOCK = 1,
	DPP_ATTR_FLIP = 2,
	DPP_ATTR_ROT = 3, /* [KITT2] OTF Rotation is spec-out. */
	DPP_ATTR_CSC = 4,
	DPP_ATTR_SCALE = 5,
	DPP_ATTR_HDR = 6,
	DPP_ATTR_C_HDR = 7,
	DPP_ATTR_C_HDR10_PLUS = 8,
	DPP_ATTR_WCG = 9,
	DPP_ATTR_SBWC = 10,
	DPP_ATTR_HDR10_PLUS = 11,

	DPP_ATTR_IDMA = 16,
	DPP_ATTR_ODMA = 17,
	DPP_ATTR_DPP = 18,
	DPP_ATTR_SRAMC = 19,
	DPP_ATTR_HDR_COMM = 20,
};

enum dpp_comp_type {
	COMP_TYPE_NONE = 0,
	COMP_TYPE_AFBC,
	COMP_TYPE_SAJC,
	COMP_TYPE_SBWC, /* lossless */
	COMP_TYPE_SBWCL, /* lossy */
};

enum dpp_comp_blk_size {
	/* SAJC indepedent block size */
	SAJC_BLK_64B = 0,
	SAJC_BLK_128B = 1,
	SAJC_BLK_256B = 2,
	/* SBWC blk byte num */
	SBWC_BLK_32x2 = 2,
	SBWC_BLK_32x3 = 3,
	SBWC_BLK_32x4 = 4,
	SBWC_BLK_32x5 = 5,
	SBWC_BLK_32x6 = 6,
};

struct dpp_regs {
	void __iomem *dpp_base_regs;
	void __iomem *dma_base_regs;
	void __iomem *dma_common_base_regs;
	void __iomem *sramc_base_regs;
	void __iomem *votf_base_regs;
	void __iomem *scl_coef_base_regs;
	void __iomem *hdr_comm_base_regs;
	void __iomem *dpp_debug_base_regs;
};

/*
 *                      src_width
 *    -----------------------------------------------
 *    | src_offset_x,                               |
 *    | src_offset_y    img_width                   |
 *    |      0-------------------------------       |
 *    |      |//blk_offset_x,///////////////|       |
 *    |      |//blk_offset_y////////////////|       |
 *    |      |////0-------------//blk///////| img   |  src
 *    |      |////|            |/height/////|height | height
 *    |      |////--------------////////////|       |
 *    |      |///////blk_width//////////////|       |
 *    |      --------------------------------       |
 *    |                                             |
 *    -----------------------------------------------
 *
 *                              ///// accessed region
 */
struct dpp_region {
	unsigned int src_width, src_height;
	unsigned int src_offset_x, src_offset_y;

	unsigned int img_width, img_height;

	unsigned int blk_width, blk_height;
	unsigned int blk_offset_x, blk_offset_y;
};

struct decon_frame {
	int x;
	int y;
	u32 w;
	u32 h;
	u32 f_w;
	u32 f_h;
};

struct decon_win_rect {
	int x;
	int y;
	u32 w;
	u32 h;
};

enum dpp_bpc {
	DPP_BPC_8 = 0,
	DPP_BPC_10,
};

struct dpp_config {
	struct dpp_region region;
	// enum dpu_pixel_format format;
	u32 idma_addr[4];

	bool is_bist;
	bool is_afbc;
	bool is_scale;
	bool is_block;
	bool is_4p;
	u32 y_2b_strd;
	u32 c_2b_strd;

	u32 afbc_recov_cnt;
};

#define MAX_PLANE_ADDR_CNT 4

struct dpp_params_info {
	struct decon_frame src;
	struct decon_frame dst;
	struct decon_win_rect block;
	u32 rot;

	u32 min_luminance; /* TODO: remove, if can */
	u32 max_luminance; /* TODO: remove, if can */
	u32 y_hd_y2_stride; /* Luminance header (or Y-2B) stride */
	u32 y_pl_c2_stride; /* Luminance payload (or C-2B) stride */
	u32 c_hd_stride; /* Chrominance header stride */
	u32 c_pl_stride; /* Chrominance payload stride */

	bool is_scale;
	bool is_block;
	u32 format;
	dma_addr_t addr[MAX_PLANE_ADDR_CNT];
	u32 dataspace;
	int h_ratio;
	int v_ratio;
	u32 standard;
	u32 transfer;
	u32 range;
	u32 split;
	enum dpp_bpc in_bpc;

	unsigned long rcv_num;
	enum dpp_comp_type comp_type;
	enum dpp_comp_blk_size blk_size;

	/* votf for output */
	bool votf_o_en;
	u32 votf_o_idx;
	u32 votf_o_base_addr;
	u32 votf_o_mfc_addr;

	bool votf_en;
	u32 votf_buf_idx;
	u32 votf_base_addr;

	/* v910 backward compatibility */;
	struct dpp_config config_v1;
};

enum dpp_irq_type {
	I_FRAMEDONE,
	I_DEADLOCK,
	I_READSLAVE,
	I_CONFIG_ERR,
	I_FRAMESTART,
};

/* reference between dpp device and decon/wod. */
struct dpp_dev_data {
	const struct dpp_cal_ops *cal_ops;
	unsigned long irqflags;

	const u8 nr_idma;
	const u8 nr_dpuf;
	const unsigned long attr;
	const u32 *pixel_formats;
	const u32 num_pixel_formats;
};

enum dpp_state {
	DPP_STATE_OFF = 0,
	DPP_STATE_ON,
	DPP_STATE_WAIT_OFF,
};

struct exynos_dpp_context {
	struct device *dev;
	void __iomem *regs;

	u32 type;
	struct clk *aclk;
};

void dpp_update(struct exynos_dpp_context *dpp, unsigned int channel,
		const struct exynos_drm_plane_state *state);

#endif
