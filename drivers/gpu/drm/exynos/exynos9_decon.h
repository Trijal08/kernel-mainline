/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) Samsung Electronics Co., Ltd.
 *
 * SoC-independent types shared between the Exynos9-class DECON core
 * (exynos9_decon.c) and the per-SoC low-level register back-ends
 * (exynos910 in exynos9_decon.c, zuma in exynos9_decon_zuma.c).
 *
 * The register maps of the exynos910 (Exynos Auto v9) and Google Tensor
 * "zuma" DECON revisions collide by macro name and cannot share a
 * translation unit, so each back-end lives in its own .c with its own
 * register header and exports a decon_cal_ops vtable consumed here.
 */

#ifndef _EXYNOS9_DECON_H_
#define _EXYNOS9_DECON_H_

#include <linux/types.h>
#include <linux/workqueue.h>

#include <video/videomode.h>

#include "exynos_drm_drv.h"

struct device;
struct drm_device;
struct clk;
struct platform_device;
struct decon_win;

enum decon_type {
	EXYNOS9_DECON0 = 0,
	EXYNOS9_DECON1 = 1,
	EXYNOS9_DECON2 = 2,
};

enum decon_op_mode {
	DECON_VIDEO_MODE = 0,
	DECON_MIPI_COMMAND_MODE = 1,
	/* TODO: ADD DP command mode */
};

enum decon_te_mode { DECON_HW_TRIG = 0, DECON_SW_TRIG };

enum decon_set_trig { DECON_TRIG_MASK = 0, DECON_TRIG_UNMASK };

enum decon_out_type {
	DECON_OUT_DSI0 = 1 << 0,
	DECON_OUT_DSI1 = 1 << 1,
	DECON_OUT_DSI = DECON_OUT_DSI0 | DECON_OUT_DSI1,
	DECON_OUT_DP0_SST1 = 1 << 4,
	DECON_OUT_DP0_SST2 = 1 << 5,
	DECON_OUT_DP0_SST3 = 1 << 6,
	DECON_OUT_DP0_SST4 = 1 << 7,
	DECON_OUT_DP1_SST1 = 1 << 8,
	DECON_OUT_DP1_SST2 = 1 << 9,
	DECON_OUT_DP1_SST3 = 1 << 10,
	DECON_OUT_DP1_SST4 = 1 << 11,
	DECON_OUT_DP0 = DECON_OUT_DP0_SST1 | DECON_OUT_DP0_SST2 |
			DECON_OUT_DP0_SST3 | DECON_OUT_DP0_SST4,
	DECON_OUT_DP1 = DECON_OUT_DP1_SST1 | DECON_OUT_DP1_SST2 |
			DECON_OUT_DP1_SST3 | DECON_OUT_DP1_SST4,
	DECON_OUT_DP = DECON_OUT_DP0 | DECON_OUT_DP1,

	DECON_OUT_WB = 1 << 12,
};

enum decon_dsi_mode {
	DSI_MODE_SINGLE = 0,
	DSI_MODE_DUAL_DSI,
	DSI_MODE_DUAL_DISPLAY,
	DSI_MODE_NONE
};

enum decon_irq {
	DECON_IRQ_FS = 0,
	DECON_IRQ_FD,
	DECON_IRQ_EXT,
	DECON_IRQ_DQE_DIM_S,
	DECON_IRQ_DQE_DIM_E,
};

enum decon_blend_mode {
	DECON_BLENDING_NONE = 0x0,
	DECON_BLENDING_PREMULT = 0x1,
	DECON_BLENDING_COVERAGE = 0x2,
	DECON_BLENDING_MAX = 0x3,
};

struct decon_mode {
	enum decon_op_mode op_mode;
	enum decon_dsi_mode dsi_mode;
	enum decon_te_mode te_mode;
};

struct decon_config {
	enum decon_out_type out_type;
	unsigned int image_height;
	unsigned int image_width;
	unsigned int out_bpc;
	struct decon_mode mode;
	unsigned int fps;
};

struct win_color_map {
	bool en;
	u32 val;
};

struct decon_win_config {
	u32 x, y, w, h;
	u32 start_time;
	u32 alpha;
	enum decon_blend_mode blend_mode;
	struct win_color_map color_map;
	u32 dpp_type;
};

struct decon_context {
	u32 idx;
	u32 plane_mask;
	enum decon_type type;
	struct device *dev;
	struct drm_device *drm_dev;
	struct exynos_drm_crtc *crtc;
	struct decon_win *win;
	u32 win_cnt;
	u32 win_max;
	u32 disable_mask;
	struct videomode v_mode;
	struct decon_config config;

	struct clk *aclk;

	u32 irq_fd; /* frame done */
	int te_irq; /* panel hardware TE, drives vblank in command mode */

	/* device ops */
	const struct decon_cal_ops *cal_ops;

	void __iomem *regs[3]; /* main, sub0, sub1*/
};

/*
 * SoC-specific low-level DECON operations.
 *
 * The Exynos9-class DECON IP is shared in shape across revisions - the
 * DRM/CRTC plumbing, the plane/window model and the decon_win_config layout
 * are identical - but the register map itself was re-laid-out between
 * exynos910 (Exynos Auto v9) and the Google Tensor "zuma" family (vendor
 * cal_9865): the SFR block bases differ, most register offsets differ, and a
 * few registers (SRAM allocation, data path) have a different programming
 * model entirely, not merely a different offset.
 *
 * Everything that actually touches the hardware is therefore reached through
 * this vtable, selected from the match data in decon_probe(). The generic core
 * in exynos9_decon.c (mode_valid, atomic_begin/flush, update/disable_plane,
 * irq dispatch, enable/disable) is SoC-independent and calls only through
 * ctx->cal_ops.
 */
struct decon_cal_ops {
	/* map the SFR blocks for this SoC's register topology */
	int (*init)(struct decon_context *ctx, struct platform_device *pdev);
	int (*enable)(struct decon_context *ctx);
	int (*disable)(struct decon_context *ctx);
	void (*set_te)(struct decon_context *ctx, enum decon_set_trig trig);
	void (*enable_window)(struct decon_context *ctx, u32 win_idx,
			      struct decon_win_config *config);
	void (*disable_window)(struct decon_context *ctx, u32 win_idx);
	void (*win_update_req)(struct decon_context *ctx, u32 win_idx);
	u32 (*win_status)(struct decon_context *ctx, u32 win_idx);
	u32 (*win_update_req_get)(struct decon_context *ctx, u32 win_idx);
	void (*update_req_global)(struct decon_context *ctx);
	u32 (*clear_interrupt)(struct decon_context *ctx, enum decon_irq irq);
};

extern const struct decon_cal_ops zuma_decon_cal_ops;

#endif /* _EXYNOS9_DECON_H_ */
