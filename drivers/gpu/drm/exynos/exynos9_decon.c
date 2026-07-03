// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/iopoll.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_mode.h>
#include <drm/drm_blend.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>
#include <video/videomode.h>

#include "exynos_drm_fb.h"
#include "exynos_dpp.h"
#include "exynos_dpu_dma.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_plane.h"

#include "regs-decon9.h"

enum decon_type {
	EXYNOS9_DECON0 = 0,
	EXYNOS9_DECON1 = 1,
	EXYNOS9_DECON2 = 2,
};

/* PD : Porter-Duff */
enum decon_blend_pd_func {
	PD_FUNC_CLEAR = 0x0,
	PD_FUNC_COPY = 0x1,
	PD_FUNC_DESTINATION = 0x2,
	PD_FUNC_SOURCE_OVER = 0x3,
	PD_FUNC_DESTINATION_OVER = 0x4,
	PD_FUNC_SOURCE_IN = 0x5,
	PD_FUNC_DESTINATION_IN = 0x6,
	PD_FUNC_SOURCE_OUT = 0x7,
	PD_FUNC_DESTINATION_OUT = 0x8,
	PD_FUNC_SOURCE_A_TOP = 0x9,
	PD_FUNC_DESTINATION_A_TOP = 0xa,
	PD_FUNC_XOR = 0xb,
	PD_FUNC_PLUS = 0xc,
	PD_FUNC_USER_DEFINED = 0xd,
};

/*
 * ALPHA_MULT_SRC_SEL_ALPHA0 : ALPHA_MULT = WINx_ALPHA_0_F * WINx_ALPHA_0_F
 * ALPHA_MULT_SRC_SEL_ALPHA1 : ALPHA_MULT = WINx_ALPHA_1_F * WINx_ALPHA_0_F
 * ALPHA_MULT_SRC_SEL_AF     : ALPHA_MULT = Af (Foreground incoming pixel-alpha) * WINx_ALPHA_0_F
 * ALPHA_MULT_SRC_SEL_AB     : ALPHA_MULT = Ab (Background incoming pixel-alpha) * WINx_ALPHA_0_F
 */
enum decon_blend_alpha_mult_src {
	ALPHA_MULT_SRC_SEL_ALPHA0 = 0x0,
	ALPHA_MULT_SRC_SEL_ALPHA1 = 0x1,
	ALPHA_MULT_SRC_SEL_AF = 0x2,
	ALPHA_MULT_SRC_SEL_AB = 0x3,
};

enum decon_blend_mode {
	DECON_BLENDING_NONE = 0x0,
	DECON_BLENDING_PREMULT = 0x1,
	DECON_BLENDING_COVERAGE = 0x2,
	DECON_BLENDING_MAX = 0x3,
};

enum decon_win_alpha_coef {
	BND_COEF_ZERO = 0x0,
	BND_COEF_ONE = 0x1,
	BND_COEF_AF = 0x2,
	BND_COEF_1_M_AF = 0x3,
	BND_COEF_AB = 0x4,
	BND_COEF_1_M_AB = 0x5,
	BND_COEF_PLANE_ALPHA0 = 0x6,
	BND_COEF_1_M_PLANE_ALPHA0 = 0x7,
	BND_COEF_PLANE_ALPHA1 = 0x8,
	BND_COEF_1_M_PLANE_ALPHA1 = 0x9,
	BND_COEF_ALPHA_MULT = 0xA,
	BND_COEF_1_M_ALPHA_MULT = 0xB,
};

enum decon_rgb_order {
	DECON_RGB = 0x0,
	DECON_GBR = 0x1,
	DECON_BRG = 0x2,
	DECON_BGR = 0x4,
	DECON_RBG = 0x5,
	DECON_GRB = 0x6,
};

enum decon_op_mode {
	DECON_VIDEO_MODE = 0,
	DECON_MIPI_COMMAND_MODE = 1,
	/* TODO: ADD DP command mode */
};

enum decon_te_mode { DECON_HW_TRIG = 0, DECON_SW_TRIG };

enum decon_set_trig { DECON_TRIG_MASK = 0, DECON_TRIG_UNMASK };

enum decon_te_from {
	DECON_TE_FROM_DDI0 = 0,
	DECON_TE_FROM_DDI1 = 1,
	DECON_TE_FROM_DDI2 = 2,
	MAX_DECON_TE_FROM_DDI = 3,
};

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

struct decon_mode {
	enum decon_op_mode op_mode;
	enum decon_dsi_mode dsi_mode;
	enum decon_te_mode te_mode;
};

struct decon_urgent {
	u32 rd_en;
	u32 rd_hi_thres;
	u32 rd_lo_thres;
	u32 rd_wait_cycle;
	u32 wr_en;
	u32 wr_hi_thres;
	u32 wr_lo_thres;
	bool dta_en;
	u32 dta_hi_thres;
	u32 dta_lo_thres;
};

struct decon_vendor_pps {
	unsigned int initial_xmit_delay;
	unsigned int initial_dec_delay;
	unsigned int scale_increment_interval;
	unsigned int final_offset;
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

#define IS_DECON0(decon_idx) ((decon_idx) & 1 ? 0 : 1)
#define IS_DSC0(dsc_idx) ((dsc_idx) & 1 ? 0 : 1)

struct decon_cal_info {
	u32 id;
	const char *name;
};

#define DECON_CAL_INFO(_type) [(_type)] = { .id = (_type), .name = #_type }

#define DECON_VIDEO_MODE_WIDTH_MIN 160
#define DECON_COMMAND_MODE_WIDTH_MIN 16
#define DECON_WIDTH_MAX 8192
#define DECON_HEIGHT_MIN 8
#define DECON_HEIGHT_MAX 8192

struct decon_win {
	u32 idx;
	struct decon_win_config config;
	struct exynos_dpp_context *dpp;
	struct exynos_drm_plane plane;
	struct exynos_drm_plane_config plane_config;
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

	/* device ops */
	const struct decon_cal_ops *cal_ops;

	bool is_colormap;

	bool fake_vblank;
	struct delayed_work dwork;
	void __iomem *regs[3]; /* main, sub0, sub1*/
};

#ifndef MSEC
#define MSEC (1000)
#endif

#define MAX_WIN_PER_DECON 8

enum {
	DPUM_DECON0 = 0,
	DPUM_DECON1,

	DPUS0_DECON0,
	DPUS0_DECON1,

	DPUS1_DECON0,
	DPUS1_DECON1,

	MAX_EXYNOS910_DECON,
};

enum decon_idma_type {
	IDMA_GF0 = 0,
	IDMA_G0,
	IDMA_VG0,
	IDMA_G1,
	IDMA_GF1,
	IDMA_G2 = 5,
	IDMA_VG1,
	IDMA_G3,
	IOTF0,
	IOTF1,
	IDMA_NONE,
};

enum decon_fifo_mode {
	DECON_FIFO_00K = 0,
	DECON_FIFO_04K,
	DECON_FIFO_08K,
	DECON_FIFO_12K,
	DECON_FIFO_16K,
	DECON_FIFO_20K,
	DECON_FIFO_24K,
	DECON_FIFO_28K,
};

enum decon_dsc_path {
	DECON_DSC_NONE = 0x0,
	DECON0_DSC_ENC0 = 0x1,
	DECON1_DSC_ENC1 = 0x2,
	DECON0_DSCC_DSC_ENC01 = 0xB,
};

enum decon_outif_path {
	DECON0_OUTFIFO0_DSIMIF0 = 0x1,
	DECON0_OUTFIFO0_DSIMIF1 = 0x2,
	DECON0_SPLITTER_OUTFIFO01_DSIMIF01 = 0x3,
	DECON0_OUTFIFO0_DPIF = 0x8,
	DECON1_OUTFIFO0_DSIMIF = 0x1,
	DECON1_OUTFIFO0_DPIF = 0x8,
};

enum decon_connection {
	DECON0_OUT0_TO_DSIM0 = 0,
	DECON0_OUT1_TO_DSIM0 = 1,
	DECON1_OUT0_TO_DSIM0 = 2,

	DECON0_OUT0_TO_DSIM1 = 0,
	DECON0_OUT1_TO_DSIM1 = 1,
	DECON1_OUT0_TO_DSIM1 = 2,

	DECON0_OUT0_TO_DP = 0,
	DECON1_OUT0_TO_DP = 1,
};

static void __iomem *regs_decon[MAX_EXYNOS910_DECON];
static inline uint32_t decon_read(u32 idx, u32 offset)
{
	return readl(regs_decon[idx] + offset);
}
static inline void decon_write(u32 idx, u32 offset, u32 val)
{
	writel(val, regs_decon[idx] + offset);
}
static inline void decon_write_mask(u32 idx, u32 offset, u32 val, u32 mask)
{
	uint32_t old = decon_read(idx, offset);
	val = (val & mask) | (old & ~mask);
	decon_write(idx, offset, val);
}

static void __iomem *regs_decon_con[MAX_EXYNOS910_DECON];
static inline uint32_t decon_con_read(u32 idx, u32 offset)
{
	return readl(regs_decon_con[idx] + offset);
}
static inline void decon_con_write(u32 idx, u32 offset, u32 val)
{
	writel(val, regs_decon_con[idx] + offset);
}
static inline void decon_con_write_mask(u32 idx, u32 offset, u32 val, u32 mask)
{
	uint32_t old = decon_con_read(idx, offset);
	val = (val & mask) | (old & ~mask);
	decon_con_write(idx, offset, val);
}
static void __iomem *regs_decon_global[MAX_EXYNOS910_DECON];
static inline uint32_t decon_global_read(u32 idx, u32 offset)
{
	return readl(regs_decon_global[idx] + offset);
}
static inline void decon_global_write(u32 idx, u32 offset, u32 val)
{
	writel(val, regs_decon_global[idx] + offset);
}
static inline void decon_global_write_mask(u32 idx, u32 offset, u32 val,
					   u32 mask)
{
	uint32_t old = decon_global_read(idx, offset);
	val = (val & mask) | (old & ~mask);
	decon_global_write(idx, offset, val);
}
static void __iomem *regs_win[MAX_EXYNOS910_DECON * 8];
static inline uint32_t win_read(u32 idx, u32 offset)
{
	return readl(regs_win[idx] + offset);
}
static inline void win_write(u32 idx, u32 offset, u32 val)
{
	writel(val, regs_win[idx] + offset);
}
static inline void win_write_mask(u32 idx, u32 offset, u32 val, u32 mask)
{
	uint32_t old = win_read(idx, offset);
	val = (val & mask) | (old & ~mask);
	win_write(idx, offset, val);
}
static void __iomem *regs_winctrl[MAX_EXYNOS910_DECON * 8];
static inline uint32_t winctrl_read(u32 idx, u32 offset)
{
	return readl(regs_winctrl[idx] + offset);
}
static inline uint32_t winctrl_read_mask(u32 idx, u32 offset, u32 mask)
{
	uint32_t val = winctrl_read(idx, offset);
	val &= (mask);
	return val;
}
static inline void winctrl_write(u32 idx, u32 offset, u32 val)
{
	writel(val, regs_winctrl[idx] + offset);
}
static inline void winctrl_write_mask(u32 idx, u32 offset, u32 val, u32 mask)
{
	uint32_t old = winctrl_read(idx, offset);
	val = (val & mask) | (old & ~mask);
	winctrl_write(idx, offset, val);
}

#define to_regwin_idx(decon_idx, win_idx) \
	((decon_idx) * MAX_WIN_PER_DECON + (win_idx))

static inline u32 decon_reg_dmaid2chmap(u32 dma_id)
{
	switch (dma_id) {
	case 0:
		return IDMA_GF0;
	case 1:
		return IDMA_G0;
	case 2:
		return IDMA_G1;
	case 3:
		return IDMA_GF1;
	case 4:
		return IDMA_VG0;
	case 5:
		return IDMA_G2;
	case 6:
		return IDMA_VG1;
	case 7:
		return IDMA_G3;
	default:
		return IDMA_NONE;
	}
}

static void decon_reg_set_window_channel(u32 decon_idx, u32 win_idx, u32 dma_id)
{
	u32 val, mask;
	u32 ch_type = decon_reg_dmaid2chmap(dma_id);

	val = WIN_CHMAP_F(1, ch_type);
	mask = WIN_CHMAP_MASK(1);
	winctrl_write_mask(to_regwin_idx(decon_idx, win_idx),
			   DATA_PATH_CONTROL_WIN, val, mask);
}

/*
 * argb_color : 32-bit
 * A[31:24] - R[23:16] - G[15:8] - B[7:0]
 */
static void decon_reg_set_window_colormap(u32 decon_idx, u32 win_idx,
					  u32 argb_color)
{
	u32 val, mask;
	u32 mc_alpha = 0, mc_red = 0;
	u32 mc_green = 0, mc_blue = 0;

	mc_alpha = (argb_color >> 24) & 0xFF;
	mc_red = (argb_color >> 16) & 0xFF;
	mc_green = (argb_color >> 8) & 0xFF;
	mc_blue = (argb_color >> 0) & 0xFF;

	val = WIN_MAPCOLOR_A_F(mc_alpha) | WIN_MAPCOLOR_R_F(mc_red);
	mask = WIN_MAPCOLOR_A_MASK | WIN_MAPCOLOR_R_MASK;
	win_write_mask(to_regwin_idx(decon_idx, win_idx), WIN_COLORMAP_0, val,
		       mask);

	val = WIN_MAPCOLOR_G_F(mc_green) | WIN_MAPCOLOR_B_F(mc_blue);
	mask = WIN_MAPCOLOR_G_MASK | WIN_MAPCOLOR_B_MASK;
	win_write_mask(to_regwin_idx(decon_idx, win_idx), WIN_COLORMAP_1, val,
		       mask);
}

/* ALPHA_MULT selection used in (a',b',c',d') coefficient */
static void decon_reg_set_win_alpha_mult(u32 decon_idx, u32 win_idx,
					 enum decon_blend_alpha_mult_src a_sel)
{
	u32 val, mask;

	val = WIN_ALPHA_MULT_SRC_SEL_F(a_sel);
	mask = WIN_ALPHA_MULT_SRC_SEL_MASK;
	win_write_mask(to_regwin_idx(decon_idx, win_idx), WIN_CONTROL_0, val,
		       mask);
}

static void decon_reg_set_win_func(u32 decon_idx, u32 win_idx,
				   enum decon_blend_pd_func pd_func)
{
	u32 val, mask;

	val = WIN_FUNC_F(pd_func);
	mask = WIN_FUNC_MASK;
	win_write_mask(to_regwin_idx(decon_idx, win_idx), WIN_CONTROL_0, val,
		       mask);
}

static void decon_reg_set_win_sub_coeff(u32 decon_idx, u32 win_idx, u32 fgd,
					u32 bgd, u32 fga, u32 bga)
{
	u32 val, mask;

	/*
	 * [ Blending Equation ]
	 * Color : Cr = (a x Cf) + (b x Cb)  <Cf=FG pxl_C, Cb=BG pxl_C>
	 * Alpha : Ar = (c x Af) + (d x Ab)  <Af=FG pxl_A, Ab=BG pxl_A>
	 *
	 * [ User-defined ]
	 * a' = WINx_FG_ALPHA_D_SEL : Af' that is multiplied by FG Pixel Color
	 * b' = WINx_BG_ALPHA_D_SEL : Ab' that is multiplied by BG Pixel Color
	 * c' = WINx_FG_ALPHA_A_SEL : Af' that is multiplied by FG Pixel Alpha
	 * d' = WINx_BG_ALPHA_A_SEL : Ab' that is multiplied by BG Pixel Alpha
	 */

	val = (WIN_FG_ALPHA_D_SEL_F(fgd) | WIN_BG_ALPHA_D_SEL_F(bgd) |
	       WIN_FG_ALPHA_A_SEL_F(fga) | WIN_BG_ALPHA_A_SEL_F(bga));
	mask = (WIN_FG_ALPHA_D_SEL_MASK | WIN_BG_ALPHA_D_SEL_MASK |
		WIN_FG_ALPHA_A_SEL_MASK | WIN_BG_ALPHA_A_SEL_MASK);
	win_write_mask(to_regwin_idx(decon_idx, win_idx), WIN_CONTROL_1, val,
		       mask);
}

static void decon_reg_set_win_plane_alpha(u32 decon_idx, u32 win_idx, u32 a0,
					  u32 a1)
{
	u32 val, mask;

	val = WIN_ALPHA1_F(a1) | WIN_ALPHA0_F(a0);
	mask = WIN_ALPHA1_MASK | WIN_ALPHA0_MASK;
	win_write_mask(to_regwin_idx(decon_idx, win_idx), WIN_CONTROL_0, val,
		       mask);
}

static void decon_reg_set_win_blend_config(u32 decon_idx, u32 win_idx,
					   struct decon_win_config *config)
{
	enum decon_blend_pd_func pd_func = PD_FUNC_USER_DEFINED;
	u32 af_d = BND_COEF_ONE, ab_d = BND_COEF_ZERO, af_a = BND_COEF_ONE,
	    ab_a = BND_COEF_ZERO;

	switch (config->blend_mode) {
	case DECON_BLENDING_COVERAGE:
		af_d = BND_COEF_ALPHA_MULT;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_ALPHA_MULT;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
		break;
	case DECON_BLENDING_PREMULT:
		af_d = BND_COEF_PLANE_ALPHA0;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_PLANE_ALPHA0;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
		break;
	case DECON_BLENDING_NONE:
		pd_func = PD_FUNC_COPY;
		pr_debug("%s:%d none blending mode\n", __func__, __LINE__);
		break;
	default:
		pr_warn("%s:%d undefined blending mode\n", __func__, __LINE__);
		break;
	}

	/* We use 'ALPHA_MULT_SRC_SEL_AF' alpha mode only */
	decon_reg_set_win_alpha_mult(decon_idx, win_idx, ALPHA_MULT_SRC_SEL_AF);
	decon_reg_set_win_plane_alpha(decon_idx, win_idx, config->alpha, 0x00);
	decon_reg_set_win_func(decon_idx, win_idx, pd_func);
	if (pd_func == PD_FUNC_USER_DEFINED)
		decon_reg_set_win_sub_coeff(decon_idx, win_idx, af_d, ab_d,
					    af_a, ab_a);
}

static inline u32 win_start_pos(int x, int y)
{
	return (WIN_STRPTR_Y_F(y) | WIN_STRPTR_X_F(x));
}

static inline u32 win_end_pos(int x, int y, u32 w, u32 h)
{
	return (WIN_ENDPTR_Y_F(y + h - 1) | WIN_ENDPTR_X_F(x + w - 1));
}

static void decon_reg_set_window_enable_colormap(u32 decon_idx, u32 win_idx,
						 u32 en)
{
	u32 val, mask;

	val = en ? ~0 : 0;
	mask = WIN_MAPCOLOR_EN_F(0);
	winctrl_write_mask(to_regwin_idx(decon_idx, win_idx),
			   DATA_PATH_CONTROL_WIN, val, mask);
}

/* Decon window setting procedure
 *
 * 1. decon_reg_clear_window_update_req()
 * 2. decon_reg_set_window_config()
 * 3. decon_reg_set_window_enable()
 * 4. decon_reg_set_window_update_req()
 */

static void decon_reg_set_window_enable(u32 decon_idx, u32 win_idx, u32 dma_id)
{
	u32 mask;

	decon_reg_set_window_channel(decon_idx, win_idx, dma_id);
	mask = WIN_EN_F(0);
	winctrl_write_mask(to_regwin_idx(decon_idx, win_idx),
			   DATA_PATH_CONTROL_WIN, ~0, mask);
}

static void decon_reg_set_window_disable(u32 decon_idx, u32 win_idx)
{
	u32 mask;

	mask = WIN_EN_F(0);
	winctrl_write_mask(to_regwin_idx(decon_idx, win_idx),
			   DATA_PATH_CONTROL_WIN, 0, mask);
	decon_reg_set_window_channel(decon_idx, win_idx, IDMA_GF0);
}

static void decon_reg_set_window_config(u32 decon_idx, u32 win_idx,
					struct decon_win_config *config)
{
	u32 start_pos = win_start_pos(config->x, config->y);
	u32 end_pos = win_end_pos(config->x, config->y, config->w, config->h);

	win_write(to_regwin_idx(decon_idx, win_idx), WIN_START_POSITION,
		  start_pos);
	win_write(to_regwin_idx(decon_idx, win_idx), WIN_END_POSITION, end_pos);
	win_write(to_regwin_idx(decon_idx, win_idx), WIN_START_TIME_CONTROL,
		  config->start_time);

	decon_reg_set_window_enable_colormap(decon_idx, win_idx,
					     config->color_map.en);
	decon_reg_set_window_colormap(decon_idx, win_idx,
				      config->color_map.val);

	decon_reg_set_win_blend_config(decon_idx, win_idx, config);
}

static void decon_reg_clear_window_config(u32 decon_idx, u32 win_idx)
{
	win_write(to_regwin_idx(decon_idx, win_idx), WIN_CONTROL_0, 0);
	win_write(to_regwin_idx(decon_idx, win_idx), WIN_CONTROL_1, 0);

	win_write(to_regwin_idx(decon_idx, win_idx), WIN_START_POSITION, 0);
	win_write(to_regwin_idx(decon_idx, win_idx), WIN_END_POSITION, 0);
	win_write(to_regwin_idx(decon_idx, win_idx), WIN_START_TIME_CONTROL, 0);
}

static void decon_reg_set_window_update_req(u32 decon_idx, u32 win_idx)
{
	u32 mask;

	mask = SHD_UP_REQ;
	winctrl_write_mask(to_regwin_idx(decon_idx, win_idx), UPDATE_REQ_WIN,
			   ~0, mask);
}

static u32 decon_reg_get_window_status(u32 decon_idx, u32 win_idx)
{
	/* FIXME: Due to HW restrictions, cannot use to the shadow register
	 * of winctrl(WINx_EN_F bit) in auto9. As an alternative, use the
	 * WIN_END_POSITION value to check the status.
	 */
	return win_read(to_regwin_idx(decon_idx, win_idx),
			WIN_END_POSITION + WIN_SHD_OFFSET) ?
		       1 :
		       0;
}

static int decon_reg_reset(u32 decon_idx)
{
	int tries;

	decon_write_mask(decon_idx, GLOBAL_CONTROL, ~0, GLOBAL_CONTROL_SRESET);
	for (tries = 2000; tries; --tries) {
		if (~decon_read(decon_idx,
				GLOBAL_CONTROL & GLOBAL_CONTROL_SRESET))
			break;
		udelay(10);
	}

	if (!tries) {
		pr_err("failed to reset Decon\n");
		return -EBUSY;
	}

	return 0;
}

static void __decon_reg_set_enable(u32 decon_idx)
{
	u32 val, mask;

	val = ~0;
	mask = (GLOBAL_CONTROL_DECON_EN | GLOBAL_CONTROL_DECON_EN_F);
	decon_write_mask(decon_idx, GLOBAL_CONTROL, val, mask);
}

static void __decon_reg_set_disable(u32 decon_idx)
{
	u32 val, mask;

	val = 0;
	mask = (GLOBAL_CONTROL_DECON_EN | GLOBAL_CONTROL_DECON_EN_F);
	decon_write_mask(decon_idx, GLOBAL_CONTROL, val, mask);
}

static void decon_reg_set_disable_per_frame(u32 decon_idx)
{
	decon_write_mask(decon_idx, GLOBAL_CONTROL, 0,
			 GLOBAL_CONTROL_DECON_EN_F);
}

static void decon_reg_set_operation_mode(u32 decon_idx, enum decon_op_mode mode)
{
	u32 val, mask;

	mask = GLOBAL_CONTROL_OPERATION_MODE_F;

	if (mode == DECON_MIPI_COMMAND_MODE)
		val = GLOBAL_CONTROL_OPERATION_MODE_CMD_F;
	else
		val = GLOBAL_CONTROL_OPERATION_MODE_VIDEO_F;

	decon_write_mask(decon_idx, GLOBAL_CONTROL, val, mask);
}

static void decon_reg_set_blender_bg_image_size(u32 decon_idx, u32 width,
						u32 height)
{
	u32 val, mask;

	val = BLENDER_BG_HEIGHT_F(height) | BLENDER_BG_WIDTH_F(width);
	mask = BLENDER_BG_HEIGHT_MASK | BLENDER_BG_WIDTH_MASK;
	decon_write_mask(decon_idx, BLENDER_BG_IMAGE_SIZE_0, val, mask);
}

static u32 decon_reg_get_run_status(u32 decon_idx)
{
	u32 val;

	val = decon_read(decon_idx, GLOBAL_CONTROL);
	if (val & GLOBAL_CONTROL_RUN_STATUS)
		return 1;

	return 0;
}

static int decon_reg_wait_run_status_timeout(u32 decon_idx,
					     unsigned long timeout)
{
	u32 val;
	int ret;

	ret = read_poll_timeout_atomic(decon_reg_get_run_status, val, val, 100,
				       timeout, false, decon_idx);
	if (ret)
		pr_err("decon%u wait timeout decon run status(%u)\n", decon_idx,
		       val);

	return ret;
}

/* Determine that DECON is perfectly shuttled off through
 * checking this function
 */
static int decon_reg_wait_run_is_off_timeout(u32 decon_idx,
					     unsigned long timeout)
{
	u32 val;
	int ret;

	ret = read_poll_timeout_atomic(decon_reg_get_run_status, val, !val, 100,
				       timeout, false, decon_idx);
	if (ret)
		pr_err("decon%u wait timeout decon run is shut-off(%u)\n",
		       decon_idx, val);

	return ret;
}

/*
 * API is considering real possible Display Scenario
 * such as following examples
 *  < Single display >
 *  < Dual/Triple display >
 *  < Dual display + DP >
 *
 * Modify/add configuration cases if necessary
 * "Resource Confliction" will happen if enabled simultaneously
 *
 * it's in EVT1
 * Total of SRAM = 28k (4k * 7ea)
 *  DECON0 + DECON1 <= under 28k
 * The SRAM should be combinated in accordance with the scenario
 * without "resource conflicts"
 *
 * DECON0 : ~5k (DSC supported dual DSI and dual DSC)
 * DECON1 : ~3k (not supported dual DSI and      DSC)
 *
 *   OF0    OF1    CAPA
 *  SRAM0  SRAM0    2K (Shared)
 *  SRAM1  SRAM4    4K
 *  SRAM2  SRAM5    4K
 *  SRAM3  SRAM6    4K
 */
static void decon_reg_set_sram_share(u32 decon_idx,
				     enum decon_fifo_mode fifo_mode)
{
	u32 val = 0;
	u32 id = IS_DECON0(decon_idx) ? 0 : 1;

	switch (fifo_mode) {
	case DECON_FIFO_04K:
		if (id == 0)
			val = SRAM0_SHARE_ENABLE_F;
		else
			val = SRAM1_SHARE_ENABLE_F;
		break;
	case DECON_FIFO_08K:
		if (id == 0)
			val = SRAM1_SHARE_ENABLE_F | SRAM4_SHARE_ENABLE_F;
		else
			val = SRAM2_SHARE_ENABLE_F | SRAM5_SHARE_ENABLE_F;
		break;
	case DECON_FIFO_12K:
		if (id == 0)
			val = SRAM1_SHARE_ENABLE_F | SRAM4_SHARE_ENABLE_F |
			      SRAM0_SHARE_ENABLE_F;
		else
			val = SRAM3_SHARE_ENABLE_F | SRAM6_SHARE_ENABLE_F |
			      SRAM0_SHARE_ENABLE_F;
		break;
	case DECON_FIFO_16K:
		if (id == 0)
			val = SRAM1_SHARE_ENABLE_F | SRAM4_SHARE_ENABLE_F |
			      SRAM2_SHARE_ENABLE_F | SRAM5_SHARE_ENABLE_F;
		else
			val = SRAM2_SHARE_ENABLE_F | SRAM5_SHARE_ENABLE_F |
			      SRAM3_SHARE_ENABLE_F | SRAM6_SHARE_ENABLE_F;
		break;
	case DECON_FIFO_20K:
		if (id == 0)
			val = SRAM1_SHARE_ENABLE_F | SRAM4_SHARE_ENABLE_F |
			      SRAM3_SHARE_ENABLE_F | SRAM6_SHARE_ENABLE_F |
			      SRAM0_SHARE_ENABLE_F;
		else
			val = SRAM2_SHARE_ENABLE_F | SRAM5_SHARE_ENABLE_F |
			      SRAM3_SHARE_ENABLE_F | SRAM6_SHARE_ENABLE_F |
			      SRAM0_SHARE_ENABLE_F;
		break;
	case DECON_FIFO_24K:
		if (id == 0)
			val = SRAM0_SHARE_ENABLE_F | SRAM4_SHARE_ENABLE_F |
			      SRAM2_SHARE_ENABLE_F | SRAM5_SHARE_ENABLE_F |
			      SRAM3_SHARE_ENABLE_F | SRAM6_SHARE_ENABLE_F;
		else
			val = SRAM1_SHARE_ENABLE_F | SRAM4_SHARE_ENABLE_F |
			      SRAM2_SHARE_ENABLE_F | SRAM5_SHARE_ENABLE_F |
			      SRAM3_SHARE_ENABLE_F | SRAM6_SHARE_ENABLE_F;
		break;
	case DECON_FIFO_28K:
		val = ALL_SRAM_SHARE_ENABLE;
		break;
	case DECON_FIFO_00K:
		val = ALL_SRAM_SHARE_DISABLE;
		break;
	default:
		break;
	}

	decon_write(decon_idx, SRAM_SHARE_ENABLE, val);
}

static void decon_reg_set_outfifo_size_ctl0(u32 decon_idx, u32 width,
					    u32 height)
{
	u32 val;
	u32 th, mask;

	/* OUTFIFO_0 */
	val = OUTFIFO_HEIGHT_F(height) | OUTFIFO_WIDTH_F(width);
	mask = OUTFIFO_HEIGHT_MASK | OUTFIFO_WIDTH_MASK;
	decon_write(decon_idx, OUTFIFO_SIZE_CONTROL_0, val);

	/* may be implemented later by considering 1/2H transfer */
	th = OUTFIFO_TH_1H_F; /* 1H transfer */
	mask = OUTFIFO_TH_MASK;
	decon_write_mask(decon_idx, OUTFIFO_TH_CONTROL_0, th, mask);
}

static void decon_reg_set_outfifo_rgb_order(u32 decon_idx,
					    enum decon_rgb_order order)
{
	u32 val, mask;

	val = OUTFIFO_PIXEL_ORDER_SWAP_F(order);
	mask = OUTFIFO_PIXEL_ORDER_SWAP_MASK;
	decon_write_mask(decon_idx, OUTFIFO_DATA_ORDER_CONTROL, val, mask);
}

static void decon_reg_clear_interrupt_all(u32 decon_idx)
{
	u32 mask;

	mask = (DPU_FRAME_DONE_INT_EN | DPU_FRAME_START_INT_EN);
	decon_write_mask(decon_idx, INTERRUPT_PENDING, ~0, mask);

	mask = (DPU_RESOURCE_CONFLICT_INT_EN | DPU_TIME_OUT_INT_EN);
	decon_write_mask(decon_idx, EXTRA_INTERRUPT_PENDING, ~0, mask);
}

static void decon_reg_set_dsc_path(u32 decon_idx, enum decon_dsc_path dpath)
{
	u32 mask;

	mask = DSC_PATH_MASK;

	decon_write_mask(decon_idx, DATA_PATH_CONTROL_2, DSC_PATH_F(dpath),
			 mask);
}

static void decon_reg_set_outif_path(u32 decon_idx, enum decon_outif_path opath)
{
	u32 mask;

	mask = OUTIF_PATH_MASK;

	decon_write_mask(decon_idx, DATA_PATH_CONTROL_2, OUTIF_PATH_F(opath),
			 mask);
}

/*  <SEL_ATB relation between DECON and DPTX>
DPUM ATB0 DP0_SST1
     ATB1 DP0_SST2
     ATB2 X(non wired)
     ATB3 X(non wired)
DPUS0 ATB0 DP0_SST3
      ATB1 DP0_SST4
      ATB2 DP1_SST1
      ATB3 DP1_SST2
DPUS1 ATB0 DP1_SST3
      ATB1 DP1_SST4
      ATB2 X(non wired)
      ATB3 X(non wired) */
static u32 decon_reg_get_sel_atb(u32 decon_idx, enum decon_out_type out_type)
{
	u32 sel_atb =
		0x4; /* The ATB reset value is 0x4.(DP does not drive any ATB path) */

	switch (out_type) {
	case DECON_OUT_DP0_SST1:
	case DECON_OUT_DP0_SST3:
	case DECON_OUT_DP1_SST3:
		sel_atb = 0x0;
		break;
	case DECON_OUT_DP0_SST2:
	case DECON_OUT_DP0_SST4:
	case DECON_OUT_DP1_SST4:
		sel_atb = 0x1;
		break;
	case DECON_OUT_DP1_SST1:
		sel_atb = 0x2;
		break;
	case DECON_OUT_DP1_SST2:
		sel_atb = 0x3;
		break;
	default:
		pr_err("decon %u, invalid out_type = %#x\n", decon_idx,
		       out_type);
		break;
	}

	return DP_CONNECTION_SEL_ATB_F(sel_atb);
}

static void decon_reg_set_output(u32 decon_idx, enum decon_out_type out_type)
{
	u32 con_ctrl, out_if, sel_if, sel_atb = 0x4; /* IF: Interface */

	if (out_type & DECON_OUT_DSI) {
		if (decon_idx > DPUM_DECON1)
			return;

		if (out_type & DECON_OUT_DSI0) {
			if (IS_DECON0(decon_idx)) {
				/*
				 * DSIM can be connected with any DSIMIF.
				 * Below two combinations are possible.
				 * 1. OUT0 + OUTFIFO0_DSIMIF0
				 * 2. OUT1 + OUTFIFO0_DSIMIF1
				 */
				out_if = DECON0_OUTFIFO0_DSIMIF0;
				sel_if = DECON0_OUT0_TO_DSIM0;
			} else {
				out_if = DECON1_OUTFIFO0_DSIMIF;
				sel_if = DECON1_OUT0_TO_DSIM0;
			}
			con_ctrl = DSIM0_CONNECTION_CONTROL;

			decon_reg_set_outif_path(decon_idx, out_if);
			decon_con_write_mask(decon_idx, con_ctrl, sel_if,
					     DSIM_CONNECTION_DSIM_MASK);
		}
		if (out_type & DECON_OUT_DSI1) {
			if (IS_DECON0(decon_idx)) {
				/*
				 * DSIM can be connected with any DSIMIF.
				 * Below two combinations are possible.
				 * 1. OUT0 + OUTFIFO0_DSIMIF0
				 * 2. OUT1 + OUTFIFO0_DSIMIF1
				 */
				out_if = DECON0_OUTFIFO0_DSIMIF1;
				sel_if = DECON0_OUT1_TO_DSIM1;
			} else {
				out_if = DECON1_OUTFIFO0_DSIMIF;
				sel_if = DECON1_OUT0_TO_DSIM1;
			}
			con_ctrl = DSIM1_CONNECTION_CONTROL;

			decon_reg_set_outif_path(decon_idx, out_if);
			decon_con_write_mask(decon_idx, con_ctrl, sel_if,
					     DSIM_CONNECTION_DSIM_MASK);
		}
	}

	if (out_type & DECON_OUT_DP) {
		sel_atb = decon_reg_get_sel_atb(decon_idx, out_type);

		if (IS_DECON0(decon_idx)) {
			out_if = DECON0_OUTFIFO0_DPIF;
			sel_if = DP_CONNECTION_SEL_F(DECON0_OUT0_TO_DP) |
				 sel_atb;
			con_ctrl = DP0_CONNECTION_CONTROL_0;
		} else {
			out_if = DECON1_OUTFIFO0_DPIF;
			sel_if = DP_CONNECTION_SEL_F(DECON1_OUT0_TO_DP) |
				 sel_atb;
			con_ctrl = DP1_CONNECTION_CONTROL_0;
		}

		decon_reg_set_outif_path(decon_idx, out_if);
		decon_con_write_mask(decon_idx, con_ctrl, sel_if,
				     DP_CONNECTION_SEL_MASK |
					     DP_CONNECTION_SEL_ATB_MASK);
	}
}

static void decon_reg_set_te(u32 decon_idx, struct decon_mode mode,
			     enum decon_set_trig trig)
{
	u32 val, mask;

	if (mode.op_mode == DECON_VIDEO_MODE)
		return;

	if (mode.te_mode == DECON_SW_TRIG) {
		val = (trig == DECON_TRIG_UNMASK) ? SW_TRIG_EN : 0;
		mask = HW_TRIG_EN | SW_TRIG_EN;
	} else { /* DECON_HW_TRIG */
		val = (trig == DECON_TRIG_UNMASK) ? HW_TRIG_EN :
						    HW_TRIG_MASK_DECON;
		mask = HW_TRIG_EN | HW_TRIG_MASK_DECON;
	}

	decon_write_mask(decon_idx, HW_SW_TRIG_CONTROL, val, mask);
}

static void decon_reg_set_update_req_global(u32 decon_idx)
{
	decon_global_write_mask(decon_idx, SHADOW_REG_UPDATE_REQ, ~0,
				SHADOW_REG_UPDATE_REQ_GLOBAL);
}

static void decon_reg_set_interrupt(u32 decon_idx, u32 en)
{
	u32 val, mask;

	decon_reg_clear_interrupt_all(decon_idx);

	if (en) {
		val = (DPU_FRAME_DONE_INT_EN | DPU_FRAME_START_INT_EN |
		       DPU_EXTRA_INT_EN | DPU_INT_EN);

		decon_write_mask(decon_idx, INTERRUPT_ENABLE, val,
				 INTERRUPT_ENABLE_MASK);
		pr_debug("decon %u, interrupt val = %x\n", decon_idx, val);

		val = (DPU_RESOURCE_CONFLICT_INT_EN | DPU_TIME_OUT_INT_EN);
		decon_write(decon_idx, EXTRA_INTERRUPT_ENABLE, val);
	} else {
		mask = (DPU_EXTRA_INT_EN | DPU_INT_EN);
		decon_write_mask(decon_idx, INTERRUPT_ENABLE, 0, mask);
	}
}

static u32 decon_reg_clear_extra_interrupt(u32 decon_idx)
{
	u32 reg = EXTRA_INTERRUPT_PENDING;
	u32 val = decon_read(decon_idx, reg);

	if (val & DPU_RESOURCE_CONFLICT_INT_PEND) {
		decon_write(decon_idx, reg, DPU_RESOURCE_CONFLICT_INT_PEND);
		pr_warn("decon%u INFO0: SRAM_RSC & DSC = 0x%x\n", decon_idx,
			decon_read(decon_idx, RESOURCE_OCCUPANCY_INFO_0));
		pr_warn("decon%u INFO1: DMA_CH_RSC= 0x%x\n", decon_idx,
			decon_read(decon_idx, RESOURCE_OCCUPANCY_INFO_1));
		pr_warn("decon%u INFO2: WIN_RSC= 0x%x\n", decon_idx,
			decon_read(decon_idx, RESOURCE_OCCUPANCY_INFO_2));
	}

	/*
	 * Timeout interrupt is occurred when aclk counter is over
	 * 'DECONx_TIME_OUT_VALUE'.
	 * This counter value is reset by FRAME_DONE signal
	 */
	if (val & DPU_TIME_OUT_INT_PEND) {
		decon_write(decon_idx, reg, DPU_TIME_OUT_INT_PEND);
		pr_warn("decon%u time out interrupt occured!\n", decon_idx);
		pr_warn("decon%u maybe stuck now!\n", decon_idx);
	}

	return val;
}

static u32 decon_reg_clear_interrupt(u32 decon_idx, enum decon_irq irq)
{
	u32 pending_val = decon_read(decon_idx, INTERRUPT_PENDING);

	if (irq == DECON_IRQ_FS && (pending_val & DPU_FRAME_START_INT_PEND)) {
		/* Clear Interrupt */
		decon_write(decon_idx, INTERRUPT_PENDING,
			    DPU_FRAME_START_INT_PEND);
	}

	if (irq == DECON_IRQ_FD && (pending_val & DPU_FRAME_DONE_INT_PEND)) {
		/* Clear Interrupt */
		decon_write(decon_idx, INTERRUPT_PENDING,
			    DPU_FRAME_DONE_INT_PEND);
	}

	if (irq == DECON_IRQ_EXT && (pending_val & DPU_EXTRA_INT_PEND)) {
		/* Clear Interrupt */
		decon_write(decon_idx, INTERRUPT_PENDING, DPU_EXTRA_INT_PEND);
		decon_reg_clear_extra_interrupt(decon_idx);
	}

	return pending_val;
}

#define SHADOW_UPDATE_TIMEOUT (50 * 1000)

static u32 decon_reg_get_window_update_req(u32 decon_idx, u32 win_idx)
{
	return winctrl_read_mask(to_regwin_idx(decon_idx, win_idx),
				 UPDATE_REQ_WIN, SHD_UP_REQ);
}

static u32 decon_reg_get_shadow_update_req(u32 decon_idx)
{
	return decon_global_read(decon_idx, SHADOW_REG_UPDATE_REQ);
}

static void decon_reg_set_bpc(u32 decon_idx, u32 bpc)
{
	u32 val;

	val = (bpc == 10) ? GLOBAL_CONTROL_TEN_BPC_MODE_F : 0;

	decon_write_mask(decon_idx, GLOBAL_CONTROL, val,
			 GLOBAL_CONTROL_TEN_BPC_MODE_MASK);
}

static int decon_reg_set_config(u32 decon_idx, struct decon_config config)
{
	enum decon_rgb_order rgb_order = DECON_RGB;

	/* OUTFIFO_PIXEL_ORDER_SWAP_F
	  * Use DECON_BGR at raw image transfer to DSIM.
	  * Use DECON_RGB at compressed stream tranfer to DSIM.
	  * Use DECON_RGB at DP connection.
	  *
	  * TODO : if DSC feature added, code must be modified like this.
	  * if((config->out_type & DECON_OUT_DSI) && (DSC==DISABLE))
	  * 	rgb_order = DECON_BGR;
	  * else
	  * 	rgb_order = DECON_RGB;
	 */
	if (config.out_type & DECON_OUT_DSI)
		rgb_order = DECON_BGR;

	decon_reg_set_operation_mode(decon_idx, config.mode.op_mode);
	decon_reg_set_blender_bg_image_size(decon_idx,
					    (config.mode.dsi_mode ==
					     DSI_MODE_DUAL_DSI) ?
						    (config.image_width * 2) :
						    config.image_width,
					    config.image_height);
	decon_reg_set_bpc(decon_idx, config.out_bpc);
	decon_reg_clear_interrupt_all(decon_idx);
	decon_reg_set_outfifo_size_ctl0(decon_idx, config.image_width,
					config.image_height);
	decon_reg_set_sram_share(decon_idx, DECON_FIFO_08K);

	decon_reg_set_outfifo_rgb_order(decon_idx, rgb_order);
	decon_reg_set_output(decon_idx, config.out_type);

	return 0;
}

static int decon_reg_set_enable(u32 decon_idx, struct decon_config config)
{
	int ret = 0;

	decon_reg_set_config(decon_idx, config);
	// if (config.dsc.enable)
	// 	decon_reg_set_dsc(decon_idx, config.dsc);
	// else
	decon_reg_set_dsc_path(decon_idx, DECON_DSC_NONE);
	__decon_reg_set_enable(decon_idx);
	decon_reg_set_update_req_global(decon_idx);
	decon_reg_wait_run_status_timeout(decon_idx, 20 * 1000);
	decon_reg_set_te(decon_idx, config.mode, DECON_TRIG_UNMASK);

	return ret;
}

static int decon_reg_set_disable(u32 decon_idx, struct decon_mode mode)
{
	int ret = 0;
	const int timeout_value = 1000 / 60 * 12 / 10 + 5;

	if (!decon_reg_get_run_status(decon_idx)) {
		pr_info("already IDLE status\n");
		return 0;
	}

	decon_reg_set_te(decon_idx, mode, DECON_TRIG_MASK);

	decon_reg_set_disable_per_frame(decon_idx);
	decon_reg_set_update_req_global(decon_idx);

	ret = decon_reg_wait_run_is_off_timeout(decon_idx,
						timeout_value * MSEC);
	if (!ret) {
		decon_reg_reset(decon_idx);
		return 0;
	} else {
		__decon_reg_set_disable(decon_idx);
		decon_reg_set_update_req_global(decon_idx);

		ret = decon_reg_wait_run_is_off_timeout(decon_idx,
							timeout_value * MSEC);
		decon_reg_clear_interrupt_all(decon_idx);
	}

	return ret;
}

static void decon_reg_enable_window(u32 decon_idx, u32 win_idx,
				    struct decon_win_config *config)
{
	if (!config) {
		pr_err("window%d config is NULL\n", win_idx);
		return;
	}

	decon_reg_set_window_config(decon_idx, win_idx, config);
	decon_reg_set_window_enable(decon_idx, win_idx, config->dpp_type);
}

static void decon_reg_disable_window(u32 decon_idx, u32 win_idx)
{
	decon_reg_clear_window_config(decon_idx, win_idx);
	decon_reg_set_window_disable(decon_idx, win_idx);
}

static int decon_reg_enable(u32 decon_idx, struct decon_config config)
{
	int ret = 0;

	ret = decon_reg_set_enable(decon_idx, config);
	if (ret)
		return ret;

	decon_reg_set_interrupt(decon_idx, 1);

	return ret;
}

static int decon_reg_disable(u32 decon_idx, struct decon_config config)
{
	int ret = 0;

	decon_reg_set_interrupt(decon_idx, 0);

	ret = decon_reg_set_disable(decon_idx, config.mode);
	if (ret)
		return ret;

	return ret;
}

struct decon_dev_data {
	const u32 nr_decon;
	const u32 nr_win;
	const struct decon_cal_ops *cal_ops;
};

static const struct decon_dev_data exynos910_decon = {
	.nr_decon = 2,
	.nr_win = 8,
};

static const struct of_device_id decon_driver_dt_match[] = {
	{ .compatible = "samsung,exynos910-decon",
	  .data = (void *)&exynos910_decon },
	{},
};

MODULE_DEVICE_TABLE(of, decon_driver_dt_match);

static struct decon_win *plane_to_decon_win(struct drm_plane *e)
{
	return container_of(to_exynos_plane(e), struct decon_win, plane);
}

/* ARGB value */
#define COLOR_MAP_VALUE 0x00ff0000

static void decon_set_win_color_map(struct decon_win *window, bool en)
{
	struct decon_win_config *config = &window->config;

	config->color_map.en = en;
	config->color_map.val = COLOR_MAP_VALUE;
}

static enum drm_mode_status
decon_mode_valid(struct exynos_drm_crtc *crtc,
		 const struct drm_display_mode *mode)
{
	struct decon_context *ctx = crtc->ctx;

	/* DECON use only video mode */
	if ((mode->hdisplay < DECON_VIDEO_MODE_WIDTH_MIN) ||
	    (mode->hdisplay > DECON_WIDTH_MAX) || (mode->hdisplay % 2)) {
		drm_err(ctx->drm_dev, "mode hdisplay(%d) is bad\n",
			mode->hdisplay);
		return MODE_BAD_HVALUE;
	}

	if ((mode->vdisplay < DECON_HEIGHT_MIN) ||
	    (mode->vdisplay > DECON_WIDTH_MAX)) {
		drm_err(ctx->drm_dev, "mode vdisplay(%d) is bad\n",
			mode->vdisplay);
		return MODE_BAD_VVALUE;
	}

	return MODE_OK;
}

static void decon_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	decon_reg_set_te(ctx->idx, ctx->config.mode, DECON_TRIG_MASK);
}

static enum decon_blend_mode to_decon_blend_mode(u16 drm_blend_mode)
{
	switch (drm_blend_mode) {
	case DRM_MODE_BLEND_PIXEL_NONE:
		return DECON_BLENDING_NONE;

	case DRM_MODE_BLEND_COVERAGE:
		return DECON_BLENDING_COVERAGE;

	case DRM_MODE_BLEND_PREMULTI:
	default:
		return DECON_BLENDING_PREMULT;
	}
}

static void decon_update_plane(struct exynos_drm_crtc *crtc,
			       struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
		to_exynos_plane_state(plane->base.state);
	struct decon_context *ctx = crtc->ctx;
	struct decon_win *window = plane_to_decon_win(&plane->base);
	struct decon_win_config *config = &window->config;
	struct exynos_drm_rect *rect = &state->crtc;
	struct drm_device *drm_dev = ctx->drm_dev;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	struct exynos_dpu_dma_context *dma_ctx = dev_get_drvdata(priv->dma_dev);

	ctx->plane_mask |= drm_plane_mask(&plane->base);

	config->x = state->crtc.x;
	config->y = state->crtc.y;
	config->w = state->crtc.w;
	config->h = state->crtc.h;
	config->start_time = 0;
	config->alpha = state->base.alpha >> 8;
	config->blend_mode = to_decon_blend_mode(state->base.pixel_blend_mode);
	config->dpp_type = window->dpp->type;

	decon_set_win_color_map(window, ctx->is_colormap);

	dpp_update(window->dpp, 0, state);
	dpu_dma_update(dma_ctx, 0, state);

	decon_reg_enable_window(ctx->idx, window->idx, config);

	drm_dbg(ctx->drm_dev, "WINDOW-%d(%s)(%4d, %4d, %4d, %4d)", window->idx,
		dev_name(window->dpp->dev), rect->x, rect->y, rect->w, rect->h);
}

static void decon_disable_plane(struct exynos_drm_crtc *crtc,
				struct exynos_drm_plane *plane)
{
	struct decon_context *ctx = crtc->ctx;
	struct decon_win *window = plane_to_decon_win(&plane->base);

	ctx->plane_mask |= drm_plane_mask(&plane->base);

	decon_reg_disable_window(ctx->idx, window->idx);

	ctx->disable_mask |= (1 << window->idx);

	drm_dbg(ctx->drm_dev, "WINDOW-%d(%s)\n", window->idx,
		dev_name(window->dpp->dev));
}

static void decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	struct drm_crtc_state *state = crtc->base.state;
	struct drm_plane *plane;
	bool req_global = true;

	synchronize_irq(ctx->irq_fd);

	drm_for_each_plane_mask(plane, ctx->drm_dev, ctx->plane_mask) {
		struct decon_win *window = plane_to_decon_win(plane);

		/* window update first to guarantee dma stop during dpp_disable */
		decon_reg_set_window_update_req(ctx->idx, window->idx);

		// if (ctx->disable_mask & (1 << window->idx))
		// 	dpp_disable(window->dpp);

		/* If at least one window is running, there is no need to set
		 * global update
		 */
		if (req_global &&
		    decon_reg_get_window_status(ctx->idx, window->idx))
			req_global = false;
	}
	ctx->disable_mask = 0;

	if (drm_atomic_crtc_needs_modeset(state) || req_global)
		decon_reg_set_update_req_global(ctx->idx);

	/* In case of fake vblank, it make vblank after 1 vsync time(16ms) */
	// if (ctx->fake_vblank)
	// 	drm_crtc_handle_vblank(&crtc->base);
	// schedule_delayed_work(&ctx->dwork, msecs_to_jiffies(16));

	decon_reg_set_te(ctx->idx, ctx->config.mode, DECON_TRIG_UNMASK);
	exynos_crtc_handle_event(crtc);

	drm_dbg(ctx->drm_dev, "flushed\n");
}

static void decon_config_print(struct decon_config *config)
{
	struct decon_context *ctx =
		container_of(config, struct decon_context, config);
	char *str_output = NULL;

	drm_dbg(ctx->drm_dev, "Operation Mode: %s\n",
		config->mode.op_mode ? "MIPI Command" : "Video");
	if (config->mode.op_mode == DECON_MIPI_COMMAND_MODE)
		drm_info(ctx->drm_dev, "Trigger Mode: %s\n",
			 config->mode.te_mode ? "SW" : "HW");

	if (config->out_type & DECON_OUT_DSI0)
		str_output = "DSI0";
	else if (config->out_type & DECON_OUT_DSI1)
		str_output = "DSI1";
	else if (config->out_type & DECON_OUT_DP0)
		str_output = "DP0";
	else if (config->out_type & DECON_OUT_DP1)
		str_output = "DP1";

	drm_info(ctx->drm_dev, "Output: %s\n", str_output);
}

static void decon_set_mode(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	struct decon_mode *mode = &(ctx->config.mode);
	enum decon_out_type out_type = 0;
	struct drm_encoder *encoder;

	drm_for_each_encoder_mask(encoder, crtc->base.dev,
				  crtc->base.state->encoder_mask)
		// TODO: make read port number and get it.
		out_type = DECON_OUT_DP0_SST1;

	if (out_type & DECON_OUT_DP)
		mode->op_mode = DECON_VIDEO_MODE;

	ctx->config.out_type = out_type;
}

static void decon_enable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	drm_display_mode_to_videomode(&crtc->base.mode, &ctx->v_mode);

	ctx->config.image_width = ctx->v_mode.hactive;
	ctx->config.image_height = ctx->v_mode.vactive;
	ctx->config.fps = drm_mode_vrefresh(&crtc->base.mode);

	decon_set_mode(crtc);

	decon_reg_enable(ctx->idx, ctx->config);

	enable_irq(ctx->irq_fd);

	decon_config_print(&ctx->config);
	drm_info(ctx->drm_dev, "enabled! crtc[%d] = %dx%d-%dHz (%d out_bpc)\n",
		 crtc->base.base.id, ctx->config.image_width,
		 ctx->config.image_height, ctx->config.fps,
		 ctx->config.out_bpc);
}

static void decon_disable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	disable_irq(ctx->irq_fd);
}

static const struct exynos_drm_crtc_ops decon_crtc_ops = {
	.atomic_enable = decon_enable,
	.atomic_disable = decon_disable,
	.mode_valid = decon_mode_valid,
	.atomic_begin = decon_atomic_begin,
	.update_plane = decon_update_plane,
	.disable_plane = decon_disable_plane,
	.atomic_flush = decon_atomic_flush,
};

/* H/W goes to stop or reset state when OS restarting during h/w operation */
static void decon_reset(struct decon_context *ctx, int rpm_req)
{
	int ret;
	struct decon_config config = { 0 };

	ret = decon_reg_disable(ctx->idx, config);
	if (ret) {
		drm_err(ctx->drm_dev, "failed to try job_abort\n");
	}
}

static enum drm_plane_type decon_get_win_type(int win_idx, int last_idx)
{
	if (win_idx == 0)
		return DRM_PLANE_TYPE_PRIMARY;
	else if (win_idx == last_idx)
		return DRM_PLANE_TYPE_CURSOR;
	else
		return DRM_PLANE_TYPE_OVERLAY;
};

static const u32 dpp_gf_formats[] = {
	DRM_FORMAT_ARGB8888,	DRM_FORMAT_ABGR8888,	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,	DRM_FORMAT_XRGB8888,	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,	DRM_FORMAT_BGRX8888,	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,	DRM_FORMAT_ARGB2101010, DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102, DRM_FORMAT_BGRA1010102,
};

static bool exynos_crtc_handle_vblank(struct exynos_drm_crtc *exynos_crtc)
{
	return drm_crtc_handle_vblank(&exynos_crtc->base);
}

static void exynos_crtc_delayed_vblank(struct work_struct *work)
{
	struct delayed_work *delayed_work =
		container_of(work, struct delayed_work, work);
	struct decon_context *ctx =
		container_of(delayed_work, struct decon_context, dwork);

	exynos_crtc_handle_vblank(ctx->crtc);
}

static int decon_bind(struct device *dev, struct device *master, void *data)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_plane *primary_plane = NULL;
	struct exynos_drm_private *priv = drm_dev->dev_private;

	int i, ret = 0;

	/* Release a pm_runtime opposite to xxx_reset */
	decon_reset(ctx, RPM_REQ_SUSPEND);

	ctx->drm_dev = drm_dev;

	for (i = 0; i < ctx->win_cnt; i++) {
		struct decon_win *win = &ctx->win[i];
		struct exynos_dpp_context *dpp = dev_get_drvdata(priv->dpp_dev);
		ctx->win[i].dpp = dpp;

		if (!dpp)
			continue;

		win->plane_config.pixel_formats = dpp_gf_formats;
		win->plane_config.num_pixel_formats = 14;
		win->plane_config.zpos = i;
		win->plane_config.type =
			decon_get_win_type(i, ctx->win_cnt - 1);

		ret = exynos_plane_init(drm_dev, &win->plane, i,
					&win->plane_config);
		if (ret)
			return ret;

		if (win->plane_config.type == DRM_PLANE_TYPE_PRIMARY)
			primary_plane = &win->plane.base;
		// if (win->plane_config.type == DRM_PLANE_TYPE_CURSOR)
		// 	cursor_plane = &win->plane.base;
	}

	ctx->crtc = exynos_drm_crtc_create(drm_dev, primary_plane, 0,
					   &decon_crtc_ops, ctx);

	INIT_DELAYED_WORK(&ctx->dwork, exynos_crtc_delayed_vblank);

	if (IS_ERR(ctx->crtc))
		return PTR_ERR(ctx->crtc);

	//encoder, connector init. not this way
	ctx->crtc->base.port = of_graph_get_port_by_id(dev->of_node, 0);

	return 0;
}

static void decon_unbind(struct device *dev, struct device *master, void *data)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	decon_disable(ctx->crtc);
}

static const struct component_ops decon_component_ops = {
	.bind = decon_bind,
	.unbind = decon_unbind,
};

static bool decon_get_window_update_req(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	struct drm_plane *plane;

	drm_for_each_plane_mask(plane, ctx->drm_dev, ctx->plane_mask) {
		u32 win_idx = plane_to_decon_win(plane)->idx;

		if (decon_reg_get_window_update_req(ctx->idx, win_idx))
			return true;
	}

	return false;
}

static irqreturn_t decon_irq_handler(int irq, void *dev_id)
{
	struct decon_context *ctx = dev_id;

	// decon_reg_clear_interrupt(ctx->idx, DECON_IRQ_FS);
	decon_reg_clear_interrupt(ctx->idx, DECON_IRQ_FD);

	decon_reg_get_shadow_update_req(ctx->idx);

	if (ctx->config.mode.op_mode == DECON_VIDEO_MODE ||
	    !decon_get_window_update_req(ctx->crtc)) {
		drm_crtc_handle_vblank(&ctx->crtc->base);
	}

	return IRQ_HANDLED;
}

static int decon_probe(struct platform_device *pdev)
{
	int ret;
	struct decon_context *ctx;
	struct device *dev = &pdev->dev;
	struct resource *res;

	ctx = devm_kzalloc(dev, sizeof(struct decon_context), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->aclk = devm_clk_get_enabled(dev, "aclk");
	if (IS_ERR(ctx->aclk))
		return dev_err_probe(dev, PTR_ERR(ctx->aclk),
				     "Cannot get aclk\n");

	ctx->irq_fd = platform_get_irq(pdev, 0);
	irq_set_status_flags(ctx->irq_fd, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, ctx->irq_fd, decon_irq_handler,
			       IRQF_ONESHOT, dev_name(dev), ctx);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register interrupt handler\n");

	ctx->type = 0;
	ctx->win_max = 8;

	ctx->win_cnt = 1;

	ctx->win = devm_kzalloc(ctx->dev, sizeof(struct decon_win) * 8,
				GFP_KERNEL);
	if (!ctx->win)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "main");
	ctx->regs[0] = devm_ioremap_resource(dev, res);
	regs_win[0] = ctx->regs[0];

	regs_decon[0] = ctx->regs[0] + 0x8000;
	regs_decon_con[0] = ctx->regs[0] + 0xc000;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sub0");
	ctx->regs[1] = devm_ioremap_resource(dev, res);
	regs_decon_global[0] = ctx->regs[1] + 0xa000;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sub1");
	ctx->regs[2] = devm_ioremap_resource(dev, res);
	regs_winctrl[0] = ctx->regs[2];

	platform_set_drvdata(pdev, ctx);

	return component_add(dev, &decon_component_ops);
}

struct platform_driver decon_driver = {
	.probe		= decon_probe,
	.driver		= {
		.name	= "exynos9-decon",
		.of_match_table = decon_driver_dt_match,
	},
};
