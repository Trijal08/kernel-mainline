// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) Samsung Electronics Co., Ltd.
 * Copyright (c) Google LLC
 *
 * zuma/zumapro (Google Tensor, gs201-class) DECON low-level back-end.
 *
 * Implements the decon_cal_ops vtable declared in exynos9_decon.h for the
 * Tensor "zuma" DECON register topology. The sequences are ported from the
 * vendor cal_9865 DECON code (decon_reg.c); the register map lives in
 * regs-decon-zuma.h. This TU is deliberately separate from exynos9_decon.c
 * because the exynos910 and zuma register headers collide by macro name.
 *
 * Scope: single active DECON, single primary window, DSI (video or command)
 * or DP output - enough to match the exynos7 DECON feature bar. DSC, CWB,
 * DQE, partial update, PLL-sleep and EWR are intentionally not ported.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

#include "exynos9_decon.h"
#include "regs-decon-zuma.h"

#define ZD_MAX_DECON		3

/*
 * DSC configuration for the komodo (google,gs-km4) panel: dual DSC, two
 * 672x34 slices, 8.0 bpp, taken verbatim from the vendor panel's
 * drm_dsc_config (wqhd_pps_config). The bootloader programs the panel's DSC
 * decoder from these same parameters, so the DECON encoder PPS must match.
 *
 * This is hardcoded for the single zuma DSI panel wired today (komodo). The
 * mainline-clean path is for the panel/connector to advertise its
 * drm_dsc_config and the DECON to read it; that is a follow-up.
 */
static const struct drm_dsc_config komodo_dsc_cfg = {
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_width = 672,
	.slice_height = 34,
	.slice_count = 2,
	.simple_422 = false,
	.pic_width = 1344,
	.pic_height = 2992,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.initial_dec_delay = 592,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126
	},
	.rc_range_params = {
		{ 0, 4, 2 }, { 0, 4, 0 }, { 1, 5, 0 }, { 1, 6, 62 },
		{ 3, 7, 60 }, { 3, 7, 58 }, { 3, 7, 56 }, { 3, 8, 56 },
		{ 3, 9, 56 }, { 3, 10, 54 }, { 5, 11, 54 }, { 5, 12, 52 },
		{ 5, 13, 52 }, { 7, 13, 52 }, { 13, 15, 52 }
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 9,
	.scale_increment_interval = 932,
	.nfl_bpg_offset = 745,
	.slice_bpg_offset = 616,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 672,
	.dsc_version_minor = 2,
	.dsc_version_major = 1,
};

/* Porter-Duff window functions (WIN_FUNC_CON_0.WIN_FUNC_F) */
enum {
	ZD_PD_FUNC_COPY		= 0x1,
	ZD_PD_FUNC_USER_DEFINED	= 0xd,
};

/* blend sub-coefficients (WIN_FUNC_CON_1) */
enum {
	ZD_BND_COEF_ZERO		= 0x0,
	ZD_BND_COEF_ONE			= 0x1,
	ZD_BND_COEF_PLANE_ALPHA0	= 0x6,
	ZD_BND_COEF_ALPHA_MULT		= 0xA,
	ZD_BND_COEF_1_M_ALPHA_MULT	= 0xB,
};

#define ZD_ALPHA_MULT_SRC_SEL_AF	0x2

/* OF_PIXEL_ORDER swap values */
#define ZD_DECON_RGB			0x0
#define ZD_DECON_BGR			0x4

/*
 * SFR block bases, per DECON instance. Each DECON is its own platform_device
 * with its own "main"/"win"/"sub"/"wincon" reg regions, so the per-instance
 * offset is already folded into the mapped base and only the per-window
 * stride (ZD_WIN_OFFSET) is applied on top.
 */
static void __iomem *zd_main[ZD_MAX_DECON];
static void __iomem *zd_win[ZD_MAX_DECON];
static void __iomem *zd_sub[ZD_MAX_DECON];
static void __iomem *zd_wincon[ZD_MAX_DECON];

static inline u32 zd_main_read(u32 id, u32 off)
{
	return readl(zd_main[id] + off);
}

static inline void zd_main_write(u32 id, u32 off, u32 val)
{
	writel(val, zd_main[id] + off);
}

static inline void zd_main_write_mask(u32 id, u32 off, u32 val, u32 mask)
{
	u32 old = zd_main_read(id, off);

	writel((val & mask) | (old & ~mask), zd_main[id] + off);
}

static inline u32 zd_win_read(u32 id, u32 off)
{
	return readl(zd_win[id] + off);
}

static inline void zd_win_write(u32 id, u32 off, u32 val)
{
	writel(val, zd_win[id] + off);
}

static inline void zd_win_write_mask(u32 id, u32 off, u32 val, u32 mask)
{
	u32 old = zd_win_read(id, off);

	writel((val & mask) | (old & ~mask), zd_win[id] + off);
}

static inline void zd_sub_write(u32 id, u32 off, u32 val)
{
	writel(val, zd_sub[id] + off);
}

static inline void zd_wincon_write_mask(u32 id, u32 off, u32 val, u32 mask)
{
	u32 old = readl(zd_wincon[id] + off);

	writel((val & mask) | (old & ~mask), zd_wincon[id] + off);
}

/* ------------------------------------------------------------------ */
/* Global control							      */
/* ------------------------------------------------------------------ */

static int zuma_reg_reset(u32 id)
{
	u32 val;
	int ret;

	zd_main_write_mask(id, ZD_GLOBAL_CON, ~0, ZD_GLOBAL_CON_SRESET);
	ret = readl_poll_timeout_atomic(zd_main[id] + ZD_GLOBAL_CON, val,
					!(val & ZD_GLOBAL_CON_SRESET), 10, 2000);
	if (ret)
		pr_err("decon%u: failed to reset\n", id);

	return ret;
}

static void zuma_reg_set_operation_mode(u32 id, enum decon_op_mode mode)
{
	u32 val = (mode == DECON_MIPI_COMMAND_MODE) ?
			  ZD_GLOBAL_CON_OPERATION_MODE_CMD_F :
			  ZD_GLOBAL_CON_OPERATION_MODE_VIDEO_F;

	zd_main_write_mask(id, ZD_GLOBAL_CON, val,
			   ZD_GLOBAL_CON_OPERATION_MODE_F);
}

static void zuma_reg_direct_on_off(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	zd_main_write_mask(id, ZD_GLOBAL_CON, val,
			   ZD_GLOBAL_CON_DECON_EN | ZD_GLOBAL_CON_DECON_EN_F);
}

static void zuma_reg_per_frame_off(u32 id)
{
	zd_main_write_mask(id, ZD_GLOBAL_CON, 0, ZD_GLOBAL_CON_DECON_EN_F);
}

static bool zuma_reg_get_run_status(u32 id)
{
	return !!(zd_main_read(id, ZD_GLOBAL_CON) & ZD_GLOBAL_CON_RUN_STATUS);
}

static int zuma_reg_wait_run_status(u32 id, unsigned long timeout_us)
{
	u32 val;

	return readl_poll_timeout_atomic(zd_main[id] + ZD_GLOBAL_CON, val,
					 (val & ZD_GLOBAL_CON_RUN_STATUS), 10,
					 timeout_us);
}

static int zuma_reg_wait_run_is_off(u32 id, unsigned long timeout_us)
{
	u32 val;

	return readl_poll_timeout_atomic(zd_main[id] + ZD_GLOBAL_CON, val,
					 !(val & ZD_GLOBAL_CON_RUN_STATUS), 10,
					 timeout_us);
}

/* clock gating is left disabled during bring-up */
static void zuma_reg_set_clkgate_mode(u32 id, bool en)
{
	u32 val = en ? ~0 : 0;

	zd_main_write_mask(id, ZD_CLOCK_CON(id), val,
			   ZD_CLOCK_CON_AUTO_CG_MASK | ZD_CLOCK_CON_QACTIVE_MASK);
}

static void zuma_reg_set_bpc(u32 id, u32 bpc)
{
	u32 val = (bpc == 10) ? ZD_GLOBAL_CON_TEN_BPC_MODE_F : 0;

	zd_main_write_mask(id, ZD_GLOBAL_CON, val,
			   ZD_GLOBAL_CON_TEN_BPC_MODE_MASK);
}

/* ------------------------------------------------------------------ */
/* SRAM sharing							      */
/* ------------------------------------------------------------------ */

/*
 * Static SRAM assignment for bring-up (vendor: decon0 owns 11 instances,
 * decon1 the next 11, decon2 shares the rest dynamically). Only the primary
 * path is programmed; the secondary (CWB) path is left disabled.
 */
static const u64 zd_pri_sram[ZD_MAX_DECON] = {
	GENMASK_ULL(10, 0),
	GENMASK_ULL(21, 11),
	0,
};

static void zuma_reg_set_sram_enable(u32 id)
{
	int i, j;

	for (j = 0; j < ZD_SRAM_EN_OF_PRI_REG_CNT; j++) {
		u32 val = 0;

		for (i = 0; i < ZD_SRAM_EN_CNT; i++) {
			if (zd_pri_sram[id] & BIT_ULL(8 * j + i))
				val |= ZD_SRAM_EN_ID(i);
		}
		zd_main_write(id, ZD_SRAM_EN_OF_PRI(j), val);
		zd_main_write(id, ZD_SRAM_EN_OF_SEC(j), 0);
	}

	if (zd_pri_sram[id] & BIT_ULL(32))
		zd_main_write(id, ZD_SRAM_EN_OF_PRI_4, ZD_SRAM32_EN_F);
}

/* ------------------------------------------------------------------ */
/* OUTFIFO / blender / data path					      */
/* ------------------------------------------------------------------ */

static void zuma_reg_set_outfifo_size(u32 id, u32 width, u32 height)
{
	zd_main_write(id, ZD_OF_SIZE_0,
		      ZD_OUTFIFO_HEIGHT_F(height) | ZD_OUTFIFO_WIDTH_F(width));
	zd_main_write_mask(id, ZD_OF_TH_TYPE, ZD_OUTFIFO_TH_1H_F,
			   ZD_OUTFIFO_TH_MASK);
}

static void zuma_reg_set_rgb_order(u32 id, u32 order)
{
	zd_main_write_mask(id, ZD_OF_PIXEL_ORDER,
			   ZD_OUTFIFO_PIXEL_ORDER_SWAP_F(order),
			   ZD_OUTFIFO_PIXEL_ORDER_SWAP_MASK);
}

static void zuma_reg_set_blender_bg_size(u32 id, u32 width, u32 height)
{
	zd_main_write(id, ZD_BLD_BG_IMG_SIZE_PRI,
		      ZD_BLENDER_BG_HEIGHT_F(height) |
			      ZD_BLENDER_BG_WIDTH_F(width));
}

static void zuma_reg_set_latency_monitor_enable(u32 id, bool en)
{
	zd_main_write_mask(id, ZD_OF_LAT_MON, en ? ~0 : 0,
			   ZD_LATENCY_COUNTER_ENABLE);
}

/*
 * Program the OUTFIFO read urgent + DTA thresholds (vendor decon_reg_set_urgent,
 * values from zumapro-drm-dpu.dtsi decon0). Without this the IDMA is not raised
 * to priority when the OUTFIFO runs low, so under a real framebuffer feed the
 * fifo underruns partway down the frame and frame_done never fires. Write path
 * urgent is unused (wr_en = 0).
 */
static void zuma_reg_set_urgent(u32 id)
{
	zd_main_write_mask(id, ZD_OF_URGENT_EN, ZD_READ_URGENT_GENERATION_EN_F,
			   ZD_READ_URGENT_GENERATION_EN_F |
			   ZD_WRITE_URGENT_GENERATION_EN_F);
	zd_main_write(id, ZD_OF_RD_URGENT_0,
		      ZD_READ_URGENT_HIGH_THRESHOLD_F(0x800) |
		      ZD_READ_URGENT_LOW_THRESHOLD_F(0x400));
	zd_main_write(id, ZD_OF_RD_URGENT_1, ZD_READ_URGENT_WAIT_CYCLE_F(0x10));
	zd_main_write_mask(id, ZD_OF_DTA_CONTROL, ZD_DTA_EN_F, ZD_DTA_EN_F);
	zd_main_write(id, ZD_OF_DTA_THRESHOLD,
		      ZD_DTA_HIGH_TH_F(0x3200) | ZD_DTA_LOW_TH_F(0x600));
}

/*
 * Route the blended output to the requested interface. In single-DSI mode
 * only OUTIF_DSI0 is used regardless of which physical DSIM drives the panel;
 * the DSIMIF_SEL register picks which DECON's OUTFIFO feeds the DSIMIF.
 */
static void zuma_reg_set_data_path(u32 id, enum decon_out_type out_type,
				   bool dsc)
{
	u32 sel = (id == 1) ? ZD_DECON1_OFIFO0 : ZD_DECON0_OFIFO0;
	u32 val;

	if ((out_type & DECON_OUT_DSI0) && (out_type & DECON_OUT_DSI1)) {
		/* dual DSI */
		val = ZD_OUTIF_DSI0 | ZD_OUTIF_DSI1;
		zd_sub_write(id, ZD_DSIMIF_SEL(0), ZD_SEL_DSIM(ZD_DECON0_OFIFO0));
		zd_sub_write(id, ZD_DSIMIF_SEL(1), ZD_SEL_DSIM(ZD_DECON0_OFIFO1));
	} else if (out_type & DECON_OUT_DSI1) {
		val = ZD_OUTIF_DSI0;
		zd_sub_write(id, ZD_DSIMIF_SEL(1), ZD_SEL_DSIM(sel));
	} else if (out_type & DECON_OUT_DSI0) {
		val = ZD_OUTIF_DSI0;
		zd_sub_write(id, ZD_DSIMIF_SEL(0), ZD_SEL_DSIM(sel));
	} else if (out_type & DECON_OUT_DP0) {
		val = ZD_OUTIF_DPIF;
		zd_sub_write(id, ZD_DPIF_SEL(0), ZD_SEL_DP(id));
	} else if (out_type & DECON_OUT_DP1) {
		val = ZD_OUTIF_DPIF;
		zd_sub_write(id, ZD_DPIF_SEL(1), ZD_SEL_DP(id));
	} else if (out_type & DECON_OUT_WB) {
		val = ZD_OUTIF_WB;
	} else {
		val = ZD_OUTIF_DSI0;
		zd_sub_write(id, ZD_DSIMIF_SEL(0), ZD_SEL_DSIM(sel));
	}

	/* dual-DSC compression: route through both encoders + the combiner */
	if (dsc)
		val |= ZD_COMP_DSCC | ZD_COMP_DSC(1) | ZD_COMP_DSC(0);

	zd_main_write_mask(id, ZD_DATA_PATH_CON_0, ZD_COMP_OUTIF_PATH_F(val),
			   ZD_COMP_OUTIF_PATH_MASK);
}

/* ------------------------------------------------------------------ */
/* DSC encoders							      */
/* ------------------------------------------------------------------ */

/*
 * Compressed output width per encoder, from the vendor formula
 * (cal_common/exynos_panel.h decon_get_comp_dsc_width):
 *   DIV_ROUND_UP(DIV_ROUND_UP(slice_width * bpc, 8), 6) * 2
 */
static u32 zuma_dsc_comp_width(const struct drm_dsc_config *cfg)
{
	u32 slice_px = DIV_ROUND_UP(cfg->slice_width * cfg->bits_per_component,
				    8);

	return DIV_ROUND_UP(slice_px, 6) * 2;
}

static void zuma_reg_dsc_config_control(u32 id, u32 dsc_id, u32 ds_en,
					u32 sm_ch, u32 slice_width)
{
	u32 remainder = (slice_width % 3) ? (slice_width % 3) : 3;
	u32 grpcntline = (slice_width + 2) / 3;

	writel(ZD_DSC_SWAP(0x0, 0x1, 0x0) | ZD_DSC_DUAL_SLICE_EN_F(ds_en) |
		       ZD_DSC_SLICE_MODE_CH_F(sm_ch) |
		       ZD_DSC_FLATNESS_DET_TH_F(0x2),
	       zd_sub[id] + ZD_DSC_CONTROL1(dsc_id));

	writel(ZD_DSC_REMAINDER_F(remainder) | ZD_DSC_GRPCNTLINE_F(grpcntline),
	       zd_sub[id] + ZD_DSC_CONTROL3(dsc_id));
}

/* Program one DSC encoder's PPS from the standard packed payload. */
static void zuma_reg_dsc_set_pps(u32 id, u32 dsc_id,
				 const struct drm_dsc_config *cfg)
{
	struct drm_dsc_picture_parameter_set pps;
	const u8 *raw = (const u8 *)&pps;
	int i;

	drm_dsc_pps_payload_pack(&pps, cfg);

	/* PPS bytes 0..87, 4 bytes (big-endian) per register from +0x40 */
	for (i = 0; i < ZD_DSC_PPS_REG_CNT; i++)
		writel(get_unaligned_be32(raw + i * 4),
		       zd_sub[id] + ZD_DSC_PPS00_03(dsc_id) + i * 4);
}

static void zuma_reg_set_dsc(u32 id, const struct drm_dsc_config *cfg)
{
	/*
	 * komodo: dsc_count == slice_count == 2, so each encoder handles one
	 * slice (dual_slice disabled) and the slice-mode-change bit is set.
	 */
	u32 ds_en = 0;
	u32 sm_ch = 1;
	u32 dsc_id;

	for (dsc_id = 0; dsc_id < cfg->slice_count; dsc_id++) {
		zuma_reg_dsc_config_control(id, dsc_id, ds_en, sm_ch,
					    cfg->slice_width);
		zuma_reg_dsc_set_pps(id, dsc_id, cfg);
	}
}

/* OUTFIFO sizing for a DSC (compressed) output. */
static void zuma_reg_set_outfifo_dsc(u32 id, const struct drm_dsc_config *cfg,
				     u32 height)
{
	u32 comp_w = zuma_dsc_comp_width(cfg);

	zd_main_write(id, ZD_OF_SIZE_0,
		      ZD_OUTFIFO_HEIGHT_F(height) | ZD_OUTFIFO_WIDTH_F(comp_w));
	/* dual DSC on decon0 uses the second outfifo too */
	if (cfg->slice_count == 2)
		zd_main_write(id, ZD_OF_SIZE_1, ZD_OUTFIFO_1_WIDTH_F(comp_w));
	zd_main_write(id, ZD_OF_SIZE_2,
		      ZD_OUTFIFO_COMPRESSED_SLICE_HEIGHT_F(cfg->slice_height) |
			      ZD_OUTFIFO_COMPRESSED_SLICE_WIDTH_F(comp_w));
	zd_main_write_mask(id, ZD_OF_TH_TYPE, ZD_OUTFIFO_TH_1H_F,
			   ZD_OUTFIFO_TH_MASK);
}

/* ------------------------------------------------------------------ */
/* Triggers / shadow update / interrupts				      */
/* ------------------------------------------------------------------ */

static void zuma_reg_init_trigger(u32 id, const struct decon_mode *mode)
{
	u32 mask = ZD_HW_TRIG_EN | ZD_HW_TRIG_SEL_MASK | ZD_HW_TRIG_MASK_DECON;
	u32 val = (mode->te_mode == DECON_SW_TRIG) ? 0 : ZD_HW_TRIG_EN;

	/* default HW TE source; keep the trigger masked until start */
	val |= ZD_HW_TRIG_SEL_FROM_DDI0;
	val |= ZD_HW_TRIG_MASK_DECON;

	zd_main_write_mask(id, ZD_TRIG_CON, val, mask);
}

static void zuma_reg_set_trigger(u32 id, const struct decon_mode *mode,
				 enum decon_set_trig trig)
{
	u32 val, mask;

	if (mode->op_mode == DECON_VIDEO_MODE)
		return;

	if (mode->te_mode == DECON_SW_TRIG) {
		val = (trig == DECON_TRIG_UNMASK) ?
			      (ZD_SW_TRIG_EN | ZD_SW_TRIG_DET_EN) : 0;
		mask = ZD_HW_TRIG_EN | ZD_SW_TRIG_EN | ZD_SW_TRIG_DET_EN;
	} else {
		val = (trig == DECON_TRIG_UNMASK) ? ZD_HW_TRIG_EN :
						    ZD_HW_TRIG_MASK_DECON;
		mask = ZD_HW_TRIG_EN | ZD_HW_TRIG_MASK_DECON;
	}

	zd_main_write_mask(id, ZD_TRIG_CON, val, mask);
}

static void zuma_reg_update_req_global(u32 id)
{
	/*
	 * Request a shadow -> active latch on the next HW-TE trigger. In command
	 * mode the vendor issues decon_reg_all_win_shadow_update_req() (all
	 * per-window bits, FOR_DECON) alongside the global/compress request every
	 * frame; without the window bits the DECON keeps scanning out the
	 * previously-latched (bootloader) window and the reprogrammed window sits
	 * unlatched in shadow. Request all three so window, compress and global
	 * config latch together.
	 */
	u32 mask = ZD_SHD_REG_UP_REQ_GLOBAL | ZD_SHD_REG_UP_REQ_FOR_DECON;

	if (id != 2)
		mask |= ZD_SHD_REG_UP_REQ_CMP;

	zd_main_write_mask(id, ZD_SHD_REG_UP_REQ, ~0, mask);
}

static void zuma_reg_clear_int_all(u32 id)
{
	zd_main_write_mask(id, ZD_DECON_INT_PEND, ~0,
			   ZD_INT_PEND_FRAME_DONE | ZD_INT_PEND_FRAME_START);
	zd_main_write_mask(id, ZD_DECON_INT_PEND_EXTRA, ~0,
			   ZD_INT_PEND_RESOURCE_CONFLICT | ZD_INT_PEND_TIME_OUT);
}

static void zuma_reg_set_interrupts(u32 id, bool en)
{
	zuma_reg_clear_int_all(id);

	if (en) {
		u32 val = ZD_INT_EN_FRAME_DONE | ZD_INT_EN_FRAME_START |
			  ZD_INT_EN_EXTRA | ZD_INT_EN;

		zd_main_write_mask(id, ZD_DECON_INT_EN, val, ZD_INT_EN_MASK);
		zd_main_write(id, ZD_DECON_INT_EN_EXTRA,
			      ZD_INT_EN_RESOURCE_CONFLICT | ZD_INT_EN_TIME_OUT);
	} else {
		zd_main_write_mask(id, ZD_DECON_INT_EN, 0,
				   ZD_INT_EN_EXTRA | ZD_INT_EN);
	}
}

/* ------------------------------------------------------------------ */
/* Window (blend / geometry / colormap / enable)			      */
/* ------------------------------------------------------------------ */

static void zuma_reg_set_win_bnd_function(u32 id, u32 win,
					  enum decon_blend_mode blend,
					  u32 plane_alpha)
{
	u32 af_d = ZD_BND_COEF_ONE, ab_d = ZD_BND_COEF_ZERO;
	u32 af_a = ZD_BND_COEF_ONE, ab_a = ZD_BND_COEF_ZERO;
	bool is_plane_a = (plane_alpha > 0) && (plane_alpha <= 0xff);

	if (blend == DECON_BLENDING_NONE && is_plane_a) {
		af_d = ZD_BND_COEF_PLANE_ALPHA0;
		af_a = ZD_BND_COEF_PLANE_ALPHA0;
	} else if (blend == DECON_BLENDING_COVERAGE) {
		af_d = ZD_BND_COEF_ALPHA_MULT;
		ab_d = ZD_BND_COEF_1_M_ALPHA_MULT;
		af_a = ZD_BND_COEF_ALPHA_MULT;
		ab_a = ZD_BND_COEF_1_M_ALPHA_MULT;
	} else if (blend == DECON_BLENDING_PREMULT) {
		af_d = ZD_BND_COEF_PLANE_ALPHA0;
		ab_d = ZD_BND_COEF_1_M_ALPHA_MULT;
		af_a = ZD_BND_COEF_PLANE_ALPHA0;
		ab_a = ZD_BND_COEF_1_M_ALPHA_MULT;
	}

	zd_win_write_mask(id, ZD_WIN_FUNC_CON_0(win),
			  ZD_WIN_ALPHA1_F(0) | ZD_WIN_ALPHA0_F(plane_alpha),
			  ZD_WIN_ALPHA1_MASK | ZD_WIN_ALPHA0_MASK);
	zd_win_write_mask(id, ZD_WIN_FUNC_CON_0(win),
			  ZD_WIN_ALPHA_MULT_SRC_SEL_F(ZD_ALPHA_MULT_SRC_SEL_AF),
			  ZD_WIN_ALPHA_MULT_SRC_SEL_MASK);
	zd_win_write_mask(id, ZD_WIN_FUNC_CON_0(win),
			  ZD_WIN_FUNC_F(ZD_PD_FUNC_USER_DEFINED),
			  ZD_WIN_FUNC_MASK);
	zd_win_write_mask(id, ZD_WIN_FUNC_CON_1(win),
			  ZD_WIN_FG_ALPHA_D_SEL_F(af_d) |
				  ZD_WIN_BG_ALPHA_D_SEL_F(ab_d) |
				  ZD_WIN_FG_ALPHA_A_SEL_F(af_a) |
				  ZD_WIN_BG_ALPHA_A_SEL_F(ab_a),
			  ZD_WIN_FG_ALPHA_D_SEL_MASK | ZD_WIN_BG_ALPHA_D_SEL_MASK |
				  ZD_WIN_FG_ALPHA_A_SEL_MASK |
				  ZD_WIN_BG_ALPHA_A_SEL_MASK);
}

static void zuma_reg_set_winmap(u32 id, u32 win, u32 argb, bool en)
{
	u32 a = (argb >> 24) & 0xff;
	u32 r = (argb >> 16) & 0xff;
	u32 g = (argb >> 8) & 0xff;
	u32 b = (argb >> 0) & 0xff;

	zd_wincon_write_mask(id, ZD_DECON_CON_WIN(win), en ? ~0 : 0,
			     ZD_WIN_MAPCOLOR_EN_F);

	zd_win_write_mask(id, ZD_WIN_COLORMAP_0(win),
			  ZD_WIN_MAPCOLOR_A_F(a) | ZD_WIN_MAPCOLOR_R_F(r),
			  ZD_WIN_MAPCOLOR_A_MASK | ZD_WIN_MAPCOLOR_R_MASK);
	zd_win_write_mask(id, ZD_WIN_COLORMAP_1(win),
			  ZD_WIN_MAPCOLOR_G_F(g) | ZD_WIN_MAPCOLOR_B_F(b),
			  ZD_WIN_MAPCOLOR_G_MASK | ZD_WIN_MAPCOLOR_B_MASK);
}

static void zuma_reg_config_win_channel(u32 id, u32 win, u32 ch)
{
	/* L7 layer is not present on zuma - skip the gap */
	if (ch > 6)
		ch++;

	zd_wincon_write_mask(id, ZD_DECON_CON_WIN(win), ZD_WIN_CHMAP_F(ch),
			     ZD_WIN_CHMAP_MASK);
}

static void zuma_reg_set_win_enable(u32 id, u32 win, bool en)
{
	zd_wincon_write_mask(id, ZD_DECON_CON_WIN(win), en ? ~0 : 0,
			     ZD_WIN_EN_F);
}

/* ------------------------------------------------------------------ */
/* decon_cal_ops							      */
/* ------------------------------------------------------------------ */

static int zuma_decon_init(struct decon_context *ctx,
			   struct platform_device *pdev)
{
	static const struct {
		const char *name;
		void __iomem **base;
	} blocks[] = {
		{ "main", zd_main },
		{ "win", zd_win },
		{ "sub", zd_sub },
		{ "wincon", zd_wincon },
	};
	u32 id = ctx->idx;
	int i;

	if (id >= ZD_MAX_DECON)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(blocks); i++) {
		struct resource *res;
		void __iomem *base;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   blocks[i].name);
		base = devm_ioremap_resource(ctx->dev, res);
		if (IS_ERR(base))
			return PTR_ERR(base);
		blocks[i].base[id] = base;
	}

	return 0;
}

/*
 * Return the DSC config for this output, or NULL for an uncompressed path.
 * Only the komodo DSI panel is wired today, matched by its native resolution.
 */
static const struct drm_dsc_config *
zuma_decon_dsc(const struct decon_config *cfg)
{
	if ((cfg->out_type & DECON_OUT_DSI) &&
	    cfg->image_width == komodo_dsc_cfg.pic_width &&
	    cfg->image_height == komodo_dsc_cfg.pic_height)
		return &komodo_dsc_cfg;

	return NULL;
}

static int zuma_decon_enable(struct decon_context *ctx)
{
	struct decon_config *cfg = &ctx->config;
	u32 id = ctx->idx;
	u32 bg_w = (cfg->mode.dsi_mode == DSI_MODE_DUAL_DSI) ?
			   cfg->image_width * 2 : cfg->image_width;
	const struct drm_dsc_config *dsc = zuma_decon_dsc(cfg);
	/* compressed output uses RGB order; raw DSI uses BGR */
	u32 order = ((cfg->out_type & DECON_OUT_DSI) && !dsc) ? ZD_DECON_BGR :
								ZD_DECON_RGB;
	int i;

	/*
	 * Clean stop before reconfiguring. The DCS command path reaches the panel
	 * fine, but the DECON-driven pixel stream never updated the visible GRAM:
	 * reconfiguring the still-running bootloader DECON left the DSIM's
	 * per-frame write_memory framing continuing mid-stream, so the panel kept
	 * dropping the writes. Per-frame-off the DECON and wait for it to go idle
	 * so the next frame begins a fresh write_memory_start. The panel keeps its
	 * image from GRAM self-refresh while the DECON is briefly stopped.
	 */
	zuma_reg_per_frame_off(id);
	zuma_reg_wait_run_is_off(id, 20 * 1000);

	/*
	 * Handover reinit (vendor _decon_reinit_locked): clear any windows the
	 * bootloader left enabled before reconfiguring.
	 */
	for (i = 0; i < 8; i++)
		zuma_reg_set_win_enable(id, i, false);

	/* init (vendor decon_reg_init) */
	zuma_reg_set_clkgate_mode(id, false);
	zuma_reg_set_sram_enable(id);
	zuma_reg_set_operation_mode(id, cfg->mode.op_mode);
	zuma_reg_set_blender_bg_size(id, bg_w, cfg->image_height);
	zuma_reg_set_bpc(id, cfg->out_bpc);
	zuma_reg_set_latency_monitor_enable(id, true);
	zuma_reg_set_urgent(id);
	zuma_reg_set_rgb_order(id, order);
	zuma_reg_set_data_path(id, cfg->out_type, dsc != NULL);
	/* DSC path must be set up after the data path enables the encoders */
	if (dsc) {
		zuma_reg_set_dsc(id, dsc);
		zuma_reg_set_outfifo_dsc(id, dsc, cfg->image_height);
	} else {
		zuma_reg_set_outfifo_size(id, cfg->image_width,
					  cfg->image_height);
	}
	zuma_reg_init_trigger(id, &cfg->mode);
	zuma_reg_clear_int_all(id);

	/* start (vendor decon_reg_start) */
	zuma_reg_direct_on_off(id, true);
	zuma_reg_update_req_global(id);
	zuma_reg_wait_run_status(id, 20 * 1000);
	zuma_reg_set_trigger(id, &cfg->mode, DECON_TRIG_UNMASK);

	zuma_reg_set_interrupts(id, true);

	return 0;
}

static int zuma_decon_disable(struct decon_context *ctx)
{
	const struct decon_mode *mode = &ctx->config.mode;
	u32 id = ctx->idx;
	/* one frame at 60fps + 20% margin, in us */
	const unsigned long timeout_us = (1000 / 60 * 12 / 10 + 5) * 1000;
	int ret;

	zuma_reg_set_interrupts(id, false);

	if (!zuma_reg_get_run_status(id))
		return 0;

	zuma_reg_set_trigger(id, mode, DECON_TRIG_MASK);

	/* try a clean per-frame off first */
	zuma_reg_per_frame_off(id);
	zuma_reg_update_req_global(id);

	ret = zuma_reg_wait_run_is_off(id, timeout_us);
	if (!ret) {
		zuma_reg_reset(id);
		return 0;
	}

	/* per-frame off timed out: force instant off */
	zuma_reg_direct_on_off(id, false);
	zuma_reg_update_req_global(id);
	ret = zuma_reg_wait_run_is_off(id, timeout_us);
	zuma_reg_clear_int_all(id);

	return ret;
}

static void zuma_decon_set_te(struct decon_context *ctx,
			      enum decon_set_trig trig)
{
	zuma_reg_set_trigger(ctx->idx, &ctx->config.mode, trig);
}

static void zuma_decon_enable_window(struct decon_context *ctx, u32 win,
				     struct decon_win_config *config)
{
	u32 id = ctx->idx;
	u32 start_pos = ZD_WIN_STRPTR_Y_F(config->y) |
			ZD_WIN_STRPTR_X_F(config->x);
	u32 end_pos = ZD_WIN_ENDPTR_Y_F(config->y + config->h - 1) |
		      ZD_WIN_ENDPTR_X_F(config->x + config->w - 1);

	zuma_reg_set_win_bnd_function(id, win, config->blend_mode,
				      config->alpha);
	zd_win_write(id, ZD_WIN_START_POSITION(win), start_pos);
	zd_win_write(id, ZD_WIN_END_POSITION(win), end_pos);
	zd_win_write(id, ZD_WIN_START_TIME_CON(win), config->start_time);
	zuma_reg_set_winmap(id, win, config->color_map.val,
			    config->color_map.en);
	zuma_reg_config_win_channel(id, win, config->dpp_type);
	zuma_reg_set_win_enable(id, win, true);
}

static void zuma_decon_disable_window(struct decon_context *ctx, u32 win)
{
	u32 id = ctx->idx;

	zd_win_write(id, ZD_WIN_FUNC_CON_0(win), 0);
	zd_win_write(id, ZD_WIN_FUNC_CON_1(win), 0);
	zd_win_write(id, ZD_WIN_START_POSITION(win), 0);
	zd_win_write(id, ZD_WIN_END_POSITION(win), 0);
	zd_win_write(id, ZD_WIN_START_TIME_CON(win), 0);
	zuma_reg_set_win_enable(id, win, false);
}

static void zuma_decon_win_update_req(struct decon_context *ctx, u32 win)
{
	zd_main_write_mask(ctx->idx, ZD_SHD_REG_UP_REQ, ~0,
			   ZD_SHD_REG_UP_REQ_WIN(win));
}

static u32 zuma_decon_win_status(struct decon_context *ctx, u32 win)
{
	/*
	 * The winctrl WIN_EN shadow cannot be read back reliably; use the
	 * shadow copy of WIN_END_POSITION as the "window is live" indicator,
	 * mirroring the exynos910 back-end.
	 */
	return zd_win_read(ctx->idx,
			   ZD_WIN_END_POSITION(win) + ZD_SHADOW_OFFSET) ?
		       1 : 0;
}

static u32 zuma_decon_win_update_req_get(struct decon_context *ctx, u32 win)
{
	return zd_main_read(ctx->idx, ZD_SHD_REG_UP_REQ) &
	       ZD_SHD_REG_UP_REQ_WIN(win);
}

static void zuma_decon_update_req_global(struct decon_context *ctx)
{
	zuma_reg_update_req_global(ctx->idx);
}

static u32 zuma_decon_clear_interrupt(struct decon_context *ctx,
				      enum decon_irq irq)
{
	u32 id = ctx->idx;
	u32 pend = zd_main_read(id, ZD_DECON_INT_PEND);

	if (irq == DECON_IRQ_FS && (pend & ZD_INT_PEND_FRAME_START))
		zd_main_write(id, ZD_DECON_INT_PEND, ZD_INT_PEND_FRAME_START);

	if (irq == DECON_IRQ_FD && (pend & ZD_INT_PEND_FRAME_DONE))
		zd_main_write(id, ZD_DECON_INT_PEND, ZD_INT_PEND_FRAME_DONE);

	if (irq == DECON_IRQ_EXT && (pend & ZD_INT_PEND_EXTRA)) {
		u32 ext = zd_main_read(id, ZD_DECON_INT_PEND_EXTRA);

		zd_main_write(id, ZD_DECON_INT_PEND, ZD_INT_PEND_EXTRA);
		if (ext)
			zd_main_write(id, ZD_DECON_INT_PEND_EXTRA, ext);
	}

	return pend;
}

const struct decon_cal_ops zuma_decon_cal_ops = {
	.init			= zuma_decon_init,
	.enable			= zuma_decon_enable,
	.disable		= zuma_decon_disable,
	.set_te			= zuma_decon_set_te,
	.enable_window		= zuma_decon_enable_window,
	.disable_window		= zuma_decon_disable_window,
	.win_update_req		= zuma_decon_win_update_req,
	.win_status		= zuma_decon_win_status,
	.win_update_req_get	= zuma_decon_win_update_req_get,
	.update_req_global	= zuma_decon_update_req_global,
	.clear_interrupt	= zuma_decon_clear_interrupt,
};
