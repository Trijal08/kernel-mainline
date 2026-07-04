/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) Samsung Electronics Co., Ltd.
 * Copyright (c) Google LLC
 *
 * Register map for the Google Tensor "zuma"/"zumapro" (gs201-class) DECON,
 * derived from the vendor cal_9865 DECON register definitions. Only the
 * registers used by the zuma cal_ops back-end are described here.
 *
 * The DECON is split across four separate SFR blocks, each its own reg
 * region in DT:
 *   DECON_MAIN    (global control, interrupts, data path, SRAM, outfifo)
 *   DECON_WIN     (per-window blending/geometry/colormap; 0x1000 stride)
 *   DECON_SUB     (DSIM/DP interface selection; DSIMIF at 0x8000 + 0x1000*i)
 *   DECONx_WINCON (per-window enable/channel/colormap-enable; 0x1000 stride)
 */

#ifndef _REGS_DECON_ZUMA_H_
#define _REGS_DECON_ZUMA_H_

/* -------- DECON_MAIN -------- */
/* per-DECON instance stride within the block */
#define ZD_DECON_OFFSET(_id)			(0x1000 * (_id))
/* shadow copy of a register is at +0x800 */
#define ZD_SHADOW_OFFSET			(0x0800)

#define ZD_GLOBAL_CON				(0x0020)
#define ZD_GLOBAL_CON_SRESET			(1 << 28)
#define ZD_GLOBAL_CON_TEN_BPC_MODE_F		(1 << 20)
#define ZD_GLOBAL_CON_TEN_BPC_MODE_MASK		(1 << 20)
#define ZD_GLOBAL_CON_OPERATION_MODE_F		(1 << 8)
#define ZD_GLOBAL_CON_OPERATION_MODE_CMD_F	(1 << 8)
#define ZD_GLOBAL_CON_OPERATION_MODE_VIDEO_F	(0 << 8)
#define ZD_GLOBAL_CON_RUN_STATUS		(1 << 4)
#define ZD_GLOBAL_CON_DECON_EN			(1 << 1)
#define ZD_GLOBAL_CON_DECON_EN_F		(1 << 0)

#define ZD_TRIG_CON				(0x0030)
#define ZD_HW_TRIG_SEL_MASK			(0x3 << 24)
#define ZD_HW_TRIG_SEL_FROM_NONE		(3 << 24)
#define ZD_HW_TRIG_SEL_FROM_DDI0		(0 << 24)
#define ZD_SW_TRIG_EN				(1 << 8)
#define ZD_HW_TRIG_MASK_DECON			(1 << 4)
#define ZD_SW_TRIG_DET_EN			(1 << 1)
#define ZD_HW_TRIG_EN				(1 << 0)

#define ZD_SHD_REG_UP_REQ			(0x0050)
#define ZD_SHD_REG_UP_REQ_GLOBAL		(1 << 31)
#define ZD_SHD_REG_UP_REQ_CMP			(1 << 20)
#define ZD_SHD_REG_UP_REQ_WIN(_win)		(1 << (_win))
/* all per-window shadow-update requests (vendor SHD_REG_UP_REQ_FOR_DECON) */
#define ZD_SHD_REG_UP_REQ_FOR_DECON		(0x3fff)

#define ZD_DECON_INT_EN				(0x0060)
#define ZD_INT_EN_FRAME_DONE			(1 << 13)
#define ZD_INT_EN_FRAME_START			(1 << 12)
#define ZD_INT_EN_EXTRA				(1 << 4)
#define ZD_INT_EN				(1 << 0)
#define ZD_INT_EN_MASK				(0x303011)

#define ZD_DECON_INT_EN_EXTRA			(0x0064)
#define ZD_INT_EN_RESOURCE_CONFLICT		(1 << 4)
#define ZD_INT_EN_TIME_OUT			(1 << 0)

#define ZD_DECON_INT_PEND			(0x0070)
#define ZD_INT_PEND_FRAME_DONE			(1 << 13)
#define ZD_INT_PEND_FRAME_START			(1 << 12)
#define ZD_INT_PEND_EXTRA			(1 << 4)

#define ZD_DECON_INT_PEND_EXTRA			(0x0074)
#define ZD_INT_PEND_RESOURCE_CONFLICT		(1 << 4)
#define ZD_INT_PEND_TIME_OUT			(1 << 0)

#define ZD_DATA_PATH_CON_0			(0x0200)
#define ZD_COMP_OUTIF_PATH_F(_v)		((_v) << 0)
#define ZD_COMP_OUTIF_PATH_MASK			(0xff << 0)
/* outif + compression selectors written into DATA_PATH_CON_0 */
#define ZD_OUTIF_DSI0				BIT(0)
#define ZD_OUTIF_DSI1				BIT(1)
#define ZD_OUTIF_WB				BIT(2)
#define ZD_OUTIF_DPIF				BIT(3)
#define ZD_COMP_DSC(_id)			BIT((_id) + 4)
#define ZD_COMP_DSCC				BIT(7)

#define ZD_BLD_BG_IMG_SIZE_PRI			(0x0220)
#define ZD_BLENDER_BG_HEIGHT_F(_v)		((_v) << 16)
#define ZD_BLENDER_BG_WIDTH_F(_v)		((_v) << 0)

#define ZD_OF_SIZE_0				(0x0290)
#define ZD_OUTFIFO_HEIGHT_F(_v)			((_v) << 16)
#define ZD_OUTFIFO_WIDTH_F(_v)			((_v) << 0)

#define ZD_OF_SIZE_1				(0x0294)	/* decon0 only */
#define ZD_OUTFIFO_1_WIDTH_F(_v)		((_v) << 0)

#define ZD_OF_SIZE_2				(0x0298)
#define ZD_OUTFIFO_COMPRESSED_SLICE_HEIGHT_F(_v) ((_v) << 16)
#define ZD_OUTFIFO_COMPRESSED_SLICE_WIDTH_F(_v)	((_v) << 0)

#define ZD_OF_TH_TYPE				(0x029C)
#define ZD_OUTFIFO_TH_1H_F			(0x5 << 0)
#define ZD_OUTFIFO_TH_MASK			(0x7 << 0)

#define ZD_OF_PIXEL_ORDER			(0x02A0)
#define ZD_OUTFIFO_PIXEL_ORDER_SWAP_F(_v)	((_v) << 4)
#define ZD_OUTFIFO_PIXEL_ORDER_SWAP_MASK	(0x7 << 4)

#define ZD_OF_LAT_MON				(0x02B8)
#define ZD_LATENCY_COUNTER_ENABLE		(1 << 28)

/* OUTFIFO read/write urgent + DTA - starve the DMA/bus into priority so the
 * IDMA keeps the OUTFIFO fed and the frame does not underrun mid-scan.
 */
#define ZD_OF_URGENT_EN				(0x02C0)
#define ZD_WRITE_URGENT_GENERATION_EN_F		(1 << 1)
#define ZD_READ_URGENT_GENERATION_EN_F		(1 << 0)
#define ZD_OF_RD_URGENT_0			(0x02C4)
#define ZD_READ_URGENT_HIGH_THRESHOLD_F(_v)	((_v) << 16)
#define ZD_READ_URGENT_LOW_THRESHOLD_F(_v)	((_v) << 0)
#define ZD_OF_RD_URGENT_1			(0x02C8)
#define ZD_READ_URGENT_WAIT_CYCLE_F(_v)		((_v) << 0)
#define ZD_OF_WR_URGENT_0			(0x02CC)
#define ZD_OF_DTA_CONTROL			(0x02D0)
#define ZD_DTA_EN_F				(1 << 0)
#define ZD_OF_DTA_THRESHOLD			(0x02D4)
#define ZD_DTA_HIGH_TH_F(_v)			((_v) << 16)
#define ZD_DTA_LOW_TH_F(_v)			((_v) << 0)

/* SRAM enable banks: primary path (secondary = CWB, unused at bring-up) */
#define ZD_SRAM_EN_OF_PRI_REG_CNT		4
#define ZD_SRAM_EN_CNT				8
#define ZD_SRAM_EN_ID(_id)			(1 << ((_id) * 4))
#define ZD_SRAM_EN_OF_PRI(_i)			(0x0300 + ((_i) * 4))
#define ZD_SRAM_EN_OF_PRI_4			(0x0310)
#define ZD_SRAM_EN_OF_SEC(_i)			(0x0320 + ((_i) * 4))
#define ZD_SRAM_EN_OF_SEC_4			(0x0330)
#define ZD_SRAM32_EN_F				(1 << 0)

#define ZD_CLOCK_CON(_id)			(0x5000 - (0x1000 * (_id)))
#define ZD_CLOCK_CON_AUTO_CG_MASK		(0x11111 << 0)
#define ZD_CLOCK_CON_QACTIVE_MASK		((0x1 << 24) | (0x1 << 28))

/* -------- DECON_WIN (0x1000 per window) -------- */
#define ZD_WIN_OFFSET(_id)			(0x1000 * (_id))

#define ZD_WIN_FUNC_CON_0(_id)			(ZD_WIN_OFFSET(_id) + 0x4)
#define ZD_WIN_ALPHA1_F(_v)			(((_v) & 0xFF) << 24)
#define ZD_WIN_ALPHA1_MASK			(0xFFU << 24)
#define ZD_WIN_ALPHA0_F(_v)			(((_v) & 0xFF) << 16)
#define ZD_WIN_ALPHA0_MASK			(0xFF << 16)
#define ZD_WIN_FUNC_F(_v)			(((_v) & 0xF) << 8)
#define ZD_WIN_FUNC_MASK			(0xF << 8)
#define ZD_WIN_ALPHA_MULT_SRC_SEL_F(_v)		(((_v) & 0x3) << 0)
#define ZD_WIN_ALPHA_MULT_SRC_SEL_MASK		(0x3 << 0)

#define ZD_WIN_FUNC_CON_1(_id)			(ZD_WIN_OFFSET(_id) + 0x8)
#define ZD_WIN_FG_ALPHA_D_SEL_F(_v)		(((_v) & 0xF) << 24)
#define ZD_WIN_FG_ALPHA_D_SEL_MASK		(0xF << 24)
#define ZD_WIN_BG_ALPHA_D_SEL_F(_v)		(((_v) & 0xF) << 16)
#define ZD_WIN_BG_ALPHA_D_SEL_MASK		(0xF << 16)
#define ZD_WIN_FG_ALPHA_A_SEL_F(_v)		(((_v) & 0xF) << 8)
#define ZD_WIN_FG_ALPHA_A_SEL_MASK		(0xF << 8)
#define ZD_WIN_BG_ALPHA_A_SEL_F(_v)		(((_v) & 0xF) << 0)
#define ZD_WIN_BG_ALPHA_A_SEL_MASK		(0xF << 0)

#define ZD_WIN_START_POSITION(_id)		(ZD_WIN_OFFSET(_id) + 0xC)
#define ZD_WIN_STRPTR_Y_F(_v)			(((_v) & 0x3FFF) << 16)
#define ZD_WIN_STRPTR_X_F(_v)			(((_v) & 0x3FFF) << 0)

#define ZD_WIN_END_POSITION(_id)		(ZD_WIN_OFFSET(_id) + 0x10)
#define ZD_WIN_ENDPTR_Y_F(_v)			(((_v) & 0x3FFF) << 16)
#define ZD_WIN_ENDPTR_X_F(_v)			(((_v) & 0x3FFF) << 0)

#define ZD_WIN_COLORMAP_0(_id)			(ZD_WIN_OFFSET(_id) + 0x14)
#define ZD_WIN_MAPCOLOR_A_F(_v)			((_v) << 16)
#define ZD_WIN_MAPCOLOR_A_MASK			(0xff << 16)
#define ZD_WIN_MAPCOLOR_R_F(_v)			((_v) << 0)
#define ZD_WIN_MAPCOLOR_R_MASK			(0x3ff << 0)

#define ZD_WIN_COLORMAP_1(_id)			(ZD_WIN_OFFSET(_id) + 0x18)
#define ZD_WIN_MAPCOLOR_G_F(_v)			((_v) << 16)
#define ZD_WIN_MAPCOLOR_G_MASK			(0x3ff << 16)
#define ZD_WIN_MAPCOLOR_B_F(_v)			((_v) << 0)
#define ZD_WIN_MAPCOLOR_B_MASK			(0x3ff << 0)

#define ZD_WIN_START_TIME_CON(_id)		(ZD_WIN_OFFSET(_id) + 0x1C)

/* -------- DECON_WINCON (0x1000 per window) -------- */
#define ZD_DECON_CON_WIN(_id)			(ZD_WIN_OFFSET(_id) + 0x0)
#define ZD_WIN_CHMAP_F(_ch)			(((_ch) & 0xF) << 4)
#define ZD_WIN_CHMAP_MASK			(0xF << 4)
#define ZD_WIN_MAPCOLOR_EN_F			(1 << 1)
#define ZD_WIN_EN_F				(1 << 0)

/* -------- DECON_SUB (DSIMIF / DPIF) -------- */
#define ZD_DSIMIF_OFFSET(_i)			(0x8000 + 0x1000 * (_i))
#define ZD_DSIMIF_SEL(_i)			(ZD_DSIMIF_OFFSET(_i) + 0x0)
#define ZD_SEL_DSIM(_v)				((_v) << 0)
#define ZD_SEL_DSIM_MASK			(0x7 << 0)

#define ZD_DPIF_OFFSET(_i)			(0xC000 + 0x1000 * (_i))
#define ZD_DPIF_SEL(_i)				(ZD_DPIF_OFFSET(_i) + 0x0)
#define ZD_SEL_DP(_v)				((_v) << 1)

/* -------- DECON_SUB: DSC encoders (0x1000 stride, in the SUB block) -------- */
#define ZD_DSC_OFFSET(_i)			(0x1000 * (_i))

#define ZD_DSC_CONTROL1(_i)			(ZD_DSC_OFFSET(_i) + 0x0004)
#define ZD_DSC_SW_RESET				(0x1 << 28)
/* DSC_SWAP(bit_swap, byte_swap, word_swap) */
#define ZD_DSC_SWAP(_b, _c, _w)			(((_b) << 10) | ((_c) << 9) | ((_w) << 8))
#define ZD_DSC_FLATNESS_DET_TH_F(_v)		((_v) << 4)
#define ZD_DSC_SLICE_MODE_CH_F(_v)		((_v) << 1)
#define ZD_DSC_DUAL_SLICE_EN_F(_v)		((_v) << 0)

#define ZD_DSC_CONTROL3(_i)			(ZD_DSC_OFFSET(_i) + 0x000C)
#define ZD_DSC_REMAINDER_F(_v)			((_v) << 12)
#define ZD_DSC_GRPCNTLINE_F(_v)			((_v) << 0)

/*
 * PPS registers hold the standard 128-byte DSC PPS payload, 4 bytes per
 * 32-bit register (byte 0 in [31:24]), starting at +0x40. PPS bytes 0..87
 * span DSC_PPS00_03 .. the 22nd register.
 */
#define ZD_DSC_PPS00_03(_i)			(ZD_DSC_OFFSET(_i) + 0x0040)
#define ZD_DSC_PPS_REG_CNT			22

/* OUTFIFO -> interface routing values written into DSIMIF_SEL */
#define ZD_DECON0_OFIFO0			0x0
#define ZD_DECON0_OFIFO1			0x1
#define ZD_DECON1_OFIFO0			0x2
#define ZD_DECON2_OFIFO0			0x4
#define ZD_DECON_OFIFO_NONE			0xF

#endif /* _REGS_DECON_ZUMA_H_ */
