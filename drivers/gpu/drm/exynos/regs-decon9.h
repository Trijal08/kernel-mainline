/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef _REGS_DECON_H
#define _REGS_DECON_H

#define CHK_DPU_ID(_decon_idx) (_decon_idx >> 1)
#define CHK_DECON_ID(_decon_idx) (_decon_idx & 0x1)
#define SYSREG_DPUM 0x18C20000
#define SYSREG_DPUS(_v) (0x1AA10000 + ((0x2 * (_v - 1)) << 20))
#define SYSREG_DPU(_dpu_id) \
	((_dpu_id == 0) ? SYSREG_DPUM : SYSREG_DPUS(_dpu_id))

#define DISP_DPU_MIPI_PHY_CON 0x0008
/* _v : [0,1] */
#define SEL_RESET_DPHY_MASK(_v) (0x1 << (4 + (_v)))
#define M_RESETN_M4S4_MODULE_MASK (0x1 << 1)
#define M_RESETN_M4S4_TOP_MASK (0x1 << 0)

#define DISP_DPU_TE_QACTIVE_PLL_EN 0x1010
#define TE_QACTIVE_PLL_EN (0x1 << 0)

/*
 * [ BLK_DPU BASE ADDRESS ]
 *
 * - CMU_DPUM			0x18C0_0000
 * - CMU_DPUS0 			0x1AA0_0000
 * - CMU_DPUS1 			0x1AC0_0000
 * - SYSREG_DPUM 		0x18C1_0000
 * - SYSREG_DPUS0 		0x1AA1_0000
 * - SYSREG_DPUS1 		0x1AC1_0000
 * - DPP_DPUM 			0x18C2_0000
 * - DPP_DPUS0 			0x1AA2_0000
 * - DPP_DPUS1 			0x1AC2_0000
 * - DECON0_DPUM 		0x18C3_0000
 * - DECON1_DPUM 		0x18C4_0000
 * - DECON0_DPUS0 		0x1AA3_0000
 * - DECON1_DPUS0 		0x1AA4_0000
 * - DECON0_DPUS1 		0x1AC3_0000
 * - DECON1_DPUS1 		0x1AC4_0000
 * - DPUM_DMA 			0x18C5_0000
 * - DPUS0_DMA 			0x1AA5_0000
 * - DPUS1_DMA 			0x1AC5_0000
 * - MIPI_DSIM0 		0x18C6_0000
 * - MIPI_DSIM1 		0x18C7_0000
 * - MIPI_DPHY 			0x18D8_0000
 * - SYSMMU_D0_DPUM 		0x18C8_0000
 * - SYSMMU_D1_DPUM 		0x18C9_0000
 * - SYSMMU_D2_DPUM 		0x18CA_0000
 * - SYSMMU_D3_DPUM 		0x18CB_0000
 * - SYSMMU_D0_DPUM_S 		0x18CC_0000
 * - SYSMMU_D1_DPUM_S 		0x18CD_0000
 * - SYSMMU_D2_DPUM_S 		0x18CE_0000
 * - SYSMMU_D3_DPUM_S 		0x18CF_0000
 * - SYSMMU_D0_DPUS0 		0x1AA8_0000
 * - SYSMMU_D1_DPUS0 		0x1AA9_0000
 * - SYSMMU_D2_DPUS0 		0x1AAA_0000
 * - SYSMMU_D3_DPUS0 		0x1AAB_0000
 * - SYSMMU_D0_DPUS0_S 		0x1AAC_0000
 * - SYSMMU_D1_DPUS0_S 		0x1AAD_0000
 * - SYSMMU_D2_DPUS0_S 		0x1AAE_0000
 * - SYSMMU_D3_DPUS0_S 		0x1AAF_0000
 * - SYSMMU_D0_DPUS1 		0x1AC8_0000
 * - SYSMMU_D1_DPUS1 		0x1AC9_0000
 * - SYSMMU_D2_DPUS1 		0x1ACA_0000
 * - SYSMMU_D3_DPUS1 		0x1ACB_0000
 * - SYSMMU_D0_DPUS1_S 		0x1ACC_0000
 * - SYSMMU_D1_DPUS1_S 		0x1ACD_0000
 * - SYSMMU_D2_DPUS1_S 		0x1ACE_0000
 * - SYSMMU_D3_DPUS1_S 		0x1ACF_0000
 * - PPMU_D0_DPUM 		0x18D0_0000
 * - PPMU_D1_DPUM 		0x18D1_0000
 * - PPMU_D2_DPUM 		0x18D2_0000
 * - PPMU_D3_DPUM 		0x18D3_0000
 * - PPMU_D0_DPUS0 		0x1AB0_0000
 * - PPMU_D1_DPUS0 		0x1AB1_0000
 * - PPMU_D2_DPUS0 		0x1AB2_0000
 * - PPMU_D3_DPUS0 		0x1AB3_0000
 * - PPMU_D0_DPUS1 		0x1AD0_0000
 * - PPMU_D1_DPUS1 		0x1AD1_0000
 * - PPMU_D2_DPUS1 		0x1AD2_0000
 * - PPMU_D3_DPUS1 		0x1AD3_0000
 * - BTM_D0_DPUM 		0x18D4_0000
 * - BTM_D1_DPUM 		0x18D5_0000
 * - BTM_D2_DPUM 		0x18D6_0000
 * - BTM_D3_DPUM 		0x18D7_0000
 * - BTM_D0_DPUS0 		0x1AB4_0000
 * - BTM_D1_DPUS0 		0x1AB5_0000
 * - BTM_D2_DPUS0 		0x1AB6_0000
 * - BTM_D3_DPUS0 		0x1AB7_0000
 * - BTM_D0_DPUS1 		0x1AD4_0000
 * - BTM_D1_DPUS1 		0x1AD5_0000
 * - BTM_D2_DPUS1 		0x1AD6_0000
 * - BTM_D3_DPUS1 		0x1AD7_0000
 * - VGEN_DPUM 			0x18DA_0000
 * - VGEN_DPUS0 		0x1ABA_0000
 * - VGEN_DPUS1 		0x1ADA_0000
 * - D_TZPC_DPUM 		0x18DB_0000
 * - D_TZPC_DPUS0 		0x1ABB_0000
 * - D_TZPC_DPUS1 		0x1ADB_0000
 */

/*
 *	IP			start_offset	end_offset
 *=================================================
 *	DECON0			0x8000		0x82A4
 * 	DECON1			0x9000 		0x92A4
 *	BLENDER			0x0004		0x701C
 *-------------------------------------------------
 *	DSC0			0x2000		0x2fff
 *	DSC1			0x3000		0x3fff
 *-------------------------------------------------
 */

/*
 * DECON registers
 * ->
 * updated by
 * SHADOW_REG_UPDATE_REQ[31] 		: 0x0000~0x0FFF
 * SHADOW_REG_UPDATE_REQ_WIN5[0] 	: 0x10E0~0x110C (0x0218[22:20],0x0214[21:20])
 * SHADOW_REG_UPDATE_REQ_WIN4[4] 	: 0x10B0~0x10DC (0x0218[18:16],0x0214[17:16])
 * SHADOW_REG_UPDATE_REQ_WIN3[3] 	: 0x1090~0x10AC (0x0218[14:12],0x0214[13:12])
 * SHADOW_REG_UPDATE_REQ_WIN2[2] 	: 0x1060~0x108C (0x0218[10:8] ,0x0214[9:8]  )
 * SHADOW_REG_UPDATE_REQ_WIN1[1] 	: 0x1030~0x105C (0x0218[6:4]  ,0x0214[5:4]  )
 * SHADOW_REG_UPDATE_REQ_WIN0[0] 	: 0x0004~0x001C (0x0218[2:0]  ,0x0214[1:0]  )
 */

/*
 * DECON0 : Base Address + 0x8xxx
 * DECON1 : Base Address + 0x9xxx
 * Base Addr was defined by DT.
 * DPUx_DECON0 : 0x****_8000
 * DPUx_DECON1 : 0x****_9000
 * ex) DPUM_DECON0 : 0x18C3_8000
 */

#define BASED_DECON0 0
#define BASED_DECON1 1

#define GLOBAL_CONTROL 0x0000
#define GLOBAL_CONTROL_SRESET (1 << 28)
#define GLOBAL_CONTROL_PSLVERR_EN (1 << 24)
#define GLOBAL_CONTROL_TEN_BPC_MODE_F (1 << 20)
#define GLOBAL_CONTROL_TEN_BPC_MODE_MASK (1 << 20)
#define GLOBAL_CONTROL_STANDALONE_MODE_F (0x0 << 12)
#define GLOBAL_CONTROL_OPERATION_MODE_VIDEO_BY_TE_F (1 << 9)
#define GLOBAL_CONTROL_OPERATION_MODE_F (1 << 8)
#define GLOBAL_CONTROL_OPERATION_MODE_VIDEO_F (0 << 8)
#define GLOBAL_CONTROL_OPERATION_MODE_CMD_F (1 << 8)
#define GLOBAL_CONTROL_IDLE_STATUS (1 << 5)
#define GLOBAL_CONTROL_RUN_STATUS (1 << 4)
#define GLOBAL_CONTROL_DECON_EN (1 << 1)
#define GLOBAL_CONTROL_DECON_EN_F (1 << 0)

#define RESOURCE_OCCUPANCY_INFO_0 0x0010
#define RESOURCE_OCCUPANCY_INFO_1 0x0014
#define RESOURCE_OCCUPANCY_INFO_2 0x0018

#define SRAM_SHARE_ENABLE 0x0030
#define SRAM0_SHARE_ENABLE_F (1 << 0)
#define SRAM1_SHARE_ENABLE_F (1 << 4)
#define SRAM2_SHARE_ENABLE_F (1 << 8)
#define SRAM3_SHARE_ENABLE_F (1 << 12)
#define SRAM4_SHARE_ENABLE_F (1 << 16)
#define SRAM5_SHARE_ENABLE_F (1 << 20)
#define SRAM6_SHARE_ENABLE_F (1 << 24)
#define ALL_SRAM_SHARE_ENABLE (0x1111111 << 0)
#define ALL_SRAM_SHARE_DISABLE (0x0)

#define INTERRUPT_ENABLE 0x0040
#define DPU_DQE_DIMMING_END_INT_EN (1 << 21)
#define DPU_DQE_DIMMING_START_INT_EN (1 << 20)
#define DPU_FRAME_DONE_INT_EN (1 << 13)
#define DPU_FRAME_START_INT_EN (1 << 12)
#define DPU_EXTRA_INT_EN (1 << 4)
#define DPU_INT_EN (1 << 0)
#define INTERRUPT_ENABLE_MASK 0x3011

#define EXTRA_INTERRUPT_ENABLE 0x0044
#define DPU_RESOURCE_CONFLICT_INT_EN (1 << 8)
#define DPU_TIME_OUT_INT_EN (1 << 4)

#define TIME_OUT_VALUE 0x0048

#define INTERRUPT_PENDING 0x004C
#define DPU_DQE_DIMMING_END_INT_PEND (1 << 21)
#define DPU_DQE_DIMMING_START_INT_PEND (1 << 20)
#define DPU_FRAME_DONE_INT_PEND (1 << 13)
#define DPU_FRAME_START_INT_PEND (1 << 12)
#define DPU_EXTRA_INT_PEND (1 << 4)

#define EXTRA_INTERRUPT_PENDING 0x0050
#define DPU_RESOURCE_CONFLICT_INT_PEND (1 << 8)
#define DPU_TIME_OUT_INT_PEND (1 << 4)

#define INTERRUPT_ENABLE_SFI 0x0054
#define DPU_ECC_ERROR_SFI_INT_EN (1 << 5)
#define DPU_FRAME_DONE_SFI_INT_EN (1 << 4)
#define DPU_INT_EN_SFI (1 << 0)

#define INTERRUPT_PENDING_SFI 0x0058
#define DPU_ECC_ERROR_SFI_INT_PEND (1 << 5)
#define DPU_FRAME_DONE_SFI_INT_PEND (1 << 4)

#define HW_SW_TRIG_CONTROL 0x0070
#define HW_SW_TRIG_HS_STATUS (1 << 28)
#define HW_TRIG_SEL(_v) ((_v) << 24)
#define HW_TRIG_SEL_MASK (0x3 << 24)
#define HW_TRIG_SEL_FROM_DDI1 (1 << 24)
#define HW_TRIG_SEL_FROM_DDI0 (0 << 24)
#define HW_TRIG_SKIP(_v) ((_v) << 16)
#define HW_TRIG_SKIP_MASK (0xff << 16)
#define HW_TRIG_ACTIVE_VALUE (1 << 13)
#define HW_TRIG_EDGE_POLARITY (1 << 12)
#define SW_TRIG_EN (1 << 8)
#define HW_TRIG_MASK_DECON (1 << 4)
#define HW_SW_TRIG_TIMER_CLEAR (1 << 3)
#define HW_SW_TRIG_TIMER_EN (1 << 2)
#define HW_TRIG_EN (1 << 0)

#define HW_SW_TRIG_TIMER 0x0074

#define HW_TE_CNT 0x0078
#define HW_TRIG_CNT_B_GET(_v) (((_v) >> 16) & 0xffff)
#define HW_TRIG_CNT_A_GET(_v) (((_v) >> 0) & 0xffff)

#define OTF_FRAME_START_SEL 0x0080
#define OTF_FS_SEL_OTF(_v) ((_v) << 0)

#define CLOCK_CONTROL_0 0x00F0
/*
 * [28] QACTIVE_PLL_VALUE = 0
 * [24] QACTIVE_VALUE = 0
 *   0: QACTIVE is dynamically changed by DECON h/w,
 *   1: QACTIVE is stuck to 1'b1
 * [16][12][8][4][0] AUTO_CG_EN_xxx
 */
/* clock gating is disabled on bringup */
#define CLOCK_CONTROL_0_CG_MASK (0x11111 << 0)
#define CLOCK_CONTROL_0_QACTIVE_MASK ((0x1 << 24) | (0x1 << 28))
#define CLOCK_CONTROL_0_TE_QACTIVE_PLL_ON (0x1 << 28)

#define SPLITTER_SIZE_CONTROL_0 0x0100
#define SPLITTER_HEIGHT_F(_v) ((_v) << 16)
#define SPLITTER_HEIGHT_MASK (0x3fff << 16)
#define SPLITTER_HEIGHT_GET(_v) (((_v) >> 16) & 0x3fff)
#define SPLITTER_WIDTH_F(_v) ((_v) << 0)
#define SPLITTER_WIDTH_MASK (0x3fff << 0)
#define SPLITTER_WIDTH_GET(_v) (((_v) >> 0) & 0x3fff)

#define SPLITTER_SPLIT_IDX_CONTROL 0x0104
#define SPLITTER_SPLIT_IDX_F(_v) ((_v) << 0)
#define SPLITTER_SPLIT_IDX_MASK (0x3fff << 0)
#define SPLITTER_OVERLAP_F(_v) ((_v) << 16)
#define SPLITTER_OVERLAP_MASK (0x7f << 16)

#define OUTFIFO_SIZE_CONTROL_0 0x0120
#define OUTFIFO_HEIGHT_F(_v) ((_v) << 16)
#define OUTFIFO_HEIGHT_MASK (0x3fff << 16)
#define OUTFIFO_HEIGHT_GET(_v) (((_v) >> 16) & 0x3fff)
#define OUTFIFO_WIDTH_F(_v) ((_v) << 0)
#define OUTFIFO_WIDTH_MASK (0x3fff << 0)
#define OUTFIFO_WIDTH_GET(_v) (((_v) >> 0) & 0x3fff)

#define OUTFIFO_SIZE_CONTROL_1 0x0124
#define OUTFIFO_1_WIDTH_F(_v) ((_v) << 0)
#define OUTFIFO_1_WIDTH_MASK (0x3fff << 0)
#define OUTFIFO_1_WIDTH_GET(_v) (((_v) >> 0) & 0x3fff)

#define OUTFIFO_SIZE_CONTROL_2 0x0128
#define OUTFIFO_COMPRESSED_SLICE_HEIGHT_F(_v) ((_v) << 16)
#define OUTFIFO_COMPRESSED_SLICE_HEIGHT_MASK (0x3fff << 16)
#define OUTFIFO_COMPRESSED_SLICE_HEIGHT_GET(_v) (((_v) >> 16) & 0x3fff)
#define OUTFIFO_COMPRESSED_SLICE_WIDTH_F(_v) ((_v) << 0)
#define OUTFIFO_COMPRESSED_SLICE_WIDTH_MASK (0x3fff << 0)
#define OUTFIFO_COMPRESSED_SLICE_WIDTH_GET(_v) (((_v) >> 0) & 0x3fff)

#define OUTFIFO_TH_CONTROL_0 0x012C
#define OUTFIFO_TH_1H_F (0x5 << 0)
#define OUTFIFO_TH_2H_F (0x6 << 0)
#define OUTFIFO_TH_F(_v) ((_v) << 0)
#define OUTFIFO_TH_MASK (0x7 << 0)
#define OUTFIFO_TH_GET(_v) ((_v) >> 0 & 0x7)

#define OUTFIFO_DATA_ORDER_CONTROL 0x0130
#define OUTFIFO_PIXEL_ORDER_SWAP_F(_v) ((_v) << 4)
#define OUTFIFO_PIXEL_ORDER_SWAP_MASK (0x7 << 4)
#define OUTFIFO_PIXEL_ORDER_SWAP_GET(_v) (((_v) >> 4) & 0x7)

#define OUTFIFO_LEVEL 0x0134
#define OUTFIFO_FIFO_LEVEL_F(_v) (((_v) & 0xffff) << 0)

#define READ_URGENT_CONTROL_0 0x0140
#define READ_URGETN_GENERATION_EN_F (0x1 << 0)

#define READ_URGENT_CONTROL_1 0x0144
#define READ_URGENT_HIGH_THRESHOLD_F(_v) ((_v) << 16)
#define READ_URGENT_HIGH_THRESHOLD_MASK (0xffff << 16)
#define READ_URGENT_HIGH_THRESHOLD_GET(_v) (((_v) >> 16) & 0xffff)
#define READ_URGENT_LOW_THRESHOLD_F(_v) ((_v) << 0)
#define READ_URGENT_LOW_THRESHOLD_MASK (0xffff << 0)
#define READ_URGENT_LOW_THRESHOLD_GET(_v) (((_v) >> 0) & 0xffff)

#define READ_URGENT_CONTROL_2 0x0148
#define READ_URGENT_WAIT_CYCLE_F(_v) ((_v) << 0)
#define READ_URGENT_WAIT_CYCLE_GET(_v) ((_v) >> 0)

#define DTA_CONTROL 0x0180
#define DTA_EN_F (1 << 0)

#define DTA_THRESHOLD 0x0184
#define DTA_HIGH_TH_F(_v) ((_v) << 16)
#define DTA_HIGH_TH_MASK (0xffff << 16)
#define DTA_HIGH_TH_GET(_v) (((_v) >> 16) & 0xffff)
#define DTA_LOW_TH_F(_v) ((_v) << 0)
#define DTA_LOW_TH_MASK (0xffff << 0)
#define DTA_LOW_TH_GET(_v) (((_v) >> 0) & 0xffff)

#define BLENDER_BG_IMAGE_SIZE_0 0x0200
#define BLENDER_BG_HEIGHT_F(_v) ((_v) << 16)
#define BLENDER_BG_HEIGHT_MASK (0x3fff << 16)
#define BLENDER_BG_HEIGHT_GET(_v) (((_v) >> 16) & 0x3fff)
#define BLENDER_BG_WIDTH_F(_v) ((_v) << 0)
#define BLENDER_BG_WIDTH_MASK (0x3fff << 0)
#define BLENDER_BG_WIDTH_GET(_v) (((_v) >> 0) & 0x3fff)

#define BLENDER_BG_IMAGE_COLOR_0 0x0208
#define BLENDER_BG_A_F(_v) ((_v) << 16)
#define BLENDER_BG_A_MASK (0xff << 16)
#define BLENDER_BG_A_GET(_v) (((_v) >> 16) & 0xff)
#define BLENDER_BG_R_F(_v) ((_v) << 0)
#define BLENDER_BG_R_MASK (0x3ff << 0)
#define BLENDER_BG_R_GET(_v) (((_v) >> 0) & 0x3ff)

#define BLENDER_BG_IMAGE_COLOR_1 0x020C
#define BLENDER_BG_G_F(_v) ((_v) << 16)
#define BLENDER_BG_G_MASK (0x3ff << 16)
#define BLENDER_BG_G_GET(_v) (((_v) >> 16) & 0x3ff)
#define BLENDER_BG_B_F(_v) ((_v) << 0)
#define BLENDER_BG_B_MASK (0x3ff << 0)
#define BLENDER_BG_B_GET(_v) (((_v) >> 0) & 0x3ff)

#define LRMERGER_MODE_CONTROL 0x0210
#define LRM23_MODE_F(_v) ((_v) << 16)
#define LRM23_MODE_MASK (0x7 << 16)
#define LRM01_MODE_F(_v) ((_v) << 0)
#define LRM01_MODE_MASK (0x7 << 0)

#define DATA_PATH_CONTROL_0 0x0214
#define WIN_MAPCOLOR_EN_F(_win) (1 << (4 * _win + 1))
#define WIN_EN_F(_win) (1 << (4 * _win + 0))

#define DATA_PATH_CONTROL_1 0x0218
#define WIN_CHMAP_F(_win, _ch) (((_ch) & 0xf) << (4 * _win))
#define WIN_CHMAP_MASK(_win) (0xf << (4 * _win))

#define DATA_PATH_CONTROL_2 0x0230
#define EHNANCE_PATH_F(_v) ((_v) << 12)
#define EHNANCE_PATH_MASK (0x7 << 12)
#define EHNANCE_PATH_GET(_v) (((_v) >> 12) & 0x7)

#define DSC_PATH_F(_v) ((_v) << 4)
#define DSC_PATH_MASK (0xF << 4)
#define DSC_PATH_GET(_v) (((_v) >> 4) & 0xF)

#define OUTIF_PATH_F(_v) ((_v) << 0)
#define OUTIF_PATH_MASK (0xF << 0)
#define OUTIF_PATH_GET(_v) (((_v) >> 0) & 0xF)

#define COMP_OUTIF_PATH_F(_v) ((_v) << 0)
#define COMP_OUTIF_PATH_MASK (0xff << 0)
#define COMP_OUTIF_PATH_GET(_v) (((_v) >> 0) & 0xff)

#define OTF_DITH_CONTROL 0x0270
#define DITH_MASK_SEL_F (1 << 1)
#define DITH_MASK_SPIN_F (1 << 0)

/* DECON CRC for ASB */
#define CRC_LINKIF_DATA_0 0x0280
#define CRC_DATA_DSIMIF1_GET(_v) (((_v) >> 16) & 0xffff)
#define CRC_DATA_DSIMIF0_GET(_v) (((_v) >> 0) & 0xffff)

#define CRC_LINKIF_DATA_2 0x0288
#define CRC_DATA_DP1_GET(_v) (((_v) >> 16) & 0xffff)
#define CRC_DATA_DP0_GET(_v) (((_v) >> 0) & 0xffff)

#define CRC_LINKIF_CONTROL 0x028C
#define CRC_COLOR_SEL(_v) ((_v) << 16)
#define CRC_COLOR_SEL_MASK (0x3 << 16)
#define CRC_START (1 << 0)

#define FRAME_COUNT 0x02A0

/* WAIT_CYCLE_AFTER_SFR_UPDATE register is hidden for customer. */
#define WAIT_CYCLE_AFTER_SFR_UPDATE 0x02A4
#define WAIT_CYCLE_AFTER_SFR_UPDATE_F(_v) (((_v) & 0x1f) << 0)

/* DECON DEBUG guided from SoC Team */
#define DECON_DEBUG_SFR_START 0x0400
#define DECON_DEBUG_SFR_END 0x05FC

/* COMMON SFR DECON0 AND DECON1 */
#define DECON_CONIF_BASE 0x0000
#define DSIM0_CONIF_BASE (DECON_CONIF_BASE + 0x0000)
#define DSIM1_CONIF_BASE (DECON_CONIF_BASE + 0x1000)
#define DP0_CONIF_BASE (DECON_CONIF_BASE + 0x2000)
#define DP1_CONIF_BASE (DECON_CONIF_BASE + 0x3000)

#define DSIM0_CONNECTION_CONTROL (DSIM0_CONIF_BASE + 0x0000)
#define DSIM1_CONNECTION_CONTROL (DSIM1_CONIF_BASE + 0x0000)
#define DSIM_CONNECTION_DSIM_F(_v) (((_v) & 0x3) << 0)
#define DSIM_CONNECTION_DSIM_GET(_v) (((_v) >> 0) & 0x3)
#define DSIM_CONNECTION_DSIM_MASK (0x7 << 0)

#define DSIM0_TE_TIMEOUT_CONTROL (DSIM0_CONIF_BASE + 0x0004)
#define DSIM1_TE_TIMEOUT_CONTROL (DSIM1_CONIF_BASE + 0x0004)
#define DSIM_TE_TIMEOUT_CNT(_v) ((_v) << 0)
#define DSIM_TE_TIMEOUT_CNT_MASK (0xffff << 0)
#define DSIM_TE_TIMEOUT_CNT_GET(_v) (((_v) >> 0) & 0xffff)

#define DSIM0_START_TIME_CONTROL (DSIM0_CONIF_BASE + 0x0008)
#define DSIM0_START_TIME(_v) ((_v) << 0)

#define DSIM1_START_TIME_CONTROL (DSIM1_CONIF_BASE + 0x0008)
#define DSIM1_START_TIME(_v) ((_v) << 0)

#define DP0_CONNECTION_CONTROL_0 DP0_CONIF_BASE
#define DP1_CONNECTION_CONTROL_0 DP1_CONIF_BASE
#define DP_CONNECTION_SEL_ATB_F(_v) (((_v) & 0x7) << 4)
#define DP_CONNECTION_SEL_F(_v) (((_v) & 0x7) << 0)
#define DP_CONNECTION_SEL_ATB_MASK (0x7 << 4)
#define DP_CONNECTION_SEL_MASK (0x7 << 0)
#define DP_CONNECTION_SEL_GET(_v) (((_v) >> 0) & 0x7)

/* DECON GLOBAL */
#define SHADOW_REG_UPDATE_REQ 0x0000
#define SHADOW_REG_UPDATE_REQ_GLOBAL (1 << 31)
#define SHADOW_REG_UPDATE_REQ_DQE (1 << 28)
#define SHADOW_REG_UPDATE_REQ_WIN(_win) (1 << (_win))
#define SHADOW_REG_UPDATE_REQ_FOR_DECON (0xff)

#define HW_TRIG_MASK 0x0030
#define HW_TRIG_MASK_SECURE 0x0034

/* BLENDER */

#define WIN_SHD_OFFSET 0x0800

#define WIN_SECURE_CONTROL 0x0000

#define WIN_CONTROL_0 0x0004
#define WIN_ALPHA1_F(_v) (((_v) & 0xFF) << 24)
#define WIN_ALPHA1_MASK (0xFF << 24)
#define WIN_ALPHA0_F(_v) (((_v) & 0xFF) << 16)
#define WIN_ALPHA0_MASK (0xFF << 16)
#define WIN_ALPHA_GET(_v, _n) (((_v) >> (16 + 8 * (_n))) & 0xFF)
#define WIN_FUNC_F(_v) (((_v) & 0xF) << 8)
#define WIN_FUNC_MASK (0xF << 8)
#define WIN_FUNC_GET(_v) (((_v) >> 8) & 0xf)
#define WIN_SRESET (1 << 4)
#define WIN_ALPHA_MULT_SRC_SEL_F(_v) (((_v) & 0x3) << 0)
#define WIN_ALPHA_MULT_SRC_SEL_MASK (0x3 << 0)

#define WIN_CONTROL_1 0x0008
#define WIN_FG_ALPHA_D_SEL_F(_v) (((_v) & 0xF) << 24)
#define WIN_FG_ALPHA_D_SEL_MASK (0xF << 24)
#define WIN_BG_ALPHA_D_SEL_F(_v) (((_v) & 0xF) << 16)
#define WIN_BG_ALPHA_D_SEL_MASK (0xF << 16)
#define WIN_FG_ALPHA_A_SEL_F(_v) (((_v) & 0xF) << 8)
#define WIN_FG_ALPHA_A_SEL_MASK (0xF << 8)
#define WIN_BG_ALPHA_A_SEL_F(_v) (((_v) & 0xF) << 0)
#define WIN_BG_ALPHA_A_SEL_MASK (0xF << 0)

#define WIN_START_POSITION 0x000C
#define WIN_STRPTR_Y_F(_v) (((_v) & 0x3FFF) << 16)
#define WIN_STRPTR_X_F(_v) (((_v) & 0x3FFF) << 0)

#define WIN_END_POSITION 0x0010
#define WIN_ENDPTR_Y_F(_v) (((_v) & 0x3FFF) << 16)
#define WIN_ENDPTR_X_F(_v) (((_v) & 0x3FFF) << 0)

#define WIN_COLORMAP_0 0x0014
#define WIN_MAPCOLOR_A_F(_v) ((_v) << 16)
#define WIN_MAPCOLOR_A_MASK (0xff << 16)
#define WIN_MAPCOLOR_R_F(_v) ((_v) << 0)
#define WIN_MAPCOLOR_R_MASK (0x3ff << 0)

#define WIN_COLORMAP_1 0x0018
#define WIN_MAPCOLOR_G_F(_v) ((_v) << 16)
#define WIN_MAPCOLOR_G_MASK (0x3ff << 16)
#define WIN_MAPCOLOR_B_F(_v) ((_v) << 0)
#define WIN_MAPCOLOR_B_MASK (0x3ff << 0)

#define WIN_START_TIME_CONTROL 0x001C
#define WIN_START_TIME_CONTROL_F(_v) ((_v) << 0)
#define WIN_START_TIME_CONTROL_MASK (0x3fff << 0)

/*
 * DSC registers (Base Addr = DECON_SUB0 + 0x2000)
 * ->
 * 0x2000 ~
 * DSC 0 : 0x0000
 * DSC 1 : 0x1000
 *
 * <-
 * DSC registers
 */

#define DSC0_OFFSET 0x0000
#define DSC1_OFFSET 0x1000

#define DSC_CONTROL0 0x0000
#define DSC_SW_RESET (0x1 << 28)
#define DSC_DCG_EN_REF(_v) ((_v) << 19)
#define DSC_DCG_EN_SSM(_v) ((_v) << 18)
#define DSC_DCG_EN_ICH(_v) ((_v) << 17)
#define DSC_DCG_EN_ALL_OFF (0x0 << 17)
#define DSC_DCG_EN_ALL_MASK (0x7 << 17)
#define DSC_BIT_SWAP(_v) ((_v) << 10)
#define DSC_BYTE_SWAP(_v) ((_v) << 9)
#define DSC_WORD_SWAP(_v) ((_v) << 8)
#define DSC_SWAP(_b, _c, _w) ((_b << 10) | (_c << 9) | (_w << 8))
#define DSC_SWAP_MASK ((1 << 10) | (1 << 9) | (1 << 8))
#define DSC_FLATNESS_DET_TH_MASK (0xf << 4)
#define DSC_FLATNESS_DET_TH_F(_v) ((_v) << 4)
#define DSC_SLICE_MODE_CH_MASK (0x1 << 3)
#define DSC_SLICE_MODE_CH_F(_v) ((_v) << 3)
#define DSC_CG_EN_MASK (0x1 << 1)
#define DSC_CG_EN_F(_v) ((_v) << 1)
#define DSC_DUAL_SLICE_EN_MASK (0x1 << 0)
#define DSC_DUAL_SLICE_EN_F(_v) ((_v) << 0)

#define DSC_CONTROL3 0x000C
#define DSC_REMAINDER_F(_v) ((_v) << 12)
#define DSC_REMAINDER_MASK (0x3 << 12)
#define DSC_REMAINDER_GET(_v) (((_v) >> 12) & 0x3)
#define DSC_GRPCNTLINE_F(_v) ((_v) << 0)
#define DSC_GRPCNTLINE_MASK (0x7ff << 0)
#define DSC_GRPCNTLINE_GET(_v) (((_v) >> 0) & 0x7ff)

#define DSC_CRC_0 0x0010
#define DSC_CRC_EN_MASK (0x1 << 16)
#define DSC_CRC_EN_F(_v) ((_v) << 16)
#define DSC_CRC_CODE_MASK (0xffff << 0)
#define DSC_CRC_CODE_F(_v) ((_v) << 0)

#define DSC_CRC_1 0x0014
#define DSC_CRC_Y_S0_MASK (0xffff << 16)
#define DSC_CRC_Y_S0_F(_v) ((_v) << 16)
#define DSC_CRC_CO_S0_MASK (0xffff << 0)
#define DSC_CRC_CO_S0_F(_v) ((_v) << 0)

#define DSC_CRC_2 0x0018
#define DSC_CRC_CG_S0_MASK (0xffff << 16)
#define DSC_CRC_CG_S0_F(_v) ((_v) << 16)
#define DSC_CRC_Y_S1_MASK (0xffff << 0)
#define DSC_CRC_Y_S1_F(_v) ((_v) << 0)

#define DSC_CRC_3 0x001C
#define DSC_CRC_CO_S1_MASK (0xffff << 16)
#define DSC_CRC_CO_S1_F(_v) ((_v) << 16)
#define DSC_CRC_CG_S1_MASK (0xffff << 0)
#define DSC_CRC_CG_S1_F(_v) ((_v) << 0)

#define DSC_PPS00_03 0x0020
#define PPS00_VER(_v) ((_v) << 24)
#define PPS00_VER_MASK (0xff << 24)
#define PPS01_ID(_v) (_v << 16)
#define PPS01_ID_MASK (0xff << 16)
#define PPS03_BPC_MASK (0x00f0 << 0)
#define PPS03_LBD_MASK (0x000f << 0)
#define PPS03_BPC_LBD(_v) (_v << 0)

#define DSC_PPS04_07 0x0024
#define PPS04_COMP_CFG(_v) ((_v) << 24)
#define PPS04_COMP_CFG_MASK (0x3f << 24)
#define PPS05_BPP(_v) (_v << 16)
#define PPS05_BPP_MASK (0xff << 16)
#define PPS06_07_PIC_HEIGHT_MASK (0xffff << 0)
#define PPS06_07_PIC_HEIGHT(_v) (_v << 0)

#define DSC_PPS08_11 0x0028
#define PPS08_09_PIC_WIDHT_MASK (0xffff << 16)
#define PPS08_09_PIC_WIDHT(_v) ((_v) << 16)
#define PPS10_11_SLICE_HEIGHT_MASK (0xffff << 0)
#define PPS10_11_SLICE_HEIGHT(_v) (_v << 0)

#define DSC_PPS12_15 0x002C
#define PPS12_13_SLICE_WIDTH_MASK (0xffff << 16)
#define PPS12_13_SLICE_WIDTH(_v) ((_v) << 16)
#define PPS14_15_CHUNK_SIZE_MASK (0xffff << 0)
#define PPS14_15_CHUNK_SIZE(_v) (_v << 0)

#define DSC_PPS16_19 0x0030
#define PPS16_17_INIT_XMIT_DELAY_MASK (0x3ff << 16)
#define PPS16_17_INIT_XMIT_DELAY(_v) ((_v) << 16)
#define PPS18_19_INIT_DEC_DELAY_MASK (0xffff << 0)
#define PPS18_19_INIT_DEC_DELAY(_v) ((_v) << 0)

#define DSC_PPS20_23 0x0034
#define PPS21_INIT_SCALE_VALUE_MASK (0x3f << 16)
#define PPS21_INIT_SCALE_VALUE(_v) ((_v) << 16)
#define PPS22_23_SCALE_INC_INTERVAL_MASK (0xffff << 0)
#define PPS22_23_SCALE_INC_INTERVAL(_v) (_v << 0)

#define DSC_PPS24_27 0x0038
#define PPS24_25_SCALE_DEC_INTERVAL_MASK (0xfff << 16)
#define PPS24_25_SCALE_DEC_INTERVAL(_v) ((_v) << 16)
/* FL : First Line */
#define PPS27_FL_BPG_OFFSET_MASK (0x1f << 0)
#define PPS27_FL_BPG_OFFSET(_v) (_v << 0)

#define DSC_PPS28_31 0x003C
/* NFL : Not First Line */
#define PPS28_29_NFL_BPG_OFFSET_MASK (0xffff << 16)
#define PPS28_29_NFL_BPG_OFFSET(_v) ((_v) << 16)
#define PPS30_31_SLICE_BPG_OFFSET_MASK (0xffff << 0)
#define PPS30_31_SLICE_BPG_OFFSET(_v) (_v << 0)

#define DSC_PPS32_35 0x0040
#define PPS32_33_INIT_OFFSET_MASK (0xffff << 16)
#define PPS32_33_INIT_OFFSET(_v) ((_v) << 16)
#define PPS34_35_FINAL_OFFSET_MASK (0xffff << 0)
#define PPS34_35_FINAL_OFFSET(_v) (_v << 0)

#define DSC_PPS36_39 0x0044
#define PPS36_FLATNESS_MIN_QP_MASK (0xff << 24)
#define PPS36_FLATNESS_MIN_QP(_v) ((_v) << 24)
#define PPS37_FLATNESS_MAX_QP_MASK (0xff << 16)
#define PPS37_FLATNESS_MAX_QP(_v) ((_v) << 16)
#define PPS38_39_RC_MODEL_SIZE_MASK (0xffff << 0)
#define PPS38_39_RC_MODEL_SIZE(_v) (_v << 0)

#define DSC_PPS40_43 0x0048
#define PPS40_RC_EDGE_FACTOR_MASK (0xff << 24)
#define PPS40_RC_EDGE_FACTOR(_v) ((_v) << 24)
#define PPS41_RC_QUANT_INCR_LIMIT0_MASK (0xff << 16)
#define PPS41_RC_QUANT_INCR_LIMIT0(_v) ((_v) << 16)
#define PPS42_RC_QUANT_INCR_LIMIT1_MASK (0xff << 8)
#define PPS42_RC_QUANT_INCR_LIMIT1(_v) ((_v) << 8)
#define PPS44_RC_TGT_OFFSET_HI_MASK (0xf << 4)
#define PPS44_RC_TGT_OFFSET_HI(_v) ((_v) << 4)
#define PPS44_RC_TGT_OFFSET_LO_MASK (0xf << 0)
#define PPS44_RC_TGT_OFFSET_LO(_v) ((_v) << 0)

#define DSC_PPS44_47 0x004C
#define PPS44_RC_BUF_THRESH_0_MASK (0xff << 24)
#define PPS44_RC_BUF_THRESH_0(_v) ((_v) << 24)
#define PPS45_RC_BUF_THRESH_1_MASK (0xff << 16)
#define PPS45_RC_BUF_THRESH_1(_v) ((_v) << 16)
#define PPS46_RC_BUF_THRESH_2_MASK (0xff << 8)
#define PPS46_RC_BUF_THRESH_3(_v) ((_v) << 8)
#define PPS47_RC_BUF_THRESH_3_MASK (0xff << 0)
#define PPS47_RC_BUF_THRESH_3(_v) ((_v) << 0)

#define DSC_PPS48_51 0x0050
#define PPS48_RC_BUF_THRESH_4_MASK (0xff << 24)
#define PPS48_RC_BUF_THRESH_4(_v) ((_v) << 24)
#define PPS49_RC_BUF_THRESH_5_MASK (0xff << 16)
#define PPS49_RC_BUF_THRESH_5(_v) ((_v) << 16)
#define PPS50_RC_BUF_THRESH_6_MASK (0xff << 8)
#define PPS50_RC_BUF_THRESH_6(_v) ((_v) << 8)
#define PPS51_RC_BUF_THRESH_7_MASK (0xff << 0)
#define PPS51_RC_BUF_THRESH_7(_v) ((_v) << 0)

#define DSC_PPS52_55 0x0054
#define PPS52_RC_BUF_THRESH_8_MASK (0xff << 24)
#define PPS52_RC_BUF_THRESH_8(_v) ((_v) << 24)
#define PPS53_RC_BUF_THRESH_9_MASK (0xff << 16)
#define PPS53_RC_BUF_THRESH_9(_v) ((_v) << 16)
#define PPS54_RC_BUF_THRESH_A_MASK (0xff << 8)
#define PPS54_RC_BUF_THRESH_A(_v) ((_v) << 8)
#define PPS55_RC_BUF_THRESH_B_MASK (0xff << 0)
#define PPS55_RC_BUF_THRESH_B(_v) ((_v) << 0)

#define DSC_PPS56_59 0x0058
#define PPS56_RC_BUF_THRESH_C_MASK (0xff << 24)
#define PPS56_RC_BUF_THRESH_C(_v) ((_v) << 24)
#define PPS57_RC_BUF_THRESH_D_MASK (0xff << 16)
#define PPS57_RC_BUF_THRESH_D(_v) ((_v) << 16)
#define PPS58_RC_RANGE_PARAM_MASK (0xff << 8)
#define PPS58_RC_RANGE_PARAM(_v) (_v << 8)
#define PPS59_RC_RANGE_PARAM_MASK (0xff << 0)
#define PPS59_RC_RANGE_PARAM(_v) (_v << 0)
#define PPS58_59_RC_RANGE_PARAM_MASK (0xFFFF << 0)
#define PPS58_59_RC_RANGE_PARAM(_v) (_v << 0)

#define DSC_PPS60_63 0x005C
#define DSC_PPS64_67 0x0060
#define DSC_PPS68_71 0x0064
#define DSC_PPS72_75 0x0068
#define DSC_PPS76_79 0x006C
#define DSC_PPS80_83 0x0070
#define DSC_PPS84_87 0x0074

#define DSC_DEBUG_EN 0x0078
#define DSC_DBG_EN_MASK (1 << 31)
#define DSC_DBG_EN(_v) ((_v) << 31)
#define DSC_DBG_SEL_MASK (0xffff << 0)
#define DSC_DBG_SEL(_v) ((_v) << 0)

#define DSC_DEBUG_DATA 0x007C

#define DSCC_DEBUG_EN 0x0080
#define DSCC_DBG_EN_MASK (1 << 31)
#define DSCC_DBG_EN(_v) ((_v) << 31)
#define DSCC_DBG_SEL_MASK (0xffff << 0)
#define DSCC_DBG_SEL(_v) ((_v) << 0)

#define DSCC_DEBUG_DATA 0x0084

/*
 * DQE registers (Base Addr : DECON1)
 * ->
 * 0x0000 ~
 * DQE 0 : 0x0000
 * DQE 1 : 0x1000
 *
 * <-
 * DQE registers
 */

#define DQE0_OFFSET 0x0000
#define DQE1_OFFSET 0x1000
#define DQE_OFFSET(_id) ((_id == 0) ? DQE0_OFFSET : DQE1_OFFSET)

#define DQE_CON(_id) (0x0000 + DQE_OFFSET(_id))
#define DQE_LPD_MODE_EXIT(_v) ((_v) << 24)
#define DQE_LPD_MODE_EXIT_MASK (1 << 24)
#define DQE_APS_SW_RESET(_v) ((_v) << 18)
#define DQE_APD_SW_RESET_MASK (1 << 18)
#define DQE_APD_SW_RESET_CLEAR (0 << 18)
#define DQE_HSC_SW_RESET(_v) ((_v) << 16)
#define DQE_HSC_SW_RESET_MASK (1 << 16)
#define DQE_HSC_SW_RESET_CLEAR (0 << 16)
#define PN_NEXT_CALC(_v) ((_v) << 7)
#define PN_NEXT_CALC_NEW (0 << 7)
#define PN_NEXT_CALC_OLD (1 << 7)
#define APS_UPGRADE_ON(_v) ((_v) << 6)
#define APS_UPGRADE_ON_MASK (1 << 6)
#define DQE_APS_ON(_v) ((_v) << 4)
#define DQE_APS_ON_MASK (1 << 4)
#define DQE_HSC_ON(_v) ((_v) << 3)
#define DQE_HSC_ON_MASK (1 << 3)
#define DQE_GAMMA_ON(_v) ((_v) << 2)
#define DQE_GAMMA_ON_MASK (1 << 2)
#define DQE_CGC_ON(_v) ((_v) << 1)
#define DQE_CGC_ON_MASK (1 << 1)

#define DQE_IMG_SIZESET_0(_id) (0x0004 + DQE_OFFSET(_id))
#define DQE_IMG_VSIZE_0(_v) (((_v) & 0x3fff) << 16)
#define DQE_IMG_VSIZE_0_MASK (0x3fff << 16)
#define DQE_IMG_HSIZE_0(_v) (((_v) & 0x3fff) << 0)
#define DQE_IMG_HSIZE_0_MASK (0x3fff << 0)

#define DQE_CGC1_RED(_id) (0x0010 + DQE_OFFSET(_id))
#define DQE_CGC2_RED(_id) (0x0410 + DQE_OFFSET(_id))
#define CGC_R_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_R_R_MASK (0x3ff << 20)
#define CGC_R_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_R_G_MASK (0x3ff << 10)
#define CGC_R_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_R_B_MASK (0x3ff << 0)

#define DQE_CGC1_GREEN(_id) (0x0014 + DQE_OFFSET(_id))
#define DQE_CGC2_GREEN(_id) (0x0414 + DQE_OFFSET(_id))
#define CGC_G_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_G_R_MASK (0x3ff << 20)
#define CGC_G_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_G_G_MASK (0x3ff << 10)
#define CGC_G_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_G_B_MASK (0x3ff << 0)

#define DQE_CGC1_BLUE(_id) (0x0018 + DQE_OFFSET(_id))
#define DQE_CGC2_BLUE(_id) (0x0418 + DQE_OFFSET(_id))
#define CGC_B_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_B_R_MASK (0x3ff << 20)
#define CGC_B_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_B_G_MASK (0x3ff << 10)
#define CGC_B_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_B_B_MASK (0x3ff << 0)

#define DQE_CGC1_CYAN(_id) (0x001C + DQE_OFFSET(_id))
#define DQE_CGC2_CYAN(_id) (0x041C + DQE_OFFSET(_id))
#define CGC_C_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_C_R_MASK (0x3ff << 20)
#define CGC_C_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_C_G_MASK (0x3ff << 10)
#define CGC_C_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_C_B_MASK (0x3ff << 0)

#define DQE_CGC1_MAGENTA(_id) (0x0020 + DQE_OFFSET(_id))
#define DQE_CGC2_MAGENTA(_id) (0x0420 + DQE_OFFSET(_id))
#define CGC_M_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_M_R_MASK (0x3ff << 20)
#define CGC_M_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_M_G_MASK (0x3ff << 10)
#define CGC_M_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_M_B_MASK (0x3ff << 0)

#define DQE_CGC1_YELLOW(_id) (0x0024 + DQE_OFFSET(_id))
#define DQE_CGC2_YELLOW(_id) (0x0424 + DQE_OFFSET(_id))
#define CGC_Y_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_Y_R_MASK (0x3ff << 20)
#define CGC_Y_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_Y_G_MASK (0x3ff << 10)
#define CGC_Y_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_Y_B_MASK (0x3ff << 0)

#define DQE_CGC1_WHITE(_id) (0x0028 + DQE_OFFSET(_id))
#define DQE_CGC2_WHITE(_id) (0x0428 + DQE_OFFSET(_id))
#define CGC_W_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_W_R_MASK (0x3ff << 20)
#define CGC_W_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_W_G_MASK (0x3ff << 10)
#define CGC_W_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_W_B_MASK (0x3ff << 0)

#define DQE_CGC1_BLACK(_id) (0x002C + DQE_OFFSET(_id))
#define DQE_CGC2_BLACK(_id) (0x042C + DQE_OFFSET(_id))
#define CGC_K_R(_v) (((_v) & 0x3ff) << 20)
#define CGC_K_R_MASK (0x3ff << 20)
#define CGC_K_G(_v) (((_v) & 0x3ff) << 10)
#define CGC_K_G_MASK (0x3ff << 10)
#define CGC_K_B(_v) (((_v) & 0x3ff) << 0)
#define CGC_K_B_MASK (0x3ff << 0)

#define DQE_CGC_CONTROL(_id) (0x0030 + DQE_OFFSET(_id))
#define CGC_MC_GAIN(_v) (((_v) & 0xfff) << 4)
#define CGC_MC_GAIN_MASK (0xfff << 4)
#define CGC_MC_EN(_v) ((_v) < 0)
#define CGC_MC_EN_MASK (1 < 0)

#define DQE_GAMMALUT_R_01_00(_id) (0x0034 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_03_02(_id) (0x0038 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_05_04(_id) (0x003C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_07_06(_id) (0x0040 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_09_08(_id) (0x0044 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_11_10(_id) (0x0048 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_13_12(_id) (0x004C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_15_14(_id) (0x0050 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_17_16(_id) (0x0054 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_19_18(_id) (0x0058 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_21_20(_id) (0x005C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_23_22(_id) (0x0060 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_25_24(_id) (0x0064 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_27_26(_id) (0x0068 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_29_28(_id) (0x006C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_31_30(_id) (0x0070 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_33_32(_id) (0x0074 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_35_34(_id) (0x0078 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_37_36(_id) (0x007C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_39_38(_id) (0x0080 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_41_40(_id) (0x0084 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_43_42(_id) (0x0088 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_45_44(_id) (0x008C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_47_46(_id) (0x0090 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_49_48(_id) (0x0094 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_51_50(_id) (0x0098 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_53_52(_id) (0x009C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_55_54(_id) (0x00A0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_57_56(_id) (0x00A4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_59_58(_id) (0x00A8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_61_60(_id) (0x00AC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_63_62(_id) (0x00B0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_R_64(_id) (0x00B4 + DQE_OFFSET(_id))

#define DQE_GAMMALUT_G_01_00(_id) (0x00B8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_03_02(_id) (0x00BC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_05_04(_id) (0x00C0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_07_06(_id) (0x00C4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_09_08(_id) (0x00C8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_11_10(_id) (0x00CC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_13_12(_id) (0x00D0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_15_14(_id) (0x00D4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_17_16(_id) (0x00D8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_19_18(_id) (0x00DC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_21_20(_id) (0x00E0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_23_22(_id) (0x00E4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_25_24(_id) (0x00E8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_27_26(_id) (0x00EC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_29_28(_id) (0x00F0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_31_30(_id) (0x00F4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_33_32(_id) (0x00F8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_35_34(_id) (0x00FC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_37_36(_id) (0x0100 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_39_38(_id) (0x0104 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_41_40(_id) (0x0108 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_43_42(_id) (0x010C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_45_44(_id) (0x0110 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_47_46(_id) (0x0114 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_49_48(_id) (0x0118 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_51_50(_id) (0x011C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_53_52(_id) (0x0120 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_55_54(_id) (0x0124 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_57_56(_id) (0x0128 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_59_58(_id) (0x012C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_61_60(_id) (0x0130 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_63_62(_id) (0x0134 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_G_64(_id) (0x0138 + DQE_OFFSET(_id))

#define DQE_GAMMALUT_B_01_00(_id) (0x013C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_03_02(_id) (0x0140 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_05_04(_id) (0x0144 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_07_06(_id) (0x0148 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_09_08(_id) (0x014C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_11_10(_id) (0x0150 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_13_12(_id) (0x0154 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_15_14(_id) (0x0158 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_17_16(_id) (0x015C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_19_18(_id) (0x0160 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_21_20(_id) (0x0164 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_23_22(_id) (0x0168 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_25_24(_id) (0x016C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_27_26(_id) (0x0170 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_29_28(_id) (0x0174 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_31_30(_id) (0x0178 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_33_32(_id) (0x017C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_35_34(_id) (0x0180 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_37_36(_id) (0x0184 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_39_38(_id) (0x0188 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_41_40(_id) (0x018C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_43_42(_id) (0x0190 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_45_44(_id) (0x0194 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_47_46(_id) (0x0198 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_49_48(_id) (0x019C + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_51_50(_id) (0x01A0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_53_52(_id) (0x01A4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_55_54(_id) (0x01A8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_57_56(_id) (0x01AC + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_59_58(_id) (0x01B0 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_61_60(_id) (0x01B4 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_63_62(_id) (0x01B8 + DQE_OFFSET(_id))
#define DQE_GAMMALUT_B_64(_id) (0x01BC + DQE_OFFSET(_id))

/*
 * ODD : 1,3,5,7,9,...
 * EVEN : 0,2,4,6,8,10,...
	 */

#define GAMMA_R_LUT_ODD(_v) (((_v) & 0x7ff) << 16)
#define GAMMA_R_LUT_ODD_MASK (0x7ff << 16)
#define GAMMA_R_LUT_EVEN(_v) (((_v) & 0x7ff) << 0)
#define GAMMA_R_LUT_EVEN_MASK (0x7ff << 0)

#define DQE_APS_GAIN(_id) (0x01C0 + DQE_OFFSET(_id))
#define APS_ST(_v) (((_v) & 0xff) << 16)
#define APS_ST_MASK (0xff << 16)
#define APS_NS(_v) (((_v) & 0xff) << 8)
#define APS_NS_MASK (0xff << 8)
#define APS_LT(_v) (((_v) & 0xff) << 0)
#define APS_LT_MASK (0xff << 0)

#define DQE_APS_WEIGHT(_id) (0x01C4 + DQE_OFFSET(_id))
#define APS_PL_W2(_v) (((_v) & 0xf) << 16)
#define APS_PL_W2_MASK (0xf << 16)
#define APS_PL_W1(_v) (((_v) & 0xf) << 0)
#define APS_PL_W1_MASK (0xf << 0)

#define DQE_APS_CTMODE(_id) (0x01C8 + DQE_OFFSET(_id))
#define APS_CTMODE(_v) (((_v) & 0x3) << 0)
#define APS_CTMODE_MASK (0x3 << 0)

#define DQE_APS_PPEN(_id) (0x01CC + DQE_OFFSET(_id))
#define APS_PP_EN(_v) ((_v) << 0)
#define APS_PP_EN_MASK (1 << 0)

#define DQE_APS_TDRMINMAX(_id) (0x01D0 + DQE_OFFSET(_id))
#define APS_TDR_MAX(_v) (((_v) & 0x3ff) << 16)
#define APS_TDR_MAX_MASK (0x3ff << 16)
#define APS_TDR_MIN(_v) (((_v) & 0x3ff) << 0)
#define APS_TDR_MIN_MASK (0x3ff << 0)

#define DQE_APS_AMBIENT_LIGHT(_id) (0x01D4 + DQE_OFFSET(_id))
#define APS_AMBIENT_LIGHT(_v) (((_v) & 0xff) << 0)
#define APS_AMBIENT_LIGHT_MASK (0xff << 0)

#define DQE_APS_BACK_LIGHT(_id) (0x01D8 + DQE_OFFSET(_id))
#define APS_BACK_LIGHT(_v) (((_v) & 0xff) << 0)
#define APS_BACK_LIGHT_MASK (0xff << 0)

#define DQE_APS_DSTEP(_id) (0x01DC + DQE_OFFSET(_id))
#define APS_DSTEP(_v) (((_v) & 0x3f) << 0)
#define APS_DSTEP_MASK (0x3f << 0)

#define DQE_APS_THRESHOLD(_id) (0x01E4 + DQE_OFFSET(_id))
#define APS_THRESHOLD_3(_v) (((_v) & 0x3) << 4)
#define APS_THRESHOLD_3_MASK (0x3 << 4)
#define APS_THRESHOLD_2(_v) (((_v) & 0x3) << 2)
#define APS_THRESHOLD_2_MASK (0x3 << 2)
#define APS_THRESHOLD_1(_v) (((_v) & 0x3) << 0)
#define APS_THRESHOLD_1_MASK (0x3 << 0)

#define DQE_APS_GAIN_LIMIT(_id) (0x01E8 + DQE_OFFSET(_id))
#define APS_GAIN_LIMIT(_v) (((_v) & 0x3ff) << 0)
#define APS_GAIN_LIMIT_MASK (0x3ff << 0)

#define DQE_APS_DIMMING_DONE_INTR(_id) (0x01EC + DQE_OFFSET(_id))
#define APS_DIMMING_IN_PROGRESS(_v) ((_v) << 0)
#define APS_DIMMING_IN_PROGRESS_MASK (1 << 0)

#define DQE_APS_LT_CALC_AB_SHIFT(_id) (0x01F0 + DQE_OFFSET(_id))
#define APS_LT_CALC_AB_SHIFT(_v) (((_v) & 0x3) << 0)
#define APS_LT_CALC_AB_SHIFT_MASK (0x3 << 0)

#define DQE_HSC_CONTROL(_id) (0x0204 + DQE_OFFSET(_id))
#define HSC_PPSC_ON(_v) (((_v) & 0x1) << 5)
#define HSC_PPSC_ON_MASK (0x1 << 5)
#define HSC_YCOMP_ON(_v) (((_v) & 0x1) << 4)
#define HSC_YCOMP_ON_MASK (0x1 << 4)
#define HSC_TSC_ON(_v) (((_v) & 0x1) << 3)
#define HSC_TSC_ON_MASK (0x1 << 3)
#define HSC_DITHER_ON(_v) (((_v) & 0x1) << 2)
#define HSC_DITHER_ON_MASK (0x1 << 2)
#define HSC_PPHC_ON(_v) (((_v) & 0x1) << 1)
#define HSC_PPHC_ON_MASK (0x1 << 1)
#define HSC_SKIN_ON(_v) (((_v) & 0x1) << 0)
#define HSC_SKIN_ON_MASK (0x1 << 0)

#define DQE_HSC_PPSCGAIN_RGB(_id) (0x0208 + DQE_OFFSET(_id))
#define HSC_PPSC_GAIN_B(_v) (((_v) & 0x3ff) << 20)
#define HSC_PPSC_GAIN_B_MASK (0x3ff << 20)
#define HSC_PPSC_GAIN_G(_v) (((_v) & 0x3ff) << 10)
#define HSC_PPSC_GAIN_G_MASK (0x3ff << 10)
#define HSC_PPSC_GAIN_R(_v) (((_v) & 0x3ff) << 0)
#define HSC_PPSC_GAIN_R_MASK (0x3ff << 0)

#define DQE_HSC_PPSCGAIN_CMY(_id) (0x020C + DQE_OFFSET(_id))
#define HSC_PPSC_GAIN_Y(_v) (((_v) & 0x3ff) << 20)
#define HSC_PPSC_GAIN_Y_MASK (0x3ff << 20)
#define HSC_PPSC_GAIN_M(_v) (((_v) & 0x3ff) << 10)
#define HSC_PPSC_GAIN_M_MASK (0x3ff << 10)
#define HSC_PPSC_GAIN_C(_v) (((_v) & 0x3ff) << 0)
#define HSC_PPSC_GAIN_C_MASK (0x3ff << 0)

#define DQE_HSC_ALPHASCALE_SHIFT(_id) (0x0210 + DQE_OFFSET(_id))
#define HSC_ALPHA_SHIFT2(_v) (((_v) & 0x1f) << 20)
#define HSC_ALPHA_SHIFT2_MASK (0x1f << 20)
#define HSC_ALPHA_SHIFT1(_v) (((_v) & 0xff) << 8)
#define HSC_ALPHA_SHIFT1_MASK (0xff << 8)
#define HSC_ALPHA_SCALE(_v) (((_v) & 0xf) << 0)
#define HSC_ALPHA_SCALE_MASK (0xf << 0)

#define DQE_HSC_POLY_CURVE0(_id) (0x0214 + DQE_OFFSET(_id))
#define HSC_POLY_CURVE_3(_v) (((_v) & 0x3ff) << 20)
#define HSC_POLY_CURVE_3_MASK (0x3ff << 20)
#define HSC_POLY_CURVE_2(_v) (((_v) & 0x3ff) << 10)
#define HSC_POLY_CURVE_2_MASK (0x3ff << 10)
#define HSC_POLY_CURVE_1(_v) (((_v) & 0x3ff) << 0)
#define HSC_POLY_CURVE_1_MASK (0x3ff << 0)

#define DQE_HSC_POLY_CURVE1(_id) (0x0218 + DQE_OFFSET(_id))
#define HSC_POLY_CURVE_6(_v) (((_v) & 0x3ff) << 20)
#define HSC_POLY_CURVE_6_MASK (0x3ff << 20)
#define HSC_POLY_CURVE_5(_v) (((_v) & 0x3ff) << 10)
#define HSC_POLY_CURVE_5_MASK (0x3ff << 10)
#define HSC_POLY_CURVE_4(_v) (((_v) & 0x3ff) << 0)
#define HSC_POLY_CURVE_4_MASK (0x3ff << 0)

#define DQE_HSC_SKIN_S(_id) (0x021C + DQE_OFFSET(_id))
#define HSC_SKIN_S2(_v) (((_v) & 0x3ff) << 16)
#define HSC_SKIN_S2_MASK (0x3ff << 16)
#define HSC_SKIN_S1(_v) (((_v) & 0x3ff) << 0)
#define HSC_SKIN_S1_MASK (0x3ff << 0)

#define DQE_HSC_PPHCGAIN_RGB(_id) (0x0220 + DQE_OFFSET(_id))
#define HSC_PPHC_GAIN_B(_v) (((_v) & 0x3ff) << 20)
#define HSC_PPHC_GAIN_B_MASK (0x3ff << 20)
#define HSC_PPHC_GAIN_G(_v) (((_v) & 0x3ff) << 10)
#define HSC_PPHC_GAIN_G_MASK (0x3ff << 10)
#define HSC_PPHC_GAIN_R(_v) (((_v) & 0x3ff) << 0)
#define HSC_PPHC_GAIN_R_MASK (0x3ff << 0)

#define DQE_HSC_PPHCGAIN_CMY(_id) (0x0224 + DQE_OFFSET(_id))
#define HSC_PPHC_GAIN_Y(_v) (((_v) & 0x3ff) << 20)
#define HSC_PPHC_GAIN_Y_MASK (0x3ff << 20)
#define HSC_PPHC_GAIN_M(_v) (((_v) & 0x3ff) << 10)
#define HSC_PPHC_GAIN_M_MASK (0x3ff << 10)
#define HSC_PPHC_GAIN_C(_v) (((_v) & 0x3ff) << 0)
#define HSC_PPHC_GAIN_C_MASK (0x3ff << 0)

#define DQE_HSC_TSC_YCOMP(_id) (0x0228 + DQE_OFFSET(_id))
#define HSC_Y_COMP_RATIO(_v) (((_v) & 0xf) << 12)
#define HSC_Y_COMP_RATIO_MASK (0xf << 12)
#define HSC_TSC_GAIN(_v) (((_v) & 0x3ff) << 0)
#define HSC_TSC_GAIN_MASK (0x3ff << 0)

#define DQE_HSC_POLY_CURVE2(_id) (0x022C + DQE_OFFSET(_id))
#define HSC_POLY_CURVE_8(_v) (((_v) & 0x3ff) << 10)
#define HSC_POLY_CURVE_8_MASK (0x3ff << 10)
#define HSC_POLY_CURVE_7(_v) (((_v) & 0x3ff) << 0)
#define HSC_POLY_CURVE_7_MASK (0x3ff << 0)

#define DQE_HSC_PARTIAL_CON(_id) (0x0230 + DQE_OFFSET(_id))
#define HSC_ROI_SAME(_v) (((_v) & 0x1) << 2)
#define HSC_ROI_SAME_MASK (0x1 << 2)
#define HSC_PARTIAL_UPDATE_METHOD(_v) (((_v) & 0x1) << 1)
#define HSC_PARTIAL_UPDATE_METHOD_MASK (0x1 << 1)
#define HSC_IMG_PARTIAL_FRAME(_v) (((_v) & 0x1) << 0)
#define HSC_IMG_PARTIAL_FRAME_MASK (0x1 << 0)

#define DQE_APS_PARTIAL_CON(_id) (0x0234 + DQE_OFFSET(_id))
#define APS_ROI_SAME(_v) (((_v) & 0x1) << 2)
#define APS_ROI_SAME_MASK (0x1 << 2)
#define APS_PARTIAL_UPDATE_METHOD(_v) (((_v) & 0x1) << 1)
#define APS_PARTIAL_UPDATE_METHOD_MASK (0x1 << 1)
#define APS_IMG_PARTIAL_FRAME(_v) (((_v) & 0x1) << 0)
#define APS_IMG_PARTIAL_FRAME_MASK (0x1 << 0)

#define DQE_APS_FULL_IMG_SIZESET(_id) (0x0238 + DQE_OFFSET(_id))
#define APS_FULL_IMG_VSIZE(_v) (((_v) & 0x3fff) << 16)
#define APS_FULL_IMG_VSIZE_MASK (0x3fff << 16)
#define APS_FULL_IMG_HSIZE(_v) (((_v) & 0x3fff) << 0)
#define APS_FULL_IMG_HSIZE_MASK (0x3fff << 0)

#define DQE_APS_PARTIAL_ROI_UP_LEFT_POS(_id) (0x023C + DQE_OFFSET(_id))
#define APS_ROI_Y1(_v) (((_v) & 0x3fff) << 16)
#define APS_ROI_Y1_MASK (0x3fff << 16)
#define APS_ROI_X1(_v) (((_v) & 0x3fff) << 0)
#define APS_ROI_X1_MASK (0x3fff << 0)

#define DQE_HSC_SKIN_H(_id) (0x0240 + DQE_OFFSET(_id))
#define HSC_SKIN_H2(_v) (((_v) & 0x3ff) << 16)
#define HSC_SKIN_H2_MASK (0x3ff << 16)
#define HSC_SKIN_H1(_v) (((_v) & 0x3ff) << 0)
#define HSC_SKIN_H1_MASK (0x3ff << 0)

#define DQE_APS_PARTIAL_IBSI_01_00(_id) (0x0308 + DQE_OFFSET(_id))
#define APS_IBSI_01(_v) (((_v) & 0xffff) << 16)
#define APS_IBSI_01_MASK (0xffff << 16)
#define APS_IBSI_00(_v) (((_v) & 0xffff) << 0)
#define APS_IBSI_00_MASK (0xffff << 0)

#define DQE_APS_PARTIAL_IBSI_11_10(_id) (0x030C + DQE_OFFSET(_id))
#define APS_IBSI_11(_v) (((_v) & 0xffff) << 16)
#define APS_IBSI_11_MASK (0xffff << 16)
#define APS_IBSI_10(_v) (((_v) & 0xffff) << 0)
#define APS_IBSI_10_MASK (0xffff << 0)

#define DQE_HSC_FULL_PXL_NUM(_id) (0x0310 + DQE_OFFSET(_id))
#define HSC_FULL_PXL_NUM(_v) (((_v) & 0xfffffff) << 0)
#define HSC_FULL_PXL_NUM_MASK (0xfffffff << 0)

#define DQE_APS_FULL_PXL_NUM(_id) (0x0320 + DQE_OFFSET(_id))
#define APS_FULL_PXL_NUM(_v) (((_v) & 0xfffffff) << 0)
#define APS_FULL_PXL_NUM_MASK (0xfffffff << 0)

#define DQE_LPD_DATA_CONTROL(_id) (0x0400 + DQE_OFFSET(_id))
#define LPD_WR_OR_RD_DIRECTION(_v) (((_v) & 0x1) << 31)
#define LPD_WR_OR_RD_DIRECTION_MASK (0x1 << 31)
#define LPD_ADDR(_v) (((_v) & 0x7f) << 24)
#define LPD_ADDR_MASK (0x7f << 24)
#define LPD_DATA(_v) (((_v) & 0xfffff) << 0)
#define LPD_DATA_MASK (0xfffff << 0)

#define DQE_VER(_id) (0x0500 + DQE_OFFSET(_id))
#define DQE_VER_F(_v) ((_v) << 0)
#define SQE_VER_GET(_v) (((_v) >> 0) & 0xffffffff)

/*
 * TELLTALE registers (Base Addr : DECON1)
 * ->
 * 0x4000 ~
 * TELLTALE0~15 : 0x4100~0x417C
 * TELLTALE_CRC_DATA0~7 : 0x4200~0x421C
 * FS_CRC_DATA : 0x4220
 *
 * <-
 * TELLTALE registers
 */

#define TELLTALE_CON 0x4000
#define CRC_DATA_SEL_TELLTALE(_v) (((_v) & 0x3) << 20)
#define CRC_DATA_SEL_TELLTALE_R (0x2 << 20)
#define CRC_DATA_SEL_TELLTALE_G (0x1 << 20)
#define CRC_DATA_SEL_TELLTALE_B (0 << 20)
#define FS_CRC_EN(_v) (((_v) & 0x1) << 16)
#define FS_CRC_EN_MASK (1 << 16)
#define ENABLE_TELLTALE_F(_v) (((_v) & 0xffff) << 0)

#define TELLTALE_WIN_RESERVE 0x4004
#define RESERVE_WIN_DECON0(_win) (0 << ((_win) * 4))
#define RESERVE_WIN_DECON1(_win) (1 << ((_win) * 4))
#define RESERVE_WIN_IDLE(_win) (0x3 << ((_win) * 4))
#define RESERVE_WIN7(_v) (((_v) & 0x3) << 28)
#define RESERVE_WIN7_DECON0 (0 << 28)
#define RESERVE_WIN7_DECON1 (1 << 28)
#define RESERVE_WIN7_IDLE (0x3 << 28)
#define RESERVE_WIN6(_v) (((_v) & 0x3) << 24)
#define RESERVE_WIN6_DECON0 (0 << 24)
#define RESERVE_WIN6_DECON1 (1 << 24)
#define RESERVE_WIN6_IDLE (0x3 << 24)
#define RESERVE_WIN5(_v) (((_v) & 0x3) << 20)
#define RESERVE_WIN5_DECON0 (0 << 20)
#define RESERVE_WIN5_DECON1 (1 << 20)
#define RESERVE_WIN5_IDLE (0x3 << 20)
#define RESERVE_WIN4(_v) (((_v) & 0x3) << 16)
#define RESERVE_WIN4_DECON0 (0 << 16)
#define RESERVE_WIN4_DECON1 (1 << 16)
#define RESERVE_WIN4_IDLE (0x3 << 16)
#define RESERVE_WIN3(_v) (((_v) & 0x3) << 12)
#define RESERVE_WIN3_DECON0 (0 << 12)
#define RESERVE_WIN3_DECON1 (1 << 12)
#define RESERVE_WIN3_IDLE (0x3 << 12)
#define RESERVE_WIN2(_v) (((_v) & 0x3) << 8)
#define RESERVE_WIN2_DECON0 (0 << 8)
#define RESERVE_WIN2_DECON1 (1 << 8)
#define RESERVE_WIN2_IDLE (0x3 << 8)
#define RESERVE_WIN1(_v) (((_v) & 0x3) << 4)
#define RESERVE_WIN1_DECON0 (0 << 4)
#define RESERVE_WIN1_DECON1 (1 << 4)
#define RESERVE_WIN1_IDLE (0x3 << 4)
#define RESERVE_WIN0(_v) (((_v) & 0x3) << 0)
#define RESERVE_WIN0_DECON0 (0 << 0)
#define RESERVE_WIN0_DECON1 (1 << 0)
#define RESERVE_WIN0_IDLE (0x3 << 0)

#define TELLTALE_START_POSITION(_v) (0x4100 + ((_v) * 8))
#define TELLTALE_STRPTR_Y_F(_v) (((_v) & 0x3fff) << 16)
#define TELLTALE_STRPTR_X_F(_v) (((_v) & 0x3fff) << 0)

#define TELLTALE_END_POSITION(_v) (0x4104 + ((_v) * 8))
#define TELLTALE_ENDPTR_Y_F(_v) (((_v) & 0x3fff) << 16)
#define TELLTALE_ENDPTR_X_F(_v) (((_v) & 0x3fff) << 0)

#define TELLTALE_CRC_DATA(_v) (0x4200 + ((_v) * 4))
#define TELLTALE_ODD_CRC_DATA(_v) (((_v) & 0xffff) << 16)
#define TELLTALE_ODD_DATA_GET(_v) (((_v) >> 16) & 0xffff)
#define TELLTALE_EVEN_CRC_DATA(_v) (((_v) & 0xffff) << 0)
#define TELLTALE_EVEN_CRC_DATA_GET(_v) (((_v) >> 0) & 0xffff)

#define FS_CRC_DATA 0x4220
#define FS_CRC_DATA_F(_v) (((_v) & 0xffff) << 0)
#define FS_CRC_DATA_GET(_v) (((_v) >> 0) & 0xffff)

#define DATA_PATH_CONTROL_WIN (0x0000)

#define UPDATE_CONTROL_WIN (0x0010)
#define SHD_UP_REQ_MASK_ALLOW (1 << 0)
#define SHD_UP_REQ_MASK_BLOCK (1 << 1)

#define UPDATE_REQ_WIN (0x0014)
#define SHD_UP_REQ (1 << 0)

#endif
