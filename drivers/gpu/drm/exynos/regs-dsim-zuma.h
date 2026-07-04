/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) Samsung Electronics Co., Ltd.
 * Copyright (c) Google LLC
 *
 * Register map for the Google Tensor "zuma"/"zumapro" MIPI-DSIM link,
 * derived from the vendor cal_9865 DSIM register definitions. Covers the DSIM
 * link registers, the integrated DCPHY (PLL / clock lane / data lanes / bias)
 * and the DPU SYSREG bit needed for a full cold-init DCPHY bring-up.
 */

#ifndef _REGS_DSIM_ZUMA_H_
#define _REGS_DSIM_ZUMA_H_

#define DSIM_VERSION			0x0000

#define DSIM_SWRST			0x0004
#define DSIM_DPHY_RST			(1 << 16)
#define DSIM_SWRST_FUNCRST		(1 << 8)
#define DSIM_SWRST_RESET		(1 << 0)

#define DSIM_DPHY_STATUS		0x001c
#define DSIM_DPHY_STATUS_TX_READY_HSCLK		(1 << 10)

#define DSIM_CLK_CTRL			0x0020
#define DSIM_CLK_CTRL_CLOCK_SEL			(1 << 26)
#define DSIM_CLK_CTRL_NONCONT_CLOCK_LANE	(1 << 25)
#define DSIM_CLK_CTRL_CLKLANE_ONOFF		(1 << 24)
#define DSIM_CLK_CTRL_TX_REQUEST_HSCLK		(1 << 20)
#define DSIM_CLK_CTRL_WORDCLK_EN		(1 << 17)
#define DSIM_CLK_CTRL_ESCCLK_EN			(1 << 16)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN(x)		((x) << 8)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK	(0x1f << 8)
#define DSIM_CLK_CTRL_ESC_PRESCALER(x)		((x) << 0)
#define DSIM_CLK_CTRL_ESC_PRESCALER_MASK	(0xff << 0)

/* clock lane + N data lanes; komodo uses 4 data lanes -> 0x1f */
#define DSIM_LANE_CLOCK				(1 << 0)

#define DSIM_DESKEW_CTRL		0x0024
#define DSIM_DESKEW_CTRL_HW_EN			(1 << 15)
#define DSIM_DESKEW_CTRL_HW_POSITION(x)		((x) << 14)
#define DSIM_DESKEW_CTRL_HW_POSITION_MASK	(1 << 14)
#define DSIM_DESKEW_CTRL_HW_INTERVAL(x)		((x) << 2)
#define DSIM_DESKEW_CTRL_HW_INTERVAL_MASK	(0xfff << 2)

#define DSIM_TIMEOUT			0x0028
#define DSIM_TIMEOUT_BTA_TOUT(x)		((x) << 16)
#define DSIM_TIMEOUT_LPRX_TOUT(x)		((x) << 0)

#define DSIM_ESCMODE			0x002c
#define DSIM_ESCMODE_STOP_STATE_CNT(x)		((x) << 21)
#define DSIM_ESCMODE_STOP_STATE_CNT_MASK	(0x7ff << 21)
#define DSIM_ESCMODE_CMD_LPDT			(1 << 7)

#define DSIM_NUM_OF_TRANSFER		0x0030
#define DSIM_NUM_OF_TRANSFER_PER_FRAME(x)	((x) << 0)

#define DSIM_UNDERRUN_CTRL		0x0034
#define DSIM_UNDERRUN_CTRL_CM_UNDERRUN_LP_REF(x)	((x) << 0)
#define DSIM_UNDERRUN_CTRL_CM_UNDERRUN_LP_REF_MASK	(0xffff << 0)

#define DSIM_THRESHOLD			0x0038
#define DSIM_THRESHOLD_LEVEL(x)			((x) << 0)

#define DSIM_RESOL			0x003c
#define DSIM_RESOL_VRESOL(x)			(((x) & 0x1fff) << 16)
#define DSIM_RESOL_HRESOL(x)			(((x) & 0x1fff) << 0)

#define DSIM_CONFIG			0x004c
#define DSIM_CONFIG_CPRS_EN			(1 << 19)
#define DSIM_CONFIG_VIDEO_MODE			(1 << 18)
#define DSIM_CONFIG_VC_ID(x)			((x) << 15)
#define DSIM_CONFIG_VC_ID_MASK			(0x3 << 15)
#define DSIM_CONFIG_PIXEL_FORMAT(x)		((x) << 9)
#define DSIM_CONFIG_PIXEL_FORMAT_MASK		(0x3f << 9)
#define DSIM_CONFIG_PER_FRAME_READ_EN(x)	((x) << 8)
#define DSIM_CONFIG_PER_FRAME_READ_EN_MASK	(1 << 8)
#define DSIM_CONFIG_EOTP_EN(x)			((x) << 7)
#define DSIM_CONFIG_EOTP_EN_MASK		(1 << 7)
#define DSIM_CONFIG_NUM_OF_DATA_LANE(x)		((x) << 5)
#define DSIM_CONFIG_NUM_OF_DATA_LANE_MASK	(0x3 << 5)
#define DSIM_CONFIG_LANES_EN(x)			(((x) & 0x1f) << 0)

#define DSIM_INTSRC			0x0050
#define DSIM_INTMSK			0x0054
#define DSIM_INTMSK_SW_RST_RELEASE		(1 << 30)
#define DSIM_INTMSK_SFR_PL_FIFO_EMPTY		(1 << 29)
#define DSIM_INTMSK_SFR_PH_FIFO_EMPTY		(1 << 28)
#define DSIM_INTMSK_FRAME_DONE			(1 << 24)
#define DSIM_INTMSK_INVALID_SFR_VALUE		(1 << 23)
#define DSIM_INTMSK_UNDER_RUN			(1 << 19)
#define DSIM_INTMSK_RX_DATA_DONE		(1 << 18)
#define DSIM_INTMSK_ERR_RX_ECC			(1 << 15)

/* Command transfer FIFO (DCS/generic writes to the panel) */
#define DSIM_PKTHDR			0x0058
#define DSIM_PKTHDR_BTA_TYPE(x)			((x) << 24)
#define DSIM_PKTHDR_DATA1(x)			((x) << 16)
#define DSIM_PKTHDR_DATA0(x)			((x) << 8)
#define DSIM_PKTHDR_ID(x)			((x) << 0)

#define DSIM_PAYLOAD			0x005c

#define DSIM_FIFOCTRL			0x0068
#define DSIM_FIFOCTRL_EMPTY_PH_SFR		(1 << 10)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR		(1 << 8)

#define DSIM_SFR_CTRL			0x0064
#define DSIM_SFR_CTRL_SHADOW_REG_READ_EN	(1 << 1)
#define DSIM_SFR_CTRL_SHADOW_EN			(1 << 0)

#define DSIM_CPRS_CTRL			0x0074
#define DSIM_CPRS_CTRL_MULI_SLICE_PACKET(x)	((x) << 3)
#define DSIM_CPRS_CTRL_NUM_OF_SLICE(x)		((x) << 0)

#define DSIM_SLICE01			0x0078
#define DSIM_SLICE01_SIZE_OF_SLICE1(x)		((x) << 16)
#define DSIM_SLICE01_SIZE_OF_SLICE0(x)		((x) << 0)

#define DSIM_CMD_CONFIG			0x0080
#define DSIM_CMD_CONFIG_PKT_GO_EN		(1 << 16)

#define DSIM_CMD_TE_CTRL0		0x0084
#define DSIM_CMD_TE_CTRL0_TIME_STABLE_VFP(x)	((x) << 0)

#define DSIM_CMD_TE_CTRL1		0x0088
#define DSIM_CMD_TE_CTRL1_TIME_TE_PROTECT_ON(x)	((x) << 16)
#define DSIM_CMD_TE_CTRL1_TIME_TE_TOUT(x)	((x) << 0)

#define DSIM_INTSRC_PLL_STABLE			(1 << 31)

#define DSIM_SLICE23			0x007c
#define DSIM_SLICE23_SIZE_OF_SLICE3(x)		((x) << 16)
#define DSIM_SLICE23_SIZE_OF_SLICE2(x)		((x) << 0)

#define DSIM_CMD_CONFIG_PKT_GO_RDY		(1 << 17)
#define DSIM_CMD_CONFIG_MULTI_CMD_PKT_EN	(1 << 8)

#define DSIM_OPTION_SUITE		0x010c
#define DSIM_OPTION_SUITE_OPT_TE_ON_CMD_ALLOW	(1 << 10)

/* ---- integrated DCPHY: PLL / clock lane / data lanes (reg-names "dphy") ---- */
#define DSIM_PHY_PLL_CON(i)		(0x0000 + 4 * (i))
#define DSIM_PHY_PLL_CON0		0x0000
#define DSIM_PHY_PLL_CON0_PLL_EN		(1 << 12)
#define DSIM_PHY_PLL_CON0_PMS_S(x)		((x) << 8)
#define DSIM_PHY_PLL_CON0_PMS_S_MASK		(0x7 << 8)
#define DSIM_PHY_PLL_CON0_PMS_P(x)		((x) << 0)
#define DSIM_PHY_PLL_CON0_PMS_P_MASK		(0x3f << 0)
#define DSIM_PHY_PLL_CON1		0x0004
#define DSIM_PHY_PLL_CON1_PMS_K(x)		((x) << 0)
#define DSIM_PHY_PLL_CON1_PMS_K_MASK		(0xffff << 0)
#define DSIM_PHY_PLL_CON2		0x0008
#define DSIM_PHY_PLL_CON2_PMS_M(x)		((x) << 0)
#define DSIM_PHY_PLL_CON2_PMS_M_MASK		(0x3ff << 0)
#define DSIM_PHY_PLL_CON5		0x0014
#define DSIM_PHY_PLL_CON5_DITHER_SEL_VCO	(1 << 2)
#define DSIM_PHY_PLL_CON6		0x0018
#define DSIM_PHY_PLL_CON6_WCLK_BUF_SFT_CNT(x)	((x) << 8)
#define DSIM_PHY_PLL_CON6_WCLK_BUF_SFT_CNT_MASK	(0xf << 8)
#define DSIM_PHY_PLL_CON7		0x001c
#define DSIM_PHY_PLL_CON7_PLL_LOCK_CNT(x)	((x) << 0)
#define DSIM_PHY_PLL_STAT0		0x0040
#define DSIM_PHY_PLL_STAT0_PLL_LOCK		(1 << 0)

/* clock lane (MC) */
#define DSIM_PHY_MC_GNR_CON0		0x0200
#define DSIM_PHY_MC_GNR_CON1		0x0204
#define DSIM_PHY_MC_ANA_CON0		0x0208
#define DSIM_PHY_MC_ANA_CON1		0x020c
#define DSIM_PHY_MC_ANA_CON2		0x0210
#define DSIM_PHY_MC_ANA_CON3		0x0214
#define DSIM_PHY_MC_TIME_CON0		0x0230
#define DSIM_PHY_MC_TIME_CON1		0x0234
#define DSIM_PHY_MC_TIME_CON2		0x0238
#define DSIM_PHY_MC_TIME_CON3		0x023c
#define DSIM_PHY_MC_TIME_CON4		0x0240
#define DSIM_PHY_MC_DESKEW_CON0		0x0250

/* data lanes (MD), x = 0..3 */
#define DSIM_PHY_MD_OFFSET(x)		(0x0300 + 0x100 * (x))
#define DSIM_PHY_MD_GNR_CON0(x)		(DSIM_PHY_MD_OFFSET(x) + 0x00)
#define DSIM_PHY_MD_GNR_CON1(x)		(DSIM_PHY_MD_OFFSET(x) + 0x04)
#define DSIM_PHY_MD_ANA_CON0(x)		(DSIM_PHY_MD_OFFSET(x) + 0x08)
#define DSIM_PHY_MD_ANA_CON1(x)		(DSIM_PHY_MD_OFFSET(x) + 0x0c)
#define DSIM_PHY_MD_ANA_CON2(x)		(DSIM_PHY_MD_OFFSET(x) + 0x10)
#define DSIM_PHY_MD_ANA_CON3(x)		(DSIM_PHY_MD_OFFSET(x) + 0x14)
#define DSIM_PHY_MD_TIME_CON0(x)	(DSIM_PHY_MD_OFFSET(x) + 0x30)
#define DSIM_PHY_MD_TIME_CON1(x)	(DSIM_PHY_MD_OFFSET(x) + 0x34)
#define DSIM_PHY_MD_TIME_CON2(x)	(DSIM_PHY_MD_OFFSET(x) + 0x38)
#define DSIM_PHY_MD_TIME_CON3(x)	(DSIM_PHY_MD_OFFSET(x) + 0x3c)
#define DSIM_PHY_MD_TIME_CON4(x)	(DSIM_PHY_MD_OFFSET(x) + 0x40)

/* lane general-control bits (shared MC/MD GNR_CON0) */
#define DSIM_PHY_GNR_PHY_READY			(1 << 1)
#define DSIM_PHY_GNR_PHY_ENABLE			(1 << 0)

/* D-PHY HS timing field packing (shared MC/MD) */
#define DSIM_PHY_TIME_CON0_HSTX_CLK_SEL		(1 << 12)
#define DSIM_PHY_TIME_CON0_TLPX(x)		((x) << 4)
#define DSIM_PHY_MC_TIME_CON1_TCLK_ZERO(x)	((x) << 8)
#define DSIM_PHY_MC_TIME_CON1_TCLK_PREPARE(x)	((x) << 0)
#define DSIM_PHY_MC_TIME_CON2_THS_EXIT(x)	((x) << 8)
#define DSIM_PHY_MC_TIME_CON2_TCLK_TRAIL(x)	((x) << 0)
#define DSIM_PHY_MC_TIME_CON3_TCLK_POST(x)	((x) << 0)
#define DSIM_PHY_MD_TIME_CON1_THS_ZERO(x)	((x) << 8)
#define DSIM_PHY_MD_TIME_CON1_THS_PREPARE(x)	((x) << 0)
#define DSIM_PHY_MD_TIME_CON2_THS_EXIT(x)	((x) << 8)
#define DSIM_PHY_MD_TIME_CON2_THS_TRAIL(x)	((x) << 0)
#define DSIM_PHY_MD_TIME_CON3_TTA_GET(x)	((x) << 4)
#define DSIM_PHY_TIME_CON4_ULPS_EXIT(x)		((x) << 0)

/* ---- DCPHY bias (reg-names "dphy-extra", shared by DSIM0/1) ---- */
#define DSIM_PHY_BIAS_CON(i)		(0x0000 + 4 * (i))

/*
 * DPU SYSREG DPHY reset control. The register lives in the DPU sysreg
 * (google,gs101-dpu-sysreg) at absolute 0x19421008, i.e. offset 0x1008 from
 * the sysreg base - accessed through a syscon phandle (samsung,disp-sysreg).
 */
#define DISP_DPU_MIPI_PHY_CON		0x1008
#define DISP_DPU_SEL_RESET_DPHY(v)		(1 << (4 + (v)))
#define DISP_DPU_M_RESETN_M0			(1 << 0)

/* vendor constants (cal_9865 dsim_reg.c) */
#define DSIM_PIXEL_FORMAT_RGB24		0x3e
#define DSIM_BTA_TIMEOUT		0xff
#define DSIM_LP_RX_TIMEOUT		0xffff
#define DSIM_STOP_STATE_CNT		0xa
#define DSIM_STABLE_VFP_VALUE		2
#define DSIM_TE_MARGIN			5	/* 5% */

#endif /* _REGS_DSIM_ZUMA_H_ */
