// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung MIPI DSIM bridge driver.
 *
 * Copyright (C) 2021 Amarula Solutions(India)
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 *
 * Based on exynos_drm_dsi from
 * Tomasz Figa <t.figa@samsung.com>
 */

#include <linux/unaligned.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/irq.h>
#include <linux/media-bus-format.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/time64.h>
#include <linux/units.h>

#include <video/mipi_display.h>

#include <drm/bridge/samsung-dsim.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/* returns true iff both arguments logically differs */
#define NEQV(a, b) (!(a) ^ !(b))

/* DSIM_STATUS or DSIM_DPHY_STATUS */
#define DSIM_STOP_STATE_DAT(x)		(((x) & 0xf) << 0)
#define DSIM_STOP_STATE_CLK		BIT(8)
#define DSIM_TX_READY_HS_CLK		BIT(10)

/* DSIM_SWRST */
#define DSIM_FUNCRST			BIT(16)
#define DSIM_SWRST			BIT(0)

/* DSIM_TIMEOUT */
#define DSIM_LPDR_TIMEOUT(x)		((x) << 0)
#define DSIM_BTA_TIMEOUT(x)		((x) << 16)

/* DSIM_CLKCTRL */
#define DSIM_ESC_PRESCALER(x)			(((x) & 0xffff) << 0)
#define DSIM_ESC_PRESCALER_MASK			(0xffff << 0)
#define DSIM_LANE_ESC_CLK_EN_DATA(x, offset)	(((x) & 0xf) << offset)
#define DSIM_LANE_ESC_CLK_EN_DATA_MASK(offset)	(0xf << offset)
#define DSIM_BYTE_CLK_SRC(x)			(((x) & 0x3) << 25)
#define DSIM_BYTE_CLK_SRC_MASK			(0x3 << 25)
#define DSIM_PLL_BYPASS				BIT(27)

/* DSIM_CONFIG */
#define DSIM_LANE_EN_CLK		BIT(0)
#define DSIM_LANE_EN(x)			(((x) & 0xf) << 1)
#define DSIM_NUM_OF_DATA_LANE(x)	(((x) & 0x3) << 5)
#define DSIM_SUB_PIX_FORMAT(x)		(((x) & 0x7) << 8)
#define DSIM_MAIN_PIX_FORMAT_MASK	(0x7 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB888	(0x7 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB666	(0x6 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB666_P	(0x5 << 12)
#define DSIM_MAIN_PIX_FORMAT_RGB565	(0x4 << 12)
#define DSIM_SUB_VC			(((x) & 0x3) << 16)
#define DSIM_MAIN_VC			(((x) & 0x3) << 18)
#define DSIM_HSA_DISABLE_MODE		BIT(20)
#define DSIM_HBP_DISABLE_MODE		BIT(21)
#define DSIM_HFP_DISABLE_MODE		BIT(22)
/*
 * The i.MX 8M Mini Applications Processor Reference Manual,
 * Rev. 3, 11/2020 Page 4091
 * The i.MX 8M Nano Applications Processor Reference Manual,
 * Rev. 2, 07/2022 Page 3058
 * The i.MX 8M Plus Applications Processor Reference Manual,
 * Rev. 1, 06/2021 Page 5436
 * all claims this bit is 'HseDisableMode' with the definition
 * 0 = Disables transfer
 * 1 = Enables transfer
 *
 * This clearly states that HSE is not a disabled bit.
 *
 * The naming convention follows as per the manual and the
 * driver logic is based on the MIPI_DSI_MODE_VIDEO_HSE flag.
 */
#define DSIM_HSE_DISABLE_MODE		BIT(23)
#define DSIM_AUTO_MODE			BIT(24)
#define DSIM_BURST_MODE			BIT(26)
#define DSIM_SYNC_INFORM		BIT(27)
#define DSIM_EOT_DISABLE		BIT(28)
#define DSIM_MFLUSH_VS			BIT(29)
/* This flag is valid only for exynos3250/3472/5260/5430 */
#define DSIM_CLKLANE_STOP		BIT(30)
#define DSIM_NON_CONTINUOUS_CLKLANE	BIT(31)

/* DSIM_ESCMODE */
#define DSIM_TX_TRIGGER_RST		BIT(4)
#define DSIM_TX_LPDT_LP			BIT(6)
#define DSIM_CMD_LPDT_LP		BIT(7)
#define DSIM_FORCE_BTA			BIT(16)
#define DSIM_FORCE_STOP_STATE		BIT(20)
#define DSIM_STOP_STATE_CNT(x)		(((x) & 0x7ff) << 21)
#define DSIM_STOP_STATE_CNT_MASK	(0x7ff << 21)

/* DSIM_MDRESOL */
#define DSIM_MAIN_STAND_BY		BIT(31)
#define DSIM_MAIN_VRESOL(x, num_bits)	(((x) & ((1 << (num_bits)) - 1)) << 16)
#define DSIM_MAIN_HRESOL(x, num_bits)	(((x) & ((1 << (num_bits)) - 1)) << 0)

/* DSIM_MVPORCH */
#define DSIM_CMD_ALLOW(x)		((x) << 28)
#define DSIM_STABLE_VFP(x)		((x) << 16)
#define DSIM_MAIN_VBP(x)		((x) << 0)
#define DSIM_CMD_ALLOW_MASK		(0xf << 28)
#define DSIM_STABLE_VFP_MASK		(0x7ff << 16)
#define DSIM_MAIN_VBP_MASK		(0x7ff << 0)

/* DSIM_MHPORCH */
#define DSIM_MAIN_HFP(x)		((x) << 16)
#define DSIM_MAIN_HBP(x)		((x) << 0)
#define DSIM_MAIN_HFP_MASK		((0xffff) << 16)
#define DSIM_MAIN_HBP_MASK		((0xffff) << 0)

/* DSIM_MSYNC */
#define DSIM_MAIN_VSA(x, offset)	((x) << offset)
#define DSIM_MAIN_HSA(x)		((x) << 0)
#define DSIM_MAIN_VSA_MASK(offset)	((0x3ff) << offset)
#define DSIM_MAIN_HSA_MASK		((0xffff) << 0)

/* DSIM_SDRESOL */
#define DSIM_SUB_STANDY(x)		((x) << 31)
#define DSIM_SUB_VRESOL(x)		((x) << 16)
#define DSIM_SUB_HRESOL(x)		((x) << 0)
#define DSIM_SUB_STANDY_MASK		((0x1) << 31)
#define DSIM_SUB_VRESOL_MASK		((0x7ff) << 16)
#define DSIM_SUB_HRESOL_MASK		((0x7ff) << 0)

/* DSIM_INTSRC */
#define DSIM_INT_PLL_STABLE		BIT(31)
#define DSIM_INT_SW_RST_RELEASE		BIT(30)
#define DSIM_INT_SFR_FIFO_EMPTY		BIT(29)
#define DSIM_INT_SFR_HDR_FIFO_EMPTY	BIT(28)
#define DSIM_INT_BTA			BIT(25)
#define DSIM_INT_FRAME_DONE		BIT(24)
#define DSIM_INT_RX_TIMEOUT		BIT(21)
#define DSIM_INT_BTA_TIMEOUT		BIT(20)
#define DSIM_INT_RX_DONE		BIT(18)
#define DSIM_INT_RX_TE			BIT(17)
#define DSIM_INT_RX_ACK			BIT(16)
#define DSIM_INT_RX_ECC_ERR		BIT(15)
#define DSIM_INT_RX_CRC_ERR		BIT(14)

/* DSIM_SFRCTRL */
#define DSIM_SFR_CTRL_STAND_BY		BIT(4)
#define DSIM_SFR_CTRL_SHADOW_UPDATE	BIT(1)
#define DSIM_SFR_CTRL_SHADOW_EN		BIT(0)

/* DSIM_FIFOCTRL */
#define DSIM_RX_DATA_FULL		BIT(25)
#define DSIM_RX_DATA_EMPTY		BIT(24)
#define DSIM_SFR_HEADER_FULL		BIT(23)
#define DSIM_SFR_HEADER_EMPTY		BIT(22)
#define DSIM_SFR_PAYLOAD_FULL		BIT(21)
#define DSIM_SFR_PAYLOAD_EMPTY		BIT(20)
#define DSIM_I80_HEADER_FULL		BIT(19)
#define DSIM_I80_HEADER_EMPTY		BIT(18)
#define DSIM_I80_PAYLOAD_FULL		BIT(17)
#define DSIM_I80_PAYLOAD_EMPTY		BIT(16)
#define DSIM_SD_HEADER_FULL		BIT(15)
#define DSIM_SD_HEADER_EMPTY		BIT(14)
#define DSIM_SD_PAYLOAD_FULL		BIT(13)
#define DSIM_SD_PAYLOAD_EMPTY		BIT(12)
#define DSIM_MD_HEADER_FULL		BIT(11)
#define DSIM_MD_HEADER_EMPTY		BIT(10)
#define DSIM_MD_PAYLOAD_FULL		BIT(9)
#define DSIM_MD_PAYLOAD_EMPTY		BIT(8)
#define DSIM_RX_FIFO			BIT(4)
#define DSIM_SFR_FIFO			BIT(3)
#define DSIM_I80_FIFO			BIT(2)
#define DSIM_SD_FIFO			BIT(1)
#define DSIM_MD_FIFO			BIT(0)

/* DSIM_PHYACCHR */
#define DSIM_AFC_EN			BIT(14)
#define DSIM_AFC_CTL(x)			(((x) & 0x7) << 5)

/* DSIM_PLLCTRL */
#define DSIM_PLL_DPDNSWAP_CLK		(1 << 25)
#define DSIM_PLL_DPDNSWAP_DAT		(1 << 24)
#define DSIM_FREQ_BAND(x)		((x) << 24)
#define DSIM_PLL_EN			BIT(23)
#define DSIM_PLL(x, offset)		((x) << (offset))

/* DSIM_PHYCTRL */
#define DSIM_PHYCTRL_ULPS_EXIT(x)	(((x) & 0x1ff) << 0)
#define DSIM_PHYCTRL_B_DPHYCTL_VREG_LP	BIT(30)
#define DSIM_PHYCTRL_B_DPHYCTL_SLEW_UP	BIT(14)

/* DSIM_PHYTIMING */
#define DSIM_PHYTIMING_LPX(x)		((x) << 8)
#define DSIM_PHYTIMING_HS_EXIT(x)	((x) << 0)

/* DSIM_PHYTIMING1 */
#define DSIM_PHYTIMING1_CLK_PREPARE(x)	((x) << 24)
#define DSIM_PHYTIMING1_CLK_ZERO(x)	((x) << 16)
#define DSIM_PHYTIMING1_CLK_POST(x)	((x) << 8)
#define DSIM_PHYTIMING1_CLK_TRAIL(x)	((x) << 0)

/* DSIM_PHYTIMING2 */
#define DSIM_PHYTIMING2_HS_PREPARE(x)	((x) << 16)
#define DSIM_PHYTIMING2_HS_ZERO(x)	((x) << 8)
#define DSIM_PHYTIMING2_HS_TRAIL(x)	((x) << 0)

/* Zumapro DSIM registers keep the PLL/timing controls in the external M4M4 D-PHY. */
#define DSIM_ZUMAPRO_NUM_OF_TRANSFER		0x0030
#define DSIM_ZUMAPRO_UNDERRUN_CTRL		0x0034
#define DSIM_ZUMAPRO_THRESHOLD			0x0038
#define DSIM_ZUMAPRO_CPRS_CTRL			0x0074
#define DSIM_ZUMAPRO_SLICE01			0x0078
#define DSIM_ZUMAPRO_SLICE23			0x007c
#define DSIM_ZUMAPRO_CMD_TE_CTRL0		0x0084
#define DSIM_ZUMAPRO_CMD_TE_CTRL1		0x0088
#define DSIM_ZUMAPRO_VPORCH			0x0104
#define DSIM_ZUMAPRO_VFP_DETAIL		0x0108
#define DSIM_ZUMAPRO_OPTION_SUITE		0x010c

#define DSIM_ZUMAPRO_CLKCTRL_NONCONT_CLOCK_LANE	BIT(25)
#define DSIM_ZUMAPRO_CLKCTRL_LANE_ESCCLK_EN(x)	(((x) & 0x1f) << 8)
#define DSIM_ZUMAPRO_CLKCTRL_LANE_ESCCLK_MASK	(0x1f << 8)
#define DSIM_ZUMAPRO_CLKCTRL_ESC_PRESCALER_MASK	(0xff << 0)

#define DSIM_ZUMAPRO_CONFIG_CPRS_EN		BIT(19)
#define DSIM_ZUMAPRO_CONFIG_VIDEO_MODE		BIT(18)
#define DSIM_ZUMAPRO_CONFIG_VC_ID_MASK		(0x3 << 15)
#define DSIM_ZUMAPRO_CONFIG_RGB24		(0x3e << 9)
#define DSIM_ZUMAPRO_CONFIG_PIXEL_FORMAT_MASK	(0x3f << 9)
#define DSIM_ZUMAPRO_CONFIG_PER_FRAME_READ	BIT(8)
#define DSIM_ZUMAPRO_CONFIG_EOTP_EN		BIT(7)
#define DSIM_ZUMAPRO_CONFIG_DATA_LANE_NUM(x)	(((x) & 0x3) << 5)
#define DSIM_ZUMAPRO_CONFIG_DATA_LANE_NUM_MASK	(0x3 << 5)
#define DSIM_ZUMAPRO_CONFIG_LANES_EN(x)		(((x) & 0x1f) << 0)
#define DSIM_ZUMAPRO_CONFIG_LANES_EN_MASK	(0x1f << 0)
#define DSIM_ZUMAPRO_CONFIG_OWNED_MASK		\
	(DSIM_ZUMAPRO_CONFIG_CPRS_EN |		\
	 DSIM_ZUMAPRO_CONFIG_VIDEO_MODE |	\
	 DSIM_ZUMAPRO_CONFIG_VC_ID_MASK |	\
	 DSIM_ZUMAPRO_CONFIG_PIXEL_FORMAT_MASK |	\
	 DSIM_ZUMAPRO_CONFIG_PER_FRAME_READ |	\
	 DSIM_ZUMAPRO_CONFIG_EOTP_EN |		\
	 DSIM_ZUMAPRO_CONFIG_DATA_LANE_NUM_MASK | \
	 DSIM_ZUMAPRO_CONFIG_LANES_EN_MASK |	\
	 DSIM_HSE_DISABLE_MODE)

#define DSIM_ZUMAPRO_CPRS_MULTI_SLICE		BIT(3)
#define DSIM_ZUMAPRO_CPRS_NUM_OF_SLICE(x)	(((x) & 0x7) << 0)
#define DSIM_ZUMAPRO_UNDERRUN_LP_REF(x)		(((x) & 0xffff) << 0)
#define DSIM_ZUMAPRO_SLICE_SIZE1(x)		(((x) & 0x1fff) << 16)
#define DSIM_ZUMAPRO_SLICE_SIZE0(x)		(((x) & 0x1fff) << 0)
#define DSIM_ZUMAPRO_VFP_CMD_ALLOW(x)		(((x) & 0xffff) << 16)
#define DSIM_ZUMAPRO_STABLE_VFP(x)		(((x) & 0xffff) << 0)
#define DSIM_ZUMAPRO_CMD_TE_PROTECT_ON(x)	(((x) & 0xffff) << 16)
#define DSIM_ZUMAPRO_CMD_TE_TIMEOUT(x)		(((x) & 0xffff) << 0)
#define DSIM_ZUMAPRO_OPT_TE_ON_CMD_ALLOW	BIT(10)
#define DSIM_ZUMAPRO_CLKCTRL_CLOCK_SEL		BIT(26)
#define DSIM_ZUMAPRO_SHADOW_REG_READ_EN	BIT(1)

#define DSIM_ZUMAPRO_STABLE_VFP_DSC_PERCENT	2
#define DSIM_ZUMAPRO_TE_IDLE_US		1000
#define DSIM_ZUMAPRO_TE_VAR_PERCENT		1
#define DSIM_ZUMAPRO_TE_MARGIN_PERCENT		5

#define DSI_MAX_BUS_WIDTH		4
#define DSI_NUM_VIRTUAL_CHANNELS	4
#define DSI_TX_FIFO_SIZE		2048
#define DSI_RX_FIFO_SIZE		256
#define DSI_XFER_TIMEOUT_MS		100
#define DSI_RX_FIFO_EMPTY		0x30800002

#define PS_TO_CYCLE(ps, hz) DIV64_U64_ROUND_CLOSEST(((ps) * (hz)), 1000000000000ULL)

enum samsung_dsim_transfer_type {
	EXYNOS_DSI_TX,
	EXYNOS_DSI_RX,
};

static struct clk_bulk_data exynos3_clk_bulk_data[] = {
	{ .id = "bus_clk" },
	{ .id = "pll_clk" },
};

static struct clk_bulk_data exynos4_clk_bulk_data[] = {
	{ .id = "bus_clk" },
	{ .id = "sclk_mipi" },
};

static struct clk_bulk_data exynos5433_clk_bulk_data[] = {
	{ .id = "bus_clk" },
	{ .id = "sclk_mipi" },
	{ .id = "phyclk_mipidphy0_bitclkdiv8" },
	{ .id = "phyclk_mipidphy0_rxclkesc0" },
	{ .id = "sclk_rgb_vclk_to_dsim0" },
};

static struct clk_bulk_data exynos7870_clk_bulk_data[] = {
	{ .id = "bus" },
	{ .id = "pll" },
	{ .id = "byte" },
	{ .id = "esc" },
};

static struct clk_bulk_data zumapro_clk_bulk_data[] = {
	{ .id = "bus" },
	{ .id = "osc" },
};

enum reg_idx {
	DSIM_STATUS_REG,	/* Status register (legacy) */
	DSIM_LINK_STATUS_REG,	/* Link status register */
	DSIM_DPHY_STATUS_REG,	/* D-PHY status register */
	DSIM_SWRST_REG,		/* Software reset register */
	DSIM_CLKCTRL_REG,	/* Clock control register */
	DSIM_TIMEOUT_REG,	/* Time out register */
	DSIM_CONFIG_REG,	/* Configuration register */
	DSIM_ESCMODE_REG,	/* Escape mode register */
	DSIM_MDRESOL_REG,
	DSIM_MVPORCH_REG,	/* Main display Vporch register */
	DSIM_MHPORCH_REG,	/* Main display Hporch register */
	DSIM_MSYNC_REG,		/* Main display sync area register */
	DSIM_INTSRC_REG,	/* Interrupt source register */
	DSIM_INTMSK_REG,	/* Interrupt mask register */
	DSIM_PKTHDR_REG,	/* Packet Header FIFO register */
	DSIM_PAYLOAD_REG,	/* Payload FIFO register */
	DSIM_RXFIFO_REG,	/* Read FIFO register */
	DSIM_SFRCTRL_REG,	/* SFR standby and shadow control register */
	DSIM_FIFOCTRL_REG,	/* FIFO status and control register */
	DSIM_PLLCTRL_REG,	/* PLL control register */
	DSIM_PHYCTRL_REG,
	DSIM_PHYTIMING_REG,
	DSIM_PHYTIMING1_REG,
	DSIM_PHYTIMING2_REG,
	NUM_REGS
};

static const unsigned int exynos_reg_ofs[] = {
	[DSIM_STATUS_REG] =  0x00,
	[DSIM_SWRST_REG] =  0x04,
	[DSIM_CLKCTRL_REG] =  0x08,
	[DSIM_TIMEOUT_REG] =  0x0c,
	[DSIM_CONFIG_REG] =  0x10,
	[DSIM_ESCMODE_REG] =  0x14,
	[DSIM_MDRESOL_REG] =  0x18,
	[DSIM_MVPORCH_REG] =  0x1c,
	[DSIM_MHPORCH_REG] =  0x20,
	[DSIM_MSYNC_REG] =  0x24,
	[DSIM_INTSRC_REG] =  0x2c,
	[DSIM_INTMSK_REG] =  0x30,
	[DSIM_PKTHDR_REG] =  0x34,
	[DSIM_PAYLOAD_REG] =  0x38,
	[DSIM_RXFIFO_REG] =  0x3c,
	[DSIM_FIFOCTRL_REG] =  0x44,
	[DSIM_PLLCTRL_REG] =  0x4c,
	[DSIM_PHYCTRL_REG] =  0x5c,
	[DSIM_PHYTIMING_REG] =  0x64,
	[DSIM_PHYTIMING1_REG] =  0x68,
	[DSIM_PHYTIMING2_REG] =  0x6c,
};

static const unsigned int exynos5433_reg_ofs[] = {
	[DSIM_STATUS_REG] = 0x04,
	[DSIM_SWRST_REG] = 0x0C,
	[DSIM_CLKCTRL_REG] = 0x10,
	[DSIM_TIMEOUT_REG] = 0x14,
	[DSIM_CONFIG_REG] = 0x18,
	[DSIM_ESCMODE_REG] = 0x1C,
	[DSIM_MDRESOL_REG] = 0x20,
	[DSIM_MVPORCH_REG] = 0x24,
	[DSIM_MHPORCH_REG] = 0x28,
	[DSIM_MSYNC_REG] = 0x2C,
	[DSIM_INTSRC_REG] = 0x34,
	[DSIM_INTMSK_REG] = 0x38,
	[DSIM_PKTHDR_REG] = 0x3C,
	[DSIM_PAYLOAD_REG] = 0x40,
	[DSIM_RXFIFO_REG] = 0x44,
	[DSIM_FIFOCTRL_REG] = 0x4C,
	[DSIM_PLLCTRL_REG] = 0x94,
	[DSIM_PHYCTRL_REG] = 0xA4,
	[DSIM_PHYTIMING_REG] = 0xB4,
	[DSIM_PHYTIMING1_REG] = 0xB8,
	[DSIM_PHYTIMING2_REG] = 0xBC,
};

static const unsigned int exynos7870_reg_ofs[] = {
	[DSIM_LINK_STATUS_REG] = 0x04,
	[DSIM_DPHY_STATUS_REG] = 0x08,
	[DSIM_SWRST_REG] = 0x0C,
	[DSIM_CLKCTRL_REG] = 0x10,
	[DSIM_TIMEOUT_REG] = 0x14,
	[DSIM_ESCMODE_REG] = 0x1C,
	[DSIM_MDRESOL_REG] = 0x20,
	[DSIM_MVPORCH_REG] = 0x24,
	[DSIM_MHPORCH_REG] = 0x28,
	[DSIM_MSYNC_REG] = 0x2C,
	[DSIM_CONFIG_REG] = 0x30,
	[DSIM_INTSRC_REG] = 0x34,
	[DSIM_INTMSK_REG] = 0x38,
	[DSIM_PKTHDR_REG] = 0x3C,
	[DSIM_PAYLOAD_REG] = 0x40,
	[DSIM_RXFIFO_REG] = 0x44,
	[DSIM_SFRCTRL_REG] = 0x48,
	[DSIM_FIFOCTRL_REG] = 0x4C,
	[DSIM_PLLCTRL_REG] = 0x94,
	[DSIM_PHYCTRL_REG] = 0xA4,
	[DSIM_PHYTIMING_REG] = 0xB4,
	[DSIM_PHYTIMING1_REG] = 0xB8,
	[DSIM_PHYTIMING2_REG] = 0xBC,
};

static const unsigned int zumapro_reg_ofs[] = {
	[DSIM_LINK_STATUS_REG] = 0x14,
	[DSIM_DPHY_STATUS_REG] = 0x1c,
	[DSIM_SWRST_REG] = 0x04,
	[DSIM_CLKCTRL_REG] = 0x20,
	[DSIM_TIMEOUT_REG] = 0x28,
	[DSIM_ESCMODE_REG] = 0x2c,
	[DSIM_MDRESOL_REG] = 0x3c,
	[DSIM_MHPORCH_REG] = 0x44,
	[DSIM_MSYNC_REG] = 0x48,
	[DSIM_CONFIG_REG] = 0x4c,
	[DSIM_INTSRC_REG] = 0x50,
	[DSIM_INTMSK_REG] = 0x54,
	[DSIM_PKTHDR_REG] = 0x58,
	[DSIM_PAYLOAD_REG] = 0x5c,
	[DSIM_RXFIFO_REG] = 0x60,
	[DSIM_SFRCTRL_REG] = 0x64,
	[DSIM_FIFOCTRL_REG] = 0x68,
};

enum reg_value_idx {
	RESET_TYPE,
	PLL_TIMER,
	STOP_STATE_CNT,
	PHYCTRL_ULPS_EXIT,
	PHYCTRL_VREG_LP,
	PHYCTRL_SLEW_UP,
	PHYTIMING_LPX,
	PHYTIMING_HS_EXIT,
	PHYTIMING_CLK_PREPARE,
	PHYTIMING_CLK_ZERO,
	PHYTIMING_CLK_POST,
	PHYTIMING_CLK_TRAIL,
	PHYTIMING_HS_PREPARE,
	PHYTIMING_HS_ZERO,
	PHYTIMING_HS_TRAIL
};

static const unsigned int reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 500,
	[STOP_STATE_CNT] = 0xf,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0x0af),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x06),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0b),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x07),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x27),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0d),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x08),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x09),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x0d),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0b),
};

static const unsigned int exynos5422_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 500,
	[STOP_STATE_CNT] = 0xf,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0xaf),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x08),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0d),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x09),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x30),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0e),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x0a),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x0c),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x11),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0d),
};

static const unsigned int exynos5433_reg_values[] = {
	[RESET_TYPE] = DSIM_FUNCRST,
	[PLL_TIMER] = 22200,
	[STOP_STATE_CNT] = 0xa,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0x190),
	[PHYCTRL_VREG_LP] = DSIM_PHYCTRL_B_DPHYCTL_VREG_LP,
	[PHYCTRL_SLEW_UP] = DSIM_PHYCTRL_B_DPHYCTL_SLEW_UP,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x07),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0c),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x09),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x2d),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0e),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x09),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x0b),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x10),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0c),
};

static const unsigned int exynos7870_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 80000,
	[STOP_STATE_CNT] = 0xa,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0x177),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x07),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0c),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x08),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x2b),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0d),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x09),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x09),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x0f),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0c),
};

static const unsigned int zumapro_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[STOP_STATE_CNT] = 0xa,
};

static const unsigned int imx8mm_dsim_reg_values[] = {
	[RESET_TYPE] = DSIM_SWRST,
	[PLL_TIMER] = 500,
	[STOP_STATE_CNT] = 0xf,
	[PHYCTRL_ULPS_EXIT] = DSIM_PHYCTRL_ULPS_EXIT(0xaf),
	[PHYCTRL_VREG_LP] = 0,
	[PHYCTRL_SLEW_UP] = 0,
	[PHYTIMING_LPX] = DSIM_PHYTIMING_LPX(0x06),
	[PHYTIMING_HS_EXIT] = DSIM_PHYTIMING_HS_EXIT(0x0b),
	[PHYTIMING_CLK_PREPARE] = DSIM_PHYTIMING1_CLK_PREPARE(0x07),
	[PHYTIMING_CLK_ZERO] = DSIM_PHYTIMING1_CLK_ZERO(0x26),
	[PHYTIMING_CLK_POST] = DSIM_PHYTIMING1_CLK_POST(0x0d),
	[PHYTIMING_CLK_TRAIL] = DSIM_PHYTIMING1_CLK_TRAIL(0x08),
	[PHYTIMING_HS_PREPARE] = DSIM_PHYTIMING2_HS_PREPARE(0x08),
	[PHYTIMING_HS_ZERO] = DSIM_PHYTIMING2_HS_ZERO(0x0d),
	[PHYTIMING_HS_TRAIL] = DSIM_PHYTIMING2_HS_TRAIL(0x0b),
};

static const struct samsung_dsim_driver_data exynos3_dsi_driver_data = {
	.reg_ofs = exynos_reg_ofs,
	.plltmr_reg = 0x50,
	.has_legacy_status_reg = 1,
	.has_freqband = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos3_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos3_clk_bulk_data),
	.max_freq = 1000,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 11,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
	.has_broken_fifoctrl_emptyhdr = 1,
};

static const struct samsung_dsim_driver_data exynos4_dsi_driver_data = {
	.reg_ofs = exynos_reg_ofs,
	.plltmr_reg = 0x50,
	.has_legacy_status_reg = 1,
	.has_freqband = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos4_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos4_clk_bulk_data),
	.max_freq = 1000,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 11,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
	.has_broken_fifoctrl_emptyhdr = 1,
};

static const struct samsung_dsim_driver_data exynos5_dsi_driver_data = {
	.reg_ofs = exynos_reg_ofs,
	.plltmr_reg = 0x58,
	.has_legacy_status_reg = 1,
	.clk_data = exynos3_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos3_clk_bulk_data),
	.max_freq = 1000,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 11,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data exynos5433_dsi_driver_data = {
	.reg_ofs = exynos5433_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_legacy_status_reg = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos5433_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos5433_clk_bulk_data),
	.max_freq = 1500,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 0,
	.num_bits_resol = 12,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = exynos5433_reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data exynos5422_dsi_driver_data = {
	.reg_ofs = exynos5433_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_legacy_status_reg = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos3_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos3_clk_bulk_data),
	.max_freq = 1500,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 1,
	.num_bits_resol = 12,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	.pll_p_offset = 13,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = exynos5422_reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data exynos7870_dsi_driver_data = {
	.reg_ofs = exynos7870_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_clklane_stop = 1,
	.has_sfrctrl = 1,
	.clk_data = exynos7870_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos7870_clk_bulk_data),
	.max_freq = 1500,
	.wait_for_hdr_fifo = 0,
	.wait_for_reset = 1,
	.num_bits_resol = 12,
	.video_mode_bit = 18,
	.pll_stable_bit = 24,
	.esc_clken_bit = 16,
	.byte_clken_bit = 17,
	.tx_req_hsclk_bit = 20,
	.lane_esc_clk_bit = 8,
	.lane_esc_data_offset = 9,
	.pll_p_offset = 13,
	.pll_m_offset = 3,
	.pll_s_offset = 0,
	.main_vsa_offset = 16,
	.reg_values = exynos7870_reg_values,
	.pll_fin_min = 6,
	.pll_fin_max = 12,
	.m_min = 41,
	.m_max = 125,
	.min_freq = 500,
};

static const struct samsung_dsim_driver_data zumapro_dsi_driver_data = {
	.reg_ofs = zumapro_reg_ofs,
	.clk_data = zumapro_clk_bulk_data,
	.num_clks = ARRAY_SIZE(zumapro_clk_bulk_data),
	.wait_for_hdr_fifo = 0,
	.wait_for_reset = 0,
	.num_bits_resol = 13,
	.video_mode_bit = 18,
	.pll_stable_bit = 0,
	.esc_clken_bit = 16,
	.byte_clken_bit = 17,
	.tx_req_hsclk_bit = 20,
	.lane_esc_clk_bit = 8,
	.lane_esc_data_offset = 9,
	.main_vsa_offset = 16,
	.reg_values = zumapro_reg_values,
	.has_zumapro_regs = 1,
	.uses_external_dphy_pll = 1,
};

static const struct samsung_dsim_driver_data imx8mm_dsi_driver_data = {
	.reg_ofs = exynos5433_reg_ofs,
	.plltmr_reg = 0xa0,
	.has_legacy_status_reg = 1,
	.has_clklane_stop = 1,
	.clk_data = exynos4_clk_bulk_data,
	.num_clks = ARRAY_SIZE(exynos4_clk_bulk_data),
	.max_freq = 2100,
	.wait_for_hdr_fifo = 1,
	.wait_for_reset = 0,
	.num_bits_resol = 12,
	.video_mode_bit = 25,
	.pll_stable_bit = 31,
	.esc_clken_bit = 28,
	.byte_clken_bit = 24,
	.tx_req_hsclk_bit = 31,
	.lane_esc_clk_bit = 19,
	.lane_esc_data_offset = 20,
	/*
	 * Unlike Exynos, PLL_P(PMS_P) offset 14 is used in i.MX8M Mini/Nano/Plus
	 * downstream driver - drivers/gpu/drm/bridge/sec-dsim.c
	 */
	.pll_p_offset = 14,
	.pll_m_offset = 4,
	.pll_s_offset = 1,
	.main_vsa_offset = 22,
	.reg_values = imx8mm_dsim_reg_values,
	.pll_fin_min = 2,
	.pll_fin_max = 30,
	.m_min = 64,
	.m_max = 1023,
	.min_freq = 1050,
};

static const struct samsung_dsim_driver_data *
samsung_dsim_types[DSIM_TYPE_COUNT] = {
	[DSIM_TYPE_EXYNOS3250] = &exynos3_dsi_driver_data,
	[DSIM_TYPE_EXYNOS4210] = &exynos4_dsi_driver_data,
	[DSIM_TYPE_EXYNOS5410] = &exynos5_dsi_driver_data,
	[DSIM_TYPE_EXYNOS5422] = &exynos5422_dsi_driver_data,
	[DSIM_TYPE_EXYNOS5433] = &exynos5433_dsi_driver_data,
	[DSIM_TYPE_EXYNOS7870] = &exynos7870_dsi_driver_data,
	[DSIM_TYPE_ZUMAPRO] = &zumapro_dsi_driver_data,
	[DSIM_TYPE_IMX8MM] = &imx8mm_dsi_driver_data,
	[DSIM_TYPE_IMX8MP] = &imx8mm_dsi_driver_data,
};

static inline struct samsung_dsim *host_to_dsi(struct mipi_dsi_host *h)
{
	return container_of(h, struct samsung_dsim, dsi_host);
}

static inline struct samsung_dsim *bridge_to_dsi(struct drm_bridge *b)
{
	return container_of(b, struct samsung_dsim, bridge);
}

static inline void samsung_dsim_write(struct samsung_dsim *dsi,
				      enum reg_idx idx, u32 val)
{
	writel(val, dsi->reg_base + dsi->driver_data->reg_ofs[idx]);
}

static inline u32 samsung_dsim_read(struct samsung_dsim *dsi, enum reg_idx idx)
{
	return readl(dsi->reg_base + dsi->driver_data->reg_ofs[idx]);
}

static inline void samsung_dsim_write_offset(struct samsung_dsim *dsi,
					     u32 offset, u32 val)
{
	writel(val, dsi->reg_base + offset);
}

static inline u32 samsung_dsim_read_offset(struct samsung_dsim *dsi, u32 offset)
{
	return readl(dsi->reg_base + offset);
}

static void samsung_dsim_zumapro_select_word_clock(struct samsung_dsim *dsi,
						   bool enable)
{
	u32 reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);

	if (enable)
		reg |= DSIM_ZUMAPRO_CLKCTRL_CLOCK_SEL;
	else
		reg &= ~DSIM_ZUMAPRO_CLKCTRL_CLOCK_SEL;

	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);
}

static inline bool samsung_dsim_init_on_transfer(enum samsung_dsim_type hw)
{
	return samsung_dsim_hw_is_exynos(hw);
}

static void samsung_dsim_wait_for_reset(struct samsung_dsim *dsi)
{
	if (wait_for_completion_timeout(&dsi->completed, msecs_to_jiffies(300)))
		return;

	dev_err(dsi->dev, "timeout waiting for reset\n");
}

static void samsung_dsim_reset(struct samsung_dsim *dsi)
{
	u32 reset_val = dsi->driver_data->reg_values[RESET_TYPE];
	int timeout;

	if (dsi->driver_data->has_zumapro_regs) {
		samsung_dsim_write(dsi, DSIM_SWRST_REG, reset_val);

		timeout = 200;
		do {
			if (!(samsung_dsim_read(dsi, DSIM_SWRST_REG) & reset_val))
				return;

			udelay(10);
		} while (--timeout);

		dev_err(dsi->dev, "timeout waiting for Zumapro reset\n");
		return;
	}

	reinit_completion(&dsi->completed);
	samsung_dsim_write(dsi, DSIM_SWRST_REG, reset_val);
}

static unsigned long samsung_dsim_pll_find_pms(struct samsung_dsim *dsi,
					       unsigned long fin,
					       unsigned long fout,
					       u8 *p, u16 *m, u8 *s)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long best_freq = 0;
	u32 min_delta = 0xffffffff;
	u8 p_min, p_max;
	u8 _p, best_p;
	u16 _m, best_m;
	u8 _s, best_s;

	p_min = DIV_ROUND_UP(fin, (driver_data->pll_fin_max * HZ_PER_MHZ));
	p_max = fin / (driver_data->pll_fin_min * HZ_PER_MHZ);

	for (_p = p_min; _p <= p_max; ++_p) {
		for (_s = 0; _s <= 5; ++_s) {
			u64 tmp;
			u32 delta;

			tmp = (u64)fout * (_p << _s);
			do_div(tmp, fin);
			_m = tmp;
			if (_m < driver_data->m_min || _m > driver_data->m_max)
				continue;

			tmp = (u64)_m * fin;
			do_div(tmp, _p);
			if (tmp < driver_data->min_freq  * HZ_PER_MHZ ||
			    tmp > driver_data->max_freq * HZ_PER_MHZ)
				continue;

			tmp = (u64)_m * fin;
			do_div(tmp, _p << _s);

			delta = abs(fout - tmp);
			if (delta < min_delta) {
				best_p = _p;
				best_m = _m;
				best_s = _s;
				min_delta = delta;
				best_freq = tmp;
			}
		}
	}

	if (best_freq) {
		*p = best_p;
		*m = best_m;
		*s = best_s;
	}

	return best_freq;
}

static unsigned long samsung_dsim_set_pll(struct samsung_dsim *dsi,
					  unsigned long freq)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long fin, fout;
	int timeout;
	u8 p, s;
	u16 m;
	u32 reg;

	if (dsi->pll_clk) {
		/*
		 * Ensure that the reference clock is generated with a power of
		 * two divider from its parent, but close to the PLLs upper
		 * limit.
		 */
		fin = clk_get_rate(clk_get_parent(dsi->pll_clk));
		while (fin > driver_data->pll_fin_max * HZ_PER_MHZ)
			fin /= 2;
		clk_set_rate(dsi->pll_clk, fin);

		fin = clk_get_rate(dsi->pll_clk);
	} else {
		fin = dsi->pll_clk_rate;
	}
	dev_dbg(dsi->dev, "PLL ref clock freq %lu\n", fin);

	fout = samsung_dsim_pll_find_pms(dsi, fin, freq, &p, &m, &s);
	if (!fout) {
		dev_err(dsi->dev,
			"failed to find PLL PMS for requested frequency\n");
		return 0;
	}
	dev_dbg(dsi->dev, "PLL freq %lu, (p %d, m %d, s %d)\n", fout, p, m, s);

	writel(driver_data->reg_values[PLL_TIMER],
	       dsi->reg_base + driver_data->plltmr_reg);

	reg = DSIM_PLL_EN | DSIM_PLL(p, driver_data->pll_p_offset)
			  | DSIM_PLL(m, driver_data->pll_m_offset)
			  | DSIM_PLL(s, driver_data->pll_s_offset);

	if (driver_data->has_freqband) {
		static const unsigned long freq_bands[] = {
			100 * HZ_PER_MHZ, 120 * HZ_PER_MHZ, 160 * HZ_PER_MHZ,
			200 * HZ_PER_MHZ, 270 * HZ_PER_MHZ, 320 * HZ_PER_MHZ,
			390 * HZ_PER_MHZ, 450 * HZ_PER_MHZ, 510 * HZ_PER_MHZ,
			560 * HZ_PER_MHZ, 640 * HZ_PER_MHZ, 690 * HZ_PER_MHZ,
			770 * HZ_PER_MHZ, 870 * HZ_PER_MHZ, 950 * HZ_PER_MHZ,
		};
		int band;

		for (band = 0; band < ARRAY_SIZE(freq_bands); ++band)
			if (fout < freq_bands[band])
				break;

		dev_dbg(dsi->dev, "band %d\n", band);

		reg |= DSIM_FREQ_BAND(band);
	}

	if (dsi->swap_dn_dp_clk)
		reg |= DSIM_PLL_DPDNSWAP_CLK;
	if (dsi->swap_dn_dp_data)
		reg |= DSIM_PLL_DPDNSWAP_DAT;

	samsung_dsim_write(dsi, DSIM_PLLCTRL_REG, reg);

	timeout = 3000;
	do {
		if (timeout-- == 0) {
			dev_err(dsi->dev, "PLL failed to stabilize\n");
			return 0;
		}
		if (driver_data->has_legacy_status_reg)
			reg = samsung_dsim_read(dsi, DSIM_STATUS_REG);
		else
			reg = samsung_dsim_read(dsi, DSIM_LINK_STATUS_REG);
	} while ((reg & BIT(driver_data->pll_stable_bit)) == 0);

	dsi->hs_clock = fout;

	return fout;
}

static int samsung_dsim_calc_clocks(struct samsung_dsim *dsi,
				    unsigned long *ret_esc_div)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long hs_clk, byte_clk, esc_clk, pix_clk;
	unsigned long esc_div;
	struct drm_display_mode *m = &dsi->mode;
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	/* m->clock is in KHz */
	pix_clk = m->clock * 1000;

	/* Use burst_clk_rate if available, otherwise use the pix_clk */
	if (dsi->burst_clk_rate)
		hs_clk = dsi->burst_clk_rate;
	else
		hs_clk = DIV_ROUND_UP(pix_clk * bpp, dsi->lanes);

	if (driver_data->uses_external_dphy_pll)
		dsi->hs_clock = hs_clk;
	else
		hs_clk = samsung_dsim_set_pll(dsi, hs_clk);

	if (!hs_clk) {
		dev_err(dsi->dev, "failed to configure DSI PLL\n");
		return -EFAULT;
	}

	byte_clk = driver_data->has_zumapro_regs ? hs_clk / 16 : hs_clk / 8;
	esc_div = DIV_ROUND_UP(byte_clk, dsi->esc_clk_rate);
	esc_clk = byte_clk / esc_div;

	if (!driver_data->has_zumapro_regs && esc_clk > 20 * HZ_PER_MHZ) {
		++esc_div;
		esc_clk = byte_clk / esc_div;
	}
	dsi->esc_clock = esc_clk;

	dev_dbg(dsi->dev, "hs_clk = %lu, byte_clk = %lu, esc_clk = %lu\n",
		hs_clk, byte_clk, esc_clk);

	*ret_esc_div = esc_div;

	return 0;
}

static int samsung_dsim_enable_clock(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	unsigned long esc_div;
	u32 reg;
	int ret;

	ret = samsung_dsim_calc_clocks(dsi, &esc_div);
	if (ret)
		return ret;

	if (driver_data->has_zumapro_regs) {
		u32 lanes_mask = BIT(dsi->lanes + 1) - 1;

		if (esc_div > 0xff) {
			dev_err(dsi->dev, "escape clock divider is too large\n");
			return -EINVAL;
		}

		reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
		reg &= ~(DSIM_ZUMAPRO_CLKCTRL_ESC_PRESCALER_MASK |
			 DSIM_ZUMAPRO_CLKCTRL_LANE_ESCCLK_MASK |
			 BIT(driver_data->esc_clken_bit) |
			 BIT(driver_data->byte_clken_bit) |
			 BIT(driver_data->tx_req_hsclk_bit) |
			 DSIM_ZUMAPRO_CLKCTRL_NONCONT_CLOCK_LANE |
			 DSIM_ZUMAPRO_CLKCTRL_CLOCK_SEL);
		reg |= DSIM_ESC_PRESCALER(esc_div) |
		       DSIM_ZUMAPRO_CLKCTRL_LANE_ESCCLK_EN(lanes_mask) |
		       BIT(driver_data->esc_clken_bit) |
		       BIT(driver_data->byte_clken_bit);

		/*
		 * The HS-clock request must not be asserted before the external
		 * D-PHY PLL has locked: requesting the HS clock out of an
		 * unlocked PLL prevents it from locking.  The bootloader and
		 * downstream dsim_reg_init() defer TX_REQUEST_HSCLK until after
		 * the PLL lock poll (in dsim_reg_start()).  For the external-PLL
		 * cold bring-up it is asserted after phy_power_on() in
		 * samsung_dsim_init(); the bootloader-adopt path sets it in
		 * samsung_dsim_zumapro_start_handoff_link().  Internal PLLs lock
		 * independently of this request, so keep the combined write.
		 */
		if (!driver_data->uses_external_dphy_pll)
			reg |= BIT(driver_data->tx_req_hsclk_bit);

		if (dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
			reg |= DSIM_ZUMAPRO_CLKCTRL_NONCONT_CLOCK_LANE;

		samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);
		return 0;
	}

	reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
	reg &= ~(DSIM_ESC_PRESCALER_MASK | BIT(driver_data->lane_esc_clk_bit)
		| DSIM_LANE_ESC_CLK_EN_DATA_MASK(driver_data->lane_esc_data_offset)
		| DSIM_PLL_BYPASS
		| DSIM_BYTE_CLK_SRC_MASK);
	reg |= BIT(driver_data->esc_clken_bit) | BIT(driver_data->byte_clken_bit)
		| DSIM_ESC_PRESCALER(esc_div)
		| BIT(driver_data->lane_esc_clk_bit)
		| DSIM_LANE_ESC_CLK_EN_DATA(BIT(dsi->lanes) - 1,
					    driver_data->lane_esc_data_offset)
		| DSIM_BYTE_CLK_SRC(0)
		| BIT(driver_data->tx_req_hsclk_bit);
	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);

	return 0;
}

static int samsung_dsim_configure_external_phy(struct samsung_dsim *dsi)
{
	union phy_configure_opts phy_opts = { };
	int ret;

	if (!dsi->driver_data->uses_external_dphy_pll)
		return 0;

	if (!dsi->phy)
		return dev_err_probe(dsi->dev, -ENODEV,
				     "external D-PHY is required\n");

	phy_opts.mipi_dphy.hs_clk_rate = dsi->hs_clock;
	phy_opts.mipi_dphy.lp_clk_rate = dsi->esc_clock;
	phy_opts.mipi_dphy.lanes = dsi->lanes;

	ret = phy_init(dsi->phy);
	if (ret)
		return ret;

	ret = phy_configure(dsi->phy, &phy_opts);
	if (ret)
		goto err_exit_phy;

	ret = phy_power_on(dsi->phy);
	if (ret)
		goto err_exit_phy;

	samsung_dsim_zumapro_select_word_clock(dsi, true);

	return 0;

err_exit_phy:
	phy_exit(dsi->phy);
	return ret;
}

static void samsung_dsim_set_phy_ctrl(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	const unsigned int *reg_values = driver_data->reg_values;
	u32 reg;
	struct phy_configure_opts_mipi_dphy cfg;
	int clk_prepare, lpx, clk_zero, clk_post, clk_trail;
	int hs_exit, hs_prepare, hs_zero, hs_trail;
	unsigned long long byte_clock = dsi->hs_clock / 8;

	if (driver_data->has_freqband || driver_data->uses_external_dphy_pll)
		return;

	phy_mipi_dphy_get_default_config_for_hsclk(dsi->hs_clock,
						   dsi->lanes, &cfg);

	/*
	 * TODO:
	 * The tech Applications Processor manuals for i.MX8M Mini, Nano,
	 * and Plus don't state what the definition of the PHYTIMING
	 * bits are beyond their address and bit position.
	 * After reviewing NXP's downstream code, it appears
	 * that the various PHYTIMING registers take the number
	 * of cycles and use various dividers on them.  This
	 * calculation does not result in an exact match to the
	 * downstream code, but it is very close to the values
	 * generated by their lookup table, and it appears
	 * to sync at a variety of resolutions. If someone
	 * can get a more accurate mathematical equation needed
	 * for these registers, this should be updated.
	 */

	lpx = PS_TO_CYCLE(cfg.lpx, byte_clock);
	hs_exit = PS_TO_CYCLE(cfg.hs_exit, byte_clock);
	clk_prepare = PS_TO_CYCLE(cfg.clk_prepare, byte_clock);
	clk_zero = PS_TO_CYCLE(cfg.clk_zero, byte_clock);
	clk_post = PS_TO_CYCLE(cfg.clk_post, byte_clock);
	clk_trail = PS_TO_CYCLE(cfg.clk_trail, byte_clock);
	hs_prepare = PS_TO_CYCLE(cfg.hs_prepare, byte_clock);
	hs_zero = PS_TO_CYCLE(cfg.hs_zero, byte_clock);
	hs_trail = PS_TO_CYCLE(cfg.hs_trail, byte_clock);

	/* B D-PHY: D-PHY Master & Slave Analog Block control */
	reg = reg_values[PHYCTRL_ULPS_EXIT] | reg_values[PHYCTRL_VREG_LP] |
		reg_values[PHYCTRL_SLEW_UP];

	samsung_dsim_write(dsi, DSIM_PHYCTRL_REG, reg);

	/*
	 * T LPX: Transmitted length of any Low-Power state period
	 * T HS-EXIT: Time that the transmitter drives LP-11 following a HS
	 *	burst
	 */

	reg  = DSIM_PHYTIMING_LPX(lpx) | DSIM_PHYTIMING_HS_EXIT(hs_exit);

	samsung_dsim_write(dsi, DSIM_PHYTIMING_REG, reg);

	/*
	 * T CLK-PREPARE: Time that the transmitter drives the Clock Lane LP-00
	 *	Line state immediately before the HS-0 Line state starting the
	 *	HS transmission
	 * T CLK-ZERO: Time that the transmitter drives the HS-0 state prior to
	 *	transmitting the Clock.
	 * T CLK_POST: Time that the transmitter continues to send HS clock
	 *	after the last associated Data Lane has transitioned to LP Mode
	 *	Interval is defined as the period from the end of T HS-TRAIL to
	 *	the beginning of T CLK-TRAIL
	 * T CLK-TRAIL: Time that the transmitter drives the HS-0 state after
	 *	the last payload clock bit of a HS transmission burst
	 */

	reg = DSIM_PHYTIMING1_CLK_PREPARE(clk_prepare)	|
	      DSIM_PHYTIMING1_CLK_ZERO(clk_zero)	|
	      DSIM_PHYTIMING1_CLK_POST(clk_post)	|
	      DSIM_PHYTIMING1_CLK_TRAIL(clk_trail);

	samsung_dsim_write(dsi, DSIM_PHYTIMING1_REG, reg);

	/*
	 * T HS-PREPARE: Time that the transmitter drives the Data Lane LP-00
	 *	Line state immediately before the HS-0 Line state starting the
	 *	HS transmission
	 * T HS-ZERO: Time that the transmitter drives the HS-0 state prior to
	 *	transmitting the Sync sequence.
	 * T HS-TRAIL: Time that the transmitter drives the flipped differential
	 *	state after last payload data bit of a HS transmission burst
	 */

	reg = DSIM_PHYTIMING2_HS_PREPARE(hs_prepare) |
	      DSIM_PHYTIMING2_HS_ZERO(hs_zero) |
	      DSIM_PHYTIMING2_HS_TRAIL(hs_trail);

	samsung_dsim_write(dsi, DSIM_PHYTIMING2_REG, reg);
}

static void samsung_dsim_disable_clock(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	u32 reg;

	if (driver_data->has_zumapro_regs) {
		reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
		reg &= ~(DSIM_ZUMAPRO_CLKCTRL_LANE_ESCCLK_MASK |
			 BIT(driver_data->esc_clken_bit) |
			 BIT(driver_data->byte_clken_bit) |
			 BIT(driver_data->tx_req_hsclk_bit) |
			 DSIM_ZUMAPRO_CLKCTRL_NONCONT_CLOCK_LANE |
			 DSIM_ZUMAPRO_CLKCTRL_CLOCK_SEL);
		samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);
		return;
	}

	reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
	reg &= ~(BIT(driver_data->lane_esc_clk_bit)
		| DSIM_LANE_ESC_CLK_EN_DATA_MASK(driver_data->lane_esc_data_offset)
		| BIT(driver_data->esc_clken_bit)
		| BIT(driver_data->byte_clken_bit));
	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);

	if (!driver_data->uses_external_dphy_pll) {
		reg = samsung_dsim_read(dsi, DSIM_PLLCTRL_REG);
		reg &= ~DSIM_PLL_EN;
		samsung_dsim_write(dsi, DSIM_PLLCTRL_REG, reg);
	}
}

static void samsung_dsim_enable_lane(struct samsung_dsim *dsi, u32 lane)
{
	u32 reg = samsung_dsim_read(dsi, DSIM_CONFIG_REG);

	reg |= (DSIM_NUM_OF_DATA_LANE(dsi->lanes - 1) | DSIM_LANE_EN_CLK |
			DSIM_LANE_EN(lane));
	samsung_dsim_write(dsi, DSIM_CONFIG_REG, reg);
}

static u32 samsung_dsim_zumapro_dsc_width(const struct drm_dsc_config *dsc)
{
	return ALIGN(DIV_ROUND_UP(dsc->slice_width, 3), 4) * dsc->slice_count;
}

static u32 samsung_dsim_zumapro_display_width(struct samsung_dsim *dsi)
{
	if (dsi->dsc)
		return samsung_dsim_zumapro_dsc_width(dsi->dsc);

	return dsi->mode.hdisplay;
}

static int samsung_dsim_zumapro_refresh_rate(struct samsung_dsim *dsi)
{
	int refresh = drm_mode_vrefresh(&dsi->mode);

	return refresh > 0 ? refresh : 60;
}

static int samsung_dsim_zumapro_calc_underrun(struct samsung_dsim *dsi,
					      u32 *underrun)
{
	struct drm_display_mode *m = &dsi->mode;
	u32 width = samsung_dsim_zumapro_display_width(dsi);
	u32 refresh = samsung_dsim_zumapro_refresh_rate(dsi);
	u64 wclk = dsi->hs_clock / 16;
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	s64 max_frame_ns;
	s64 transfer_ns;
	u64 frame_data;
	u64 packet_header;
	s64 max_lp_ns;

	if (!dsi->lanes || !wclk || bpp <= 0)
		return -EINVAL;

	max_frame_ns = div_u64((u64)NSEC_PER_SEC * 100,
			       refresh * (100 + DSIM_ZUMAPRO_TE_VAR_PERCENT));
	max_frame_ns -= DSIM_ZUMAPRO_TE_IDLE_US * NSEC_PER_USEC;

	frame_data = (u64)m->vdisplay * width * bpp / 8;
	packet_header = (u64)m->vdisplay * 7;
	transfer_ns = div_u64((frame_data + packet_header) * NSEC_PER_SEC,
			      2 * dsi->lanes * wclk);
	max_lp_ns = max_frame_ns - transfer_ns;
	if (max_lp_ns <= 0) {
		dev_err(dsi->dev, "Zumapro command underrun budget is negative\n");
		return -EINVAL;
	}

	*underrun = DIV64_U64_ROUND_UP((u64)max_lp_ns * wclk,
				       (u64)NSEC_PER_SEC * 100);
	return 0;
}

static int samsung_dsim_zumapro_set_command_mode(struct samsung_dsim *dsi)
{
	struct drm_display_mode *m = &dsi->mode;
	u32 hs_clk_mhz = DIV_ROUND_CLOSEST_ULL(dsi->hs_clock, HZ_PER_MHZ);
	u32 refresh = samsung_dsim_zumapro_refresh_rate(dsi);
	u32 underrun;
	u32 stable_vfp;
	u32 te_protect;
	u32 te_timeout;
	u32 reg;
	int ret;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO)
		return 0;

	/*
	 * Downstream uses panel-provided TE idle/variance for the underrun
	 * budget. TG4C currently matches the default 1000us/1% values; trace
	 * this when panel timing data is plumbed into mainline.
	 */
	ret = samsung_dsim_zumapro_calc_underrun(dsi, &underrun);
	if (ret)
		return ret;

	reg = DSIM_ZUMAPRO_UNDERRUN_LP_REF(underrun);
	samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_UNDERRUN_CTRL, reg);

	if (dsi->dsc)
		stable_vfp = m->hdisplay * DSIM_ZUMAPRO_STABLE_VFP_DSC_PERCENT / 100;
	else
		stable_vfp = m->hdisplay * DSIM_ZUMAPRO_STABLE_VFP_DSC_PERCENT * 3 / 100;
	samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_CMD_TE_CTRL0, stable_vfp);

	te_protect = hs_clk_mhz * (100 - DSIM_ZUMAPRO_TE_MARGIN_PERCENT) *
		     100 / refresh / 16;
	te_timeout = hs_clk_mhz * (100 + DSIM_ZUMAPRO_TE_MARGIN_PERCENT * 2) *
		     100 / refresh / 16;
	reg = DSIM_ZUMAPRO_CMD_TE_PROTECT_ON(te_protect) |
	      DSIM_ZUMAPRO_CMD_TE_TIMEOUT(te_timeout);
	samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_CMD_TE_CTRL1, reg);

	reg = samsung_dsim_read_offset(dsi, DSIM_ZUMAPRO_OPTION_SUITE);
	reg |= DSIM_ZUMAPRO_OPT_TE_ON_CMD_ALLOW;
	samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_OPTION_SUITE, reg);

	return 0;
}

static int samsung_dsim_zumapro_init_link(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	u32 data_lanes_mask = BIT(dsi->lanes) - 1;
	u32 lanes_mask = BIT(dsi->lanes + 1) - 1;
	int timeout;
	u32 reg;

	if (dsi->format != MIPI_DSI_FMT_RGB888) {
		dev_err(dsi->dev, "unsupported Zumapro pixel format\n");
		return -EINVAL;
	}

	reg = samsung_dsim_read(dsi, DSIM_CONFIG_REG);
	reg &= ~DSIM_ZUMAPRO_CONFIG_OWNED_MASK;
	reg |= DSIM_ZUMAPRO_CONFIG_RGB24 |
	       DSIM_ZUMAPRO_CONFIG_DATA_LANE_NUM(dsi->lanes - 1) |
	       DSIM_ZUMAPRO_CONFIG_LANES_EN(lanes_mask);

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO)
		reg |= DSIM_ZUMAPRO_CONFIG_VIDEO_MODE;
	else
		reg |= DSIM_HSE_DISABLE_MODE;

	if (!(dsi->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET))
		reg |= DSIM_ZUMAPRO_CONFIG_EOTP_EN;

	if (dsi->dsc)
		reg |= DSIM_ZUMAPRO_CONFIG_CPRS_EN;

	samsung_dsim_write(dsi, DSIM_CONFIG_REG, reg);

	timeout = 100;
	do {
		if (timeout-- == 0) {
			dev_err(dsi->dev, "waiting for Zumapro bus lanes timed out\n");
			return -EFAULT;
		}

		reg = samsung_dsim_read(dsi, DSIM_DPHY_STATUS_REG);
		if ((reg & DSIM_STOP_STATE_DAT(data_lanes_mask)) !=
		    DSIM_STOP_STATE_DAT(data_lanes_mask))
			continue;
	} while (!(reg & (DSIM_STOP_STATE_CLK | DSIM_TX_READY_HS_CLK)));

	reg = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);
	reg &= ~DSIM_STOP_STATE_CNT_MASK;
	reg |= DSIM_STOP_STATE_CNT(driver_data->reg_values[STOP_STATE_CNT]);
	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, reg);

	reg = DSIM_BTA_TIMEOUT(0xff) | DSIM_LPDR_TIMEOUT(0xffff);
	samsung_dsim_write(dsi, DSIM_TIMEOUT_REG, reg);

	/*
	 * Shadowed writes stay off until the frame geometry has been
	 * programmed into the operating registers; the shadow bank is
	 * enabled at the end of samsung_dsim_zumapro_set_display_mode().
	 */
	reg = samsung_dsim_read(dsi, DSIM_SFRCTRL_REG);
	reg |= DSIM_ZUMAPRO_SHADOW_REG_READ_EN;
	samsung_dsim_write(dsi, DSIM_SFRCTRL_REG, reg);

	return samsung_dsim_zumapro_set_command_mode(dsi);
}

static int samsung_dsim_init_link(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	int timeout;
	u32 reg;
	u32 lanes_mask;

	if (driver_data->has_zumapro_regs)
		return samsung_dsim_zumapro_init_link(dsi);

	/* Initialize FIFO pointers */
	reg = samsung_dsim_read(dsi, DSIM_FIFOCTRL_REG);
	reg &= ~0x1f;
	samsung_dsim_write(dsi, DSIM_FIFOCTRL_REG, reg);

	usleep_range(9000, 11000);

	reg |= 0x1f;
	samsung_dsim_write(dsi, DSIM_FIFOCTRL_REG, reg);
	usleep_range(9000, 11000);

	/* DSI configuration */
	reg = 0;

	/*
	 * The first bit of mode_flags specifies display configuration.
	 * If this bit is set[= MIPI_DSI_MODE_VIDEO], dsi will support video
	 * mode, otherwise it will support command mode.
	 */
	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		reg |= BIT(driver_data->video_mode_bit);

		/*
		 * The user manual describes that following bits are ignored in
		 * command mode.
		 */
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			reg |= DSIM_SYNC_INFORM;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			reg |= DSIM_BURST_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_AUTO_VERT)
			reg |= DSIM_AUTO_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_HSE)
			reg |= DSIM_HSE_DISABLE_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_NO_HFP)
			reg |= DSIM_HFP_DISABLE_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_NO_HBP)
			reg |= DSIM_HBP_DISABLE_MODE;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_NO_HSA)
			reg |= DSIM_HSA_DISABLE_MODE;
	}

	if (dsi->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET)
		reg |= DSIM_EOT_DISABLE;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB888;
		break;
	case MIPI_DSI_FMT_RGB666:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB666;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB666_P;
		break;
	case MIPI_DSI_FMT_RGB565:
		reg |= DSIM_MAIN_PIX_FORMAT_RGB565;
		break;
	default:
		dev_err(dsi->dev, "invalid pixel format\n");
		return -EINVAL;
	}

	/*
	 * Use non-continuous clock mode if the periparal wants and
	 * host controller supports
	 *
	 * In non-continous clock mode, host controller will turn off
	 * the HS clock between high-speed transmissions to reduce
	 * power consumption.
	 */
	if (driver_data->has_clklane_stop &&
	    dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		if (!samsung_dsim_hw_is_exynos(dsi->plat_data->hw_type))
			reg |= DSIM_NON_CONTINUOUS_CLKLANE;

		reg |= DSIM_CLKLANE_STOP;
	}
	samsung_dsim_write(dsi, DSIM_CONFIG_REG, reg);

	lanes_mask = BIT(dsi->lanes) - 1;
	samsung_dsim_enable_lane(dsi, lanes_mask);

	/* Check clock and data lane state are stop state */
	timeout = 100;
	do {
		if (timeout-- == 0) {
			dev_err(dsi->dev, "waiting for bus lanes timed out\n");
			return -EFAULT;
		}

		if (driver_data->has_legacy_status_reg)
			reg = samsung_dsim_read(dsi, DSIM_STATUS_REG);
		else
			reg = samsung_dsim_read(dsi, DSIM_DPHY_STATUS_REG);
		if ((reg & DSIM_STOP_STATE_DAT(lanes_mask))
		    != DSIM_STOP_STATE_DAT(lanes_mask))
			continue;
	} while (!(reg & (DSIM_STOP_STATE_CLK | DSIM_TX_READY_HS_CLK)));

	reg = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);
	reg &= ~DSIM_STOP_STATE_CNT_MASK;
	reg |= DSIM_STOP_STATE_CNT(driver_data->reg_values[STOP_STATE_CNT]);
	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, reg);

	reg = DSIM_BTA_TIMEOUT(0xff) | DSIM_LPDR_TIMEOUT(0xffff);
	samsung_dsim_write(dsi, DSIM_TIMEOUT_REG, reg);

	return 0;
}

static void samsung_dsim_zumapro_config_dsc(struct samsung_dsim *dsi)
{
	const struct drm_dsc_config *dsc = dsi->dsc;
	u32 slice = dsc->slice_width;
	u32 val;

	val = DSIM_ZUMAPRO_CPRS_NUM_OF_SLICE(dsc->slice_count);
	if (dsc->slice_count > 1)
		val |= DSIM_ZUMAPRO_CPRS_MULTI_SLICE;
	samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_CPRS_CTRL, val);

	if (dsc->slice_count == 4) {
		val = DSIM_ZUMAPRO_SLICE_SIZE1(slice) |
		      DSIM_ZUMAPRO_SLICE_SIZE0(slice);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_SLICE01, val);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_SLICE23, val);
	} else if (dsc->slice_count == 2) {
		val = DSIM_ZUMAPRO_SLICE_SIZE1(slice) |
		      DSIM_ZUMAPRO_SLICE_SIZE0(slice);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_SLICE01, val);
	} else if (dsc->slice_count == 1) {
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_SLICE01,
					  DSIM_ZUMAPRO_SLICE_SIZE0(slice));
	} else {
		dev_warn(dsi->dev, "unsupported DSC slice count %u\n",
			 dsc->slice_count);
	}
}

static void samsung_dsim_zumapro_set_display_mode(struct samsung_dsim *dsi)
{
	struct drm_display_mode *m = &dsi->mode;
	unsigned int num_bits_resol = dsi->driver_data->num_bits_resol;
	unsigned int main_vsa_offset = dsi->driver_data->main_vsa_offset;
	u32 width = m->hdisplay;
	u32 threshold = width;
	u32 num_of_transfer;
	u32 reg;
	int ret;

	if (dsi->dsc) {
		width = samsung_dsim_zumapro_display_width(dsi);
		threshold = width;
	}

	reg = DSIM_MAIN_HRESOL(width, num_bits_resol) |
	      DSIM_MAIN_VRESOL(m->vdisplay, num_bits_resol);
	samsung_dsim_write(dsi, DSIM_MDRESOL_REG, reg);

	samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_THRESHOLD, threshold);

	if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		if (dsi->dsc)
			num_of_transfer = m->vdisplay;
		else
			num_of_transfer = DIV_ROUND_UP(m->hdisplay * m->vdisplay,
						       threshold);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_NUM_OF_TRANSFER,
					  num_of_transfer);
		ret = samsung_dsim_zumapro_set_command_mode(dsi);
		if (ret)
			dev_warn(dsi->dev,
				 "failed to configure Zumapro command mode: %d\n",
				 ret);
	} else {
		u64 byte_clk = dsi->hs_clock / 8;
		u64 pix_clk = m->clock * 1000;
		int hfp, hbp, hsa;

		num_of_transfer = DIV_ROUND_UP(m->hdisplay * m->vdisplay,
					       threshold);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_NUM_OF_TRANSFER,
					  num_of_transfer);

		reg = DSIM_ZUMAPRO_VFP_CMD_ALLOW(4) |
		      DSIM_ZUMAPRO_STABLE_VFP(2);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_VFP_DETAIL, reg);

		reg = DSIM_STABLE_VFP(m->vsync_start - m->vdisplay) |
		      DSIM_MAIN_VBP(m->vtotal - m->vsync_end);
		samsung_dsim_write_offset(dsi, DSIM_ZUMAPRO_VPORCH, reg);

		hfp = DIV64_U64_ROUND_UP((m->hsync_start - m->hdisplay) * byte_clk,
					 pix_clk);
		hbp = DIV64_U64_ROUND_UP((m->htotal - m->hsync_end) * byte_clk,
					 pix_clk);
		hsa = DIV64_U64_ROUND_UP((m->hsync_end - m->hsync_start) * byte_clk,
					 pix_clk);

		hfp = max(hfp - 6, 0);
		hbp = max(hbp - 6, 0);
		hsa = max(hsa - 6, 0);

		reg = DSIM_MAIN_HFP(hfp) | DSIM_MAIN_HBP(hbp);
		samsung_dsim_write(dsi, DSIM_MHPORCH_REG, reg);

		reg = DSIM_MAIN_VSA(m->vsync_end - m->vsync_start, main_vsa_offset) |
		      DSIM_MAIN_HSA(hsa);
		samsung_dsim_write(dsi, DSIM_MSYNC_REG, reg);
	}

	/*
	 * Downstream programs resolution/threshold/transfer count with the
	 * shadow bank disabled so they land in the operating registers, and
	 * only shadows the compression set below.
	 */
	reg = samsung_dsim_read(dsi, DSIM_SFRCTRL_REG);
	reg |= DSIM_SFR_CTRL_SHADOW_EN;
	samsung_dsim_write(dsi, DSIM_SFRCTRL_REG, reg);

	if (dsi->dsc)
		samsung_dsim_zumapro_config_dsc(dsi);

	dev_dbg(dsi->dev, "Zumapro LCD size = %ux%u, DSIM width = %u\n",
		m->hdisplay, m->vdisplay, width);
}

static void samsung_dsim_set_display_mode(struct samsung_dsim *dsi)
{
	struct drm_display_mode *m = &dsi->mode;
	unsigned int num_bits_resol = dsi->driver_data->num_bits_resol;
	unsigned int main_vsa_offset = dsi->driver_data->main_vsa_offset;
	u32 reg;

	if (dsi->driver_data->has_zumapro_regs) {
		samsung_dsim_zumapro_set_display_mode(dsi);
		return;
	}

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		u64 byte_clk = dsi->hs_clock / 8;
		u64 pix_clk = m->clock * 1000;

		int hfp = DIV64_U64_ROUND_UP((m->hsync_start - m->hdisplay) * byte_clk, pix_clk);
		int hbp = DIV64_U64_ROUND_UP((m->htotal - m->hsync_end) * byte_clk, pix_clk);
		int hsa = DIV64_U64_ROUND_UP((m->hsync_end - m->hsync_start) * byte_clk, pix_clk);

		/* remove packet overhead when possible */
		hfp = max(hfp - 6, 0);
		hbp = max(hbp - 6, 0);
		hsa = max(hsa - 6, 0);

		dev_dbg(dsi->dev, "calculated hfp: %u, hbp: %u, hsa: %u",
			hfp, hbp, hsa);

		reg = DSIM_CMD_ALLOW(0xf)
			| DSIM_STABLE_VFP(m->vsync_start - m->vdisplay)
			| DSIM_MAIN_VBP(m->vtotal - m->vsync_end);
		samsung_dsim_write(dsi, DSIM_MVPORCH_REG, reg);

		reg = DSIM_MAIN_HFP(hfp) | DSIM_MAIN_HBP(hbp);
		samsung_dsim_write(dsi, DSIM_MHPORCH_REG, reg);

		reg = DSIM_MAIN_VSA(m->vsync_end - m->vsync_start, main_vsa_offset)
			| DSIM_MAIN_HSA(hsa);
		samsung_dsim_write(dsi, DSIM_MSYNC_REG, reg);
	}
	reg =  DSIM_MAIN_HRESOL(m->hdisplay, num_bits_resol) |
		DSIM_MAIN_VRESOL(m->vdisplay, num_bits_resol);

	samsung_dsim_write(dsi, DSIM_MDRESOL_REG, reg);

	dev_dbg(dsi->dev, "LCD size = %dx%d\n", m->hdisplay, m->vdisplay);
}

static void samsung_dsim_set_display_enable(struct samsung_dsim *dsi, bool enable)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	u32 reg;

	if (driver_data->has_zumapro_regs)
		return;

	reg = samsung_dsim_read(dsi, DSIM_MDRESOL_REG);
	if (enable)
		reg |= DSIM_MAIN_STAND_BY;
	else
		reg &= ~DSIM_MAIN_STAND_BY;
	samsung_dsim_write(dsi, DSIM_MDRESOL_REG, reg);

	if (driver_data->has_sfrctrl) {
		reg = samsung_dsim_read(dsi, DSIM_SFRCTRL_REG);
		if (enable)
			reg |= DSIM_SFR_CTRL_STAND_BY;
		else
			reg &= ~DSIM_SFR_CTRL_STAND_BY;
		samsung_dsim_write(dsi, DSIM_SFRCTRL_REG, reg);
	}
}

static int samsung_dsim_wait_for_hdr_fifo(struct samsung_dsim *dsi)
{
	int timeout = 2000;

	do {
		u32 reg = samsung_dsim_read(dsi, DSIM_FIFOCTRL_REG);

		if (!dsi->driver_data->has_broken_fifoctrl_emptyhdr) {
			if (reg & DSIM_SFR_HEADER_EMPTY)
				return 0;
		} else {
			if (!(reg & DSIM_SFR_HEADER_FULL)) {
				/*
				 * Wait a little bit, so the pending data can
				 * actually leave the FIFO to avoid overflow.
				 */
				if (!cond_resched())
					usleep_range(950, 1050);
				return 0;
			}
		}

		if (!cond_resched())
			usleep_range(950, 1050);
	} while (--timeout);

	return -ETIMEDOUT;
}

static void samsung_dsim_set_cmd_lpm(struct samsung_dsim *dsi, bool lpm)
{
	u32 v = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);

	if (lpm)
		v |= DSIM_CMD_LPDT_LP;
	else
		v &= ~DSIM_CMD_LPDT_LP;

	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, v);
}

static void samsung_dsim_force_bta(struct samsung_dsim *dsi)
{
	u32 v = samsung_dsim_read(dsi, DSIM_ESCMODE_REG);

	v |= DSIM_FORCE_BTA;
	samsung_dsim_write(dsi, DSIM_ESCMODE_REG, v);
}

static void samsung_dsim_send_to_fifo(struct samsung_dsim *dsi,
				      struct samsung_dsim_transfer *xfer)
{
	struct device *dev = dsi->dev;
	struct mipi_dsi_packet *pkt = &xfer->packet;
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	const u8 *payload = pkt->payload + xfer->tx_done;
	u16 length = pkt->payload_length - xfer->tx_done;
	bool first = !xfer->tx_done;
	u32 reg;

	dev_dbg(dev, "< xfer %p: tx len %u, done %u, rx len %u, done %u\n",
		xfer, length, xfer->tx_done, xfer->rx_len, xfer->rx_done);

	if (length > DSI_TX_FIFO_SIZE)
		length = DSI_TX_FIFO_SIZE;

	xfer->tx_done += length;

	/* Send payload */
	while (length >= 4) {
		reg = get_unaligned_le32(payload);
		samsung_dsim_write(dsi, DSIM_PAYLOAD_REG, reg);
		payload += 4;
		length -= 4;
	}

	reg = 0;
	switch (length) {
	case 3:
		reg |= payload[2] << 16;
		fallthrough;
	case 2:
		reg |= payload[1] << 8;
		fallthrough;
	case 1:
		reg |= payload[0];
		samsung_dsim_write(dsi, DSIM_PAYLOAD_REG, reg);
		break;
	}

	/* Send packet header */
	if (!first)
		return;

	reg = get_unaligned_le32(pkt->header);
	if (driver_data->wait_for_hdr_fifo) {
		if (samsung_dsim_wait_for_hdr_fifo(dsi)) {
			dev_err(dev, "waiting for header FIFO timed out\n");
			return;
		}
	}

	if (NEQV(xfer->flags & MIPI_DSI_MSG_USE_LPM,
		 dsi->state & DSIM_STATE_CMD_LPM)) {
		samsung_dsim_set_cmd_lpm(dsi, xfer->flags & MIPI_DSI_MSG_USE_LPM);
		dsi->state ^= DSIM_STATE_CMD_LPM;
	}

	samsung_dsim_write(dsi, DSIM_PKTHDR_REG, reg);

	if (xfer->flags & MIPI_DSI_MSG_REQ_ACK)
		samsung_dsim_force_bta(dsi);
}

static void samsung_dsim_read_from_fifo(struct samsung_dsim *dsi,
					struct samsung_dsim_transfer *xfer)
{
	u8 *payload = xfer->rx_payload + xfer->rx_done;
	bool first = !xfer->rx_done;
	struct device *dev = dsi->dev;
	u16 length;
	u32 reg;

	if (first) {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);

		switch (reg & 0x3f) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (xfer->rx_len >= 2) {
				payload[1] = reg >> 16;
				++xfer->rx_done;
			}
			fallthrough;
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			payload[0] = reg >> 8;
			++xfer->rx_done;
			xfer->rx_len = xfer->rx_done;
			xfer->result = 0;
			goto clear_fifo;
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			dev_err(dev, "DSI Error Report: 0x%04x\n", (reg >> 8) & 0xffff);
			xfer->result = 0;
			goto clear_fifo;
		}

		length = (reg >> 8) & 0xffff;
		if (length > xfer->rx_len) {
			dev_err(dev,
				"response too long (%u > %u bytes), stripping\n",
				xfer->rx_len, length);
			length = xfer->rx_len;
		} else if (length < xfer->rx_len) {
			xfer->rx_len = length;
		}
	}

	length = xfer->rx_len - xfer->rx_done;
	xfer->rx_done += length;

	/* Receive payload */
	while (length >= 4) {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);
		payload[0] = (reg >>  0) & 0xff;
		payload[1] = (reg >>  8) & 0xff;
		payload[2] = (reg >> 16) & 0xff;
		payload[3] = (reg >> 24) & 0xff;
		payload += 4;
		length -= 4;
	}

	if (length) {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);
		switch (length) {
		case 3:
			payload[2] = (reg >> 16) & 0xff;
			fallthrough;
		case 2:
			payload[1] = (reg >> 8) & 0xff;
			fallthrough;
		case 1:
			payload[0] = reg & 0xff;
		}
	}

	if (xfer->rx_done == xfer->rx_len)
		xfer->result = 0;

clear_fifo:
	length = DSI_RX_FIFO_SIZE / 4;
	do {
		reg = samsung_dsim_read(dsi, DSIM_RXFIFO_REG);
		if (reg == DSI_RX_FIFO_EMPTY)
			break;
	} while (--length);
}

static void __samsung_dsim_transfer_start(struct samsung_dsim *dsi)
{
	unsigned long flags;
	struct samsung_dsim_transfer *xfer;

	lockdep_assert_held(&dsi->cmd_lock);

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	while (!list_empty(&dsi->transfer_list)) {
		xfer = list_first_entry(&dsi->transfer_list,
					struct samsung_dsim_transfer, list);

		spin_unlock_irqrestore(&dsi->transfer_lock, flags);

		if (xfer->packet.payload_length &&
		    xfer->tx_done == xfer->packet.payload_length)
			/* waiting for RX */
			return;

		samsung_dsim_send_to_fifo(dsi, xfer);

		if (xfer->packet.payload_length || xfer->rx_len)
			return;

		/*
		 * Short packets complete here.  Unlink the xfer from the list
		 * *before* signalling completion: once complete() wakes the
		 * waiter (samsung_dsim_host_transfer) it takes the success path
		 * and frees this on-stack xfer immediately, so it must already
		 * be off the list by then -- cmd_lock keeps concurrent owners
		 * out, but a later transfer_finish() would still find a freed
		 * xfer via list_first_entry() if it were left linked.  This
		 * mirrors the ordering used by samsung_dsim_transfer_finish()
		 * (list_del_init then complete); keep complete() outside
		 * transfer_lock to avoid a transfer_lock -> wait.lock nesting.
		 */
		spin_lock_irqsave(&dsi->transfer_lock, flags);
		list_del_init(&xfer->list);
		spin_unlock_irqrestore(&dsi->transfer_lock, flags);

		xfer->result = 0;
		complete(&xfer->completed);

		spin_lock_irqsave(&dsi->transfer_lock, flags);
	}

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);
}

static void samsung_dsim_transfer_start(struct samsung_dsim *dsi)
{
	mutex_lock(&dsi->cmd_lock);
	__samsung_dsim_transfer_start(dsi);
	mutex_unlock(&dsi->cmd_lock);
}

static bool __samsung_dsim_transfer_finish(struct samsung_dsim *dsi)
{
	struct samsung_dsim_transfer *xfer;
	unsigned long flags;
	bool start = true;

	lockdep_assert_held(&dsi->cmd_lock);

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	if (list_empty(&dsi->transfer_list)) {
		spin_unlock_irqrestore(&dsi->transfer_lock, flags);
		return false;
	}

	xfer = list_first_entry(&dsi->transfer_list,
				struct samsung_dsim_transfer, list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);

	dev_dbg(dsi->dev,
		"> xfer %p, tx_len %zu, tx_done %u, rx_len %u, rx_done %u\n",
		xfer, xfer->packet.payload_length, xfer->tx_done, xfer->rx_len,
		xfer->rx_done);

	if (xfer->tx_done != xfer->packet.payload_length)
		return true;

	if (xfer->rx_done != xfer->rx_len)
		samsung_dsim_read_from_fifo(dsi, xfer);

	if (xfer->rx_done != xfer->rx_len)
		return true;

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	list_del_init(&xfer->list);
	start = !list_empty(&dsi->transfer_list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);

	if (!xfer->rx_len)
		xfer->result = 0;
	complete(&xfer->completed);

	return start;
}

static void samsung_dsim_remove_transfer(struct samsung_dsim *dsi,
					 struct samsung_dsim_transfer *xfer)
{
	unsigned long flags;
	bool start;

	/*
	 * Taking cmd_lock here is what makes the caller's on-stack xfer safe to
	 * free once this returns: any other context that might still hold a
	 * pointer to it -- the threaded IRQ in samsung_dsim_transfer_finish(),
	 * or another waiter's samsung_dsim_transfer_start() -- can only be
	 * there while holding cmd_lock, so it has drained by the time we get
	 * it.  Once we drop the lock the xfer is off the list and unreachable.
	 */
	mutex_lock(&dsi->cmd_lock);
	spin_lock_irqsave(&dsi->transfer_lock, flags);

	if (!list_empty(&dsi->transfer_list) &&
	    xfer == list_first_entry(&dsi->transfer_list,
				     struct samsung_dsim_transfer, list)) {
		list_del_init(&xfer->list);
		start = !list_empty(&dsi->transfer_list);
		spin_unlock_irqrestore(&dsi->transfer_lock, flags);
		if (start)
			__samsung_dsim_transfer_start(dsi);
		mutex_unlock(&dsi->cmd_lock);
		return;
	}

	list_del_init(&xfer->list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);
	mutex_unlock(&dsi->cmd_lock);
}

static int samsung_dsim_transfer(struct samsung_dsim *dsi,
				 struct samsung_dsim_transfer *xfer)
{
	unsigned long flags;
	bool stopped;

	xfer->tx_done = 0;
	xfer->rx_done = 0;
	xfer->result = -ETIMEDOUT;
	init_completion(&xfer->completed);

	spin_lock_irqsave(&dsi->transfer_lock, flags);

	stopped = list_empty(&dsi->transfer_list);
	list_add_tail(&xfer->list, &dsi->transfer_list);

	spin_unlock_irqrestore(&dsi->transfer_lock, flags);

	if (stopped)
		samsung_dsim_transfer_start(dsi);

	/*
	 * Decide timeout from the completion itself, not from xfer->result.
	 * The threaded IRQ samsung_dsim_transfer_finish() sets xfer->result = 0
	 * immediately *before* it calls complete(), so a transfer that finishes
	 * right at the timeout boundary can leave result == 0 while complete()
	 * has not run yet. Keying the timeout off result would then take the
	 * success path and free this on-stack xfer before the handler's
	 * complete() touches it -- a use-after-free whose stray try_to_wake_up
	 * write smashes whatever now owns the reused stack (seen as a NULL-ish
	 * spinlock deref in complete(), or a pointer-auth/FPAC fault on a
	 * corrupted return address). wait_for_completion_timeout() returning 0
	 * means the completion was never signalled.
	 */
	if (!wait_for_completion_timeout(&xfer->completed,
					 msecs_to_jiffies(DSI_XFER_TIMEOUT_MS))) {
		struct mipi_dsi_packet *pkt = &xfer->packet;

		samsung_dsim_remove_transfer(dsi, xfer);
		/*
		 * samsung_dsim_remove_transfer() already drained every context
		 * that could still hold this xfer, via cmd_lock. Keep
		 * synchronize_irq() as a cheap backstop on a path that has
		 * already waited DSI_XFER_TIMEOUT_MS: it costs nothing here and
		 * still catches a handler that touches an xfer outside
		 * cmd_lock, which is exactly the mistake this used to be.
		 */
		synchronize_irq(dsi->irq);
		dev_err(dsi->dev, "xfer timed out: %*ph %*ph\n", 4, pkt->header,
			(int)pkt->payload_length, pkt->payload);
		return -ETIMEDOUT;
	}

	/* Also covers hardware timeout condition */
	return xfer->result;
}

static irqreturn_t samsung_dsim_irq(int irq, void *dev_id)
{
	struct samsung_dsim *dsi = dev_id;
	u32 status;

	status = samsung_dsim_read(dsi, DSIM_INTSRC_REG);
	if (!status) {
		static unsigned long j;

		if (printk_timed_ratelimit(&j, 500))
			dev_warn(dsi->dev, "spurious interrupt\n");
		return IRQ_HANDLED;
	}
	samsung_dsim_write(dsi, DSIM_INTSRC_REG, status);

	if (status & DSIM_INT_SW_RST_RELEASE) {
		unsigned long mask = ~(DSIM_INT_RX_DONE |
				       DSIM_INT_SFR_FIFO_EMPTY |
				       DSIM_INT_SFR_HDR_FIFO_EMPTY |
				       DSIM_INT_RX_ECC_ERR |
				       DSIM_INT_SW_RST_RELEASE);
		samsung_dsim_write(dsi, DSIM_INTMSK_REG, mask);
		complete(&dsi->completed);
		return IRQ_HANDLED;
	}

	/*
	 * The remaining interrupts drive the command-transfer state machine,
	 * which is only meaningful while the link is initialized (HS clock and
	 * D-PHY up).  System suspend tears the link down by clearing
	 * DSIM_STATE_INITIALIZED before gating the clocks/PHY; a FIFO or PLL
	 * interrupt latched on the dying link must be dropped here, since the
	 * in-flight transfer has already gone away and completing it would
	 * touch freed (on-stack) memory.  Downstream gates its whole handler on
	 * DSIM_STATE_HSCLKEN for the same reason.  The reset handshake above
	 * runs before INITIALIZED is set and is intentionally left ungated.
	 */
	if (!(dsi->state & DSIM_STATE_INITIALIZED))
		return IRQ_HANDLED;

	if (!(status & (DSIM_INT_RX_DONE | DSIM_INT_SFR_FIFO_EMPTY |
			DSIM_INT_PLL_STABLE)))
		return IRQ_HANDLED;

	/*
	 * One cmd_lock section for both halves: transfer_finish() hands the
	 * list straight over to transfer_start(), and dropping the lock in
	 * between would reopen the window this lock exists to close.
	 */
	mutex_lock(&dsi->cmd_lock);
	if (__samsung_dsim_transfer_finish(dsi))
		__samsung_dsim_transfer_start(dsi);
	mutex_unlock(&dsi->cmd_lock);

	return IRQ_HANDLED;
}

static void samsung_dsim_enable_irq(struct samsung_dsim *dsi)
{
	enable_irq(dsi->irq);

	if (dsi->te_gpio)
		enable_irq(gpiod_to_irq(dsi->te_gpio));
}

static void samsung_dsim_zumapro_start_handoff_link(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	u32 mask;
	u32 reg;

	reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);
	reg |= BIT(driver_data->tx_req_hsclk_bit);
	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);

	/*
	 * The DSIM_INT_* bits are BIT() values, so the complement is computed
	 * as unsigned long; narrow it explicitly for the 32-bit register.
	 */
	mask = (u32)~(DSIM_INT_RX_DONE |
		      DSIM_INT_SFR_FIFO_EMPTY |
		      DSIM_INT_SFR_HDR_FIFO_EMPTY |
		      DSIM_INT_RX_ECC_ERR |
		      DSIM_INT_SW_RST_RELEASE);
	samsung_dsim_write(dsi, DSIM_INTMSK_REG, mask);
	samsung_dsim_write(dsi, DSIM_INTSRC_REG, 0xffffffff);
}

/*
 * Request the HS byte clock once the external D-PHY PLL is locked.  Deferred
 * out of samsung_dsim_enable_clock() so the cold bring-up locks the PLL with
 * TX_REQUEST_HSCLK clear, matching the bootloader/downstream order.
 */
static void samsung_dsim_zumapro_request_hs_clock(struct samsung_dsim *dsi)
{
	u32 reg = samsung_dsim_read(dsi, DSIM_CLKCTRL_REG);

	reg |= BIT(dsi->driver_data->tx_req_hsclk_bit);
	samsung_dsim_write(dsi, DSIM_CLKCTRL_REG, reg);
}

static void samsung_dsim_disable_irq(struct samsung_dsim *dsi)
{
	if (dsi->te_gpio)
		disable_irq(gpiod_to_irq(dsi->te_gpio));

	disable_irq(dsi->irq);
}

static int samsung_dsim_init(struct samsung_dsim *dsi)
{
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	int ret;

	if (dsi->state & DSIM_STATE_INITIALIZED)
		return 0;

	/*
	 * A link running on the word clock was handed over live by the
	 * bootloader, scanning out the splash frame.  Downstream never
	 * resets it (dsim handover, skip_init): resetting the DSIM or
	 * reprogramming the D-PHY kills the clock the scanout runs on and
	 * risks stalling the interconnect.  Adopt the running link instead.
	 * The PHY calls keep the phy core refcounts balanced; the PHY
	 * driver itself skips reprogramming a locked PLL.  Our own
	 * teardown selects the OSC clock back, so this path only triggers
	 * on a bootloader handover.
	 */
	if (driver_data->uses_external_dphy_pll &&
	    samsung_dsim_read(dsi, DSIM_CLKCTRL_REG) &
	    DSIM_ZUMAPRO_CLKCTRL_CLOCK_SEL) {
		unsigned long esc_div;

		/*
		 * Compute hs/esc clocks for the PHY configuration and the
		 * command-mode timing, but leave DSIM_CLKCTRL alone: the
		 * adopted link runs on the bootloader's prescaler and clock
		 * enables, and rewriting them would glitch the live link.
		 */
		ret = samsung_dsim_calc_clocks(dsi, &esc_div);
		if (ret)
			return ret;

		ret = samsung_dsim_configure_external_phy(dsi);
		if (ret)
			return ret;

		ret = samsung_dsim_init_link(dsi);
		if (ret)
			return ret;

		samsung_dsim_zumapro_start_handoff_link(dsi);
		samsung_dsim_enable_irq(dsi);
		dsi->state |= DSIM_STATE_INITIALIZED;

		return 0;
	}

	/*
	 * Run the link on the OSC clock while the D-PHY is reset and
	 * reprogrammed below.  A cold-booting bootloader hands the DSIM off
	 * with the word clock selected; resetting the PHY then kills the
	 * selected link clock mid-sequence and the next access to a register
	 * in that clock domain stalls the interconnect.  Downstream's
	 * dsim_reg_init() selects the OSC clock first for the same reason;
	 * the word clock is selected again once the PHY PLL is running.
	 */
	if (driver_data->uses_external_dphy_pll)
		samsung_dsim_zumapro_select_word_clock(dsi, false);

	samsung_dsim_reset(dsi);
	samsung_dsim_enable_irq(dsi);

	if (driver_data->reg_values[RESET_TYPE] == DSIM_FUNCRST)
		samsung_dsim_enable_lane(dsi, BIT(dsi->lanes) - 1);

	ret = samsung_dsim_enable_clock(dsi);
	if (ret)
		goto err_disable_irq;

	ret = samsung_dsim_configure_external_phy(dsi);
	if (ret)
		goto err_disable_clock;

	if (driver_data->wait_for_reset)
		samsung_dsim_wait_for_reset(dsi);
	samsung_dsim_set_phy_ctrl(dsi);

	ret = samsung_dsim_init_link(dsi);
	if (ret)
		goto err_disable_phy;

	if (driver_data->uses_external_dphy_pll)
		samsung_dsim_zumapro_request_hs_clock(dsi);

	dsi->state |= DSIM_STATE_INITIALIZED;

	return 0;

err_disable_phy:
	if (driver_data->uses_external_dphy_pll) {
		samsung_dsim_zumapro_select_word_clock(dsi, false);
		phy_power_off(dsi->phy);
		phy_exit(dsi->phy);
	}
err_disable_clock:
	samsung_dsim_disable_clock(dsi);
err_disable_irq:
	samsung_dsim_disable_irq(dsi);

	return ret;
}

static void samsung_dsim_atomic_pre_enable(struct drm_bridge *bridge,
					   struct drm_atomic_commit *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);
	int ret;

	if (dsi->state & DSIM_STATE_ENABLED)
		return;

	ret = pm_runtime_resume_and_get(dsi->dev);
	if (ret < 0) {
		dev_err(dsi->dev, "failed to enable DSI device.\n");
		dsi->state |= DSIM_STATE_PRE_ENABLE_FAILED;
		return;
	}

	dsi->state &= ~DSIM_STATE_PRE_ENABLE_FAILED;
	dsi->state |= DSIM_STATE_ENABLED;

	/*
	 * For Exynos-DSIM the downstream bridge, or panel are expecting
	 * the host initialization during DSI transfer.
	 */
	if (!samsung_dsim_init_on_transfer(dsi->plat_data->hw_type)) {
		ret = samsung_dsim_init(dsi);
		if (ret) {
			dsi->state |= DSIM_STATE_PRE_ENABLE_FAILED;
			return;
		}
	}
}

static void samsung_dsim_atomic_enable(struct drm_bridge *bridge,
				       struct drm_atomic_commit *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	if (dsi->state & DSIM_STATE_PRE_ENABLE_FAILED)
		return;

	samsung_dsim_set_display_mode(dsi);
	samsung_dsim_set_display_enable(dsi, true);

	dsi->state |= DSIM_STATE_VIDOUT_AVAILABLE;
}

static void samsung_dsim_atomic_disable(struct drm_bridge *bridge,
					struct drm_atomic_commit *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	if ((dsi->state & DSIM_STATE_PRE_ENABLE_FAILED) ||
	    !(dsi->state & DSIM_STATE_ENABLED))
		return;

	samsung_dsim_set_display_enable(dsi, false);
	dsi->state &= ~DSIM_STATE_VIDOUT_AVAILABLE;
}

static void samsung_dsim_atomic_post_disable(struct drm_bridge *bridge,
					     struct drm_atomic_commit *state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	if (!(dsi->state & DSIM_STATE_ENABLED)) {
		dsi->state &= ~DSIM_STATE_PRE_ENABLE_FAILED;
		return;
	}

	dsi->state &= ~DSIM_STATE_PRE_ENABLE_FAILED;
	dsi->state &= ~DSIM_STATE_ENABLED;
	pm_runtime_put_sync(dsi->dev);
}

/*
 * This pixel output formats list referenced from,
 * AN13573 i.MX 8/RT MIPI DSI/CSI-2, Rev. 0, 21 March 2022
 * 3.7.4 Pixel formats
 * Table 14. DSI pixel packing formats
 */
static const u32 samsung_dsim_pixel_output_fmts[] = {
	MEDIA_BUS_FMT_YUYV10_1X20,
	MEDIA_BUS_FMT_YUYV12_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_RGB101010_1X30,
	MEDIA_BUS_FMT_RGB121212_1X36,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB888_1X24,
};

static bool samsung_dsim_pixel_output_fmt_supported(u32 fmt)
{
	int i;

	if (fmt == MEDIA_BUS_FMT_FIXED)
		return false;

	for (i = 0; i < ARRAY_SIZE(samsung_dsim_pixel_output_fmts); i++) {
		if (samsung_dsim_pixel_output_fmts[i] == fmt)
			return true;
	}

	return false;
}

static u32 *
samsung_dsim_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
				       struct drm_bridge_state *bridge_state,
				       struct drm_crtc_state *crtc_state,
				       struct drm_connector_state *conn_state,
				       u32 output_fmt,
				       unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	input_fmts = kmalloc_obj(*input_fmts);
	if (!input_fmts)
		return NULL;

	if (!samsung_dsim_pixel_output_fmt_supported(output_fmt))
		/*
		 * Some bridge/display drivers are still not able to pass the
		 * correct format, so handle those pipelines by falling back
		 * to the default format till the supported formats finalized.
		 */
		output_fmt = MEDIA_BUS_FMT_RGB888_1X24;

	input_fmts[0] = output_fmt;
	*num_input_fmts = 1;

	return input_fmts;
}

static int samsung_dsim_atomic_check(struct drm_bridge *bridge,
				     struct drm_bridge_state *bridge_state,
				     struct drm_crtc_state *crtc_state,
				     struct drm_connector_state *conn_state)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;

	/*
	 * The i.MX8M Mini/Nano glue logic between LCDIF and DSIM
	 * inverts HS/VS/DE sync signals polarity, therefore, while
	 * i.MX 8M Mini Applications Processor Reference Manual Rev. 3, 11/2020
	 * 13.6.3.5.2 RGB interface
	 * i.MX 8M Nano Applications Processor Reference Manual Rev. 2, 07/2022
	 * 13.6.2.7.2 RGB interface
	 * both claim "Vsync, Hsync, and VDEN are active high signals.", the
	 * LCDIF must generate inverted HS/VS/DE signals, i.e. active LOW.
	 *
	 * The i.MX8M Plus glue logic between LCDIFv3 and DSIM does not
	 * implement the same behavior, therefore LCDIFv3 must generate
	 * HS/VS/DE signals active HIGH.
	 */
	if (dsi->plat_data->hw_type == DSIM_TYPE_IMX8MM) {
		adjusted_mode->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
		adjusted_mode->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	} else if (dsi->plat_data->hw_type == DSIM_TYPE_IMX8MP) {
		adjusted_mode->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
		adjusted_mode->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	}

	/*
	 * When using video sync pulses, the HFP, HBP, and HSA are divided between
	 * the available lanes if there is more than one lane.  For certain
	 * timings and lane configurations, the HFP may not be evenly divisible.
	 * If the HFP is rounded down, it ends up being too small which can cause
	 * some monitors to not sync properly. In these instances, adjust htotal
	 * and hsync to round the HFP up, and recalculate the htotal. Through trial
	 * and error, it appears that the HBP and HSA do not appearto need the same
	 * correction that HFP does.
	 */
	if (dsi->lanes > 1) {
		int hfp = adjusted_mode->hsync_start - adjusted_mode->hdisplay;
		int remainder = hfp % dsi->lanes;

		if (remainder) {
			adjusted_mode->hsync_start += remainder;
			adjusted_mode->hsync_end   += remainder;
			adjusted_mode->htotal      += remainder;
		}
	}

	return 0;
}

static void samsung_dsim_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	drm_mode_copy(&dsi->mode, adjusted_mode);
}

static int samsung_dsim_attach(struct drm_bridge *bridge,
			       struct drm_encoder *encoder,
			       enum drm_bridge_attach_flags flags)
{
	struct samsung_dsim *dsi = bridge_to_dsi(bridge);

	return drm_bridge_attach(encoder, dsi->bridge.next_bridge, bridge,
				 flags);
}

static const struct drm_bridge_funcs samsung_dsim_bridge_funcs = {
	.atomic_duplicate_state		= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state		= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset			= drm_atomic_helper_bridge_reset,
	.atomic_get_input_bus_fmts	= samsung_dsim_atomic_get_input_bus_fmts,
	.atomic_check			= samsung_dsim_atomic_check,
	.atomic_pre_enable		= samsung_dsim_atomic_pre_enable,
	.atomic_enable			= samsung_dsim_atomic_enable,
	.atomic_disable			= samsung_dsim_atomic_disable,
	.atomic_post_disable		= samsung_dsim_atomic_post_disable,
	.mode_set			= samsung_dsim_mode_set,
	.attach				= samsung_dsim_attach,
};

static irqreturn_t samsung_dsim_te_irq_handler(int irq, void *dev_id)
{
	struct samsung_dsim *dsi = (struct samsung_dsim *)dev_id;
	const struct samsung_dsim_plat_data *pdata = dsi->plat_data;

	if (pdata->host_ops && pdata->host_ops->te_irq_handler)
		return pdata->host_ops->te_irq_handler(dsi);

	return IRQ_HANDLED;
}

static int samsung_dsim_register_te_irq(struct samsung_dsim *dsi, struct device *dev)
{
	int te_gpio_irq;
	int ret;

	dsi->te_gpio = devm_gpiod_get_optional(dev, "te", GPIOD_IN);
	if (!dsi->te_gpio)
		return 0;
	else if (IS_ERR(dsi->te_gpio))
		return dev_err_probe(dev, PTR_ERR(dsi->te_gpio), "failed to get te GPIO\n");

	te_gpio_irq = gpiod_to_irq(dsi->te_gpio);

	ret = request_threaded_irq(te_gpio_irq, samsung_dsim_te_irq_handler, NULL,
				   IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN, "TE", dsi);
	if (ret) {
		dev_err(dsi->dev, "request interrupt failed with %d\n", ret);
		gpiod_put(dsi->te_gpio);
		return ret;
	}

	return 0;
}

static void samsung_dsim_unregister_te_irq(struct samsung_dsim *dsi)
{
	if (dsi->te_gpio) {
		free_irq(gpiod_to_irq(dsi->te_gpio), dsi);
		gpiod_put(dsi->te_gpio);
	}
}

static int samsung_dsim_host_attach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *device)
{
	struct samsung_dsim *dsi = host_to_dsi(host);
	const struct samsung_dsim_plat_data *pdata = dsi->plat_data;
	struct drm_bridge *next_bridge __free(drm_bridge_put) = NULL;
	struct device *dev = dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *remote;
	struct drm_panel *panel;
	int ret = 0;

	/*
	 * Devices can also be child nodes when we also control that device
	 * through the upstream device (ie, MIPI-DCS for a MIPI-DSI device).
	 *
	 * Lookup for a child node of the given parent that isn't either port
	 * or ports.
	 */
	for_each_available_child_of_node(np, remote) {
		if (of_node_name_eq(remote, "port") ||
		    of_node_name_eq(remote, "ports"))
			continue;

		goto of_find_panel_or_bridge;
	}

	/*
	 * of_graph_get_remote_node() produces a noisy error message if port
	 * node isn't found and the absence of the port is a legit case here,
	 * so at first we silently check whether graph presents in the
	 * device-tree node.
	 */
	if (!of_graph_is_present(np))
		return -ENODEV;

	remote = of_graph_get_remote_node(np, 1, 0);

of_find_panel_or_bridge:
	if (!remote)
		return -ENODEV;

	panel = of_drm_find_panel(remote);
	if (!IS_ERR(panel)) {
		next_bridge = devm_drm_panel_bridge_add(dev, panel);
		if (IS_ERR(next_bridge)) {
			ret = PTR_ERR(next_bridge);
			next_bridge = NULL; // Inhibit the cleanup action on an ERR_PTR
		} else {
			drm_bridge_get(next_bridge);
		}
	} else {
		next_bridge = of_drm_find_and_get_bridge(remote);
		if (!next_bridge)
			ret = -EINVAL;
	}

	of_node_put(remote);

	if (ret) {
		DRM_DEV_ERROR(dev, "failed to find the bridge: %d\n", ret);
		return ret;
	}

	DRM_DEV_INFO(dev, "Attached %s device (lanes:%d bpp:%d mode-flags:0x%lx)\n",
		     device->name, device->lanes,
		     mipi_dsi_pixel_format_to_bpp(device->format),
		     device->mode_flags);

	drm_bridge_add(&dsi->bridge);

	/*
	 * This is a temporary solution and should be made by more generic way.
	 *
	 * If attached panel device is for command mode one, dsi should register
	 * TE interrupt handler.
	 */
	if (!(device->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		ret = samsung_dsim_register_te_irq(dsi, &device->dev);
		if (ret)
			goto err_remove_bridge;
	}

	// The next bridge can be used by host_ops->attach
	dsi->bridge.next_bridge = drm_bridge_get(next_bridge);

	if (pdata->host_ops && pdata->host_ops->attach) {
		ret = pdata->host_ops->attach(dsi, device);
		if (ret)
			goto err_release_next_bridge;
	}

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;
	dsi->dsc = device->dsc;

	return 0;

err_release_next_bridge:
	drm_bridge_clear_and_put(&dsi->bridge.next_bridge);
	if (!(device->mode_flags & MIPI_DSI_MODE_VIDEO))
		samsung_dsim_unregister_te_irq(dsi);
err_remove_bridge:
	drm_bridge_remove(&dsi->bridge);
	return ret;
}

static int samsung_dsim_host_detach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *device)
{
	struct samsung_dsim *dsi = host_to_dsi(host);
	const struct samsung_dsim_plat_data *pdata = dsi->plat_data;

	if (pdata->host_ops && pdata->host_ops->detach)
		pdata->host_ops->detach(dsi, device);

	drm_bridge_clear_and_put(&dsi->bridge.next_bridge);

	samsung_dsim_unregister_te_irq(dsi);

	drm_bridge_remove(&dsi->bridge);

	return 0;
}

static ssize_t samsung_dsim_host_transfer(struct mipi_dsi_host *host,
					  const struct mipi_dsi_msg *msg)
{
	struct samsung_dsim *dsi = host_to_dsi(host);
	struct samsung_dsim_transfer xfer;
	int ret;

	if ((dsi->state & DSIM_STATE_PRE_ENABLE_FAILED) ||
	    !(dsi->state & DSIM_STATE_ENABLED))
		return -EINVAL;

	ret = samsung_dsim_init(dsi);
	if (ret)
		return ret;

	ret = mipi_dsi_create_packet(&xfer.packet, msg);
	if (ret < 0)
		return ret;

	xfer.rx_len = msg->rx_len;
	xfer.rx_payload = msg->rx_buf;
	xfer.flags = msg->flags;

	ret = samsung_dsim_transfer(dsi, &xfer);
	return (ret < 0) ? ret : xfer.rx_done;
}

static const struct mipi_dsi_host_ops samsung_dsim_ops = {
	.attach = samsung_dsim_host_attach,
	.detach = samsung_dsim_host_detach,
	.transfer = samsung_dsim_host_transfer,
};

static int samsung_dsim_of_read_u32(const struct device_node *np,
				    const char *propname, u32 *out_value, bool optional)
{
	int ret = of_property_read_u32(np, propname, out_value);

	if (ret < 0 && !optional)
		pr_err("%pOF: failed to get '%s' property\n", np, propname);

	return ret;
}

static int samsung_dsim_parse_dt(struct samsung_dsim *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *node = dev->of_node;
	u32 lane_polarities[5] = { 0 };
	struct device_node *endpoint;
	int i, nr_lanes, ret;

	ret = samsung_dsim_of_read_u32(node, "samsung,pll-clock-frequency",
				       &dsi->pll_clk_rate, 1);
	/* If it doesn't exist, read it from the clock instead of failing */
	if (ret < 0) {
		dev_dbg(dev, "Using sclk_mipi for pll clock frequency\n");
		dsi->pll_clk = devm_clk_get(dev, "sclk_mipi");
		if (IS_ERR(dsi->pll_clk))
			return PTR_ERR(dsi->pll_clk);
	}

	/* If it doesn't exist, use pixel clock instead of failing */
	ret = samsung_dsim_of_read_u32(node, "samsung,burst-clock-frequency",
				       &dsi->burst_clk_rate, 1);
	if (ret < 0) {
		dev_dbg(dev, "Using pixel clock for HS clock frequency\n");
		dsi->burst_clk_rate = 0;
	}

	ret = samsung_dsim_of_read_u32(node, "samsung,esc-clock-frequency",
				       &dsi->esc_clk_rate, 0);
	if (ret < 0)
		return ret;

	endpoint = of_graph_get_endpoint_by_regs(node, 1, -1);
	nr_lanes = of_property_count_u32_elems(endpoint, "data-lanes");
	if (nr_lanes > 0 && nr_lanes <= 4) {
		/* Polarity 0 is clock lane, 1..4 are data lanes. */
		of_property_read_u32_array(endpoint, "lane-polarities",
					   lane_polarities, nr_lanes + 1);
		for (i = 1; i <= nr_lanes; i++) {
			if (lane_polarities[1] != lane_polarities[i])
				DRM_DEV_ERROR(dsi->dev, "Data lanes polarities do not match");
		}
		if (lane_polarities[0])
			dsi->swap_dn_dp_clk = true;
		if (lane_polarities[1])
			dsi->swap_dn_dp_data = true;
	}

	return 0;
}

static int generic_dsim_register_host(struct samsung_dsim *dsi)
{
	return mipi_dsi_host_register(&dsi->dsi_host);
}

static void generic_dsim_unregister_host(struct samsung_dsim *dsi)
{
	mipi_dsi_host_unregister(&dsi->dsi_host);
}

static const struct samsung_dsim_host_ops generic_dsim_host_ops = {
	.register_host = generic_dsim_register_host,
	.unregister_host = generic_dsim_unregister_host,
};

static const struct drm_bridge_timings samsung_dsim_bridge_timings_de_high = {
	.input_bus_flags = DRM_BUS_FLAG_DE_HIGH,
};

static const struct drm_bridge_timings samsung_dsim_bridge_timings_de_low = {
	.input_bus_flags = DRM_BUS_FLAG_DE_LOW,
};

int samsung_dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samsung_dsim *dsi;
	int ret;

	dsi = devm_drm_bridge_alloc(dev, struct samsung_dsim, bridge, &samsung_dsim_bridge_funcs);
	if (IS_ERR(dsi))
		return PTR_ERR(dsi);

	init_completion(&dsi->completed);
	mutex_init(&dsi->cmd_lock);
	spin_lock_init(&dsi->transfer_lock);
	INIT_LIST_HEAD(&dsi->transfer_list);

	dsi->dsi_host.ops = &samsung_dsim_ops;
	dsi->dsi_host.dev = dev;

	dsi->dev = dev;
	dsi->plat_data = of_device_get_match_data(dev);
	dsi->driver_data = samsung_dsim_types[dsi->plat_data->hw_type];

	dsi->supplies[0].supply = "vddcore";
	dsi->supplies[1].supply = "vddio";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(dsi->supplies),
				      dsi->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = devm_clk_bulk_get(dev, dsi->driver_data->num_clks,
				dsi->driver_data->clk_data);
	if (ret) {
		dev_err(dev, "failed to get clocks in bulk (%d)\n", ret);
		return ret;
	}

	dsi->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->reg_base))
		return PTR_ERR(dsi->reg_base);

	dsi->phy = devm_phy_optional_get(dev, "dsim");
	if (!dsi->phy && dsi->driver_data->uses_external_dphy_pll)
		dsi->phy = devm_phy_optional_get(dev, "dsim_dphy");
	if (IS_ERR(dsi->phy)) {
		dev_info(dev, "failed to get dsim phy\n");
		return PTR_ERR(dsi->phy);
	}

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0)
		return dsi->irq;

	ret = devm_request_threaded_irq(dev, dsi->irq, NULL,
					samsung_dsim_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), dsi);
	if (ret) {
		dev_err(dev, "failed to request dsi irq\n");
		return ret;
	}

	ret = samsung_dsim_parse_dt(dsi);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dsi);

	pm_runtime_enable(dev);

	dsi->bridge.of_node = dev->of_node;
	dsi->bridge.type = DRM_MODE_CONNECTOR_DSI;

	/* DE_LOW: i.MX8M Mini/Nano LCDIF-DSIM glue logic inverts HS/VS/DE */
	if (dsi->plat_data->hw_type == DSIM_TYPE_IMX8MM)
		dsi->bridge.timings = &samsung_dsim_bridge_timings_de_low;
	else
		dsi->bridge.timings = &samsung_dsim_bridge_timings_de_high;

	if (dsi->plat_data->host_ops && dsi->plat_data->host_ops->register_host) {
		ret = dsi->plat_data->host_ops->register_host(dsi);
		if (ret)
			goto err_disable_runtime;
	}

	return 0;

err_disable_runtime:
	pm_runtime_disable(dev);

	return ret;
}
EXPORT_SYMBOL_GPL(samsung_dsim_probe);

void samsung_dsim_remove(struct platform_device *pdev)
{
	struct samsung_dsim *dsi = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	if (dsi->plat_data->host_ops && dsi->plat_data->host_ops->unregister_host)
		dsi->plat_data->host_ops->unregister_host(dsi);
}
EXPORT_SYMBOL_GPL(samsung_dsim_remove);

static int samsung_dsim_suspend(struct device *dev)
{
	struct samsung_dsim *dsi = dev_get_drvdata(dev);
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	bool was_initialized = dsi->state & DSIM_STATE_INITIALIZED;
	int ret;

	usleep_range(10000, 20000);

	if (was_initialized) {
		dsi->state &= ~DSIM_STATE_INITIALIZED;

		if (driver_data->uses_external_dphy_pll) {
			samsung_dsim_zumapro_select_word_clock(dsi, false);
			phy_power_off(dsi->phy);
			phy_exit(dsi->phy);
		}

		samsung_dsim_disable_clock(dsi);
		samsung_dsim_disable_irq(dsi);
	}

	dsi->state &= ~DSIM_STATE_CMD_LPM;

	if (!driver_data->uses_external_dphy_pll)
		phy_power_off(dsi->phy);

	clk_bulk_disable_unprepare(driver_data->num_clks, driver_data->clk_data);

	ret = regulator_bulk_disable(ARRAY_SIZE(dsi->supplies), dsi->supplies);
	if (ret < 0)
		dev_err(dsi->dev, "cannot disable regulators %d\n", ret);

	return 0;
}

static int samsung_dsim_resume(struct device *dev)
{
	struct samsung_dsim *dsi = dev_get_drvdata(dev);
	const struct samsung_dsim_driver_data *driver_data = dsi->driver_data;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(dsi->supplies), dsi->supplies);
	if (ret < 0) {
		dev_err(dsi->dev, "cannot enable regulators %d\n", ret);
		return ret;
	}

	ret = clk_bulk_prepare_enable(driver_data->num_clks, driver_data->clk_data);
	if (ret < 0)
		goto err_clk;

	if (!driver_data->uses_external_dphy_pll) {
		ret = phy_power_on(dsi->phy);
		if (ret < 0) {
			dev_err(dsi->dev, "cannot enable phy %d\n", ret);
			goto err_clk;
		}
	}

	return 0;

err_clk:
	clk_bulk_disable_unprepare(driver_data->num_clks, driver_data->clk_data);
	regulator_bulk_disable(ARRAY_SIZE(dsi->supplies), dsi->supplies);

	return ret;
}

const struct dev_pm_ops samsung_dsim_pm_ops = {
	RUNTIME_PM_OPS(samsung_dsim_suspend, samsung_dsim_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};
EXPORT_SYMBOL_GPL(samsung_dsim_pm_ops);

static const struct samsung_dsim_plat_data samsung_dsim_imx8mm_pdata = {
	.hw_type = DSIM_TYPE_IMX8MM,
	.host_ops = &generic_dsim_host_ops,
};

static const struct samsung_dsim_plat_data samsung_dsim_imx8mp_pdata = {
	.hw_type = DSIM_TYPE_IMX8MP,
	.host_ops = &generic_dsim_host_ops,
};

static const struct of_device_id samsung_dsim_of_match[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &samsung_dsim_imx8mm_pdata,
	},
	{
		.compatible = "fsl,imx8mp-mipi-dsim",
		.data = &samsung_dsim_imx8mp_pdata,
	},
	{ /* sentinel. */ }
};
MODULE_DEVICE_TABLE(of, samsung_dsim_of_match);

static struct platform_driver samsung_dsim_driver = {
	.probe = samsung_dsim_probe,
	.remove = samsung_dsim_remove,
	.driver = {
		   .name = "samsung-dsim",
		   .pm = pm_ptr(&samsung_dsim_pm_ops),
		   .of_match_table = samsung_dsim_of_match,
	},
};

module_platform_driver(samsung_dsim_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Samsung MIPI DSIM controller bridge");
MODULE_LICENSE("GPL");
