// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos SoC series PCIe PHY driver
 *
 * Phy provider for PCIe controller on Exynos SoC series
 *
 * Copyright (C) 2017-2020 Samsung Electronics Co., Ltd.
 * Jaehoon Chung <jh80.chung@samsung.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>

#define PCIE_PHY_OFFSET(x)		((x) * 0x4)

/* Sysreg FSYS register offsets and bits for Exynos5433 */
#define PCIE_EXYNOS5433_PHY_MAC_RESET		0x0208
#define PCIE_MAC_RESET_MASK			0xFF
#define PCIE_MAC_RESET				BIT(4)
#define PCIE_EXYNOS5433_PHY_L1SUB_CM_CON	0x1010
#define PCIE_REFCLK_GATING_EN			BIT(0)
#define PCIE_EXYNOS5433_PHY_COMMON_RESET	0x1020
#define PCIE_PHY_RESET				BIT(0)
#define PCIE_EXYNOS5433_PHY_GLOBAL_RESET	0x1040
#define PCIE_GLOBAL_RESET			BIT(0)
#define PCIE_REFCLK				BIT(1)
#define PCIE_REFCLK_MASK			0x16
#define PCIE_APP_REQ_EXIT_L1_MODE		BIT(5)

/* PMU PCIE PHY isolation control */
#define EXYNOS5433_PMU_PCIE_PHY_OFFSET		0x730

struct exynos_pcie_phy;

struct exynos_pcie_phy_drvdata {
	const struct phy_ops *ops;
	/*
	 * Newer (Tensor/zuma) PHYs expose several register banks by name
	 * (phy/pcs/soc/udbg) and take the PHY isolation control from the PMU
	 * syscon; the Exynos5433 PHY has a single bank plus an FSYS sysreg.
	 */
	bool multi_region;
};

/* For Exynos pcie phy */
struct exynos_pcie_phy {
	const struct exynos_pcie_phy_drvdata *drvdata;
	void __iomem *base;		/* "phy" bank */
	void __iomem *pcs_base;		/* "pcs" bank (zuma) */
	void __iomem *soc_base;		/* "soc" bank (zuma) */
	void __iomem *udbg_base;	/* "udbg" bank (zuma) */
	struct regmap *pmureg;
	struct regmap *fsysreg;		/* Exynos5433 only */
	u32 pmu_offset;			/* PHY isolation control (zuma) */
	u32 num_lanes;
};

static void exynos_pcie_phy_writel(void __iomem *base, u32 val, u32 offset)
{
	writel(val, base + offset);
}

/* Exynos5433 specific functions */
static int exynos5433_pcie_phy_init(struct phy *phy)
{
	struct exynos_pcie_phy *ep = phy_get_drvdata(phy);

	regmap_update_bits(ep->pmureg, EXYNOS5433_PMU_PCIE_PHY_OFFSET,
			   BIT(0), 1);
	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_GLOBAL_RESET,
			   PCIE_APP_REQ_EXIT_L1_MODE, 0);
	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_L1SUB_CM_CON,
			   PCIE_REFCLK_GATING_EN, 0);

	regmap_update_bits(ep->fsysreg,	PCIE_EXYNOS5433_PHY_COMMON_RESET,
			   PCIE_PHY_RESET, 1);
	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_MAC_RESET,
			   PCIE_MAC_RESET, 0);

	/* PHY refclk 24MHz */
	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_GLOBAL_RESET,
			   PCIE_REFCLK_MASK, PCIE_REFCLK);
	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_GLOBAL_RESET,
			   PCIE_GLOBAL_RESET, 0);


	exynos_pcie_phy_writel(ep->base, 0x11, PCIE_PHY_OFFSET(0x3));

	/* band gap reference on */
	exynos_pcie_phy_writel(ep->base, 0, PCIE_PHY_OFFSET(0x20));
	exynos_pcie_phy_writel(ep->base, 0, PCIE_PHY_OFFSET(0x4b));

	/* jitter tuning */
	exynos_pcie_phy_writel(ep->base, 0x34, PCIE_PHY_OFFSET(0x4));
	exynos_pcie_phy_writel(ep->base, 0x02, PCIE_PHY_OFFSET(0x7));
	exynos_pcie_phy_writel(ep->base, 0x41, PCIE_PHY_OFFSET(0x21));
	exynos_pcie_phy_writel(ep->base, 0x7F, PCIE_PHY_OFFSET(0x14));
	exynos_pcie_phy_writel(ep->base, 0xC0, PCIE_PHY_OFFSET(0x15));
	exynos_pcie_phy_writel(ep->base, 0x61, PCIE_PHY_OFFSET(0x36));

	/* D0 uninit.. */
	exynos_pcie_phy_writel(ep->base, 0x44, PCIE_PHY_OFFSET(0x3D));

	/* 24MHz */
	exynos_pcie_phy_writel(ep->base, 0x94, PCIE_PHY_OFFSET(0x8));
	exynos_pcie_phy_writel(ep->base, 0xA7, PCIE_PHY_OFFSET(0x9));
	exynos_pcie_phy_writel(ep->base, 0x93, PCIE_PHY_OFFSET(0xA));
	exynos_pcie_phy_writel(ep->base, 0x6B, PCIE_PHY_OFFSET(0xC));
	exynos_pcie_phy_writel(ep->base, 0xA5, PCIE_PHY_OFFSET(0xF));
	exynos_pcie_phy_writel(ep->base, 0x34, PCIE_PHY_OFFSET(0x16));
	exynos_pcie_phy_writel(ep->base, 0xA3, PCIE_PHY_OFFSET(0x17));
	exynos_pcie_phy_writel(ep->base, 0xA7, PCIE_PHY_OFFSET(0x1A));
	exynos_pcie_phy_writel(ep->base, 0x71, PCIE_PHY_OFFSET(0x23));
	exynos_pcie_phy_writel(ep->base, 0x4C, PCIE_PHY_OFFSET(0x24));

	exynos_pcie_phy_writel(ep->base, 0x0E, PCIE_PHY_OFFSET(0x26));
	exynos_pcie_phy_writel(ep->base, 0x14, PCIE_PHY_OFFSET(0x7));
	exynos_pcie_phy_writel(ep->base, 0x48, PCIE_PHY_OFFSET(0x43));
	exynos_pcie_phy_writel(ep->base, 0x44, PCIE_PHY_OFFSET(0x44));
	exynos_pcie_phy_writel(ep->base, 0x03, PCIE_PHY_OFFSET(0x45));
	exynos_pcie_phy_writel(ep->base, 0xA7, PCIE_PHY_OFFSET(0x48));
	exynos_pcie_phy_writel(ep->base, 0x13, PCIE_PHY_OFFSET(0x54));
	exynos_pcie_phy_writel(ep->base, 0x04, PCIE_PHY_OFFSET(0x31));
	exynos_pcie_phy_writel(ep->base, 0, PCIE_PHY_OFFSET(0x32));

	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_COMMON_RESET,
			   PCIE_PHY_RESET, 0);
	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_MAC_RESET,
			   PCIE_MAC_RESET_MASK, PCIE_MAC_RESET);
	return 0;
}

static int exynos5433_pcie_phy_exit(struct phy *phy)
{
	struct exynos_pcie_phy *ep = phy_get_drvdata(phy);

	regmap_update_bits(ep->fsysreg, PCIE_EXYNOS5433_PHY_L1SUB_CM_CON,
			   PCIE_REFCLK_GATING_EN, PCIE_REFCLK_GATING_EN);
	regmap_update_bits(ep->pmureg, EXYNOS5433_PMU_PCIE_PHY_OFFSET,
			   BIT(0), 0);
	return 0;
}

static const struct phy_ops exynos5433_phy_ops = {
	.init		= exynos5433_pcie_phy_init,
	.exit		= exynos5433_pcie_phy_exit,
	.owner		= THIS_MODULE,
};

static const struct exynos_pcie_phy_drvdata exynos5433_pcie_phy_drvdata = {
	.ops		= &exynos5433_phy_ops,
	.multi_region	= false,
};

/*
 * Google Tensor (zuma/zumapro) Synopsys PCIe PHY (Gen3 x1).
 *
 * The init sequence is a fixed register table provided by the vendor, applied
 * while the controller holds the PMA in reset (the controller brackets
 * phy_init()/phy_power_on() with the ELBI soft-power/PMA-reset writes).
 */
#define ZUMA_PHY_INPUT_CLK		0x032C
#define ZUMA_PHY_INPUT_CLK_UNGATE	0x3
#define ZUMA_PHY_PLL_LOCK		0x0A80	/* bit 0 */
#define ZUMA_PHY_CDR_LOCK		0x15C0	/* bit 4 */
#define ZUMA_PHY_OC_DONE		0x140C	/* bit 7 */

#define ZUMA_UDBG_EXT_PLL		0xC700
#define ZUMA_UDBG_EXT_PLL_INIT		BIT(0)
#define ZUMA_UDBG_EXT_PLL_RESETB	BIT(1)
#define ZUMA_UDBG_EXT_PLL_LOCK		0xC734	/* bit 2 */

#define ZUMA_SOC_PHY_PWR		0x4000	/* all-power-down clear */
#define ZUMA_SOC_PHY_PWR_ON		0x15
#define ZUMA_SOC_PHY_CLK_SEL		0x4004
#define ZUMA_SOC_PHY_CLK_TCXO		BIT(2)	/* 0 = external PLL */
#define ZUMA_SOC_PHY_CLK_EXTPLL		(0x3 << 4)
#define ZUMA_SOC_PHY_CLK_INPUT_EN	0x3	/* bits[1:0]: input-clock enable */

#define ZUMA_PCS_PER_LANE		0x1000

struct zuma_phy_reg {
	u32 off;
	u32 val;
};

/* PMA common-block configuration (applied at the "phy" bank base). */
static const struct zuma_phy_reg zuma_pma_common[] = {
	{ 0x0000, 0x88 }, { 0x001C, 0x66 }, { 0x01F4, 0x00 }, { 0x0514, 0x59 },
	{ 0x051C, 0x11 }, { 0x062C, 0x0E }, { 0x0644, 0x22 }, { 0x0688, 0x03 },
	{ 0x06D4, 0x28 }, { 0x0788, 0x64 }, { 0x078C, 0x64 }, { 0x0790, 0x50 },
	{ 0x0794, 0x50 }, { 0x0944, 0x05 }, { 0x0948, 0x05 }, { 0x094C, 0x05 },
	{ 0x0950, 0x05 },
	/* REFCLK 38.4MHz to external-PLL path */
	{ 0x0590, 0x02 }, { 0x07F8, 0xB0 }, { 0x0730, 0x08 },
	/* delay @L1.2 = 7.68us */
	{ 0x0344, 0xC0 },
	/* decouple CLKREQ from L1.2 */
	{ 0x0040, 0x04 }, { 0x0204, 0x03 }, { 0x02D4, 0x1F }, { 0x0358, 0x10 },
	/* ERIO on */
	{ 0x0018, 0x01 },
	/* lane common */
	{ 0x0514, 0x5B }, { 0x0608, 0x0C }, { 0x060C, 0x0F }, { 0x0610, 0x0F },
	{ 0x0614, 0x0F }, { 0x0618, 0x0F }, { 0x0510, 0x80 }, { 0x0688, 0x03 },
	{ 0x0644, 0x23 }, { 0x0624, 0x11 },
	/* PLL margin for ERIO (Gen1 & Gen2) */
	{ 0x0630, 0x0F }, { 0x06D0, 0x53 },
};

/* Per-lane configuration (applied at "phy" base + lane * 0x1000). */
static const struct zuma_phy_reg zuma_pma_lane[] = {
	{ 0x1140, 0x04 }, { 0x1144, 0x04 }, { 0x1148, 0x04 }, { 0x114C, 0x02 },
	{ 0x1150, 0x00 }, { 0x1154, 0x00 }, { 0x1158, 0x00 }, { 0x115C, 0x00 },
	{ 0x12CC, 0x1C }, { 0x12DC, 0x6C }, { 0x130C, 0x29 }, { 0x13B4, 0x2F },
	{ 0x1A64, 0x05 }, { 0x1A68, 0x05 }, { 0x1A84, 0x05 }, { 0x1A88, 0x05 },
	{ 0x1A98, 0x00 }, { 0x1A9C, 0x00 }, { 0x1AA8, 0x07 }, { 0x1AB8, 0x00 },
	{ 0x1ABC, 0x00 }, { 0x1AF8, 0x90 }, { 0x1B34, 0x03 }, { 0x1BB0, 0x03 },
	{ 0x1BB4, 0x03 }, { 0x1BC0, 0x06 }, { 0x1BC4, 0x06 }, { 0x1BE8, 0x01 },
	{ 0x1BF8, 0x04 }, { 0x1C98, 0x00 }, { 0x1CA4, 0x81 },
};

/* BCM Wi-Fi endpoint tuning (L1SS exit link-down workaround). */
static const struct zuma_phy_reg zuma_pma_lane_bcm[] = {
	{ 0x1B20, 0x02 }, { 0x1340, 0x23 }, { 0x002C, 0x3C },
};

/* PCS configuration (applied at the "pcs" bank base). */
static const struct zuma_phy_reg zuma_pcs_cfg[] = {
	{ 0x190, 0x100B0604 }, { 0x154, 0x000700D5 }, { 0x100, 0x16400000 },
	{ 0x104, 0x08600000 }, { 0x114, 0x18500000 }, { 0x124, 0x60700000 },
	{ 0x174, 0x00000007 }, { 0x178, 0x00000100 },
};

static void zuma_phy_write_table(void __iomem *base,
				 const struct zuma_phy_reg *tbl, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		writel(tbl[i].val, base + tbl[i].off);
}

static bool zuma_phy_wait_bit(void __iomem *base, u32 off, u8 bit)
{
	u32 val;

	/*
	 * The external PLL and offset-calibration can take tens of
	 * milliseconds; poll for up to 100ms (10us * 10000), matching the
	 * vendor sequence. The external-PLL lock in particular must complete
	 * before it is selected as the PHY input clock, or the link will not
	 * train off the unstable clock.
	 */
	return !readl_poll_timeout(base + off, val, val & BIT(bit), 10, 100000);
}

/*
 * Bring up the PHY input clock and reference PLL. This must run *before* the
 * controller issues the ELBI soft-power/PMA reset (the vendor order), so that
 * the reset propagates with the PHY input clock ungated.
 */
static int zuma_pcie_phy_reset(struct phy *phy)
{
	struct exynos_pcie_phy *ep = phy_get_drvdata(phy);
	void __iomem *p = ep->base;
	u32 val;

	/* take the PHY out of isolation */
	regmap_update_bits(ep->pmureg, ep->pmu_offset, BIT(0), BIT(0));

	/*
	 * Clear the PHY power-downs. The vendor power-on does this before PHY
	 * config; firmware normally leaves it set, but a cold boot does not, so
	 * without it the PHY comes up with sub-blocks powered down and the link
	 * trains to Polling but never completes.
	 */
	writel(ZUMA_SOC_PHY_PWR_ON, ep->soc_base + ZUMA_SOC_PHY_PWR);

	/* ungate the PHY input clock */
	writel(ZUMA_PHY_INPUT_CLK_UNGATE, p + ZUMA_PHY_INPUT_CLK);

	/* release external PLL, override its RESETB, wait for lock */
	val = readl(ep->udbg_base + ZUMA_UDBG_EXT_PLL) | ZUMA_UDBG_EXT_PLL_INIT;
	writel(val, ep->udbg_base + ZUMA_UDBG_EXT_PLL);
	val = readl(ep->udbg_base + ZUMA_UDBG_EXT_PLL) & ~ZUMA_UDBG_EXT_PLL_RESETB;
	writel(val, ep->udbg_base + ZUMA_UDBG_EXT_PLL);
	if (!zuma_phy_wait_bit(ep->udbg_base, ZUMA_UDBG_EXT_PLL_LOCK, 2))
		dev_warn(&phy->dev, "external PLL lock timeout\n");

	/* select the external PLL as the PHY input clock */
	/*
	 * A working board reads 0x33 here: external-PLL select (bits 5:4) with
	 * bit 2 clear, plus the input-clock enable (bits 1:0) that firmware
	 * normally sets. The vendor PHY sequence only programs the select bits
	 * and relies on firmware for the enable, which a cold boot doesn't do.
	 */
	val = readl(ep->soc_base + ZUMA_SOC_PHY_CLK_SEL);
	val &= ~ZUMA_SOC_PHY_CLK_TCXO;
	val |= ZUMA_SOC_PHY_CLK_EXTPLL | ZUMA_SOC_PHY_CLK_INPUT_EN;
	writel(val, ep->soc_base + ZUMA_SOC_PHY_CLK_SEL);

	return 0;
}

static int zuma_pcie_phy_init(struct phy *phy)
{
	struct exynos_pcie_phy *ep = phy_get_drvdata(phy);
	void __iomem *p = ep->base;
	u32 lane, val;

	/* PMA: common block then each lane */
	zuma_phy_write_table(p, zuma_pma_common, ARRAY_SIZE(zuma_pma_common));
	for (lane = 0; lane < ep->num_lanes; lane++)
		zuma_phy_write_table(p + lane * ZUMA_PCS_PER_LANE,
				     zuma_pma_lane, ARRAY_SIZE(zuma_pma_lane));

	/* BCM Wi-Fi endpoint (komodo) tuning on lane 0 */
	zuma_phy_write_table(p, zuma_pma_lane_bcm, ARRAY_SIZE(zuma_pma_lane_bcm));

	val = readl(p + 0x1350) | BIT(0);
	writel(val, p + 0x1350);

	/* PCS */
	zuma_phy_write_table(ep->pcs_base, zuma_pcs_cfg, ARRAY_SIZE(zuma_pcs_cfg));
	writel(0x00000700, ep->pcs_base + 0x17c);	/* BCM Wi-Fi */
	writel(0x01600202, ep->pcs_base + 0x110);	/* BCM Wi-Fi */

	return 0;
}

static int zuma_pcie_phy_power_on(struct phy *phy)
{
	struct exynos_pcie_phy *ep = phy_get_drvdata(phy);

	/* the controller has released the PMA reset by now: check the locks */
	if (!zuma_phy_wait_bit(ep->base, ZUMA_PHY_PLL_LOCK, 0))
		dev_warn(&phy->dev, "PHY PLL lock timeout\n");
	if (!zuma_phy_wait_bit(ep->base, ZUMA_PHY_CDR_LOCK, 4))
		dev_warn(&phy->dev, "PHY CDR lock timeout\n");
	if (!zuma_phy_wait_bit(ep->base, ZUMA_PHY_OC_DONE, 7))
		dev_warn(&phy->dev, "PHY offset-calibration timeout\n");

	/* re-gate the PHY input clock */
	writel(0x0, ep->base + ZUMA_PHY_INPUT_CLK);
	return 0;
}

static int zuma_pcie_phy_power_off(struct phy *phy)
{
	struct exynos_pcie_phy *ep = phy_get_drvdata(phy);

	/* put the PHY back into isolation */
	regmap_update_bits(ep->pmureg, ep->pmu_offset, BIT(0), 0);
	return 0;
}

static const struct phy_ops zuma_phy_ops = {
	.reset		= zuma_pcie_phy_reset,
	.init		= zuma_pcie_phy_init,
	.power_on	= zuma_pcie_phy_power_on,
	.power_off	= zuma_pcie_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct exynos_pcie_phy_drvdata zuma_pcie_phy_drvdata = {
	.ops		= &zuma_phy_ops,
	.multi_region	= true,
};

static const struct of_device_id exynos_pcie_phy_match[] = {
	{
		.compatible = "samsung,exynos5433-pcie-phy",
		.data = &exynos5433_pcie_phy_drvdata,
	},
	{
		.compatible = "google,zuma-pcie-phy",
		.data = &zuma_pcie_phy_drvdata,
	},
	{},
};

static int exynos_pcie_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_pcie_phy *exynos_phy;
	struct phy *generic_phy;
	struct phy_provider *phy_provider;

	exynos_phy = devm_kzalloc(dev, sizeof(*exynos_phy), GFP_KERNEL);
	if (!exynos_phy)
		return -ENOMEM;

	exynos_phy->drvdata = of_device_get_match_data(dev);
	if (!exynos_phy->drvdata)
		return -EINVAL;

	exynos_phy->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
							"samsung,pmu-syscon");
	if (IS_ERR(exynos_phy->pmureg))
		return dev_err_probe(dev, PTR_ERR(exynos_phy->pmureg),
				     "PMU regmap lookup failed\n");

	if (exynos_phy->drvdata->multi_region) {
		exynos_phy->base =
			devm_platform_ioremap_resource_byname(pdev, "phy");
		if (IS_ERR(exynos_phy->base))
			return PTR_ERR(exynos_phy->base);

		exynos_phy->pcs_base =
			devm_platform_ioremap_resource_byname(pdev, "pcs");
		if (IS_ERR(exynos_phy->pcs_base))
			return PTR_ERR(exynos_phy->pcs_base);

		exynos_phy->soc_base =
			devm_platform_ioremap_resource_byname(pdev, "soc");
		if (IS_ERR(exynos_phy->soc_base))
			return PTR_ERR(exynos_phy->soc_base);

		exynos_phy->udbg_base =
			devm_platform_ioremap_resource_byname(pdev, "udbg");
		if (IS_ERR(exynos_phy->udbg_base))
			return PTR_ERR(exynos_phy->udbg_base);

		if (of_property_read_u32(dev->of_node, "samsung,pmu-offset",
					 &exynos_phy->pmu_offset))
			return dev_err_probe(dev, -EINVAL,
					     "missing samsung,pmu-offset\n");

		exynos_phy->num_lanes = 1;
		of_property_read_u32(dev->of_node, "num-lanes",
				     &exynos_phy->num_lanes);
	} else {
		exynos_phy->base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(exynos_phy->base))
			return PTR_ERR(exynos_phy->base);

		exynos_phy->fsysreg = syscon_regmap_lookup_by_phandle(
				dev->of_node, "samsung,fsys-sysreg");
		if (IS_ERR(exynos_phy->fsysreg))
			return dev_err_probe(dev, PTR_ERR(exynos_phy->fsysreg),
					     "FSYS sysreg regmap lookup failed\n");
	}

	generic_phy = devm_phy_create(dev, dev->of_node, exynos_phy->drvdata->ops);
	if (IS_ERR(generic_phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(generic_phy);
	}

	phy_set_drvdata(generic_phy, exynos_phy);
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver exynos_pcie_phy_driver = {
	.probe	= exynos_pcie_phy_probe,
	.driver = {
		.of_match_table	= exynos_pcie_phy_match,
		.name		= "exynos_pcie_phy",
		.suppress_bind_attrs = true,
	}
};
builtin_platform_driver(exynos_pcie_phy_driver);
