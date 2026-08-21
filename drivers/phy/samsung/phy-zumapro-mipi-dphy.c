// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor G4 / Samsung Zumapro MIPI D-PHY provider.
 *
 * This programs the M4M4 D-PHY block used by DSIM0.  The register values and
 * order follow downstream cal_9865/dsim_reg.c; SYSREG reset control is kept
 * optional until the mainline DT binding is finalized.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/units.h>

#define ZUMAPRO_DPHY_FIN_DEFAULT	24576000UL
#define ZUMAPRO_DPHY_PLL_OPTIMUM	13000000UL
#define ZUMAPRO_DPHY_FOUT_MIN		52000000ULL
#define ZUMAPRO_DPHY_FOUT_MAX		6600000000ULL
#define ZUMAPRO_DPHY_FVCO_MIN		3300000000ULL
#define ZUMAPRO_DPHY_FVCO_MAX		6600000000ULL
#define ZUMAPRO_DPHY_K_BITS		16

#define DISP_DPU_MIPI_PHY_CON		0x0008
#define SEL_RESET_DPHY_MASK(x)		BIT(4 + (x))
#define M_RESETN_M0_MASK		BIT(0)
#define M_RESETN_M1_MASK		BIT(1)

#define DSIM_PHY_BIAS_CON(x)		(0x0000 + (4 * (x)))

#define DSIM_PHY_PLL_CON(x)		(0x0000 + (4 * (x)))
#define DSIM_PHY_PLL_CON0		0x0000
#define DSIM_PHY_PLL_CON1		0x0004
#define DSIM_PHY_PLL_CON2		0x0008
#define DSIM_PHY_PLL_CON5		0x0014
#define DSIM_PHY_PLL_CON6		0x0018
#define DSIM_PHY_PLL_CON7		0x001c
#define DSIM_PHY_PLL_STAT0		0x0040

#define DSIM_PHY_PLL_EN_MASK		BIT(12)
#define DSIM_PHY_PMS_S(x)		(((x) & 0x7) << 8)
#define DSIM_PHY_PMS_S_MASK		(0x7 << 8)
#define DSIM_PHY_PMS_P(x)		(((x) & 0x3f) << 0)
#define DSIM_PHY_PMS_P_MASK		(0x3f << 0)
#define DSIM_PHY_PMS_K(x)		(((x) & 0xffff) << 0)
#define DSIM_PHY_PMS_M(x)		(((x) & 0x3ff) << 0)
#define DSIM_PHY_PMS_M_MASK		(0x3ff << 0)
#define DSIM_PHY_DITHER_SEL_VCO(x)	((x) << 2)
#define DSIM_PHY_DITHER_SEL_VCO_MASK	BIT(2)
#define DSIM_PHY_WCLK_BUF_SFT_CNT(x)	(((x) & 0xf) << 8)
#define DSIM_PHY_WCLK_BUF_SFT_CNT_MASK	(0xf << 8)
#define DSIM_PHY_PLL_LOCK_CNT(x)	(((x) & 0xffff) << 0)
#define DSIM_PHY_PLL_LOCK_GET(x)	(((x) >> 0) & 0x1)

#define DSIM_PHY_MC_GNR_CON(x)		(0x0200 + (4 * (x)))
#define DSIM_PHY_MC_GNR_CON0		0x0200
#define DSIM_PHY_MC_ANA_CON(x)		(0x0208 + (4 * (x)))
#define DSIM_PHY_MC_TIME_CON0		0x0230
#define DSIM_PHY_MC_TIME_CON1		0x0234
#define DSIM_PHY_MC_TIME_CON2		0x0238
#define DSIM_PHY_MC_TIME_CON3		0x023c
#define DSIM_PHY_MC_TIME_CON4		0x0240
#define DSIM_PHY_MC_DESKEW_CON0	0x0250

#define DSIM_PHY_MD_GNR_CON0(x)		(0x0300 + (0x100 * (x)))
#define DSIM_PHY_MD_GNR_CON1(x)		(0x0304 + (0x100 * (x)))
#define DSIM_PHY_MD_ANA_CON0(x)		(0x0308 + (0x100 * (x)))
#define DSIM_PHY_MD_ANA_CON1(x)		(0x030c + (0x100 * (x)))
#define DSIM_PHY_MD_ANA_CON2(x)		(0x0310 + (0x100 * (x)))
#define DSIM_PHY_MD_ANA_CON3(x)		(0x0314 + (0x100 * (x)))
#define DSIM_PHY_MD_TIME_CON0(x)	(0x0330 + (0x100 * (x)))
#define DSIM_PHY_MD_TIME_CON1(x)	(0x0334 + (0x100 * (x)))
#define DSIM_PHY_MD_TIME_CON2(x)	(0x0338 + (0x100 * (x)))
#define DSIM_PHY_MD_TIME_CON3(x)	(0x033c + (0x100 * (x)))
#define DSIM_PHY_MD_TIME_CON4(x)	(0x0340 + (0x100 * (x)))

#define DSIM_PHY_PHY_READY_GET(x)	(((x) >> 1) & 0x1)
#define DSIM_PHY_PHY_ENABLE		BIT(0)
#define DSIM_PHY_HSTX_CLK_SEL(x)	(((x) & 0x1) << 12)
#define DSIM_PHY_TLPX(x)		(((x) & 0xff) << 4)
#define DSIM_PHY_TLP_EXIT_SKEW(x)	(((x) & 0x3) << 2)
#define DSIM_PHY_TLP_ENTRY_SKEW(x)	(((x) & 0x3) << 0)
#define DSIM_PHY_TCLK_ZERO(x)		(((x) & 0xff) << 8)
#define DSIM_PHY_TCLK_PREPARE(x)	(((x) & 0xff) << 0)
#define DSIM_PHY_THS_ZERO(x)		(((x) & 0xff) << 8)
#define DSIM_PHY_THS_PREPARE(x)		(((x) & 0xff) << 0)
#define DSIM_PHY_THS_EXIT(x)		(((x) & 0xff) << 8)
#define DSIM_PHY_TCLK_TRAIL(x)		(((x) & 0xff) << 0)
#define DSIM_PHY_THS_TRAIL(x)		(((x) & 0xff) << 0)
#define DSIM_PHY_TCLK_POST(x)		(((x) & 0xff) << 0)
#define DSIM_PHY_TTA_GET(x)		(((x) & 0xf) << 4)
#define DSIM_PHY_TTA_GO(x)		(((x) & 0xf) << 0)
#define DSIM_PHY_ULPS_EXIT(x)		(((x) & 0x3ff) << 0)

struct zumapro_dphy_timing {
	u32 bps;
	u32 clk_prepare;
	u32 clk_zero;
	u32 clk_post;
	u32 clk_trail;
	u32 hs_prepare;
	u32 hs_zero;
	u32 hs_trail;
	u32 lpx;
	u32 hs_exit;
	u32 b_dphyctl;
};

struct zumapro_mipi_dphy {
	struct device *dev;
	void __iomem *dphy;
	void __iomem *bias;
	struct regmap *sysreg;
	struct clk *ref_clk;
	struct phy *phy;
	u32 id;
	u32 p;
	u32 m;
	u32 s;
	u32 k;
	u32 hs_clk_mhz;
	u32 esc_clk_mhz;
	u8 lanes;
	struct zumapro_dphy_timing timing;
};

static const u32 zumapro_dphy_bias_con[] = {
	0x00000010, 0x00000110, 0x00003223,
	0x00000000, 0x00000200, 0x00000002,
};

static const u32 zumapro_dphy_pll_con[] = {
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000500, 0x0000008e, 0x00003d40, 0x00001e00, 0x00001300,
};

static const u32 zumapro_dphy_mc_gnr_con[] = {
	0x00000000, 0x00001334,
};

static const u32 zumapro_dphy_mc_ana_con[] = {
	0x00007122, 0x00000000, 0x00000002, 0x00000000,
};

static const u32 zumapro_dphy_md_gnr_con[] = {
	0x00000000, 0x00001334,
};

static const u32 zumapro_dphy_md_ana_con[] = {
	0x00007122, 0x00000000, 0x00000000, 0x00000000,
};

/*
 * Rows are the downstream dphy_timing[] table (cal_9865/dsim_reg.c), which is
 * quantised to 10 MHz steps.  zumapro_dphy_get_timing() scans upwards from the
 * last entry and takes the first row at or above the requested rate, so the
 * table must stay sorted by descending bps.
 *
 * 1370/1290 cover the komodo (google,gs-km4) panel's 1368 Mbps link and its
 * 1288 Mbps FFC alternate.
 */
static const struct zumapro_dphy_timing zumapro_dphy_timings[] = {
	{ 1370, 54, 21, 5, 66, 54, 8, 67, 41, 8 },
	{ 1290, 51, 20, 5, 63, 50, 8, 64, 38, 7 },
	{ 1120, 44, 17, 4, 57, 44, 6, 58, 33, 6 },
	{ 1110, 43, 16, 4, 57, 44, 6, 58, 33, 6 },
	{ 1100, 43, 16, 4, 56, 43, 6, 58, 32, 6 },
	{ 1010, 39, 15, 4, 53, 40, 5, 55, 30, 5 },
	{ 1000, 39, 14, 3, 53, 39, 5, 54, 29, 5 },
	{  990, 39, 14, 3, 53, 39, 5, 54, 29, 5 },
};

static const u32 zumapro_dphy_b_dphyctl[] = {
	0x0af, 0x0c8, 0x0e1, 0x0fa, 0x113, 0x12c, 0x145,
	0x15e, 0x177, 0x190, 0x1a9, 0x1c2, 0x1db, 0x1f4,
};

static void zumapro_dphy_update_bits(void __iomem *base, u32 offset,
				     u32 mask, u32 val)
{
	u32 reg = readl(base + offset);

	reg &= ~mask;
	reg |= val & mask;
	writel(reg, base + offset);
}

static void zumapro_dphy_sysreg_update(struct zumapro_mipi_dphy *dphy,
				       u32 mask, u32 val)
{
	if (dphy->sysreg)
		regmap_update_bits(dphy->sysreg, DISP_DPU_MIPI_PHY_CON,
				   mask, val);
}

static int zumapro_dphy_calc_pmsk(struct zumapro_mipi_dphy *dphy,
				  unsigned long ref_rate, u32 hs_mhz)
{
	u64 hs_clk = (u64)hs_mhz * HZ_PER_MHZ;
	u64 fvco = 0;
	u64 q;
	u32 p;
	u32 m;
	u32 s;
	u32 k;

	if (hs_clk < ZUMAPRO_DPHY_FOUT_MIN || hs_clk > ZUMAPRO_DPHY_FOUT_MAX)
		return dev_err_probe(dphy->dev, -EINVAL,
				     "HS clock %u MHz out of range\n", hs_mhz);

	p = DIV_ROUND_CLOSEST(ref_rate, ZUMAPRO_DPHY_PLL_OPTIMUM);
	if (!p)
		p = 1;
	if (p > 0x3f)
		return dev_err_probe(dphy->dev, -EINVAL,
				     "PLL P value %u out of range\n", p);

	for (s = 0; fvco < ZUMAPRO_DPHY_FVCO_MIN; s++)
		fvco = hs_clk * (1 << s);
	s--;

	if (fvco > ZUMAPRO_DPHY_FVCO_MAX || s > 6)
		return dev_err_probe(dphy->dev, -EINVAL,
				     "no valid PLL S for %u MHz\n", hs_mhz);

	fvco >>= 1;
	q = fvco << (ZUMAPRO_DPHY_K_BITS + 1);
	do_div(q, ref_rate / p);

	m = q >> (ZUMAPRO_DPHY_K_BITS + 1);
	if (m < 64 || m > 1023)
		return dev_err_probe(dphy->dev, -EINVAL,
				     "PLL M value %u out of range\n", m);

	k = q & ((1 << (ZUMAPRO_DPHY_K_BITS + 1)) - 1);
	k = DIV_ROUND_UP(k, 2);
	if (k & BIT(ZUMAPRO_DPHY_K_BITS - 1))
		m++;

	dphy->p = p;
	dphy->m = m;
	dphy->s = s;
	dphy->k = k;

	return 0;
}

static int zumapro_dphy_get_timing(struct zumapro_mipi_dphy *dphy,
				   u32 hs_mhz, u32 esc_mhz)
{
	int i;

	for (i = ARRAY_SIZE(zumapro_dphy_timings) - 1; i >= 0; i--) {
		if (zumapro_dphy_timings[i].bps >= hs_mhz) {
			dphy->timing = zumapro_dphy_timings[i];
			break;
		}
	}

	if (i < 0)
		return dev_err_probe(dphy->dev, -EINVAL,
				     "no timing row for %u MHz HS clock\n",
				     hs_mhz);

	if (esc_mhz < 7 || esc_mhz > 20)
		return dev_err_probe(dphy->dev, -EINVAL,
				     "%u MHz escape clock is out of range\n",
				     esc_mhz);

	dphy->timing.bps = hs_mhz;
	dphy->timing.b_dphyctl = zumapro_dphy_b_dphyctl[esc_mhz - 7];

	return 0;
}

static u32 zumapro_dphy_adjust_esc_clk(u32 hs_mhz, u32 req_esc_mhz)
{
	u32 word_mhz = hs_mhz / 16;
	u32 esc_div;

	if (!req_esc_mhz || !word_mhz)
		return 0;

	esc_div = word_mhz / req_esc_mhz;
	if (!esc_div)
		esc_div = 1;
	if (word_mhz / esc_div > req_esc_mhz)
		esc_div++;

	return word_mhz / esc_div;
}

static void zumapro_dphy_set_pll_freq(struct zumapro_mipi_dphy *dphy)
{
	writel(DSIM_PHY_PMS_K(dphy->k), dphy->dphy + DSIM_PHY_PLL_CON1);
	zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_PLL_CON0,
				 DSIM_PHY_PMS_P_MASK | DSIM_PHY_PMS_S_MASK,
				 DSIM_PHY_PMS_P(dphy->p) |
				 DSIM_PHY_PMS_S(dphy->s));
	zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_PLL_CON2,
				 DSIM_PHY_PMS_M_MASK, DSIM_PHY_PMS_M(dphy->m));
}

static void zumapro_dphy_set_timing(struct zumapro_mipi_dphy *dphy)
{
	const struct zumapro_dphy_timing *t = &dphy->timing;
	u32 hs_en = dphy->hs_clk_mhz < 1500;
	u32 skewcal_en = !hs_en;
	int i;

	writel(DSIM_PHY_ULPS_EXIT(t->b_dphyctl),
	       dphy->dphy + DSIM_PHY_MC_TIME_CON4);
	writel(DSIM_PHY_HSTX_CLK_SEL(hs_en) | DSIM_PHY_TLPX(t->lpx),
	       dphy->dphy + DSIM_PHY_MC_TIME_CON0);
	writel(skewcal_en, dphy->dphy + DSIM_PHY_MC_DESKEW_CON0);
	writel(DSIM_PHY_TCLK_PREPARE(t->clk_prepare) |
	       DSIM_PHY_TCLK_ZERO(t->clk_zero),
	       dphy->dphy + DSIM_PHY_MC_TIME_CON1);
	writel(DSIM_PHY_THS_EXIT(t->hs_exit) |
	       DSIM_PHY_TCLK_TRAIL(t->clk_trail),
	       dphy->dphy + DSIM_PHY_MC_TIME_CON2);
	writel(DSIM_PHY_TCLK_POST(t->clk_post),
	       dphy->dphy + DSIM_PHY_MC_TIME_CON3);

	for (i = 0; i < dphy->lanes; i++) {
		writel(DSIM_PHY_ULPS_EXIT(t->b_dphyctl),
		       dphy->dphy + DSIM_PHY_MD_TIME_CON4(i));
		writel(DSIM_PHY_HSTX_CLK_SEL(hs_en) | DSIM_PHY_TLPX(t->lpx) |
		       DSIM_PHY_TLP_EXIT_SKEW(0) | DSIM_PHY_TLP_ENTRY_SKEW(0),
		       dphy->dphy + DSIM_PHY_MD_TIME_CON0(i));
		writel(DSIM_PHY_THS_PREPARE(t->hs_prepare) |
		       DSIM_PHY_THS_ZERO(t->hs_zero),
		       dphy->dphy + DSIM_PHY_MD_TIME_CON1(i));
		writel(DSIM_PHY_THS_EXIT(t->hs_exit) |
		       DSIM_PHY_THS_TRAIL(t->hs_trail),
		       dphy->dphy + DSIM_PHY_MD_TIME_CON2(i));
		writel(DSIM_PHY_TTA_GET(3) | DSIM_PHY_TTA_GO(0),
		       dphy->dphy + DSIM_PHY_MD_TIME_CON3(i));
	}
}

static void zumapro_dphy_write_defaults(struct zumapro_mipi_dphy *dphy)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_dphy_bias_con); i++)
		writel(zumapro_dphy_bias_con[i],
		       dphy->bias + DSIM_PHY_BIAS_CON(i));

	for (i = 0; i < ARRAY_SIZE(zumapro_dphy_pll_con); i++)
		writel(zumapro_dphy_pll_con[i], dphy->dphy + DSIM_PHY_PLL_CON(i));

	for (i = 0; i < ARRAY_SIZE(zumapro_dphy_mc_gnr_con); i++)
		writel(zumapro_dphy_mc_gnr_con[i],
		       dphy->dphy + DSIM_PHY_MC_GNR_CON(i));

	for (i = 0; i < ARRAY_SIZE(zumapro_dphy_mc_ana_con); i++)
		writel(zumapro_dphy_mc_ana_con[i],
		       dphy->dphy + DSIM_PHY_MC_ANA_CON(i));

	for (i = 0; i < dphy->lanes; i++) {
		writel(zumapro_dphy_md_gnr_con[0],
		       dphy->dphy + DSIM_PHY_MD_GNR_CON0(i));
		writel(zumapro_dphy_md_gnr_con[1],
		       dphy->dphy + DSIM_PHY_MD_GNR_CON1(i));
		writel(zumapro_dphy_md_ana_con[0],
		       dphy->dphy + DSIM_PHY_MD_ANA_CON0(i));
		writel(zumapro_dphy_md_ana_con[1],
		       dphy->dphy + DSIM_PHY_MD_ANA_CON1(i));
		writel(zumapro_dphy_md_ana_con[2],
		       dphy->dphy + DSIM_PHY_MD_ANA_CON2(i));
		writel(zumapro_dphy_md_ana_con[3],
		       dphy->dphy + DSIM_PHY_MD_ANA_CON3(i));
	}
}

static int zumapro_dphy_wait_pll(struct zumapro_mipi_dphy *dphy, bool on)
{
	u32 val;

	return readl_poll_timeout(dphy->dphy + DSIM_PHY_PLL_STAT0, val,
				  !!DSIM_PHY_PLL_LOCK_GET(val) == on,
				  10, 2000);
}

static int zumapro_dphy_set_pll(struct zumapro_mipi_dphy *dphy, bool on)
{
	u32 val = on ? DSIM_PHY_PLL_EN_MASK : 0;
	int ret;

	zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_PLL_CON0,
				 DSIM_PHY_PLL_EN_MASK, val);

	ret = zumapro_dphy_wait_pll(dphy, on);
	if (ret)
		return dev_err_probe(dphy->dev, ret,
				     "PHY PLL %s timed out\n",
				     on ? "enable" : "disable");

	return 0;
}

static int zumapro_dphy_wait_lane_ready(struct zumapro_mipi_dphy *dphy,
					u32 offset, bool on)
{
	u32 val;

	return readl_poll_timeout(dphy->dphy + offset, val,
				  !!DSIM_PHY_PHY_READY_GET(val) == on,
				  10, 2000);
}

static int zumapro_dphy_enable_lanes(struct zumapro_mipi_dphy *dphy, bool on)
{
	u32 val = on ? DSIM_PHY_PHY_ENABLE : 0;
	int ret;
	int i;

	zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_MC_GNR_CON0,
				 DSIM_PHY_PHY_ENABLE, val);
	for (i = 0; i < dphy->lanes; i++)
		zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_MD_GNR_CON0(i),
					 DSIM_PHY_PHY_ENABLE, val);

	ret = zumapro_dphy_wait_lane_ready(dphy, DSIM_PHY_MC_GNR_CON0, on);
	if (ret)
		return dev_err_probe(dphy->dev, ret,
				     "clock lane ready timed out\n");

	for (i = 0; i < dphy->lanes; i++) {
		ret = zumapro_dphy_wait_lane_ready(dphy,
						   DSIM_PHY_MD_GNR_CON0(i),
						   on);
		if (ret)
			return dev_err_probe(dphy->dev, ret,
					     "data lane %d ready timed out\n",
					     i);
	}

	return 0;
}

static int zumapro_mipi_dphy_configure(struct phy *phy,
				       union phy_configure_opts *opts)
{
	struct zumapro_mipi_dphy *dphy = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *cfg = &opts->mipi_dphy;
	unsigned long ref_rate;
	u32 esc_mhz;
	u32 hs_mhz;
	int ret;

	if (!cfg->hs_clk_rate || !cfg->lanes || cfg->lanes > 4)
		return -EINVAL;

	ref_rate = clk_get_rate(dphy->ref_clk);
	if (!ref_rate)
		ref_rate = ZUMAPRO_DPHY_FIN_DEFAULT;

	hs_mhz = DIV_ROUND_CLOSEST_ULL(cfg->hs_clk_rate, HZ_PER_MHZ);
	esc_mhz = cfg->lp_clk_rate ?
		  DIV_ROUND_CLOSEST_ULL(cfg->lp_clk_rate, HZ_PER_MHZ) : 20;
	esc_mhz = zumapro_dphy_adjust_esc_clk(hs_mhz, esc_mhz);

	ret = zumapro_dphy_calc_pmsk(dphy, ref_rate, hs_mhz);
	if (ret)
		return ret;

	dphy->hs_clk_mhz = hs_mhz;
	dphy->esc_clk_mhz = esc_mhz;
	dphy->lanes = cfg->lanes;

	return zumapro_dphy_get_timing(dphy, hs_mhz, esc_mhz);
}

static int zumapro_mipi_dphy_init(struct phy *phy)
{
	struct zumapro_mipi_dphy *dphy = phy_get_drvdata(phy);
	int ret;

	ret = clk_prepare_enable(dphy->ref_clk);
	if (ret)
		return ret;

	zumapro_dphy_sysreg_update(dphy, SEL_RESET_DPHY_MASK(dphy->id), 0);

	return 0;
}

static int zumapro_mipi_dphy_exit(struct phy *phy)
{
	struct zumapro_mipi_dphy *dphy = phy_get_drvdata(phy);

	clk_disable_unprepare(dphy->ref_clk);

	return 0;
}

static int zumapro_mipi_dphy_power_on(struct phy *phy)
{
	struct zumapro_mipi_dphy *dphy = phy_get_drvdata(phy);
	u32 reset_mask = dphy->id ? M_RESETN_M1_MASK : M_RESETN_M0_MASK;
	int ret;

	if (!dphy->hs_clk_mhz || !dphy->lanes)
		return -EINVAL;

	/*
	 * Always bring the PHY up from scratch rather than adopting whatever the
	 * bootloader left running.
	 *
	 * Adopting a live link (downstream's "DPHY PLL is already stable" path)
	 * only makes sense while the splash scanout is still using it.  By the
	 * time this runs, samsung_dsim_init() has already software-reset the DSIM
	 * and the panel has been reset in prepare(), so there is no scanout left
	 * to preserve -- and the shortcut skips both the PLL reprogramming, which
	 * leaves the link at the bootloader's rate rather than the mode's, and the
	 * M_RESETN release below, so the DSIM never sees TX_READY_HSCLK even
	 * though the PLL reads locked and the lanes read enabled.
	 */

	zumapro_dphy_sysreg_update(dphy, reset_mask, 0);
	zumapro_dphy_write_defaults(dphy);

	if ((dphy->hs_clk_mhz << dphy->s) < 3000)
		zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_PLL_CON5,
					 DSIM_PHY_DITHER_SEL_VCO_MASK,
					 DSIM_PHY_DITHER_SEL_VCO(1));

	zumapro_dphy_set_timing(dphy);
	zumapro_dphy_set_pll_freq(dphy);
	zumapro_dphy_update_bits(dphy->dphy, DSIM_PHY_PLL_CON6,
				 DSIM_PHY_WCLK_BUF_SFT_CNT_MASK,
				 DSIM_PHY_WCLK_BUF_SFT_CNT(3));
	writel(DSIM_PHY_PLL_LOCK_CNT(500 * dphy->p),
	       dphy->dphy + DSIM_PHY_PLL_CON7);

	ret = zumapro_dphy_set_pll(dphy, true);
	if (ret)
		return ret;

	ret = zumapro_dphy_enable_lanes(dphy, true);
	if (ret)
		goto err_disable_pll;

	zumapro_dphy_sysreg_update(dphy, reset_mask, reset_mask);

	dev_dbg(dphy->dev, "configured HS=%u MHz ESC=%u MHz lanes=%u PMSK=%u,%u,%u,0x%x\n",
		dphy->hs_clk_mhz, dphy->esc_clk_mhz, dphy->lanes,
		dphy->p, dphy->m, dphy->s, dphy->k);

	return 0;

err_disable_pll:
	zumapro_dphy_set_pll(dphy, false);
	return ret;
}

static int zumapro_mipi_dphy_power_off(struct phy *phy)
{
	struct zumapro_mipi_dphy *dphy = phy_get_drvdata(phy);
	u32 reset_mask = dphy->id ? M_RESETN_M1_MASK : M_RESETN_M0_MASK;

	zumapro_dphy_sysreg_update(dphy, reset_mask, 0);
	zumapro_dphy_enable_lanes(dphy, false);
	zumapro_dphy_set_pll(dphy, false);

	return 0;
}

static struct phy *zumapro_mipi_dphy_xlate(struct device *dev,
					   const struct of_phandle_args *args)
{
	struct zumapro_mipi_dphy *dphy = dev_get_drvdata(dev);

	if (args->args_count && args->args[0] != dphy->id)
		return ERR_PTR(-ENODEV);

	return dphy->phy;
}

static const struct phy_ops zumapro_mipi_dphy_ops = {
	.init = zumapro_mipi_dphy_init,
	.exit = zumapro_mipi_dphy_exit,
	.configure = zumapro_mipi_dphy_configure,
	.power_on = zumapro_mipi_dphy_power_on,
	.power_off = zumapro_mipi_dphy_power_off,
	.owner = THIS_MODULE,
};

static int zumapro_mipi_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct zumapro_mipi_dphy *dphy;
	void __iomem *base;
	int ret;

	dphy = devm_kzalloc(dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;

	dphy->dev = dev;

	base = devm_platform_ioremap_resource_byname(pdev, "bias");
	if (IS_ERR(base))
		base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	dphy->bias = base;

	base = devm_platform_ioremap_resource_byname(pdev, "dphy");
	if (IS_ERR(base))
		base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(base))
		return PTR_ERR(base);
	dphy->dphy = base;

	dphy->ref_clk = devm_clk_get_optional(dev, "ref");
	if (IS_ERR(dphy->ref_clk))
		return dev_err_probe(dev, PTR_ERR(dphy->ref_clk),
				     "failed to get ref clock\n");

	dphy->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "samsung,sysreg");
	if (IS_ERR(dphy->sysreg))
		dphy->sysreg = NULL;

	of_property_read_u32(dev->of_node, "samsung,id", &dphy->id);

	dphy->phy = devm_phy_create(dev, NULL, &zumapro_mipi_dphy_ops);
	if (IS_ERR(dphy->phy))
		return PTR_ERR(dphy->phy);

	phy_set_drvdata(dphy->phy, dphy);
	dev_set_drvdata(dev, dphy);

	provider = devm_of_phy_provider_register(dev, zumapro_mipi_dphy_xlate);
	ret = PTR_ERR_OR_ZERO(provider);
	if (ret)
		return ret;

	dev_info(dev, "registered Zumapro M4M4 MIPI D-PHY\n");
	return 0;
}

static const struct of_device_id zumapro_mipi_dphy_of_match[] = {
	{ .compatible = "google,zumapro-mipi-dphy" },
	{ .compatible = "samsung,mipi-phy-m4m4" },
	{ }
};
MODULE_DEVICE_TABLE(of, zumapro_mipi_dphy_of_match);

static struct platform_driver zumapro_mipi_dphy_driver = {
	.probe = zumapro_mipi_dphy_probe,
	.driver = {
		.name = "zumapro-mipi-dphy",
		.of_match_table = zumapro_mipi_dphy_of_match,
	},
};
module_platform_driver(zumapro_mipi_dphy_driver);

MODULE_DESCRIPTION("Google Tensor G4 Zumapro M4M4 MIPI D-PHY driver");
MODULE_LICENSE("GPL");
