// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) Google LLC
 *
 * MIPI-DSIM encoder for the Google Tensor "zuma"/"zumapro" DPU.
 *
 * The zuma DSIM is a newer IP generation than the mainline samsung-dsim
 * bridge (DSIM_LINK + integrated DCPHY, VERSION 0x02090100), so it needs its
 * own encoder. The bootloader brings up the DCPHY PLL, trains the lanes and
 * powers the panel (the splash streams over the link), so this driver leaves
 * the PHY/PLL alone - like the vendor's DSIM_STATE_HANDOVER path, which skips
 * dsim_reg_init when the PLL is stable.
 *
 * It does, however, run the command-mode link config + start (the vendor
 * dsim_reg_set_config()/dsim_reg_start() subset) on enable, because the
 * bootloader configures the DSIM for its own splash write path, not the
 * DECON-driven video-through-command DSC path - without this the DSIM emits
 * abnormal command transfers (DSIM_INTSRC.ABNORMAL_CMD_ST) that the panel
 * drops, leaving the bootloader image on screen.
 *
 * All panel-specific parameters are taken from the DRM mode (resolution,
 * refresh rate) and the attached MIPI-DSI device (lane count, HS bit rate),
 * so the encoder itself carries no panel constants.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "regs-dsim-zuma.h"

/* fallback refresh rate until the first mode_set (any sane value works) */
#define DSIM_DEFAULT_VREFRESH	60

struct zuma_dsim {
	struct device *dev;
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct clk *bus_clk;
	void __iomem *regs;		/* "dsi" link registers */
	void __iomem *phy_regs;		/* "dphy" DCPHY PLL/lane/timing */
	void __iomem *phy_extra_regs;	/* "dphy-extra" DCPHY bias */
	struct regmap *sysreg;		/* DPU SYSREG (DPHY reset control) */

	unsigned int lanes;
	unsigned int format;
	unsigned long mode_flags;
	unsigned int hs_clk_mbps;	/* per-lane HS bit rate (from the panel) */
	unsigned int hactive;		/* active width (from mode_set) */
	unsigned int vactive;		/* active height (from mode_set) */
	unsigned int vrefresh;		/* active refresh rate (from mode_set) */
	const struct drm_dsc_config *dsc;	/* DSC config (from the panel) */
	bool needs_cold_init;		/* link was torn down (suspend): re-init */
};

static inline struct zuma_dsim *host_to_dsim(struct mipi_dsi_host *h)
{
	return container_of(h, struct zuma_dsim, dsi_host);
}

static inline struct zuma_dsim *bridge_to_dsim(struct drm_bridge *b)
{
	return container_of(b, struct zuma_dsim, bridge);
}

static inline void dsim_rmw(struct zuma_dsim *dsim, u32 off, u32 val, u32 mask)
{
	u32 old = readl(dsim->regs + off);

	writel((val & mask) | (old & ~mask), dsim->regs + off);
}

/* ------------------------------------------------------------------ */
/* Cold DCPHY/PLL bring-up (vendor dsim_reg_init) - own the link with  */
/* no bootloader, and re-init on resume.                               */
/* ------------------------------------------------------------------ */

static inline void dphy_write(struct zuma_dsim *dsim, u32 off, u32 val)
{
	writel(val, dsim->phy_regs + off);
}

static inline void dphy_rmw(struct zuma_dsim *dsim, u32 off, u32 val, u32 mask)
{
	u32 old = readl(dsim->phy_regs + off);

	writel((val & mask) | (old & ~mask), dsim->phy_regs + off);
}

/*
 * zuma DCPHY PLL features (SoC constants): Fin 24.576 MHz, Fopt 13 MHz,
 * Fout 52 MHz..6.6 GHz, Fvco 3.3..6.6 GHz, P 1..63, M 64..1023, S 0..6,
 * K is a 16-bit two's-complement fraction. Mirrors dsim_calc_pmsk().
 */
#define DSIM_PLL_FIN		24576000u
#define DSIM_PLL_FOPTIMUM	13000000u
#define DSIM_PLL_K_BITS		16

struct dsim_pms { u32 p, m, s, k; };

static int zuma_dsim_calc_pms(u64 hs_clk, struct dsim_pms *pms)
{
	u64 fvco, q;
	u32 p, m, s, k;

	p = DIV_ROUND_CLOSEST(DSIM_PLL_FIN, DSIM_PLL_FOPTIMUM);
	if (!p)
		p = 1;
	if (hs_clk < 52000000ULL || hs_clk > 6600000000ULL)
		return -EINVAL;

	/* find s: fvco_min <= hs_clk * 2^s <= fvco_max */
	for (s = 0, fvco = 0; fvco < 3300000000ULL; s++)
		fvco = hs_clk << s;
	--s;
	if (fvco > 6600000000ULL)
		return -EINVAL;

	fvco >>= 1;
	q = fvco << (DSIM_PLL_K_BITS + 1);	/* 1 extra bit for round-up */
	q = div_u64(q, DSIM_PLL_FIN / p);
	m = q >> (DSIM_PLL_K_BITS + 1);
	if (m < 64 || m > 1023)
		return -EINVAL;

	k = q & ((1 << (DSIM_PLL_K_BITS + 1)) - 1);
	k = DIV_ROUND_UP(k, 2);
	if (k & (1 << (DSIM_PLL_K_BITS - 1)))	/* two's complement */
		m++;

	pms->p = p;
	pms->m = m;
	pms->s = s;
	pms->k = k;
	return 0;
}

/*
 * D-PHY HS timing per bit rate - a SoC characterisation table (verbatim from
 * the vendor). Columns: bps, clk_prepare, clk_zero, clk_post, clk_trail,
 * hs_prepare, hs_zero, hs_trail, lpx, hs_exit. Sorted descending by bps.
 */
static const u32 dphy_timing[][10] = {
	{2500, 11, 42, 11, 10, 11, 19, 10, 9, 16},
	{2490, 11, 42, 11, 10, 11, 18, 10, 9, 16},
	{2480, 11, 42, 11, 10, 11, 18, 10, 9, 16},
	{2470, 11, 41, 11, 10, 11, 18, 10, 9, 15},
	{2460, 11, 41, 11, 10, 11, 18, 10, 9, 15},
	{2450, 11, 41, 11, 10, 11, 18, 10, 9, 15},
	{2440, 11, 41, 11, 10, 11, 18, 10, 9, 15},
	{2430, 11, 41, 11, 10, 11, 18, 10, 9, 15},
	{2420, 11, 40, 11, 10, 11, 17, 10, 9, 15},
	{2410, 11, 40, 11, 10, 11, 17, 10, 9, 15},
	{2400, 11, 40, 10, 10, 11, 17, 10, 8, 15},
	{2390, 11, 40, 10, 10, 11, 17, 10, 8, 15},
	{2380, 11, 39, 10, 10, 11, 17, 10, 8, 15},
	{2370, 11, 39, 10, 10, 11, 17, 10, 8, 15},
	{2360, 11, 39, 10, 10, 11, 17, 10, 8, 15},
	{2350, 11, 39, 10, 9, 10, 18, 10, 8, 15},
	{2340, 11, 38, 10, 9, 10, 17, 10, 8, 15},
	{2330, 11, 38, 10, 9, 10, 17, 10, 8, 15},
	{2320, 11, 38, 10, 9, 10, 17, 10, 8, 14},
	{2310, 11, 38, 10, 9, 10, 17, 9, 8, 14},
	{2300, 10, 39, 10, 9, 10, 17, 9, 8, 14},
	{2290, 10, 38, 10, 9, 10, 17, 9, 8, 14},
	{2280, 10, 38, 10, 9, 10, 17, 9, 8, 14},
	{2270, 10, 38, 10, 9, 10, 16, 9, 8, 14},
	{2260, 10, 38, 10, 9, 10, 16, 9, 8, 14},
	{2250, 10, 37, 10, 9, 10, 16, 9, 8, 14},
	{2240, 10, 37, 10, 9, 10, 16, 9, 8, 14},
	{2230, 10, 37, 10, 9, 10, 16, 9, 8, 14},
	{2220, 10, 37, 10, 9, 10, 16, 9, 8, 14},
	{2210, 10, 36, 10, 9, 10, 16, 9, 8, 14},
	{2200, 10, 36, 9, 9, 10, 16, 9, 8, 14},
	{2190, 10, 36, 9, 9, 10, 15, 9, 8, 14},
	{2180, 10, 36, 9, 9, 10, 15, 9, 8, 13},
	{2170, 10, 35, 9, 9, 10, 15, 9, 8, 13},
	{2160, 10, 35, 9, 9, 10, 15, 9, 8, 13},
	{2150, 10, 35, 9, 9, 10, 15, 9, 8, 13},
	{2140, 10, 35, 9, 9, 9, 16, 9, 8, 13},
	{2130, 10, 35, 9, 9, 9, 16, 9, 7, 13},
	{2120, 10, 34, 9, 8, 9, 15, 9, 7, 13},
	{2110, 10, 34, 9, 8, 9, 15, 9, 7, 13},
	{2100, 9, 35, 9, 8, 9, 15, 9, 7, 13},
	{2090, 9, 35, 9, 8, 9, 15, 8, 7, 13},
	{2080, 9, 34, 9, 8, 9, 15, 8, 7, 13},
	{2070, 9, 34, 9, 8, 9, 15, 8, 7, 13},
	{2060, 9, 34, 9, 8, 9, 15, 8, 7, 13},
	{2050, 9, 34, 9, 8, 9, 15, 8, 7, 13},
	{2040, 9, 33, 9, 8, 9, 14, 8, 7, 13},
	{2030, 9, 33, 9, 8, 9, 14, 8, 7, 12},
	{2020, 9, 33, 9, 8, 9, 14, 8, 7, 12},
	{2010, 9, 33, 9, 8, 9, 14, 8, 7, 12},
	{2000, 9, 33, 8, 8, 9, 14, 8, 7, 12},
	{1990, 9, 32, 8, 8, 9, 14, 8, 7, 12},
	{1980, 9, 32, 8, 8, 9, 14, 8, 7, 12},
	{1970, 9, 32, 8, 8, 9, 13, 8, 7, 12},
	{1960, 9, 32, 8, 8, 9, 13, 8, 7, 12},
	{1950, 9, 31, 8, 8, 9, 13, 8, 7, 12},
	{1940, 9, 31, 8, 8, 9, 13, 8, 7, 12},
	{1930, 9, 31, 8, 8, 8, 14, 8, 7, 12},
	{1920, 9, 31, 8, 8, 8, 14, 8, 7, 12},
	{1910, 9, 30, 8, 8, 8, 14, 8, 7, 12},
	{1900, 8, 31, 8, 8, 8, 13, 8, 7, 12},
	{1890, 8, 31, 8, 7, 8, 13, 8, 7, 11},
	{1880, 8, 31, 8, 7, 8, 13, 8, 7, 11},
	{1870, 8, 31, 8, 7, 8, 13, 8, 7, 11},
	{1860, 8, 30, 8, 7, 8, 13, 7, 6, 11},
	{1850, 8, 30, 8, 7, 8, 13, 7, 6, 11},
	{1840, 8, 30, 8, 7, 8, 13, 7, 6, 11},
	{1830, 8, 30, 8, 7, 8, 13, 7, 6, 11},
	{1820, 8, 29, 8, 7, 8, 12, 7, 6, 11},
	{1810, 8, 29, 8, 7, 8, 12, 7, 6, 11},
	{1800, 8, 29, 7, 7, 8, 12, 7, 6, 11},
	{1790, 8, 29, 7, 7, 8, 12, 7, 6, 11},
	{1780, 8, 28, 7, 7, 8, 12, 7, 6, 11},
	{1770, 8, 28, 7, 7, 8, 12, 7, 6, 11},
	{1760, 8, 28, 7, 7, 8, 12, 7, 6, 11},
	{1750, 8, 28, 7, 7, 8, 11, 7, 6, 11},
	{1740, 8, 28, 7, 7, 8, 11, 7, 6, 10},
	{1730, 8, 27, 7, 7, 8, 11, 7, 6, 10},
	{1720, 8, 27, 7, 7, 7, 12, 7, 6, 10},
	{1710, 8, 27, 7, 7, 7, 12, 7, 6, 10},
	{1700, 7, 28, 7, 7, 7, 12, 7, 6, 10},
	{1690, 7, 27, 7, 7, 7, 12, 7, 6, 10},
	{1680, 7, 27, 7, 7, 7, 12, 7, 6, 10},
	{1670, 7, 27, 7, 6, 7, 11, 7, 6, 10},
	{1660, 7, 27, 7, 6, 7, 11, 7, 6, 10},
	{1650, 7, 26, 7, 6, 7, 11, 7, 6, 10},
	{1640, 7, 26, 7, 6, 7, 11, 7, 6, 10},
	{1630, 7, 26, 7, 6, 7, 11, 6, 6, 10},
	{1620, 7, 26, 7, 6, 7, 11, 6, 6, 10},
	{1610, 7, 25, 7, 6, 7, 11, 6, 6, 10},
	{1600, 7, 25, 6, 6, 7, 10, 6, 5, 9},
	{1590, 7, 25, 6, 6, 7, 10, 6, 5, 9},
	{1580, 7, 25, 6, 6, 7, 10, 6, 5, 9},
	{1570, 7, 25, 6, 6, 7, 10, 6, 5, 9},
	{1560, 7, 24, 6, 6, 7, 10, 6, 5, 9},
	{1550, 7, 24, 6, 6, 7, 10, 6, 5, 9},
	{1540, 7, 24, 6, 6, 7, 10, 6, 5, 9},
	{1530, 7, 24, 6, 6, 7, 9, 6, 5, 9},
	{1520, 7, 23, 6, 6, 7, 9, 6, 5, 9},
	{1510, 7, 23, 6, 6, 6, 10, 6, 5, 9},
	{1500, 6, 24, 6, 6, 6, 10, 6, 5, 9},
	{1490, 59, 23, 6, 70, 58, 10, 71, 44, 9},
	{1480, 58, 23, 6, 70, 58, 9, 71, 44, 9},
	{1470, 58, 23, 6, 69, 57, 9, 71, 44, 9},
	{1460, 57, 23, 6, 69, 57, 9, 70, 43, 9},
	{1450, 57, 23, 6, 69, 57, 9, 70, 43, 8},
	{1440, 57, 22, 6, 68, 56, 9, 70, 43, 8},
	{1430, 56, 22, 6, 68, 56, 9, 69, 42, 8},
	{1420, 56, 22, 6, 68, 55, 9, 69, 42, 8},
	{1410, 55, 22, 6, 67, 55, 9, 69, 42, 8},
	{1400, 55, 22, 5, 67, 55, 9, 68, 41, 8},
	{1390, 55, 21, 5, 67, 54, 9, 68, 41, 8},
	{1380, 54, 21, 5, 66, 54, 9, 68, 41, 8},
	{1370, 54, 21, 5, 66, 54, 8, 67, 41, 8},
	{1360, 53, 21, 5, 66, 53, 8, 67, 40, 8},
	{1350, 53, 21, 5, 65, 53, 8, 66, 40, 8},
	{1340, 53, 20, 5, 65, 52, 8, 66, 40, 8},
	{1330, 52, 20, 5, 65, 52, 8, 66, 39, 8},
	{1320, 52, 20, 5, 64, 52, 8, 65, 39, 8},
	{1310, 51, 20, 5, 64, 51, 8, 65, 39, 8},
	{1300, 51, 20, 5, 63, 51, 8, 65, 38, 7},
	{1290, 51, 20, 5, 63, 50, 8, 64, 38, 7},
	{1280, 50, 19, 5, 63, 50, 8, 64, 38, 7},
	{1270, 50, 19, 5, 62, 50, 8, 64, 38, 7},
	{1260, 49, 19, 5, 62, 49, 8, 63, 37, 7},
	{1250, 49, 19, 5, 62, 49, 7, 63, 37, 7},
	{1240, 49, 19, 5, 61, 49, 7, 63, 37, 7},
	{1230, 48, 19, 5, 61, 48, 7, 62, 36, 7},
	{1220, 48, 18, 5, 61, 48, 7, 62, 36, 7},
	{1210, 47, 18, 5, 60, 47, 7, 62, 36, 7},
	{1200, 47, 18, 4, 60, 47, 7, 61, 35, 7},
	{1190, 47, 18, 4, 60, 47, 7, 61, 35, 7},
	{1180, 46, 18, 4, 59, 46, 7, 60, 35, 7},
	{1170, 46, 17, 4, 59, 46, 7, 60, 35, 7},
	{1160, 45, 17, 4, 59, 46, 7, 60, 34, 6},
	{1150, 45, 17, 4, 58, 45, 7, 59, 34, 6},
	{1140, 45, 17, 4, 58, 45, 6, 59, 34, 6},
	{1130, 44, 17, 4, 57, 44, 6, 59, 33, 6},
	{1120, 44, 17, 4, 57, 44, 6, 58, 33, 6},
	{1110, 43, 16, 4, 57, 44, 6, 58, 33, 6},
	{1100, 43, 16, 4, 56, 43, 6, 58, 32, 6},
	{1090, 43, 16, 4, 56, 43, 6, 57, 32, 6},
	{1080, 42, 16, 4, 56, 43, 6, 57, 32, 6},
	{1070, 42, 16, 4, 55, 42, 6, 57, 32, 6},
	{1060, 41, 15, 4, 55, 42, 6, 56, 31, 6},
	{1050, 41, 15, 4, 55, 41, 6, 56, 31, 6},
	{1040, 41, 15, 4, 54, 41, 6, 56, 31, 6},
	{1030, 40, 15, 4, 54, 41, 5, 55, 30, 6},
	{1020, 40, 15, 4, 54, 40, 5, 55, 30, 6},
	{1010, 39, 15, 4, 53, 40, 5, 55, 30, 5},
	{1000, 39, 14, 3, 53, 39, 5, 54, 29, 5},
	{990, 39, 14, 3, 53, 39, 5, 54, 29, 5},
	{980, 38, 14, 3, 52, 39, 5, 53, 29, 5},
	{970, 38, 14, 3, 52, 38, 5, 53, 29, 5},
	{960, 37, 14, 3, 52, 38, 5, 53, 28, 5},
	{950, 37, 13, 3, 51, 38, 5, 52, 28, 5},
	{940, 37, 13, 3, 51, 37, 5, 52, 28, 5},
	{930, 36, 13, 3, 50, 37, 5, 52, 27, 5},
	{920, 36, 13, 3, 50, 36, 5, 51, 27, 5},
	{910, 35, 13, 3, 50, 36, 4, 51, 27, 5},
	{900, 35, 13, 3, 49, 36, 4, 51, 26, 5},
	{890, 35, 12, 3, 49, 35, 4, 50, 26, 5},
	{880, 34, 12, 3, 49, 35, 4, 50, 26, 5},
	{870, 34, 12, 3, 48, 35, 4, 50, 26, 4},
	{860, 33, 12, 3, 48, 34, 4, 49, 25, 4},
	{850, 33, 12, 3, 48, 34, 4, 49, 25, 4},
	{840, 33, 11, 3, 47, 33, 4, 49, 25, 4},
	{830, 32, 11, 3, 47, 33, 4, 48, 24, 4},
	{820, 32, 11, 3, 47, 33, 4, 48, 24, 4},
	{810, 31, 11, 3, 46, 32, 4, 47, 24, 4},
	{800, 31, 11, 2, 46, 32, 3, 47, 23, 4},
	{790, 31, 10, 2, 46, 31, 3, 47, 23, 4},
	{780, 30, 10, 2, 45, 31, 3, 46, 23, 4},
	{770, 30, 10, 2, 45, 31, 3, 46, 23, 4},
	{760, 29, 10, 2, 44, 30, 3, 46, 22, 4},
	{750, 29, 10, 2, 44, 30, 3, 45, 22, 4},
	{740, 29, 10, 2, 44, 30, 3, 45, 22, 4},
	{730, 28, 9, 2, 43, 29, 3, 45, 21, 4},
	{720, 28, 9, 2, 43, 29, 3, 44, 21, 3},
	{710, 27, 9, 2, 43, 28, 3, 44, 21, 3},
	{700, 27, 9, 2, 42, 28, 3, 44, 20, 3},
	{690, 27, 9, 2, 42, 28, 3, 43, 20, 3},
	{680, 26, 9, 2, 42, 27, 2, 43, 20, 3},
	{670, 26, 8, 2, 41, 27, 2, 43, 20, 3},
	{660, 25, 8, 2, 41, 27, 2, 42, 19, 3},
	{650, 25, 8, 2, 41, 26, 2, 42, 19, 3},
	{640, 25, 8, 2, 40, 26, 2, 42, 19, 3},
	{630, 24, 8, 2, 40, 25, 2, 41, 18, 3},
	{620, 24, 7, 2, 40, 25, 2, 41, 18, 3},
	{610, 23, 7, 2, 39, 25, 2, 40, 18, 3},
	{600, 23, 7, 1, 39, 24, 2, 40, 17, 3},
	{590, 23, 7, 1, 38, 24, 2, 40, 17, 3},
	{580, 22, 7, 1, 38, 24, 2, 39, 17, 2},
	{570, 22, 7, 1, 38, 23, 2, 39, 17, 2},
	{560, 21, 6, 1, 37, 23, 1, 39, 16, 2},
	{550, 21, 6, 1, 37, 22, 1, 38, 16, 2},
	{540, 21, 6, 1, 37, 22, 1, 38, 16, 2},
	{530, 20, 6, 1, 36, 22, 1, 38, 15, 2},
	{520, 20, 6, 1, 36, 21, 1, 37, 15, 2},
	{510, 19, 5, 1, 36, 21, 1, 37, 15, 2},
	{500, 19, 5, 1, 35, 20, 1, 37, 14, 2},
	{490, 19, 5, 1, 35, 20, 1, 36, 14, 2},
	{480, 18, 5, 1, 35, 20, 1, 36, 14, 2},
	{470, 18, 5, 1, 34, 19, 1, 36, 14, 2},
	{460, 17, 5, 1, 34, 19, 1, 35, 13, 2},
	{450, 17, 4, 1, 34, 19, 0, 35, 13, 2},
	{440, 17, 4, 1, 33, 18, 0, 34, 13, 2},
	{430, 16, 4, 1, 33, 18, 0, 34, 12, 1},
	{420, 16, 4, 1, 33, 17, 0, 34, 12, 1},
	{410, 15, 4, 1, 32, 17, 0, 33, 12, 1},
	{400, 15, 3, 0, 32, 17, 0, 33, 11, 1},
	{390, 15, 3, 0, 31, 16, 0, 33, 11, 1},
	{380, 14, 3, 0, 31, 16, 0, 32, 11, 1},
	{370, 14, 3, 0, 31, 16, 0, 32, 11, 1},
	{360, 13, 3, 0, 30, 15, 0, 32, 10, 1},
	{350, 13, 3, 0, 30, 15, 0, 31, 10, 1},
	{340, 13, 2, 0, 30, 14, 0, 31, 10, 1},
	{330, 12, 2, 0, 29, 14, 0, 31, 9, 1},
	{320, 12, 2, 0, 29, 14, 0, 30, 9, 1},
	{310, 11, 2, 0, 29, 13, 0, 30, 9, 1},
	{300, 11, 2, 0, 28, 13, 0, 30, 8, 1},
	{290, 11, 1, 0, 28, 13, 0, 29, 8, 0},
	{280, 10, 1, 0, 28, 12, 0, 29, 8, 0},
	{270, 10, 1, 0, 27, 12, 0, 28, 8, 0},
	{260, 9, 1, 0, 27, 11, 0, 28, 7, 0},
	{250, 9, 1, 0, 27, 11, 0, 28, 7, 0},
	{240, 9, 0, 0, 26, 11, 0, 27, 7, 0},
	{230, 8, 0, 0, 26, 10, 0, 27, 6, 0},
	{220, 8, 0, 0, 25, 10, 0, 27, 6, 0},
	{210, 7, 0, 0, 25, 9, 0, 26, 6, 0},
	{200, 7, 0, 0, 25, 9, 0, 26, 5, 0},
	{190, 7, 0, 0, 24, 9, 0, 26, 5, 0},
	{180, 6, 0, 0, 24, 8, 0, 25, 5, 0},
	{170, 6, 0, 0, 24, 8, 0, 25, 5, 0},
	{160, 5, 0, 0, 23, 8, 0, 25, 4, 0},
	{150, 5, 0, 0, 23, 7, 0, 24, 4, 0},
	{140, 5, 0, 0, 23, 7, 0, 24, 4, 0},
	{130, 4, 0, 0, 22, 6, 0, 24, 3, 0},
	{120, 4, 0, 0, 22, 6, 0, 23, 3, 0},
	{110, 3, 0, 0, 22, 6, 0, 23, 3, 0},
	{100, 3, 0, 0, 21, 5, 0, 23, 2, 0},
	{90, 3, 0, 0, 21, 5, 0, 22, 2, 0},
	{80, 2, 0, 0, 21, 5, 0, 22, 2, 0},
};

/* HS D-PHY control byte per escape clock (index = esc_clk_mhz - 7) */
static const u32 b_dphyctl[14] = {
	0x0af, 0x0c8, 0x0e1, 0x0fa,		/* esc 7..10 */
	0x113, 0x12c, 0x145, 0x15e, 0x177,	/* esc 11..15 */
	0x190, 0x1a9, 0x1c2, 0x1db, 0x1f4,	/* esc 16..20 */
};

/* pick the smallest tabulated bps that is >= the target (table is descending) */
static const u32 *zuma_dsim_dphy_timing(u32 hs_mbps)
{
	int i;

	for (i = ARRAY_SIZE(dphy_timing) - 1; i >= 0; i--)
		if (dphy_timing[i][0] >= hs_mbps)
			return dphy_timing[i];
	return dphy_timing[0];
}

static void zuma_dsim_sysreg_dphy_reset(struct zuma_dsim *dsim, bool assert)
{
	/* rst-n: 0 asserts reset, 1 releases it */
	regmap_update_bits(dsim->sysreg, DISP_DPU_MIPI_PHY_CON,
			   DISP_DPU_M_RESETN_M0,
			   assert ? 0 : DISP_DPU_M_RESETN_M0);
}

/* Program the PLL, D-PHY timing and enable/lock the PLL (vendor set_clocks) */
static int zuma_dsim_dphy_set_clocks(struct zuma_dsim *dsim)
{
	static const u32 bias_con[6] = {
		0x10, 0x110, 0x3223, 0, 0x200, 0x2 };
	static const u32 pll_con[10] = {
		0, 0, 0, 0, 0, 0x500, 0x8e, 0x3d40, 0x1e00, 0x1300 };
	static const u32 md_ana[4] = { 0x7122, 0, 0, 0 };
	u32 hs_mbps = dsim->hs_clk_mbps;
	u32 word_clk = hs_mbps / 16;
	u32 esc_div = DIV_ROUND_UP(word_clk, 20);
	u32 esc_clk = word_clk / esc_div;
	const u32 *t = zuma_dsim_dphy_timing(hs_mbps);
	u32 bctl = b_dphyctl[clamp(esc_clk, 7u, 20u) - 7];
	struct dsim_pms pms;
	u32 val;
	int i, ret;

	ret = zuma_dsim_calc_pms((u64)hs_mbps * 1000000, &pms);
	if (ret) {
		dev_err(dsim->dev, "no PLL PMS for %u Mbps\n", hs_mbps);
		return ret;
	}

	for (i = 0; i < 6; i++)
		writel(bias_con[i], dsim->phy_extra_regs + DSIM_PHY_BIAS_CON(i));
	for (i = 0; i < 10; i++)
		dphy_write(dsim, DSIM_PHY_PLL_CON(i), pll_con[i]);

	if ((hs_mbps << pms.s) < 3000)
		dphy_rmw(dsim, DSIM_PHY_PLL_CON5,
			 DSIM_PHY_PLL_CON5_DITHER_SEL_VCO,
			 DSIM_PHY_PLL_CON5_DITHER_SEL_VCO);

	dsim_rmw(dsim, DSIM_CLK_CTRL,
		 DSIM_CLK_CTRL_ESCCLK_EN | DSIM_CLK_CTRL_ESC_PRESCALER(esc_div),
		 DSIM_CLK_CTRL_ESCCLK_EN | DSIM_CLK_CTRL_ESC_PRESCALER_MASK);

	/* clock-lane HS timing */
	dphy_write(dsim, DSIM_PHY_MC_TIME_CON4, DSIM_PHY_TIME_CON4_ULPS_EXIT(bctl));
	dphy_write(dsim, DSIM_PHY_MC_TIME_CON0,
		   DSIM_PHY_TIME_CON0_HSTX_CLK_SEL |
		   DSIM_PHY_TIME_CON0_TLPX(t[8]));
	dphy_write(dsim, DSIM_PHY_MC_DESKEW_CON0, 0);
	dphy_write(dsim, DSIM_PHY_MC_TIME_CON1,
		   DSIM_PHY_MC_TIME_CON1_TCLK_PREPARE(t[1]) |
		   DSIM_PHY_MC_TIME_CON1_TCLK_ZERO(t[2]));
	dphy_write(dsim, DSIM_PHY_MC_TIME_CON2,
		   DSIM_PHY_MC_TIME_CON2_THS_EXIT(t[9]) |
		   DSIM_PHY_MC_TIME_CON2_TCLK_TRAIL(t[4]));
	dphy_write(dsim, DSIM_PHY_MC_TIME_CON3,
		   DSIM_PHY_MC_TIME_CON3_TCLK_POST(t[3]));

	/* data-lane HS timing */
	for (i = 0; i < 4; i++) {
		dphy_write(dsim, DSIM_PHY_MD_TIME_CON4(i),
			   DSIM_PHY_TIME_CON4_ULPS_EXIT(bctl));
		dphy_write(dsim, DSIM_PHY_MD_TIME_CON0(i),
			   DSIM_PHY_TIME_CON0_HSTX_CLK_SEL |
			   DSIM_PHY_TIME_CON0_TLPX(t[8]));
		dphy_write(dsim, DSIM_PHY_MD_TIME_CON1(i),
			   DSIM_PHY_MD_TIME_CON1_THS_PREPARE(t[5]) |
			   DSIM_PHY_MD_TIME_CON1_THS_ZERO(t[6]));
		dphy_write(dsim, DSIM_PHY_MD_TIME_CON2(i),
			   DSIM_PHY_MD_TIME_CON2_THS_EXIT(t[9]) |
			   DSIM_PHY_MD_TIME_CON2_THS_TRAIL(t[7]));
		dphy_write(dsim, DSIM_PHY_MD_TIME_CON3(i),
			   DSIM_PHY_MD_TIME_CON3_TTA_GET(3));
	}

	/* clock/data-lane general + analog seeds */
	dphy_write(dsim, DSIM_PHY_MC_GNR_CON0, 0);
	dphy_write(dsim, DSIM_PHY_MC_GNR_CON1, 0x1334);
	dphy_write(dsim, DSIM_PHY_MC_ANA_CON0, 0x7122);
	dphy_write(dsim, DSIM_PHY_MC_ANA_CON1, 0);
	dphy_write(dsim, DSIM_PHY_MC_ANA_CON2, 0x2);
	dphy_write(dsim, DSIM_PHY_MC_ANA_CON3, 0);
	for (i = 0; i < 4; i++) {
		dphy_write(dsim, DSIM_PHY_MD_GNR_CON0(i), 0);
		dphy_write(dsim, DSIM_PHY_MD_GNR_CON1(i), 0x1334);
		dphy_write(dsim, DSIM_PHY_MD_ANA_CON0(i), md_ana[0]);
		dphy_write(dsim, DSIM_PHY_MD_ANA_CON1(i), md_ana[1]);
		dphy_write(dsim, DSIM_PHY_MD_ANA_CON2(i), md_ana[2]);
		dphy_write(dsim, DSIM_PHY_MD_ANA_CON3(i), md_ana[3]);
	}

	/* PLL P/M/S/K */
	dphy_write(dsim, DSIM_PHY_PLL_CON1, DSIM_PHY_PLL_CON1_PMS_K(pms.k));
	dphy_rmw(dsim, DSIM_PHY_PLL_CON0,
		 DSIM_PHY_PLL_CON0_PMS_P(pms.p) | DSIM_PHY_PLL_CON0_PMS_S(pms.s),
		 DSIM_PHY_PLL_CON0_PMS_P_MASK | DSIM_PHY_PLL_CON0_PMS_S_MASK);
	dphy_rmw(dsim, DSIM_PHY_PLL_CON2, DSIM_PHY_PLL_CON2_PMS_M(pms.m),
		 DSIM_PHY_PLL_CON2_PMS_M_MASK);
	dphy_rmw(dsim, DSIM_PHY_PLL_CON6, DSIM_PHY_PLL_CON6_WCLK_BUF_SFT_CNT(3),
		 DSIM_PHY_PLL_CON6_WCLK_BUF_SFT_CNT_MASK);
	dphy_write(dsim, DSIM_PHY_PLL_CON7, DSIM_PHY_PLL_CON7_PLL_LOCK_CNT(500 * pms.p));

	/* enable + wait for lock */
	writel(DSIM_INTSRC_PLL_STABLE, dsim->regs + DSIM_INTSRC);
	dphy_rmw(dsim, DSIM_PHY_PLL_CON0, DSIM_PHY_PLL_CON0_PLL_EN,
		 DSIM_PHY_PLL_CON0_PLL_EN);
	ret = readl_poll_timeout(dsim->phy_regs + DSIM_PHY_PLL_STAT0, val,
				 val & DSIM_PHY_PLL_STAT0_PLL_LOCK, 10, 2000);
	if (ret)
		dev_err(dsim->dev, "DCPHY PLL failed to lock\n");
	return ret;
}

/* enable the D-PHY lanes and wait for them ready (vendor set_lanes_dphy) */
static int zuma_dsim_dphy_enable_lanes(struct zuma_dsim *dsim)
{
	u32 val;
	int i, ret;

	dphy_rmw(dsim, DSIM_PHY_MC_GNR_CON0, DSIM_PHY_GNR_PHY_ENABLE,
		 DSIM_PHY_GNR_PHY_ENABLE);
	for (i = 0; i < 4; i++)
		dphy_rmw(dsim, DSIM_PHY_MD_GNR_CON0(i), DSIM_PHY_GNR_PHY_ENABLE,
			 DSIM_PHY_GNR_PHY_ENABLE);

	ret = readl_poll_timeout(dsim->phy_regs + DSIM_PHY_MC_GNR_CON0, val,
				 val & DSIM_PHY_GNR_PHY_READY, 10, 2000);
	for (i = 0; !ret && i < 4; i++)
		ret = readl_poll_timeout(dsim->phy_regs + DSIM_PHY_MD_GNR_CON0(i),
					 val, val & DSIM_PHY_GNR_PHY_READY, 10, 2000);
	if (ret)
		dev_err(dsim->dev, "DCPHY lanes not ready\n");
	return ret;
}

/* Program the command-mode + DSC link config (vendor set_config subset) */
static void zuma_dsim_set_config(struct zuma_dsim *dsim)
{
	const struct drm_dsc_config *dsc = dsim->dsc;
	u32 slice_cnt = dsc ? dsc->slice_count : 1;
	u32 slice_px = dsc ? DIV_ROUND_UP(dsc->slice_width *
					  dsc->bits_per_component, 8) : 0;
	u32 comp_w = dsc ? DIV_ROUND_UP(slice_px, 6) * 2 : 0;
	u32 width = dsc ? comp_w * slice_cnt : dsim->hactive;

	dsim_rmw(dsim, DSIM_SFR_CTRL, DSIM_SFR_CTRL_SHADOW_REG_READ_EN,
		 DSIM_SFR_CTRL_SHADOW_REG_READ_EN);
	dsim_rmw(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_NONCONT_CLOCK_LANE,
		 DSIM_CLK_CTRL_NONCONT_CLOCK_LANE);
	dsim_rmw(dsim, DSIM_DESKEW_CTRL, 0, DSIM_DESKEW_CTRL_HW_EN);
	writel(DSIM_TIMEOUT_BTA_TOUT(DSIM_BTA_TIMEOUT) |
	       DSIM_TIMEOUT_LPRX_TOUT(DSIM_LP_RX_TIMEOUT),
	       dsim->regs + DSIM_TIMEOUT);
	dsim_rmw(dsim, DSIM_ESCMODE,
		 DSIM_ESCMODE_STOP_STATE_CNT(DSIM_STOP_STATE_CNT),
		 DSIM_ESCMODE_STOP_STATE_CNT_MASK | DSIM_ESCMODE_CMD_LPDT);
	dsim_rmw(dsim, DSIM_OPTION_SUITE, DSIM_OPTION_SUITE_OPT_TE_ON_CMD_ALLOW,
		 DSIM_OPTION_SUITE_OPT_TE_ON_CMD_ALLOW);

	writel(DSIM_THRESHOLD_LEVEL(width), dsim->regs + DSIM_THRESHOLD);
	writel(DSIM_RESOL_VRESOL(dsim->vactive) | DSIM_RESOL_HRESOL(width),
	       dsim->regs + DSIM_RESOL);
	/* command + DSC: one transfer per line */
	writel(DSIM_NUM_OF_TRANSFER_PER_FRAME(dsim->vactive),
	       dsim->regs + DSIM_NUM_OF_TRANSFER);

	dsim_rmw(dsim, DSIM_CONFIG,
		 DSIM_CONFIG_NUM_OF_DATA_LANE(dsim->lanes - 1) |
		 DSIM_CONFIG_EOTP_EN(1) |
		 DSIM_CONFIG_PIXEL_FORMAT(DSIM_PIXEL_FORMAT_RGB24) |
		 DSIM_CONFIG_VC_ID(0),
		 DSIM_CONFIG_NUM_OF_DATA_LANE_MASK | DSIM_CONFIG_EOTP_EN_MASK |
		 DSIM_CONFIG_PER_FRAME_READ_EN_MASK |
		 DSIM_CONFIG_PIXEL_FORMAT_MASK | DSIM_CONFIG_VC_ID_MASK);
	dsim_rmw(dsim, DSIM_SFR_CTRL, DSIM_SFR_CTRL_SHADOW_EN,
		 DSIM_SFR_CTRL_SHADOW_EN);
	dsim_rmw(dsim, DSIM_CONFIG, 0, DSIM_CONFIG_VIDEO_MODE);
	dsim_rmw(dsim, DSIM_CONFIG, dsc ? DSIM_CONFIG_CPRS_EN : 0,
		 DSIM_CONFIG_CPRS_EN);

	if (dsc) {
		writel(DSIM_CPRS_CTRL_MULI_SLICE_PACKET(1) |
		       DSIM_CPRS_CTRL_NUM_OF_SLICE(slice_cnt),
		       dsim->regs + DSIM_CPRS_CTRL);
		writel(DSIM_SLICE01_SIZE_OF_SLICE0(dsc->slice_width) |
		       DSIM_SLICE01_SIZE_OF_SLICE1(slice_cnt > 1 ? dsc->slice_width : 0),
		       dsim->regs + DSIM_SLICE01);
	}
	dsim_rmw(dsim, DSIM_CMD_CONFIG, 0, DSIM_CMD_CONFIG_PKT_GO_EN);
}

/* Full cold link init (vendor dsim_reg_init): own the DCPHY from scratch. */
static int zuma_dsim_cold_init(struct zuma_dsim *dsim)
{
	u32 lanes = DSIM_LANE_CLOCK | GENMASK(dsim->lanes, 1);
	u32 val;
	int ret;

	/* DPHY reset controlled from SYSREG, not the DSIM link */
	regmap_update_bits(dsim->sysreg, DISP_DPU_MIPI_PHY_CON,
			   DISP_DPU_SEL_RESET_DPHY(0), 0);

	dsim_rmw(dsim, DSIM_CLK_CTRL, 0, DSIM_CLK_CTRL_CLOCK_SEL);	/* OSC */
	writel(DSIM_SWRST_RESET, dsim->regs + DSIM_SWRST);
	ret = readl_poll_timeout(dsim->regs + DSIM_SWRST, val,
				 !(val & DSIM_SWRST_RESET), 10, 2000);
	if (ret)
		return ret;

	dsim_rmw(dsim, DSIM_CONFIG, DSIM_CONFIG_LANES_EN(lanes),
		 DSIM_CONFIG_LANES_EN(0x1f));
	dsim_rmw(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_LANE_ESCCLK_EN(lanes),
		 DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK);
	dsim_rmw(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_WORDCLK_EN,
		 DSIM_CLK_CTRL_WORDCLK_EN);

	zuma_dsim_sysreg_dphy_reset(dsim, true);	/* assert */
	ret = zuma_dsim_dphy_set_clocks(dsim);
	if (ret)
		return ret;
	ret = zuma_dsim_dphy_enable_lanes(dsim);
	if (ret)
		return ret;
	zuma_dsim_sysreg_dphy_reset(dsim, false);	/* release */

	dsim_rmw(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_CLOCK_SEL,
		 DSIM_CLK_CTRL_CLOCK_SEL);		/* word clock */

	zuma_dsim_set_config(dsim);
	return 0;
}

static bool zuma_dsim_pll_is_stable(struct zuma_dsim *dsim)
{
	return readl(dsim->phy_regs + DSIM_PHY_PLL_STAT0) &
	       DSIM_PHY_PLL_STAT0_PLL_LOCK;
}

static void zuma_dsim_configure(struct zuma_dsim *dsim)
{
	u32 stable_vfp, te_protect, te_tout;
	u32 vrefresh = dsim->vrefresh ? dsim->vrefresh : DSIM_DEFAULT_VREFRESH;
	u32 hs_mbps = dsim->hs_clk_mbps;

	if (!dsim->regs || !hs_mbps || !dsim->hactive)
		return;

	/*
	 * Take the handover path only on the initial boot, where the bootloader
	 * left the DCPHY PLL running: adopt the live link and just re-arm the
	 * command-mode transfer below. After a suspend/disable (needs_cold_init),
	 * or a cold boot with no splash (PLL down), bring the whole
	 * DCPHY/PLL/lanes + link config up from scratch - matching the panel,
	 * which does a full cold power-on in the same cases.
	 */
	if (dsim->needs_cold_init || !zuma_dsim_pll_is_stable(dsim))
		zuma_dsim_cold_init(dsim);
	dsim->needs_cold_init = false;

	/*
	 * Command-mode transfer TE timing (vendor dsim_reg_set_config subset).
	 * The bootloader sized this for its one-shot splash write; a continuous
	 * DECON-driven write_memory needs the transfer gated to the panel's
	 * TE-safe window or the panel drops each frame. Allow the transfer on TE
	 * and (re)program the stable-VFP / TE protect+timeout. SFR-only, does not
	 * touch the D-PHY, so the live HS link survives.
	 */
	dsim_rmw(dsim, DSIM_OPTION_SUITE, DSIM_OPTION_SUITE_OPT_TE_ON_CMD_ALLOW,
		 DSIM_OPTION_SUITE_OPT_TE_ON_CMD_ALLOW);
	stable_vfp = dsim->hactive * DSIM_STABLE_VFP_VALUE / 100;
	/* TE protect/timeout windows scale with the frame period (vrefresh) */
	te_protect = hs_mbps * (100 - DSIM_TE_MARGIN) * 100 / vrefresh / 16;
	te_tout = hs_mbps * (100 + DSIM_TE_MARGIN * 2) * 100 / vrefresh / 16;
	writel(DSIM_CMD_TE_CTRL0_TIME_STABLE_VFP(stable_vfp),
	       dsim->regs + DSIM_CMD_TE_CTRL0);
	writel(DSIM_CMD_TE_CTRL1_TIME_TE_PROTECT_ON(te_protect) |
	       DSIM_CMD_TE_CTRL1_TIME_TE_TOUT(te_tout),
	       dsim->regs + DSIM_CMD_TE_CTRL1);

	/*
	 * Vendor handover path (DSIM_STATE_HANDOVER): skip dsim_reg_init() but
	 * still run dsim_reg_start() - request the HS clock, unmask the transfer
	 * interrupts and clear any stale pending ones.
	 */
	dsim_rmw(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_TX_REQUEST_HSCLK,
		 DSIM_CLK_CTRL_TX_REQUEST_HSCLK);
	dsim_rmw(dsim, DSIM_INTMSK, 0,
		 DSIM_INTMSK_SW_RST_RELEASE | DSIM_INTMSK_SFR_PL_FIFO_EMPTY |
			 DSIM_INTMSK_SFR_PH_FIFO_EMPTY | DSIM_INTMSK_FRAME_DONE |
			 DSIM_INTMSK_INVALID_SFR_VALUE | DSIM_INTMSK_UNDER_RUN |
			 DSIM_INTMSK_RX_DATA_DONE | DSIM_INTMSK_ERR_RX_ECC);
	writel(0xffffffff, dsim->regs + DSIM_INTSRC);
}

/* ------------------------------------------------------------------ */
/* drm_bridge								      */
/* ------------------------------------------------------------------ */

static int zuma_dsim_bridge_attach(struct drm_bridge *bridge,
				   struct drm_encoder *encoder,
				   enum drm_bridge_attach_flags flags)
{
	struct zuma_dsim *dsim = bridge_to_dsim(bridge);

	return drm_bridge_attach(encoder, dsim->panel_bridge, bridge, flags);
}

static void zuma_dsim_bridge_pre_enable(struct drm_bridge *bridge,
					struct drm_atomic_commit *state)
{
	zuma_dsim_configure(bridge_to_dsim(bridge));
}

static void zuma_dsim_bridge_post_disable(struct drm_bridge *bridge,
					  struct drm_atomic_commit *state)
{
	/*
	 * The link is being torn down (e.g. system suspend). Force the next
	 * enable to do a full cold DCPHY bring-up rather than adopting whatever
	 * stale PLL state survives, keeping the DSIM in step with the panel's
	 * cold power-on.
	 */
	bridge_to_dsim(bridge)->needs_cold_init = true;
}

static void zuma_dsim_bridge_mode_set(struct drm_bridge *bridge,
				      const struct drm_display_mode *mode,
				      const struct drm_display_mode *adjusted)
{
	struct zuma_dsim *dsim = bridge_to_dsim(bridge);

	dsim->hactive = adjusted->hdisplay;
	dsim->vactive = adjusted->vdisplay;
	dsim->vrefresh = drm_mode_vrefresh(adjusted);
}

static const struct drm_bridge_funcs zuma_dsim_bridge_funcs = {
	.attach = zuma_dsim_bridge_attach,
	.mode_set = zuma_dsim_bridge_mode_set,
	.atomic_pre_enable = zuma_dsim_bridge_pre_enable,
	.atomic_post_disable = zuma_dsim_bridge_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

/* ------------------------------------------------------------------ */
/* mipi_dsi_host							      */
/* ------------------------------------------------------------------ */

static int zuma_dsim_host_attach(struct mipi_dsi_host *host,
				 struct mipi_dsi_device *device)
{
	struct zuma_dsim *dsim = host_to_dsim(host);
	struct drm_bridge *bridge;
	struct exynos_drm_crtc *crtc;

	dsim->lanes = device->lanes;
	dsim->format = device->format;
	dsim->mode_flags = device->mode_flags;
	dsim->hs_clk_mbps = device->hs_rate / 1000000;
	dsim->dsc = device->dsc;

	bridge = devm_drm_of_get_bridge(dsim->dev, dsim->dev->of_node, 1, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dsim->dev, PTR_ERR(bridge),
				     "failed to find panel bridge\n");
	dsim->panel_bridge = bridge;

	drm_bridge_attach(&dsim->encoder, &dsim->bridge, NULL, 0);

	crtc = exynos_drm_crtc_get_by_type(dsim->encoder.dev,
					   EXYNOS_DISPLAY_TYPE_LCD);
	if (!IS_ERR(crtc))
		crtc->i80_mode = !(dsim->mode_flags & MIPI_DSI_MODE_VIDEO);

	return 0;
}

static int zuma_dsim_host_detach(struct mipi_dsi_host *host,
				 struct mipi_dsi_device *device)
{
	return 0;
}

/*
 * Send one command packet to the panel, ported from the vendor
 * __dsim_cmd_write_locked()/dsim_write_payload()/dsim_reg_wr_tx_header(): fill
 * the payload FIFO 4 bytes at a time, then write the packet header (which kicks
 * the transfer), then wait for the PH/PL FIFOs to drain. The vendor waits on an
 * IRQ completion; we poll the FIFO-empty status, which needs no DSIM IRQ.
 */
static ssize_t zuma_dsim_host_transfer(struct mipi_dsi_host *host,
				       const struct mipi_dsi_msg *msg)
{
	struct zuma_dsim *dsim = host_to_dsim(host);
	struct mipi_dsi_packet packet;
	const u8 *pl;
	size_t i;
	u32 val, mask;
	int ret;

	if (!dsim->regs)
		return -ENODEV;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret < 0)
		return ret;

	/* long-packet payload first, packed little-endian 4 bytes per word */
	pl = packet.payload;
	for (i = 0; i < packet.payload_length; i += 4) {
		size_t n = min_t(size_t, 4, packet.payload_length - i);

		val = 0;
		switch (n) {
		case 4:
			val |= (u32)pl[i + 3] << 24;
			fallthrough;
		case 3:
			val |= (u32)pl[i + 2] << 16;
			fallthrough;
		case 2:
			val |= (u32)pl[i + 1] << 8;
			fallthrough;
		default:
			val |= pl[i];
			break;
		}
		writel(val, dsim->regs + DSIM_PAYLOAD);
	}

	/* header kicks the transfer */
	writel(DSIM_PKTHDR_ID(packet.header[0]) |
		       DSIM_PKTHDR_DATA0(packet.header[1]) |
		       DSIM_PKTHDR_DATA1(packet.header[2]),
	       dsim->regs + DSIM_PKTHDR);

	/* wait for the header (and payload) FIFO to drain */
	mask = DSIM_FIFOCTRL_EMPTY_PH_SFR;
	if (packet.payload_length)
		mask |= DSIM_FIFOCTRL_EMPTY_PL_SFR;
	ret = readl_poll_timeout_atomic(dsim->regs + DSIM_FIFOCTRL, val,
					(val & mask) == mask, 10, 20000);
	if (ret) {
		dev_warn(dsim->dev, "cmd tx timeout (type 0x%02x, FIFOCTRL=0x%08x)\n",
			 msg->type, val);
		return ret;
	}

	return msg->tx_len;
}

static const struct mipi_dsi_host_ops zuma_dsim_host_ops = {
	.attach = zuma_dsim_host_attach,
	.detach = zuma_dsim_host_detach,
	.transfer = zuma_dsim_host_transfer,
};

/* ------------------------------------------------------------------ */
/* component / platform						      */
/* ------------------------------------------------------------------ */

static int zuma_dsim_bind(struct device *dev, struct device *master, void *data)
{
	struct zuma_dsim *dsim = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	drm_simple_encoder_init(drm_dev, &dsim->encoder, DRM_MODE_ENCODER_DSI);

	ret = exynos_drm_set_possible_crtcs(&dsim->encoder,
					    EXYNOS_DISPLAY_TYPE_LCD);
	if (ret < 0)
		return ret;

	return mipi_dsi_host_register(&dsim->dsi_host);
}

static void zuma_dsim_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct zuma_dsim *dsim = dev_get_drvdata(dev);

	mipi_dsi_host_unregister(&dsim->dsi_host);
}

static const struct component_ops zuma_dsim_component_ops = {
	.bind = zuma_dsim_bind,
	.unbind = zuma_dsim_unbind,
};

static int zuma_dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zuma_dsim *dsim;
	int ret;

	dsim = devm_drm_bridge_alloc(dev, struct zuma_dsim, bridge,
				     &zuma_dsim_bridge_funcs);
	if (IS_ERR(dsim))
		return PTR_ERR(dsim);

	dsim->dev = dev;

	dsim->regs = devm_platform_ioremap_resource_byname(pdev, "dsi");
	if (IS_ERR(dsim->regs))
		return PTR_ERR(dsim->regs);

	/*
	 * The integrated DCPHY (PLL/lane/timing + bias) and the DPU SYSREG DPHY
	 * reset control are needed for a cold link bring-up. They are unused on
	 * the boot-handover path (bootloader owns the DCPHY), so absence is not
	 * fatal here.
	 */
	dsim->phy_regs = devm_platform_ioremap_resource_byname(pdev, "dphy");
	if (IS_ERR(dsim->phy_regs))
		return PTR_ERR(dsim->phy_regs);

	dsim->phy_extra_regs =
		devm_platform_ioremap_resource_byname(pdev, "dphy-extra");
	if (IS_ERR(dsim->phy_extra_regs))
		return PTR_ERR(dsim->phy_extra_regs);

	dsim->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "samsung,disp-sysreg");
	if (IS_ERR(dsim->sysreg))
		return dev_err_probe(dev, PTR_ERR(dsim->sysreg),
				     "failed to get disp-sysreg\n");

	/* keep the DSIM block clocked+powered; the bootloader link stays up */
	dsim->bus_clk = devm_clk_get_optional_enabled(dev, "bus_clk");
	if (IS_ERR(dsim->bus_clk))
		return dev_err_probe(dev, PTR_ERR(dsim->bus_clk),
				     "failed to get bus_clk\n");

	dsim->dsi_host.ops = &zuma_dsim_host_ops;
	dsim->dsi_host.dev = dev;

	ret = devm_drm_bridge_add(dev, &dsim->bridge);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dsim);

	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		return ret;
	}

	ret = component_add(dev, &zuma_dsim_component_ops);
	if (ret) {
		pm_runtime_put(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	return 0;
}

static void zuma_dsim_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	component_del(dev, &zuma_dsim_component_ops);
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
}

static const struct of_device_id zuma_dsim_of_match[] = {
	{ .compatible = "google,zuma-dsim" },
	{ }
};
MODULE_DEVICE_TABLE(of, zuma_dsim_of_match);

struct platform_driver zuma_dsim_driver = {
	.probe = zuma_dsim_probe,
	.remove = zuma_dsim_remove,
	.driver = {
		.name = "zuma-dsim",
		.of_match_table = zuma_dsim_of_match,
	},
};
