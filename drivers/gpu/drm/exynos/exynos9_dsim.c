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
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

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
	unsigned int vrefresh;		/* active refresh rate (from mode_set) */
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

/*
 * Bootloader handover: the bootloader owns the DCPHY/PLL and lane training, so
 * this only touches the SFR side of the command-mode transfer and never the
 * D-PHY - the live HS link survives.
 *
 * A full (re)program of the link (the vendor dsim_reg_set_config() subset)
 * would need the complete dsim_reg_init() DCPHY bring-up, which the handover
 * deliberately avoids; doing the SFR half alone desyncs the running link
 * (parks the D-PHY in LP-11 and raises ABNORMAL_CMD_ST). So program only the
 * TE transfer window and re-arm the transfer.
 */
static void zuma_dsim_configure(struct zuma_dsim *dsim)
{
	u32 stable_vfp, te_protect, te_tout;
	u32 vrefresh = dsim->vrefresh ? dsim->vrefresh : DSIM_DEFAULT_VREFRESH;
	u32 hs_mbps = dsim->hs_clk_mbps;

	if (!dsim->regs || !hs_mbps || !dsim->hactive)
		return;

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

static void zuma_dsim_bridge_mode_set(struct drm_bridge *bridge,
				      const struct drm_display_mode *mode,
				      const struct drm_display_mode *adjusted)
{
	struct zuma_dsim *dsim = bridge_to_dsim(bridge);

	dsim->hactive = adjusted->hdisplay;
	dsim->vrefresh = drm_mode_vrefresh(adjusted);
}

static const struct drm_bridge_funcs zuma_dsim_bridge_funcs = {
	.attach = zuma_dsim_bridge_attach,
	.mode_set = zuma_dsim_bridge_mode_set,
	.atomic_pre_enable = zuma_dsim_bridge_pre_enable,
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
