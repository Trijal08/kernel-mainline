// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Samsung Exynos SoCs
 *
 * Copyright (C) 2013-2020 Samsung Electronics Co., Ltd.
 *		https://www.samsung.com
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *	   Jaehoon Chung <jh80.chung@samsung.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>

#include "pcie-designware.h"

#define to_exynos_pcie(x)	dev_get_drvdata((x)->dev)

/* PCIe ELBI registers */
#define PCIE_IRQ_PULSE			0x000
#define IRQ_INTA_ASSERT			BIT(0)
#define IRQ_INTB_ASSERT			BIT(2)
#define IRQ_INTC_ASSERT			BIT(4)
#define IRQ_INTD_ASSERT			BIT(6)
#define PCIE_IRQ_LEVEL			0x004
#define PCIE_IRQ_SPECIAL		0x008
#define PCIE_IRQ_EN_PULSE		0x00c
#define PCIE_IRQ_EN_LEVEL		0x010
#define PCIE_IRQ_EN_SPECIAL		0x014
/*
 * The Zuma/GS-family ELBI lays out a second IRQ bank ("IRQ2") that carries
 * the integrated MSI-receive summary. Its enable register sits at 0x018,
 * which on the older exynos5433 map is PCIE_SW_WAKE - so these must only be
 * used on the zuma path.
 */
#define PCIE_ZUMA_IRQ2			0x008
#define PCIE_ZUMA_IRQ2_EN		0x018
#define PCIE_ZUMA_IRQ2_MSI		BIT(17)
#define PCIE_SW_WAKE			0x018
#define PCIE_BUS_EN			BIT(1)
#define PCIE_CORE_RESET			0x01c
#define PCIE_CORE_RESET_ENABLE		BIT(0)
#define PCIE_STICKY_RESET		0x020
#define PCIE_NONSTICKY_RESET		0x024
#define PCIE_APP_INIT_RESET		0x028
#define PCIE_APP_LTSSM_ENABLE		0x02c
#define PCIE_ELBI_RDLH_LINKUP		0x074
#define PCIE_ELBI_XMLH_LINKUP		BIT(4)
#define PCIE_ELBI_LTSSM_ENABLE		0x1
#define PCIE_ELBI_SLV_AWMISC		0x11c
#define PCIE_ELBI_SLV_ARMISC		0x120
#define PCIE_ELBI_SLV_DBI_ENABLE	BIT(21)

/* ELBI registers for the Google Tensor (zuma/zumapro) controller */
#define PCIE_ZUMA_DEVICE_TYPE		0x0080
#define   PCIE_ZUMA_DEVICE_TYPE_RC	0x4
#define PCIE_ZUMA_SOFT_PWR_RESET	0x03a4
#define PCIE_ZUMA_PMA_RST_PCS		0x1400
#define PCIE_ZUMA_PMA_RST_PHY		0x1404
#define PCIE_ZUMA_PMA_RST_CMN		0x1408
#define PCIE_ZUMA_SLV_PEND_SEL_NAK	0x03d8
#define PCIE_ZUMA_APP_REQ_EXIT_L1	0x03bc
#define   APP_REQ_EXIT_L1_MODE		BIT(0)
#define   L1_REQ_NAK_CTRL_MASTER	BIT(4)
#define PCIE_ZUMA_LINKDOWN_RST_CTRL	0x03a0
#define   LINKDOWN_RST_MANUAL		BIT(1)
#define PCIE_ZUMA_APP_XFER_PENDING	0x0074
#define PCIE_ZUMA_QCH_SEL		0x03a8
#define   CLOCK_GATING_PMU_MASK		(0xf << 8)
#define   CLOCK_GATING_APB_MASK		(0xf << 4)
#define   CLOCK_GATING_AXI_MASK		(0xf << 0)
#define PCIE_ZUMA_MSTR_PEND_SEL_NAK	0x0474
#define   NACK_ENABLE			BIT(0)
#define PCIE_ZUMA_DBI_L1_EXIT_DISABLE	0x1078
#define   DBI_L1_EXIT_DISABLE		BIT(0)
#define PCIE_ZUMA_APP_LTSSM_ENABLE	0x0054
#define   LTSSM_ENABLE			BIT(0)
#define PCIE_ZUMA_RDLH_LINKUP		0x02c8
#define   LTSSM_STATE_MASK		0x3f
#define   LTSSM_STATE_L0		0x11
/* PMU PCIE_PHY control bit set during link bring-up */
#define PCIE_ZUMA_PMU_PHY_CTRL		BIT(10)
/* Auxiliary-clock frequency (drives the LTSSM timers) */
#define PCIE_AUX_CLK_FREQ_OFF		0xb40
#define PCIE_AUX_CLK_FREQ_24MHZ		0x18

struct exynos_pcie;

struct exynos_pcie_drvdata {
	const struct dw_pcie_ops	*dw_pcie_ops;
	const struct dw_pcie_host_ops	*host_ops;
	/* Tensor (zuma) controllers drive the link via GPIO PERST + PMU */
	bool				zuma;
};

struct exynos_pcie {
	struct dw_pcie			pci;
	const struct exynos_pcie_drvdata *drvdata;
	struct clk_bulk_data		*clks;
	struct phy			*phy;
	struct regulator_bulk_data	supplies[2];
	/* zuma only */
	struct regmap			*pmureg;
	struct gpio_desc		*perst_gpio;
	struct gpio_desc		*wlan_gpio;
	u32				pmu_offset;
	u32				perst_delay_us;
};

static void exynos_pcie_writel(void __iomem *base, u32 val, u32 reg)
{
	writel(val, base + reg);
}

static u32 exynos_pcie_readl(void __iomem *base, u32 reg)
{
	return readl(base + reg);
}

static void exynos_pcie_sideband_dbi_w_mode(struct exynos_pcie *ep, bool on)
{
	struct dw_pcie *pci = &ep->pci;
	u32 val;

	val = exynos_pcie_readl(pci->elbi_base, PCIE_ELBI_SLV_AWMISC);
	if (on)
		val |= PCIE_ELBI_SLV_DBI_ENABLE;
	else
		val &= ~PCIE_ELBI_SLV_DBI_ENABLE;
	exynos_pcie_writel(pci->elbi_base, val, PCIE_ELBI_SLV_AWMISC);
}

static void exynos_pcie_sideband_dbi_r_mode(struct exynos_pcie *ep, bool on)
{
	struct dw_pcie *pci = &ep->pci;
	u32 val;

	val = exynos_pcie_readl(pci->elbi_base, PCIE_ELBI_SLV_ARMISC);
	if (on)
		val |= PCIE_ELBI_SLV_DBI_ENABLE;
	else
		val &= ~PCIE_ELBI_SLV_DBI_ENABLE;
	exynos_pcie_writel(pci->elbi_base, val, PCIE_ELBI_SLV_ARMISC);
}

static void exynos_pcie_assert_core_reset(struct exynos_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	u32 val;

	val = exynos_pcie_readl(pci->elbi_base, PCIE_CORE_RESET);
	val &= ~PCIE_CORE_RESET_ENABLE;
	exynos_pcie_writel(pci->elbi_base, val, PCIE_CORE_RESET);
	exynos_pcie_writel(pci->elbi_base, 0, PCIE_STICKY_RESET);
	exynos_pcie_writel(pci->elbi_base, 0, PCIE_NONSTICKY_RESET);
}

static void exynos_pcie_deassert_core_reset(struct exynos_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	u32 val;

	val = exynos_pcie_readl(pci->elbi_base, PCIE_CORE_RESET);
	val |= PCIE_CORE_RESET_ENABLE;

	exynos_pcie_writel(pci->elbi_base, val, PCIE_CORE_RESET);
	exynos_pcie_writel(pci->elbi_base, 1, PCIE_STICKY_RESET);
	exynos_pcie_writel(pci->elbi_base, 1, PCIE_NONSTICKY_RESET);
	exynos_pcie_writel(pci->elbi_base, 1, PCIE_APP_INIT_RESET);
	exynos_pcie_writel(pci->elbi_base, 0, PCIE_APP_INIT_RESET);
}

static int exynos_pcie_start_link(struct dw_pcie *pci)
{
	u32 val;

	val = exynos_pcie_readl(pci->elbi_base, PCIE_SW_WAKE);
	val &= ~PCIE_BUS_EN;
	exynos_pcie_writel(pci->elbi_base, val, PCIE_SW_WAKE);

	/* assert LTSSM enable */
	exynos_pcie_writel(pci->elbi_base, PCIE_ELBI_LTSSM_ENABLE,
			  PCIE_APP_LTSSM_ENABLE);
	return 0;
}

static void exynos_pcie_clear_irq_pulse(struct exynos_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;

	u32 val = exynos_pcie_readl(pci->elbi_base, PCIE_IRQ_PULSE);

	exynos_pcie_writel(pci->elbi_base, val, PCIE_IRQ_PULSE);
}

static irqreturn_t exynos_pcie_irq_handler(int irq, void *arg)
{
	struct exynos_pcie *ep = arg;

	/*
	 * On zuma the controller multiplexes the integrated-MSI receive
	 * summary into this single line via the IRQ2 bank. There is no
	 * separate "msi" interrupt, so the DWC core never registers a chained
	 * MSI handler (pp->msi_irq[0] is forced to -ENODEV); demux it here.
	 */
	if (ep->drvdata->zuma) {
		struct dw_pcie *pci = &ep->pci;
		struct dw_pcie_rp *pp = &pci->pp;
		u32 val2 = exynos_pcie_readl(pci->elbi_base, PCIE_ZUMA_IRQ2);

		if (val2)
			exynos_pcie_writel(pci->elbi_base, val2, PCIE_ZUMA_IRQ2);
		if (IS_ENABLED(CONFIG_PCI_MSI) && (val2 & PCIE_ZUMA_IRQ2_MSI)) {
			int ctrl, num_ctrls;

			dw_handle_msi_irq(pp);

			/*
			 * The elbi MSI summary (IRQ2 bit17) latches on the
			 * rising edge of the integrated-MSI receiver output.
			 * A new MSI whose status bit gets set while a previous
			 * one is still pending produces no fresh rising edge
			 * and would be lost. Toggle the per-vector mask to
			 * force the receiver output low then high again,
			 * regenerating the edge for any still-pending MSI.
			 */
			num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;
			for (ctrl = 0; ctrl < num_ctrls; ctrl++) {
				u32 off = ctrl * MSI_REG_CTRL_BLOCK_SIZE;

				dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_MASK + off,
						   0xffffffff);
				dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_MASK + off,
						   pp->irq_mask[ctrl]);
			}
		}
		return IRQ_HANDLED;
	}

	exynos_pcie_clear_irq_pulse(ep);
	return IRQ_HANDLED;
}

static void exynos_pcie_enable_irq_pulse(struct exynos_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;

	u32 val = IRQ_INTA_ASSERT | IRQ_INTB_ASSERT |
		  IRQ_INTC_ASSERT | IRQ_INTD_ASSERT;

	exynos_pcie_writel(pci->elbi_base, val, PCIE_IRQ_EN_PULSE);
	exynos_pcie_writel(pci->elbi_base, 0, PCIE_IRQ_EN_LEVEL);
	exynos_pcie_writel(pci->elbi_base, 0, PCIE_IRQ_EN_SPECIAL);
}

static u32 exynos_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
				u32 reg, size_t size)
{
	struct exynos_pcie *ep = to_exynos_pcie(pci);
	u32 val;

	exynos_pcie_sideband_dbi_r_mode(ep, true);
	dw_pcie_read(base + reg, size, &val);
	exynos_pcie_sideband_dbi_r_mode(ep, false);
	return val;
}

static void exynos_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
				  u32 reg, size_t size, u32 val)
{
	struct exynos_pcie *ep = to_exynos_pcie(pci);

	exynos_pcie_sideband_dbi_w_mode(ep, true);
	dw_pcie_write(base + reg, size, val);
	exynos_pcie_sideband_dbi_w_mode(ep, false);
}

static int exynos_pcie_rd_own_conf(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(bus->sysdata);

	if (PCI_SLOT(devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	*val = dw_pcie_read_dbi(pci, where, size);
	return PCIBIOS_SUCCESSFUL;
}

static int exynos_pcie_wr_own_conf(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(bus->sysdata);

	if (PCI_SLOT(devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	dw_pcie_write_dbi(pci, where, size, val);
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops exynos_pci_ops = {
	.read = exynos_pcie_rd_own_conf,
	.write = exynos_pcie_wr_own_conf,
};

static bool exynos_pcie_link_up(struct dw_pcie *pci)
{
	u32 val = exynos_pcie_readl(pci->elbi_base, PCIE_ELBI_RDLH_LINKUP);

	return val & PCIE_ELBI_XMLH_LINKUP;
}

static int exynos_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *ep = to_exynos_pcie(pci);

	pp->bridge->ops = &exynos_pci_ops;

	exynos_pcie_assert_core_reset(ep);

	phy_init(ep->phy);
	phy_power_on(ep->phy);

	exynos_pcie_deassert_core_reset(ep);
	exynos_pcie_enable_irq_pulse(ep);

	return 0;
}

static const struct dw_pcie_host_ops exynos_pcie_host_ops = {
	.init = exynos_pcie_host_init,
};

/* Google Tensor (zuma/zumapro) controller -------------------------------- */

/*
 * The PMA reset bits live in the ELBI bank, so the controller (not the PHY)
 * owns them and brackets the PHY callbacks: assert before phy_init(), release
 * before phy_power_on() (which then waits for the PLL/CDR locks).
 */
static void exynos_zuma_assert_pma_reset(struct exynos_pcie *ep)
{
	void __iomem *elbi = ep->pci.elbi_base;

	/* device type = root complex */
	exynos_pcie_writel(elbi, PCIE_ZUMA_DEVICE_TYPE_RC, PCIE_ZUMA_DEVICE_TYPE);

	/* soft power reset */
	exynos_pcie_writel(elbi, 0xf, PCIE_ZUMA_SOFT_PWR_RESET);
	exynos_pcie_writel(elbi, 0xd, PCIE_ZUMA_SOFT_PWR_RESET);
	udelay(10);
	exynos_pcie_writel(elbi, 0xf, PCIE_ZUMA_SOFT_PWR_RESET);
	udelay(10);

	/* pulse the PMA reset */
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_PMA_RST_PHY);
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_PMA_RST_CMN);
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_PMA_RST_PCS);
	exynos_pcie_writel(elbi, 0, PCIE_ZUMA_PMA_RST_PHY);
	exynos_pcie_writel(elbi, 0, PCIE_ZUMA_PMA_RST_CMN);
	exynos_pcie_writel(elbi, 0, PCIE_ZUMA_PMA_RST_PCS);

	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_SLV_PEND_SEL_NAK);
}

static void exynos_zuma_release_pma_reset(struct exynos_pcie *ep)
{
	void __iomem *elbi = ep->pci.elbi_base;

	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_PMA_RST_PHY);
	udelay(10);
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_PMA_RST_CMN);
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_PMA_RST_PCS);
}

static int exynos_zuma_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos_pcie *ep = to_exynos_pcie(pci);

	pp->bridge->ops = &exynos_pci_ops;

	exynos_pcie_assert_core_reset(ep);

	/* power the endpoint (e.g. Wi-Fi WLAN_EN / REG_ON) */
	gpiod_set_value_cansleep(ep->wlan_gpio, 1);

	/* enable the PCIE_PHY control in the PMU */
	regmap_update_bits(ep->pmureg, ep->pmu_offset,
			   PCIE_ZUMA_PMU_PHY_CTRL, PCIE_ZUMA_PMU_PHY_CTRL);

	/*
	 * Vendor ordering: bring up the PHY input clock + reference PLL first,
	 * then issue the ELBI soft-power/PMA reset, then program the PHY.
	 */
	phy_reset(ep->phy);
	exynos_zuma_assert_pma_reset(ep);
	phy_init(ep->phy);
	exynos_zuma_release_pma_reset(ep);
	phy_power_on(ep->phy);

	exynos_pcie_deassert_core_reset(ep);
	exynos_pcie_enable_irq_pulse(ep);

	/*
	 * Route the integrated-MSI receive summary onto the controller IRQ.
	 * The endpoint (e.g. the BCM4390 Wi-Fi) signals all msgbuf completions
	 * via MSI; without this the host never sees the interrupt and every
	 * ioctl to the dongle times out.
	 */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		u32 val = exynos_pcie_readl(pci->elbi_base, PCIE_ZUMA_IRQ2_EN);

		val |= PCIE_ZUMA_IRQ2_MSI;
		exynos_pcie_writel(pci->elbi_base, val, PCIE_ZUMA_IRQ2_EN);
	}

	return 0;
}

static int exynos_zuma_pcie_start_link(struct dw_pcie *pci)
{
	struct exynos_pcie *ep = to_exynos_pcie(pci);
	void __iomem *elbi = pci->elbi_base;
	u8 exp_cap;
	u32 val;

	/* deassert PERST to the endpoint and let it come up */
	gpiod_set_value_cansleep(ep->perst_gpio, 1);
	usleep_range(ep->perst_delay_us, ep->perst_delay_us + 2000);

	/* keep the link out of L1 during training */
	val = exynos_pcie_readl(elbi, PCIE_ZUMA_APP_REQ_EXIT_L1);
	val |= APP_REQ_EXIT_L1_MODE | L1_REQ_NAK_CTRL_MASTER;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_APP_REQ_EXIT_L1);

	exynos_pcie_writel(elbi, LINKDOWN_RST_MANUAL, PCIE_ZUMA_LINKDOWN_RST_CTRL);
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_APP_XFER_PENDING);

	/* do not clock-gate the link while training */
	val = exynos_pcie_readl(elbi, PCIE_ZUMA_QCH_SEL);
	val &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK |
		 CLOCK_GATING_AXI_MASK);
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_QCH_SEL);

	exynos_pcie_writel(elbi, NACK_ENABLE, PCIE_ZUMA_MSTR_PEND_SEL_NAK);
	exynos_pcie_writel(elbi, DBI_L1_EXIT_DISABLE,
			   PCIE_ZUMA_DBI_L1_EXIT_DISABLE);

	/*
	 * Tell the controller its real auxiliary-clock rate (24.576MHz
	 * oscclk) so the LTSSM timers are calibrated correctly. The vendor
	 * value of 26MHz is for the OOT's 26MHz aux clock and miscalibrates
	 * detection on this 24.576MHz mainline clock.
	 */
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writel_dbi(pci, PCIE_AUX_CLK_FREQ_OFF, PCIE_AUX_CLK_FREQ_24MHZ);

	/*
	 * Do not advertise ASPM L1 on this link. Reliable L1 exit on the
	 * Synopsys PHY requires a CLKREQ#/LTR handshake that the endpoint
	 * driver is expected to negotiate once the device is fully up; until
	 * then the link cannot reliably leave L1 and config-space accesses
	 * time out. Clear the root port's advertised L1 support so the PCI
	 * core never enables ASPM L1 on this link.
	 */
	exp_cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	val = dw_pcie_readl_dbi(pci, exp_cap + PCI_EXP_LNKCAP);
	val &= ~PCI_EXP_LNKCAP_ASPM_L1;
	dw_pcie_writel_dbi(pci, exp_cap + PCI_EXP_LNKCAP, val);
	dw_pcie_dbi_ro_wr_dis(pci);

	/* assert LTSSM enable */
	exynos_pcie_writel(elbi, LTSSM_ENABLE, PCIE_ZUMA_APP_LTSSM_ENABLE);

	return 0;
}

static bool exynos_zuma_pcie_link_up(struct dw_pcie *pci)
{
	u32 val = exynos_pcie_readl(pci->elbi_base, PCIE_ZUMA_RDLH_LINKUP);

	return (val & LTSSM_STATE_MASK) == LTSSM_STATE_L0;
}

static const struct dw_pcie_host_ops exynos_zuma_pcie_host_ops = {
	.init = exynos_zuma_pcie_host_init,
};

static int exynos_add_pcie_port(struct exynos_pcie *ep,
				       struct platform_device *pdev)
{
	struct dw_pcie *pci = &ep->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->irq = platform_get_irq(pdev, 0);
	if (pp->irq < 0)
		return pp->irq;

	ret = devm_request_irq(dev, pp->irq, exynos_pcie_irq_handler,
			       IRQF_SHARED, "exynos-pcie", ep);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		return ret;
	}

	pp->ops = ep->drvdata->host_ops;
	pp->msi_irq[0] = -ENODEV;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.read_dbi = exynos_pcie_read_dbi,
	.write_dbi = exynos_pcie_write_dbi,
	.link_up = exynos_pcie_link_up,
	.start_link = exynos_pcie_start_link,
};

static const struct dw_pcie_ops exynos_zuma_dw_pcie_ops = {
	.read_dbi = exynos_pcie_read_dbi,
	.write_dbi = exynos_pcie_write_dbi,
	.link_up = exynos_zuma_pcie_link_up,
	.start_link = exynos_zuma_pcie_start_link,
};

static const struct exynos_pcie_drvdata exynos5433_pcie_drvdata = {
	.dw_pcie_ops	= &dw_pcie_ops,
	.host_ops	= &exynos_pcie_host_ops,
	.zuma		= false,
};

static const struct exynos_pcie_drvdata zuma_pcie_drvdata = {
	.dw_pcie_ops	= &exynos_zuma_dw_pcie_ops,
	.host_ops	= &exynos_zuma_pcie_host_ops,
	.zuma		= true,
};

static int exynos_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_pcie *ep;
	struct device_node *np = dev->of_node;
	int ret;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->drvdata = of_device_get_match_data(dev);
	if (!ep->drvdata)
		return -EINVAL;

	ep->pci.dev = dev;
	ep->pci.ops = ep->drvdata->dw_pcie_ops;

	ep->phy = devm_of_phy_get(dev, np, NULL);
	if (IS_ERR(ep->phy))
		return PTR_ERR(ep->phy);

	ret = devm_clk_bulk_get_all_enabled(dev, &ep->clks);
	if (ret < 0)
		return ret;

	if (ep->drvdata->zuma) {
		ep->pmureg = syscon_regmap_lookup_by_phandle(np,
							"samsung,pmu-syscon");
		if (IS_ERR(ep->pmureg))
			return dev_err_probe(dev, PTR_ERR(ep->pmureg),
					     "PMU regmap lookup failed\n");

		if (of_property_read_u32(np, "samsung,pmu-offset",
					 &ep->pmu_offset))
			return dev_err_probe(dev, -EINVAL,
					     "missing samsung,pmu-offset\n");

		ep->perst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(ep->perst_gpio))
			return dev_err_probe(dev, PTR_ERR(ep->perst_gpio),
					     "failed to get PERST GPIO\n");

		ep->wlan_gpio = devm_gpiod_get_optional(dev, "wlan-reg-on",
							GPIOD_OUT_LOW);
		if (IS_ERR(ep->wlan_gpio))
			return dev_err_probe(dev, PTR_ERR(ep->wlan_gpio),
					     "failed to get WLAN GPIO\n");

		ep->perst_delay_us = 15000;
		of_property_read_u32(np, "samsung,perst-delay-us",
				     &ep->perst_delay_us);
	} else {
		ep->supplies[0].supply = "vdd18";
		ep->supplies[1].supply = "vdd10";
		ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ep->supplies),
					      ep->supplies);
		if (ret)
			return ret;

		ret = regulator_bulk_enable(ARRAY_SIZE(ep->supplies),
					    ep->supplies);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, ep);

	ret = exynos_add_pcie_port(ep, pdev);
	if (ret < 0)
		goto fail_probe;

	return 0;

fail_probe:
	phy_exit(ep->phy);
	if (!ep->drvdata->zuma)
		regulator_bulk_disable(ARRAY_SIZE(ep->supplies), ep->supplies);

	return ret;
}

static void exynos_pcie_remove(struct platform_device *pdev)
{
	struct exynos_pcie *ep = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&ep->pci.pp);
	exynos_pcie_assert_core_reset(ep);
	phy_power_off(ep->phy);
	phy_exit(ep->phy);
	if (!ep->drvdata->zuma)
		regulator_bulk_disable(ARRAY_SIZE(ep->supplies), ep->supplies);
}

static int exynos_pcie_suspend_noirq(struct device *dev)
{
	struct exynos_pcie *ep = dev_get_drvdata(dev);

	exynos_pcie_assert_core_reset(ep);
	phy_power_off(ep->phy);
	phy_exit(ep->phy);
	if (!ep->drvdata->zuma)
		regulator_bulk_disable(ARRAY_SIZE(ep->supplies), ep->supplies);

	return 0;
}

static int exynos_pcie_resume_noirq(struct device *dev)
{
	struct exynos_pcie *ep = dev_get_drvdata(dev);
	struct dw_pcie *pci = &ep->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	int ret;

	if (!ep->drvdata->zuma) {
		ret = regulator_bulk_enable(ARRAY_SIZE(ep->supplies),
					    ep->supplies);
		if (ret)
			return ret;
	}

	/* the host_init callback controls ep->phy */
	ep->drvdata->host_ops->init(pp);
	dw_pcie_setup_rc(pp);
	ep->drvdata->dw_pcie_ops->start_link(pci);
	return dw_pcie_wait_for_link(pci);
}

static const struct dev_pm_ops exynos_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos_pcie_suspend_noirq,
				  exynos_pcie_resume_noirq)
};

static const struct of_device_id exynos_pcie_of_match[] = {
	{
		.compatible = "samsung,exynos5433-pcie",
		.data = &exynos5433_pcie_drvdata,
	},
	{
		.compatible = "google,zuma-pcie",
		.data = &zuma_pcie_drvdata,
	},
	{ },
};

static struct platform_driver exynos_pcie_driver = {
	.probe		= exynos_pcie_probe,
	.remove		= exynos_pcie_remove,
	.driver = {
		.name	= "exynos-pcie",
		.of_match_table = exynos_pcie_of_match,
		.pm		= &exynos_pcie_pm_ops,
	},
};
module_platform_driver(exynos_pcie_driver);
MODULE_DESCRIPTION("Samsung Exynos PCIe host controller driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, exynos_pcie_of_match);
