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

#include <linux/bitmap.h>
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
#include <linux/soc/samsung/exynos-cp-power.h>
#include <linux/module.h>

#include <linux/pcie-zumapro.h>

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
#define   SOFT_RESET_ALL		0xf
#define   SOFT_RESET_PWR_PULSE		0xd	/* SOFT_RESET_ALL with PWR bit low */
#define   SOFT_PWR_RESET		BIT(1)	/* active-low: 1 released, 0 asserted */
#define   SOFT_NON_STICKY_RESET		BIT(3)	/* active-low, as above */
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
#define   LTSSM_STATE_RCVRY_LOCK	0x0d
#define   LTSSM_STATE_L0		0x11
#define   LTSSM_STATE_L1_IDLE		0x14
#define   LTSSM_STATE_L2_IDLE		0x15
/* Orderly-L2 power-down handshake (modem runtime PM; downstream poweroff_pcie) */
#define PCIE_APP_REQ_EXIT_L1		0x006c	/* L1-exit request trigger */
#define PCIE_XMIT_PME_TURNOFF		0x0118
#define   IRQ_RADM_PM_TO_ACK		BIT(29)	/* PM_Turn_Off_Ack in PCIE_IRQ_PULSE */
#define PCIE_L2_ENTER_WAIT_US		20000
#define PCIE_L2_ENTER_WAIT_STEP_US	10
/* Runtime relink retrains from L2 -- the first attempt can fail; retry with a
 * full re-config each time (downstream establish_link retries the same way). */
#define PCIE_MODEM_RELINK_RETRIES	10
#define PCIE_LINK_WAIT_US		50000
#define PCIE_LINK_WAIT_STEP_US		10
#define PCIE_PERST_DELAY_US		20000
#define PCIE_GEN3_RELATED		0x890	/* DBI: GEN3 EQ control */
#define   GEN3_EQ_OFF			0x12000	/* skip Gen3 EQ phases 2/3 */
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
	/*
	 * Modem (CH0) CP power/reset sequencing lives in the CP power driver
	 * (exynos-cp-power); the RC keeps only AP2CP_WAKEUP, the runtime PCIe
	 * link-request signal it drives outside reset.
	 */
	struct gpio_desc		*cp_wakeup;
	struct exynos_cp_power		*cp_power;
	/* modem runtime-relink state (downstream establish_link) */
	bool				cp_phy_off;	/* PHY left powered off by link_down */
	bool				msi_saved;
	u32				saved_msi_enable;
	u32				saved_msi_mask;
	u32				saved_msi_addr_lo;
	u32				saved_msi_addr_hi;
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

static int exynos_pcie_cp_get_gpios(struct exynos_pcie *ep)
{
	struct device *dev = ep->pci.dev;

	/*
	 * The CP power/reset sequencer (rail GPIOs + CP PMIC/S5910 over SPMI) is
	 * a separate device.  A non-modem RC (e.g. WiFi CH1) has no
	 * google,cp-power phandle and skips all of this.
	 */
	ep->cp_power = exynos_cp_power_get(dev);
	if (IS_ERR(ep->cp_power)) {
		int ret = PTR_ERR(ep->cp_power);

		ep->cp_power = NULL;
		if (ret == -ENOENT)
			return 0;	/* not a modem RC */
		return dev_err_probe(dev, ret, "CP power sequencer\n");
	}

	/*
	 * AP2CP_WAKEUP stays with the RC: it is the runtime PCIe link-request
	 * signal (driven high to wake a parked CP), used by the link PM path and
	 * not just at reset.  Request it as an output (start low = not requested)
	 * or every gpiod_set_value() on it is a silent no-op.
	 */
	ep->cp_wakeup = devm_gpiod_get(dev, "google,cp-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(ep->cp_wakeup))
		return dev_err_probe(dev, PTR_ERR(ep->cp_wakeup), "cp-wakeup\n");

	/* Warm-reset the CP into its boot ROM before link training. */
	return exynos_cp_power_warm_reset(ep->cp_power);
}

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

	/* Cold-power the modem CP (no-op for a non-modem RC) before training. */
	ret = exynos_pcie_cp_get_gpios(ep);
	if (ret < 0)
		goto fail_probe;

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

/*
 * Bring-up hooks for the Exynos Modem 5300 boot driver (see
 * <linux/pcie-zumapro.h>). The modem's mask ROM needs its MSI target moved
 * into the CP's carveout and a mid-boot link bounce; both are RC-internal
 * operations the WWAN driver reaches through the RC's DT phandle.
 */
static struct exynos_pcie *zumapro_pcie_from_dev(struct device *rc_dev)
{
	if (!rc_dev->driver ||
	    rc_dev->driver->of_match_table != exynos_pcie_of_match)
		return NULL;

	return dev_get_drvdata(rc_dev);
}

int zumapro_pcie_set_msi_target(struct device *rc_dev, phys_addr_t target)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);

	if (!ep)
		return -ENODEV;

	ep->pci.pp.msi_data = target;
	dw_pcie_writel_dbi(&ep->pci, PCIE_MSI_ADDR_LO, lower_32_bits(target));
	dw_pcie_writel_dbi(&ep->pci, PCIE_MSI_ADDR_HI, upper_32_bits(target));

	dev_info(rc_dev, "MSI target moved to %pap\n", &target);
	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_set_msi_target);

/*
 * Reserve the first @count MSI vectors on this root complex so the next
 * endpoint allocation lands above them.  The s5300 CP signals the AP on MSI
 * message-data base 4 (m1n1 trace of a working downstream boot: EP data 4,
 * INIT_START and every MAIN-phase interrupt on bit 4).  The generic DWC MSI
 * domain otherwise hands out the lowest free region (base 0); reserving
 * vectors 0-3 makes the modem's four-vector request land at base 4 with MME=2
 * -- the width the mask ROM tolerates (requesting eight to reach bit 4 sets
 * MME=3 and aborts the PBL download at boot_stage 0x1ff).  Must run before the
 * endpoint's MSI vectors are allocated.
 */
int zumapro_pcie_reserve_msi_base(struct device *rc_dev, unsigned int count)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);

	if (!ep)
		return -ENODEV;
	if (!count || count > ep->pci.pp.num_vectors)
		return -EINVAL;

	bitmap_set(ep->pci.pp.msi_irq_in_use, 0, count);
	dev_info(rc_dev,
		 "reserved MSI vectors 0-%u so the modem lands at base %u\n",
		 count - 1, count);
	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_reserve_msi_base);

/*
 * Controller soft-reset pulse used per runtime-relink attempt (downstream
 * establish_link / controller_reset_pulse).  A bare PERST retrain out of the
 * orderly-L2 park parks the LTSSM in Polling; pulsing SOFT_PWR_RESET then
 * SOFT_NON_STICKY_RESET (both active-low) re-arms the MAC so training completes.
 */
static void exynos_zuma_controller_reset_pulse(struct exynos_pcie *ep)
{
	void __iomem *elbi = ep->pci.elbi_base;
	u32 val;

	val = exynos_pcie_readl(elbi, PCIE_ZUMA_SOFT_PWR_RESET);
	val &= ~SOFT_PWR_RESET;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_SOFT_PWR_RESET);
	mdelay(1);
	val |= SOFT_PWR_RESET;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_SOFT_PWR_RESET);

	exynos_pcie_writel(elbi, PCIE_ZUMA_DEVICE_TYPE_RC, PCIE_ZUMA_DEVICE_TYPE);

	val = exynos_pcie_readl(elbi, PCIE_ZUMA_SOFT_PWR_RESET);
	val |= SOFT_NON_STICKY_RESET;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_SOFT_PWR_RESET);
	usleep_range(10, 12);
	val &= ~SOFT_NON_STICKY_RESET;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_SOFT_PWR_RESET);
	mdelay(1);
	val |= SOFT_NON_STICKY_RESET;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_SOFT_PWR_RESET);
}

/* ELBI runtime-relink config (downstream config_elbi): L1-exit disable, L1 mode
 * + master NAK, manual link-down reset, transfer-pending fence, Q-channel clock
 * ungated, and master-port NAK. */
static void exynos_zuma_config_elbi(struct exynos_pcie *ep)
{
	void __iomem *elbi = ep->pci.elbi_base;
	u32 val;

	exynos_pcie_writel(elbi, DBI_L1_EXIT_DISABLE, PCIE_ZUMA_DBI_L1_EXIT_DISABLE);

	val = exynos_pcie_readl(elbi, PCIE_ZUMA_APP_REQ_EXIT_L1);
	val |= APP_REQ_EXIT_L1_MODE | L1_REQ_NAK_CTRL_MASTER;
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_APP_REQ_EXIT_L1);

	exynos_pcie_writel(elbi, LINKDOWN_RST_MANUAL, PCIE_ZUMA_LINKDOWN_RST_CTRL);
	exynos_pcie_writel(elbi, 1, PCIE_ZUMA_APP_XFER_PENDING);

	val = exynos_pcie_readl(elbi, PCIE_ZUMA_QCH_SEL);
	val &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK |
		 CLOCK_GATING_AXI_MASK);
	exynos_pcie_writel(elbi, val, PCIE_ZUMA_QCH_SEL);

	exynos_pcie_writel(elbi, NACK_ENABLE, PCIE_ZUMA_MSTR_PEND_SEL_NAK);
}

/* Wait for the retrained modem link to reach an up state (Recovery..L1_IDLE),
 * downstream wait_link_up -- broader than link_up()'s strict L0 so an L1-parked
 * link (the CP idles fast) still counts as trained. */
static int exynos_zuma_modem_wait_link(struct exynos_pcie *ep)
{
	u32 state;

	return readl_poll_timeout(ep->pci.elbi_base + PCIE_ZUMA_RDLH_LINKUP,
				  state,
				  (state & LTSSM_STATE_MASK) >=
				  LTSSM_STATE_RCVRY_LOCK &&
				  (state & LTSSM_STATE_MASK) <=
				  LTSSM_STATE_L1_IDLE,
				  PCIE_LINK_WAIT_STEP_US, PCIE_LINK_WAIT_US);
}

int zumapro_pcie_modem_link_down(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);
	void __iomem *elbi;
	u32 val, mode;
	int ret;

	if (!ep)
		return -ENODEV;
	elbi = ep->pci.elbi_base;

	/*
	 * Keep the CP's PCIe refclk alive across the bounce: the PMA lanes are
	 * power-cycled (so the link retrains), but the PHY is NOT isolated, so the
	 * external PLL that clocks the discrete CP keeps running.  Without this the
	 * CP loses its reference clock during the relink and the retrain parks in
	 * Polling (ltssm 0x3).  Cleared in modem_link_up() once the link is back.
	 */
	exynos_pcie_phy_keep_refclk(ep->phy, true);

	/*
	 * Re-establishing the CP link needs a full PHY re-cal (a light bounce
	 * leaves the LTSSM stuck at PRE_DETECT_QUIET).  Park the link the way the
	 * downstream establish_link teardown does: drop AP2CP_WAKEUP (tell the CP
	 * the AP intends the link down), take the link orderly to L2, snapshot the
	 * integrated-MSI receiver (the controller reset zeroes it), assert PERST,
	 * switch the PHY sub-block clock to the safe OSC, then soft-reset the
	 * controller and power the PHY off.  link_up() reverses each step per
	 * retrain attempt.
	 */
	gpiod_set_value_cansleep(ep->cp_wakeup, 0);
	usleep_range(5000, 6000);

	/*
	 * Orderly L2 power-down BEFORE PERST: PME_Turn_Off -> wait PM_TO_ACK ->
	 * wait for the link to reach L2_IDLE, mirroring the vendor
	 * exynos_pcie_rc_send_pme_turn_off().  At runtime (CP-driven PM) the CP's
	 * inbound doorbell decode survives an orderly L2 entry but NOT a surprise
	 * PERST-at-L1, so MAIN keeps its runtime IPC armed across the relink; a
	 * bare PERST here would drop MAIN.  Only run it while the link is up (LTSSM
	 * in the recovery..L1 range); ELBI access is safe -- the EP still drives
	 * CLKREQ#.  Proceed on timeout (like downstream) but log the reached state.
	 */
	val = exynos_pcie_readl(elbi, PCIE_ZUMA_RDLH_LINKUP) & LTSSM_STATE_MASK;
	dev_info(ep->pci.dev, "link_down: entry ltssm %#x\n", val);
	if (val >= LTSSM_STATE_RCVRY_LOCK && val <= LTSSM_STATE_L1_IDLE) {
		/* PCIE_IRQ_PULSE is write-1-to-clear; clear any stale PM_TO_ACK. */
		val = exynos_pcie_readl(elbi, PCIE_IRQ_PULSE);
		exynos_pcie_writel(elbi, val, PCIE_IRQ_PULSE);

		exynos_pcie_writel(elbi, 1, PCIE_APP_REQ_EXIT_L1);
		mode = exynos_pcie_readl(elbi, PCIE_ZUMA_APP_REQ_EXIT_L1);
		mode &= ~APP_REQ_EXIT_L1_MODE;
		mode |= L1_REQ_NAK_CTRL_MASTER;
		exynos_pcie_writel(elbi, mode, PCIE_ZUMA_APP_REQ_EXIT_L1);

		exynos_pcie_writel(elbi, 1, PCIE_XMIT_PME_TURNOFF);
		ret = readl_poll_timeout(elbi + PCIE_IRQ_PULSE, val,
					 val & IRQ_RADM_PM_TO_ACK,
					 PCIE_L2_ENTER_WAIT_STEP_US,
					 PCIE_L2_ENTER_WAIT_US);
		if (ret)
			dev_warn(ep->pci.dev,
				 "no PM_TO_ACK from CP (irq %#x)\n", val);
		udelay(10);
		exynos_pcie_writel(elbi, 0, PCIE_XMIT_PME_TURNOFF);

		ret = readl_poll_timeout(elbi + PCIE_ZUMA_RDLH_LINKUP, val,
					 (val & LTSSM_STATE_MASK) ==
					 LTSSM_STATE_L2_IDLE,
					 PCIE_L2_ENTER_WAIT_STEP_US,
					 PCIE_L2_ENTER_WAIT_US);
		if (ret)
			dev_info(ep->pci.dev,
				 "link_down: did NOT reach L2_IDLE (ltssm %#x)\n",
				 val & LTSSM_STATE_MASK);
		else
			dev_info(ep->pci.dev, "link_down: reached L2_IDLE\n");
	}

	/*
	 * Snapshot the integrated-MSI receiver before the controller reset zeroes
	 * it.  SOFT_RESET clears the MSI enable/mask/target registers, and
	 * dw_pcie_setup_rc() on the relink does not re-arm the RC's own receiver,
	 * so without this the CP's post-relink MSI (message 4 = INIT_START) is
	 * silently dropped.  link_up() restores it after setup_rc.
	 */
	ep->saved_msi_addr_lo = dw_pcie_readl_dbi(&ep->pci, PCIE_MSI_ADDR_LO);
	ep->saved_msi_addr_hi = dw_pcie_readl_dbi(&ep->pci, PCIE_MSI_ADDR_HI);
	ep->saved_msi_enable = dw_pcie_readl_dbi(&ep->pci, PCIE_MSI_INTR0_ENABLE);
	ep->saved_msi_mask = dw_pcie_readl_dbi(&ep->pci, PCIE_MSI_INTR0_MASK);
	ep->msi_saved = true;

	gpiod_set_value_cansleep(ep->perst_gpio, 0);	/* assert PERST */

	/*
	 * With the link down the CP stops driving CLKREQ#, so the external-PLL
	 * PHY clock is gated -- an ELBI access on it would stall the interconnect.
	 * Switch the PHY sub-block clock to the safe OSC before the soft reset.
	 */
	exynos_pcie_phy_safe_clk(ep->phy, true);

	exynos_pcie_writel(elbi, 0, PCIE_ZUMA_APP_LTSSM_ENABLE);

	/*
	 * Controller soft reset (downstream link_down): pulse SOFT_RESET with the
	 * power bit low, then assert all.  Clears the half-dead MAC state so the
	 * next link_up re-config trains cleanly.
	 */
	exynos_pcie_writel(elbi, SOFT_RESET_PWR_PULSE, PCIE_ZUMA_SOFT_PWR_RESET);
	udelay(20);
	exynos_pcie_writel(elbi, SOFT_RESET_ALL, PCIE_ZUMA_SOFT_PWR_RESET);

	/*
	 * Power the PHY down (isolate) but keep it INIT'd -- link_up() cycles only
	 * the PHY power + PMA reset per retrain attempt, matching the downstream
	 * establish_link path, and re-running phy_init() each attempt would leak
	 * the init refcount across the repeated runtime relinks.  phy_power_off()
	 * isolates the PHY (PMU bit0=0), which would hang the first ELBI access;
	 * link_up() un-isolates with phy_reset() before it touches the controller.
	 */
	phy_power_off(ep->phy);
	ep->cp_phy_off = true;
	usleep_range(1000, 2000);
	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_link_down);

/*
 * The CP mask ROM links CH0 at Gen1; its second-stage bootloader runs the link
 * at Gen3, and the vendor poweron_pcie() retrains to max speed after the boot
 * bounce.  Direct a Gen3 speed change so the CP endpoint sees the full-speed
 * link it expects post-bounce (its doorbell stays dead on the Gen1 relink).
 */
static void exynos_zuma_pcie_gen3(struct dw_pcie *pci)
{
	u8 exp = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u16 reg16 = 0;
	u32 reg;
	int i;

	dw_pcie_dbi_ro_wr_en(pci);
	reg = dw_pcie_readl_dbi(pci, exp + PCI_EXP_LNKCAP);
	reg = (reg & ~PCI_EXP_LNKCAP_SLS) | PCI_EXP_LNKCAP_SLS_8_0GB;
	dw_pcie_writel_dbi(pci, exp + PCI_EXP_LNKCAP, reg);
	dw_pcie_dbi_ro_wr_dis(pci);

	reg16 = dw_pcie_readw_dbi(pci, exp + PCI_EXP_LNKCTL2);
	reg16 = (reg16 & ~PCI_EXP_LNKCTL2_TLS) | PCI_EXP_LNKCTL2_TLS_8_0GT;
	dw_pcie_writew_dbi(pci, exp + PCI_EXP_LNKCTL2, reg16);

	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	reg |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, reg);

	reg16 = dw_pcie_readw_dbi(pci, exp + PCI_EXP_LNKCTL);
	reg16 |= PCI_EXP_LNKCTL_RL;			/* retrain link */
	dw_pcie_writew_dbi(pci, exp + PCI_EXP_LNKCTL, reg16);

	for (i = 0; i < 100; i++) {
		reg16 = dw_pcie_readw_dbi(pci, exp + PCI_EXP_LNKSTA);
		if ((reg16 & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_8_0GB)
			break;
		usleep_range(1000, 2000);
	}
	dev_info(pci->dev, "CH0 link speed after retrain: %s (LNKSTA %#x)\n",
		 (reg16 & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_8_0GB ?
			"Gen3" : "not Gen3", reg16);
}

/*
 * Retrain CH0 to Gen3 on the LIVE link (no PHY teardown).  The CP's mask ROM
 * links at Gen1; its second-stage bootloader runs the link at Gen3 and its
 * doorbell stays dead until the retrain, so the modem driver calls this once
 * the bootloader is up.  The vendor never bounces the link -- mainline's
 * inbound path is already 1:1 passthrough (the HSI1 PCIe SysMMU is unmanaged =
 * bypass), so the CP can DMA the IPC ring and raise INIT_START in place.
 */
int zumapro_pcie_modem_gen3(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);

	if (!ep)
		return -ENODEV;

	exynos_zuma_pcie_gen3(&ep->pci);
	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_gen3);

int zumapro_pcie_modem_link_up(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);
	struct dw_pcie *pci;
	int ret = -ETIMEDOUT, try;

	if (!ep)
		return -ENODEV;
	pci = &ep->pci;

	/* AP2CP_WAKEUP high: the CP gates its link participation on it. */
	gpiod_set_value_cansleep(ep->cp_wakeup, 1);

	/*
	 * The post-PBL CP endpoint is 2-lane and gates its IPC start on a Gen3 x2
	 * link, so re-link at x2/Gen3 -- dw_pcie_setup_rc() arms the directed speed
	 * change, so training completes at Gen1 and upshifts.
	 */
	pci->num_lanes = 2;
	pci->max_link_speed = 3;

	/*
	 * Runtime relink retrain loop, ported wholesale from the downstream
	 * establish_link path (steffan-modem): retraining out of the orderly-L2
	 * park parks the LTSSM in Polling on the first PERST cycle, and a bare
	 * retry wedges the half-dead core, so re-run the FULL re-config -- PHY
	 * power cycle + PMA re-config + controller soft-reset pulse + ELBI
	 * re-config + PERST cycle + RC re-setup + Gen3-EQ-off -- on EVERY attempt.
	 *
	 * Crucially the PMA reset wipes the PHY's PMA/PCS configuration, which
	 * this driver programs in the PHY .init op (zuma_pcie_phy_init), so the
	 * tables MUST be re-written between the assert and release of the PMA
	 * reset -- exactly the cold host_init order (phy_reset, assert_pma,
	 * phy_init, release_pma, phy_power_on).  phy_init() is refcounted and
	 * would no-op on the second call, so drop the init count first with
	 * phy_exit(); the exit/init pair nets to zero across the bounce (the
	 * downstream PHY instead carries these table writes in its power_on, which
	 * its power_off/on cycle re-runs -- same effect, different split).  Without
	 * the re-write the PMA comes up unconfigured and the link parks in Polling.
	 *
	 * link_down() left the PHY powered off + PERST asserted, so the first pass
	 * skips the extra phy_power_off().
	 */
	for (try = 0; try < PCIE_MODEM_RELINK_RETRIES; try++) {
		gpiod_set_value_cansleep(ep->perst_gpio, 0);	/* assert PERST */
		usleep_range(1000, 2000);

		/*
		 * link_down() already parked the clock safe and powered the PHY
		 * off (cp_phy_off), so the first pass skips it; a retry re-enters
		 * with the PHY powered, so park + power it off before re-cal.
		 */
		if (!ep->cp_phy_off) {
			exynos_pcie_phy_safe_clk(ep->phy, true);
			phy_power_off(ep->phy);
		}
		ep->cp_phy_off = false;

		phy_exit(ep->phy);		/* drop init count so phy_init re-runs */
		phy_reset(ep->phy);		/* un-isolate PHY (PMU), re-lock clk */

		/* PMU PCIE_PHY/WAKE control (bit10) so the CP's PHY re-enables. */
		regmap_update_bits(ep->pmureg, ep->pmu_offset,
				   PCIE_ZUMA_PMU_PHY_CTRL, PCIE_ZUMA_PMU_PHY_CTRL);

		exynos_zuma_assert_pma_reset(ep);
		phy_init(ep->phy);		/* re-write PMA/PCS config tables */
		exynos_zuma_release_pma_reset(ep);
		ret = phy_power_on(ep->phy);
		if (ret)
			return ret;
		phy_calibrate(ep->phy);

		exynos_zuma_controller_reset_pulse(ep);
		exynos_zuma_config_elbi(ep);

		gpiod_set_value_cansleep(ep->perst_gpio, 1);	/* deassert PERST */
		usleep_range(PCIE_PERST_DELAY_US, PCIE_PERST_DELAY_US + 2000);

		dw_pcie_setup_rc(&pci->pp);

		/*
		 * Restore the integrated-MSI receiver the controller reset zeroed
		 * (link_down snapshot); setup_rc does not re-arm the RC's own
		 * enable/mask/target, so the CP's post-relink MSI would otherwise
		 * be dropped.
		 */
		if (ep->msi_saved) {
			dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO,
					   ep->saved_msi_addr_lo);
			dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI,
					   ep->saved_msi_addr_hi);
			dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_ENABLE,
					   ep->saved_msi_enable);
			dw_pcie_writel_dbi(pci, PCIE_MSI_INTR0_MASK,
					   ep->saved_msi_mask);
		}

		/* setup_rc reset the RC's integrated-MSI summary routing; re-arm it
		 * so the CP's post-link MSI (INIT_START etc.) still reaches us. */
		if (IS_ENABLED(CONFIG_PCI_MSI)) {
			u32 v = exynos_pcie_readl(pci->elbi_base,
						  PCIE_ZUMA_IRQ2_EN);

			exynos_pcie_writel(pci->elbi_base, v | PCIE_ZUMA_IRQ2_MSI,
					   PCIE_ZUMA_IRQ2_EN);
		}

		/* Gen3 EQ off: this PHY trains Gen3 without EQ phases 2/3. */
		dw_pcie_dbi_ro_wr_en(pci);
		dw_pcie_writel_dbi(pci, PCIE_GEN3_RELATED, GEN3_EQ_OFF);
		dw_pcie_dbi_ro_wr_dis(pci);

		exynos_pcie_writel(pci->elbi_base, LTSSM_ENABLE,
				   PCIE_ZUMA_APP_LTSSM_ENABLE);
		usleep_range(1000, 1500);

		ret = exynos_zuma_modem_wait_link(ep);
		if (!ret) {
			u8 exp = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
			u16 lnksta;

			/*
			 * Let the directed speed change settle, then confirm
			 * Gen3: the post-PBL CP gates its IPC start on a Gen3 x2
			 * link.  A retrain out of the orderly-L2 park can train
			 * but stick below Gen3, so redo the full re-config
			 * (downstream establish_link retries the whole sequence).
			 */
			usleep_range(2800, 3000);
			lnksta = dw_pcie_readw_dbi(pci, exp + PCI_EXP_LNKSTA);
			if ((lnksta & PCI_EXP_LNKSTA_CLS) !=
			    PCI_EXP_LNKSTA_CLS_8_0GB &&
			    try < PCIE_MODEM_RELINK_RETRIES - 1) {
				dev_warn(pci->dev,
					 "modem relink below Gen3 (LNKSTA %#x), retrying\n",
					 lnksta);
				ret = -EAGAIN;
				continue;
			}
			dev_info(pci->dev, "modem link up (LNKSTA %#x)\n", lnksta);
			ret = 0;
			break;
		}
		dev_warn(pci->dev,
			 "modem relink attempt %d/%d failed (%d, ltssm %#x)\n",
			 try + 1, PCIE_MODEM_RELINK_RETRIES, ret,
			 exynos_pcie_readl(pci->elbi_base, PCIE_ZUMA_RDLH_LINKUP) &
			 LTSSM_STATE_MASK);
	}

	if (ret) {
		/*
		 * Every attempt failed: stop holding the refclk, park the PHY
		 * clock on the safe OSC so the RC stays touchable.
		 */
		exynos_pcie_phy_keep_refclk(ep->phy, false);
		exynos_pcie_phy_safe_clk(ep->phy, true);
		return ret;
	}

	/* Link back: drop the refclk hold and the transfer-pending fence. */
	exynos_pcie_phy_keep_refclk(ep->phy, false);
	exynos_pcie_writel(pci->elbi_base, 0, PCIE_ZUMA_APP_XFER_PENDING);
	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_link_up);

/*
 * Nudge AP2CP_WAKEUP so a parked CP raises CP2AP_WAKEUP and the modem driver's
 * wakeup IRQ can relink (downstream s5100_try_gpio_cp_wakeup(), the AP-initiated
 * half of pcie_send_ap2cp_irq()).  Only the GPIO edge is driven here -- the
 * actual relink runs from zumapro_pcie_modem_link_up() once the CP answers --
 * so this is a cheap, non-blocking poke the modem send path can issue while the
 * link is parked.  Uses the non-sleeping accessor: the send path reaches this
 * from the MSI hard-IRQ handler and cp_wakeup is a memory-mapped SoC GPIO.
 */
int zumapro_pcie_modem_wake(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);

	if (!ep)
		return -ENODEV;
	gpiod_set_value(ep->cp_wakeup, 1);
	dev_info(ep->pci.dev, "AP2CP_WAKEUP nudged, reads back %d\n",
		 gpiod_get_value(ep->cp_wakeup));
	return 0;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_wake);

/*
 * Warm-reset the CP back into its boot ROM and wait for its endpoint to
 * re-link, so the modem driver can (re-)run its boot sequence from scratch.
 *
 * At cold boot the controller probe already reset the CP (exynos_cp_power_warm_reset()
 * before link training).  But on a bare modem-module reload (rmmod/insmod with no
 * reboot) nothing else resets it, so BL1 would land on a still-running MAIN and
 * boot_stage never advances.  The modem driver calls this at the top of its probe
 * -- after it has found the (already-enumerated) endpoint but before it programs
 * MSI/BAR0 -- so both the cold and the reload path continue against a fresh boot
 * ROM.  cp_pwr stays on (MAIN survives in the CP's self-powered DRAM), only the
 * baseband is pulsed, then we wait for the Gen1 boot-ROM link to re-train; the
 * caller reprograms the endpoint's config (MSI target, doorbell BAR) afterwards.
 */
/*
 * Report the CP link state so the modem driver can decide between ADOPTING a
 * live MAIN (Gen3 endpoint after a bare module reload) and warm-resetting into
 * the boot ROM.  Returns the PCI_EXP_LNKSTA_CLS speed (1 = Gen1 boot ROM,
 * 3 = Gen3 running MAIN) or -ENOLINK when the link is parked or down.
 */
int zumapro_pcie_modem_link_speed(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);
	u32 ltssm;
	u8 exp;

	if (!ep)
		return -ENODEV;
	if (ep->cp_phy_off)
		return -ENOLINK;
	ltssm = exynos_pcie_readl(ep->pci.elbi_base, PCIE_ZUMA_RDLH_LINKUP) &
		LTSSM_STATE_MASK;
	if (ltssm < LTSSM_STATE_RCVRY_LOCK || ltssm > LTSSM_STATE_L1_IDLE)
		return -ENOLINK;
	exp = dw_pcie_find_capability(&ep->pci, PCI_CAP_ID_EXP);
	return dw_pcie_readw_dbi(&ep->pci, exp + PCI_EXP_LNKSTA) &
	       PCI_EXP_LNKSTA_CLS;
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_link_speed);

/*
 * FULL CP cold power cycle -- a faithful port of the downstream power_on_cp()'s
 * gpio_power_offon_cp() (non-WRESET_WA).  Unlike the warm reset
 * (exynos_cp_power_warm_reset(), a CP_WRST pulse that leaves CP_PWR asserted so a
 * running MAIN survives in self-refresh DRAM), this genuinely powers the CP
 * down (drops CP_PWR and PM_WRST, resetting the CP PMIC) and back up, so a
 * crashed / self-reset MAIN is guaranteed back in its boot ROM.  This is what
 * the vendor runs to recover a CP after the cp_active crash indication; a
 * warm reset alone cannot stop a live MAIN, which is why a fresh download then
 * lands on it and wedges at INIT_START.
 *
 * The S5910 DCXO clock buffer is only ever RE-ARMED here (its on_seq, applied
 * while the CP is held in NRESET), never shut down -- the downstream normal
 * power path does the same; the shutdown sequence belongs to power_shutdown.
 * Delays and ordering are byte-for-byte the vendor's (mif_gpio_set_value's
 * post-set msleep).
 */
int zumapro_pcie_modem_power_cycle(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);

	if (!ep || !ep->cp_power)
		return -ENODEV;

	/*
	 * Isolate the RC before CP_PWR drops: with the endpoint about to vanish,
	 * an in-flight RC access would take an SError.  link_down asserts PERST
	 * and powers the PHY off (sets cp_phy_off), so link_up() below does the
	 * full re-init retrain.  Skip if the previous instance already parked it.
	 */
	if (!ep->cp_phy_off)
		zumapro_pcie_modem_link_down(rc_dev);

	/*
	 * Pulse AP2CP_WAKEUP (the RC-owned link signal) then delegate the rail
	 * cold cycle -- power off with the S5910 re-armed while held in reset,
	 * then back on -- to the CP power sequencer.
	 */
	gpiod_set_value_cansleep(ep->cp_wakeup, 1);
	msleep(10);
	gpiod_set_value_cansleep(ep->cp_wakeup, 0);
	exynos_cp_power_cold_cycle(ep->cp_power);

	return zumapro_pcie_modem_link_up(rc_dev);
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_power_cycle);

int zumapro_pcie_modem_reset(struct device *rc_dev)
{
	struct exynos_pcie *ep = zumapro_pcie_from_dev(rc_dev);

	if (!ep || !ep->cp_power)
		return -ENODEV;

	exynos_cp_power_warm_reset(ep->cp_power);

	/*
	 * If the previous modem instance parked the link (link_down: PHY off,
	 * LTSSM disabled), a plain wait can never train -- run the full
	 * re-init relink.  Otherwise the LTSSM is still armed and the
	 * freshly-reset boot ROM re-trains by itself; fall back to the full
	 * relink if it does not.
	 */
	if (!ep->cp_phy_off) {
		if (!dw_pcie_wait_for_link(&ep->pci))
			return 0;
		dev_warn(ep->pci.dev,
			 "post-reset light retrain failed; full relink\n");
	}
	return zumapro_pcie_modem_link_up(rc_dev);
}
EXPORT_SYMBOL_GPL(zumapro_pcie_modem_reset);

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
