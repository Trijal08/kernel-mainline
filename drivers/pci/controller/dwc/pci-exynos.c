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
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
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
#define PCIE_SW_WAKE			0x018
#define PCIE_BUS_EN			BIT(1)
#define PCIE_CORE_RESET			0x01c
#define PCIE_CORE_RESET_ENABLE		BIT(0)
#define PCIE_STICKY_RESET		0x020
#define PCIE_NONSTICKY_RESET		0x024
#define PCIE_APP_INIT_RESET		0x028
#define PCIE_APP_LTSSM_ENABLE		0x02c
#define PCIE_APP_XFER_PENDING		0x074
#define PCIE_ELBI_RDLH_LINKUP		0x074
#define PCIE_ELBI_LTSSM_STATE_MASK	0x3f
#define PCIE_ELBI_XMLH_LINKUP		BIT(4)
#define PCIE_ELBI_LTSSM_ENABLE		0x1
#define PCIE_ELBI_SLV_AWMISC		0x11c
#define PCIE_ELBI_SLV_ARMISC		0x120
#define PCIE_ELBI_SLV_DBI_ENABLE	BIT(21)
#define PCIE_LINKDOWN_RST_CTRL_SEL	0x3a0
#define PCIE_LINKDOWN_RST_MANUAL	BIT(1)
#define PCIE_QCH_SEL			0x3a8
#define CLOCK_GATING_PMU_MASK		(0xf << 8)
#define CLOCK_GATING_APB_MASK		(0xf << 4)
#define CLOCK_GATING_AXI_MASK		(0xf << 0)
#define PCIE_APP_REQ_EXIT_L1_MODE	0x3bc
#define APP_REQ_EXIT_L1_MODE		BIT(0)
#define L1_REQ_NAK_CONTROL_MASTER	BIT(4)
#define PCIE_MSTR_PEND_SEL_NAK		0x474
#define NACK_ENABLE			0x1
#define PCIE_DBI_L1_EXIT_DISABLE	0x1078
#define DBI_L1_EXIT_DISABLE		0x1
#define ZUMA_WLAN_PWR_DELAY_US		100000
#define ZUMA_WLAN_OFF_DELAY_US		20000
#define EP_BCM_WIFI			0x1
#define ZUMA_LINKUP_POLL_MAX		5000
#define ZUMA_LINKUP_RETRY_MAX		3
#define ZUMA_PMU_LINKUP_REQ_MASK	BIT(10)
#define ZUMA_UDBG_LINK_CTRL		0xC800

struct exynos_pcie {
	struct dw_pcie			pci;
	struct clk_bulk_data		*clks;
	struct phy			*phy;
	struct regulator_bulk_data	supplies[2];
	struct regmap			*pmureg;
	void __iomem			*udbg_base;
	struct gpio_desc		*perst_gpio;
	u32				perst_delay_us;
	u32				ip_ver;
	u32				pmu_offset;
	u32				ch_num;
	u32				ep_device_type;
	int				wlan_gpio;
	bool				is_zuma;
};

static void exynos_pcie_writel(void __iomem *base, u32 val, u32 reg)
{
	writel(val, base + reg);
}

static u32 exynos_pcie_readl(void __iomem *base, u32 reg)
{
	return readl(base + reg);
}

static void exynos_pcie_zuma_pre_ltssm(struct exynos_pcie *ep)
{
	struct dw_pcie *pci = &ep->pci;
	u32 val;

	if (ep->pmureg)
		regmap_update_bits(ep->pmureg, ep->pmu_offset,
				   ZUMA_PMU_LINKUP_REQ_MASK,
				   ZUMA_PMU_LINKUP_REQ_MASK);

	if (ep->udbg_base) {
		if (ep->ch_num == 0) {
			val = readl(ep->udbg_base + ZUMA_UDBG_LINK_CTRL);
			val &= ~(0x3 << 5);
			writel(val, ep->udbg_base + ZUMA_UDBG_LINK_CTRL);
		} else {
			writel(0x421, ep->udbg_base + ZUMA_UDBG_LINK_CTRL);
		}
	}

	val = exynos_pcie_readl(pci->elbi_base, PCIE_APP_REQ_EXIT_L1_MODE);
	val |= APP_REQ_EXIT_L1_MODE | L1_REQ_NAK_CONTROL_MASTER;
	exynos_pcie_writel(pci->elbi_base, val, PCIE_APP_REQ_EXIT_L1_MODE);

	exynos_pcie_writel(pci->elbi_base, PCIE_LINKDOWN_RST_MANUAL,
			   PCIE_LINKDOWN_RST_CTRL_SEL);
	exynos_pcie_writel(pci->elbi_base, 0x1, PCIE_APP_XFER_PENDING);

	val = exynos_pcie_readl(pci->elbi_base, PCIE_QCH_SEL);
	if (ep->ip_ver >= 0x889500)
		val &= ~(CLOCK_GATING_PMU_MASK |
			 CLOCK_GATING_APB_MASK |
			 CLOCK_GATING_AXI_MASK);
	exynos_pcie_writel(pci->elbi_base, val, PCIE_QCH_SEL);

	exynos_pcie_writel(pci->elbi_base, NACK_ENABLE, PCIE_MSTR_PEND_SEL_NAK);
	exynos_pcie_writel(pci->elbi_base, DBI_L1_EXIT_DISABLE,
			   PCIE_DBI_L1_EXIT_DISABLE);
}

static void exynos_pcie_zuma_post_link(struct exynos_pcie *ep)
{
	exynos_pcie_writel(ep->pci.elbi_base, 0x0, PCIE_APP_XFER_PENDING);
}

static void exynos_pcie_zuma_prepare_retry(struct exynos_pcie *ep)
{
	int ret;

	ret = phy_reset(ep->phy);
	if (ret)
		dev_dbg(ep->pci.dev, "zuma phy_reset failed: %d\n", ret);
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
	struct exynos_pcie *ep = to_exynos_pcie(pci);
	u32 val;
	int attempt, poll;

	if (ep->is_zuma)
		exynos_pcie_zuma_pre_ltssm(ep);

	if (ep->is_zuma && ep->ep_device_type == EP_BCM_WIFI &&
	    gpio_is_valid(ep->wlan_gpio)) {
		gpio_set_value(ep->wlan_gpio, 0);
		usleep_range(ZUMA_WLAN_OFF_DELAY_US, ZUMA_WLAN_OFF_DELAY_US + 5000);
		gpio_set_value(ep->wlan_gpio, 1);
		usleep_range(ZUMA_WLAN_PWR_DELAY_US, ZUMA_WLAN_PWR_DELAY_US + 10000);
		dev_info(pci->dev, "zuma wlan power-cycled, gpio=%d state=%d\n",
			 ep->wlan_gpio, gpio_get_value(ep->wlan_gpio));
	}

	if (ep->is_zuma && ep->ep_device_type == EP_BCM_WIFI) {
		for (attempt = 0; attempt < ZUMA_LINKUP_RETRY_MAX; attempt++) {
			if (attempt > 0) {
				exynos_pcie_writel(pci->elbi_base, 0,
						   PCIE_APP_LTSSM_ENABLE);
				if (ep->perst_gpio)
					gpiod_set_raw_value_cansleep(ep->perst_gpio, 0);
				usleep_range(2000, 3000);
			}

			exynos_pcie_zuma_prepare_retry(ep);
			exynos_pcie_zuma_pre_ltssm(ep);

			if (ep->perst_gpio) {
				gpiod_set_raw_value_cansleep(ep->perst_gpio, 1);
				usleep_range(ep->perst_delay_us,
					     ep->perst_delay_us + 2000);
			}

			val = exynos_pcie_readl(pci->elbi_base, PCIE_SW_WAKE);
			val &= ~PCIE_BUS_EN;
			exynos_pcie_writel(pci->elbi_base, val, PCIE_SW_WAKE);

			exynos_pcie_writel(pci->elbi_base, PCIE_ELBI_LTSSM_ENABLE,
					   PCIE_APP_LTSSM_ENABLE);

			for (poll = 0; poll < ZUMA_LINKUP_POLL_MAX; poll++) {
				val = exynos_pcie_readl(pci->elbi_base,
							PCIE_ELBI_RDLH_LINKUP);
				if (val & PCIE_ELBI_XMLH_LINKUP)
					return 0;
				usleep_range(10, 12);
			}

			dev_info(pci->dev,
				 "zuma link retry %d failed (ltssm=0x%x, perst=%d, wlan=%d)\n",
				 attempt + 1,
				 exynos_pcie_readl(pci->elbi_base,
						   PCIE_ELBI_RDLH_LINKUP) &
				 PCIE_ELBI_LTSSM_STATE_MASK,
				 ep->perst_gpio ? gpiod_get_raw_value(ep->perst_gpio) : -1,
				 gpio_is_valid(ep->wlan_gpio) ?
				 gpio_get_value(ep->wlan_gpio) : -1);
		}

		dev_warn(pci->dev, "zuma link did not come up after %d attempts\n",
			 ZUMA_LINKUP_RETRY_MAX);
		return 0;
	}

	val = exynos_pcie_readl(pci->elbi_base, PCIE_SW_WAKE);
	val &= ~PCIE_BUS_EN;
	exynos_pcie_writel(pci->elbi_base, val, PCIE_SW_WAKE);

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

	pp->ops = &exynos_pcie_host_ops;
	pp->msi_irq[0] = -ENODEV;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	if (ep->is_zuma)
		exynos_pcie_zuma_post_link(ep);

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.read_dbi = exynos_pcie_read_dbi,
	.write_dbi = exynos_pcie_write_dbi,
	.link_up = exynos_pcie_link_up,
	.start_link = exynos_pcie_start_link,
};

static int exynos_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_pcie *ep;
	struct device_node *np = dev->of_node;
	u32 perst_delay_us;
	int ret;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pci.dev = dev;
	ep->pci.ops = &dw_pcie_ops;
	ep->is_zuma = of_device_is_compatible(np, "google,zuma-pcie");
	ep->perst_delay_us = 20000;
	ep->pmu_offset = 0;
	ep->ch_num = 0;
	ep->ep_device_type = 0;
	ep->wlan_gpio = -EINVAL;
	ep->pmureg = NULL;
	ep->udbg_base = NULL;
	of_property_read_u32(np, "ip-ver", &ep->ip_ver);
	of_property_read_u32(np, "pmu-offset", &ep->pmu_offset);
	of_property_read_u32(np, "ch-num", &ep->ch_num);
	of_property_read_u32(np, "ep-device-type", &ep->ep_device_type);

	if (ep->is_zuma &&
	    !of_property_read_u32(np, "perst-delay-us", &perst_delay_us))
		ep->perst_delay_us = perst_delay_us;

	if (ep->is_zuma) {
		struct resource *res;

		ep->perst_gpio = devm_gpiod_get_optional(dev, NULL, GPIOD_ASIS);
		if (IS_ERR(ep->perst_gpio))
			return dev_err_probe(dev, PTR_ERR(ep->perst_gpio),
					     "failed to get PERST gpio\n");
		if (ep->perst_gpio) {
			ret = gpiod_direction_output_raw(ep->perst_gpio, 0);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to set PERST gpio direction\n");
		}

		ep->pmureg = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
		if (IS_ERR(ep->pmureg))
			return dev_err_probe(dev, PTR_ERR(ep->pmureg),
					     "failed to lookup pmu syscon\n");

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "udbg");
		if (res) {
			ep->udbg_base = devm_ioremap_resource(dev, res);
			if (IS_ERR(ep->udbg_base))
				return PTR_ERR(ep->udbg_base);
		}

		if (ep->ep_device_type == EP_BCM_WIFI) {
			ep->wlan_gpio = of_get_named_gpio(np, "pcie,wlan-gpio", 0);
			if (ep->wlan_gpio == -EPROBE_DEFER)
				return dev_err_probe(dev, -EPROBE_DEFER,
						     "wlan gpio provider not ready\n");
			if (gpio_is_valid(ep->wlan_gpio)) {
				ret = devm_gpio_request_one(dev, ep->wlan_gpio,
							    GPIOF_OUT_INIT_LOW, "pcie_wlan");
				if (ret)
					return dev_err_probe(dev, ret,
							     "failed to request wlan gpio\n");
			}
		}

		dev_info(dev,
			 "zuma mode enabled (ch=%u ip-ver=0x%x, ep-device-type=%u, pmu-offset=0x%x, perst-delay-us=%u, wlan-gpio=%d)\n",
			 ep->ch_num, ep->ip_ver, ep->ep_device_type,
			 ep->pmu_offset, ep->perst_delay_us, ep->wlan_gpio);
	}

	ep->phy = devm_of_phy_get(dev, np, NULL);
	if (IS_ERR(ep->phy))
		return PTR_ERR(ep->phy);

	ret = devm_clk_bulk_get_all_enabled(dev, &ep->clks);
	if (ret < 0)
		return ret;

	ep->supplies[0].supply = "vdd18";
	ep->supplies[1].supply = "vdd10";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ep->supplies),
				      ep->supplies);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ep->supplies), ep->supplies);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, ep);

	ret = exynos_add_pcie_port(ep, pdev);
	if (ret < 0)
		goto fail_probe;

	return 0;

fail_probe:
	phy_exit(ep->phy);
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
	regulator_bulk_disable(ARRAY_SIZE(ep->supplies), ep->supplies);
}

static int exynos_pcie_suspend_noirq(struct device *dev)
{
	struct exynos_pcie *ep = dev_get_drvdata(dev);

	exynos_pcie_assert_core_reset(ep);
	phy_power_off(ep->phy);
	phy_exit(ep->phy);
	regulator_bulk_disable(ARRAY_SIZE(ep->supplies), ep->supplies);

	return 0;
}

static int exynos_pcie_resume_noirq(struct device *dev)
{
	struct exynos_pcie *ep = dev_get_drvdata(dev);
	struct dw_pcie *pci = &ep->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ep->supplies), ep->supplies);
	if (ret)
		return ret;

	/* exynos_pcie_host_init controls ep->phy */
	exynos_pcie_host_init(pp);
	dw_pcie_setup_rc(pp);
	exynos_pcie_start_link(pci);
	return dw_pcie_wait_for_link(pci);
}

static const struct dev_pm_ops exynos_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos_pcie_suspend_noirq,
				  exynos_pcie_resume_noirq)
};

static const struct of_device_id exynos_pcie_of_match[] = {
	{ .compatible = "google,zuma-pcie", },
	{ .compatible = "samsung,exynos5433-pcie", },
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
