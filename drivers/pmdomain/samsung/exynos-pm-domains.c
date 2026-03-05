// SPDX-License-Identifier: GPL-2.0
//
// Exynos Generic power domain support.
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
//
// Implementation of Exynos specific power domain control which is used in
// conjunction with runtime-pm. Support for both device-tree and non-device-tree
// based power domain support is included.

#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/arm-smccc.h>

/*
 * Secure PD transition SMC interface used by newer Exynos/Google SoCs.
 * "need_smc" DT property carries the secure transition ID.
 */
#define EXYNOS_SMC_PREPARE_PD_ONOFF	0x82000410
#define EXYNOS_GET_IN_PD_DOWN		0
#define EXYNOS_WAKEUP_PD_DOWN		1
#define EXYNOS_RUNTIME_PM_TZPC_GROUP	2

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 secure_transition_id;
	bool needs_secure_transition;
};

static int exynos_pd_secure_prepare(struct exynos_pm_domain *pd, bool power_on)
{
	struct arm_smccc_res res;
	unsigned long mode = power_on ? EXYNOS_WAKEUP_PD_DOWN :
				       EXYNOS_GET_IN_PD_DOWN;

	arm_smccc_smc(EXYNOS_SMC_PREPARE_PD_ONOFF, mode,
		      pd->secure_transition_id, EXYNOS_RUNTIME_PM_TZPC_GROUP,
		      0, 0, 0, 0, &res);

	return (int)res.a0;
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	const char *op;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	if (pd->needs_secure_transition) {
		int ret = exynos_pd_secure_prepare(pd, power_on);
		u32 before = readl_relaxed(base + 0x4) & pd->local_pwr_cfg;
		u32 after;

		if (ret) {
			pr_err("exynos-pd: %s: secure prepare %s failed: %d\n",
			       domain->name, power_on ? "on" : "off", ret);
			return ret;
		}

		/*
		 * Secure world applies the transition; poll status to confirm
		 * whether the local power state actually changed.
		 */
		timeout = 10;
		pwr = power_on ? pd->local_pwr_cfg : 0;
		after = readl_relaxed(base + 0x4) & pd->local_pwr_cfg;
		while (after != pwr && timeout--) {
			cpu_relax();
			usleep_range(80, 100);
			after = readl_relaxed(base + 0x4) & pd->local_pwr_cfg;
		}

		pr_info("exynos-pd: %s: secure power %s status %#x->%#x (%s)\n",
			domain->name, power_on ? "on" : "off",
			before, after, after == pwr ? "ok" : "timeout");
		return 0;
	}

	op = power_on ? "on" : "off";
	pr_info("exynos-pd: %s: power %s\n", domain->name, op);

	pwr = power_on ? pd->local_pwr_cfg : 0;
	writel_relaxed(pwr, base);

	/* Wait max 1ms */
	timeout = 10;

	while ((readl_relaxed(base + 0x4) & pd->local_pwr_cfg) != pwr) {
		if (!timeout) {
			pr_err("Power domain %s %s failed\n", domain->name, op);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	return 0;
}

static int exynos_pd_power_on(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, true);
}

static int exynos_pd_power_off(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, false);
}

static const struct exynos_pm_domain_config exynos4210_cfg = {
	.local_pwr_cfg		= 0x7,
};

static const struct exynos_pm_domain_config exynos5433_cfg = {
	.local_pwr_cfg		= 0xf,
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	},
	{ },
};

static const char *exynos_get_domain_name(struct device *dev,
					  struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = kbasename(node->full_name);
	return devm_kstrdup_const(dev, name, GFP_KERNEL);
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *parent_np;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	int on, ret;
	u32 secure_id;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(dev, np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = of_iomap(np, 0);
	if (!pd->base)
		return -ENODEV;

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	ret = of_property_read_u32(np, "need_smc", &secure_id);
	if (!ret) {
		pd->needs_secure_transition = true;
		pd->secure_transition_id = secure_id;
	}
	parent_np = of_parse_phandle(np, "power-domains", 0);
	if (parent_np) {
		if (!pd->needs_secure_transition &&
		    !of_property_read_u32(parent_np, "need_smc", &secure_id)) {
			pd->needs_secure_transition = true;
			pd->secure_transition_id = secure_id;
		}
		of_node_put(parent_np);
	}

	/*
	 * Some Samsung platforms with bootloaders turning on the splash-screen
	 * and handing it over to the kernel, requires the power-domains to be
	 * reset during boot.
	 */
	if (IS_ENABLED(CONFIG_ARM) &&
	    of_device_is_compatible(np, "samsung,exynos4210-pd"))
		exynos_pd_power_off(&pd->pd);

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;
	if (pd->needs_secure_transition)
		on = pd->local_pwr_cfg;

	pm_genpd_init(&pd->pd, NULL, !on);
	ret = of_genpd_add_provider_simple(np, &pd->pd);

	if (ret == 0 && of_parse_phandle_with_args(np, "power-domains",
				      "#power-domain-cells", 0, &parent) == 0) {
		child.np = np;
		child.args_count = 0;

		if (of_genpd_add_subdomain(&parent, &child))
			pr_warn("%pOF failed to add subdomain: %pOF\n",
				parent.np, child.np);
		else
			pr_info("%pOF has as child subdomain: %pOF.\n",
				parent.np, child.np);
	}

	pm_runtime_enable(dev);
	return ret;
}

static struct platform_driver exynos_pd_driver = {
	.probe	= exynos_pd_probe,
	.driver	= {
		.name		= "exynos-pd",
		.of_match_table	= exynos_pm_domain_of_match,
		.suppress_bind_attrs = true,
	}
};

static __init int exynos4_pm_init_power_domain(void)
{
	return platform_driver_register(&exynos_pd_driver);
}
core_initcall(exynos4_pm_init_power_domain);
