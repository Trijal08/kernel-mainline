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
#include <linux/regmap.h>
#include <linux/arm-smccc.h>
#include <linux/soc/samsung/exynos-pmu.h>

/*
 * Secure PD transition SMC interface used by newer Exynos/Google SoCs.
 * "need_smc" DT property carries the secure transition ID.
 */
#define EXYNOS_SMC_PREPARE_PD_ONOFF	0x82000410
#define EXYNOS_GET_IN_PD_DOWN		0
#define EXYNOS_WAKEUP_PD_DOWN		1
#define EXYNOS_RUNTIME_PM_TZPC_GROUP	2

/* Offset of the STATUS register relative to the CONFIGURATION register. */
#define EXYNOS_PD_STATUS_OFFSET		0x4

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	/*
	 * Tensor (gs101 and derivatives) keep the local-power CONFIGURATION
	 * registers inside the EL3-protected PMU_ALIVE block: they can be read
	 * over MMIO but only written through the secure PMU regmap. When set,
	 * drive the domain via that regmap instead of a raw MMIO write.
	 */
	bool pmu_smc;
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
	/* Tensor secure-PMU access (pmu_smc domains). */
	struct regmap *pmureg;
	u32 pmu_offset;
	bool pmu_smc;
};

static int exynos_pd_secure_prepare(struct exynos_pm_domain *pd, bool power_on)
{
	struct arm_smccc_res res;
	unsigned long mode = power_on ? EXYNOS_WAKEUP_PD_DOWN :
				       EXYNOS_GET_IN_PD_DOWN;

	arm_smccc_smc(EXYNOS_SMC_PREPARE_PD_ONOFF, mode,
		      pd->secure_transition_id, EXYNOS_RUNTIME_PM_TZPC_GROUP,
		      0, 0, 0, 0, &res);

	/* PDDBG: expose the raw SMC result so we can tell if EL3 even honoured it */
	pr_info("PDDBG: %s: SMC(0x%08x, mode=%lu, id=%#x, grp=%u) -> a0=%#lx a1=%#lx\n",
		pd->pd.name, EXYNOS_SMC_PREPARE_PD_ONOFF, mode,
		pd->secure_transition_id, EXYNOS_RUNTIME_PM_TZPC_GROUP,
		res.a0, res.a1);

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

	if (pd->pmu_smc) {
		u32 val, before, cfg;
		int polls = 0;

		/*
		 * Power-off is a no-op for now. Tensor's PMU power-down is a
		 * handshake that only completes once the block's clocks are
		 * gated and its bus master is idle; the coordinated
		 * clock/bus-idle sequencing the vendor cal framework performs is
		 * not wired up here yet. Writing the config bit with the block
		 * still active stalls the sequencer and wedges the APM (which
		 * also serves the ACPM PMIC channel), so leave the domain in the
		 * state the bootloader left it (on) instead of half-tearing it
		 * down.
		 */
		if (!power_on)
			return 0;

		pwr = pd->local_pwr_cfg;

		regmap_read(pd->pmureg, pd->pmu_offset + EXYNOS_PD_STATUS_OFFSET,
			    &before);

		/*
		 * The CONFIGURATION register lives in the protected PMU block,
		 * so the write is routed through the secure PMU regmap (SMC).
		 * Writing the local-power bit kicks the PMU sequencer; the
		 * matching STATUS bit reflects the result and is MMIO-readable.
		 *
		 * Use an explicit read/modify/plain-write instead of
		 * regmap_update_bits(): on Tensor the PMU regmap turns
		 * update_bits into a hardware atomic set/clear that targets an
		 * aliased offset, which does not drive the local-power
		 * sequencer. The vendor pmucal writes the full value, so mirror
		 * that.
		 */
		regmap_read(pd->pmureg, pd->pmu_offset, &cfg);
		cfg = (cfg & ~pd->local_pwr_cfg) | (pwr & pd->local_pwr_cfg);
		regmap_write(pd->pmureg, pd->pmu_offset, cfg);

		timeout = 10;
		regmap_read(pd->pmureg, pd->pmu_offset + EXYNOS_PD_STATUS_OFFSET,
			    &val);
		while ((val & pd->local_pwr_cfg) != pwr && timeout--) {
			usleep_range(80, 100);
			regmap_read(pd->pmureg,
				    pd->pmu_offset + EXYNOS_PD_STATUS_OFFSET,
				    &val);
			polls++;
		}

		/* PDDBG: did the secure config write actually move the status bit? */
		pr_info("PDDBG: %s: pmu_smc power %s off=%#x status %#x->%#x polls=%d (%s)\n",
			domain->name, power_on ? "on" : "off", pd->pmu_offset,
			before & pd->local_pwr_cfg, val & pd->local_pwr_cfg, polls,
			(val & pd->local_pwr_cfg) == pwr ? "ok" : "TIMEOUT");

		if ((val & pd->local_pwr_cfg) != pwr) {
			pr_err("exynos-pd: %s: power %s failed\n",
			       domain->name, power_on ? "on" : "off");
			return -ETIMEDOUT;
		}

		/*
		 * Powering the block on is not enough: the block's bus master is
		 * fenced off from DRAM by the DTZPC until the secure world reapplies
		 * its runtime-PM protection group. The vendor cal path runs this
		 * DTZPC restore SMC on every domain power-on alongside the PMU
		 * sequence; without it, a master like the G3D GPU faults on its very
		 * first DRAM access (its pagetable walk). Mirror that here.
		 */
		if (pd->needs_secure_transition) {
			int sret = exynos_pd_secure_prepare(pd, power_on);

			if (sret)
				pr_err("exynos-pd: %s: DTZPC restore failed: %d\n",
				       domain->name, sret);
		}
		return 0;
	}

	if (pd->needs_secure_transition) {
		u32 raw_before = readl_relaxed(base + 0x4);
		u32 before = raw_before & pd->local_pwr_cfg;
		u32 raw_after, after;
		int polls = 0;
		int ret;

		/* PDDBG: prove genpd actually asked us to toggle this domain */
		pr_info("PDDBG: %s: power_%s ENTER, status@%p+4 raw=%#x masked=%#x cfg=%#x\n",
			domain->name, power_on ? "on" : "off",
			base, raw_before, before, pd->local_pwr_cfg);

		ret = exynos_pd_secure_prepare(pd, power_on);
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
		raw_after = readl_relaxed(base + 0x4);
		after = raw_after & pd->local_pwr_cfg;
		while (after != pwr && timeout--) {
			cpu_relax();
			usleep_range(80, 100);
			raw_after = readl_relaxed(base + 0x4);
			after = raw_after & pd->local_pwr_cfg;
			polls++;
		}

		/* PDDBG: did the PMU status register actually move? */
		pr_info("PDDBG: %s: secure power %s raw %#x->%#x masked %#x->%#x polls=%d (%s)\n",
			domain->name, power_on ? "on" : "off",
			raw_before, raw_after, before, after, polls,
			after == pwr ? "ok" : "TIMEOUT-status-never-changed");
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

/*
 * gs101 and its derivatives (Tensor) gate each local power domain with a
 * single bit in a CONFIGURATION register that only the secure world may write.
 * Reads (the paired STATUS register) go over MMIO, writes over the secure PMU
 * regmap.
 */
static const struct exynos_pm_domain_config gs101_cfg = {
	.local_pwr_cfg		= 0x1,
	.pmu_smc		= true,
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	}, {
		.compatible = "google,gs101-pd",
		.data = &gs101_cfg,
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
	pd->pmu_smc = pm_domain_cfg->pmu_smc;

	if (pd->pmu_smc) {
		struct resource res;

		/*
		 * The CONFIGURATION/STATUS registers sit inside the PMU_ALIVE
		 * block. Grab the SMC-backed PMU regmap for writes; the offset
		 * within that block is the low bits of the domain's reg address.
		 */
		pd->pmureg = exynos_get_pmu_regmap();
		if (IS_ERR_OR_NULL(pd->pmureg)) {
			iounmap(pd->base);
			return dev_err_probe(dev,
					     pd->pmureg ? PTR_ERR(pd->pmureg) :
					     -EPROBE_DEFER,
					     "no PMU regmap\n");
		}
		if (of_address_to_resource(np, 0, &res)) {
			iounmap(pd->base);
			return -EINVAL;
		}
		pd->pmu_offset = res.start & 0xffff;
	}

	ret = of_property_read_u32(np, "need_smc", &secure_id);
	if (!ret) {
		/*
		 * "need_smc" is the DTZPC handle the secure world uses to (re)apply
		 * the domain's master-access protection on power transitions. This
		 * is orthogonal to how the block itself is powered: a pmu_smc domain
		 * (Tensor) still needs the DTZPC restore, so do not gate it on
		 * !pmu_smc - the vendor cal path runs both.
		 */
		pd->needs_secure_transition = true;
		pd->secure_transition_id = secure_id;
	}
	parent_np = of_parse_phandle(np, "power-domains", 0);
	if (parent_np) {
		if (!pd->pmu_smc && !pd->needs_secure_transition &&
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

	/* PDDBG: what does the PMU actually report before we override it? */
	pr_info("PDDBG: %s: probe secure=%d raw_status=%#x hw_on=%#x cfg=%#x\n",
		pd->pd.name, pd->needs_secure_transition,
		readl_relaxed(pd->base + 0x4), on, pd->local_pwr_cfg);

	if (pd->needs_secure_transition)
		on = pd->local_pwr_cfg;

	/*
	 * PDDBG: genpd is told is_off=%d. If we assert "on" for a secure domain
	 * that HW says is off, genpd will never call power_on and the block stays
	 * dark while genpd believes it is lit.
	 */
	pr_info("PDDBG: %s: pm_genpd_init(is_off=%d)%s\n", pd->pd.name, !on,
		(pd->needs_secure_transition && !(readl_relaxed(pd->base + 0x4) &
		 pd->local_pwr_cfg)) ? " [OVERRIDE: HW says OFF!]" : "");

	pm_genpd_init(&pd->pd, NULL, !on);

	/*
	 * A domain the bootloader left powered on is registered with is_off=0,
	 * so genpd never invokes our power_on callback - and the DTZPC master
	 * grant that lives in that callback would never run. Apply it once here
	 * so masters behind this domain (e.g. the G3D GPU) can reach DRAM from
	 * first use rather than bus-faulting on their first transaction.
	 */
	if (pd->needs_secure_transition && on) {
		int sret = exynos_pd_secure_prepare(pd, true);

		pr_info("PDDBG: %s: probe-time DTZPC restore -> %d\n",
			pd->pd.name, sret);
	}

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
