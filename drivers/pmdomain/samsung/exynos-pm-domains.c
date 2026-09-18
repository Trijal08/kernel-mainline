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

#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

#include "exynos-pm-domains.h"

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	bool secure_pmu;
	/* The vendor's power sequences, one entry per domain, then a sentinel */
	const struct exynos_pd_sequences *sequences;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	phys_addr_t base_addr;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 secure_pwr_id;
	bool secure_pmu;

	/* Only with vendor power sequences */
	void __iomem *windows[EXYNOS_PD_NR_WINDOWS];
	resource_size_t window_size[EXYNOS_PD_NR_WINDOWS];
	const struct exynos_pd_sequences *seq;
	/* One slot per step of seq->save; only the save steps are used */
	u32 *saved;
	bool have_saved;
};

#define EXYNOS_PD_SMC_CMD		0x82000410
#define EXYNOS_PD_SMC_SAVE		0
#define EXYNOS_PD_SMC_RESTORE		1
#define EXYNOS_PD_SMC_TZPC_GROUP	2
#define EXYNOS_PRIV_REG_SMC_CMD		0x82000504
#define EXYNOS_PRIV_REG_WRITE		1

/*
 * A PMU_ALIVE register below offset 0x4000 has a set-bit alias at
 * offset | 0xc000 (and a clear-bit alias at offset | 0x8000), for registers
 * that several masters share.
 */
#define EXYNOS_PMU_ALIVE_OFFSET_MASK	0xffff
#define EXYNOS_PMU_ALIVE_ATOMIC_LIMIT	0x4000
#define EXYNOS_PMU_SET_BITS_ALIAS	0xc000

/* The vendor sequences poll a status for up to 5 ms */
#define EXYNOS_PD_SEQ_TIMEOUT_US	5000

static const char * const exynos_pd_window_names[EXYNOS_PD_NR_WINDOWS] = {
	[EXYNOS_PD_PMU] = "pmu",
	[EXYNOS_PD_CMU] = "cmu",
	[EXYNOS_PD_SYSREG] = "sysreg",
};

static void exynos_pd_secure_control(struct exynos_pm_domain *pd, bool power_on)
{
	struct arm_smccc_res res;

	if (!pd->secure_pwr_id)
		return;

	arm_smccc_smc(EXYNOS_PD_SMC_CMD,
		      power_on ? EXYNOS_PD_SMC_RESTORE : EXYNOS_PD_SMC_SAVE,
		      pd->secure_pwr_id, EXYNOS_PD_SMC_TZPC_GROUP,
		      0, 0, 0, 0, &res);

	if (res.a0)
		pr_warn("Power domain %s secure %s returned %lu\n",
			pd->pd.name, power_on ? "restore" : "save", res.a0);
}

static int exynos_pd_write_pmu_secure(struct exynos_pm_domain *pd,
				      phys_addr_t addr, u32 value)
{
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS_PRIV_REG_SMC_CMD, addr, EXYNOS_PRIV_REG_WRITE,
		      value, 0, 0, 0, 0, &res);

	if (res.a0) {
		pr_err("Power domain %s secure PMU write to %pa returned %lu\n",
		       pd->pd.name, &addr, res.a0);
		return -EIO;
	}

	return 0;
}

static int exynos_pd_write_pmu(struct exynos_pm_domain *pd, u32 offset,
			       u32 value)
{
	if (!pd->secure_pmu) {
		writel_relaxed(value, pd->base + offset);
		return 0;
	}

	return exynos_pd_write_pmu_secure(pd, pd->base_addr + offset, value);
}

static int exynos_pd_set_bits_pmu(struct exynos_pm_domain *pd, u32 offset,
				  u32 value)
{
	phys_addr_t reg = pd->base_addr + offset;
	phys_addr_t alias;

	if (!pd->secure_pmu)
		return -EOPNOTSUPP;

	if ((reg & EXYNOS_PMU_ALIVE_OFFSET_MASK) >= EXYNOS_PMU_ALIVE_ATOMIC_LIMIT)
		return -EINVAL;

	alias = reg | EXYNOS_PMU_SET_BITS_ALIAS;

	return exynos_pd_write_pmu_secure(pd, alias, value);
}

static void __iomem *exynos_pd_step_addr(struct exynos_pm_domain *pd,
					 const struct exynos_pd_step *step)
{
	return pd->windows[step->window] + step->offset;
}

static int exynos_pd_step_write(struct exynos_pm_domain *pd,
				const struct exynos_pd_step *step, u32 value)
{
	void __iomem *addr = exynos_pd_step_addr(pd, step);

	if (step->mask != U32_MAX)
		value = (readl(addr) & ~step->mask) | (value & step->mask);

	if (step->window == EXYNOS_PD_PMU)
		return exynos_pd_write_pmu(pd, step->offset, value);

	writel(value, addr);
	return 0;
}

static int exynos_pd_step_wait(struct exynos_pm_domain *pd,
			       const struct exynos_pd_step *step)
{
	void __iomem *addr = exynos_pd_step_addr(pd, step);
	u32 val;
	int ret;

	ret = readl_poll_timeout(addr, val, (val & step->mask) == step->value,
				 10, EXYNOS_PD_SEQ_TIMEOUT_US);
	if (ret)
		pr_err("Power domain %s: %s +%#x reads %#x, waited for %#x under %#x\n",
		       pd->pd.name, exynos_pd_window_names[step->window],
		       step->offset, val, step->value, step->mask);

	return ret;
}

enum exynos_pd_pass {
	EXYNOS_PD_PASS_RUN,	/* the on and off sequences */
	EXYNOS_PD_PASS_SAVE,	/* the save sequence, before power-off */
	EXYNOS_PD_PASS_RESTORE,	/* the save sequence, after power-on */
};

/*
 * The save pass only reads what the save steps name.  The other passes run
 * the steps in order: a save step then writes back what was read, if
 * anything was, and a skip-if step decides whether the step after it runs.
 */
static int exynos_pd_run_sequence(struct exynos_pm_domain *pd, const char *what,
				  const struct exynos_pd_step *seq,
				  unsigned int nr_steps, enum exynos_pd_pass pass)
{
	bool skip = false;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < nr_steps; i++) {
		const struct exynos_pd_step *step = &seq[i];

		if (pass == EXYNOS_PD_PASS_SAVE) {
			if (step->op == EXYNOS_PD_OP_SAVE)
				pd->saved[i] = readl(exynos_pd_step_addr(pd, step)) &
					       step->mask;
			continue;
		}

		if (skip) {
			skip = false;
			continue;
		}

		switch (step->op) {
		case EXYNOS_PD_OP_WRITE:
			ret = exynos_pd_step_write(pd, step, step->value);
			break;
		case EXYNOS_PD_OP_WAIT:
			ret = exynos_pd_step_wait(pd, step);
			break;
		case EXYNOS_PD_OP_SAVE:
			if (pass == EXYNOS_PD_PASS_RESTORE && pd->have_saved)
				ret = exynos_pd_step_write(pd, step, pd->saved[i]);
			break;
		case EXYNOS_PD_OP_SKIP_IF:
			skip = (readl(exynos_pd_step_addr(pd, step)) & step->mask) ==
			       step->value;
			break;
		case EXYNOS_PD_OP_SET_BITS:
			ret = exynos_pd_set_bits_pmu(pd, step->offset, step->value);
			break;
		case EXYNOS_PD_OP_DELAY:
			fsleep(step->value);
			break;
		}

		if (ret) {
			pr_err("Power domain %s: %s sequence failed at step %u: %d\n",
			       pd->pd.name, what, i, ret);
			return ret;
		}
	}

	return 0;
}

/*
 * The vendor's order: save the CMU state, let the secure world save its
 * part, then take the domain down; bring it up, let the secure world
 * restore, then restore the CMU state.
 */
static int exynos_pd_sequence_power_on(struct exynos_pm_domain *pd)
{
	const struct exynos_pd_sequences *seq = pd->seq;
	int ret;

	ret = exynos_pd_run_sequence(pd, "on", seq->on, seq->nr_on,
				     EXYNOS_PD_PASS_RUN);
	if (ret)
		return ret;

	exynos_pd_secure_control(pd, true);

	ret = exynos_pd_run_sequence(pd, "restore", seq->save, seq->nr_save,
				     EXYNOS_PD_PASS_RESTORE);
	pd->have_saved = false;

	return ret;
}

static int exynos_pd_sequence_power_off(struct exynos_pm_domain *pd)
{
	const struct exynos_pd_sequences *seq = pd->seq;
	int ret;

	exynos_pd_run_sequence(pd, "save", seq->save, seq->nr_save,
			       EXYNOS_PD_PASS_SAVE);
	pd->have_saved = true;

	exynos_pd_secure_control(pd, false);

	ret = exynos_pd_run_sequence(pd, "off", seq->off, seq->nr_off,
				     EXYNOS_PD_PASS_RUN);
	if (!ret)
		return 0;

	/*
	 * An off step that fails leaves the domain wherever the sequence got
	 * to, and genpd keeps a domain whose power-off failed as on without
	 * ever running the on callback for it again.  Try to make that true
	 * by bringing it back up; the vendor kernel reboots at this point, so
	 * there is no better recovery to copy.
	 */
	pr_err("Power domain %s: power-off failed, bringing it back up\n",
	       pd->pd.name);
	if (exynos_pd_sequence_power_on(pd))
		pr_err("Power domain %s: could not bring it back up\n",
		       pd->pd.name);

	return ret;
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 timeout, pwr;
	char *op;
	int ret;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	if (pd->seq)
		return power_on ? exynos_pd_sequence_power_on(pd) :
				  exynos_pd_sequence_power_off(pd);

	if (!power_on)
		exynos_pd_secure_control(pd, false);

	pwr = power_on ? pd->local_pwr_cfg : 0;
	ret = exynos_pd_write_pmu(pd, 0, pwr);
	if (ret)
		return ret;

	/* Wait max 1ms */
	timeout = 10;

	while ((readl_relaxed(base + 0x4) & pd->local_pwr_cfg) != pwr) {
		if (!timeout) {
			op = (power_on) ? "enable" : "disable";
			pr_err("Power domain %s %s failed\n", domain->name, op);
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	if (power_on)
		exynos_pd_secure_control(pd, true);

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

static const struct exynos_pm_domain_config zumapro_cfg = {
	.local_pwr_cfg		= BIT(0),
	.secure_pmu		= true,
	.sequences		= zumapro_pd_sequences,
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	}, {
		.compatible = "google,zumapro-pd",
		.data = &zumapro_cfg,
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

static int exynos_pd_check_sequence(struct exynos_pm_domain *pd,
				    const char *what,
				    const struct exynos_pd_step *seq,
				    unsigned int nr_steps)
{
	unsigned int i;

	for (i = 0; i < nr_steps; i++) {
		const struct exynos_pd_step *step = &seq[i];

		if (step->op == EXYNOS_PD_OP_DELAY)
			continue;

		if (step->window >= EXYNOS_PD_NR_WINDOWS ||
		    !pd->windows[step->window]) {
			pr_err("Power domain %s: %s step %u needs a %s window\n",
			       pd->pd.name, what, i,
			       step->window < EXYNOS_PD_NR_WINDOWS ?
			       exynos_pd_window_names[step->window] : "?");
			return -EINVAL;
		}

		if (step->offset + sizeof(u32) > pd->window_size[step->window]) {
			pr_err("Power domain %s: %s step %u is outside the %s window\n",
			       pd->pd.name, what, i,
			       exynos_pd_window_names[step->window]);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * The CMU and SYSREG windows are shared with the block's clock provider and
 * with sibling domains, so they are mapped without being claimed.
 */
static int exynos_pd_init_sequences(struct platform_device *pdev,
				    struct exynos_pm_domain *pd,
				    const struct exynos_pm_domain_config *cfg)
{
	const struct exynos_pd_sequences *seq;
	struct device *dev = &pdev->dev;
	struct resource *res;
	unsigned int i;
	int ret;

	for (seq = cfg->sequences; seq->pmu; seq++) {
		if (seq->pmu == pd->base_addr) {
			pd->seq = seq;
			break;
		}
	}

	if (!pd->seq) {
		dev_err(dev, "no power sequence for the domain at %pa\n",
			&pd->base_addr);
		return -ENODEV;
	}

	for (i = EXYNOS_PD_CMU; i < EXYNOS_PD_NR_WINDOWS; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   exynos_pd_window_names[i]);
		if (!res)
			continue;

		pd->windows[i] = devm_ioremap(dev, res->start,
					      resource_size(res));
		if (!pd->windows[i])
			return -ENOMEM;
		pd->window_size[i] = resource_size(res);
	}

	ret = exynos_pd_check_sequence(pd, "on", pd->seq->on, pd->seq->nr_on);
	if (ret)
		return ret;
	ret = exynos_pd_check_sequence(pd, "save", pd->seq->save,
				       pd->seq->nr_save);
	if (ret)
		return ret;
	ret = exynos_pd_check_sequence(pd, "off", pd->seq->off,
				       pd->seq->nr_off);
	if (ret)
		return ret;

	if (pd->seq->nr_save) {
		pd->saved = devm_kcalloc(dev, pd->seq->nr_save,
					 sizeof(*pd->saved), GFP_KERNEL);
		if (!pd->saved)
			return -ENOMEM;
	}

	return 0;
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	int on, ret;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(dev, np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pd->base))
		return PTR_ERR(pd->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	pd->base_addr = res->start;
	pd->windows[EXYNOS_PD_PMU] = pd->base;
	pd->window_size[EXYNOS_PD_PMU] = resource_size(res);

	of_property_read_u32(np, "samsung,secure-pd-id", &pd->secure_pwr_id);

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	pd->secure_pmu = pm_domain_cfg->secure_pmu;

	if (pm_domain_cfg->sequences) {
		ret = exynos_pd_init_sequences(pdev, pd, pm_domain_cfg);
		if (ret)
			return ret;
	}
	if (of_property_read_bool(np, "samsung,always-on"))
		pd->pd.flags |= GENPD_FLAG_ALWAYS_ON;

	/*
	 * Some Samsung platforms with bootloaders turning on the splash-screen
	 * and handing it over to the kernel, requires the power-domains to be
	 * reset during boot.
	 */
	if (IS_ENABLED(CONFIG_ARM) &&
	    of_device_is_compatible(np, "samsung,exynos4210-pd"))
		exynos_pd_power_off(&pd->pd);

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;

	ret = pm_genpd_init(&pd->pd, NULL, !on);
	if (ret)
		return ret;

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
