// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2011-2014 Samsung Electronics Co., Ltd.
//		http://www.samsung.com/
//
// Exynos - CPU PMU(Power Management Unit) support

#include <linux/array_size.h>
#include <linux/bitmap.h>
#include <linux/cpuhotplug.h>
#include <linux/cpu_pm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include "exynos-pmu.h"

struct exynos_pmu_context {
	struct device *dev;
	const struct exynos_pmu_data *pmu_data;
	struct regmap *pmureg;
	struct regmap *pmuintrgen;
	/*
	 * Serialization lock for CPU hot plug and cpuidle ACPM hint
	 * programming. Also protects in_cpuhp, sys_insuspend & sys_inreboot
	 * flags.
	 */
	raw_spinlock_t cpupm_lock;
	unsigned long *in_cpuhp;
	bool sys_insuspend;
	bool sys_inreboot;
};

void __iomem *pmu_base_addr;
static struct exynos_pmu_context *pmu_context;
/* forward declaration */
static struct platform_driver exynos_pmu_driver;

void pmu_raw_writel(u32 val, u32 offset)
{
	writel_relaxed(val, pmu_base_addr + offset);
}

u32 pmu_raw_readl(u32 offset)
{
	return readl_relaxed(pmu_base_addr + offset);
}

void exynos_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int i;
	const struct exynos_pmu_data *pmu_data;

	if (!pmu_context || !pmu_context->pmu_data)
		return;

	pmu_data = pmu_context->pmu_data;

	if (pmu_data->powerdown_conf)
		pmu_data->powerdown_conf(mode);

	if (pmu_data->pmu_config) {
		for (i = 0; (pmu_data->pmu_config[i].offset != PMU_TABLE_END); i++)
			pmu_raw_writel(pmu_data->pmu_config[i].val[mode],
					pmu_data->pmu_config[i].offset);
	}

	if (pmu_data->powerdown_conf_extra)
		pmu_data->powerdown_conf_extra(mode);

	if (pmu_data->pmu_config_extra) {
		for (i = 0; pmu_data->pmu_config_extra[i].offset != PMU_TABLE_END; i++)
			pmu_raw_writel(pmu_data->pmu_config_extra[i].val[mode],
				       pmu_data->pmu_config_extra[i].offset);
	}
}

/*
 * Split the data between ARM architectures because it is relatively big
 * and useless on other arch.
 */
#ifdef CONFIG_EXYNOS_PMU_ARM_DRIVERS
#define exynos_pmu_data_arm_ptr(data)	(&data)
#else
#define exynos_pmu_data_arm_ptr(data)	NULL
#endif

static const struct regmap_config regmap_smccfg = {
	.name = "pmu_regs",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.use_single_read = true,
	.use_single_write = true,
	.reg_read = tensor_sec_reg_read,
	.reg_write = tensor_sec_reg_write,
	.reg_update_bits = tensor_sec_update_bits,
	.use_raw_spinlock = true,
};

static const struct regmap_config regmap_pmu_intr = {
	.name = "pmu_intr_gen",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.use_raw_spinlock = true,
};

/*
 * PMU platform driver and devicetree bindings.
 */
static const struct of_device_id exynos_pmu_of_device_ids[] = {
	{
		.compatible = "google,gs101-pmu",
		.data = &gs101_pmu_data,
	}, {
		.compatible = "google,zumapro-pmu",
		.data = &zumapro_pmu_data,
	}, {
		.compatible = "samsung,exynos3250-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos3250_pmu_data),
	}, {
		.compatible = "samsung,exynos4210-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4210_pmu_data),
	}, {
		.compatible = "samsung,exynos4212-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4212_pmu_data),
	}, {
		.compatible = "samsung,exynos4412-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4412_pmu_data),
	}, {
		.compatible = "samsung,exynos5250-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos5250_pmu_data),
	}, {
		.compatible = "samsung,exynos5410-pmu",
	}, {
		.compatible = "samsung,exynos5420-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos5420_pmu_data),
	}, {
		.compatible = "samsung,exynos5433-pmu",
	}, {
		.compatible = "samsung,exynos7-pmu",
	}, {
		.compatible = "samsung,exynos850-pmu",
	},
	{ /*sentinel*/ },
};

static const struct mfd_cell exynos_pmu_devs[] = {
	{ .name = "exynos-clkout", },
};

/**
 * exynos_get_pmu_regmap() - Obtain pmureg regmap
 *
 * Find the pmureg regmap previously configured in probe() and return regmap
 * pointer.
 *
 * Return: A pointer to regmap if found or ERR_PTR error value.
 */
struct regmap *exynos_get_pmu_regmap(void)
{
	struct device_node *np = of_find_matching_node(NULL,
						      exynos_pmu_of_device_ids);
	if (np)
		return exynos_get_pmu_regmap_by_phandle(np, NULL);
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(exynos_get_pmu_regmap);

/**
 * exynos_get_pmu_regmap_by_phandle() - Obtain pmureg regmap via phandle
 * @np: Device node holding PMU phandle property
 * @propname: Name of property holding phandle value
 *
 * Find the pmureg regmap previously configured in probe() and return regmap
 * pointer.
 *
 * Return: A pointer to regmap if found or ERR_PTR error value.
 */
struct regmap *exynos_get_pmu_regmap_by_phandle(struct device_node *np,
						const char *propname)
{
	struct device_node *pmu_np;
	struct device *dev;

	if (propname)
		pmu_np = of_parse_phandle(np, propname, 0);
	else
		pmu_np = np;

	if (!pmu_np)
		return ERR_PTR(-ENODEV);

	/*
	 * Determine if exynos-pmu device has probed and therefore regmap
	 * has been created and can be returned to the caller. Otherwise we
	 * return -EPROBE_DEFER.
	 */
	dev = driver_find_device_by_of_node(&exynos_pmu_driver.driver,
					    (void *)pmu_np);

	if (propname)
		of_node_put(pmu_np);

	if (!dev)
		return ERR_PTR(-EPROBE_DEFER);

	put_device(dev);

	return syscon_node_to_regmap(pmu_np);
}
EXPORT_SYMBOL_GPL(exynos_get_pmu_regmap_by_phandle);

/*
 * CPU_INFORM register "hint" values are required to be programmed in addition to
 * the standard PSCI calls to have functional CPU hotplug and CPU idle states.
 * This is required to workaround limitations in the el3mon/ACPM firmware.
 */
#define CPU_INFORM_CLEAR	0
#define CPU_INFORM_C2		1
#define CPU_INFORM_SICD		3

/* PMU_INFORM0 value telling EL3/TF-A that Linux may use the C2 idle state. */
#define PMU_ALLOWED_C2		1

/*
 * __gs101_cpu_pmu_ prefix functions are common code shared by CPU PM notifiers
 * (CPUIdle) and CPU hotplug callbacks. Functions should be called with IRQs
 * disabled and cpupm_lock held.
 */
static int __gs101_cpu_pmu_online(unsigned int cpu)
	__must_hold(&pmu_context->cpupm_lock)
{
	unsigned int cpuhint = smp_processor_id();
	u32 reg, mask;

	/* clear cpu inform hint */
	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpuhint),
		     CPU_INFORM_CLEAR);

	mask = BIT(cpu);

	regmap_update_bits(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_ENABLE,
			   mask, (0 << cpu));

	regmap_read(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_UPEND, &reg);

	regmap_write(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_CLEAR,
		     reg & mask);

	return 0;
}

/* Called from CPU PM notifier (CPUIdle code path) with IRQs disabled */
static int gs101_cpu_pmu_online(void)
{
	int cpu;

	raw_spin_lock(&pmu_context->cpupm_lock);

	if (pmu_context->sys_inreboot) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_OK;
	}

	cpu = smp_processor_id();
	__gs101_cpu_pmu_online(cpu);
	raw_spin_unlock(&pmu_context->cpupm_lock);

	return NOTIFY_OK;
}

/* Called from CPU hot plug callback with IRQs enabled */
static int gs101_cpuhp_pmu_online(unsigned int cpu)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&pmu_context->cpupm_lock, flags);

	__gs101_cpu_pmu_online(cpu);
	/*
	 * Mark this CPU as having finished the hotplug.
	 * This means this CPU can now enter C2 idle state.
	 */
	clear_bit(cpu, pmu_context->in_cpuhp);
	raw_spin_unlock_irqrestore(&pmu_context->cpupm_lock, flags);

	return 0;
}

/* Common function shared by both CPU hot plug and CPUIdle */
static int __gs101_cpu_pmu_offline(unsigned int cpu)
	__must_hold(&pmu_context->cpupm_lock)
{
	unsigned int cpuhint = smp_processor_id();
	u32 reg, mask;

	/* set cpu inform hint */
	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpuhint),
		     CPU_INFORM_C2);

	mask = BIT(cpu);
	regmap_update_bits(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_ENABLE,
			   mask, BIT(cpu));

	regmap_read(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_UPEND, &reg);
	regmap_write(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_CLEAR,
		     reg & mask);

	mask = (BIT(cpu + 8));
	regmap_read(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_UPEND, &reg);
	regmap_write(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_CLEAR,
		     reg & mask);

	return 0;
}

/* Called from CPU PM notifier (CPUIdle code path) with IRQs disabled */
static int gs101_cpu_pmu_offline(void)
{
	int cpu;

	raw_spin_lock(&pmu_context->cpupm_lock);
	cpu = smp_processor_id();

	if (test_bit(cpu, pmu_context->in_cpuhp)) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_BAD;
	}

	/* Ignore CPU_PM_ENTER event in reboot or suspend sequence. */
	if (pmu_context->sys_insuspend || pmu_context->sys_inreboot) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_OK;
	}

	__gs101_cpu_pmu_offline(cpu);
	raw_spin_unlock(&pmu_context->cpupm_lock);

	return NOTIFY_OK;
}

/* Called from CPU hot plug callback with IRQs enabled */
static int gs101_cpuhp_pmu_offline(unsigned int cpu)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&pmu_context->cpupm_lock, flags);
	/*
	 * Mark this CPU as entering hotplug. So as not to confuse
	 * ACPM the CPU entering hotplug should not enter C2 idle state.
	 */
	set_bit(cpu, pmu_context->in_cpuhp);
	__gs101_cpu_pmu_offline(cpu);

	raw_spin_unlock_irqrestore(&pmu_context->cpupm_lock, flags);

	return 0;
}

static int gs101_cpu_pm_notify_callback(struct notifier_block *self,
					unsigned long action, void *v)
{
	switch (action) {
	case CPU_PM_ENTER:
		return gs101_cpu_pmu_offline();

	case CPU_PM_EXIT:
		return gs101_cpu_pmu_online();
	}

	return NOTIFY_OK;
}

static struct notifier_block gs101_cpu_pm_notifier = {
	.notifier_call = gs101_cpu_pm_notify_callback,
	/*
	 * We want to be called first, as the ACPM hint and handshake is what
	 * puts the CPU into C2.
	 */
	.priority = INT_MAX
};

static int exynos_cpupm_reboot_notifier(struct notifier_block *nb,
					unsigned long event, void *v)
{
	unsigned long flags;

	switch (event) {
	case SYS_POWER_OFF:
	case SYS_RESTART:
		raw_spin_lock_irqsave(&pmu_context->cpupm_lock, flags);
		pmu_context->sys_inreboot = true;
		raw_spin_unlock_irqrestore(&pmu_context->cpupm_lock, flags);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_cpupm_reboot_nb = {
	.priority = INT_MAX,
	.notifier_call = exynos_cpupm_reboot_notifier,
};

static int setup_cpuhp_and_cpuidle(struct device *dev)
{
	struct device_node *intr_gen_node;
	struct resource intrgen_res;
	void __iomem *virt_addr;
	int ret, cpu;

	intr_gen_node = of_parse_phandle(dev->of_node,
					 "google,pmu-intr-gen-syscon", 0);
	if (!intr_gen_node) {
		/*
		 * To maintain support for older DTs that didn't specify syscon
		 * phandle just issue a warning rather than fail to probe.
		 */
		dev_warn(dev, "pmu-intr-gen syscon unavailable\n");
		return 0;
	}

	/*
	 * To avoid lockdep issues (CPU PM notifiers use raw spinlocks) create
	 * a mmio regmap for pmu-intr-gen that uses raw spinlocks instead of
	 * syscon provided regmap.
	 */
	ret = of_address_to_resource(intr_gen_node, 0, &intrgen_res);
	of_node_put(intr_gen_node);

	virt_addr = devm_ioremap(dev, intrgen_res.start,
				 resource_size(&intrgen_res));
	if (!virt_addr)
		return -ENOMEM;

	pmu_context->pmuintrgen = devm_regmap_init_mmio(dev, virt_addr,
							&regmap_pmu_intr);
	if (IS_ERR(pmu_context->pmuintrgen)) {
		dev_err(dev, "failed to initialize pmu-intr-gen regmap\n");
		return PTR_ERR(pmu_context->pmuintrgen);
	}

	/* register custom mmio regmap with syscon */
	ret = of_syscon_register_regmap(intr_gen_node,
					pmu_context->pmuintrgen);
	if (ret)
		return ret;

	pmu_context->in_cpuhp = devm_bitmap_zalloc(dev, num_possible_cpus(),
						   GFP_KERNEL);
	if (!pmu_context->in_cpuhp)
		return -ENOMEM;

	/* set PMU to power on */
	for_each_online_cpu(cpu)
		gs101_cpuhp_pmu_online(cpu);

	/* register CPU hotplug callbacks */
	cpuhp_setup_state(CPUHP_BP_PREPARE_DYN,	"soc/exynos-pmu:prepare",
			  gs101_cpuhp_pmu_online, NULL);

	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "soc/exynos-pmu:online",
			  NULL, gs101_cpuhp_pmu_offline);

	/* register CPU PM notifiers for cpuidle */
	cpu_pm_register_notifier(&gs101_cpu_pm_notifier);
	register_reboot_notifier(&exynos_cpupm_reboot_nb);
	return 0;
}

/*
 * Zumapro's firmware does not program the PMU wakeup interrupt enables for the
 * powered-down system idle/sleep states.  Arm them around system suspend so the
 * firmware has a valid wake source; mirrors the downstream cpupm "wakeup-mask"
 * node (the external/pin EINT masks are programmed separately by pinctrl).
 */
static const struct {
	unsigned int stat_reg;
	unsigned int en_reg;
	u32 mask;
} zumapro_wakeup_mask[] = {
	{ GS101_WAKEUP_STAT, GS101_TOP_INT_EN, 0xff00000 },
	{ 0x3970, GS101_WAKEUP2_INT_EN, 0x0 },
};

static void zumapro_set_wakeup_mask(bool arm)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_wakeup_mask); i++) {
		regmap_write(pmu_context->pmureg,
			     zumapro_wakeup_mask[i].stat_reg, 0);
		regmap_write(pmu_context->pmureg, zumapro_wakeup_mask[i].en_reg,
			     arm ? zumapro_wakeup_mask[i].mask : 0);
	}
}

static struct cpumask zumapro_idle_cpus;
/* cpu currently asserting system idle (SICD), or -1 if none */
static int zumapro_sicd_holder = -1;
/* debug counters, printed once per resume */
static u32 zumapro_dbg_c2, zumapro_dbg_sicd, zumapro_dbg_fail;

/*
 * Plain PSCI does not keep zumapro's cores powered down during system idle: the
 * firmware needs every idling core to publish its idle intent through its
 * CPU_INFORM register, and the last core down to request system idle (SICD)
 * with the wakeup mask armed (see zumapro_set_wakeup_mask()); otherwise it
 * powers the cores straight back up.  Mirrors the downstream exynos-cpupm
 * CPU_INFORM hints; no pmu-intr-gen handshake is needed (downstream does not
 * touch it on the idle-enter path).  Only active inside a system suspend, when
 * the mask is armed and all cores are parking; awake idle uses standard PSCI.
 */
static int zumapro_cpu_pm_notify(struct notifier_block *self,
				 unsigned long action, void *v)
{
	unsigned int cpu = smp_processor_id();
	u32 hint;

	raw_spin_lock(&pmu_context->cpupm_lock);

	if (!pmu_context->sys_insuspend) {
		raw_spin_unlock(&pmu_context->cpupm_lock);
		return NOTIFY_OK;
	}

	switch (action) {
	case CPU_PM_ENTER:
		cpumask_set_cpu(cpu, &zumapro_idle_cpus);
		/*
		 * Elect a single SICD holder: the first core to go idle claims
		 * system idle, the rest report plain C2.  Exactly one SICD hint
		 * (plus the armed wakeup mask) is what makes the firmware hold
		 * the cluster; a kernel cpumask "all idle" test never fires
		 * because the cores enter/exit faster than they ever coincide.
		 */
		if (zumapro_sicd_holder < 0) {
			zumapro_sicd_holder = cpu;
			hint = CPU_INFORM_SICD;
			zumapro_dbg_sicd++;
		} else {
			hint = CPU_INFORM_C2;
			zumapro_dbg_c2++;
		}
		if (regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu), hint))
			zumapro_dbg_fail++;
		break;
	case CPU_PM_EXIT:
		cpumask_clear_cpu(cpu, &zumapro_idle_cpus);
		if (zumapro_sicd_holder == cpu)
			zumapro_sicd_holder = -1;
		regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu),
			     CPU_INFORM_CLEAR);
		break;
	}

	raw_spin_unlock(&pmu_context->cpupm_lock);
	return NOTIFY_OK;
}

static struct notifier_block zumapro_cpu_pm_notifier = {
	.notifier_call = zumapro_cpu_pm_notify,
	.priority = INT_MAX,
};

/*
 * SYSREG_CPUCL0 (CMU_CPUCL0 base 0x29c00000 + 0x20000) BUS_COMPONENT_DRCG_EN
 * registers for the boot cluster's DynamIQ Shared Unit.
 */
#define ZUMAPRO_CPUCL0_SYSREG		0x29c20000
#define CPUCL0_DSU_DRCG_EN		0x0104
#define CPUCL0_DSU_DRCG_EN_INT		0x010c

/*
 * Enable dynamic root clock gating of the CPUCL0 DynamIQ Shared Unit. The DSU
 * clock is shared by the whole boot cluster; until DRCG is enabled it never
 * gates while a core is idle, so the firmware will not hold the cluster in the
 * C2 power-down state and bounces the cores straight back out - suspend-to-idle
 * never settles. Downstream programs this from pmucal_lpm_init. Mainline does
 * not model the CPUCL0 CMU (CPU DVFS is ACPM-managed and the clk driver only
 * sees the CMU_TOP feeds), and the generic DRCG helper writes a single register,
 * so enable both DSU DRCG registers with a small direct SYSREG write here.
 */
static void zumapro_enable_dsu_drcg(struct device *dev)
{
	void __iomem *va = ioremap(ZUMAPRO_CPUCL0_SYSREG, 0x1000);

	if (!va) {
		dev_warn(dev, "CPUCL0 DSU DRCG: ioremap failed\n");
		return;
	}
	writel(~0u, va + CPUCL0_DSU_DRCG_EN);
	writel(~0u, va + CPUCL0_DSU_DRCG_EN_INT);
	iounmap(va);
}

static int exynos_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap_config pmu_regmcfg;
	struct regmap *regmap;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	/*
	 * The PMU register block is a syscon that can contain child devices
	 * with their own reg windows - e.g. the Tensor/Zumapro power domains,
	 * whose control registers sit at PMU offsets.  Map it without an
	 * exclusive request_mem_region(), like the generic syscon of_iomap()
	 * path, so those sub-devices can claim their windows instead of the
	 * PMU probe failing with -EBUSY.
	 */
	pmu_base_addr = devm_ioremap(dev, res->start, resource_size(res));
	if (!pmu_base_addr)
		return -ENOMEM;

	pmu_context = devm_kzalloc(&pdev->dev,
			sizeof(struct exynos_pmu_context),
			GFP_KERNEL);
	if (!pmu_context)
		return -ENOMEM;

	pmu_context->pmu_data = of_device_get_match_data(dev);

	/* For SoCs that secure PMU register writes use custom regmap */
	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_secure) {
		pmu_regmcfg = regmap_smccfg;
		pmu_regmcfg.max_register = resource_size(res) -
					   pmu_regmcfg.reg_stride;
		pmu_regmcfg.wr_table = pmu_context->pmu_data->wr_table;
		pmu_regmcfg.rd_table = pmu_context->pmu_data->rd_table;

		/* Need physical address for SMC call */
		regmap = devm_regmap_init(dev, NULL,
					  (void *)(uintptr_t)res->start,
					  &pmu_regmcfg);

		if (IS_ERR(regmap))
			return dev_err_probe(&pdev->dev, PTR_ERR(regmap),
					     "regmap init failed\n");

		ret = of_syscon_register_regmap(dev->of_node, regmap);
		if (ret)
			return ret;
	} else {
		/* let syscon create mmio regmap */
		regmap = syscon_node_to_regmap(dev->of_node);
		if (IS_ERR(regmap))
			return dev_err_probe(&pdev->dev, PTR_ERR(regmap),
					     "syscon_node_to_regmap failed\n");
	}

	pmu_context->pmureg = regmap;
	pmu_context->dev = dev;
	raw_spin_lock_init(&pmu_context->cpupm_lock);
	pmu_context->sys_inreboot = false;
	pmu_context->sys_insuspend = false;

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_cpuhp) {
		ret = setup_cpuhp_and_cpuidle(dev);
		if (ret)
			return ret;
	}

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		unsigned int inform0 = 0;

		/*
		 * Tell EL3/TF-A that Linux is allowed to use the C2 (CPU
		 * power-down) idle state by writing PMU_INFORM0, mirroring what
		 * the downstream cpupm does at init. Without it the firmware
		 * bounces the cores straight back out of C2, so s2idle never
		 * settles even with the per-core CPU_INFORM hints (armed by the
		 * notifier registered below).
		 */
		ret = regmap_write(pmu_context->pmureg, GS101_INFORM0,
				   PMU_ALLOWED_C2);
		regmap_read(pmu_context->pmureg, GS101_INFORM0, &inform0);
		dev_info(dev, "PMU_INFORM0 C2-allow: ret=%d readback=0x%x\n",
			 ret, inform0);

		cpu_pm_register_notifier(&zumapro_cpu_pm_notifier);
		zumapro_enable_dsu_drcg(dev);
	}

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_init)
		pmu_context->pmu_data->pmu_init();

	platform_set_drvdata(pdev, pmu_context);

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, exynos_pmu_devs,
				   ARRAY_SIZE(exynos_pmu_devs), NULL, 0, NULL);
	if (ret)
		return ret;

	if (devm_of_platform_populate(dev))
		dev_err(dev, "Error populating children, reboot and poweroff might not work properly\n");

	dev_dbg(dev, "Exynos PMU Driver probe done\n");
	return 0;
}

static int exynos_cpupm_suspend_noirq(struct device *dev)
{
	raw_spin_lock(&pmu_context->cpupm_lock);
	pmu_context->sys_insuspend = true;
	/* start the idle-hint tracking clean for this suspend */
	cpumask_clear(&zumapro_idle_cpus);
	zumapro_sicd_holder = -1;
	zumapro_dbg_c2 = zumapro_dbg_sicd = zumapro_dbg_fail = 0;
	raw_spin_unlock(&pmu_context->cpupm_lock);

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		unsigned int v = 0xdead;

		zumapro_set_wakeup_mask(true);
		regmap_read(pmu_context->pmureg, GS101_TOP_INT_EN, &v);
		pr_info("zumapro: suspend: wakeup mask armed, TOP_INT_EN(0x3944)=0x%x\n",
			v);
	}

	return 0;
}

static int exynos_cpupm_resume_noirq(struct device *dev)
{
	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		zumapro_set_wakeup_mask(false);
		pr_info("zumapro: resume: CPU_INFORM hints c2=%u sicd=%u fails=%u\n",
			zumapro_dbg_c2, zumapro_dbg_sicd, zumapro_dbg_fail);
	}

	raw_spin_lock(&pmu_context->cpupm_lock);
	pmu_context->sys_insuspend = false;
	raw_spin_unlock(&pmu_context->cpupm_lock);
	return 0;
}

static const struct dev_pm_ops cpupm_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos_cpupm_suspend_noirq,
				  exynos_cpupm_resume_noirq)
};

static struct platform_driver exynos_pmu_driver = {
	.driver  = {
		.name   = "exynos-pmu",
		.of_match_table = exynos_pmu_of_device_ids,
		.pm = pm_sleep_ptr(&cpupm_pm_ops),
	},
	.probe = exynos_pmu_probe,
};

static int __init exynos_pmu_init(void)
{
	return platform_driver_register(&exynos_pmu_driver);

}
postcore_initcall(exynos_pmu_init);
