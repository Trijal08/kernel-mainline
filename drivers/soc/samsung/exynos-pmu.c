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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>

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
	struct device_node *np __free(device_node) =
		of_find_matching_node(NULL, exynos_pmu_of_device_ids);
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
#define CPU_INFORM_CPD		2
#define CPU_INFORM_SICD		3
#define CPU_INFORM_SLEEP	4

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

/*
 * Build the raw-spinlock mmio regmap for the PMU interrupt generator and
 * register it as that node's syscon.  Both the gs101 cpuidle/hotplug hints and
 * the zumapro SYS_SLEEP enter sequence program GRP*_INTR_BID through it.
 *
 * Returns 0 with pmu_context->pmuintrgen left NULL when the DT has no
 * google,pmu-intr-gen-syscon phandle, so older DTs keep probing; callers that
 * need the block must check for NULL.
 */
static int setup_pmu_intr_gen(struct device *dev)
{
	struct device_node *intr_gen_node __free(device_node) =
		of_parse_phandle(dev->of_node, "google,pmu-intr-gen-syscon", 0);
	struct resource intrgen_res;
	void __iomem *virt_addr;
	int ret;

	if (pmu_context->pmuintrgen)
		return 0;

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

	return 0;
}

static int setup_cpuhp_and_cpuidle(struct device *dev)
{
	int ret, cpu;

	ret = setup_pmu_intr_gen(dev);
	if (ret)
		return ret;

	/* older DTs without the phandle: keep probing without the hints */
	if (!pmu_context->pmuintrgen)
		return 0;

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
	u32 sicd_mask;
	u32 sleep_mask;
} zumapro_wakeup_mask[] = {
	/*
	 * System idle (SICD) keeps the GIC alive, so the per-core
	 * CLUSTERn_CPUm_GIC_WAKEUP bits are the wake sources and an armed
	 * device interrupt reaches the core the ordinary way.
	 *
	 * Suspend-to-RAM (SYS_SLEEP) powers the GIC down, so those bits can
	 * wake nothing; the sources are the aggregates the PMU itself latches
	 * -- RTC/TRTC alarm and tick (bits 0-3), the external-interrupt
	 * aggregate that carries the power button (bit 4), and the PCIe/USB
	 * and timer bits above them.  Values are downstream's, from the
	 * exynos-pm node's wakeup_int_en in research/dumped.dts:17683 and
	 * confirmed live in research/tracing/downstream-sys-sleep-decoded.md.
	 */
	{ GS101_WAKEUP_STAT, GS101_TOP_INT_EN, 0xff00000, 0x1d0bf },
	/*
	 * The status register paired with GS101_WAKEUP2_INT_EN.  Spelled out
	 * rather than using GS101_WAKEUP2_STAT, which the header puts at
	 * 0x3954: the register layout around it (0x3960 IN, 0x3964 EN, 0x3968
	 * TYPE, 0x396c DIR) and the downstream exynos-pm node's
	 * wakeup_stat_offset in research/dumped.dts:17681 both say 0x3970.
	 */
	{ 0x3970, GS101_WAKEUP2_INT_EN, 0x0, 0x1f0 },
};

static void zumapro_set_wakeup_mask(bool arm)
{
	bool deep = pm_suspend_target_state == PM_SUSPEND_MEM;
	int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_wakeup_mask); i++) {
		u32 mask = deep ? zumapro_wakeup_mask[i].sleep_mask
				: zumapro_wakeup_mask[i].sicd_mask;

		regmap_write(pmu_context->pmureg,
			     zumapro_wakeup_mask[i].stat_reg, 0);
		regmap_write(pmu_context->pmureg, zumapro_wakeup_mask[i].en_reg,
			     arm ? mask : 0);
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
 *
 * Suspend-to-RAM is a different power mode with a different hint (SLEEP, from
 * the syscore callback below) and its secondaries are hotplugged out rather
 * than idled, so this election must stay out of that path.  The gate is load
 * bearing, not defensive: cpu_pm_suspend() is itself a syscore callback, and
 * syscore suspend runs the list in reverse registration order, so it fires
 * CPU_PM_ENTER on the boot core *after* the sleep hint below has been
 * published and before the firmware call -- overwriting SLEEP with SICD.
 */
static int zumapro_cpu_pm_notify(struct notifier_block *self,
				 unsigned long action, void *v)
{
	unsigned int cpu = smp_processor_id();
	u32 hint;

	raw_spin_lock(&pmu_context->cpupm_lock);

	if (!pmu_context->sys_insuspend ||
	    pm_suspend_target_state != PM_SUSPEND_TO_IDLE) {
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

/*
 * Suspend-to-RAM (the firmware's SYS_SLEEP power mode).
 *
 * Downstream reaches it from exynos-pm's syscore callbacks: cal_pm_enter()
 * publishes the boot core's SLEEP hint and runs a three-register "enter"
 * sequence, and the cores that went down before it published a C2 (or CPD, for
 * the last core of a cluster) hint from the cpu-hotplug teardown.  The kernel
 * then makes the ordinary PSCI SYSTEM_SUSPEND call; the ACPM firmware watches
 * those hints to decide how deep to take the SoC and how to bring it back.
 *
 * Mainline made none of those writes, so an "echo mem" so far armed the system
 * idle state's registers and then asked the firmware to power the SoC down --
 * a request the firmware has no wake path for.  The sequences below are the
 * downstream ones, from research/tracing/downstream-sys-sleep-decoded.md and
 * the zuma flexpmu_cal_system_zuma.h tables.
 */

/* tegu cluster membership: cluster0 = {0-3}, cluster1 = {4-6}, cluster2 = {7} */
static const u8 zumapro_cpu_cluster[] = { 0, 0, 0, 0, 1, 1, 1, 2 };

/*
 * Set while a suspend-to-RAM is in flight, for the hotplug hints below.  Unlike
 * its siblings in pmu_context this needs no lock: it is written in the noirq
 * phases, and the CPU teardown that reads it is separated from both writes by
 * the hotplug machinery, with userspace frozen throughout.
 */
static bool zumapro_in_sys_sleep;

static void zumapro_sys_sleep_arm(void)
{
	unsigned int cl0_int_en =
		GS101_CLUSTER_CPU_INT_EN(GS101_CLUSTER0_OFFSET, 0);
	unsigned int reg;

	/*
	 * Only the boot core's hint is published here, exactly like downstream:
	 * the secondaries' CPU_INFORM belongs to the hotplug teardown, which
	 * ran before this on each dying core, and the firmware stamps its own
	 * "powered down" acknowledgement into the same word as it takes each
	 * core down.  Rewriting them here would clobber that.
	 */
	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(0), CPU_INFORM_SLEEP);

	/*
	 * Without the interrupt generator the wake cannot be routed back, so
	 * leave the rest of the sequence alone rather than half-arm it; the
	 * disarm below bails at the same point.
	 */
	if (!pmu_context->pmuintrgen)
		return;

	regmap_update_bits(pmu_context->pmuintrgen,
			   GS101_GRP2_INTR_BID_ENABLE, BIT(0), BIT(0));

	/*
	 * Clear-pending is "read the pending register, write what was pending
	 * into the clear register", so it writes 0 when nothing is pending.
	 * The capture in downstream-sys-sleep-decoded.md shows a literal 1 here
	 * because that machine had the bit pending at the time, not because the
	 * value is a constant; mainline reads 0 while awake, so it writes 0 and
	 * clears nothing, which is the same behaviour with nothing to clear.
	 */
	regmap_read(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_UPEND, &reg);
	regmap_write(pmu_context->pmuintrgen, GS101_GRP1_INTR_BID_CLEAR,
		     reg & BIT(0));

	/*
	 * A plain read-modify-write, not regmap_update_bits(): for PMU_ALIVE
	 * offsets the secure regmap turns update_bits into the hardware
	 * set/clear-bit alias, which is a different operation from the masked
	 * write downstream's pmucal issues here.  (Whether bit 3 latches at all
	 * is a separate question -- it reads back clear on both kernels, so it
	 * appears to be hardware-gated rather than stored.)
	 */
	regmap_read(pmu_context->pmureg, cl0_int_en, &reg);
	regmap_write(pmu_context->pmureg, cl0_int_en, reg | BIT(3));
}

static void zumapro_sys_sleep_disarm(void)
{
	unsigned int cl0_int_en =
		GS101_CLUSTER_CPU_INT_EN(GS101_CLUSTER0_OFFSET, 0);
	unsigned int reg;

	if (!pmu_context->pmuintrgen)
		return;

	regmap_update_bits(pmu_context->pmuintrgen,
			   GS101_GRP2_INTR_BID_ENABLE, BIT(0), 0);
	regmap_read(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_UPEND, &reg);
	regmap_write(pmu_context->pmuintrgen, GS101_GRP2_INTR_BID_CLEAR,
		     reg & BIT(0));

	regmap_read(pmu_context->pmureg, cl0_int_en, &reg);
	regmap_write(pmu_context->pmureg, cl0_int_en, reg & ~BIT(3));
	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(0), CPU_INFORM_CLEAR);
}

/*
 * syscore callbacks run on the boot core after the secondaries are down and
 * immediately before (and after) the PSCI SYSTEM_SUSPEND call -- the same
 * position downstream's exynos-pm uses.  They are reached for every system
 * sleep state, so check which one is in flight.
 *
 * This undoes only what the enter sequence did.  Downstream's full exit also
 * re-enables bus clock gating across sixteen fabric blocks and walks the PMU
 * state machine, which matters only once the SoC is coming back far enough to
 * need them; that is the next step, not this one.
 */
static int zumapro_sys_sleep_suspend(void *data)
{
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;

	zumapro_sys_sleep_arm();
	return 0;
}

static void zumapro_sys_sleep_resume(void *data)
{
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return;

	zumapro_sys_sleep_disarm();
}

static const struct syscore_ops zumapro_sys_sleep_syscore_ops = {
	.suspend	= zumapro_sys_sleep_suspend,
	.resume		= zumapro_sys_sleep_resume,
};

static struct syscore zumapro_sys_sleep_syscore = {
	.ops = &zumapro_sys_sleep_syscore_ops,
};

/*
 * Publish a dying core's idle hint before its PSCI CPU_OFF, which is where
 * downstream's cpu-hotplug teardown puts it.  Without it the firmware takes
 * the core down while its hint still reads "running", and the recorded state
 * of the machine at SYSTEM_SUSPEND does not match what downstream produces.
 *
 * CPD when this is the last online core of its cluster, C2 otherwise.  Scoped
 * to a suspend-to-RAM in flight so that runtime hotplug and s2idle keep the
 * behaviour they have today.
 *
 * Note the resulting register signature differs from the downstream capture,
 * and legitimately so: freeze_secondary_cpus() here takes the cores down in
 * descending order where the vendor kernel goes up, so the core that ends up
 * last in cluster1 is cpu4 rather than cpu6.  Expect 4,1,1,1,2,1,1,2 across
 * CPU_INFORM[0..7], not the vendor's 4,1,1,1,1,1,2,2.
 */
static int zumapro_cpuhp_sleep_hint_set(unsigned int cpu)
{
	bool cluster_last = true;
	int c;

	if (!zumapro_in_sys_sleep || cpu >= ARRAY_SIZE(zumapro_cpu_cluster))
		return 0;

	for_each_online_cpu(c) {
		if (c != cpu && c < ARRAY_SIZE(zumapro_cpu_cluster) &&
		    zumapro_cpu_cluster[c] == zumapro_cpu_cluster[cpu]) {
			cluster_last = false;
			break;
		}
	}

	regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu),
		     cluster_last ? CPU_INFORM_CPD : CPU_INFORM_C2);
	return 0;
}

static int zumapro_cpuhp_sleep_hint_clear(unsigned int cpu)
{
	if (zumapro_in_sys_sleep && cpu < ARRAY_SIZE(zumapro_cpu_cluster))
		regmap_write(pmu_context->pmureg, GS101_CPU_INFORM(cpu),
			     CPU_INFORM_CLEAR);
	return 0;
}

/*
 * How long the PMU's power-up sequencer waits for each external regulator and
 * for the TCXO to settle when it brings the SoC back.  Downstream programs
 * these once at boot from pmucal_lpm_init[]; mainline has no pmucal, so they
 * sit at their reset values, and a sequencer that does not wait long enough
 * for a rail cannot complete the wake.
 *
 * PMU_ALIVE is write-protected: the writes go out as EL3 calls, which can be
 * refused, so each one is read back.  The read side is an ordinary MMIO read
 * through the kernel's own mapping (only userspace mappings of this block
 * fault), so a readback that differs from what was written means EL3 declined
 * it.  Values from flexpmu_cal_system_zuma.h.
 */
static const struct {
	unsigned int reg;
	u32 mask;
	u32 val;
	const char *name;
} zumapro_lpm_durations[] = {
	{ 0x3cb0, ~0u,      0x244, "MIF" },
	{ 0x3cb4, ~0u,      0x0a0, "TOP" },
	{ 0x3cb8, ~0u,      0x0a0, "CPUCL2" },
	{ 0x3cbc, ~0u,      0x0a0, "CPUCL1" },
	{ 0x3cc0, ~0u,      0x0be, "G3D" },
	{ 0x3cc4, ~0u,      0x0a0, "TPU" },
	/* TCXO is the one entry downstream masks: bits 20-31 are left alone. */
	{ 0x3cc8, 0xfffff,  0x66c, "TCXO" },
};

static void zumapro_program_lpm_durations(struct device *dev)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(zumapro_lpm_durations); i++) {
		u32 mask = zumapro_lpm_durations[i].mask;
		u32 val = zumapro_lpm_durations[i].val;
		unsigned int rb = 0;

		/*
		 * A masked entry is a read-modify-write of those bits and a
		 * full write of the rest, which is what pmucal_rae_write()
		 * does.  Not regmap_update_bits(): for PMU_ALIVE offsets the
		 * secure regmap turns that into the hardware set/clear-bit
		 * alias, which is a different operation.
		 */
		if (mask != ~0u) {
			ret = regmap_read(pmu_context->pmureg,
					  zumapro_lpm_durations[i].reg, &rb);
			if (ret)
				continue;
			val = (rb & ~mask) | (val & mask);
		}

		ret = regmap_write(pmu_context->pmureg,
				   zumapro_lpm_durations[i].reg, val);
		rb = 0;
		regmap_read(pmu_context->pmureg,
			    zumapro_lpm_durations[i].reg, &rb);
		if (ret || rb != val)
			dev_warn(dev, "%s settle duration: wrote 0x%x ret=%d readback=0x%x\n",
				 zumapro_lpm_durations[i].name, val, ret, rb);
		else
			dev_info(dev, "%s settle duration=0x%x\n",
				 zumapro_lpm_durations[i].name, val);
	}
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

		/*
		 * Suspend-to-RAM: the interrupt generator carries the enter
		 * sequence's wake routing, the hotplug teardown publishes each
		 * dying core's hint, and the syscore callbacks bracket the
		 * PSCI SYSTEM_SUSPEND call.
		 */
		ret = setup_pmu_intr_gen(dev);
		if (ret)
			return ret;

		zumapro_program_lpm_durations(dev);
		register_syscore(&zumapro_sys_sleep_syscore);

		ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
					"soc/exynos-pmu:sleep-hint",
					zumapro_cpuhp_sleep_hint_clear,
					zumapro_cpuhp_sleep_hint_set);
		if (ret < 0)
			return ret;
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

	/*
	 * Set before the secondaries are taken offline, which happens after
	 * this phase, so their hotplug teardown can tell a suspend-to-RAM from
	 * a runtime hotplug.
	 */
	zumapro_in_sys_sleep = pm_suspend_target_state == PM_SUSPEND_MEM;

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		zumapro_set_wakeup_mask(true);
	}

	return 0;
}

static int exynos_cpupm_resume_noirq(struct device *dev)
{
	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_sicd_wakeup) {
		zumapro_set_wakeup_mask(false);
		pr_debug("zumapro: resume: CPU_INFORM hints c2=%u sicd=%u fails=%u\n",
			zumapro_dbg_c2, zumapro_dbg_sicd, zumapro_dbg_fail);
	}

	raw_spin_lock(&pmu_context->cpupm_lock);
	pmu_context->sys_insuspend = false;
	raw_spin_unlock(&pmu_context->cpupm_lock);

	/* Cleared last: the secondaries came back online before this phase. */
	zumapro_in_sys_sleep = false;
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
