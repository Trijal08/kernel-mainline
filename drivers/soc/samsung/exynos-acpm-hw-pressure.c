// SPDX-License-Identifier: GPL-2.0-only
/*
 * Report firmware-imposed CPU frequency capping to the scheduler.
 *
 * The ACPM firmware owns DVFS on these SoCs, and what it applies is not always
 * what it was asked for: it arbitrates between the kernel's request and its own
 * thermal and current limits.  The kernel never learns that from the clock API,
 * because the request is accepted either way, so the scheduler goes on
 * believing a capped CPU still has its full capacity -- it keeps placing work
 * there, and the energy model keeps costing that work against a frequency the
 * core cannot reach.  That is a loop, and it pushes work towards whichever core
 * is already too hot to run it.
 *
 * The firmware publishes the rate it actually applied per domain, which the
 * ACPM driver exposes through its DVFS get_rate(); comparing that against what
 * cpufreq last asked for gives the cap, and arch_update_hw_pressure() is how
 * the scheduler is told about it.
 *
 * Copyright 2026 Trijal Saha
 */

#include <linux/arch_topology.h>
#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/units.h>
#include <linux/workqueue.h>

/*
 * How often to look. The firmware moves a cap on its own schedule and gives no
 * interrupt for it, so this is a poll -- but a cheap one: with the
 * shared-memory DVFS path the read is a load, not a round trip, and the driver
 * only registers when that path is available. Nothing is capped most of the
 * time, so back off until something is.
 */
#define ACPM_HWP_POLL_CAPPED_MS		500
#define ACPM_HWP_POLL_IDLE_MS		2000

/**
 * struct acpm_hw_pressure - driver private data.
 * @handle:	ACPM handle used to read the applied rates.
 * @dev:	this device.
 * @work:	the polling work.
 * @chan:	ACPM mailbox channel carrying the DVFS protocol.
 */
struct acpm_hw_pressure {
	struct acpm_handle *handle;
	struct device *dev;
	struct delayed_work work;
	unsigned int chan;
};

/**
 * acpm_hwp_cpu_domain() - find a CPU's ACPM DVFS domain.
 * @cpu: the CPU.
 *
 * The CPU's clock is the firmware's DVFS domain for its cluster, and the
 * specifier's argument is the domain ID the protocol uses, so take it from
 * there rather than from a table of CPU numbers.
 *
 * Return: the domain ID, or negative errno.
 */
static int acpm_hwp_cpu_domain(unsigned int cpu)
{
	struct of_phandle_args args;
	int ret;

	struct device_node *np __free(device_node) = of_get_cpu_node(cpu, NULL);

	if (!np)
		return -ENODEV;

	ret = of_parse_phandle_with_args(np, "clocks", "#clock-cells", 0, &args);
	if (ret)
		return ret;

	of_node_put(args.np);

	if (args.args_count < 1)
		return -EINVAL;

	return args.args[0];
}

/**
 * acpm_hwp_update_policy() - tell the scheduler this policy's current cap.
 * @ahp:	driver private data.
 * @policy:	the cpufreq policy.
 *
 * Return: true if the firmware is holding this policy below what cpufreq
 * asked for.
 */
static bool acpm_hwp_update_policy(struct acpm_hw_pressure *ahp,
				   struct cpufreq_policy *policy)
{
	unsigned long applied_khz;
	unsigned int capped_khz;
	int domain;

	if (!policy->cur)
		return false;

	domain = acpm_hwp_cpu_domain(policy->cpu);
	if (domain < 0)
		return false;

	applied_khz = ahp->handle->ops->dvfs.get_rate(ahp->handle, ahp->chan,
						      domain) / HZ_PER_KHZ;
	if (!applied_khz)
		return false;

	/*
	 * Only a rate below the request is a cap. Running at or above it means
	 * the governor simply asked for less, which is not pressure, so report
	 * the policy's own ceiling and let the core clear the pressure.
	 */
	if (applied_khz >= policy->cur) {
		capped_khz = policy->cpuinfo.max_freq;
		arch_update_hw_pressure(policy->related_cpus, capped_khz);
		return false;
	}

	arch_update_hw_pressure(policy->related_cpus, applied_khz);

	return true;
}

static void acpm_hwp_work(struct work_struct *work)
{
	struct acpm_hw_pressure *ahp = container_of(to_delayed_work(work),
						    struct acpm_hw_pressure,
						    work);
	cpumask_var_t seen;
	bool capped = false;
	unsigned int cpu;

	if (!zalloc_cpumask_var(&seen, GFP_KERNEL))
		goto again;

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;

		if (cpumask_test_cpu(cpu, seen))
			continue;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		cpumask_or(seen, seen, policy->related_cpus);
		capped |= acpm_hwp_update_policy(ahp, policy);
		cpufreq_cpu_put(policy);
	}

	free_cpumask_var(seen);

again:
	schedule_delayed_work(&ahp->work,
			      msecs_to_jiffies(capped ? ACPM_HWP_POLL_CAPPED_MS
						      : ACPM_HWP_POLL_IDLE_MS));
}

static void acpm_hwp_cancel(void *data)
{
	struct acpm_hw_pressure *ahp = data;
	unsigned int cpu;

	cancel_delayed_work_sync(&ahp->work);

	/* Leave nothing behind: a stale pressure never decays on its own. */
	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;

		arch_update_hw_pressure(policy->related_cpus,
					policy->cpuinfo.max_freq);
		cpufreq_cpu_put(policy);
	}
}

static int acpm_hw_pressure_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct acpm_hw_pressure *ahp;
	int ret;

	ahp = devm_kzalloc(dev, sizeof(*ahp), GFP_KERNEL);
	if (!ahp)
		return -ENOMEM;

	ahp->dev = dev;
	ahp->handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(ahp->handle))
		return dev_err_probe(dev, PTR_ERR(ahp->handle),
				     "Failed to get acpm handle\n");

	INIT_DELAYED_WORK(&ahp->work, acpm_hwp_work);

	ret = devm_add_action_or_reset(dev, acpm_hwp_cancel, ahp);
	if (ret)
		return ret;

	schedule_delayed_work(&ahp->work,
			      msecs_to_jiffies(ACPM_HWP_POLL_IDLE_MS));

	return 0;
}

static const struct platform_device_id acpm_hw_pressure_id[] = {
	{ "exynos-acpm-hw-pressure" },
	{ }
};
MODULE_DEVICE_TABLE(platform, acpm_hw_pressure_id);

static struct platform_driver acpm_hw_pressure_driver = {
	.driver = {
		.name = "exynos-acpm-hw-pressure",
	},
	.probe = acpm_hw_pressure_probe,
	.id_table = acpm_hw_pressure_id,
};
module_platform_driver(acpm_hw_pressure_driver);

MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_DESCRIPTION("Exynos ACPM firmware CPU frequency capping reporter");
MODULE_LICENSE("GPL");
