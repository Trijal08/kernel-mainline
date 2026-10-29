// SPDX-License-Identifier: GPL-2.0
/*
 * Raise bus frequency floors while the CPUs are stalling on memory.
 *
 * The buses behind the CPUs - on Zuma the DSU and the BCI - follow cpufreq
 * through the passive governor, which is enough when a workload's speed is
 * decided by how fast the cores run. It is not enough when the cores are
 * waiting on memory instead, because then the bus wants to be faster than
 * the CPU frequency alone suggests.
 *
 * Sample each CPU's activity monitors for the share of its cycles that went
 * into a backend memory stall. Once that share passes a threshold, ask the
 * bus devices for a floor taken from a per-cluster table, indexed by the
 * frequency the CPU actually achieved over the sample window. The floor is a
 * PM QoS minimum-frequency request, so devfreq keeps whichever of it and the
 * governor's own answer is higher, and dropping the request simply returns
 * the bus to the governor.
 *
 * Copyright 2026 Trijal Saha
 */

#define pr_fmt(fmt) "cpu-memstall: " fmt

#include <linux/cleanup.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/cpumask.h>
#include <linux/devfreq.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/timer.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#include <asm/cpufeature.h>
#include <asm/sysreg.h>

/* Bus devices one instance can hold a floor on. */
#define MEMSTALL_MAX_DEVS		4
#define MEMSTALL_DEFAULT_INTERVAL_US	8000
/* How many times the sampling interval may stretch while a CPU is quiet. */
#define MEMSTALL_IDLE_BACKOFF		8

/* An effective CPU rate, and the floor each bus device is asked for above it. */
struct memstall_row {
	u32 cpu_khz;
	u32 dev_khz[MEMSTALL_MAX_DEVS];
};

struct memstall_cluster {
	u32 stall_threshold;
	struct memstall_row *rows;
	unsigned int nr_rows;
};

struct memstall_dev {
	struct devfreq *devfreq;
	struct dev_pm_qos_request qos;
};

/* Per-CPU sampler state. Written by that CPU's timer, read by the work. */
struct memstall_cpu {
	struct memstall_cluster *cluster;
	struct timer_list timer;
	bool enabled;
	unsigned int interval_us;
	u64 last_cyc;
	u64 last_stall;
	u64 last_ns;
	bool primed;
	u32 dev_khz[MEMSTALL_MAX_DEVS];
};

struct memstall {
	struct device *dev;
	unsigned int interval_us;
	struct memstall_dev devs[MEMSTALL_MAX_DEVS];
	unsigned int nr_devs;
	struct memstall_cluster *clusters;
	unsigned int nr_clusters;
	struct memstall_cpu *cpus;
	struct work_struct work;
	enum cpuhp_state hp_state;
};

/* One such device per system; the hotplug callbacks need to find it. */
static struct memstall *memstall_instance;

/*
 * The table entries are thresholds: take the highest row at or below the rate
 * the CPU achieved, and nothing at all below the first row.
 */
static const struct memstall_row *memstall_lookup(struct memstall_cluster *cluster,
						  u32 cpu_khz)
{
	const struct memstall_row *row = NULL;
	unsigned int i;

	for (i = 0; i < cluster->nr_rows; i++) {
		if (cluster->rows[i].cpu_khz > cpu_khz)
			break;
		row = &cluster->rows[i];
	}

	return row;
}

static void memstall_update_qos(struct work_struct *work)
{
	struct memstall *ms = container_of(work, struct memstall, work);
	unsigned int i, cpu;

	for (i = 0; i < ms->nr_devs; i++) {
		u32 floor = PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE;

		for_each_possible_cpu(cpu) {
			u32 khz = READ_ONCE(ms->cpus[cpu].dev_khz[i]);

			if (khz > floor)
				floor = khz;
		}

		dev_pm_qos_update_request(&ms->devs[i].qos, floor);
	}
}

static void memstall_tick(struct timer_list *t)
{
	struct memstall_cpu *cd = timer_container_of(cd, t, timer);
	struct memstall *ms = memstall_instance;
	struct memstall_cluster *cluster = cd->cluster;
	const struct memstall_row *row = NULL;
	u64 cyc, stall, now, d_cyc, d_stall, d_ns;
	bool changed = false;
	unsigned int i;

	cyc = read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
	stall = read_sysreg_s(SYS_AMEVCNTR0_MEM_STALL);
	now = sched_clock();

	d_cyc = cyc - cd->last_cyc;
	d_stall = stall - cd->last_stall;
	d_ns = now - cd->last_ns;

	cd->last_cyc = cyc;
	cd->last_stall = stall;
	cd->last_ns = now;

	/*
	 * The monitors stop in the deeper idle states and can come back reset,
	 * so only trust a window that looks like one: some time passed, some
	 * cycles were counted, and no more of them stalled than there were
	 * cycles to stall in. A window that fails that asks for nothing, which
	 * is also the right answer for a CPU that was idle through it - so the
	 * floor is never left holding a stale request.
	 */
	if (cd->primed && d_ns && d_cyc && d_stall <= d_cyc) {
		u32 stall_permille = div64_u64(d_stall * 1000, d_cyc);

		if (stall_permille >= cluster->stall_threshold) {
			u32 cpu_khz = div64_u64(d_cyc * (NSEC_PER_SEC / HZ_PER_KHZ),
						d_ns);

			row = memstall_lookup(cluster, cpu_khz);
		}
	}
	cd->primed = true;

	for (i = 0; i < ms->nr_devs; i++) {
		u32 khz = row ? row->dev_khz[i] : 0;

		if (khz != cd->dev_khz[i]) {
			WRITE_ONCE(cd->dev_khz[i], khz);
			changed = true;
		}
	}

	if (changed)
		queue_work(system_dfl_wq, &ms->work);

	/*
	 * Sampling every CPU at the full rate costs a wakeup per core per
	 * window whether or not anything is happening, and on an idle machine
	 * that is the largest thing keeping the cores out of their deeper idle
	 * states - measured at around a thousand extra wakeups a second across
	 * eight cores, against a timer tick that is otherwise almost silent.
	 *
	 * A core that asked for nothing is unlikely to ask for something in the
	 * next window either, so back off on it and come back to the full rate
	 * the moment it does ask. Waiting longer costs only how quickly a floor
	 * appears once a workload turns up, and the floors exist for sustained
	 * memory-bound work rather than for a single window of it.
	 */
	if (!READ_ONCE(cd->enabled))
		return;

	if (row)
		cd->interval_us = ms->interval_us;
	else
		cd->interval_us = min(cd->interval_us * 2,
				      ms->interval_us * MEMSTALL_IDLE_BACKOFF);

	mod_timer(&cd->timer, jiffies + usecs_to_jiffies(cd->interval_us));
}

static int memstall_cpu_online(unsigned int cpu)
{
	struct memstall *ms = memstall_instance;
	struct memstall_cpu *cd;

	if (!ms)
		return 0;

	cd = &ms->cpus[cpu];
	if (!cd->cluster)
		return 0;

	cd->primed = false;
	cd->interval_us = ms->interval_us;
	WRITE_ONCE(cd->enabled, true);
	add_timer_on(&cd->timer, cpu);

	return 0;
}

static int memstall_cpu_offline(unsigned int cpu)
{
	struct memstall *ms = memstall_instance;
	struct memstall_cpu *cd;
	unsigned int i;

	if (!ms)
		return 0;

	cd = &ms->cpus[cpu];
	if (!cd->cluster)
		return 0;

	WRITE_ONCE(cd->enabled, false);
	timer_delete_sync(&cd->timer);

	/* An offline CPU stalls on nothing; drop whatever it was asking for. */
	for (i = 0; i < ms->nr_devs; i++)
		WRITE_ONCE(cd->dev_khz[i], 0);
	queue_work(system_dfl_wq, &ms->work);

	return 0;
}

static int memstall_parse_cluster(struct memstall *ms, struct device_node *np,
				  struct memstall_cluster *cluster)
{
	struct device *dev = ms->dev;
	unsigned int stride = ms->nr_devs + 1;
	unsigned int i, j;
	int count, nr_cpus;

	if (of_property_read_u32(np, "google,mem-stall-threshold-permille",
				 &cluster->stall_threshold))
		return dev_err_probe(dev, -EINVAL,
				     "%pOFn: no stall threshold\n", np);
	if (!cluster->stall_threshold || cluster->stall_threshold > 1000)
		return dev_err_probe(dev, -EINVAL,
				     "%pOFn: stall threshold out of range\n", np);

	count = of_property_count_u32_elems(np, "google,freq-map-khz");
	if (count <= 0 || count % stride)
		return dev_err_probe(dev, -EINVAL,
				     "%pOFn: map is not a multiple of %u\n",
				     np, stride);

	cluster->nr_rows = count / stride;
	cluster->rows = devm_kcalloc(dev, cluster->nr_rows,
				     sizeof(*cluster->rows), GFP_KERNEL);
	if (!cluster->rows)
		return -ENOMEM;

	for (i = 0; i < cluster->nr_rows; i++) {
		u32 val;
		int ret;

		ret = of_property_read_u32_index(np, "google,freq-map-khz",
						 i * stride, &val);
		if (ret)
			return ret;
		cluster->rows[i].cpu_khz = val;

		if (i && val <= cluster->rows[i - 1].cpu_khz)
			return dev_err_probe(dev, -EINVAL,
					     "%pOFn: map is not ascending\n", np);

		for (j = 0; j < ms->nr_devs; j++) {
			ret = of_property_read_u32_index(np, "google,freq-map-khz",
							 i * stride + j + 1, &val);
			if (ret)
				return ret;
			cluster->rows[i].dev_khz[j] = val;
		}
	}

	nr_cpus = 0;
	for (i = 0; ; i++) {
		struct device_node *cpu_np __free(device_node) =
			of_parse_phandle(np, "cpus", i);
		int cpu;

		if (!cpu_np)
			break;

		cpu = of_cpu_node_to_id(cpu_np);
		if (cpu < 0)
			return dev_err_probe(dev, -EINVAL,
					     "%pOFn: bad cpus entry %u\n", np, i);
		/*
		 * Without the activity monitors there is nothing to sample, so
		 * leave that CPU without a timer rather than failing: the bus
		 * still follows cpufreq, it just loses the stall floor.
		 */
		if (!cpu_has_amu_feat(cpu)) {
			dev_warn(dev, "cpu%d has no activity monitors\n", cpu);
			continue;
		}

		ms->cpus[cpu].cluster = cluster;
		nr_cpus++;
	}

	if (!nr_cpus)
		return dev_err_probe(dev, -ENODEV,
				     "%pOFn: no usable CPUs\n", np);

	return 0;
}

static void memstall_remove_qos(void *data)
{
	dev_pm_qos_remove_request(data);
}

static void memstall_cancel_work(void *data)
{
	cancel_work_sync(data);
}

static void memstall_remove_hp(void *data)
{
	cpuhp_remove_state(*(enum cpuhp_state *)data);
}

static void memstall_clear_instance(void *data)
{
	memstall_instance = NULL;
}

static int memstall_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct memstall_cluster *cluster;
	struct memstall *ms;
	unsigned int cpu, i, sampled;
	int ret, count;

	if (memstall_instance)
		return -EBUSY;

	ms = devm_kzalloc(dev, sizeof(*ms), GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->dev = dev;
	ms->cpus = devm_kcalloc(dev, nr_cpu_ids, sizeof(*ms->cpus), GFP_KERNEL);
	if (!ms->cpus)
		return -ENOMEM;

	ms->interval_us = MEMSTALL_DEFAULT_INTERVAL_US;
	of_property_read_u32(np, "google,sample-interval-us", &ms->interval_us);
	if (!usecs_to_jiffies(ms->interval_us))
		return dev_err_probe(dev, -EINVAL,
				     "sample interval is shorter than a jiffy\n");

	count = of_count_phandle_with_args(np, "devfreq", NULL);
	if (count <= 0 || count > MEMSTALL_MAX_DEVS)
		return dev_err_probe(dev, -EINVAL, "need 1 to %u bus devices\n",
				     MEMSTALL_MAX_DEVS);
	ms->nr_devs = count;

	for (i = 0; i < ms->nr_devs; i++) {
		struct devfreq *df = devfreq_get_devfreq_by_phandle(dev, "devfreq", i);

		if (IS_ERR(df))
			return dev_err_probe(dev,
					     PTR_ERR(df) == -ENODEV ? -EPROBE_DEFER :
					     PTR_ERR(df),
					     "cannot get bus devfreq %u\n", i);
		ms->devs[i].devfreq = df;

		ret = dev_pm_qos_add_request(df->dev.parent, &ms->devs[i].qos,
					     DEV_PM_QOS_MIN_FREQUENCY,
					     PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "cannot add QoS request %u\n", i);
		ret = devm_add_action_or_reset(dev, memstall_remove_qos,
					       &ms->devs[i].qos);
		if (ret)
			return ret;
	}

	ms->nr_clusters = of_get_available_child_count(np);
	if (!ms->nr_clusters)
		return dev_err_probe(dev, -EINVAL, "no clusters described\n");
	ms->clusters = devm_kcalloc(dev, ms->nr_clusters, sizeof(*ms->clusters),
				    GFP_KERNEL);
	if (!ms->clusters)
		return -ENOMEM;

	cluster = ms->clusters;
	for_each_available_child_of_node_scoped(np, child) {
		ret = memstall_parse_cluster(ms, child, cluster++);
		if (ret)
			return ret;
	}

	INIT_WORK(&ms->work, memstall_update_qos);
	ret = devm_add_action_or_reset(dev, memstall_cancel_work, &ms->work);
	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (!ms->cpus[cpu].cluster)
			continue;
		timer_setup(&ms->cpus[cpu].timer, memstall_tick,
			    TIMER_DEFERRABLE | TIMER_PINNED);
	}

	memstall_instance = ms;
	ret = devm_add_action_or_reset(dev, memstall_clear_instance, NULL);
	if (ret)
		return ret;

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "devfreq/cpu-memstall:online",
				memstall_cpu_online, memstall_cpu_offline);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot register hotplug state\n");
	ms->hp_state = ret;

	ret = devm_add_action_or_reset(dev, memstall_remove_hp, &ms->hp_state);
	if (ret)
		return ret;

	sampled = 0;
	for_each_possible_cpu(cpu)
		if (ms->cpus[cpu].cluster)
			sampled++;
	dev_info(dev, "sampling %u CPUs every %uus\n", sampled, ms->interval_us);

	return 0;
}

static const struct of_device_id memstall_of_match[] = {
	{ .compatible = "google,zuma-cpu-memstall", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, memstall_of_match);

static struct platform_driver memstall_driver = {
	.probe = memstall_probe,
	.driver = {
		.name = "google-cpu-memstall",
		.of_match_table = memstall_of_match,
	},
};
builtin_platform_driver(memstall_driver);
