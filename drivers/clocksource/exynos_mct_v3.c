// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011, 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Exynos MCT (Multi-Core Timer) v3 support.
 *
 * The v3 MCT block provides a free-running counter (FRC) and a set of per-CPU
 * comparators.  Unlike the older exynos4210-mct global/local timer layout, the
 * v3 block has no global comparator; each comparator raises a per-CPU
 * interrupt.  On Google Tensor (Zuma/Zumapro) this is the only MCT block whose
 * interrupts are actually wired -- the legacy mct@10050000 local-timer IRQs are
 * routed from this block, so the exynos4210-mct driver cannot drive them.
 *
 * The comparators live in the always-on MISC block, so they keep counting and
 * can wake a CPU across the c2 power-down idle state in which the per-CPU ARM
 * architected timer stops (the cpuidle states declare local-timer-stop).  Each
 * comparator is registered as a per-CPU one-shot clockevent with a rating
 * *below* the architected timer, so the tick framework keeps the architected
 * timer as the tick and installs the comparator as that CPU's one-shot wakeup
 * device (tick_set_oneshot_wakeup_device()).  On entry to c2 the tick framework
 * then parks the architected timer and reprograms the CPU's own comparator with
 * the pending tick deadline, so no global broadcast device -- and no broadcast
 * IPI -- is needed.  The SoC has no usable global comparator anyway.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/percpu.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clocksource.h>

#define EXYNOS_MCT_MCT_CFG		0x000
#define EXYNOS_MCT_MCT_INCR_RTCCLK	0x004
#define EXYNOS_MCT_MCT_FRC_ENABLE	0x100
#define EXYNOS_MCT_CNT_L		0x110
#define EXYNOS_MCT_CNT_U		0x114
#define EXYNOS_MCT_COMPENSATE_VALUE	0x124
#define EXYNOS_MCT_COMP_L(i)		(0x200 + ((i) * 0x100))
#define EXYNOS_MCT_COMP_U(i)		(0x204 + ((i) * 0x100))
#define EXYNOS_MCT_COMP_MODE(i)		(0x208 + ((i) * 0x100))
#define EXYNOS_MCT_COMP_PERIOD(i)	(0x20C + ((i) * 0x100))
#define EXYNOS_MCT_COMP_ENABLE(i)	(0x210 + ((i) * 0x100))
#define EXYNOS_MCT_INT_ENB(i)		(0x214 + ((i) * 0x100))
#define EXYNOS_MCT_INT_CSTAT(i)		(0x218 + ((i) * 0x100))

#define MCT_FRC_ENABLE			0x1
#define MCT_COMP_ENABLE			0x1
#define MCT_COMP_DISABLE		0x0
#define MCT_COMP_CIRCULAR_MODE		0x1
#define MCT_COMP_NON_CIRCULAR_MODE	0x0
#define MCT_INT_ENABLE			0x1
#define MCT_INT_DISABLE			0x0
#define MCT_CSTAT_CLEAR			0x1
#define MCT_DIV_REQ_BIT			8

#define DEFAULT_RTC_CLK_RATE		32768
#define DEFAULT_CLK_DIV			3

/* The block has 12 comparators that can each raise an interrupt. */
#define MCT_NR_COMPS			12

/*
 * Deliberately below the arm64 architected timer (450), matching downstream:
 * the architected timer stays the per-CPU tick and the comparator is installed
 * as the per-CPU one-shot wakeup device that covers the c2 power-down state.
 * Raising this above 450 would make the MCT the tick instead, moving every tick
 * onto a GIC SPI for no benefit.
 */
#define MCT_CLKEVENTS_RATING		350
#define MCT_CLKSOURCE_RATING		350

struct mct_clock_event_device {
	struct clock_event_device evt;
	char name[10];
	unsigned int comp_index;
};

static void __iomem *reg_base;
static unsigned long osc_clk_rate;
static int mct_div;
static int mct_irqs[MCT_NR_COMPS];

static void exynos_mct_set_compensation(unsigned long osc, unsigned long rtc)
{
	unsigned int osc_rtc;
	unsigned int incr_rtcclk;
	unsigned int compen_val;

	osc_rtc = (unsigned int)(osc * 1000 / rtc);

	/* Integer part of (OSCCLK frequency / RTCCLK frequency). */
	incr_rtcclk = (osc / rtc) + ((osc % rtc) ? 1 : 0);

	/* Decimal part of (OSCCLK frequency / RTCCLK frequency). */
	compen_val = ((osc_rtc + 5) / 10) % 100;
	if (compen_val)
		compen_val = 100 - compen_val;

	pr_info("exynos-mct-v3: osc-%lu rtc-%lu incr_rtcclk:0x%08x compen_val:0x%08x\n",
		osc, rtc, incr_rtcclk, compen_val);

	writel_relaxed(incr_rtcclk, reg_base + EXYNOS_MCT_MCT_INCR_RTCCLK);
	writel_relaxed(compen_val, reg_base + EXYNOS_MCT_COMPENSATE_VALUE);
}

/* Clocksource handling */
static void exynos_mct_frc_start(int div)
{
	writel_relaxed(BIT(MCT_DIV_REQ_BIT) + ((div - 1) & 0xffUL),
		       reg_base + EXYNOS_MCT_MCT_CFG);
	writel_relaxed(MCT_FRC_ENABLE, reg_base + EXYNOS_MCT_MCT_FRC_ENABLE);
}

static u64 exynos_frc_read(struct clocksource *cs)
{
	return readl_relaxed(reg_base + EXYNOS_MCT_CNT_L);
}

static void exynos_frc_resume(struct clocksource *cs)
{
	exynos_mct_frc_start(mct_div);
}

static struct clocksource mct_frc = {
	.name		= "mct-frc",
	.rating		= MCT_CLKSOURCE_RATING,
	.read		= exynos_frc_read,
	.resume		= exynos_frc_resume,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int exynos_clocksource_init(void)
{
	if (clocksource_register_hz(&mct_frc, osc_clk_rate))
		panic("%s: can't register clocksource\n", mct_frc.name);

	return 0;
}

static inline int exynos_mct_comp_wait(int index, int comp_enable)
{
	unsigned int comp_stat;
	int i;

	for (i = 0; i < loops_per_jiffy / 1000 * HZ; i++) {
		comp_stat = readl_relaxed(reg_base + EXYNOS_MCT_COMP_ENABLE(index));
		if (comp_stat == comp_enable)
			return 1;
		cpu_relax();
	}

	return 0;
}

static void exynos_mct_comp_stop(struct mct_clock_event_device *mevt)
{
	unsigned int index = mevt->comp_index;

	writel_relaxed(MCT_COMP_DISABLE, reg_base + EXYNOS_MCT_COMP_ENABLE(index));

	/* Wait maximum 1 ms until COMP_ENABLE_n = 0 */
	if (!exynos_mct_comp_wait(index, MCT_COMP_DISABLE))
		panic("MCT(comp%d) disable timeout\n", index);

	writel_relaxed(MCT_COMP_NON_CIRCULAR_MODE, reg_base + EXYNOS_MCT_COMP_MODE(index));
	writel_relaxed(MCT_INT_DISABLE, reg_base + EXYNOS_MCT_INT_ENB(index));
	writel_relaxed(MCT_CSTAT_CLEAR, reg_base + EXYNOS_MCT_INT_CSTAT(index));
}

static void exynos_mct_comp_start(struct mct_clock_event_device *mevt,
				  bool periodic, unsigned long cycles)
{
	unsigned int index = mevt->comp_index;
	unsigned int comp_stat;

	comp_stat = readl_relaxed(reg_base + EXYNOS_MCT_COMP_ENABLE(index));
	if (comp_stat == MCT_COMP_ENABLE)
		exynos_mct_comp_stop(mevt);

	if (periodic)
		writel_relaxed(MCT_COMP_CIRCULAR_MODE, reg_base + EXYNOS_MCT_COMP_MODE(index));

	writel_relaxed(cycles, reg_base + EXYNOS_MCT_COMP_PERIOD(index));
	writel_relaxed(MCT_INT_ENABLE, reg_base + EXYNOS_MCT_INT_ENB(index));
	writel_relaxed(MCT_COMP_ENABLE, reg_base + EXYNOS_MCT_COMP_ENABLE(index));

	/* Wait maximum 1 ms until COMP_ENABLE_n = 1 */
	if (!exynos_mct_comp_wait(index, MCT_COMP_ENABLE))
		panic("MCT(comp%d) enable timeout\n", index);
}

static int exynos_comp_set_next_event(unsigned long cycles,
				      struct clock_event_device *evt)
{
	struct mct_clock_event_device *mevt;

	mevt = container_of(evt, struct mct_clock_event_device, evt);
	exynos_mct_comp_start(mevt, false, cycles);

	return 0;
}

static int mct_set_state_shutdown(struct clock_event_device *evt)
{
	struct mct_clock_event_device *mevt;

	mevt = container_of(evt, struct mct_clock_event_device, evt);
	exynos_mct_comp_stop(mevt);

	return 0;
}

static int mct_set_state_periodic(struct clock_event_device *evt)
{
	unsigned long cycles_per_jiffy;
	struct mct_clock_event_device *mevt;

	mevt = container_of(evt, struct mct_clock_event_device, evt);
	cycles_per_jiffy = (((unsigned long long)NSEC_PER_SEC / HZ * evt->mult) >> evt->shift);
	exynos_mct_comp_start(mevt, true, cycles_per_jiffy);

	return 0;
}

static irqreturn_t exynos_mct_comp_isr(int irq, void *dev_id)
{
	struct mct_clock_event_device *mevt = dev_id;
	struct clock_event_device *evt = &mevt->evt;
	unsigned int index = mevt->comp_index;

	writel_relaxed(MCT_CSTAT_CLEAR, reg_base + EXYNOS_MCT_INT_CSTAT(index));
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static DEFINE_PER_CPU(struct mct_clock_event_device, percpu_mct_tick);

static int exynos_mct_starting_cpu(unsigned int cpu)
{
	struct mct_clock_event_device *mevt = per_cpu_ptr(&percpu_mct_tick, cpu);
	struct clock_event_device *evt = &mevt->evt;

	snprintf(mevt->name, sizeof(mevt->name), "mct_comp%d", cpu);

	evt->name = mevt->name;
	evt->cpumask = cpumask_of(cpu);
	evt->set_next_event = exynos_comp_set_next_event;
	evt->set_state_periodic = mct_set_state_periodic;
	evt->set_state_shutdown = mct_set_state_shutdown;
	evt->set_state_oneshot = mct_set_state_shutdown;
	evt->set_state_oneshot_stopped = mct_set_state_shutdown;
	evt->tick_resume = mct_set_state_shutdown;
	evt->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT |
			CLOCK_EVT_FEAT_PERCPU;
	evt->rating = MCT_CLKEVENTS_RATING;

	/*
	 * No .suspend/.resume: the comparators are in the always-on MISC block
	 * and keep their state, and clockevents_suspend() runs from
	 * timekeeping_suspend() -- i.e. inside tick_freeze(), with interrupts
	 * off and the other CPUs already in c2.  Stopping all comparators there
	 * would busy-poll (and can panic) on the s2idle entry path for nothing.
	 */
	if (evt->irq == -1)
		return -EIO;

	irq_force_affinity(evt->irq, cpumask_of(cpu));
	enable_irq(evt->irq);

	/*
	 * max_delta 0x7fffffff cycles is ~87 s at 24.576 MHz, and that bound is
	 * load-bearing rather than arbitrary.  Unlike the broadcast path, which
	 * bails out when the tick device has no next event
	 * (___tick_broadcast_oneshot_control()), tick_oneshot_wakeup_control()
	 * programs the wakeup device unconditionally, and
	 * clockevents_program_event() clamps to max_delta.  So a fully idle CPU
	 * arms this comparator ~87 s out instead of not at all, and wakes once
	 * per period.  Negligible for power, but it is why an idle core is not
	 * completely silent -- and it bounds how late a comparator left armed
	 * across s2idle entry can fire.
	 */
	clockevents_config_and_register(evt, osc_clk_rate, 0xf, 0x7fffffff);

	return 0;
}

static int exynos_mct_dying_cpu(unsigned int cpu)
{
	struct mct_clock_event_device *mevt = per_cpu_ptr(&percpu_mct_tick, cpu);
	struct clock_event_device *evt = &mevt->evt;
	unsigned int index = mevt->comp_index;

	evt->set_state_shutdown(evt);
	if (evt->irq != -1)
		disable_irq_nosync(evt->irq);

	writel_relaxed(MCT_CSTAT_CLEAR, reg_base + EXYNOS_MCT_INT_CSTAT(index));

	return 0;
}

static int __init exynos_timer_resources(struct device_node *np)
{
	struct clk *mct_clk, *tick_clk, *rtc_clk;
	unsigned long rtc_clk_rate;
	int err, cpu;

	if (of_property_read_u32(np, "div", &mct_div) || !mct_div)
		mct_div = DEFAULT_CLK_DIV;

	tick_clk = of_clk_get_by_name(np, "fin_pll");
	if (IS_ERR(tick_clk))
		panic("exynos-mct-v3: unable to determine tick clock rate\n");
	osc_clk_rate = clk_get_rate(tick_clk) / mct_div;

	mct_clk = of_clk_get_by_name(np, "mct");
	if (IS_ERR(mct_clk))
		panic("exynos-mct-v3: unable to retrieve mct clock instance\n");
	clk_prepare_enable(mct_clk);

	rtc_clk = of_clk_get_by_name(np, "rtc");
	if (IS_ERR(rtc_clk))
		rtc_clk_rate = DEFAULT_RTC_CLK_RATE;
	else
		rtc_clk_rate = clk_get_rate(rtc_clk);

	exynos_mct_set_compensation(osc_clk_rate, rtc_clk_rate);
	exynos_mct_frc_start(mct_div);

	for_each_possible_cpu(cpu) {
		struct mct_clock_event_device *pcpu_mevt =
			per_cpu_ptr(&percpu_mct_tick, cpu);
		int mct_irq;

		if (WARN_ON(cpu >= ARRAY_SIZE(mct_irqs)))
			break;

		mct_irq = mct_irqs[cpu];
		pcpu_mevt->evt.irq = -1;
		pcpu_mevt->comp_index = cpu;

		irq_set_status_flags(mct_irq, IRQ_NOAUTOEN);
		if (request_irq(mct_irq, exynos_mct_comp_isr,
				IRQF_TIMER | IRQF_NOBALANCING | IRQF_PERCPU,
				"exynos-mct-v3", pcpu_mevt)) {
			pr_err("exynos-mct-v3: cannot register IRQ (cpu%d)\n", cpu);
			continue;
		}
		pcpu_mevt->evt.irq = mct_irq;
	}

	/*
	 * Install hotplug callbacks which configure the timer on this CPU.
	 *
	 * CPUHP_AP_EXYNOS4_MCT_TIMER_STARTING runs before
	 * CPUHP_AP_ARM_ARCH_TIMER_STARTING, so on a CPU coming up the comparator
	 * is registered first and briefly becomes that CPU's tick device.  The
	 * architected timer then displaces it, and clockevents_register_device()
	 * re-offers the released device to tick_check_new_device() via
	 * clockevents_notify_released(), which is where it lands as the one-shot
	 * wakeup device.  So the ordering is harmless, but the wakeup device is
	 * only installed on that second pass -- do not "optimise" the release
	 * path away.
	 */
	err = cpuhp_setup_state(CPUHP_AP_EXYNOS4_MCT_TIMER_STARTING,
				"clockevents/exynos/mct_timer_v3:starting",
				exynos_mct_starting_cpu, exynos_mct_dying_cpu);
	if (err)
		goto out_irq;

	return 0;

out_irq:
	for_each_possible_cpu(cpu) {
		struct mct_clock_event_device *pcpu_mevt =
			per_cpu_ptr(&percpu_mct_tick, cpu);

		if (pcpu_mevt->evt.irq != -1) {
			free_irq(pcpu_mevt->evt.irq, pcpu_mevt);
			pcpu_mevt->evt.irq = -1;
		}
	}
	return err;
}

static int __init mct_init_dt(struct device_node *np)
{
	struct of_phandle_args irq;
	int nr_irqs = 0, i, ret;

	/* Count the interrupts the comparators can produce. */
	while (of_irq_parse_one(np, nr_irqs, &irq) == 0)
		nr_irqs++;

	if (nr_irqs > ARRAY_SIZE(mct_irqs)) {
		pr_err("exynos-mct-v3: too many (%d) interrupts configured in DT\n",
		       nr_irqs);
		nr_irqs = ARRAY_SIZE(mct_irqs);
	}
	for (i = 0; i < nr_irqs; i++)
		mct_irqs[i] = irq_of_parse_and_map(np, i);

	reg_base = of_iomap(np, 0);
	if (!reg_base)
		panic("exynos-mct-v3: unable to ioremap mct address space\n");

	ret = exynos_timer_resources(np);
	if (ret)
		return ret;

	return exynos_clocksource_init();
}

TIMER_OF_DECLARE(exynos_mct_v3, "samsung,exynos-mct-v3", mct_init_dt);
