// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011, 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Exynos MCT (Multi-Core Timer) v3 support.
 *
 * The v3 MCT block provides a free-running counter (FRC) and a set of
 * comparators.  Unlike the older exynos4210-mct global/local timer layout, the
 * v3 block has no dedicated global comparator; each comparator raises its own
 * interrupt.  On Google Tensor (Zuma/Zumapro) this is the only MCT block whose
 * interrupts are actually wired -- the legacy mct@10050000 IRQs are routed from
 * this block, so the exynos4210-mct driver cannot drive them.
 *
 * The per-CPU arm64 architected timer is used as the tick.  It stops in the c2
 * power-down idle state (the cpuidle states declare local-timer-stop), so the
 * tick framework needs a broadcast clockevent to wake a CPU out of c2.  The MCT
 * lives in the always-on MISC block, so one of its comparators is registered as
 * a single global one-shot broadcast device: it keeps counting through c2 and
 * its interrupt wakes a CPU to deliver the broadcast.  The FRC is registered as
 * a clocksource.
 */

#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
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

/* Comparator used as the global broadcast clockevent (its IRQ is DT index 0). */
#define MCT_COMP_BROADCAST		0

/*
 * The broadcast device must sit below the arm64 architected timer (rating 450)
 * so the latter stays the per-CPU tick; the tick framework then selects the MCT
 * comparator as the broadcast device when a CPU's architected timer stops in c2.
 */
#define MCT_CLKEVENTS_RATING		250
#define MCT_CLKSOURCE_RATING		350

static void __iomem *reg_base;
static unsigned long osc_clk_rate;
static int mct_div;
static int mct_comp_irq;

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

static inline int exynos_mct_comp_wait(int comp_enable)
{
	unsigned int comp_stat;
	int i;

	for (i = 0; i < loops_per_jiffy / 1000 * HZ; i++) {
		comp_stat = readl_relaxed(reg_base +
					  EXYNOS_MCT_COMP_ENABLE(MCT_COMP_BROADCAST));
		if (comp_stat == comp_enable)
			return 1;
		cpu_relax();
	}

	return 0;
}

static void exynos_mct_comp_stop(void)
{
	writel_relaxed(MCT_COMP_DISABLE,
		       reg_base + EXYNOS_MCT_COMP_ENABLE(MCT_COMP_BROADCAST));

	/* Wait maximum 1 ms until COMP_ENABLE_n = 0 */
	if (!exynos_mct_comp_wait(MCT_COMP_DISABLE))
		panic("MCT(comp%d) disable timeout\n", MCT_COMP_BROADCAST);

	writel_relaxed(MCT_COMP_NON_CIRCULAR_MODE,
		       reg_base + EXYNOS_MCT_COMP_MODE(MCT_COMP_BROADCAST));
	writel_relaxed(MCT_INT_DISABLE,
		       reg_base + EXYNOS_MCT_INT_ENB(MCT_COMP_BROADCAST));
	writel_relaxed(MCT_CSTAT_CLEAR,
		       reg_base + EXYNOS_MCT_INT_CSTAT(MCT_COMP_BROADCAST));
}

static void exynos_mct_comp_start(bool periodic, unsigned long cycles)
{
	unsigned int comp_stat;

	comp_stat = readl_relaxed(reg_base +
				  EXYNOS_MCT_COMP_ENABLE(MCT_COMP_BROADCAST));
	if (comp_stat == MCT_COMP_ENABLE)
		exynos_mct_comp_stop();

	if (periodic)
		writel_relaxed(MCT_COMP_CIRCULAR_MODE,
			       reg_base + EXYNOS_MCT_COMP_MODE(MCT_COMP_BROADCAST));

	writel_relaxed(cycles, reg_base + EXYNOS_MCT_COMP_PERIOD(MCT_COMP_BROADCAST));
	writel_relaxed(MCT_INT_ENABLE,
		       reg_base + EXYNOS_MCT_INT_ENB(MCT_COMP_BROADCAST));
	writel_relaxed(MCT_COMP_ENABLE,
		       reg_base + EXYNOS_MCT_COMP_ENABLE(MCT_COMP_BROADCAST));

	/* Wait maximum 1 ms until COMP_ENABLE_n = 1 */
	if (!exynos_mct_comp_wait(MCT_COMP_ENABLE))
		panic("MCT(comp%d) enable timeout\n", MCT_COMP_BROADCAST);
}

static int exynos_comp_set_next_event(unsigned long cycles,
				      struct clock_event_device *evt)
{
	exynos_mct_comp_start(false, cycles);

	return 0;
}

static int mct_set_state_shutdown(struct clock_event_device *evt)
{
	exynos_mct_comp_stop();

	return 0;
}

static int mct_set_state_periodic(struct clock_event_device *evt)
{
	unsigned long cycles_per_jiffy;

	cycles_per_jiffy = (((unsigned long long)NSEC_PER_SEC / HZ * evt->mult) >> evt->shift);
	exynos_mct_comp_start(true, cycles_per_jiffy);

	return 0;
}

static struct clock_event_device mct_comp_device = {
	.name			= "mct-comp",
	.features		= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.rating			= MCT_CLKEVENTS_RATING,
	.set_next_event		= exynos_comp_set_next_event,
	.set_state_periodic	= mct_set_state_periodic,
	.set_state_shutdown	= mct_set_state_shutdown,
	.set_state_oneshot	= mct_set_state_shutdown,
	.set_state_oneshot_stopped = mct_set_state_shutdown,
	.tick_resume		= mct_set_state_shutdown,
};

static irqreturn_t exynos_mct_comp_isr(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	writel_relaxed(MCT_CSTAT_CLEAR,
		       reg_base + EXYNOS_MCT_INT_CSTAT(MCT_COMP_BROADCAST));
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int __init exynos_clockevent_init(void)
{
	/*
	 * Registered for CPU0 but with a rating below the architected timer, so
	 * it is never used as the per-CPU tick; the tick framework picks it up
	 * as the (non-per-CPU) broadcast device instead.
	 */
	mct_comp_device.cpumask = cpumask_of(0);
	clockevents_config_and_register(&mct_comp_device, osc_clk_rate,
					0xf, 0x7fffffff);
	if (request_irq(mct_comp_irq, exynos_mct_comp_isr,
			IRQF_TIMER | IRQF_IRQPOLL, "mct_comp_irq",
			&mct_comp_device))
		pr_err("exynos-mct-v3: request_irq() failed for the broadcast comparator\n");

	return 0;
}

static int __init exynos_timer_resources(struct device_node *np)
{
	struct clk *mct_clk, *tick_clk, *rtc_clk;
	unsigned long rtc_clk_rate;

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

	return 0;
}

static int __init mct_init_dt(struct device_node *np)
{
	int ret;

	mct_comp_irq = irq_of_parse_and_map(np, MCT_COMP_BROADCAST);
	if (!mct_comp_irq) {
		pr_err("exynos-mct-v3: unable to map the broadcast comparator IRQ\n");
		return -EINVAL;
	}

	reg_base = of_iomap(np, 0);
	if (!reg_base)
		panic("exynos-mct-v3: unable to ioremap mct address space\n");

	ret = exynos_timer_resources(np);
	if (ret)
		return ret;

	ret = exynos_clocksource_init();
	if (ret)
		return ret;

	return exynos_clockevent_init();
}

TIMER_OF_DECLARE(exynos_mct_v3, "samsung,exynos-mct-v3", mct_init_dt);
