// SPDX-License-Identifier: GPL-2.0
/*
 * force_panic.c - debug aid: panic the kernel a fixed time after boot.
 *
 * Enable via the kernel command line, e.g. "force_panic_secs=30".
 * On expiry the kernel panics, which (with ramoops/pstore configured) flushes
 * the dmesg ring buffer to console-ramoops for offline inspection. This makes
 * it possible to deterministically capture a full boot log on hardware where a
 * live console is unavailable.
 *
 * This is a temporary debugging facility; remove once no longer needed.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

static unsigned int force_panic_secs;

static int __init force_panic_setup(char *str)
{
	if (kstrtouint(str, 0, &force_panic_secs))
		force_panic_secs = 0;
	return 1;
}
__setup("force_panic_secs=", force_panic_setup);

static void force_panic_fn(struct timer_list *t)
{
	panic("force_panic: %u seconds elapsed since boot\n", force_panic_secs);
}

static DEFINE_TIMER(force_panic_timer, force_panic_fn);

static int __init force_panic_init(void)
{
	if (force_panic_secs) {
		pr_warn("force_panic: will panic %u seconds after boot\n",
			force_panic_secs);
		mod_timer(&force_panic_timer,
			  jiffies + msecs_to_jiffies(force_panic_secs * 1000U));
	}
	return 0;
}
late_initcall(force_panic_init);
