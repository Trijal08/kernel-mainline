/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos modem (CP) power/reset sequencer
 *
 * Copyright 2026 Trijal Saha
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_CP_POWER_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_CP_POWER_H

#include <linux/err.h>
#include <linux/types.h>

struct device;
struct exynos_cp_power;

#if IS_ENABLED(CONFIG_EXYNOS_CP_POWER)
/*
 * Resolve the "google,cp-power" phandle on @consumer's node to its CP-power
 * sequencer.  Returns ERR_PTR(-EPROBE_DEFER) until the sequencer has bound.
 */
struct exynos_cp_power *exynos_cp_power_get(struct device *consumer);

/*
 * Warm-reset the CP into its boot ROM (rails + PMIC/DCXO patch); MAIN survives.
 * @dump latches AP2CP_DUMP_NOTI across the reset so the ROM starts its minidump
 * agent, which decodes the CP's crash record into srinfo as ASCII.
 */
int exynos_cp_power_warm_reset(struct exynos_cp_power *cp, bool dump);

/* Full cold power cycle of the CP rails (GPIO half; wipes the CP's DRAM). */
int exynos_cp_power_cold_cycle(struct exynos_cp_power *cp);

/*
 * Carry the AP's sleep state to the CP over AP2CP_PDA_ACTIVE.  Does not sleep,
 * so it is callable from noirq system-sleep callbacks.
 */
int exynos_cp_power_set_ap_active(struct exynos_cp_power *cp, bool active);
#else
static inline struct exynos_cp_power *exynos_cp_power_get(struct device *consumer)
{
	return ERR_PTR(-ENODEV);
}

static inline int exynos_cp_power_warm_reset(struct exynos_cp_power *cp,
					     bool dump)
{
	return -ENODEV;
}

static inline int exynos_cp_power_cold_cycle(struct exynos_cp_power *cp)
{
	return -ENODEV;
}

static inline int exynos_cp_power_set_ap_active(struct exynos_cp_power *cp,
						bool active)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_CP_POWER_H */
