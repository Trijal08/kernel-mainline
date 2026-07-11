/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos modem (CP) power/reset sequencer
 *
 * Copyright 2026 Trijal Saha
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_CP_POWER_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_CP_POWER_H

#include <linux/err.h>

struct device;
struct exynos_cp_power;

#if IS_ENABLED(CONFIG_EXYNOS_CP_POWER)
/*
 * Resolve the "google,cp-power" phandle on @consumer's node to its CP-power
 * sequencer.  Returns ERR_PTR(-EPROBE_DEFER) until the sequencer has bound.
 */
struct exynos_cp_power *exynos_cp_power_get(struct device *consumer);

/* Warm-reset the CP into its boot ROM (rails + PMIC/DCXO patch); MAIN survives. */
int exynos_cp_power_warm_reset(struct exynos_cp_power *cp);

/* Full cold power cycle of the CP rails (GPIO half; wipes the CP's DRAM). */
int exynos_cp_power_cold_cycle(struct exynos_cp_power *cp);
#else
static inline struct exynos_cp_power *exynos_cp_power_get(struct device *consumer)
{
	return ERR_PTR(-ENODEV);
}

static inline int exynos_cp_power_warm_reset(struct exynos_cp_power *cp)
{
	return -ENODEV;
}

static inline int exynos_cp_power_cold_cycle(struct exynos_cp_power *cp)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_CP_POWER_H */
