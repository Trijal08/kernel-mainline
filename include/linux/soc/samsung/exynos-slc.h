/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2019 Google LLC.
 * Copyright 2025 Linaro Ltd.
 *
 * Client interface to the Exynos/Google System Level Cache (SLC) partition
 * manager. A bus master (e.g. the GPU) obtains a partition by name, enables it,
 * and tags its memory traffic with the returned PBHA so the SLC caches it.
 */
#ifndef __SOC_SAMSUNG_EXYNOS_SLC_H__
#define __SOC_SAMSUNG_EXYNOS_SLC_H__

#include <linux/err.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;
struct exynos_slc_partition;

#if IS_ENABLED(CONFIG_EXYNOS_SLC)
struct exynos_slc_partition *
devm_exynos_slc_partition_get(struct device *dev, const char *name);
int exynos_slc_partition_enable(struct exynos_slc_partition *part);
void exynos_slc_partition_disable(struct exynos_slc_partition *part);
int exynos_slc_partition_pbha(struct exynos_slc_partition *part);
#else
static inline struct exynos_slc_partition *
devm_exynos_slc_partition_get(struct device *dev, const char *name)
{
	return ERR_PTR(-ENODEV);
}

static inline int exynos_slc_partition_enable(struct exynos_slc_partition *part)
{
	return -ENODEV;
}

static inline void
exynos_slc_partition_disable(struct exynos_slc_partition *part) { }

static inline int exynos_slc_partition_pbha(struct exynos_slc_partition *part)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS_SLC_H__ */
