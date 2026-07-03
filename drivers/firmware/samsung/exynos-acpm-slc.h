/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2019 Google LLC.
 * Copyright 2025 Linaro Ltd.
 */
#ifndef __EXYNOS_ACPM_SLC_H__
#define __EXYNOS_ACPM_SLC_H__

#include <linux/types.h>

struct acpm_handle;

int acpm_slc_request(struct acpm_handle *handle, unsigned int acpm_chan_id,
		     u32 command, u32 arg, u32 arg1, u32 reply[8]);

#endif /* __EXYNOS_ACPM_SLC_H__ */
