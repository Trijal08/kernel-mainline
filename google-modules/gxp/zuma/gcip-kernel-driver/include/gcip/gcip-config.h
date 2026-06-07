/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Define configuration macros.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __GCIP_CONFIG_H__
#define __GCIP_CONFIG_H__


/* Macros to check the availability of features and APIs */

/* NOTE(mainline): iommu_map()/iommu_map_sg() require the gfp argument. */
#define GCIP_IOMMU_MAP_HAS_GFP 1

#endif /* __GCIP_CONFIG_H__ */
