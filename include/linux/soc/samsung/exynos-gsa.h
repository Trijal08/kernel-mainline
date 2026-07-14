/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface to the Google GSA (Google Security Algorithm subsystem) security
 * core on Tensor SoCs, for drivers that need it to authenticate and load a
 * coprocessor's firmware.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_GSA_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_GSA_H

#include <linux/types.h>

struct device;

/**
 * gsa_load_aoc_fw_image() - have the GSA authenticate and load an AOC image
 * @gsa:    the GSA device (resolve it from a "gsa-device" phandle)
 * @header: DMA address of the image's authentication header (in coherent memory
 *          the GSA can read by DMA)
 * @body:   physical address of the image body, staged where the GSA can read it
 *          (the AOC DRAM carveout)
 *
 * Return: 0 on success, or a negative errno (-EACCES on authentication failure,
 * -ENODEV if the GSA is not ready).
 */
int gsa_load_aoc_fw_image(struct device *gsa, dma_addr_t header, phys_addr_t body);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_GSA_H */
