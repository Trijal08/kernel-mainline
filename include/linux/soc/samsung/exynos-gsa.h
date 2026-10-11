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
 * gsa_gsc_nos_call() - call a Titan/GSC application through the GSA
 * @gsa: the bound GSA device; the caller must keep it bound during the call
 * @app_id: NOS application ID
 * @params: application-specific command parameter
 * @args: request data (may be NULL when @arg_len is zero)
 * @arg_len: request length, at most 4096 bytes
 * @reply: response buffer (may be NULL when its capacity is zero)
 * @reply_len: response capacity on entry, actual length on success (max 4096)
 * @call_status: NOS application status on success, zero means APP_SUCCESS
 *
 * Sleeps. Request/reply data is copied through a coherent DMA buffer allocated
 * on the GSA device. Calls share the mailbox with AoC operations. Output lengths
 * and status are unchanged on transport errors; a nonzero application status is
 * returned separately from the transport result. A timed-out call disables
 * further GSC calls and quarantines the DMA buffer until reboot.
 *
 * Return: 0 for a completed call (inspect @call_status), or a negative errno.
 */
int gsa_gsc_nos_call(struct device *gsa, u8 app_id, u16 params,
		     const void *args, size_t arg_len, void *reply,
		     size_t *reply_len, u32 *call_status);

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

/**
 * gsa_aoc_start() - ask the GSA to take the AOC out of reset
 * @gsa: the GSA device
 *
 * The AOC's reset lives in the secure domain, so the non-secure AP cannot
 * release it directly; the GSA does.  Call only after gsa_load_aoc_fw_image()
 * has succeeded.
 *
 * Return: 0 on success, or a negative errno.
 */
int gsa_aoc_start(struct device *gsa);

/**
 * gsa_aoc_get_state() - query the AOC's state from the GSA
 * @gsa: the GSA device
 *
 * Return: the GSA-reported AOC state (>= 0), or a negative errno.
 */
int gsa_aoc_get_state(struct device *gsa);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_GSA_H */
