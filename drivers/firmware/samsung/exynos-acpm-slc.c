// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos ACPM System Level Cache (SLC) partition protocol.
 *
 * The SLC is a shared last-level cache carved into partitions ("pt"). Bus
 * masters (GPU, codecs, ...) request a partition and tag their traffic with
 * the partition's PBHA so the SLC caches it. Partitions are created, enabled
 * and queried through the APM firmware over a dedicated ACPM IPC channel using
 * an 8-word message.
 *
 * Copyright 2019 Google LLC.
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/array_size.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/string.h>
#include <linux/types.h>

#include "exynos-acpm.h"
#include "exynos-acpm-slc.h"

#define ACPM_SLC_CMD_WORDS	8

/**
 * acpm_slc_request() - issue one SLC partition command to the APM firmware.
 * @handle:		ACPM protocol handle.
 * @acpm_chan_id:	SLC ACPM channel identifier.
 * @command:		command opcode (PT_ENABLE/PT_DISABLE/PT_VERSION/...).
 * @arg:		command argument (word 1).
 * @arg1:		second command argument (word 3).
 * @reply:		optional 8-word buffer to receive the full response.
 *
 * The firmware returns its primary result in word 1 of the response. The
 * remaining words carry command-specific payload and are copied to @reply when
 * provided.
 *
 * Return: the firmware response word 1 on success, a negative errno otherwise.
 */
int acpm_slc_request(struct acpm_handle *handle, unsigned int acpm_chan_id,
		     u32 command, u32 arg, u32 arg1, u32 reply[8])
{
	u32 cmd[ACPM_SLC_CMD_WORDS] = { 0, arg, command, arg1 };
	struct acpm_xfer xfer = {0};
	int ret;

	acpm_set_xfer(&xfer, cmd, ARRAY_SIZE(cmd), acpm_chan_id, true);

	ret = acpm_do_xfer(handle, &xfer);
	if (ret)
		return ret;

	if (reply)
		memcpy(reply, xfer.rxd, sizeof(cmd));

	return xfer.rxd[1];
}
