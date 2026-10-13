/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google AOC - the seam where the core is brought out of reset.
 *
 * The GSA security core authenticates the AOC image over its own mailbox, but
 * nothing in that mailbox protocol releases the core: the reset and the AOC's
 * secure SysMMU context live in the secure domain, where the AP cannot reach
 * them. Only a secure agent can, and reaching it means talking to a TEE.
 *
 * That is a different concern from staging firmware and running the IPC, and
 * the TEE involved is not necessarily one the kernel has a driver for, so the
 * rest of this driver does not know how the core starts. It asks a backend.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#ifndef _GOOGLE_AOC_BOOT_H
#define _GOOGLE_AOC_BOOT_H

struct device;

/**
 * struct aoc_boot_backend - brings the AOC core out of reset
 * @name:	identifies the backend in log messages
 * @attach:	acquire what the backend needs and return its private data, or
 *		an ERR_PTR. Called once from probe. A backend whose transport
 *		may not be up yet should connect lazily instead of here.
 * @detach:	release what @attach acquired.
 * @get_state:	the core's state as the secure side reports it, or a negative
 *		errno.
 * @start:	set the core up and release it from reset. Returns the state
 *		the secure side reports afterwards, or a negative errno.
 */
struct aoc_boot_backend {
	const char *name;
	void *(*attach)(struct device *dev);
	void (*detach)(void *priv);
	int (*get_state)(void *priv);
	int (*start)(void *priv);
};

/*
 * Declared unconditionally: the core selects it with if (IS_ENABLED(...)),
 * which keeps the branch compiled -- and so needs the declaration -- while
 * leaving no reference behind for the linker when it is not built.
 */
extern const struct aoc_boot_backend aoc_boot_trusty;

#endif /* _GOOGLE_AOC_BOOT_H */
