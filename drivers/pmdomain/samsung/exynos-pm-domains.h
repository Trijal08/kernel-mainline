/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Power sequences for Samsung Exynos power domains.
 *
 * Copyright (c) 2026 Steffen Deusch
 */

#ifndef __EXYNOS_PM_DOMAINS_H
#define __EXYNOS_PM_DOMAINS_H

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/limits.h>
#include <linux/types.h>

/* The register windows a domain node names, in "reg-names" order */
enum exynos_pd_window {
	EXYNOS_PD_PMU,		/* the domain's block in the PMU */
	EXYNOS_PD_CMU,		/* the clock controller inside the domain */
	EXYNOS_PD_SYSREG,	/* the system registers inside the domain */
	EXYNOS_PD_NR_WINDOWS,
};

enum exynos_pd_op {
	/* Write (reg & ~mask) | (value & mask) */
	EXYNOS_PD_OP_WRITE,
	/* Poll until (reg & mask) == value */
	EXYNOS_PD_OP_WAIT,
	/* Save reg & mask before power-off, write it back after power-on */
	EXYNOS_PD_OP_SAVE,
	/* Skip the next step if (reg & mask) == value */
	EXYNOS_PD_OP_SKIP_IF,
	/* PMU only: set the bits in value through the register's set-bit alias */
	EXYNOS_PD_OP_SET_BITS,
	/* Wait value microseconds */
	EXYNOS_PD_OP_DELAY,
};

struct exynos_pd_step {
	u8 op;
	u8 window;
	u32 offset;
	u32 mask;
	u32 value;
};

/*
 * The vendor's power sequences for one domain: @on brings it up, @off takes
 * it down, and @save lists what is saved before @off and written back after
 * @on, in order, together with the writes and waits that go with the
 * restore.  A table of these ends with an entry whose @pmu is zero.
 */
struct exynos_pd_sequences {
	/* Physical address of the domain's CONFIGURATION register */
	phys_addr_t pmu;
	const struct exynos_pd_step *on;
	const struct exynos_pd_step *save;
	const struct exynos_pd_step *off;
	unsigned int nr_on;
	unsigned int nr_save;
	unsigned int nr_off;
};

#define EXYNOS_PD_STEP(_op, _window, _offset, _mask, _value) {	\
	.op = EXYNOS_PD_OP_##_op,					\
	.window = EXYNOS_PD_##_window,					\
	.offset = _offset,						\
	.mask = _mask,							\
	.value = _value,						\
}

#define PD_WRITE(_win, _off, _mask, _val)				\
	EXYNOS_PD_STEP(WRITE, _win, _off, _mask, _val)
#define PD_WAIT(_win, _off, _mask, _val)				\
	EXYNOS_PD_STEP(WAIT, _win, _off, _mask, _val)
#define PD_SAVE(_win, _off)						\
	EXYNOS_PD_STEP(SAVE, _win, _off, U32_MAX, 0)
#define PD_SKIP_IF(_win, _off, _mask, _val)				\
	EXYNOS_PD_STEP(SKIP_IF, _win, _off, _mask, _val)
#define PD_SET_BITS(_off, _val)						\
	EXYNOS_PD_STEP(SET_BITS, PMU, _off, U32_MAX, _val)
#define PD_DELAY_US(_us)						\
	EXYNOS_PD_STEP(DELAY, PMU, 0, 0, _us)

#define EXYNOS_PD_SEQUENCES(_pmu, _on, _save, _off) {			\
	.pmu = _pmu,							\
	.on = _on,							\
	.save = _save,							\
	.off = _off,							\
	.nr_on = ARRAY_SIZE(_on),					\
	.nr_save = ARRAY_SIZE(_save),					\
	.nr_off = ARRAY_SIZE(_off),					\
}

#define EXYNOS_PD_SEQUENCES_NO_SAVE(_pmu, _on, _off) {			\
	.pmu = _pmu,							\
	.on = _on,							\
	.off = _off,							\
	.nr_on = ARRAY_SIZE(_on),					\
	.nr_off = ARRAY_SIZE(_off),					\
}

extern const struct exynos_pd_sequences zumapro_pd_sequences[];

#endif /* __EXYNOS_PM_DOMAINS_H */
