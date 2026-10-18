/* SPDX-License-Identifier: GPL-2.0
 *
 * CS40L26 Boosted Haptic Driver with Integrated DSP and Waveform Memory
 *
 * Copyright 2024 Cirrus Logic, Inc.
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 *
 * Modelled on the CS40L50 driver by James Ogletree <james.ogletree@cirrus.com>.
 * The two parts share the haptic programming model -- a Halo DSP holding a
 * waveform table, driven by writing a waveform index to a mailbox -- but not
 * the register map: CS40L26 places its control registers where CS35L41 does.
 */

#ifndef __MFD_CS40L26_H__
#define __MFD_CS40L26_H__

#include <linux/bits.h>
#include <linux/firmware/cirrus/cs_dsp.h>
#include <linux/gpio/consumer.h>
#include <linux/pm.h>
#include <linux/regmap.h>

/* Device identification */
#define CS40L26_DEVID			0x0
#define CS40L26_REVID			0x4
#define CS40L26_DEVID_MASK		GENMASK(23, 0)
#define CS40L26_REVID_MASK		GENMASK(7, 0)
#define CS40L26_DEVID_L26_A		0x40A260
#define CS40L26_DEVID_L26_B		0x40A26B
#define CS40L26_DEVID_L27_A		0x40A270
#define CS40L26_DEVID_L27_B		0x40A27B
#define CS40L26_REVID_A1		0xA1
#define CS40L26_REVID_B0		0xB0
#define CS40L26_REVID_B1		0xB1
#define CS40L26_REVID_B2		0xB2

/* Interrupts */
#define CS40L26_IRQ1_EINT_1		0x10010
#define CS40L26_IRQ1_EINT_2		0x10014
#define CS40L26_IRQ1_MASK_1		0x10110
#define CS40L26_IRQ1_MASK_2		0x10114

/*
 * Mailbox.  Commands are written here and the firmware zeroes the register to
 * acknowledge them; the queue the CS40L50 uses for the same purpose does not
 * exist on this part, where that address range holds the GPIO block instead.
 */
#define CS40L26_DSP_VIRTUAL1_MBOX_1	0x13020

/* Mailbox commands */
#define CS40L26_PREVENT_HIBER		0x02000003
#define CS40L26_ALLOW_HIBER		0x02000004
#define CS40L26_SHUTDOWN		0x02000005
#define CS40L26_STOP_PLAYBACK		0x05000000

/* DSP */
#define CS40L26_XMEM_PACKED_0		0x2000000
#define CS40L26_SYS_INFO_ID		0x25E0000
#define CS40L26_XMEM_UNPACKED24_0	0x2800000
#define CS40L26_CORE_BASE		0x2B80000
#define CS40L26_CCM_CORE_CONTROL	0x2BC1000
#define CS40L26_YMEM_PACKED_0		0x2C00000
#define CS40L26_YMEM_UNPACKED24_0	0x3400000
#define CS40L26_PMEM_0			0x3800000

/*
 * Writing CCM_CORE_CONTROL directly rather than letting cs_dsp start the core:
 * cs_dsp_halo_start_core() drives only HALO_CORE_RESET | HALO_CORE_EN, and
 * this part additionally requires BIT(7) to be held across both the stopped
 * and the running state.
 */
#define CS40L26_CCM_CORE_KILL		0x00000080
#define CS40L26_CCM_CORE_RESET		0x00000281

/* DSP state, as reported by the power management algorithm */
#define CS40L26_DSP_STATE_HIBERNATE	0
#define CS40L26_DSP_STATE_SHUTDOWN	1
#define CS40L26_DSP_STATE_STANDBY	2
#define CS40L26_DSP_STATE_ACTIVE	3
#define CS40L26_DSP_STATE_MASK		GENMASK(7, 0)
#define CS40L26_DSP_HALO_STATE_RUN	2

/* Timings */
#define CS40L26_RESET_PULSE_US		1500
#define CS40L26_CP_READY_US		6000
#define CS40L26_DSP_POLL_US		1000
#define CS40L26_DSP_TIMEOUT_COUNT	100
#define CS40L26_AUTOSUSPEND_MS		2000

/* Firmware files */
#define CS40L26_FW			"cs40l26.wmfw"
#define CS40L26_WT			"cs40l26.bin"

/*
 * Addresses of the ROM firmware's state variables.  The ROM image differs
 * between silicon revisions, so these move; they are only needed before the
 * RAM patch is loaded, after which the same values are reachable as firmware
 * controls.
 */
struct cs40l26_rom_regs {
	u32 pm_cur_state;
	u32 dsp_halo_state;
};

struct cs40l26 {
	struct device *dev;
	struct regmap *regmap;
	/* Serialises firmware bring-up against the DSP's run state */
	struct mutex lock;
	struct cs_dsp dsp;
	struct gpio_desc *reset_gpio;
	const struct firmware *fw;
	const struct firmware *bin;
	const struct cs40l26_rom_regs *rom_regs;
	u32 devid;
	u32 revid;
};

int cs40l26_dsp_write(struct device *dev, struct regmap *regmap, u32 val);
int cs40l26_probe(struct cs40l26 *cs40l26);

extern const struct regmap_config cs40l26_regmap;
extern const struct dev_pm_ops cs40l26_pm_ops;

#endif /* __MFD_CS40L26_H__ */
