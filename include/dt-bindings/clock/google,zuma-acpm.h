/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright 2025 Linaro Ltd.
 *
 * Device Tree binding constants for Google zuma ACPM clock controller.
 */

#ifndef _DT_BINDINGS_CLOCK_GOOGLE_ZUMA_ACPM_H
#define _DT_BINDINGS_CLOCK_GOOGLE_ZUMA_ACPM_H

/*
 * zuma has a different set of ACPM DVFS domains than gs101: it inserts DSU and
 * BCI after the CPU clusters, which shifts G3D/G3DL2 (and everything after)
 * relative to gs101.  The IDs must match the firmware's domain numbering.
 */
#define GS101_CLK_ACPM_DVFS_MIF				0
#define GS101_CLK_ACPM_DVFS_INT				1
#define GS101_CLK_ACPM_DVFS_CPUCL0			2
#define GS101_CLK_ACPM_DVFS_CPUCL1			3
#define GS101_CLK_ACPM_DVFS_CPUCL2			4
#define GS101_CLK_ACPM_DVFS_DSU				5
#define GS101_CLK_ACPM_DVFS_BCI				6
#define GS101_CLK_ACPM_DVFS_G3D				7
#define GS101_CLK_ACPM_DVFS_G3DL2			8
#define GS101_CLK_ACPM_DVFS_TPU				9
#define GS101_CLK_ACPM_DVFS_INTCAM			10
#define GS101_CLK_ACPM_DVFS_TNR				11
#define GS101_CLK_ACPM_DVFS_CAM				12
#define GS101_CLK_ACPM_DVFS_MFC				13
#define GS101_CLK_ACPM_DVFS_DISP			14
#define GS101_CLK_ACPM_DVFS_AOC				15
#define GS101_CLK_ACPM_DVFS_BW				16
#define GS101_CLK_ACPM_DVFS_SPARE			17
#define GS101_CLK_ACPM_DVFS_LPM				18
#define GS101_CLK_ACPM_DVFS_AUR				19

#endif /* _DT_BINDINGS_CLOCK_GOOGLE_ZUMA_ACPM_H */
