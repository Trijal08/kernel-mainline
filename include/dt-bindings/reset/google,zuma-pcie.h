/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
/*
 * Reset lines the Google Tensor (zuma/zumapro) PCIe root complex offers to the
 * endpoint it hosts.
 *
 * Copyright 2026 Trijal Saha
 */
#ifndef _DT_BINDINGS_RESET_GOOGLE_ZUMA_PCIE_H
#define _DT_BINDINGS_RESET_GOOGLE_ZUMA_PCIE_H

/*
 * The link itself: asserting parks it (PME_Turn_Off, L2, PERST, PHY off),
 * deasserting re-calibrates the PHY and retrains.  An endpoint whose firmware
 * re-links mid-boot drives this.
 */
#define ZUMA_PCIE_RESET_LINK		0

/*
 * Cold power-cycle of the attached baseband, link parked and retrained around
 * it, so a crashed or self-reset modem is back in its boot ROM.
 */
#define ZUMA_PCIE_RESET_CP_COLD		1

/*
 * Warm reset of the attached baseband with its dump-notify line latched, so it
 * comes back running its minidump agent rather than its normal image.
 */
#define ZUMA_PCIE_RESET_CP_DUMP		2

#endif /* _DT_BINDINGS_RESET_GOOGLE_ZUMA_PCIE_H */
