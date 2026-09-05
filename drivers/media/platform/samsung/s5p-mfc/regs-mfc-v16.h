/* SPDX-License-Identifier: GPL-2.0-only */
/* Register definitions for the MFC v16 firmware interface. */

#ifndef _REGS_MFC_V16_H
#define _REGS_MFC_V16_H

#include "regs-mfc-v12.h"

#define S5P_FIMV_RISC_NONSECURE_BASE_V16		0x0068
#define S5P_FIMV_BUS_STATUS_V16			0x7018
#define S5P_FIMV_MFC_VERSION_V16			0xf028
#define S5P_FIMV_FIRMWARE_STATUS_V16		0xf0a4

#define MFC_VERSION_V16				0x160
#define MFC_VERSION_ZUMAPRO			0x16060000

/* The RISC maps its registers above this firmware-relative memory window. */
#define MFC_V16_MEM_WINDOW_SIZE			0xf0000000ULL

#endif /* _REGS_MFC_V16_H */
