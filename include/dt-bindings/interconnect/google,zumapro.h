/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Interconnect IDs for Google Zumapro.
 */

#ifndef __DT_BINDINGS_INTERCONNECT_GOOGLE_ZUMAPRO_H
#define __DT_BINDINGS_INTERCONNECT_GOOGLE_ZUMAPRO_H

#define ZUMAPRO_BTS_SLAVE_MIF		1
#define ZUMAPRO_BTS_MASTER_MFC		2

/* MFC workload floors; zero selects the 100 MHz workload. */
#define ZUMAPRO_BTS_MFC_310		(1 << 0)
#define ZUMAPRO_BTS_MFC_400		(1 << 1)
#define ZUMAPRO_BTS_MFC_465		(1 << 2)
#define ZUMAPRO_BTS_MFC_664		(1 << 3)

#endif /* __DT_BINDINGS_INTERCONNECT_GOOGLE_ZUMAPRO_H */
