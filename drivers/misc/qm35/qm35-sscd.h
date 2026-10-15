/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __QM35_SSCD__
#define __QM35_SSCD__

#include "qm35.h"

/* Google's SSCD coredump collector is not in this tree; keep the hooks inert. */
static inline int qm35_report_coredump(struct qm35_ctx *qm35_ctx) { return 0; }
static inline int qm35_register_coredump(struct qm35_ctx *qm35_ctx) { return 0; }
static inline void qm35_unregister_coredump(struct qm35_ctx *qm35_ctx) {}

#endif /* __QM35_SSCD__ */
