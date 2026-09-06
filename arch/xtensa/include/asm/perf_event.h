#ifndef __ASM_XTENSA_PERF_EVENT_H
#define __ASM_XTENSA_PERF_EVENT_H

#include <linux/irqreturn.h>

irqreturn_t xtensa_pmu_irq_handler(int irq, void *dev_id);

#endif /* __ASM_XTENSA_PERF_EVENT_H */
