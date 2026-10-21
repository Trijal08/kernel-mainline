/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _EXYNOSAUTO_DPU_DMA_H_
#define _EXYNOSAUTO_DPU_DMA_H_

#include <linux/pm_domain.h>

#include "exynos_drm_drv.h"

struct exynos_dpu_dma_context {
	struct device *dev;
	struct clk *aclk;
	int irq;
	void __iomem *regs;
	void __iomem *regs_hi;
	struct dev_pm_domain_list *pd_list;
	void *dma_priv;
};

int dpu_dma_update(struct exynos_dpu_dma_context *ctx, unsigned int idma,
		   struct exynos_drm_plane_state *state);

#endif
