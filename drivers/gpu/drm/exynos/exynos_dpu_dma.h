#ifndef _EXYNOSAUTO_DPU_DMA_H_
#define _EXYNOSAUTO_DPU_DMA_H_

#include "exynos_drm_drv.h"

struct exynos_dpu_dma_context {
	struct clk *aclk;
	int irq;
	void __iomem *regs;
	void *dma_priv;
};

int dpu_dma_update(struct exynos_dpu_dma_context *ctx, unsigned int channel,
		   struct exynos_drm_plane_state *state);

#endif