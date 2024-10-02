// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>

#include "exynos_drm_fb.h"
#include "exynos_dpu_dma.h"

#define DMA_SHD_OFFSET 0x0800

#define DPU_DMA_TEST_PATTERN0_3 0x0020
#define DPU_DMA_TEST_PATTERN0_2 0x0024
#define DPU_DMA_TEST_PATTERN0_1 0x0028
#define DPU_DMA_TEST_PATTERN0_0 0x002C
#define DPU_DMA_TEST_PATTERN1_3 0x0030
#define DPU_DMA_TEST_PATTERN1_2 0x0034
#define DPU_DMA_TEST_PATTERN1_1 0x0038
#define DPU_DMA_TEST_PATTERN1_0 0x003C
#define DPU_DMA_DEBUG_DATA 0x0104

/*
 * 1.1 - IDMA Register
 * < DMA.offset >
 *  G0      G1      G2      G3      GF0     GF1     VG0     VG1
 *  0x1000  0x2000  0x3000  0x4000  0x5000  0x6000  0x7000  0x8000
 */

/* Channel order GF0, G0, VG0, G1, GF1, G2, VG1, G3 */
static unsigned int channel_map[] = { 5, 1, 7, 2, 6, 3, 8, 4 };

#define IDMA_ENABLE 0x0000
#define IDMA_IRQ 0x0004
#define IDMA_AFBC_CONFLICT_IRQ BIT(25)
#define IDMA_VR_CONFLICT_IRQ BIT(24)
#define IDMA_AFBC_TIMEOUT_IRQ BIT(23)
#define IDMA_RECOVERY_START_IRQ BIT(22)
#define IDMA_CONFIG_ERROR BIT(21)
#define IDMA_LOCAL_HW_RESET_DONE BIT(20)
#define IDMA_READ_SLAVE_ERROR BIT(19)
#define IDMA_STATUS_DEADLOCK_IRQ BIT(17)
#define IDMA_STATUS_FRAMEDONE_IRQ BIT(16)
#define IDMA_ALL_IRQ_CLEAR (0x3FB << 16)

#define IDMA_ALL_IRQ                                                           \
	(IDMA_AFBC_CONFLICT_IRQ | IDMA_VR_CONFLICT_IRQ |                       \
	 IDMA_AFBC_TIMEOUT_IRQ | IDMA_RECOVERY_START_IRQ | IDMA_CONFIG_ERROR | \
	 IDMA_LOCAL_HW_RESET_DONE | IDMA_READ_SLAVE_ERROR |                    \
	 IDMA_STATUS_DEADLOCK_IRQ | IDMA_STATUS_FRAMEDONE_IRQ)

#define IDMA_AFBC_CONFLICT_MASK BIT(10)
#define IDMA_CONFIG_ERROR_MASK BIT(6)
#define IDMA_READ_SLAVE_ERROR_MASK BIT(4)
#define IDMA_IRQ_DEADLOCK_MASK BIT(2)
#define IDMA_IRQ_FRAMEDONE_MASK BIT(1)
#define IDMA_ALL_IRQ_MASK (0x3FB << 1)
#define IDMA_IRQ_ENABLE BIT(0)

#define IDMA_IN_CON 0x0008
#define IDMA_IMG_FORMAT(_v) ((_v) << 11)
#define IDMA_IMG_FORMAT_MASK (0x3f << 11)
#define IDMA_IMG_FORMAT_ARGB8888 (0)

#define IDMA_BLOCK_EN BIT(3)

#define IDMA_SRC_SIZE 0x0010
#define IDMA_SRC_HEIGHT(_v) ((_v) << 16)
#define IDMA_SRC_HEIGHT_MASK (0x3FFF << 16)
#define IDMA_SRC_WIDTH(_v) ((_v) << 0)
#define IDMA_SRC_WIDTH_MASK (0xFFFF << 0)

#define IDMA_SRC_OFFSET 0x0014
#define IDMA_SRC_OFFSET_Y(_v) ((_v) << 16)
#define IDMA_SRC_OFFSET_Y_MASK (0x1FFF << 16)
#define IDMA_SRC_OFFSET_X(_v) ((_v) << 0)
#define IDMA_SRC_OFFSET_X_MASK (0x1FFF << 0)

#define IDMA_IMG_SIZE 0x0018
#define IDMA_IMG_HEIGHT(_v) ((_v) << 16)
#define IDMA_IMG_HEIGHT_MASK (0x1FFF << 16)
#define IDMA_IMG_WIDTH(_v) ((_v) << 0)
#define IDMA_IMG_WIDTH_MASK (0x1FFF << 0)

#define IDMA_BLOCK_OFFSET 0x0024
#define IDMA_BLK_OFFSET_Y(_v) ((_v) << 16)
#define IDMA_BLK_OFFSET_Y_MASK (0x1FFF << 16)
#define IDMA_BLK_OFFSET_X(_v) ((_v) << 0)
#define IDMA_BLK_OFFSET_X_MASK (0x1FFF << 0)

#define IDMA_BLOCK_SIZE 0x0028
#define IDMA_BLK_HEIGHT(_v) ((_v) << 16)
#define IDMA_BLK_HEIGHT_MASK (0x1FFF << 16)
#define IDMA_BLK_WIDTH(_v) ((_v) << 0)
#define IDMA_BLK_WIDTH_MASK (0x1FFF << 0)

#define IDMA_2BIT_STRIDE 0x0030
#define IDMA_CHROMA_2B_STRIDE(_v) ((_v) << 16)
#define IDMA_CHROMA_2B_STRIDE_MASK (0xFFFF << 16)
#define IDMA_LUMA_2B_STRIDE(_v) ((_v) << 0)
#define IDMA_LUMA_2B_STRIDE_MASK (0xFFFF << 0)

#define IDMA_IN_BASE_ADDR_Y 0x0040
#define IDMA_IN_BASE_ADDR_C 0x0044
#define IDMA_IN_BASE_ADDR_Y2 0x0048
#define IDMA_IN_BASE_ADDR_C2 0x004C

#define IDMA_IN_REQ_DEST 0x0068
#define IDMA_IN_REG_DEST_SEL(_v) ((_v) << 0)
#define IDMA_IN_REG_DEST_SEL_MASK (0x3 << 0)

#define IDMA_S_CFG_ERR_STATE 0x0870
#define IDMA_S_CFG_ERR_IMG_AFBC BIT(13)
#define IDMA_S_CFG_ERR_IMG_ROTATION BIT(12)
#define IDMA_S_CFG_ERR_IMG_FORMAT BIT(11)
#define IDMA_S_CFG_ERR_SRC_WIDTH BIT(10)
#define IDMA_S_CFG_ERR_CHROM_STRIDE BIT(9)
#define IDMA_S_CFG_ERR_BASE_ADDR_Y BIT(8)
#define IDMA_S_CFG_ERR_BASE_ADDR_C BIT(7)
#define IDMA_S_CFG_ERR_IMG_WIDTH_AFBC BIT(6)
#define IDMA_S_CFG_ERR_IMG_WIDTH BIT(5)
#define IDMA_S_CFG_ERR_IMG_HEIGHT_ROTATION BIT(4)
#define IDMA_S_CFG_ERR_IMG_HEIGHT BIT(3)
#define IDMA_S_CFG_ERR_BLOCKING BIT(2)
#define IDMA_S_CFG_ERR_SRC_OFFSET_X BIT(1)
#define IDMA_S_CFG_ERR_SRC_OFFSET_Y BIT(0)
#define IDMA_S_CFG_ERR_GET(_v) (((_v) >> 0) & 0x7FF)

static inline uint32_t dma_read(struct exynos_dpu_dma_context *ctx, u32 idx,
				u32 offset)
{
	return readl(ctx->regs + 0x1000 * idx + offset);
}
static inline uint32_t dma_read_mask(struct exynos_dpu_dma_context *ctx,
				     u32 idx, u32 offset, u32 mask)
{
	uint32_t val = readl(ctx->regs + 0x1000 * idx + offset);
	val &= (mask);
	return val;
}
static inline void dma_write(struct exynos_dpu_dma_context *ctx, u32 idx,
			     u32 offset, u32 val)
{
	writel(val, ctx->regs + 0x1000 * idx + offset);
}
static inline void dma_write_mask(struct exynos_dpu_dma_context *ctx, u32 idx,
				  u32 offset, u32 val, u32 mask)
{
	uint32_t old = dma_read(ctx, idx, offset);
	val = (val & mask) | (old & ~mask);
	dma_write(ctx, idx, offset, val);
}

static void dma_reg_clear_irq(struct exynos_dpu_dma_context *ctx, u32 id,
			      u32 irq)
{
	dma_write_mask(ctx, id, IDMA_IRQ, ~0, irq);
}

static u32 idma_reg_get_irq_and_clear(struct exynos_dpu_dma_context *ctx,
				      u32 id)
{
	u32 val, cfg_err;

	val = dma_read_mask(ctx, id, IDMA_IRQ, IDMA_ALL_IRQ);

	if (val & IDMA_AFBC_CONFLICT_IRQ) {
		pr_err("AFBC conflict occur\n");
	}

	if (val & IDMA_VR_CONFLICT_IRQ) {
		pr_err("VR conflict occur\n");
	}

	if (val & IDMA_RECOVERY_START_IRQ) {
		pr_err("recovery start occur\n");
	}

	if (val & IDMA_CONFIG_ERROR) {
		cfg_err = dma_read(ctx, id, IDMA_S_CFG_ERR_STATE);
		pr_err("config error occur(0x%x)\n", cfg_err);
	}

	if (val & IDMA_LOCAL_HW_RESET_DONE) {
		pr_err("local hw reset done\n");
	}

	if (val & IDMA_READ_SLAVE_ERROR) {
		pr_err("read slave error occur\n");
	}

	if (val & IDMA_STATUS_DEADLOCK_IRQ) {
		pr_err("status deadlock occur\n");
	}

	if (val & IDMA_STATUS_FRAMEDONE_IRQ) {
		pr_debug("frame done occur\n");
	}

	dma_reg_clear_irq(ctx, id, val);

	return val;
}

static void dma_reg_init(struct exynos_dpu_dma_context *ctx, u32 id,
			 const unsigned long attr)
{
	dma_write_mask(ctx, id, IDMA_IRQ, 0, IDMA_ALL_IRQ_MASK);
	dma_write_mask(ctx, id, IDMA_IRQ, ~0, IDMA_IRQ_ENABLE);
}

static irqreturn_t dma_irq_handler(int irq, void *priv)
{
	u32 val_dma = 0;
	struct exynos_dpu_dma_context *ctx = priv;

	val_dma = idma_reg_get_irq_and_clear(ctx, 5);

	/* This interrupt is not mine */
	if (!val_dma)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

int dpu_dma_update(struct exynos_dpu_dma_context *ctx, unsigned int channel,
		   struct exynos_drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->base.fb;
	unsigned int idma = channel_map[channel];

	dma_reg_init(ctx, idma, 0);
	dma_write(ctx, idma, IDMA_IN_BASE_ADDR_Y,
		  exynos_drm_fb_dma_addr(fb, 0));
	dma_write(ctx, idma, IDMA_SRC_OFFSET,
		  IDMA_SRC_OFFSET_Y(state->src.y) |
			  IDMA_SRC_OFFSET_X(state->src.x));
	dma_write(ctx, idma, IDMA_SRC_SIZE,
		  IDMA_SRC_HEIGHT(fb->height) |
			  IDMA_SRC_WIDTH(fb->pitches[0] / fb->format->cpp[0]));
	dma_write(ctx, idma, IDMA_IMG_SIZE,
		  IDMA_IMG_HEIGHT(state->src.h) | IDMA_IMG_WIDTH(state->src.w));

	dma_write_mask(ctx, idma, IDMA_IN_CON,
		       IDMA_IMG_FORMAT(IDMA_IMG_FORMAT_ARGB8888),
		       IDMA_IMG_FORMAT_MASK);

	return 0;
}

static int dma_bind(struct device *dev, struct device *master, void *data)
{
	struct exynos_dpu_dma_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;

	return exynos_drm_register_dma(drm_dev, dev, &ctx->dma_priv);
};

static const struct component_ops dma_component_ops = {
	.bind = dma_bind,
};

static int dpu_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_dpu_dma_context *ctx;
	struct resource *res;

	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct exynos_dpu_dma_context),
			   GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctx);

	ctx->aclk = devm_clk_get_enabled(dev, "aclk");
	if (IS_ERR(ctx->aclk))
		return dev_err_probe(dev, PTR_ERR(ctx->aclk),
				     "Cannot get aclk\n");

	ctx->irq = platform_get_irq(pdev, 0);
	if (ctx->irq < 0)
		return ctx->irq;

	ret = devm_request_irq(dev, ctx->irq, dma_irq_handler, 0, pdev->name,
			       ctx);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	ctx->regs = devm_ioremap_resource(dev, res);
	if (!ctx->regs)
		return -ENOMEM;

	pm_runtime_enable(dev);
	/* For turn on attached SYSMMU */
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		return ret;
	}

	component_add(dev, &dma_component_ops);

	return 0;
}

static const struct of_device_id dpu_dma_of_match[] = {
	{ .compatible = "samsung,exynosauto-dpu-dma" },
	{},
};
MODULE_DEVICE_TABLE(of, dpu_dma_of_match);

struct platform_driver dpu_dma_driver = {
    .probe          = dpu_dma_probe,
    .driver         = {
        .name   = "dpu_dma_driver",
        .of_match_table = of_match_ptr(dpu_dma_of_match),
    },
};
