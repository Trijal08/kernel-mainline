// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/irq.h>

#include <drm/drm_atomic.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_vblank.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/exynos_drm.h>
#include <drm/drm_blend.h>
#include <drm/drm_framebuffer.h>

#include "exynos_dpp.h"
#include "exynos_drm_fb.h"

#define DPP_RGB_IMG_WIDTH_MIN 16
#define DPP_RGB_IMG_WIDTH_MAX 8192
#define DPP_RGB_IMG_WIDTH_ALIGN 1

#define DPP_RGB_IMG_HEIGHT_MIN 16
#define DPP_RGB_IMG_HEIGHT_MAX 4096
#define DPP_RGB_IMG_HEIGHT_ALIGN 1

#define DPP_YUV_IMG_WIDTH_MIN 32
#define DPP_YUV_IMG_WIDTH_MAX 8192
#define DPP_YUV_IMG_WIDTH_ALIGN 2

#define DPP_YUV_IMG_HEIGHT_MIN 16
#define DPP_YUV_IMG_HEIGHT_MAX 4096
#define DPP_YUV_IMG_HEIGHT_ALIGN 2

static const struct of_device_id dpp_of_match[] = {
	{ .compatible = "samsung,exynosauto-dpp" },
	{},
};

void dpp_update(struct exynos_dpp_context *dpp, unsigned int channel,
		const struct exynos_drm_plane_state *state)
{
	/*
	 * DPP block: the pixel format lives in DPP_COM_IO_CON and the image size
	 * in DPP_COM_IMG_SIZE (the vendor dpp_reg_set_format()/set_img_size()).
	 * Writing the old 0x0008/0x0018 offsets left the block's size at 0, so it
	 * produced no pixels and the DECON channel-5 read starved (OF_LEVEL=0).
	 * The DPP takes 8bpc RGB (ARGB8888); X-vs-A is the IDMA's concern.
	 */
	writel(DPP_IMG_FORMAT(DPP_IMG_FORMAT_ARGB8888),
	       dpp->regs + 0x1000 + DPP_COM_IO_CON);
	writel(DPP_IMG_HEIGHT(state->src.h) | DPP_IMG_WIDTH(state->src.w),
	       dpp->regs + 0x1000 + DPP_COM_IMG_SIZE);
}

static int dpp_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm_dev = data;
	struct exynos_drm_private *devpriv = drm_dev->dev_private;

	devpriv->dpp_dev = dev;

	return 0;
}

static const struct component_ops dpp_component_ops = {
	.bind = dpp_bind,
};

static int dpp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_dpp_context *dpp;
	struct resource *res;

	dpp = devm_kzalloc(dev, sizeof(struct exynos_dpp_context), GFP_KERNEL);
	if (!dpp)
		return -ENOMEM;

	dpp->dev = dev;

	/*
	 * Physical DPU channel this DPP/IDMA drives. The DECON maps each window
	 * to a channel via WIN_CHMAP (config->dpp_type), and the DPU-DMA feeds
	 * that same channel: dpu_dma_update() uses logical channel 0, which
	 * channel_map[0] resolves to physical IDMA 5 (the GF0 block at
	 * 0x19905000, dpp,id = 5). WIN_CHMAP must therefore be 5, or the DECON
	 * blender reads an unfed channel and stalls mid-frame waiting for pixels.
	 */
	dpp->type = 5;

	dpp->aclk = devm_clk_get_enabled(dpp->dev, "aclk");
	if (IS_ERR(dpp->aclk))
		pr_err("failed to get clock");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	dpp->regs = devm_ioremap_resource(dev, res);

	platform_set_drvdata(pdev, dpp);

	return component_add(dev, &dpp_component_ops);
}

struct platform_driver dpp_driver = {
	.probe = dpp_probe,
	.driver = {
		   .name = "dpp",
		   .of_match_table = dpp_of_match,
	},
};
