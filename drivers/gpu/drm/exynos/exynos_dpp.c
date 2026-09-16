// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>

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

void dpp_update(struct exynos_dpp_context *dpp,
		const struct exynos_drm_plane_state *state)
{
	const struct drm_framebuffer *fb = state->base.fb;
	u32 fmt;

	if (pm_runtime_resume_and_get(dpp->dev) < 0)
		return;

	/*
	 * DPP block: the pixel format lives in DPP_COM_IO_CON and the image size
	 * in DPP_COM_IMG_SIZE (the vendor dpp_reg_set_format()/set_img_size()).
	 * Writing the old 0x0008/0x0018 offsets left the block's size at 0, so it
	 * produced no pixels and the DECON channel-5 read starved (OF_LEVEL=0).
	 * The DPP block only needs the bit depth - 8bpc vs 10bpc RGB; the
	 * component order (X-vs-A, RGB-vs-BGR) is the IDMA's concern.
	 */
	switch (fb->format->format) {
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_ABGR2101010:
	case DRM_FORMAT_RGBA1010102:
	case DRM_FORMAT_BGRA1010102:
		fmt = DPP_IMG_FORMAT_ARGB8101010;
		break;
	default:
		fmt = DPP_IMG_FORMAT_ARGB8888;
		break;
	}

	writel(DPP_IMG_FORMAT(fmt), dpp->regs + DPP_COM_IO_CON);
	writel(DPP_IMG_HEIGHT(state->src.h) | DPP_IMG_WIDTH(state->src.w),
	       dpp->regs + DPP_COM_IMG_SIZE);

	pm_runtime_put_sync(dpp->dev);
}

/* Nothing to do: each DECON picks its own DPP out of DT. */
static int dpp_bind(struct device *dev, struct device *master, void *data)
{
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
	 * Physical DPU channel this DPP/IDMA drives, and the one number the
	 * whole fetch path is indexed by: the DECON's WIN_CHMAP, the DPP block
	 * (0x19930000 + ch * 0x1000) and the IDMA block (0x19900000 + ch *
	 * 0x1000). GF0 is 5, GF1 is 6. Get it wrong and the DECON blender reads
	 * an unfed channel and stalls mid-frame waiting for pixels.
	 */
	if (of_property_read_u32(dev->of_node, "dpp,id", &dpp->type))
		dpp->type = 5;

	dpp->aclk = devm_clk_get_enabled(dpp->dev, "aclk");
	if (IS_ERR(dpp->aclk))
		pr_err("failed to get clock");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	dpp->regs = devm_ioremap_resource(dev, res);

	platform_set_drvdata(pdev, dpp);

	/* Taken around the register writes in dpp_update(). */
	pm_runtime_enable(dev);

	return component_add(dev, &dpp_component_ops);
}

struct platform_driver dpp_driver = {
	.probe = dpp_probe,
	.driver = {
		   .name = "dpp",
		   .of_match_table = dpp_of_match,
	},
};
