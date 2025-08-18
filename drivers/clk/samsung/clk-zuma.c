\
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal clock provider for Google Zuma (Tensor G3) CMU_TOP
 * Safe for early bring-up: exposes a single fixed-rate clock (OSCCLK).
 */
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* Matches include/dt-bindings/clock/google,zuma.h */
#define ZUMA_CMU_TOP_OSCCLK	0

struct zuma_cmu_top {
	struct clk_hw_onecell_data *clk_data;
};

static int zuma_cmu_top_probe(struct platform_device *pdev)
{
	struct zuma_cmu_top *z;
	struct clk_hw *osc_hw;

	z = devm_kzalloc(&pdev->dev, sizeof(*z), GFP_KERNEL);
	if (!z)
		return -ENOMEM;

	z->clk_data = devm_kzalloc(&pdev->dev,
				   struct_size(z->clk_data, hws, 1), GFP_KERNEL);
	if (!z->clk_data)
		return -ENOMEM;

	/* Provide a stable fixed-rate source for consumers */
	osc_hw = devm_clk_hw_register_fixed_rate(&pdev->dev,
				"oscclk", NULL, 0, 24576000);
	if (IS_ERR(osc_hw))
		return PTR_ERR(osc_hw);

	z->clk_data->num = 1;
	z->clk_data->hws[ZUMA_CMU_TOP_OSCCLK] = osc_hw;

	return devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_onecell_get,
					   z->clk_data);
}

static const struct of_device_id zuma_cmu_top_of_match[] = {
	{ .compatible = "google,zuma-cmu-top" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zuma_cmu_top_of_match);

static struct platform_driver zuma_cmu_top_driver = {
	.probe = zuma_cmu_top_probe,
	.driver = {
		.name = "clk-zuma",
		.of_match_table = zuma_cmu_top_of_match,
	},
};
module_platform_driver(zuma_cmu_top_driver);

MODULE_DESCRIPTION("Google Zuma CMU_TOP minimal clock provider");
MODULE_LICENSE("GPL");
