// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * Author: Tomasz Figa <t.figa@samsung.com>
 *
 * Clock driver for Exynos clock output
 */

#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-pmu.h>

#define DRV_NAME			"exynos-clkout"

#define EXYNOS_CLKOUT_NR_CLKS		1
#define EXYNOS_CLKOUT_PARENTS		32

#define EXYNOS_PMU_DEBUG_REG		0xa00
#define EXYNOS_CLKOUT_DISABLE_SHIFT	0
#define EXYNOS_CLKOUT_MUX_SHIFT		8
#define EXYNOS4_CLKOUT_MUX_MASK		0xf
#define EXYNOS5_CLKOUT_MUX_MASK		0x1f

/*
 * gs101 and later place the clock-output pads in the PMU register block, one
 * enable bit per pad, and (on Tensor SoCs) the block is behind secure firmware.
 * They are therefore driven as plain gates through the PMU regmap rather than
 * the exynos4/5 mux+gate composite at a directly-mapped debug register.
 */
#define GS101_CLKOUT_ENABLE		BIT(16)

struct exynos_clkout_gate_desc {
	const char *name;
	u32 offset;
};

struct exynos_clkout {
	struct clk_gate gate;
	struct clk_mux mux;
	spinlock_t slock;
	void __iomem *reg;
	struct device_node *np;
	u32 pmu_debug_save;
	struct clk_hw_onecell_data data;
};

/* One PMU-regmap gate clock (gs101-style). */
struct exynos_clkout_pmu_gate {
	struct clk_hw hw;
	struct regmap *pmu;
	u32 offset;
};

#define to_exynos_clkout_pmu_gate(_hw) \
	container_of(_hw, struct exynos_clkout_pmu_gate, hw)

struct exynos_clkout_variant {
	u32 mux_mask;
	/* gs101-style PMU-regmap gates; when set the mux path is unused. */
	const struct exynos_clkout_gate_desc *gates;
	unsigned int nr_gates;
};

static const struct exynos_clkout_variant exynos_clkout_exynos4 = {
	.mux_mask	= EXYNOS4_CLKOUT_MUX_MASK,
};

static const struct exynos_clkout_variant exynos_clkout_exynos5 = {
	.mux_mask	= EXYNOS5_CLKOUT_MUX_MASK,
};

static const struct exynos_clkout_gate_desc gs101_clkout_gates[] = {
	{ .name = "clkout0", .offset = 0x3e80 },
	{ .name = "clkout1", .offset = 0x3e84 },
};

static const struct exynos_clkout_variant exynos_clkout_gs101 = {
	.gates		= gs101_clkout_gates,
	.nr_gates	= ARRAY_SIZE(gs101_clkout_gates),
};

static int exynos_clkout_pmu_gate_enable(struct clk_hw *hw)
{
	struct exynos_clkout_pmu_gate *g = to_exynos_clkout_pmu_gate(hw);

	return regmap_update_bits(g->pmu, g->offset, GS101_CLKOUT_ENABLE,
				  GS101_CLKOUT_ENABLE);
}

static void exynos_clkout_pmu_gate_disable(struct clk_hw *hw)
{
	struct exynos_clkout_pmu_gate *g = to_exynos_clkout_pmu_gate(hw);

	regmap_update_bits(g->pmu, g->offset, GS101_CLKOUT_ENABLE, 0);
}

static int exynos_clkout_pmu_gate_is_enabled(struct clk_hw *hw)
{
	struct exynos_clkout_pmu_gate *g = to_exynos_clkout_pmu_gate(hw);
	unsigned int val;

	if (regmap_read(g->pmu, g->offset, &val))
		return 0;

	return (val & GS101_CLKOUT_ENABLE) == GS101_CLKOUT_ENABLE;
}

static const struct clk_ops exynos_clkout_pmu_gate_ops = {
	.enable		= exynos_clkout_pmu_gate_enable,
	.disable	= exynos_clkout_pmu_gate_disable,
	.is_enabled	= exynos_clkout_pmu_gate_is_enabled,
};

static const struct of_device_id exynos_clkout_ids[] = {
	{
		.compatible = "samsung,exynos3250-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4210-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4212-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4412-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos5250-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5410-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5420-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5433-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "google,gs101-pmu",
		.data = &exynos_clkout_gs101,
	}, { }
};

/*
 * Device will be instantiated as child of PMU device without its own
 * device node.  Therefore match compatibles against parent.
 */
static const struct exynos_clkout_variant *
exynos_clkout_match_parent_dev(struct device *dev)
{
	const struct of_device_id *match;

	if (!dev->parent) {
		dev_err(dev, "not instantiated from MFD\n");
		return ERR_PTR(-EINVAL);
	}

	/*
	 * 'exynos_clkout_ids' arrays is not the ids array matched by
	 * the dev->parent driver, so of_device_get_match_data() or
	 * device_get_match_data() cannot be used here.
	 */
	match = of_match_device(exynos_clkout_ids, dev->parent);
	if (!match) {
		dev_err(dev, "cannot match parent device\n");
		return ERR_PTR(-EINVAL);
	}

	return match->data;
}

/*
 * gs101-style: register one PMU-regmap gate per clock-output pad.  The pads
 * live in the PMU register block (secured on Tensor), so they are driven
 * through the PMU regmap rather than a directly-mapped register.
 */
static int exynos_clkout_probe_gs101(struct platform_device *pdev,
				     struct device_node *np,
				     const struct exynos_clkout_variant *variant)
{
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct regmap *pmu;
	unsigned int i;

	pmu = exynos_get_pmu_regmap_by_phandle(np, NULL);
	if (IS_ERR(pmu))
		return dev_err_probe(dev, PTR_ERR(pmu),
				     "cannot get PMU regmap\n");

	data = devm_kzalloc(dev, struct_size(data, hws, variant->nr_gates),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = variant->nr_gates;

	for (i = 0; i < variant->nr_gates; i++) {
		const struct exynos_clkout_gate_desc *desc = &variant->gates[i];
		struct exynos_clkout_pmu_gate *g;
		struct clk_init_data init = { };
		int ret;

		g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
		if (!g)
			return -ENOMEM;

		g->pmu = pmu;
		g->offset = desc->offset;

		/*
		 * Only the gate is modelled; the pad's source and rate are
		 * fixed in hardware, so the clock is registered parent-less.
		 */
		init.name = desc->name;
		init.ops = &exynos_clkout_pmu_gate_ops;
		init.num_parents = 0;
		g->hw.init = &init;

		ret = devm_clk_hw_register(dev, &g->hw);
		if (ret)
			return ret;

		data->hws[i] = &g->hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static int exynos_clkout_probe(struct platform_device *pdev)
{
	const char *parent_names[EXYNOS_CLKOUT_PARENTS];
	struct clk *parents[EXYNOS_CLKOUT_PARENTS];
	const struct exynos_clkout_variant *variant;
	struct exynos_clkout *clkout;
	struct device_node *np;
	int parent_count, ret, i;

	variant = exynos_clkout_match_parent_dev(&pdev->dev);
	if (IS_ERR(variant))
		return PTR_ERR(variant);

	np = pdev->dev.of_node;
	if (!np) {
		/*
		 * pdev->dev.parent was checked by exynos_clkout_match_parent_dev()
		 * so it is not NULL.
		 */
		np = pdev->dev.parent->of_node;
	}

	/* gs101 and later drive the clkout pads as PMU-regmap gates. */
	if (variant->nr_gates)
		return exynos_clkout_probe_gs101(pdev, np, variant);

	clkout = devm_kzalloc(&pdev->dev,
			      struct_size(clkout, data.hws, EXYNOS_CLKOUT_NR_CLKS),
			      GFP_KERNEL);
	if (!clkout)
		return -ENOMEM;

	clkout->np = np;
	platform_set_drvdata(pdev, clkout);

	spin_lock_init(&clkout->slock);

	parent_count = 0;
	for (i = 0; i < EXYNOS_CLKOUT_PARENTS; ++i) {
		char name[] = "clkoutXX";

		snprintf(name, sizeof(name), "clkout%d", i);
		parents[i] = of_clk_get_by_name(clkout->np, name);
		if (IS_ERR(parents[i])) {
			parent_names[i] = "none";
			continue;
		}

		parent_names[i] = __clk_get_name(parents[i]);
		parent_count = i + 1;
	}

	if (!parent_count)
		return -EINVAL;

	clkout->reg = of_iomap(clkout->np, 0);
	if (!clkout->reg) {
		ret = -ENODEV;
		goto clks_put;
	}

	clkout->gate.reg = clkout->reg + EXYNOS_PMU_DEBUG_REG;
	clkout->gate.bit_idx = EXYNOS_CLKOUT_DISABLE_SHIFT;
	clkout->gate.flags = CLK_GATE_SET_TO_DISABLE;
	clkout->gate.lock = &clkout->slock;

	clkout->mux.reg = clkout->reg + EXYNOS_PMU_DEBUG_REG;
	clkout->mux.mask = variant->mux_mask;
	clkout->mux.shift = EXYNOS_CLKOUT_MUX_SHIFT;
	clkout->mux.lock = &clkout->slock;

	clkout->data.num = EXYNOS_CLKOUT_NR_CLKS;
	clkout->data.hws[0] = clk_hw_register_composite(NULL, "clkout",
				parent_names, parent_count, &clkout->mux.hw,
				&clk_mux_ops, NULL, NULL, &clkout->gate.hw,
				&clk_gate_ops, CLK_SET_RATE_PARENT
				| CLK_SET_RATE_NO_REPARENT);
	if (IS_ERR(clkout->data.hws[0])) {
		ret = PTR_ERR(clkout->data.hws[0]);
		goto err_unmap;
	}

	ret = of_clk_add_hw_provider(clkout->np, of_clk_hw_onecell_get, &clkout->data);
	if (ret)
		goto err_clk_unreg;

	return 0;

err_clk_unreg:
	clk_hw_unregister(clkout->data.hws[0]);
err_unmap:
	iounmap(clkout->reg);
clks_put:
	for (i = 0; i < EXYNOS_CLKOUT_PARENTS; ++i)
		if (!IS_ERR(parents[i]))
			clk_put(parents[i]);

	dev_err(&pdev->dev, "failed to register clkout clock\n");

	return ret;
}

static void exynos_clkout_remove(struct platform_device *pdev)
{
	struct exynos_clkout *clkout = platform_get_drvdata(pdev);

	/* gs101 registers its gates with devm and sets no drvdata. */
	if (!clkout)
		return;

	of_clk_del_provider(clkout->np);
	clk_hw_unregister(clkout->data.hws[0]);
	iounmap(clkout->reg);
}

static int __maybe_unused exynos_clkout_suspend(struct device *dev)
{
	struct exynos_clkout *clkout = dev_get_drvdata(dev);

	if (!clkout)
		return 0;

	clkout->pmu_debug_save = readl(clkout->reg + EXYNOS_PMU_DEBUG_REG);

	return 0;
}

static int __maybe_unused exynos_clkout_resume(struct device *dev)
{
	struct exynos_clkout *clkout = dev_get_drvdata(dev);

	if (!clkout)
		return 0;

	writel(clkout->pmu_debug_save, clkout->reg + EXYNOS_PMU_DEBUG_REG);

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_clkout_pm_ops, exynos_clkout_suspend,
			 exynos_clkout_resume);

static struct platform_driver exynos_clkout_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &exynos_clkout_pm_ops,
	},
	.probe = exynos_clkout_probe,
	.remove = exynos_clkout_remove,
};
module_platform_driver(exynos_clkout_driver);

MODULE_AUTHOR("Krzysztof Kozlowski <krzk@kernel.org>");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("Samsung Exynos clock output driver");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL");
