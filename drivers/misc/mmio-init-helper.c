// SPDX-License-Identifier: GPL-2.0-only
/*
 * Early one-shot MMIO initializer.
 *
 * DT example:
 *
 * mmio_init_0: mmio-init@19470030 {
 *	compatible = "linux,mmio-init-helper";
 *	reg = <0x0 0x19470030 0x0 0x4>;
 *	value = <0x00003061>;
 * };
 *
 * For read-modify-write:
 *
 * mmio_init_1: mmio-init@deadbeef {
 *	compatible = "linux,mmio-init-helper";
 *	reg = <0x0 0xdeadbeef 0x0 0x4>;
 *	mask = <0x0000ffff>;
 *	value = <0x00003061>;
 * };
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

static int mmio_init_helper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *base;
	u32 value;
	u32 mask;
	u32 reg;
	bool has_mask;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "failed to map MMIO resource\n");

	if (of_property_read_u32(dev->of_node, "value", &value))
		return dev_err_probe(dev, -EINVAL,
				     "missing required property: value\n");

	has_mask = !of_property_read_u32(dev->of_node, "mask", &mask);
	if (has_mask) {
		reg = readl(base);
		reg = (reg & ~mask) | (value & mask);
		writel(reg, base);
	} else {
		writel(value, base);
	}

	dev_info(dev, "MMIO init write complete%s\n",
		 has_mask ? " (masked)" : "");
	return 0;
}

static const struct of_device_id mmio_init_helper_of_match[] = {
	{ .compatible = "linux,mmio-init-helper" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mmio_init_helper_of_match);

static struct platform_driver mmio_init_helper_driver = {
	.probe = mmio_init_helper_probe,
	.driver = {
		.name = "mmio-init-helper",
		.of_match_table = mmio_init_helper_of_match,
	},
};

static int __init mmio_init_helper_driver_init(void)
{
	return platform_driver_register(&mmio_init_helper_driver);
}
/*
 * Register at subsys_initcall so the write can happen before most
 * module_init/device_initcall display drivers like simpledrm.
 */
subsys_initcall(mmio_init_helper_driver_init);

MODULE_AUTHOR("OpenAI");
MODULE_DESCRIPTION("Generic early MMIO initializer");
MODULE_LICENSE("GPL");
