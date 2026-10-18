// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MPG14/S2MPG15 PMIC GPIOs
 *
 * One control byte per pin after a status byte (two on S2MPG15), all in
 * the PMIC's GPIO register block.
 */

#include <linux/bits.h>
#include <linux/gpio/regmap.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define S2MPG1415_GPIO_OUT	BIT(5)
#define S2MPG1415_GPIO_OEN	BIT(6)

struct s2mpg1415_gpio_data {
	unsigned int status;
	unsigned int ctrl;
	int ngpio;
};

static const struct s2mpg1415_gpio_data s2mpg14_gpio_data = {
	.status = 0x4, .ctrl = 0x5, .ngpio = 6,
};

static const struct s2mpg1415_gpio_data s2mpg15_gpio_data = {
	.status = 0x6, .ctrl = 0x8, .ngpio = 10,
};

static int s2mpg1415_gpio_xlate(struct gpio_regmap *gpio,
				enum gpio_regmap_operation op,
				unsigned int base, unsigned int offset,
				unsigned int *reg, unsigned int *mask)
{
	const struct s2mpg1415_gpio_data *d = gpio_regmap_get_drvdata(gpio);

	if (base == d->status) {
		*reg = d->status + offset / 8;
		*mask = BIT(offset % 8);
		return 0;
	}

	*reg = d->ctrl + offset;
	*mask = (op == GPIO_REGMAP_GET_DIR_OP || op == GPIO_REGMAP_SET_DIR_OP) ?
		S2MPG1415_GPIO_OEN : S2MPG1415_GPIO_OUT;
	return 0;
}

static int s2mpg1415_gpio_probe(struct platform_device *pdev)
{
	const struct s2mpg1415_gpio_data *d = device_get_match_data(&pdev->dev);
	struct gpio_regmap_config config = {};
	struct regmap *regmap;

	regmap = dev_get_regmap(pdev->dev.parent, "gpio");
	if (!regmap)
		return -ENODEV;

	config.parent = &pdev->dev;
	config.regmap = regmap;
	config.ngpio = d->ngpio;
	config.reg_dat_base = d->status;
	config.reg_set_base = d->ctrl;
	config.reg_dir_out_base = d->ctrl;
	config.reg_mask_xlate = s2mpg1415_gpio_xlate;
	config.drvdata = (void *)d;

	return PTR_ERR_OR_ZERO(devm_gpio_regmap_register(&pdev->dev, &config));
}

static const struct of_device_id s2mpg1415_gpio_of_match[] = {
	{ .compatible = "samsung,s2mpg14-gpio", .data = &s2mpg14_gpio_data },
	{ .compatible = "samsung,s2mpg15-gpio", .data = &s2mpg15_gpio_data },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg1415_gpio_of_match);

static struct platform_driver s2mpg1415_gpio_driver = {
	.driver = {
		.name = "s2mpg1415-gpio",
		.of_match_table = s2mpg1415_gpio_of_match,
	},
	.probe = s2mpg1415_gpio_probe,
};
module_platform_driver(s2mpg1415_gpio_driver);

MODULE_DESCRIPTION("Samsung S2MPG14/S2MPG15 GPIO driver");
MODULE_LICENSE("GPL");
