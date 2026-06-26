// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779_charger.c - VBUS/OTG boost regulator for the MAX77779 charger.
 *
 * Derived from max77759_charger.c. The MAX77779 charger block is register
 * compatible with the MAX77759 for the CHG_CNFG_00.MODE control used here, but
 * boards such as zumapro/komodo source OTG VBUS through an *external* boost
 * enabled by a GPIO (vendor "extbst-ctl") rather than the internal reverse
 * boost. Both paths are supported.
 *
 * Copyright 2025 Google LLC.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/max77779.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

struct max77779_charger {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev *otg_rdev;
	/* Optional external VBUS boost enable (e.g. komodo extbst-ctl). */
	struct gpio_desc *ext_bst_ctl;
	struct mutex lock; /* protects mode */
	enum max77779_chgr_mode mode;
};

static int charger_set_mode(struct max77779_charger *chg,
			    enum max77779_chgr_mode mode)
{
	int ret;

	guard(mutex)(&chg->lock);

	if (chg->mode == mode)
		return 0;

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00,
				 MAX77779_CHGR_REG_CHG_CNFG_00_MODE, mode);
	if (ret)
		return ret;

	chg->mode = mode;
	return 0;
}

/*
 * VBUS sourcing for USB OTG. On boards with an external boost (extbst-ctl
 * GPIO, e.g. zumapro/komodo) the charger MODE is left ALL_OFF and VBUS is
 * driven by the external boost; otherwise the internal reverse boost
 * (OTG_BOOST_ON) is used. Mirrors the vendor max77779_get_usecase()
 * GSU_MODE_USB_OTG path.
 */
static int max77779_otg_enable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	if (chg->ext_bst_ctl) {
		/*
		 * External boost sources VBUS; keep the charger off. Assert the
		 * boost-enable regardless of the mode write so VBUS comes up.
		 */
		ret = charger_set_mode(chg, MAX77779_CHGR_MODE_ALL_OFF);
		if (ret)
			dev_warn(chg->dev, "OTG: ALL_OFF failed: %d\n", ret);

		gpiod_set_value_cansleep(chg->ext_bst_ctl, 1);
		dev_info(chg->dev, "OTG VBUS on: external boost, ext_bst=%d\n",
			 gpiod_get_value_cansleep(chg->ext_bst_ctl));
		return 0;
	}

	dev_info(chg->dev, "OTG VBUS on: internal reverse boost\n");
	return charger_set_mode(chg, MAX77779_CHGR_MODE_OTG_BOOST_ON);
}

static int max77779_otg_disable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);

	if (chg->ext_bst_ctl)
		gpiod_set_value_cansleep(chg->ext_bst_ctl, 0);

	return charger_set_mode(chg, MAX77779_CHGR_MODE_ALL_OFF);
}

static int max77779_otg_is_enabled(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);

	if (chg->ext_bst_ctl)
		return gpiod_get_value_cansleep(chg->ext_bst_ctl);

	guard(mutex)(&chg->lock);

	return chg->mode == MAX77779_CHGR_MODE_OTG_BOOST_ON;
}

static const struct regulator_ops max77779_otg_ops = {
	.enable = max77779_otg_enable,
	.disable = max77779_otg_disable,
	.is_enabled = max77779_otg_is_enabled,
};

static const struct regulator_desc max77779_otg_desc = {
	.name = "otg",
	.of_match = of_match_ptr("otg-regulator"),
	.owner = THIS_MODULE,
	.ops = &max77779_otg_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int max77779_charger_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	struct device *dev = &pdev->dev;
	struct max77779_charger *chg;
	u32 regval;
	int ret;

	device_set_of_node_from_dev(dev, dev->parent);

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->regmap = dev_get_regmap(dev->parent, "charger");
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "Missing charger regmap\n");

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize lock\n");

	chg->ext_bst_ctl = devm_gpiod_get_optional(dev, "extbst", GPIOD_OUT_LOW);
	if (IS_ERR(chg->ext_bst_ctl))
		return dev_err_probe(dev, PTR_ERR(chg->ext_bst_ctl),
				     "Failed to get extbst GPIO\n");

	/* Cache the current MODE; the external boost starts disabled (above). */
	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00, &regval);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read CHG_CNFG_00\n");

	chg->mode = FIELD_GET(MAX77779_CHGR_REG_CHG_CNFG_00_MODE, regval);

	config.dev = dev;
	config.driver_data = chg;
	config.of_node = dev_of_node(dev);

	chg->otg_rdev = devm_regulator_register(dev, &max77779_otg_desc,
						&config);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->otg_rdev),
				     "Failed to register OTG regulator\n");

	return 0;
}

static const struct platform_device_id max77779_charger_id[] = {
	{ .name = "max77779-charger", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77779_charger_id);

static struct platform_driver max77779_charger_driver = {
	.driver = {
		.name = "max77779-charger",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = max77779_charger_probe,
	.id_table = max77779_charger_id,
};
module_platform_driver(max77779_charger_driver);

MODULE_DESCRIPTION("Maxim MAX77779 charger OTG regulator driver");
MODULE_LICENSE("GPL");
