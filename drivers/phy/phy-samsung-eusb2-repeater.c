// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung eUSB2 repeater driver
 *
 * An I2C signal-conditioning repeater that sits between the SoC eUSB2 PHY and
 * the legacy USB 2.0 connector on Google Tensor (zuma/zumapro) boards. eUSB2 is
 * a low-swing on-die PHY, so this repeater is mandatory to produce real USB 2.0
 * signalling on the Type-C connector.
 *
 * The repeater is configured entirely at probe time (regulator on + register
 * tuning) so it works regardless of how the board device tree wires it: the
 * downstream/board DTS (zuma-caimito-usb.dtsi etc.) describes it only as a
 * standalone I2C device (no "phys" link back to the eUSB2 PHY), so it cannot
 * rely on the consuming PHY calling phy_init(). A "phys" link is still honoured
 * (the PHY is registered as a provider) for boards that wire it that way.
 *
 * Two device-tree tuning formats are accepted:
 *   - mainline:   samsung,tune-param = <reg value>, <reg value>, ...;
 *   - downstream: repeater_tune_param = <&tune_node>; with child nodes each
 *                 holding tune_value = <reg value shift mask>;
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/* Time for the repeater to become ready after vdd33 is applied (microseconds) */
#define SAMSUNG_EUSB_REPEATER_RH_READY_US	3000

/*
 * Global config register. Writing 0 here clears the per-port disable bits and
 * brings the repeater's data path out of its disabled state; without it no USB2
 * signalling reaches the connector even after tuning. (Matches the downstream
 * eusb_repeater_ctrl(enable) which writes 0 to I2C_GLOBAL_CONFIG.)
 */
#define SAMSUNG_EUSB_REPEATER_GLOBAL_CONFIG	0xb2

struct samsung_eusb_repeater_tune {
	u32 reg;
	u32 value;
	u32 shift;
	u32 mask;
};

struct samsung_eusb_repeater {
	struct device *dev;
	struct regmap *regmap;
	struct phy *phy;
	struct samsung_eusb_repeater_tune *tune;
	int num_tune;
};

static const struct regmap_config samsung_eusb_repeater_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xef,
};

/* Parse the mainline "samsung,tune-param" <reg value> pairs. */
static int samsung_eusb_repeater_parse_pairs(struct samsung_eusb_repeater *rptr,
					     int count)
{
	struct device *dev = rptr->dev;
	u32 *raw;
	int i, ret;

	if (count % 2)
		return dev_err_probe(dev, -EINVAL,
			"samsung,tune-param must hold <reg value> pairs\n");

	raw = kcalloc(count, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, "samsung,tune-param",
					 raw, count);
	if (ret) {
		kfree(raw);
		return dev_err_probe(dev, ret, "failed to read tune params\n");
	}

	rptr->num_tune = count / 2;
	rptr->tune = devm_kcalloc(dev, rptr->num_tune, sizeof(*rptr->tune),
				  GFP_KERNEL);
	if (!rptr->tune) {
		kfree(raw);
		return -ENOMEM;
	}

	for (i = 0; i < rptr->num_tune; i++) {
		rptr->tune[i].reg = raw[2 * i];
		rptr->tune[i].value = raw[2 * i + 1];
		rptr->tune[i].shift = 0;
		rptr->tune[i].mask = 0xff;
	}

	kfree(raw);
	return 0;
}

/*
 * Parse the downstream "repeater_tune_param" phandle: a node whose children
 * each carry tune_value = <reg value shift mask>.
 */
static int samsung_eusb_repeater_parse_node(struct samsung_eusb_repeater *rptr)
{
	struct device *dev = rptr->dev;
	struct device_node *tune_node, *child;
	int count, i = 0, ret;

	tune_node = of_parse_phandle(dev->of_node, "repeater_tune_param", 0);
	if (!tune_node)
		return 0;	/* no tuning described, not an error */

	count = of_get_child_count(tune_node);
	if (count <= 0) {
		of_node_put(tune_node);
		return 0;
	}

	rptr->tune = devm_kcalloc(dev, count, sizeof(*rptr->tune), GFP_KERNEL);
	if (!rptr->tune) {
		of_node_put(tune_node);
		return -ENOMEM;
	}

	for_each_child_of_node(tune_node, child) {
		u32 res[4];

		ret = of_property_read_u32_array(child, "tune_value", res, 4);
		if (ret) {
			of_node_put(child);
			of_node_put(tune_node);
			return dev_err_probe(dev, ret,
				"missing tune_value in %pOFn\n", child);
		}

		rptr->tune[i].reg = res[0];
		rptr->tune[i].value = res[1];
		rptr->tune[i].shift = res[2];
		rptr->tune[i].mask = res[3];
		i++;
	}

	rptr->num_tune = i;
	of_node_put(tune_node);
	return 0;
}

static int samsung_eusb_repeater_parse_tunes(struct samsung_eusb_repeater *rptr)
{
	int count;

	count = of_property_count_u32_elems(rptr->dev->of_node,
					    "samsung,tune-param");
	if (count > 0)
		return samsung_eusb_repeater_parse_pairs(rptr, count);

	return samsung_eusb_repeater_parse_node(rptr);
}

static int samsung_eusb_repeater_apply_tunes(struct samsung_eusb_repeater *rptr)
{
	int i, ret;

	for (i = 0; i < rptr->num_tune; i++) {
		struct samsung_eusb_repeater_tune *t = &rptr->tune[i];

		ret = regmap_update_bits(rptr->regmap, t->reg,
					 t->mask << t->shift,
					 (t->value & t->mask) << t->shift);
		if (ret) {
			dev_err(rptr->dev,
				"failed to write tune reg 0x%02x: %d\n",
				t->reg, ret);
			return ret;
		}
	}

	return 0;
}

/*
 * Hardware is fully configured at probe; phy_init() is a no-op so boards that
 * do wire a "phys" link (and thus call phy_init on us) still work.
 */
static int samsung_eusb_repeater_init(struct phy *phy)
{
	return 0;
}

static const struct phy_ops samsung_eusb_repeater_ops = {
	.init	= samsung_eusb_repeater_init,
	.owner	= THIS_MODULE,
};

static int samsung_eusb_repeater_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct samsung_eusb_repeater *rptr;
	struct phy_provider *phy_provider;
	int ret;

	rptr = devm_kzalloc(dev, sizeof(*rptr), GFP_KERNEL);
	if (!rptr)
		return -ENOMEM;

	rptr->dev = dev;

	rptr->regmap = devm_regmap_init_i2c(client,
					    &samsung_eusb_repeater_regmap);
	if (IS_ERR(rptr->regmap))
		return dev_err_probe(dev, PTR_ERR(rptr->regmap),
				     "failed to init regmap\n");

	ret = samsung_eusb_repeater_parse_tunes(rptr);
	if (ret)
		return ret;

	/* Power the repeater and let it come out of reset before any I2C. */
	ret = devm_regulator_get_enable(dev, "vdd33");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd33 supply\n");

	fsleep(SAMSUNG_EUSB_REPEATER_RH_READY_US);

	/* Bring the repeater data path out of its disabled state before tuning. */
	ret = regmap_write(rptr->regmap, SAMSUNG_EUSB_REPEATER_GLOBAL_CONFIG, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable repeater\n");

	ret = samsung_eusb_repeater_apply_tunes(rptr);
	if (ret)
		return ret;

	rptr->phy = devm_phy_create(dev, dev->of_node,
				    &samsung_eusb_repeater_ops);
	if (IS_ERR(rptr->phy))
		return dev_err_probe(dev, PTR_ERR(rptr->phy),
				     "failed to create PHY\n");

	phy_set_drvdata(rptr->phy, rptr);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	dev_info(dev, "eUSB2 repeater configured (%d tune regs)\n",
		 rptr->num_tune);

	return 0;
}

static const struct i2c_device_id samsung_eusb_repeater_id[] = {
	{ "eusb-repeater" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, samsung_eusb_repeater_id);

static const struct of_device_id samsung_eusb_repeater_of_match[] = {
	{ .compatible = "samsung,eusb-repeater" },
	{ }
};
MODULE_DEVICE_TABLE(of, samsung_eusb_repeater_of_match);

static struct i2c_driver samsung_eusb_repeater_driver = {
	.driver = {
		.name = "samsung-eusb2-repeater",
		.of_match_table = samsung_eusb_repeater_of_match,
	},
	.probe = samsung_eusb_repeater_probe,
	.id_table = samsung_eusb_repeater_id,
};
module_i2c_driver(samsung_eusb_repeater_driver);

MODULE_DESCRIPTION("Samsung eUSB2 repeater driver");
MODULE_LICENSE("GPL");
