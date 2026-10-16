// SPDX-License-Identifier: GPL-2.0+
/*
 * Dialog SLG51002 camera PMIC: eight LDOs plus four GPIO-driven outputs.
 *
 * Not an SLG51000: different register map (CONF1/2 current limits, LDO8,
 * GPIO outputs switched through the mux array in software test mode).
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define SLG51002_SYSCTL_PATN_ID_B0	0x1105
#define SLG51002_SYSCTL_MATRIX_CONF_A	0x110d
#define SLG51002_SYSCTL_TEST_EN		0x1119
#define SLG51002_SW_TEST_MODE_1		0x111a
#define SLG51002_MUXARRAY_INPUT_SEL_16	0x1710

#define SLG51002_REVISION_AA		0xafda01
#define SLG51002_REVISION_AB		0xb13103

#define SLG51002_NUM_LDOS		8
#define SLG51002_NUM_GPIOS		4

struct slg51002 {
	struct device *dev;
	struct regmap *regmap;
	u32 chip_id;
};

/* VSEL, MINV at +0x60, CONF1 at +0x64 (+0x65 for LDO6..8), per LDO base. */
static const unsigned int slg51002_ldo_base[SLG51002_NUM_LDOS] = {
	0x2000, 0x2200, 0x2300, 0x2500, 0x2700, 0x2900, 0x3100, 0x3200,
};

static int slg51002_test_mode(struct slg51002 *chip, bool on)
{
	static const u8 key[] = { 0x45, 0x53, 0x54, 0x4d };

	if (!on)
		return regmap_write(chip->regmap, SLG51002_SYSCTL_TEST_EN, 0);
	return regmap_bulk_write(chip->regmap, SLG51002_SW_TEST_MODE_1, key,
				 ARRAY_SIZE(key));
}

static int slg51002_gpio_enable(struct regulator_dev *rdev)
{
	struct slg51002 *chip = rdev_get_drvdata(rdev);
	int ret;

	/* Revision AA only takes mux writes in test mode */
	if (chip->chip_id != SLG51002_REVISION_AB)
		slg51002_test_mode(chip, true);
	ret = regulator_enable_regmap(rdev);
	if (chip->chip_id != SLG51002_REVISION_AB)
		slg51002_test_mode(chip, false);
	return ret;
}

static int slg51002_gpio_disable(struct regulator_dev *rdev)
{
	struct slg51002 *chip = rdev_get_drvdata(rdev);
	int ret;

	if (chip->chip_id != SLG51002_REVISION_AB)
		slg51002_test_mode(chip, true);
	ret = regulator_disable_regmap(rdev);
	if (chip->chip_id != SLG51002_REVISION_AB)
		slg51002_test_mode(chip, false);
	return ret;
}

static const struct regulator_ops slg51002_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_ops slg51002_gpio_ops = {
	.enable = slg51002_gpio_enable,
	.disable = slg51002_gpio_disable,
	.is_enabled = regulator_is_enabled_regmap,
};

#define SLG51002_LDO(_n, _supply, _min, _step)			\
	{							\
		.name = "ldo" #_n,				\
		.supply_name = _supply,				\
		.of_match = of_match_ptr("ldo" #_n),		\
		.regulators_node = of_match_ptr("regulators"),	\
		.id = _n - 1,					\
		.ops = &slg51002_ldo_ops,			\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
		.min_uV = _min,					\
		.uV_step = _step,				\
		.vsel_reg = 0, /* filled in from base */	\
		.vsel_mask = 0xff,				\
		.enable_reg = SLG51002_SYSCTL_MATRIX_CONF_A,	\
		.enable_mask = BIT(_n - 1),			\
	}

#define SLG51002_GPIO(_n)					\
	{							\
		.name = "gpio" #_n,				\
		.of_match = of_match_ptr("gpio" #_n),		\
		.regulators_node = of_match_ptr("regulators"),	\
		.id = SLG51002_NUM_LDOS + _n - 1,		\
		.ops = &slg51002_gpio_ops,			\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
		.enable_reg = SLG51002_MUXARRAY_INPUT_SEL_16 + _n - 1, \
		.enable_mask = BIT(0),				\
	}

static const struct regulator_desc slg51002_regulators[] = {
	SLG51002_LDO(1, "vin1_2", 1200000, 10000),
	SLG51002_LDO(2, "vin1_2", 1200000, 10000),
	SLG51002_LDO(3, "vin3", 1200000, 10000),
	SLG51002_LDO(4, "vin4", 1200000, 10000),
	SLG51002_LDO(5, "vin5", 1200000, 10000),
	SLG51002_LDO(6, "vin6", 400000, 5000),
	SLG51002_LDO(7, "vin7", 400000, 5000),
	SLG51002_LDO(8, "vin8", 400000, 5000),
	SLG51002_GPIO(1),
	SLG51002_GPIO(2),
	SLG51002_GPIO(3),
	SLG51002_GPIO(4),
};

/* OTP revision AA ships without the current limits the board expects. */
static int slg51002_init_aa(struct slg51002 *chip)
{
	int i, ret;

	for (i = 0; i < SLG51002_NUM_LDOS; i++) {
		unsigned int base = slg51002_ldo_base[i];
		unsigned int conf1 = base + (i >= 5 ? 0x65 : 0x64);
		unsigned int conf2 = i == 7 ? 0x32 : i >= 5 ? 0x2a : 0x2d;

		ret = regmap_write(chip->regmap, conf1, 0x0a);
		if (!ret)
			ret = regmap_write(chip->regmap, conf1 + 1, conf2);
		if (!ret && i >= 5)
			ret = regmap_write(chip->regmap, base + 0x63, 0);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct regmap_config slg51002_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x800f,
};

static int slg51002_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { .dev = dev };
	struct gpio_desc *cs;
	struct slg51002 *chip;
	u8 id[3];
	int i, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;

	/* CS high wakes the chip from reset; ready in ~10 ms */
	cs = devm_gpiod_get(dev, "dlg,cs", GPIOD_OUT_HIGH);
	if (IS_ERR(cs))
		return dev_err_probe(dev, PTR_ERR(cs), "no cs gpio\n");
	usleep_range(10000, 11000);

	chip->regmap = devm_regmap_init_i2c(client, &slg51002_regmap_config);
	if (IS_ERR(chip->regmap))
		return PTR_ERR(chip->regmap);

	ret = regmap_bulk_read(chip->regmap, SLG51002_SYSCTL_PATN_ID_B0, id, 3);
	if (ret)
		return dev_err_probe(dev, ret, "chip id read failed\n");
	chip->chip_id = id[2] << 16 | id[1] << 8 | id[0];
	dev_info(dev, "chip id 0x%06x\n", chip->chip_id);

	if (chip->chip_id == SLG51002_REVISION_AA) {
		slg51002_test_mode(chip, true);
		ret = slg51002_init_aa(chip);
		slg51002_test_mode(chip, false);
		if (ret)
			dev_warn(dev, "revision AA init failed: %d\n", ret);
	}

	config.regmap = chip->regmap;
	config.driver_data = chip;
	for (i = 0; i < ARRAY_SIZE(slg51002_regulators); i++) {
		struct regulator_desc *desc;
		struct regulator_dev *rdev;

		desc = devm_kmemdup(dev, &slg51002_regulators[i], sizeof(*desc),
				    GFP_KERNEL);
		if (!desc)
			return -ENOMEM;

		if (i < SLG51002_NUM_LDOS) {
			u8 range[2];

			desc->vsel_reg = slg51002_ldo_base[i];
			ret = regmap_bulk_read(chip->regmap,
					       slg51002_ldo_base[i] + 0x60,
					       range, 2);
			if (ret)
				return ret;
			desc->linear_min_sel = range[0];
			desc->n_voltages = range[1] + 1;
			desc->min_uV += range[0] * desc->uV_step;
		}

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n", desc->name);
	}

	return 0;
}

static const struct of_device_id slg51002_of_match[] = {
	{ .compatible = "dlg,slg51002" },
	{ }
};
MODULE_DEVICE_TABLE(of, slg51002_of_match);

static struct i2c_driver slg51002_driver = {
	.driver = {
		.name = "slg51002",
		.of_match_table = slg51002_of_match,
	},
	.probe = slg51002_probe,
};
module_i2c_driver(slg51002_driver);

MODULE_DESCRIPTION("Dialog SLG51002 regulator driver");
MODULE_LICENSE("GPL");
