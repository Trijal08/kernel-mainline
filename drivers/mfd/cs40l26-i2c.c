// SPDX-License-Identifier: GPL-2.0
/*
 * CS40L26 Boosted Haptic Driver with Integrated DSP and Waveform Memory
 *
 * Copyright 2024 Cirrus Logic, Inc.
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/i2c.h>
#include <linux/mfd/cs40l26.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

static int cs40l26_i2c_probe(struct i2c_client *client)
{
	struct cs40l26 *cs40l26;

	cs40l26 = devm_kzalloc(&client->dev, sizeof(*cs40l26), GFP_KERNEL);
	if (!cs40l26)
		return -ENOMEM;

	i2c_set_clientdata(client, cs40l26);

	cs40l26->regmap = devm_regmap_init_i2c(client, &cs40l26_regmap);
	if (IS_ERR(cs40l26->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(cs40l26->regmap),
				     "Failed to initialize register map\n");

	cs40l26->dev = &client->dev;

	return cs40l26_probe(cs40l26);
}

static const struct i2c_device_id cs40l26_id_i2c[] = {
	{ "cs40l26a" },
	{ "cs40l26b" },
	{ "cs40l27a" },
	{ "cs40l27b" },
	{}
};
MODULE_DEVICE_TABLE(i2c, cs40l26_id_i2c);

static const struct of_device_id cs40l26_of_match[] = {
	{ .compatible = "cirrus,cs40l26a" },
	{ .compatible = "cirrus,cs40l26b" },
	{ .compatible = "cirrus,cs40l27a" },
	{ .compatible = "cirrus,cs40l27b" },
	{}
};
MODULE_DEVICE_TABLE(of, cs40l26_of_match);

static struct i2c_driver cs40l26_i2c_driver = {
	.driver = {
		.name		= "cs40l26",
		.of_match_table	= cs40l26_of_match,
		.pm		= pm_ptr(&cs40l26_pm_ops),
	},
	.id_table	= cs40l26_id_i2c,
	.probe		= cs40l26_i2c_probe,
};
module_i2c_driver(cs40l26_i2c_driver);

MODULE_DESCRIPTION("CS40L26 I2C Driver");
MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_LICENSE("GPL");
