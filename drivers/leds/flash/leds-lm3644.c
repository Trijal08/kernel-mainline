// SPDX-License-Identifier: GPL-2.0-only
/*
 * Texas Instruments LM3644 dual-channel flash LED driver
 *
 * Copyright (C) 2026 Oleksii Onchul <oleksiionchul@gmail.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define LM3644_REG_ENABLE			0x01
#define LM3644_REG_FLASH_LED1			0x03
#define LM3644_REG_FLASH_LED2			0x04
#define LM3644_REG_TORCH_LED1			0x05
#define LM3644_REG_TORCH_LED2			0x06
#define LM3644_REG_TIMING			0x08
#define LM3644_REG_FLAGS1			0x0a
#define LM3644_REG_FLAGS2			0x0b
#define LM3644_REG_DEVICE_ID			0x0c

#define LM3644_ENABLE_LED1			BIT(0)
#define LM3644_ENABLE_LED2			BIT(1)
#define LM3644_ENABLE_LEDS			GENMASK(1, 0)
#define LM3644_ENABLE_MODE			GENMASK(3, 2)
#define LM3644_MODE_STANDBY			0
#define LM3644_MODE_TORCH			BIT(3)
#define LM3644_MODE_FLASH			GENMASK(3, 2)

#define LM3644_CURRENT_MASK			GENMASK(6, 0)
#define LM3644_LED2_OVERRIDE			BIT(7)

#define LM3644_TIMING_TIMEOUT			GENMASK(3, 0)

#define LM3644_FLAG1_TIMEOUT			BIT(0)
#define LM3644_FLAG1_UVLO			BIT(1)
#define LM3644_FLAG1_THERMAL_SHUTDOWN		BIT(2)
#define LM3644_FLAG1_CURRENT_LIMIT		BIT(3)
#define LM3644_FLAG1_LED2_SHORT			BIT(4)
#define LM3644_FLAG1_LED1_SHORT			BIT(5)
#define LM3644_FLAG1_VOUT_SHORT			BIT(6)

#define LM3644_FLAG2_TEMP_TRIP			BIT(0)
#define LM3644_FLAG2_OVP			BIT(1)
#define LM3644_FLAG2_IVFM			BIT(2)

#define LM3644_LED1				BIT(0)
#define LM3644_LED2				BIT(1)
#define LM3644_LED_ALL				(LM3644_LED1 | LM3644_LED2)

#define LM3644_FLASH_MIN_UA			10900U
#define LM3644_FLASH_STEP_UA			11725U
#define LM3644_FLASH_MAX_UA			1500000U

#define LM3644_TORCH_MIN_UA			977U
#define LM3644_TORCH_STEP_UA			1400U
#define LM3644_TT_TORCH_MIN_UA			1954U
#define LM3644_TT_TORCH_STEP_UA			2800U

#define LM3644_TIMEOUT_MIN_US			10000U
#define LM3644_TIMEOUT_STEP_US			10000U
#define LM3644_TIMEOUT_LINEAR_MAX_US		100000U
#define LM3644_TT_TIMEOUT_MIN_US		40000U
#define LM3644_TT_TIMEOUT_STEP_US		40000U
#define LM3644_TT_TIMEOUT_LINEAR_MAX_US		400000U

struct lm3644 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct mutex lock; /* Protects register accesses. */
	struct led_classdev_flash flash;
	u32 sources;
	u32 torch_min_ua;
	u32 torch_step_ua;
	u32 timeout_min_us;
	u32 timeout_step_us;
	u32 timeout_max_us;
};

static u8 lm3644_source_enable_mask(struct lm3644 *chip)
{
	u8 mask = 0;

	if (chip->sources & LM3644_LED1)
		mask |= LM3644_ENABLE_LED1;
	if (chip->sources & LM3644_LED2)
		mask |= LM3644_ENABLE_LED2;

	return mask;
}

static int lm3644_set_current(struct lm3644 *chip, unsigned int led1_reg,
			      unsigned int led2_reg, u8 code)
{
	int ret;

	if (chip->sources == LM3644_LED_ALL)
		return regmap_write(chip->regmap, led1_reg,
				    LM3644_LED2_OVERRIDE | code);

	ret = regmap_update_bits(chip->regmap, led1_reg,
				 LM3644_LED2_OVERRIDE, 0);
	if (ret)
		return ret;

	if (chip->sources & LM3644_LED1)
		ret = regmap_update_bits(chip->regmap, led1_reg,
					 LM3644_CURRENT_MASK, code);
	else
		ret = regmap_update_bits(chip->regmap, led2_reg,
					 LM3644_CURRENT_MASK, code);

	return ret;
}

static int lm3644_torch_brightness_set(struct led_classdev *led_cdev,
				       enum led_brightness brightness)
{
	struct led_classdev_flash *flash = lcdev_to_flcdev(led_cdev);
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	u8 enable = 0;
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);

	ret = regmap_read(chip->regmap, LM3644_REG_ENABLE, &val);
	if (ret)
		goto out;

	if (brightness && (val & LM3644_ENABLE_MODE) == LM3644_MODE_FLASH) {
		ret = -EBUSY;
		goto out;
	}

	if (!brightness) {
		ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
					 LM3644_ENABLE_MODE | LM3644_ENABLE_LEDS,
					 LM3644_MODE_STANDBY);
	} else if ((val & LM3644_ENABLE_MODE) == LM3644_MODE_TORCH) {
		ret = lm3644_set_current(chip, LM3644_REG_TORCH_LED1,
					 LM3644_REG_TORCH_LED2,
					 brightness - 1);
	} else {
		ret = lm3644_set_current(chip, LM3644_REG_TORCH_LED1,
					 LM3644_REG_TORCH_LED2,
					 brightness - 1);
		if (ret)
			goto out;

		enable = LM3644_MODE_TORCH | lm3644_source_enable_mask(chip);
		ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
					 LM3644_ENABLE_MODE | LM3644_ENABLE_LEDS,
					 enable);
	}

out:
	mutex_unlock(&chip->lock);
	return ret;
}

static enum led_brightness
lm3644_torch_brightness_get(struct led_classdev *led_cdev)
{
	struct led_classdev_flash *flash = lcdev_to_flcdev(led_cdev);
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	unsigned int enable;
	unsigned int brightness;
	unsigned int reg;
	int ret;

	mutex_lock(&chip->lock);

	ret = regmap_read(chip->regmap, LM3644_REG_ENABLE, &enable);
	if (ret)
		goto error;

	if ((enable & LM3644_ENABLE_MODE) != LM3644_MODE_TORCH) {
		mutex_unlock(&chip->lock);
		return LED_OFF;
	}

	reg = chip->sources & LM3644_LED1 ?
	      LM3644_REG_TORCH_LED1 : LM3644_REG_TORCH_LED2;
	ret = regmap_read(chip->regmap, reg, &brightness);
	if (ret)
		goto error;

	mutex_unlock(&chip->lock);
	return (brightness & LM3644_CURRENT_MASK) + 1;

error:
	mutex_unlock(&chip->lock);
	return LED_OFF;
}

static int
lm3644_flash_brightness_set(struct led_classdev_flash *flash, u32 brightness)
{
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	struct led_flash_setting *setting = &flash->brightness;
	u8 code = (brightness - setting->min) / setting->step;
	int ret;

	mutex_lock(&chip->lock);
	ret = lm3644_set_current(chip, LM3644_REG_FLASH_LED1,
				 LM3644_REG_FLASH_LED2, code);
	mutex_unlock(&chip->lock);

	return ret;
}

static int lm3644_strobe_set(struct led_classdev_flash *flash, bool state)
{
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	u8 enable = 0;
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);

	ret = regmap_read(chip->regmap, LM3644_REG_ENABLE, &val);
	if (ret)
		goto out;

	if (state && (val & LM3644_ENABLE_MODE) == LM3644_MODE_TORCH) {
		ret = -EBUSY;
		goto out;
	}

	if (state)
		enable = LM3644_MODE_FLASH | lm3644_source_enable_mask(chip);

	ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
				 LM3644_ENABLE_MODE | LM3644_ENABLE_LEDS,
				 enable);

out:
	mutex_unlock(&chip->lock);
	return ret;
}

static int lm3644_strobe_get(struct led_classdev_flash *flash, bool *state)
{
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	unsigned int enable;
	int ret;

	mutex_lock(&chip->lock);
	ret = regmap_read(chip->regmap, LM3644_REG_ENABLE, &enable);
	mutex_unlock(&chip->lock);

	if (!ret)
		*state = (enable & LM3644_ENABLE_MODE) == LM3644_MODE_FLASH;

	return ret;
}

static int lm3644_timeout_set(struct led_classdev_flash *flash, u32 timeout)
{
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	u8 code = (timeout - chip->timeout_min_us) / chip->timeout_step_us;
	int ret;

	mutex_lock(&chip->lock);
	ret = regmap_update_bits(chip->regmap, LM3644_REG_TIMING,
				 LM3644_TIMING_TIMEOUT, code);
	mutex_unlock(&chip->lock);

	return ret;
}

static int lm3644_fault_get(struct led_classdev_flash *flash, u32 *fault)
{
	struct lm3644 *chip = container_of(flash, struct lm3644, flash);
	unsigned int flags1;
	unsigned int flags2;
	u32 faults = 0;
	int ret;

	mutex_lock(&chip->lock);
	ret = regmap_read(chip->regmap, LM3644_REG_FLAGS1, &flags1);
	if (!ret)
		ret = regmap_read(chip->regmap, LM3644_REG_FLAGS2, &flags2);
	mutex_unlock(&chip->lock);
	if (ret)
		return ret;

	if (flags1 & LM3644_FLAG1_TIMEOUT)
		faults |= LED_FAULT_TIMEOUT;
	if (flags1 & LM3644_FLAG1_UVLO)
		faults |= LED_FAULT_UNDER_VOLTAGE;
	if (flags1 & LM3644_FLAG1_THERMAL_SHUTDOWN)
		faults |= LED_FAULT_OVER_TEMPERATURE;
	if (flags1 & LM3644_FLAG1_CURRENT_LIMIT)
		faults |= LED_FAULT_OVER_CURRENT;
	if (flags1 & (LM3644_FLAG1_LED1_SHORT |
		      LM3644_FLAG1_LED2_SHORT |
		      LM3644_FLAG1_VOUT_SHORT))
		faults |= LED_FAULT_SHORT_CIRCUIT;
	if (flags2 & LM3644_FLAG2_TEMP_TRIP)
		faults |= LED_FAULT_LED_OVER_TEMPERATURE;
	if (flags2 & LM3644_FLAG2_OVP)
		faults |= LED_FAULT_OVER_VOLTAGE;
	if (flags2 & LM3644_FLAG2_IVFM)
		faults |= LED_FAULT_INPUT_VOLTAGE;

	*fault = faults;
	return 0;
}

static const struct led_flash_ops lm3644_flash_ops = {
	.flash_brightness_set = lm3644_flash_brightness_set,
	.strobe_set = lm3644_strobe_set,
	.strobe_get = lm3644_strobe_get,
	.timeout_set = lm3644_timeout_set,
	.fault_get = lm3644_fault_get,
};

static int lm3644_parse_led(struct lm3644 *chip,
			    struct fwnode_handle *child)
{
	struct led_classdev_flash *flash = &chip->flash;
	struct led_classdev *led_cdev = &flash->led_cdev;
	struct led_flash_setting *setting;
	u32 max_ua;
	u32 sources[2];
	int count;
	int ret;
	int i;

	count = fwnode_property_count_u32(child, "led-sources");
	if (count < 1 || count > ARRAY_SIZE(sources))
		return dev_err_probe(chip->dev, -EINVAL,
				     "expected one or two LED sources\n");

	ret = fwnode_property_read_u32_array(child, "led-sources",
					     sources, count);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		if (sources[i] > 1 || chip->sources & BIT(sources[i]))
			return dev_err_probe(chip->dev, -EINVAL,
					     "invalid LED sources\n");
		chip->sources |= BIT(sources[i]);
	}

	ret = fwnode_property_read_u32(child, "led-max-microamp", &max_ua);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "missing led-max-microamp\n");

	max_ua = clamp_val(max_ua, chip->torch_min_ua,
			   chip->torch_min_ua +
			   LM3644_CURRENT_MASK * chip->torch_step_ua);
	led_cdev->max_brightness =
		(max_ua - chip->torch_min_ua) / chip->torch_step_ua + 1;
	led_cdev->brightness_set_blocking = lm3644_torch_brightness_set;
	led_cdev->brightness_get = lm3644_torch_brightness_get;
	led_cdev->flags |= LED_DEV_CAP_FLASH | LED_CORE_SUSPENDRESUME;

	ret = fwnode_property_read_u32(child, "flash-max-microamp", &max_ua);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "missing flash-max-microamp\n");

	setting = &flash->brightness;
	setting->min = LM3644_FLASH_MIN_UA;
	setting->step = LM3644_FLASH_STEP_UA;
	setting->max = clamp_val(max_ua, setting->min,
				 LM3644_FLASH_MAX_UA);
	setting->max = setting->min +
		       (setting->max - setting->min) / setting->step *
		       setting->step;
	setting->val = setting->max;

	ret = fwnode_property_read_u32(child, "flash-max-timeout-us",
				       &max_ua);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "missing flash-max-timeout-us\n");

	setting = &flash->timeout;
	setting->min = chip->timeout_min_us;
	setting->step = chip->timeout_step_us;
	setting->max = clamp_val(max_ua, setting->min,
				 chip->timeout_max_us);
	setting->max = setting->min +
		       (setting->max - setting->min) / setting->step *
		       setting->step;
	setting->val = setting->max;

	flash->ops = &lm3644_flash_ops;

	return 0;
}

static void lm3644_disable(void *data)
{
	struct lm3644 *chip = data;

	regmap_write(chip->regmap, LM3644_REG_ENABLE, LM3644_MODE_STANDBY);
	gpiod_set_value_cansleep(chip->enable_gpio, 0);
}

static bool lm3644_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == LM3644_REG_FLAGS1 || reg == LM3644_REG_FLAGS2;
}

static const struct regmap_config lm3644_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x0d,
	.volatile_reg = lm3644_volatile_reg,
};

static int lm3644_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_init_data init_data = {};
	struct fwnode_handle *child;
	struct lm3644 *chip;
	unsigned int device_id;
	unsigned int unused;
	int ret;

	if (device_get_child_node_count(dev) != 1)
		return dev_err_probe(dev, -EINVAL,
				     "expected exactly one LED child node\n");

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	chip->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->enable_gpio),
				     "failed to request enable GPIO\n");

	usleep_range(1000, 2000);

	chip->regmap = devm_regmap_init_i2c(client, &lm3644_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to allocate register map\n");

	ret = devm_add_action_or_reset(dev, lm3644_disable, chip);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, LM3644_REG_DEVICE_ID, &device_id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device ID\n");

	if ((device_id & 0x7) == 0x2 || (device_id & 0x7) == 0x4) {
		chip->torch_min_ua = LM3644_TT_TORCH_MIN_UA;
		chip->torch_step_ua = LM3644_TT_TORCH_STEP_UA;
		chip->timeout_min_us = LM3644_TT_TIMEOUT_MIN_US;
		chip->timeout_step_us = LM3644_TT_TIMEOUT_STEP_US;
		chip->timeout_max_us = LM3644_TT_TIMEOUT_LINEAR_MAX_US;
	} else {
		chip->torch_min_ua = LM3644_TORCH_MIN_UA;
		chip->torch_step_ua = LM3644_TORCH_STEP_UA;
		chip->timeout_min_us = LM3644_TIMEOUT_MIN_US;
		chip->timeout_step_us = LM3644_TIMEOUT_STEP_US;
		chip->timeout_max_us = LM3644_TIMEOUT_LINEAR_MAX_US;
	}

	ret = regmap_write(chip->regmap, LM3644_REG_ENABLE,
			   LM3644_MODE_STANDBY);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enter standby\n");

	/* Reading both flag registers clears any latched power-on faults. */
	regmap_read(chip->regmap, LM3644_REG_FLAGS1, &unused);
	regmap_read(chip->regmap, LM3644_REG_FLAGS2, &unused);

	child = device_get_next_child_node(dev, NULL);
	if (!child)
		return -EINVAL;

	ret = lm3644_parse_led(chip, child);
	if (ret)
		goto put_child;

	ret = lm3644_flash_brightness_set(&chip->flash,
					  chip->flash.brightness.val);
	if (ret)
		goto put_child;

	ret = lm3644_timeout_set(&chip->flash, chip->flash.timeout.val);
	if (ret)
		goto put_child;

	init_data.fwnode = child;
	ret = devm_led_classdev_flash_register_ext(dev, &chip->flash,
						   &init_data);

put_child:
	fwnode_handle_put(child);
	return ret;
}

static void lm3644_shutdown(struct i2c_client *client)
{
	struct lm3644 *chip = i2c_get_clientdata(client);

	lm3644_disable(chip);
}

static const struct of_device_id lm3644_of_match[] = {
	{ .compatible = "ti,lm3644" },
	{}
};
MODULE_DEVICE_TABLE(of, lm3644_of_match);

static const struct i2c_device_id lm3644_id[] = {
	{ "lm3644" },
	{}
};
MODULE_DEVICE_TABLE(i2c, lm3644_id);

static struct i2c_driver lm3644_driver = {
	.driver = {
		.name = "lm3644",
		.of_match_table = lm3644_of_match,
	},
	.probe = lm3644_probe,
	.shutdown = lm3644_shutdown,
	.id_table = lm3644_id,
};
module_i2c_driver(lm3644_driver);

MODULE_AUTHOR("Oleksii Onchul <oleksiionchul@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments LM3644 flash LED driver");
MODULE_LICENSE("GPL");
