// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only fuel-gauge driver for the Maxim MAX77779.
 *
 * The MAX77779 PMIC integrates a ModelGauge m5 fuel gauge reachable as an
 * independent I2C device (default address 0x36). This driver exposes the
 * gauge's autonomously tracked state (state-of-charge, voltage, current,
 * temperature) through the power_supply class.
 *
 * It never touches a charge rail. The gauge keeps its model in battery-backed
 * RAM, so a power-on-reset wipes it and state-of-charge stays unavailable until
 * a model is loaded again; the driver checks Status.POR / FStat.DNR before
 * returning any model-derived value rather than handing back a default-model
 * guess. When POR is pending and the device tree carries model data, probe
 * loads it (ModelGauge m5 loading procedure 0.2.1), which is the only way back
 * on a board whose bootloader does not do it.
 *
 * The register layout and the ModelGauge m5 conversions mirror the in-tree
 * max1720x_battery.c driver.
 *
 * Copyright (C) 2026 Steffen Deusch
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* ModelGauge m5 registers (16-bit, little-endian on the wire) */
#define MAX77779_FG_STATUS		0x00
#define MAX77779_FG_STATUS_POR		BIT(1)	/* model lost since last load */
#define MAX77779_FG_STATUS_BST		BIT(3)	/* 0 = battery present */
#define MAX77779_FG_HIBCFG		0x02
#define MAX77779_FG_REPCAP		0x06
#define MAX77779_FG_REPSOC		0x07
#define MAX77779_FG_FULLCAPREP		0x10
#define MAX77779_FG_AVGVCELL		0x19
#define MAX77779_FG_VCELL		0x1a
#define MAX77779_FG_TEMP		0x1b
#define MAX77779_FG_CURRENT		0x1c
#define MAX77779_FG_AVGCURRENT		0x1d
#define MAX77779_FG_DEVNAME		0x21
#define MAX77779_FG_DEVNAME_RADIX	GENMASK(15, 8)
#define MAX77779_FG_FSTAT		0x3d
#define MAX77779_FG_FSTAT_DNR		BIT(0)	/* data not ready */
#define MAX77779_FG_BOOT_CHECK		0x40
#define MAX77779_FG_BOOT_CHECK_OK	0x82	/* on-die RISC-V came up */
#define MAX77779_FG_MODEL_START		0x80
#define MAX77779_FG_MODEL_WORDS		32
#define MAX77779_FG_CONFIG2		0xab
#define MAX77779_FG_CONFIG2_LDMDL	BIT(15)
#define MAX77779_FG_INT_STS		0xe0
#define MAX77779_FG_INT_STS_POR		BIT(1)
#define MAX77779_FG_USR			0xff
#define MAX77779_FG_USR_LOCK		(BIT(1) | BIT(2) | BIT(3)) /* N/V/R lock */

#define MAX77779_FG_MAX_REG		0xff

/*
 * Most model parameters live in the gauge's non-volatile shadow on its own I2C
 * address; only these three are plain gauge registers. Keeping the device-tree
 * list a flat (register, value) array means a new battery's block can be copied
 * across verbatim, so the split is resolved here instead.
 */
#define MAX77779_FG_NVM_ADDRESS		0x37
#define MAX77779_FG_NVM_NHIBCFG		0xbb
#define MAX77779_FG_HIBCFG_DEFAULT	0x0909

#define MAX77779_FG_DPACC		0x46

static const u8 max77779_fg_direct_params[] = {
	0x0c,	/* Config */
	MAX77779_FG_DPACC,
	MAX77779_FG_CONFIG2,
};

/* Treat |Current| below this raw value as "no measurable flow". */
#define MAX77779_FG_CURRENT_DEADBAND	8

struct max77779_fg {
	struct regmap *regmap;
	struct regmap *nvm;	/* non-volatile shadow, NULL if no model data */
	int rsense;	/* micro-ohms; 0 = unknown -> current/charge disabled */
};

static bool max77779_fg_devname_valid(unsigned int devname)
{
	switch (FIELD_GET(MAX77779_FG_DEVNAME_RADIX, devname)) {
	case 0x51:
	case 0x62:
	case 0x63:
		return true;
	default:
		return false;
	}
}

/*
 * The state-of-charge and capacity registers are only meaningful once the
 * battery model is loaded. Returns 0 if the model is ready, -ENODATA if it is
 * not (POR pending or data-not-ready), or a negative errno on bus failure.
 */
static int max77779_fg_model_ready(struct max77779_fg *fg)
{
	unsigned int status, fstat;
	int ret;

	ret = regmap_read(fg->regmap, MAX77779_FG_STATUS, &status);
	if (ret)
		return ret;

	ret = regmap_read(fg->regmap, MAX77779_FG_FSTAT, &fstat);
	if (ret)
		return ret;

	if ((status & MAX77779_FG_STATUS_POR) || (fstat & MAX77779_FG_FSTAT_DNR))
		return -ENODATA;

	return 0;
}

/*
 * The lock bits cover three register sections; the model load needs all of
 * them. Written twice, as the vendor driver does, so that the pair of writes
 * holds the regmap lock for the whole update.
 */
static int max77779_fg_set_lock(struct max77779_fg *fg, bool lock)
{
	struct reg_sequence cmds[2];
	unsigned int usr;
	int ret;

	ret = regmap_read(fg->regmap, MAX77779_FG_USR, &usr);
	if (ret)
		return ret;

	if (lock)
		usr |= MAX77779_FG_USR_LOCK;
	else
		usr &= ~MAX77779_FG_USR_LOCK;

	cmds[0].reg = cmds[1].reg = MAX77779_FG_USR;
	cmds[0].def = cmds[1].def = usr;
	cmds[0].delay_us = cmds[1].delay_us = 0;

	return regmap_multi_reg_write(fg->regmap, cmds, ARRAY_SIZE(cmds));
}

static int max77779_fg_write_model(struct device *dev, struct max77779_fg *fg,
				   const u16 *model)
{
	u16 readback[MAX77779_FG_MODEL_WORDS];
	int ret;

	ret = regmap_write(fg->regmap, MAX77779_FG_REPCAP, 0);
	if (ret)
		return ret;

	ret = regmap_raw_write(fg->regmap, MAX77779_FG_MODEL_START, model,
			       sizeof(readback));
	if (ret)
		return ret;

	ret = regmap_raw_read(fg->regmap, MAX77779_FG_MODEL_START, readback,
			      sizeof(readback));
	if (ret)
		return ret;

	if (memcmp(model, readback, sizeof(readback))) {
		dev_err(dev, "model did not read back\n");
		return -EIO;
	}

	return 0;
}

static int max77779_fg_write_params(struct max77779_fg *fg, const u16 *params,
				    int pairs, u16 *config2)
{
	int i, j, ret;

	for (i = 0; i < pairs; i++) {
		u16 reg = params[2 * i], val = params[2 * i + 1];
		struct regmap *map = fg->nvm;

		for (j = 0; j < ARRAY_SIZE(max77779_fg_direct_params); j++) {
			if (reg == max77779_fg_direct_params[j]) {
				map = fg->regmap;
				break;
			}
		}

		if (reg == MAX77779_FG_CONFIG2)
			*config2 = val;

		/* dPAcc does not always take; the vendor driver retries too. */
		if (reg == MAX77779_FG_DPACC) {
			unsigned int read;
			int try;

			for (try = 0; try < 3; try++) {
				ret = regmap_write(map, reg, val);
				if (ret)
					continue;

				usleep_range(2000, 2500);

				ret = regmap_read(map, reg, &read);
				if (!ret && read == val)
					break;
				ret = -EIO;
			}
			if (ret)
				return ret;

			continue;
		}

		ret = regmap_write(map, reg, val);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * ModelGauge m5 loading procedure 0.2.1, as the vendor driver implements it.
 * Deliberately left out: saving and restoring learned state (capacity,
 * resistance, cycle count) across the load, which needs persistent storage this
 * driver does not have, and the workarounds for gauge firmware older than 2.6.
 * A fresh load therefore starts the battery's learning over, which is the cost
 * of the POR having wiped it in the first place.
 */
static int max77779_fg_load_model(struct device *dev, struct max77779_fg *fg,
				  const u16 *model, const u16 *params, int pairs)
{
	unsigned int val;
	u16 config2, hibcfg;
	int ret;

	ret = regmap_read_poll_timeout(fg->regmap, MAX77779_FG_BOOT_CHECK, val,
				       (val & MAX77779_FG_BOOT_CHECK_OK) ==
				       MAX77779_FG_BOOT_CHECK_OK,
				       10 * USEC_PER_MSEC, 200 * USEC_PER_MSEC);
	if (ret)
		return dev_err_probe(dev, ret, "gauge core did not come up\n");

	ret = regmap_read_poll_timeout(fg->regmap, MAX77779_FG_FSTAT, val,
				       !(val & MAX77779_FG_FSTAT_DNR),
				       10 * USEC_PER_MSEC, 200 * USEC_PER_MSEC);
	if (ret)
		return dev_err_probe(dev, ret, "FStat.DNR never cleared\n");

	ret = max77779_fg_set_lock(fg, false);
	if (ret)
		return dev_err_probe(dev, ret, "cannot unlock the gauge\n");

	ret = regmap_read(fg->regmap, MAX77779_FG_HIBCFG, &val);
	if (ret)
		goto relock;
	hibcfg = val ? val : MAX77779_FG_HIBCFG_DEFAULT;

	/* Hibernation has to be off across the load. */
	ret = regmap_write(fg->regmap, MAX77779_FG_HIBCFG, 0);
	if (ret)
		goto relock;

	ret = max77779_fg_write_model(dev, fg, model);
	if (ret)
		goto relock;

	ret = regmap_read(fg->regmap, MAX77779_FG_CONFIG2, &val);
	if (ret)
		goto relock;
	config2 = val;

	ret = max77779_fg_write_params(fg, params, pairs, &config2);
	if (ret)
		goto relock;

	ret = regmap_write(fg->nvm, MAX77779_FG_NVM_NHIBCFG, hibcfg);
	if (ret)
		goto relock;

	ret = regmap_write(fg->regmap, MAX77779_FG_CONFIG2,
			   config2 | MAX77779_FG_CONFIG2_LDMDL);
	if (ret)
		goto relock;

	ret = regmap_read_poll_timeout(fg->regmap, MAX77779_FG_CONFIG2, val,
				       !(val & MAX77779_FG_CONFIG2_LDMDL),
				       10 * USEC_PER_MSEC, USEC_PER_SEC);
	if (ret) {
		dev_err(dev, "model load never completed\n");
		goto relock;
	}

	ret = regmap_write(fg->regmap, MAX77779_FG_CONFIG2, config2);
	if (ret)
		goto relock;

	/* The gauge needs this long to republish its internal registers. */
	msleep(200);

	/*
	 * POR is write-one-to-clear and does not take until the gauge has
	 * finished digesting the model, so this has to be retried.
	 */
	ret = -ETIMEDOUT;
	for (int i = 0; i < 10; i++) {
		if (regmap_write(fg->regmap, MAX77779_FG_INT_STS,
				 MAX77779_FG_INT_STS_POR))
			continue;

		msleep(100);

		if (regmap_read(fg->regmap, MAX77779_FG_INT_STS, &val))
			continue;

		if (!(val & MAX77779_FG_INT_STS_POR)) {
			ret = 0;
			break;
		}
	}
	if (ret)
		dev_err(dev, "cannot clear POR, FG_INT_STS=0x%04x\n", val);

relock:
	if (max77779_fg_set_lock(fg, true))
		dev_err(dev, "cannot lock the gauge again\n");

	return ret;
}

static int max77779_fg_uvolt(unsigned int reg)
{
	return reg * 625 / 8;	/* 78.125 uV/LSB */
}

static int max77779_fg_temp(unsigned int reg)
{
	return (s16)reg * 10 / 256;	/* tenths of a degree Celsius */
}

static int max77779_fg_uamp(struct max77779_fg *fg, unsigned int reg)
{
	/* signed; 1.5625 uV / Rsense per LSB */
	return div_s64((s64)(s16)reg * 1562500, fg->rsense);
}

static int max77779_fg_uah(struct max77779_fg *fg, unsigned int reg)
{
	/* 5.0 uVh / Rsense per LSB */
	return div_s64((s64)reg * 5000000, fg->rsense);
}

static int max77779_fg_status(struct max77779_fg *fg,
			      union power_supply_propval *val)
{
	unsigned int cur, soc;
	int ret;

	ret = max77779_fg_model_ready(fg);
	if (ret == -ENODATA) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	} else if (ret) {
		return ret;
	}

	ret = regmap_read(fg->regmap, MAX77779_FG_CURRENT, &cur);
	if (ret)
		return ret;

	if ((s16)cur > MAX77779_FG_CURRENT_DEADBAND) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	} else if ((s16)cur < -MAX77779_FG_CURRENT_DEADBAND) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		ret = regmap_read(fg->regmap, MAX77779_FG_REPSOC, &soc);
		if (ret)
			return ret;
		val->intval = (soc >> 8) >= 100 ? POWER_SUPPLY_STATUS_FULL :
						  POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return 0;
}

static int max77779_fg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77779_fg *fg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(fg->regmap, MAX77779_FG_STATUS, &reg);
		if (ret) {
			val->intval = 0;
			return 0;
		}
		val->intval = !(reg & MAX77779_FG_STATUS_BST);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		return max77779_fg_status(fg, val);
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max77779_fg_model_ready(fg);
		if (ret)
			return ret;
		ret = regmap_read(fg->regmap, MAX77779_FG_REPSOC, &reg);
		if (ret)
			return ret;
		val->intval = min(reg >> 8, 100u);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(fg->regmap, MAX77779_FG_VCELL, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uvolt(reg);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(fg->regmap, MAX77779_FG_AVGVCELL, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uvolt(reg);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = regmap_read(fg->regmap, MAX77779_FG_TEMP, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_temp(reg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!fg->rsense)
			return -ENODATA;
		ret = regmap_read(fg->regmap, MAX77779_FG_CURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uamp(fg, reg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (!fg->rsense)
			return -ENODATA;
		ret = regmap_read(fg->regmap, MAX77779_FG_AVGCURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uamp(fg, reg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!fg->rsense)
			return -ENODATA;
		ret = max77779_fg_model_ready(fg);
		if (ret)
			return ret;
		ret = regmap_read(fg->regmap, MAX77779_FG_REPCAP, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uah(fg, reg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (!fg->rsense)
			return -ENODATA;
		ret = max77779_fg_model_ready(fg);
		if (ret)
			return ret;
		ret = regmap_read(fg->regmap, MAX77779_FG_FULLCAPREP, &reg);
		if (ret)
			return ret;
		val->intval = max77779_fg_uah(fg, reg);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "max77779fg";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Maxim Integrated";
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property max77779_fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct regmap_config max77779_fg_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX77779_FG_MAX_REG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static const struct power_supply_desc max77779_fg_desc = {
	.name = "max77779fg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = max77779_fg_props,
	.num_properties = ARRAY_SIZE(max77779_fg_props),
	.get_property = max77779_fg_get_property,
};

static void max77779_fg_unregister_nvm(void *client)
{
	i2c_unregister_device(client);
}

/*
 * The model table is a fixed 32-word block; the parameters are a flat
 * (register, value) array so a battery's block can be copied from the vendor
 * device tree unchanged. Returns -ENODATA when the device tree carries no
 * model, which is not an error: the gauge simply cannot be recovered from a POR
 * on that board.
 */
static int max77779_fg_load_from_fwnode(struct i2c_client *client,
					struct max77779_fg *fg)
{
	struct device *dev = &client->dev;
	u16 model[MAX77779_FG_MODEL_WORDS], *params;
	struct i2c_client *nvm;
	int count, ret;

	count = device_property_count_u16(dev, "maxim,fg-model");
	if (count < 0)
		return -ENODATA;
	if (count != ARRAY_SIZE(model))
		return dev_err_probe(dev, -EINVAL,
				     "maxim,fg-model has %d words, need %zu\n",
				     count, ARRAY_SIZE(model));

	ret = device_property_read_u16_array(dev, "maxim,fg-model", model,
					    ARRAY_SIZE(model));
	if (ret)
		return ret;

	count = device_property_count_u16(dev, "maxim,fg-params");
	if (count <= 0 || count % 2)
		return dev_err_probe(dev, -EINVAL,
				     "maxim,fg-params must be (register, value) pairs\n");

	params = kmalloc_array(count, sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	ret = device_property_read_u16_array(dev, "maxim,fg-params", params,
					    count);
	if (ret)
		goto free_params;

	nvm = i2c_new_ancillary_device(client, "nvm",
				       MAX77779_FG_NVM_ADDRESS);
	if (IS_ERR(nvm)) {
		ret = PTR_ERR(nvm);
		goto free_params;
	}

	ret = devm_add_action_or_reset(dev, max77779_fg_unregister_nvm, nvm);
	if (ret)
		goto free_params;

	fg->nvm = devm_regmap_init_i2c(nvm, &max77779_fg_regmap_cfg);
	if (IS_ERR(fg->nvm)) {
		ret = PTR_ERR(fg->nvm);
		goto free_params;
	}

	ret = max77779_fg_load_model(dev, fg, model, params, count / 2);

free_params:
	kfree(params);
	return ret;
}

static int max77779_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct max77779_fg *fg;
	struct power_supply *psy;
	unsigned int devname, status, fstat;
	u32 rsense;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->regmap = devm_regmap_init_i2c(client, &max77779_fg_regmap_cfg);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap),
				     "regmap init failed\n");

	ret = regmap_read(fg->regmap, MAX77779_FG_DEVNAME, &devname);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read DevName\n");

	if (!max77779_fg_devname_valid(devname))
		return dev_err_probe(dev, -ENODEV,
				     "unexpected DevName 0x%04x\n", devname);

	/* Rsense is only needed to scale current and charge readings. */
	if (device_property_read_u32(dev, "shunt-resistor-micro-ohms", &rsense))
		rsense = 0;
	fg->rsense = rsense;
	if (!fg->rsense)
		dev_info(dev, "no shunt-resistor-micro-ohms; current/charge reporting disabled\n");

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(dev);

	psy = devm_power_supply_register(dev, &max77779_fg_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "failed to register power supply\n");

	/*
	 * Only load on a POR. A gauge that already has a model has also been
	 * learning this battery's capacity and resistance since it was loaded,
	 * and reloading would throw that away.
	 */
	if (regmap_read(fg->regmap, MAX77779_FG_STATUS, &status) ||
	    regmap_read(fg->regmap, MAX77779_FG_FSTAT, &fstat))
		return 0;

	if (!(status & MAX77779_FG_STATUS_POR) &&
	    !(fstat & MAX77779_FG_FSTAT_DNR))
		return 0;

	/*
	 * A failed load must not take the power supply down with it: voltage,
	 * current and temperature do not depend on the model, and reporting
	 * those beats reporting nothing.
	 */
	ret = max77779_fg_load_from_fwnode(client, fg);
	if (ret == -ENODATA)
		dev_warn(dev,
			 "model not ready (Status=0x%04x FStat=0x%04x) and no maxim,fg-model in DT; state-of-charge unavailable\n",
			 status, fstat);
	else if (ret)
		dev_warn(dev,
			 "cannot load the battery model (%d); state-of-charge unavailable\n",
			 ret);
	else
		dev_info(dev, "battery model loaded after POR\n");

	return 0;
}

static const struct of_device_id max77779_fg_of_match[] = {
	{ .compatible = "maxim,max77779fg" },
	{}
};
MODULE_DEVICE_TABLE(of, max77779_fg_of_match);

static const struct i2c_device_id max77779_fg_id[] = {
	{ "max77779fg" },
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_fg_id);

static struct i2c_driver max77779_fg_i2c_driver = {
	.driver = {
		.name = "max77779-fg",
		.of_match_table = max77779_fg_of_match,
	},
	.id_table = max77779_fg_id,
	.probe = max77779_fg_probe,
};
module_i2c_driver(max77779_fg_i2c_driver);

MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_DESCRIPTION("Maxim MAX77779 fuel gauge driver");
MODULE_LICENSE("GPL");
