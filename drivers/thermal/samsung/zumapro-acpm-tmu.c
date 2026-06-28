// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2025 Google LLC.
 *
 * Thermal sensor driver for the Google Tensor (zumapro) Thermal Management
 * Unit.  The TMU is owned by the ACPM firmware, so this driver is a read-only
 * thermal-zone sensor provider that asks the firmware for each zone's
 * temperature over ACPM IPC; it never touches TMU registers and programs no
 * trip points, leaving the firmware's own thermal protection untouched.
 */
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/types.h>

#include "../thermal_hwmon.h"

struct zumapro_acpm_tmu;

struct zumapro_acpm_tmu_zone {
	struct zumapro_acpm_tmu *tmu;
	u8 tzid;
};

struct zumapro_acpm_tmu {
	struct acpm_handle *acpm;
	unsigned int acpm_chan_id;
	struct zumapro_acpm_tmu_zone zones[];
};

struct zumapro_acpm_tmu_data {
	unsigned int acpm_chan_id;
	unsigned int num_zones;
};

static int zumapro_acpm_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct zumapro_acpm_tmu_zone *zone = thermal_zone_device_priv(tz);
	struct zumapro_acpm_tmu *tmu = zone->tmu;
	int t, ret;

	ret = tmu->acpm->ops->tmu.read_temp(tmu->acpm, tmu->acpm_chan_id,
					       zone->tzid, &t);
	if (ret)
		return ret;

	*temp = t * 1000;
	return 0;
}

static const struct thermal_zone_device_ops zumapro_acpm_tmu_ops = {
	.get_temp = zumapro_acpm_tmu_get_temp,
};

static int zumapro_acpm_tmu_probe(struct platform_device *pdev)
{
	const struct zumapro_acpm_tmu_data *data;
	struct device *dev = &pdev->dev;
	struct zumapro_acpm_tmu *tmu;
	unsigned int i, registered = 0;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	tmu = devm_kzalloc(dev, struct_size(tmu, zones, data->num_zones),
			   GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->acpm = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(tmu->acpm))
		return dev_err_probe(dev, PTR_ERR(tmu->acpm),
				     "failed to get ACPM handle\n");

	if (!tmu->acpm->ops->tmu.read_temp)
		return dev_err_probe(dev, -ENODEV,
				     "ACPM firmware lacks TMU support\n");

	tmu->acpm_chan_id = data->acpm_chan_id;

	for (i = 0; i < data->num_zones; i++) {
		struct thermal_zone_device *tzd;
		int ret;

		tmu->zones[i].tmu = tmu;
		tmu->zones[i].tzid = i;

		tzd = devm_thermal_of_zone_register(dev, i, &tmu->zones[i],
						    &zumapro_acpm_tmu_ops);
		if (IS_ERR(tzd)) {
			/* Sensors without a thermal-zone in DT are skipped. */
			if (PTR_ERR(tzd) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tzd),
					     "failed to register sensor %u\n", i);
		}

		/*
		 * OF-registered zones set no_hwmon, so the thermal core does
		 * not create hwmon nodes for them; add them explicitly.  hwmon
		 * is supplementary, so a failure here is not fatal.
		 */
		ret = devm_thermal_add_hwmon_sysfs(dev, tzd);
		if (ret)
			dev_warn(dev, "sensor %u: failed to add hwmon: %d\n",
				 i, ret);

		registered++;
	}

	if (!registered)
		return dev_err_probe(dev, -ENODEV,
				     "no thermal zones referenced this sensor\n");

	return 0;
}

static const struct zumapro_acpm_tmu_data zumapro_acpm_tmu_match_data = {
	.acpm_chan_id = 9,
	.num_zones = 7,
};

static const struct of_device_id zumapro_acpm_tmu_match[] = {
	{
		.compatible = "google,zumapro-acpm-tmu",
		.data = &zumapro_acpm_tmu_match_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, zumapro_acpm_tmu_match);

static struct platform_driver zumapro_acpm_tmu_driver = {
	.probe = zumapro_acpm_tmu_probe,
	.driver = {
		.name = "zumapro-acpm-tmu",
		.of_match_table = zumapro_acpm_tmu_match,
	},
};
module_platform_driver(zumapro_acpm_tmu_driver);

MODULE_DESCRIPTION("Google Tensor zumapro ACPM TMU thermal sensor driver");
MODULE_LICENSE("GPL");
