// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2019 Samsung Electronics Co., Ltd.
 * Copyright 2025 Google LLC.
 * Copyright 2026 Linaro Ltd.
 *
 * Exynos ACPM TMU driver.
 *
 * The AP maps only the TMU INTPEND registers and takes the TMU interrupt;
 * everything else -- temperature reads, trip thresholds, interrupt enable and
 * acknowledge -- goes to the ACPM firmware over its TMU IPC channel.  Trip
 * crossings are therefore delivered as hardware interrupts rather than found
 * by polling.
 */

#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device/devres.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/units.h>

#include "../thermal_hwmon.h"

#define EXYNOS_TMU_SENSOR(i)		BIT(i)
#define EXYNOS_TMU_SENSORS_MAX_COUNT	16

/*
 * Probe-to-zone mapping, from downstream gs_tmu_v3.c (CONFIG_SOC_ZUMA branch,
 * which covers zuma and zumapro).  TMU_TOP carries the three CPU clusters,
 * TMU_SUB the four accelerator zones; the two instances have independent
 * register blocks, so the overlapping probe numbers do not collide.
 */
#define ZUMAPRO_BIG_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(1) |	\
				   EXYNOS_TMU_SENSOR(2))
#define ZUMAPRO_MID_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(4) |	\
				   EXYNOS_TMU_SENSOR(5) |	\
				   EXYNOS_TMU_SENSOR(6) |	\
				   EXYNOS_TMU_SENSOR(7))
#define ZUMAPRO_LIT_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(8) |	\
				   EXYNOS_TMU_SENSOR(9))
#define ZUMAPRO_G3D_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(10) |	\
				   EXYNOS_TMU_SENSOR(11) |	\
				   EXYNOS_TMU_SENSOR(12))
#define ZUMAPRO_ISP_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(14))
#define ZUMAPRO_TPU_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(2) |	\
				   EXYNOS_TMU_SENSOR(3) |	\
				   EXYNOS_TMU_SENSOR(4))
#define ZUMAPRO_AUR_SENSOR_MASK	  (EXYNOS_TMU_SENSOR(6) |	\
				   EXYNOS_TMU_SENSOR(7) |	\
				   EXYNOS_TMU_SENSOR(8))

/* zumapro: P0_INTPEND at 0x0128, stride 0x40 (gs101 uses 0x00f8 / 0x50) */
#define ZUMAPRO_REG_INTPEND(i)		((i) * 0x40 + 0x128)

enum {
	P0_INTPEND,
	P1_INTPEND,
	P2_INTPEND,
	P3_INTPEND,
	P4_INTPEND,
	P5_INTPEND,
	P6_INTPEND,
	P7_INTPEND,
	P8_INTPEND,
	P9_INTPEND,
	P10_INTPEND,
	P11_INTPEND,
	P12_INTPEND,
	P13_INTPEND,
	P14_INTPEND,
	P15_INTPEND,
	REG_INTPEND_COUNT,
};

struct acpm_tmu_sensor_group {
	u16 mask;
	u8 id;
};

struct acpm_tmu_priv;

struct acpm_tmu_sensor {
	const struct acpm_tmu_sensor_group *group;
	struct thermal_zone_device *tzd;
	struct acpm_tmu_priv *priv;
	struct mutex lock; /* protects sensor state */
	bool enabled;
};

struct acpm_tmu_priv {
	struct regmap_field *regmap_fields[REG_INTPEND_COUNT];
	struct acpm_handle *handle;
	struct device *dev;
	struct clk *clk;
	unsigned int mbox_chan_id;
	unsigned int num_sensors;
	int irq;
	struct acpm_tmu_sensor sensors[] __counted_by(num_sensors);
};

struct acpm_tmu_driver_data {
	const struct reg_field *reg_fields;
	const struct acpm_tmu_sensor_group *sensor_groups;
	unsigned int num_sensor_groups;
	unsigned int mbox_chan_id;
	unsigned int max_register;
};

#define ACPM_TMU_SENSOR_GROUP(_mask, _id)	\
	{					\
		.mask	= _mask,		\
		.id	= _id,			\
	}

/*
 * .id is the global ACPM thermal-zone id (BIG 0, MID 1, LITTLE 2, G3D 3,
 * ISP 4, TPU 5, AUR 6); the DT thermal-sensor cell is the index into this
 * array, i.e. local to the TMU instance.
 */
static const struct acpm_tmu_sensor_group zumapro_top_sensor_groups[] = {
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_BIG_SENSOR_MASK, 0),
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_MID_SENSOR_MASK, 1),
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_LIT_SENSOR_MASK, 2),
};

static const struct acpm_tmu_sensor_group zumapro_sub_sensor_groups[] = {
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_G3D_SENSOR_MASK, 3),
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_ISP_SENSOR_MASK, 4),
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_TPU_SENSOR_MASK, 5),
	ACPM_TMU_SENSOR_GROUP(ZUMAPRO_AUR_SENSOR_MASK, 6),
};

static const struct reg_field zumapro_reg_fields[REG_INTPEND_COUNT] = {
	[P0_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(0), 0, 31),
	[P1_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(1), 0, 31),
	[P2_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(2), 0, 31),
	[P3_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(3), 0, 31),
	[P4_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(4), 0, 31),
	[P5_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(5), 0, 31),
	[P6_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(6), 0, 31),
	[P7_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(7), 0, 31),
	[P8_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(8), 0, 31),
	[P9_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(9), 0, 31),
	[P10_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(10), 0, 31),
	[P11_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(11), 0, 31),
	[P12_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(12), 0, 31),
	[P13_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(13), 0, 31),
	[P14_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(14), 0, 31),
	[P15_INTPEND] = REG_FIELD(ZUMAPRO_REG_INTPEND(15), 0, 31),
};

static const struct acpm_tmu_driver_data acpm_tmu_zumapro_top = {
	.reg_fields = zumapro_reg_fields,
	.sensor_groups = zumapro_top_sensor_groups,
	.num_sensor_groups = ARRAY_SIZE(zumapro_top_sensor_groups),
	.mbox_chan_id = 9,
	.max_register = ZUMAPRO_REG_INTPEND(15),
};

static const struct acpm_tmu_driver_data acpm_tmu_zumapro_sub = {
	.reg_fields = zumapro_reg_fields,
	.sensor_groups = zumapro_sub_sensor_groups,
	.num_sensor_groups = ARRAY_SIZE(zumapro_sub_sensor_groups),
	.mbox_chan_id = 9,
	.max_register = ZUMAPRO_REG_INTPEND(15),
};

static int acpm_tmu_op_tz_control(struct acpm_tmu_sensor *sensor, bool on)
{
	struct acpm_tmu_priv *priv = sensor->priv;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	int ret;

	ret = ops->tz_control(handle, priv->mbox_chan_id, sensor->group->id,
			      on);
	if (ret)
		return ret;

	sensor->enabled = on;

	return 0;
}

static void acpm_tmu_control_rollback(struct acpm_tmu_priv *priv, int start_idx)
{
	int i;

	for (i = start_idx; i >= 0; i--) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];
		int ret;

		if (!sensor->tzd)
			continue;

		mutex_lock(&sensor->lock);
		ret = acpm_tmu_op_tz_control(sensor, false);
		mutex_unlock(&sensor->lock);
		if (ret)
			dev_err(priv->dev, "Rollback: Failed to disable sensor %d: %d\n",
				i, ret);
	}
}

static int acpm_tmu_control(struct acpm_tmu_priv *priv, bool on, bool rollback)
{
	struct device *dev = priv->dev;
	int i, ret, err = 0;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	for (i = 0; i < priv->num_sensors; i++) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];

		/* Skip sensors that weren't found in DT */
		if (!sensor->tzd)
			continue;

		mutex_lock(&sensor->lock);
		ret = acpm_tmu_op_tz_control(sensor, on);
		mutex_unlock(&sensor->lock);
		if (ret) {
			if (!err)
				err = ret;

			/* On enable, stop on first error if rollback is requested */
			if (on && rollback) {
				acpm_tmu_control_rollback(priv, i - 1);
				break;
			}
		}
	}

	pm_runtime_put_sync(dev);

	return err;
}

static int acpm_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct acpm_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct acpm_tmu_priv *priv = sensor->priv;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	struct device *dev = priv->dev;
	int acpm_temp = 0, ret;

	/*
	 * Fast path: check locklessly if the sensor is enabled to avoid
	 * expensive runtime PM operations when it is disabled. Any race with
	 * concurrent disabling is caught by the second check under the lock
	 * after PM resume.
	 */
	if (!sensor->enabled)
		return -EAGAIN;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	scoped_guard(mutex, &sensor->lock) {
		if (!sensor->enabled) {
			ret = -EAGAIN;
		} else {
			ret = ops->read_temp(handle, priv->mbox_chan_id,
					     sensor->group->id, &acpm_temp);
		}
	}

	pm_runtime_put_autosuspend(dev);

	if (ret)
		return ret;

	*temp = acpm_temp * MILLIDEGREE_PER_DEGREE;

	return 0;
}

static int acpm_tmu_update_thresholds(struct acpm_tmu_sensor *sensor,
				      u8 thresholds[2], u8 inten)
{
	struct acpm_tmu_priv *priv = sensor->priv;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	unsigned int mbox_chan_id = priv->mbox_chan_id;
	u8 acpm_sensor_id = sensor->group->id;
	bool was_enabled;
	int ret, restore_ret;

	guard(mutex)(&sensor->lock);

	was_enabled = sensor->enabled;

	if (was_enabled) {
		ret = acpm_tmu_op_tz_control(sensor, false);
		if (ret)
			return ret;
	}

	ret = ops->set_threshold(handle, mbox_chan_id, acpm_sensor_id,
				 thresholds, 2);
	if (!ret) {
		ret = ops->set_interrupt_enable(handle, mbox_chan_id,
						acpm_sensor_id, inten);
	}

	if (was_enabled) {
		restore_ret = acpm_tmu_op_tz_control(sensor, true);
		if (restore_ret)
			dev_err(priv->dev, "Failed to restore sensor state: %d\n",
				restore_ret);
		if (!ret)
			ret = restore_ret;
	}

	return ret;
}

static int acpm_tmu_set_trips(struct thermal_zone_device *tz, int low, int high)
{
	struct acpm_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct acpm_tmu_priv *priv = sensor->priv;
	struct device *dev = priv->dev;
	u8 thresholds[2] = {};
	u8 inten = 0;
	int ret;

	/* If a valid lower bound exists, set the threshold and enable its interrupt */
	if (low > -INT_MAX) {
		thresholds[0] = clamp_val(low / MILLIDEGREE_PER_DEGREE, 0, 255);
		inten |= BIT(0);
	}

	/* If a valid upper bound exists, set the threshold and enable its interrupt */
	if (high < INT_MAX) {
		thresholds[1] = clamp_val(high / MILLIDEGREE_PER_DEGREE, 0, 255);
		inten |= BIT(1);
	}

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	ret = acpm_tmu_update_thresholds(sensor, thresholds, inten);

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct thermal_zone_device_ops acpm_tmu_sensor_ops = {
	.get_temp = acpm_tmu_get_temp,
	.set_trips = acpm_tmu_set_trips,
};

static int acpm_tmu_has_pending_irq(struct acpm_tmu_sensor *sensor,
				    bool *pending_irq)
{
	struct acpm_tmu_priv *priv = sensor->priv;
	unsigned long mask = sensor->group->mask;
	int i, ret;
	u32 val;

	guard(mutex)(&sensor->lock);

	for_each_set_bit(i, &mask, EXYNOS_TMU_SENSORS_MAX_COUNT) {
		ret = regmap_field_read(priv->regmap_fields[i], &val);
		if (ret)
			return ret;

		if (val) {
			*pending_irq = true;
			break;
		}
	}

	return 0;
}

static irqreturn_t acpm_tmu_thread_fn(int irq, void *id)
{
	struct acpm_tmu_priv *priv = id;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	struct device *dev = priv->dev;
	bool handled = false;
	int i, ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err(dev, "Failed to resume: %d\n", ret);
		return IRQ_NONE;
	}

	for (i = 0; i < priv->num_sensors; i++) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];
		bool pending_irq = false;

		if (!sensor->tzd)
			continue;

		ret = acpm_tmu_has_pending_irq(sensor, &pending_irq);
		if (ret || !pending_irq)
			continue;

		handled = true;

		scoped_guard(mutex, &sensor->lock) {
			ret = ops->clear_tz_irq(handle, priv->mbox_chan_id,
						sensor->group->id);
			if (ret)
				dev_err(priv->dev, "Sensor %d: failed to clear IRQ (%d)\n",
					i, ret);
		}

		thermal_zone_device_update(sensor->tzd,
					   THERMAL_EVENT_UNSPECIFIED);
	}

	pm_runtime_put_autosuspend(dev);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static const struct of_device_id acpm_tmu_match[] = {
	{
		.compatible = "google,zumapro-tmu-top",
		.data = &acpm_tmu_zumapro_top,
	}, {
		.compatible = "google,zumapro-tmu-sub",
		.data = &acpm_tmu_zumapro_sub,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, acpm_tmu_match);

static int acpm_tmu_probe(struct platform_device *pdev)
{
	const struct acpm_tmu_driver_data *data;
	struct device *dev = &pdev->dev;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.use_relaxed_mmio = true,
	};
	struct acpm_handle *acpm_handle;
	struct acpm_tmu_priv *priv;
	struct regmap *regmap;
	void __iomem *base;
	int i, ret;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	acpm_handle = devm_acpm_get_by_phandle(dev);
	if (IS_ERR(acpm_handle))
		return dev_err_probe(dev, PTR_ERR(acpm_handle),
				     "Failed to get ACPM handle\n");

	if (!acpm_handle->ops->tmu.tz_control)
		return dev_err_probe(dev, -ENODEV,
				     "ACPM firmware lacks TMU support\n");

	priv = devm_kzalloc(dev,
			    struct_size(priv, sensors, data->num_sensor_groups),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->handle = acpm_handle;
	priv->mbox_chan_id = data->mbox_chan_id;
	priv->num_sensors = data->num_sensor_groups;

	platform_set_drvdata(pdev, priv);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base), "Failed to ioremap resource\n");

	regmap_config.max_register = data->max_register;
	regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to init regmap\n");

	ret = devm_regmap_field_bulk_alloc(dev, regmap, priv->regmap_fields,
					   data->reg_fields, REG_INTPEND_COUNT);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Unable to map syscon registers\n");

	/*
	 * The TMU APB PCLK gates the INTPEND reads.  zumapro has no CMU_MISC
	 * TMU gates modelled yet, and the bootloader leaves the clock running
	 * -- our IPC-only reads have relied on that since bring-up.  Treat the
	 * clock as optional until those gates exist; see
	 * research/thermal-acpm-tmu.md (TRACE NEEDED).
	 */
	priv->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "Failed to get the clock\n");

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return dev_err_probe(dev, priv->irq, "Failed to get irq\n");

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable runtime PM\n");

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to resume device\n");

	ret = acpm_handle->ops->tmu.init(acpm_handle, priv->mbox_chan_id);
	if (ret) {
		ret = dev_err_probe(dev, ret, "Failed to init TMU\n");
		goto err_pm_put;
	}

	for (i = 0; i < priv->num_sensors; i++) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];

		mutex_init(&sensor->lock);
		sensor->group = &data->sensor_groups[i];
		sensor->priv = priv;

		sensor->tzd = devm_thermal_of_zone_register(dev, i, sensor,
							    &acpm_tmu_sensor_ops);
		if (IS_ERR(sensor->tzd)) {
			ret = PTR_ERR(sensor->tzd);
			if (ret == -ENODEV) {
				sensor->tzd = NULL;
				dev_dbg(dev, "Sensor %d not used in DT, skipping\n", i);
				continue;
			}

			ret = dev_err_probe(dev, ret, "Failed to register sensor %d\n", i);
			goto err_rollback;
		}

		mutex_lock(&sensor->lock);
		ret = acpm_tmu_op_tz_control(sensor, true);
		mutex_unlock(&sensor->lock);
		if (ret) {
			ret = dev_err_probe(dev, ret, "Failed to enable sensor %d\n", i);
			goto err_rollback;
		}

		thermal_zone_device_update(sensor->tzd,
					   THERMAL_EVENT_UNSPECIFIED);

		ret = devm_thermal_add_hwmon_sysfs(dev, sensor->tzd);
		if (ret)
			dev_warn(dev, "Failed to add hwmon sysfs!\n");
	}

	ret = devm_request_threaded_irq(dev, priv->irq, NULL,
					acpm_tmu_thread_fn, IRQF_ONESHOT,
					dev_name(dev), priv);
	if (ret) {
		ret = dev_err_probe(dev, ret, "Failed to request irq\n");
		goto err_rollback;
	}

	pm_runtime_put_autosuspend(dev);

	return 0;

err_rollback:
	acpm_tmu_control_rollback(priv, i - 1);
err_pm_put:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_put_sync(dev);
	return ret;
}

static void acpm_tmu_remove(struct platform_device *pdev)
{
	struct acpm_tmu_priv *priv = platform_get_drvdata(pdev);

	/* Stop IRQ first to prevent race with thread_fn */
	disable_irq(priv->irq);

	/*
	 * Disable autosuspend to force the subsequent pm_runtime_put_sync()
	 * inside acpm_tmu_control() to synchronously suspend the device
	 * immediately, preventing clock leaks when the driver is removed.
	 */
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	acpm_tmu_control(priv, false, false);
}

static int acpm_tmu_pm_suspend(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	int ret, restore_ret;

	ret = acpm_tmu_control(priv, false, false);
	if (ret)
		goto err_restore_sensors;

	/* APB clock not required for this specific msg */
	ret = ops->suspend(handle, priv->mbox_chan_id);
	if (ret)
		goto err_restore_sensors;

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		goto err_resume_acpm;

	return 0;

err_resume_acpm:
	restore_ret = ops->resume(handle, priv->mbox_chan_id);
	if (restore_ret)
		dev_err(dev, "Failed to resume ACPM after force suspend failure: %d\n",
			restore_ret);

err_restore_sensors:
	restore_ret = acpm_tmu_control(priv, true, false);
	if (restore_ret)
		dev_err(dev, "Failed to re-enable TMU after suspend failure: %d\n",
			restore_ret);

	return ret;
}

static int acpm_tmu_pm_resume(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	int ret, restore_ret;

	/* APB clock not required for this specific msg */
	ret = ops->resume(handle, priv->mbox_chan_id);
	if (ret)
		return ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		goto err_suspend_acpm;

	ret = acpm_tmu_control(priv, true, true);
	if (ret)
		goto err_suspend_pm;

	return 0;

err_suspend_pm:
	restore_ret = pm_runtime_force_suspend(dev);
	if (restore_ret)
		dev_err(dev, "Failed to force suspend during resume rollback: %d\n",
			restore_ret);
err_suspend_acpm:
	restore_ret = ops->suspend(handle, priv->mbox_chan_id);
	if (restore_ret)
		dev_err(dev, "Failed to suspend ACPM during resume rollback: %d\n",
			restore_ret);
	return ret;
}

static int acpm_tmu_runtime_suspend(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static int acpm_tmu_runtime_resume(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);

	return clk_prepare_enable(priv->clk);
}

static const struct dev_pm_ops acpm_tmu_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(acpm_tmu_pm_suspend, acpm_tmu_pm_resume)
	RUNTIME_PM_OPS(acpm_tmu_runtime_suspend, acpm_tmu_runtime_resume, NULL)
};

static struct platform_driver acpm_tmu_driver = {
	.driver = {
		.name   = "gs-tmu",
		.pm     = pm_ptr(&acpm_tmu_pm_ops),
		.of_match_table = acpm_tmu_match,
	},
	.probe = acpm_tmu_probe,
	.remove = acpm_tmu_remove,
};
module_platform_driver(acpm_tmu_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM TMU Driver");
MODULE_LICENSE("GPL");
