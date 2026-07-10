// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright 2023 Google LLC
// Copyright 2026 Trijal Saha
//
// NVMEM driver for the Maxim MAX77779 scratchpad

#include <linux/align.h>
#include <linux/cleanup.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mfd/max77779.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/types.h>

struct max77779_nvmem {
	struct regmap *regmap;
	/* Serialises a page select against the access it applies to, guards buf */
	struct mutex lock;
	/* Bounce buffer for the register-aligned window covering a transfer */
	u8 buf[MAX77779_SP_PAGE_SIZE];
};

/*
 * PAGE_CTRL is a 16-bit register like everything else in this sub-block, so
 * this also writes the byte above it. That byte is reserved, and the vendor
 * driver selects the page the same way.
 */
static int max77779_nvmem_select_page0(struct max77779_nvmem *nvmem)
{
	return regmap_write(nvmem->regmap, MAX77779_SP_REG_PAGE_CTRL, 0);
}

/*
 * Scratchpad bytes at @offset and @offset + 1 share one 16-bit register, so a
 * transfer has to be widened to the enclosing register-aligned window.
 */
static unsigned int max77779_nvmem_window(unsigned int offset, size_t bytes,
					  unsigned int *reg, size_t *len)
{
	unsigned int start = ALIGN_DOWN(offset, 2);

	*reg = MAX77779_SP_REG_DATA + start / 2;
	*len = ALIGN(offset + bytes, 2) - start;

	return start;
}

static int max77779_nvmem_reg_read(void *priv, unsigned int offset,
				   void *val, size_t bytes)
{
	struct max77779_nvmem *nvmem = priv;
	unsigned int reg, start;
	size_t len;
	int ret;

	if (offset + bytes > MAX77779_SP_PAGE_SIZE)
		return -EINVAL;

	start = max77779_nvmem_window(offset, bytes, &reg, &len);

	guard(mutex)(&nvmem->lock);

	ret = max77779_nvmem_select_page0(nvmem);
	if (ret)
		return ret;

	ret = regmap_raw_read(nvmem->regmap, reg, nvmem->buf, len);
	if (ret)
		return ret;

	memcpy(val, nvmem->buf + (offset - start), bytes);

	return 0;
}

static int max77779_nvmem_reg_write(void *priv, unsigned int offset,
				    void *val, size_t bytes)
{
	struct max77779_nvmem *nvmem = priv;
	unsigned int reg, start;
	size_t len;
	int ret;

	if (offset + bytes > MAX77779_SP_PAGE_SIZE)
		return -EINVAL;

	start = max77779_nvmem_window(offset, bytes, &reg, &len);

	guard(mutex)(&nvmem->lock);

	ret = max77779_nvmem_select_page0(nvmem);
	if (ret)
		return ret;

	/*
	 * Read back first when either end of the transfer falls inside a
	 * register, so that the neighbouring byte survives the write.
	 */
	if (len != bytes) {
		ret = regmap_raw_read(nvmem->regmap, reg, nvmem->buf, len);
		if (ret)
			return ret;
	}

	memcpy(nvmem->buf + (offset - start), val, bytes);

	return regmap_raw_write(nvmem->regmap, reg, nvmem->buf, len);
}

static int max77779_nvmem_probe(struct platform_device *pdev)
{
	struct nvmem_config config = {
		.dev = &pdev->dev,
		.name = dev_name(&pdev->dev),
		.id = NVMEM_DEVID_NONE,
		.type = NVMEM_TYPE_BATTERY_BACKED,
		.ignore_wp = true,
		.size = MAX77779_SP_PAGE_SIZE,
		.word_size = sizeof(u8),
		.stride = sizeof(u8),
		.reg_read = max77779_nvmem_reg_read,
		.reg_write = max77779_nvmem_reg_write,
	};
	struct max77779_nvmem *nvmem;
	unsigned int page;
	int ret;

	nvmem = devm_kzalloc(&pdev->dev, sizeof(*nvmem), GFP_KERNEL);
	if (!nvmem)
		return -ENOMEM;

	nvmem->regmap = dev_get_regmap(pdev->dev.parent, "scratch");
	if (!nvmem->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "failed to get scratchpad regmap\n");

	ret = devm_mutex_init(&pdev->dev, &nvmem->lock);
	if (ret)
		return ret;

	/* The scratchpad answers on its own I2C address; make sure it is there. */
	ret = regmap_read(nvmem->regmap, MAX77779_SP_REG_PAGE_CTRL, &page);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "scratchpad is not responding\n");

	config.priv = nvmem;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(config.dev, &config));
}

static const struct of_device_id max77779_nvmem_of_id[] = {
	{ .compatible = "maxim,max77779-nvmem", },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_nvmem_of_id);

static const struct platform_device_id max77779_nvmem_platform_id[] = {
	{ "max77779-nvmem", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77779_nvmem_platform_id);

static struct platform_driver max77779_nvmem_driver = {
	.driver = {
		.name = "max77779-nvmem",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = max77779_nvmem_of_id,
	},
	.probe = max77779_nvmem_probe,
	.id_table = max77779_nvmem_platform_id,
};

module_platform_driver(max77779_nvmem_driver);

MODULE_AUTHOR("Trijal Saha <andreid021101@gmail.com>");
MODULE_DESCRIPTION("NVMEM driver for the Maxim MAX77779 scratchpad");
MODULE_LICENSE("GPL");
