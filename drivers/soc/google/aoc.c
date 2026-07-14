// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google AOC (Always-On Compute) coprocessor - firmware load.
 *
 * The AOC is an always-on coprocessor on Tensor SoCs that owns Bluetooth,
 * audio and the sensor hub.  Its DRAM carveout is fenced from the non-secure
 * AP, and its firmware must be authenticated and unlocked by the GSA security
 * core before the AOC can run.
 *
 * This driver stages the AOC image into the carveout and asks the GSA to
 * authenticate it: the signed 4K header goes in coherent memory the GSA reads
 * by DMA, the body goes into the carveout, and gsa_load_aoc_fw_image() hands
 * the GSA both.  Bringing the AOC out of reset and the runtime IPC are separate,
 * later steps.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/soc/samsung/exynos-gsa.h>

#define AOC_FIRMWARE_NAME	"google/aoc.bin"

/* A signed AOC image begins with a fixed-size authentication header (the cert
 * the GSA verifies); the body that the certificate signs follows it.
 */
#define AOC_AUTH_HEADER_SIZE	4096

struct aoc_data {
	struct device *dev;
	struct device *gsa;		/* the GSA that authenticates our image */
	phys_addr_t carveout_base;
	size_t carveout_size;
	void *carveout;			/* mapped for staging the body */
};

/* Stage the image and have the GSA authenticate it. */
static int aoc_load_image(struct aoc_data *aoc, const struct firmware *fw)
{
	struct device *dev = aoc->dev;
	size_t body_size;
	dma_addr_t hdr_da;
	void *hdr;
	int ret;

	if (fw->size <= AOC_AUTH_HEADER_SIZE) {
		dev_err(dev, "firmware too small (%zu bytes)\n", fw->size);
		return -EINVAL;
	}
	body_size = fw->size - AOC_AUTH_HEADER_SIZE;
	if (body_size > aoc->carveout_size) {
		dev_err(dev, "body (%zu) exceeds carveout (%zu)\n",
			body_size, aoc->carveout_size);
		return -EFBIG;
	}

	/* The 4K signed header, in coherent memory the GSA reads by DMA. */
	hdr = dma_alloc_coherent(dev, AOC_AUTH_HEADER_SIZE, &hdr_da, GFP_KERNEL);
	if (!hdr)
		return -ENOMEM;
	memcpy(hdr, fw->data, AOC_AUTH_HEADER_SIZE);

	/* The body, staged into the carveout for the GSA to verify + unlock. */
	memcpy(aoc->carveout, fw->data + AOC_AUTH_HEADER_SIZE, body_size);
	/* flush the write-combined body to DRAM before the GSA reads it */
	wmb();

	ret = gsa_load_aoc_fw_image(aoc->gsa, hdr_da, aoc->carveout_base);
	if (ret)
		dev_err(dev, "GSA rejected the AOC image: %d\n", ret);
	else
		dev_info(dev, "AOC firmware authenticated and loaded by the GSA\n");

	dma_free_coherent(dev, AOC_AUTH_HEADER_SIZE, hdr, hdr_da);
	return ret;
}

static int aoc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *gsa_pdev;
	const struct firmware *fw;
	struct reserved_mem *rmem;
	struct device_node *np;
	struct aoc_data *aoc;
	int ret;

	aoc = devm_kzalloc(dev, sizeof(*aoc), GFP_KERNEL);
	if (!aoc)
		return -ENOMEM;
	aoc->dev = dev;
	platform_set_drvdata(pdev, aoc);

	/* The GSA that will authenticate our image. */
	np = of_parse_phandle(dev->of_node, "gsa-device", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no gsa-device phandle\n");
	gsa_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!gsa_pdev)
		return -EPROBE_DEFER;
	aoc->gsa = &gsa_pdev->dev;
	if (!aoc->gsa->driver) {
		put_device(aoc->gsa);
		return -EPROBE_DEFER;
	}

	/* The AOC DRAM carveout the firmware body is staged into. */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		ret = dev_err_probe(dev, -EINVAL, "no memory-region\n");
		goto err_put_gsa;
	}
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem) {
		ret = dev_err_probe(dev, -EINVAL, "bad memory-region\n");
		goto err_put_gsa;
	}
	aoc->carveout_base = rmem->base;
	aoc->carveout_size = rmem->size;

	/*
	 * Write-combining, not cached: the GSA reads the staged body straight
	 * from DRAM, so our writes must not linger in the CPU cache.
	 */
	aoc->carveout = devm_memremap(dev, aoc->carveout_base,
				      aoc->carveout_size, MEMREMAP_WC);
	if (IS_ERR(aoc->carveout)) {
		ret = PTR_ERR(aoc->carveout);
		goto err_put_gsa;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		goto err_put_gsa;

	dev_info(dev, "carveout %pa (%zu MiB), gsa %s\n", &aoc->carveout_base,
		 aoc->carveout_size >> 20, dev_name(aoc->gsa));

	/* Loaded as a module after the rootfs is up, so a plain synchronous
	 * request is fine and reports the authentication result inline.
	 */
	ret = request_firmware(&fw, AOC_FIRMWARE_NAME, dev);
	if (ret) {
		dev_err(dev, "cannot load %s: %d\n", AOC_FIRMWARE_NAME, ret);
		goto err_put_gsa;
	}
	dev_info(dev, "loaded %s (%zu bytes)\n", AOC_FIRMWARE_NAME, fw->size);

	if (aoc_load_image(aoc, fw) == 0) {
		/*
		 * The core's reset is in the secure domain - the AP cannot poke
		 * it directly - so ask the GSA to take the AOC out of reset.
		 */
		ret = gsa_aoc_start(aoc->gsa);
		if (ret)
			dev_err(dev, "GSA failed to start the AOC: %d\n", ret);
		else
			dev_info(dev, "AOC started by the GSA\n");
	}
	release_firmware(fw);

	return 0;

err_put_gsa:
	put_device(aoc->gsa);
	return ret;
}

static void aoc_remove(struct platform_device *pdev)
{
	struct aoc_data *aoc = platform_get_drvdata(pdev);

	put_device(aoc->gsa);
}

static const struct of_device_id aoc_of_match[] = {
	{ .compatible = "google,aoc" },
	{}
};
MODULE_DEVICE_TABLE(of, aoc_of_match);

static struct platform_driver aoc_driver = {
	.probe = aoc_probe,
	.remove = aoc_remove,
	.driver = {
		.name = "google-aoc",
		.of_match_table = aoc_of_match,
	},
};
module_platform_driver(aoc_driver);

MODULE_DESCRIPTION("Google AOC coprocessor firmware loader");
MODULE_LICENSE("GPL");
