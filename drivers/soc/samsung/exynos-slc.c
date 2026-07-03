// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung/Google Exynos System Level Cache (SLC) partition manager.
 *
 * The SLC is a shared last-level cache that is carved into partitions. Bus
 * masters (GPU, codecs, ...) request a partition, enable it, and tag their
 * memory transactions with the partition's PBHA so the cache treats them
 * according to that partition's policy. Partitions are created and enabled
 * through the APM firmware over ACPM; this driver owns the ACPM channel,
 * describes the available partitions from device tree, and hands out
 * per-partition handles to consumer drivers.
 *
 * Copyright 2019 Google LLC.
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/list.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/samsung/exynos-slc.h>

/* ACPM SLC partition commands. */
#define PT_ENABLE			0xd001
#define PT_DISABLE			0xd002
#define PT_MUTATE			0xd004
#define PT_VERSION			0xd005

/* Major version of the ACPM SLC protocol this driver speaks. */
#define PT_VERSION_MAJOR		0x1

#define PT_PTID_INVALID			(-1)

/* Reply encoding: [ptid:10 | data:22]. */
#define PT_REPLY_PTID(r)		((r) >> 22)

struct exynos_slc;

/**
 * struct exynos_slc_partition - a single SLC partition.
 * @slc:	back-pointer to the owning SLC instance.
 * @node:	list membership in &exynos_slc.partitions.
 * @name:	partition label (matched against consumer requests).
 * @vptid:	virtual partition id programmed into the firmware.
 * @size_bits:	partition size selector.
 * @priority:	partition eviction priority (0 highest, 15 lowest).
 * @pbha:	page-based hardware attribute value masters must tag traffic with.
 * @ptid:	firmware-assigned partition id, valid once allocated.
 * @enabled:	whether the partition is currently enabled in the SLC.
 */
struct exynos_slc_partition {
	struct exynos_slc *slc;
	struct list_head node;
	const char *name;
	u32 vptid;
	u32 size_bits;
	u32 priority;
	u32 pbha;
	int ptid;
	bool enabled;
	bool always_on;
};

/**
 * struct exynos_slc - SLC partition manager instance.
 * @dev:	owning device.
 * @acpm:	ACPM protocol handle.
 * @chan_id:	ACPM channel index used for SLC commands.
 * @lock:	serialises firmware commands and partition state.
 * @partitions:	list of &exynos_slc_partition parsed from device tree.
 */
struct exynos_slc {
	struct device *dev;
	struct acpm_handle *acpm;
	unsigned int chan_id;
	struct mutex lock;
	struct list_head partitions;
	struct list_head node;
};

static int exynos_slc_cmd(struct exynos_slc *slc, u32 command, u32 arg, u32 arg1)
{
	return slc->acpm->ops->slc.request(slc->acpm, slc->chan_id, command, arg,
					   arg1, NULL);
}

int exynos_slc_partition_enable(struct exynos_slc_partition *part)
{
	struct exynos_slc *slc = part->slc;
	u32 arg;
	int ret;

	guard(mutex)(&slc->lock);

	if (part->enabled)
		return part->pbha;

	/* Allocate the partition (size 0), obtaining a firmware ptid. */
	arg = (part->priority & 0xff) | ((part->vptid & 0xff) << 8) |
	      ((part->pbha & 0xff) << 16);
	ret = exynos_slc_cmd(slc, PT_ENABLE, arg, 1);
	if (ret < 0)
		return ret;

	part->ptid = PT_REPLY_PTID(ret);

	/* Resize it to its configured size to put it into service. */
	arg = ((part->ptid & 0xff) << 8) | ((part->vptid & 0xff) << 16);
	ret = exynos_slc_cmd(slc, PT_MUTATE, arg, part->size_bits);
	if (ret < 0)
		return ret;

	part->enabled = true;
	dev_dbg(slc->dev, "partition '%s' enabled (ptid %d, pbha %#x)\n",
		part->name, part->ptid, part->pbha);

	return part->pbha;
}
EXPORT_SYMBOL_GPL(exynos_slc_partition_enable);

void exynos_slc_partition_disable(struct exynos_slc_partition *part)
{
	struct exynos_slc *slc = part->slc;
	u32 arg;

	guard(mutex)(&slc->lock);

	if (!part->enabled)
		return;

	arg = ((part->ptid & 0xff) << 8) | ((part->vptid & 0xff) << 16);
	exynos_slc_cmd(slc, PT_DISABLE, arg, 0);
	part->enabled = false;
	part->ptid = PT_PTID_INVALID;
}
EXPORT_SYMBOL_GPL(exynos_slc_partition_disable);

int exynos_slc_partition_pbha(struct exynos_slc_partition *part)
{
	return part->pbha;
}
EXPORT_SYMBOL_GPL(exynos_slc_partition_pbha);

/*
 * Probed SLC instances, keyed on of_node so a consumer holding a "google,slc"
 * phandle can locate its provider.
 */
static LIST_HEAD(exynos_slc_instances);
static DEFINE_MUTEX(exynos_slc_list_lock);

/**
 * devm_exynos_slc_partition_get() - resolve a consumer's SLC partition.
 * @dev:	consumer device. Its node must carry a "google,slc" phandle to
 *		the SLC provider and a "google,slc-partition" partition label.
 * @name:	partition label to look up.
 *
 * Return: partition handle, or ERR_PTR on failure (incl. -EPROBE_DEFER while
 * the SLC provider has not probed yet).
 */
struct exynos_slc_partition *
devm_exynos_slc_partition_get(struct device *dev, const char *name)
{
	struct exynos_slc_partition *part;
	struct exynos_slc *slc = NULL, *iter;
	struct device_node *np __free(device_node) =
		of_parse_phandle(dev->of_node, "google,slc", 0);

	if (!np)
		return ERR_PTR(-EINVAL);

	scoped_guard(mutex, &exynos_slc_list_lock) {
		list_for_each_entry(iter, &exynos_slc_instances, node)
			if (iter->dev->of_node == np) {
				slc = iter;
				break;
			}
	}
	if (!slc)
		return ERR_PTR(-EPROBE_DEFER);

	list_for_each_entry(part, &slc->partitions, node)
		if (!strcmp(part->name, name))
			return part;

	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL_GPL(devm_exynos_slc_partition_get);

static int exynos_slc_parse_partitions(struct exynos_slc *slc)
{
	struct device *dev = slc->dev;

	for_each_child_of_node_scoped(dev->of_node, child) {
		struct exynos_slc_partition *part;

		if (!of_property_present(child, "google,slc-vptid"))
			continue;

		part = devm_kzalloc(dev, sizeof(*part), GFP_KERNEL);
		if (!part)
			return -ENOMEM;

		part->slc = slc;
		part->ptid = PT_PTID_INVALID;
		if (of_property_read_string(child, "label", &part->name))
			part->name = child->name;
		of_property_read_u32(child, "google,slc-vptid", &part->vptid);
		of_property_read_u32(child, "google,slc-size-bits",
				     &part->size_bits);
		of_property_read_u32(child, "google,slc-priority",
				     &part->priority);
		of_property_read_u32(child, "google,slc-pbha", &part->pbha);
		part->always_on = of_property_read_bool(child, "google,slc-default");

		list_add_tail(&part->node, &slc->partitions);
		dev_dbg(dev, "partition '%s': vptid %u pbha %#x\n",
			part->name, part->vptid, part->pbha);
	}

	return 0;
}

static int exynos_slc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_slc_partition *part;
	struct exynos_slc *slc;
	int ret, version;
	u32 fw_chan;

	slc = devm_kzalloc(dev, sizeof(*slc), GFP_KERNEL);
	if (!slc)
		return -ENOMEM;

	slc->dev = dev;
	mutex_init(&slc->lock);
	INIT_LIST_HEAD(&slc->partitions);

	slc->acpm = devm_acpm_get_by_phandle(dev);
	if (IS_ERR(slc->acpm))
		return dev_err_probe(dev, PTR_ERR(slc->acpm),
				     "failed to get ACPM handle\n");

	ret = of_property_read_u32(dev->of_node, "acpm-ipc-channel", &fw_chan);
	if (ret)
		return dev_err_probe(dev, ret, "missing acpm-ipc-channel\n");

	ret = acpm_get_chan_by_id(slc->acpm, fw_chan);
	if (ret < 0)
		return dev_err_probe(dev, ret, "no ACPM channel %u\n", fw_chan);
	slc->chan_id = ret;

	/* Handshake: confirm the firmware speaks a compatible SLC protocol. */
	version = slc->acpm->ops->slc.request(slc->acpm, slc->chan_id,
					      PT_VERSION, 0, 0, NULL);
	if (version < 0)
		return dev_err_probe(dev, version, "SLC version query failed\n");
	if ((version >> 16) != PT_VERSION_MAJOR)
		return dev_err_probe(dev, -ENOTSUPP,
				     "unsupported SLC firmware %d.%d\n",
				     version >> 16, version & 0xffff);
	dev_info(dev, "SLC firmware %d.%d\n", version >> 16, version & 0xffff);

	ret = exynos_slc_parse_partitions(slc);
	if (ret)
		return ret;

	/*
	 * Enable always-on partitions now. A failure here is not fatal: the
	 * partition may be a firmware-managed default (e.g. bypass, PBHA 0)
	 * that cannot be allocated from the AP, in which case untagged traffic
	 * bypasses the cache anyway.
	 */
	list_for_each_entry(part, &slc->partitions, node) {
		if (part->always_on && exynos_slc_partition_enable(part) < 0)
			dev_warn(dev, "could not enable partition '%s'\n",
				 part->name);
	}

	scoped_guard(mutex, &exynos_slc_list_lock)
		list_add_tail(&slc->node, &exynos_slc_instances);

	platform_set_drvdata(pdev, slc);

	return 0;
}

static void exynos_slc_remove(struct platform_device *pdev)
{
	struct exynos_slc *slc = platform_get_drvdata(pdev);

	scoped_guard(mutex, &exynos_slc_list_lock)
		list_del(&slc->node);
}

static const struct of_device_id exynos_slc_of_match[] = {
	{ .compatible = "google,gs101-slc" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_slc_of_match);

static struct platform_driver exynos_slc_driver = {
	.probe	= exynos_slc_probe,
	.remove	= exynos_slc_remove,
	.driver	= {
		.name		= "exynos-slc",
		.of_match_table	= exynos_slc_of_match,
	},
};
module_platform_driver(exynos_slc_driver);

MODULE_DESCRIPTION("Exynos/Google System Level Cache partition manager");
MODULE_LICENSE("GPL");
