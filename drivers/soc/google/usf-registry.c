// SPDX-License-Identifier: GPL-2.0
/*
 * Kernel-side loader for the AoC USF sensor registry.
 *
 * The USF servers come up dormant: the firmware probes no physical sensor
 * until it has been handed a registry describing what is on the board.  That
 * registry is a set of `.reg` scripts -- node/property text carrying ?+CDT
 * stage directives -- which are uploaded one per RegistryLoadScript, each
 * stamped with the device CDT, and followed by a single write of /.loaded = 1
 * that fires the AoC's one-shot startup.
 *
 * The vendor stack does this from a userspace HAL, and the mainline port it
 * was reverse-engineered from uses a userspace daemon, because on Android the
 * scripts are split between a read-only vendor partition and a read-write
 * persist partition holding per-unit calibration.  Here they come through
 * request_firmware() instead: the kernel owns the whole path, so the sensors
 * work from an initramfs with no daemon and no ordering against userspace.
 *
 * The AoC evaluates the ?+/?- directives itself against the CDT we stamp in,
 * so nothing here parses the scripts -- each file goes up verbatim, in the
 * order the device tree lists it, which is the order the HAL uploads them in.
 *
 * The one thing this cannot do is calibration write-back.  Online calibration
 * on the vendor stack writes refined values back to persist, and the kernel
 * has no business writing to a partition; sensors probe and report, and cal
 * refinement does not happen.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#define pr_fmt(fmt) "usf-registry: " fmt

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include <linux/soc/google/usf.h>

/*
 * A script is NUL-terminated on the wire and the terminator is counted.  Most
 * firmware files already end in one; append it when they do not rather than
 * trusting the packaging.
 */
static int usf_upload_script(struct usf_session *s, struct device *dev,
			     const char *name, u32 cdt)
{
	const struct firmware *fw;
	const u8 *script;
	u8 *tmp = NULL;
	size_t len;
	int ret;

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		dev_err(dev, "registry script %s not found: %d\n", name, ret);
		return ret;
	}

	if (fw->size && fw->data[fw->size - 1] == '\0') {
		script = fw->data;
		len = fw->size;
	} else {
		tmp = kmalloc(fw->size + 1, GFP_KERNEL);
		if (!tmp) {
			release_firmware(fw);
			return -ENOMEM;
		}
		memcpy(tmp, fw->data, fw->size);
		tmp[fw->size] = '\0';
		script = tmp;
		len = fw->size + 1;
	}

	ret = usf_session_load_script(s, script, len, cdt);
	if (ret)
		dev_err(dev, "LoadScript(%s, %zu bytes) failed: %d\n", name,
			len, ret);
	else
		dev_dbg(dev, "LoadScript(%s, %zu bytes) ok\n", name, len);

	kfree(tmp);
	release_firmware(fw);

	return ret;
}

/*
 * The CDT (0xPPPPSJIV) identifies the board and stepping.  The bootloader
 * leaves it in /chosen; a device-tree override exists for boards where it does
 * not, and for bring-up.
 */
static u32 usf_registry_cdt(struct device *dev, struct device_node *np)
{
	struct device_node *chosen;
	u32 cdt = 0;

	if (!of_property_read_u32(np, "google,usf-cdt", &cdt))
		return cdt;

	chosen = of_find_node_by_path("/chosen");
	if (chosen) {
		of_property_read_u32(chosen, "plat", &cdt);
		of_node_put(chosen);
	}

	if (!cdt)
		dev_warn(dev,
			 "no CDT in /chosen or DT; scripts gated on ?+CDT will not apply\n");

	return cdt;
}

/*
 * The registry is AoC-global state, not per-session: the IIO bridge and the
 * wake-gesture driver each hold their own session but talk to one AoC, and
 * /.loaded fires a one-shot startup.  Whichever probes first loads it; the
 * other finds it done.  A firmware restart would need this cleared, which
 * there is currently no notification for.
 */
static DEFINE_MUTEX(usf_registry_lock);
static bool usf_registry_done;

static int usf_registry_load_locked(struct usf_session *s,
				    struct device_node *np)
{
	struct device *dev = usf_session_dev(s);
	int count, i, ret;
	u32 cdt;

	count = of_property_count_strings(np, "google,usf-registry");
	if (count <= 0) {
		dev_err(dev, "no google,usf-registry scripts listed\n");
		return count ? count : -EINVAL;
	}

	ret = usf_session_registry_open(s);
	if (ret) {
		dev_err(dev, "Registry server not reachable: %d\n", ret);
		return ret;
	}

	cdt = usf_registry_cdt(dev, np);

	for (i = 0; i < count; i++) {
		const char *name;

		ret = of_property_read_string_index(np, "google,usf-registry",
						    i, &name);
		if (ret)
			return ret;

		ret = usf_upload_script(s, dev, name, cdt);
		if (ret)
			return ret;
	}

	/*
	 * Only now: the AoC starts USF on the falling edge of this write, and
	 * anything not yet uploaded would be missed by that one-shot.
	 */
	ret = usf_session_set_loaded(s);
	if (ret) {
		dev_err(dev, "setting /.loaded failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "sensor registry loaded (%d scripts, CDT 0x%08x)\n",
		 count, cdt);

	return 0;
}

int usf_registry_load(struct usf_session *s, struct device_node *np)
{
	int ret;

	mutex_lock(&usf_registry_lock);
	if (usf_registry_done) {
		mutex_unlock(&usf_registry_lock);
		return 0;
	}

	ret = usf_registry_load_locked(s, np);
	if (!ret)
		usf_registry_done = true;
	mutex_unlock(&usf_registry_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(usf_registry_load);
