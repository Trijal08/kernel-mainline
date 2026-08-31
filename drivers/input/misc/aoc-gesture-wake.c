// SPDX-License-Identifier: GPL-2.0-only
/*
 * Wake gestures hosted by the AoC USF firmware, as input events.
 *
 * The AoC coprocessor keeps watching for user gestures while the AP is
 * suspended: a single tap on the (powered-down) touch panel, and lift-to-wake
 * from the accelerometer. It reports each as a USF sample, which this driver
 * turns into KEY_WAKEUP so the display comes back the way a power-key press
 * would bring it back.
 *
 * Two things make the gestures actually able to wake the AP:
 *
 * - The stream must be created with USF_MODE_WAKE_GESTURE. The firmware
 *   delivers the streaming modes on com.google.usf.non_wake_up, whose mailbox
 *   doorbell is masked while the AP is suspended, so those samples sit in the
 *   ring until something else wakes it. A wake-gesture stream arrives on
 *   com.google.usf instead, which is not masked.
 *
 * - The session must stay open. A USF sampling session belongs to the AOCC
 *   channel that created it and dies with it, so the channel is held for the
 *   lifetime of the driver rather than opened around each use.
 *
 * Gestures are armed only while the panel is off, both to keep AoC (and the
 * proximity sensor it pulls up to gate them) idle while the screen is on, and
 * because a wake gesture means nothing to a user who is already looking at the
 * display.
 *
 * Copyright 2026 Steffen Deusch
 */

#define pr_fmt(fmt) "aoc-gesture-wake: " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#include <linux/soc/google/usf.h>

#define AGW_RETRY_MS		1000	/* bootstrap retry cadence */
#define AGW_MAX_RETRIES		15	/* ~15 s before giving up */
#define AGW_LIST_MAX		64	/* enumerated sensor handles */
#define AGW_WAKE_HOLD_MS	1000	/* keep the AP up long enough to react */

/*
 * A gesture is an on-change event, not a stream, so the period is a formality;
 * the firmware reports when the gesture happens. This mirrors what the vendor
 * HAL asks for.
 */
#define AGW_PERIOD_NS		2500000

struct agw_gesture {
	const char *name;	/* sysfs attribute name */
	const char *usf_name;	/* USF sensor name substring to match */
	bool default_on;
};

/*
 * Match sensors by name: USF handles are assigned by the firmware and are not
 * stable across devices or firmware versions.
 *
 * Lift-to-wake defaults off. It fires from the accelerometer alone, so a bag or
 * a nudged desk can trigger it, and an unwanted wake costs battery silently.
 * Tap needs a deliberate touch on the panel and defaults on.
 */
static const struct agw_gesture agw_gestures[] = {
	{ "single_tap", "Proximity Gated Single Tap", true },
	{ "lift_to_wake", "Lift to Wake", false },
};

#define AGW_NR_GESTURES ARRAY_SIZE(agw_gestures)

struct agw {
	struct device *dev;
	struct input_dev *input;
	struct usf_session *usf;
	struct device *aoc_dev;

	struct delayed_work bootstrap_work;
	unsigned int retries;
	bool ready;			/* USF bootstrapped, handles resolved */

	struct drm_panel_follower follower;
	bool following;

	/* Arming state and everything it depends on. */
	struct mutex lock;
	bool panel_off;
	bool enabled[AGW_NR_GESTURES];
	u32 handle[AGW_NR_GESTURES];	/* 0 = sensor not present */
	u32 client_id[AGW_NR_GESTURES];
	u32 sampling_id[AGW_NR_GESTURES];	/* 0 = not sampling */
};

/* Case-insensitive substring test. */
static bool agw_name_has(const char *name, const char *sub)
{
	size_t nlen = strlen(name), slen = strlen(sub);
	size_t i;

	if (slen > nlen)
		return false;
	for (i = 0; i + slen <= nlen; i++)
		if (!strncasecmp(name + i, sub, slen))
			return true;
	return false;
}

/*
 * A gesture fired. This runs in the AOCC demux thread, so it must not sleep:
 * report the wakeup and the key, and leave the arming state alone.
 */
static void agw_sample(void *ctx, const u8 *pay, u32 plen)
{
	struct agw *agw = ctx;
	const struct usf_sample_hdr *hdr;
	u32 sid;
	int i;

	if (plen < sizeof(*hdr))
		return;
	hdr = (const struct usf_sample_hdr *)pay;
	sid = hdr->sampling_id;
	if (!sid)
		return;

	for (i = 0; i < AGW_NR_GESTURES; i++) {
		/*
		 * Unlocked read: sampling_id is written under the lock but read
		 * here only to attribute the sample. A stale match at worst
		 * reports a wake for a gesture that was just disarmed, which is
		 * the same race the user creates by disarming mid-gesture.
		 */
		if (READ_ONCE(agw->sampling_id[i]) != sid)
			continue;

		dev_dbg(agw->dev, "%s gesture\n", agw_gestures[i].name);
		pm_wakeup_event(agw->dev, AGW_WAKE_HOLD_MS);
		input_report_key(agw->input, KEY_WAKEUP, 1);
		input_sync(agw->input);
		input_report_key(agw->input, KEY_WAKEUP, 0);
		input_sync(agw->input);
		return;
	}
}

/* Start or stop one gesture's stream. Caller holds @lock. */
static void agw_set_sampling(struct agw *agw, int i, bool on)
{
	u32 sid;
	int ret;

	if (!agw->handle[i])
		return;
	if (on == !!agw->sampling_id[i])
		return;

	if (on) {
		ret = usf_session_start_sampling(agw->usf, agw->handle[i],
						 USF_MODE_WAKE_GESTURE,
						 AGW_PERIOD_NS,
						 agw->client_id[i], &sid);
		if (ret) {
			dev_warn(agw->dev, "%s: arming failed: %d\n",
				 agw_gestures[i].name, ret);
			return;
		}
		WRITE_ONCE(agw->sampling_id[i], sid);
		dev_dbg(agw->dev, "%s armed (sampling_id=%u)\n",
			agw_gestures[i].name, sid);
	} else {
		sid = agw->sampling_id[i];
		WRITE_ONCE(agw->sampling_id[i], 0);
		usf_session_stop_sampling(agw->usf, agw->handle[i], sid);
		dev_dbg(agw->dev, "%s disarmed\n", agw_gestures[i].name);
	}
}

/*
 * Bring the streams in line with the arming state: a gesture samples when it is
 * enabled, the panel is off, and the device is allowed to wake the system.
 * Caller holds @lock.
 */
static void agw_sync(struct agw *agw)
{
	bool arm = agw->ready && agw->panel_off && device_may_wakeup(agw->dev);
	int i;

	for (i = 0; i < AGW_NR_GESTURES; i++)
		agw_set_sampling(agw, i, arm && agw->enabled[i]);
}

/* ---- sysfs ------------------------------------------------------------- */

static int agw_attr_index(struct device_attribute *attr)
{
	int i;

	for (i = 0; i < AGW_NR_GESTURES; i++)
		if (!strcmp(attr->attr.name, agw_gestures[i].name))
			return i;
	return -EINVAL;
}

static ssize_t agw_gesture_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct agw *agw = dev_get_drvdata(dev);
	int i = agw_attr_index(attr);

	if (i < 0)
		return i;
	return sysfs_emit(buf, "%d\n", agw->enabled[i]);
}

static ssize_t agw_gesture_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct agw *agw = dev_get_drvdata(dev);
	int i = agw_attr_index(attr);
	bool val;
	int ret;

	if (i < 0)
		return i;
	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&agw->lock);
	agw->enabled[i] = val;
	/* Take effect now, not at the next blank. */
	agw_sync(agw);
	mutex_unlock(&agw->lock);

	return count;
}

static DEVICE_ATTR(single_tap, 0644, agw_gesture_show, agw_gesture_store);
static DEVICE_ATTR(lift_to_wake, 0644, agw_gesture_show, agw_gesture_store);

static ssize_t gestures_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct agw *agw = dev_get_drvdata(dev);
	int i, len = 0;

	/* Only the gestures this device actually found in the USF registry. */
	for (i = 0; i < AGW_NR_GESTURES; i++)
		if (agw->handle[i])
			len += sysfs_emit_at(buf, len, "%s%s",
					     len ? " " : "",
					     agw_gestures[i].name);
	return len + sysfs_emit_at(buf, len, "\n");
}
static DEVICE_ATTR_RO(gestures);

static ssize_t armed_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct agw *agw = dev_get_drvdata(dev);
	bool armed = false;
	int i;

	mutex_lock(&agw->lock);
	for (i = 0; i < AGW_NR_GESTURES; i++)
		armed |= !!agw->sampling_id[i];
	mutex_unlock(&agw->lock);

	return sysfs_emit(buf, "%d\n", armed);
}
static DEVICE_ATTR_RO(armed);

/*
 * The USF registry is uploaded by a userspace daemon, so "no sensors yet" is a
 * timing accident rather than a verdict -- this re-runs the search.
 */
static ssize_t rescan_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct agw *agw = dev_get_drvdata(dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	if (val && !agw->ready) {
		agw->retries = 0;
		mod_delayed_work(system_wq, &agw->bootstrap_work, 0);
	}
	return count;
}
static DEVICE_ATTR_WO(rescan);

static struct attribute *agw_attrs[] = {
	&dev_attr_gestures.attr,
	&dev_attr_rescan.attr,
	&dev_attr_armed.attr,
	&dev_attr_single_tap.attr,
	&dev_attr_lift_to_wake.attr,
	NULL,
};

static umode_t agw_attr_visible(struct kobject *kobj, struct attribute *attr,
				int n)
{
	struct agw *agw = dev_get_drvdata(kobj_to_dev(kobj));
	int i;

	/* Hide the switch for a gesture this firmware does not provide. */
	for (i = 0; i < AGW_NR_GESTURES; i++)
		if (!strcmp(attr->name, agw_gestures[i].name))
			return agw->handle[i] ? attr->mode : 0;
	return attr->mode;
}

static const struct attribute_group agw_group = {
	.attrs = agw_attrs,
	.is_visible = agw_attr_visible,
};
__ATTRIBUTE_GROUPS(agw);

/* ---- panel following --------------------------------------------------- */

static int agw_panel_prepared(struct drm_panel_follower *follower)
{
	struct agw *agw = container_of(follower, struct agw, follower);

	mutex_lock(&agw->lock);
	agw->panel_off = false;
	agw_sync(agw);
	mutex_unlock(&agw->lock);
	return 0;
}

static int agw_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct agw *agw = container_of(follower, struct agw, follower);

	mutex_lock(&agw->lock);
	agw->panel_off = true;
	agw_sync(agw);
	mutex_unlock(&agw->lock);
	return 0;
}

static const struct drm_panel_follower_funcs agw_follower_funcs = {
	.panel_prepared = agw_panel_prepared,
	.panel_unpreparing = agw_panel_unpreparing,
};

/* ---- bootstrap --------------------------------------------------------- */

/* Find the USF handle for each configured gesture. Returns how many resolved. */
static int agw_resolve_handles(struct agw *agw)
{
	u32 handles[AGW_LIST_MAX];
	char name[USF_NAME_MAX];
	int count, found = 0, i, g;

	count = usf_session_sensor_list(agw->usf, handles, AGW_LIST_MAX);
	if (count <= 0)
		return count;

	for (i = 0; i < count; i++) {
		if (usf_session_sensor_info(agw->usf, handles[i], name,
					    sizeof(name), NULL))
			continue;
		for (g = 0; g < AGW_NR_GESTURES; g++) {
			if (agw->handle[g] ||
			    !agw_name_has(name, agw_gestures[g].usf_name))
				continue;
			agw->handle[g] = handles[i];
			agw->client_id[g] = 0xC0DE0000u | handles[i];
			found++;
			dev_info(agw->dev, "%s: USF '%s' (handle 0x%x)\n",
				 agw_gestures[g].name, name, handles[i]);
		}
	}
	return found;
}

static void agw_retry(struct agw *agw, const char *why)
{
	if (agw->retries++ < AGW_MAX_RETRIES) {
		dev_dbg(agw->dev, "%s; retry %u/%u\n", why, agw->retries,
			AGW_MAX_RETRIES);
		schedule_delayed_work(&agw->bootstrap_work,
				      msecs_to_jiffies(AGW_RETRY_MS));
		return;
	}
	dev_warn(agw->dev,
		 "no gestures after %u retries: %s -- write 1 to rescan once the registry is loaded\n",
		 agw->retries, why);
}

static void agw_bootstrap_work(struct work_struct *work)
{
	struct agw *agw = container_of(to_delayed_work(work), struct agw,
				       bootstrap_work);
	int ret;

	if (agw->ready)
		return;

	/*
	 * Opened once and kept: usf_session_open() is idempotent, and closing
	 * between attempts would tear the AoC-side transport down and stand a
	 * new one up on every retry.
	 */
	ret = usf_session_open(agw->usf);
	if (ret) {
		/* AoC / AOCC not up yet: transient, keep retrying. */
		agw_retry(agw, "AOCC channels not ready");
		return;
	}

	/*
	 * The servers stay dormant until the registry is up, so load it, once,
	 * for whichever of us gets here first.  Not reaching it yet is the
	 * ordinary case early on -- the scripts come out of the firmware path,
	 * which may not be mounted -- so retry rather than give up.  The
	 * session stays open across the retry, per above.
	 */
	ret = usf_registry_load(agw->usf, agw->dev->of_node);
	if (ret) {
		agw_retry(agw, "sensor registry not loaded");
		return;
	}

	ret = usf_session_bootstrap(agw->usf);
	if (ret) {
		/* -EAGAIN: the servers did not come up after the registry. */
		agw_retry(agw, "USF servers not responding after registry load");
		return;
	}

	ret = agw_resolve_handles(agw);
	if (ret <= 0) {
		agw_retry(agw, "no wake gestures in the USF sensor list");
		return;
	}

	/*
	 * The attributes are visible only for gestures that resolved, so they
	 * can only be published once the handles are known.
	 */
	ret = sysfs_update_groups(&agw->dev->kobj, agw_groups);
	if (ret)
		dev_warn(agw->dev, "sysfs update failed: %d\n", ret);

	mutex_lock(&agw->lock);
	agw->ready = true;
	agw_sync(agw);
	mutex_unlock(&agw->lock);
}

/* ---- driver ------------------------------------------------------------ */

/* The AoC carries the USF services; resolve it the way the IIO bridge does. */
static struct device *agw_get_aoc(struct device *dev)
{
	struct platform_device *aoc_pdev;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "google,aoc", 0);
	if (!np) {
		dev_err(dev, "no google,aoc phandle\n");
		return ERR_PTR(-EINVAL);
	}

	aoc_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!aoc_pdev)
		return ERR_PTR(-EPROBE_DEFER);

	if (!platform_get_drvdata(aoc_pdev)) {
		put_device(&aoc_pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return &aoc_pdev->dev;
}

static int agw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct agw *agw;
	int i, ret;

	agw = devm_kzalloc(dev, sizeof(*agw), GFP_KERNEL);
	if (!agw)
		return -ENOMEM;

	agw->dev = dev;
	mutex_init(&agw->lock);
	INIT_DELAYED_WORK(&agw->bootstrap_work, agw_bootstrap_work);
	for (i = 0; i < AGW_NR_GESTURES; i++)
		agw->enabled[i] = agw_gestures[i].default_on;
	platform_set_drvdata(pdev, agw);

	agw->aoc_dev = agw_get_aoc(dev);
	if (IS_ERR(agw->aoc_dev))
		return PTR_ERR(agw->aoc_dev);

	agw->usf = usf_session_alloc(dev, agw->aoc_dev, agw_sample, agw);
	if (IS_ERR(agw->usf))
		return PTR_ERR(agw->usf);

	agw->input = devm_input_allocate_device(dev);
	if (!agw->input)
		return -ENOMEM;
	agw->input->name = "AoC gesture wake";
	agw->input->phys = "aoc-gesture-wake/input0";
	agw->input->id.bustype = BUS_HOST;
	input_set_capability(agw->input, EV_KEY, KEY_WAKEUP);
	ret = input_register_device(agw->input);
	if (ret)
		return ret;

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	/*
	 * Follow the panel: gestures are armed while it is off. Without a panel
	 * to follow there is no screen-off signal, so nothing arms -- the
	 * driver stays loaded and inert rather than sampling with the display
	 * on.
	 */
	agw->follower.funcs = &agw_follower_funcs;
	ret = drm_panel_add_follower(dev, &agw->follower);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_warn(dev, "not following a panel (%d): gestures stay disarmed\n",
			 ret);
	else
		agw->following = true;

	schedule_delayed_work(&agw->bootstrap_work, 0);
	return 0;
}

static void agw_remove(struct platform_device *pdev)
{
	struct agw *agw = platform_get_drvdata(pdev);
	int i;

	cancel_delayed_work_sync(&agw->bootstrap_work);

	if (agw->following)
		drm_panel_remove_follower(&agw->follower);

	mutex_lock(&agw->lock);
	for (i = 0; i < AGW_NR_GESTURES; i++)
		agw_set_sampling(agw, i, false);
	mutex_unlock(&agw->lock);

	usf_session_close(agw->usf);
	put_device(agw->aoc_dev);
}

static const struct of_device_id agw_of_match[] = {
	{ .compatible = "google,aoc-gesture-wake" },
	{ }
};
MODULE_DEVICE_TABLE(of, agw_of_match);

static struct platform_driver agw_driver = {
	.driver = {
		.name = "aoc-gesture-wake",
		.of_match_table = agw_of_match,
		.dev_groups = agw_groups,
	},
	.probe = agw_probe,
	.remove = agw_remove,
};
module_platform_driver(agw_driver);

MODULE_DESCRIPTION("AoC USF wake gestures as input events");
MODULE_LICENSE("GPL");
