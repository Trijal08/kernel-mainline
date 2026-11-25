// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm ultrasonic fingerprint IPC handler.
 *
 * This is the application-processor side of a Qualcomm ultrasonic
 * under-display fingerprint sensor (e.g. QFS4008). The sensor itself hangs
 * off a SPI bus owned by TrustZone and is therefore invisible to Linux: the
 * secure firmware drives the sensor and performs matching. All this driver
 * does is relay an "IPC doorbell" interrupt (the secure firmware signalling
 * that data is ready) up to userspace, and - on boards that wire it - a
 * finger-detect GPIO. Finger presence on under-display designs is otherwise
 * derived from the touchscreen by userspace.
 *
 * Two character devices are exposed:
 *   /dev/qbt_ipc - read() returns one enum qbt_fw_event per call.
 *   /dev/qbt_fd  - read() returns a struct qbt_fd_userspace_buf.
 *
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <uapi/linux/qbt_handler.h>

#define QBT_DEV			"qbt"
#define QBT_NUM_DEVS		2
#define QBT_MINOR_FD		0
#define QBT_MINOR_IPC		1

#define QBT_IOCTL_WAKELOCK_TIMEOUT_MS	5000
#define QBT_IPC_WAKELOCK_TIMEOUT_MS	100

#define QBT_TOUCH_FD_VERSION_2	2
#define QBT_TOUCH_FD_VERSION_3	3

/* Maximum multitouch slots snooped from the touchscreen. */
#define QBT_MT_MAX_SLOTS		10
/* Hold the system awake while a touch is processed into a finger-detect event. */
#define QBT_TOUCH_WAKELOCK_TIMEOUT_MS	100

/*
 * struct qbt_touch_slot - latest coordinates of one multitouch slot, as
 * accumulated from the touchscreen's EV_ABS stream.
 * @id: tracking id, or negative once the finger in this slot lifts.
 */
struct qbt_touch_slot {
	int		x;
	int		y;
	int		id;
	bool		updated;
};

/*
 * struct qbt_touch_fd - finger-detect-by-touch state.
 *
 * Under-display ultrasonic sensors have no hardware finger-detect line; finger
 * presence is derived by snooping the touchscreen and reporting touches that
 * fall inside the sensor's area-of-interest (AoI) to the secure HAL through
 * /dev/qbt_fd. @up_config is an enlarged AoI used for finger-up hysteresis so a
 * reported-down finger is not dropped by small movements.
 *
 * @lock guards @current_events against the work; the remaining per-slot arrays
 * are touched only by the (singly-scheduled) work.
 */
struct qbt_touch_fd {
	struct qbt_touch_config_v3	config;
	struct qbt_touch_config_v3	up_config;
	struct work_struct		work;
	spinlock_t			lock;
	struct qbt_touch_slot		current_events[QBT_MT_MAX_SLOTS];
	struct qbt_touch_slot		last_events[QBT_MT_MAX_SLOTS];
	int				delta_x[QBT_MT_MAX_SLOTS];
	int				delta_y[QBT_MT_MAX_SLOTS];
	bool				finger_in[QBT_MT_MAX_SLOTS];
	int				slot_state[QBT_MT_MAX_SLOTS];
	int				current_slot;
};

struct qbt_drvdata {
	struct device		*dev;
	struct class		*class;
	dev_t			devt;
	struct cdev		fd_cdev;
	struct cdev		ipc_cdev;
	struct input_dev	*in_dev;

	int			ipc_irq;
	bool			ipc_irq_enabled;

	struct gpio_desc	*fd_gpio;
	int			fd_irq;
	bool			fd_irq_enabled;

	struct gpio_desc	*intr2_gpio;
	bool			is_wuhb_connected;

	/* serialises ioctls */
	struct mutex		mutex;

	/* guards the ipc_events kfifo */
	struct mutex		ipc_events_mutex;
	/* guards the fd_events kfifo */
	struct mutex		fd_events_mutex;
	DECLARE_KFIFO(ipc_events, enum qbt_fw_event, QBT_MAX_FD_EVENTS);
	DECLARE_KFIFO(fd_events, struct qbt_fd_event, QBT_MAX_FD_EVENTS);
	wait_queue_head_t	ipc_wait;
	wait_queue_head_t	fd_wait;

	/* one opener per node */
	atomic_t		ipc_available;
	atomic_t		fd_available;

	struct wakeup_source	*ws;

	/* finger-detect-by-touch: the snooped touchscreen and derived state */
	struct input_dev	*touch_dev;
	bool			fd_enabled;
	struct qbt_touch_fd	touch;

	/* bounce buffer for /dev/qbt_fd reads (too large for the stack) */
	struct qbt_fd_userspace_buf *fd_scratch;
};

static void qbt_queue_ipc_event(struct qbt_drvdata *drvdata,
				enum qbt_fw_event ev)
{
	mutex_lock(&drvdata->ipc_events_mutex);
	if (!kfifo_put(&drvdata->ipc_events, ev))
		dev_warn(drvdata->dev, "IPC event fifo full, dropping %d\n", ev);
	mutex_unlock(&drvdata->ipc_events_mutex);
	wake_up_interruptible(&drvdata->ipc_wait);
}

static void qbt_queue_fd_event(struct qbt_drvdata *drvdata,
			       const struct qbt_fd_event *evt)
{
	mutex_lock(&drvdata->fd_events_mutex);
	if (!kfifo_put(&drvdata->fd_events, *evt))
		dev_warn(drvdata->dev, "FD event fifo full, dropping event\n");
	mutex_unlock(&drvdata->fd_events_mutex);
	wake_up_interruptible(&drvdata->fd_wait);
}

/*
 * Finger-detect by touchscreen snooping.
 *
 * The driver attaches as an input handler to the touchscreen, accumulates the
 * multitouch slot state from its EV_ABS stream, and - from a workqueue - turns
 * touches that fall inside the sensor's area-of-interest into finger
 * down/up/move events delivered to the secure HAL via /dev/qbt_fd. This is how
 * finger presence is detected on under-display designs that have no hardware
 * finger-detect line.
 */

static bool qbt_touch_in_aoi(const struct qbt_touch_slot *slot,
			     const struct qbt_touch_config_v3 *cfg)
{
	return slot->x >= cfg->left && slot->x <= cfg->right &&
	       slot->y >= cfg->top  && slot->y <= cfg->bottom;
}

/*
 * Accumulate movement since the last report and only let a move through once it
 * exceeds the configured radius, to throttle the move-event rate.
 */
static bool qbt_touch_radius_ok(struct qbt_touch_fd *fd, int slot,
				int cur_x, int cur_y, int last_x, int last_y)
{
	fd->delta_x[slot] += cur_x - last_x;
	fd->delta_y[slot] += cur_y - last_y;

	if (!fd->config.rad_filter_enable ||
	    abs(fd->delta_x[slot]) > fd->config.rad_x ||
	    abs(fd->delta_y[slot]) > fd->config.rad_y) {
		fd->delta_x[slot] = 0;
		fd->delta_y[slot] = 0;
		return true;
	}

	return false;
}

static void qbt_touch_work_func(struct work_struct *work)
{
	struct qbt_touch_fd *fd = container_of(work, struct qbt_touch_fd, work);
	struct qbt_drvdata *drvdata = container_of(fd, struct qbt_drvdata, touch);
	struct qbt_touch_slot snapshot[QBT_MT_MAX_SLOTS];
	unsigned long flags;
	int slot, i;

	/* Snapshot the accumulated slot state, then clear the updated flags. */
	spin_lock_irqsave(&fd->lock, flags);
	memcpy(snapshot, fd->current_events, sizeof(snapshot));
	for (slot = 0; slot < QBT_MT_MAX_SLOTS; slot++)
		fd->current_events[slot].updated = false;
	spin_unlock_irqrestore(&fd->lock, flags);

	for (slot = 0; slot < QBT_MT_MAX_SLOTS; slot++) {
		struct qbt_touch_slot *cur = &snapshot[slot];
		struct qbt_touch_slot old_last = fd->last_events[slot];
		bool *finger_in = &fd->finger_in[slot];
		struct qbt_fd_event evt = {};
		struct timespec64 ts;
		int state;

		if (!cur->updated)
			continue;

		/* Derive the finger state from the tracking-id transition. */
		if (cur->id < 0)
			state = QBT_EVENT_FINGER_UP;
		else if (old_last.id < 0)
			state = QBT_EVENT_FINGER_DOWN;
		else if (old_last.id == cur->id)
			state = QBT_EVENT_FINGER_MOVE;
		else
			state = QBT_EVENT_FINGER_DOWN;

		fd->last_events[slot] = *cur;

		/*
		 * Gate on the AoI: a finger must first appear inside the small
		 * AoI to count as down, and is only reported up once it leaves
		 * the enlarged AoI (hysteresis).
		 */
		if (!*finger_in) {
			if (cur->id >= 0 && qbt_touch_in_aoi(cur, &fd->config)) {
				state = QBT_EVENT_FINGER_DOWN;
				*finger_in = true;
			} else {
				continue;
			}
		} else if (cur->id < 0) {
			*finger_in = false;
		} else if (!qbt_touch_in_aoi(cur, &fd->up_config)) {
			state = QBT_EVENT_FINGER_UP;
			*finger_in = false;
		}

		if (state == QBT_EVENT_FINGER_MOVE &&
		    !qbt_touch_radius_ok(fd, slot, cur->x, cur->y,
					 old_last.x, old_last.y))
			continue;

		/* Drive the optional INTR2 line from the aggregate down state. */
		if (state != QBT_EVENT_FINGER_MOVE) {
			bool any_down = false;

			fd->slot_state[slot] = state;
			for (i = 0; i < QBT_MT_MAX_SLOTS; i++)
				if (fd->slot_state[i] == QBT_EVENT_FINGER_DOWN) {
					any_down = true;
					break;
				}
			if (fd->config.intr2_enable && drvdata->intr2_gpio)
				gpiod_set_value(drvdata->intr2_gpio, any_down);
		}

		if (!fd->config.touch_fd_enable)
			continue;

		ktime_get_ts64(&ts);
		evt.timestamp.tv_sec = ts.tv_sec;
		evt.timestamp.tv_nsec = ts.tv_nsec;
		evt.x = cur->x;
		evt.y = cur->y;
		evt.id = slot;
		evt.state = state;
		evt.touch_valid = true;
		qbt_queue_fd_event(drvdata, &evt);
	}
}

/*
 * Input-handler event callback. Runs in the input core's event path (atomic,
 * the device's event_lock held), so it only accumulates slot state and defers
 * the work to a workqueue on each SYN_REPORT.
 */
static void qbt_touch_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	struct qbt_drvdata *drvdata = handle->private;
	struct qbt_touch_fd *fd = &drvdata->touch;
	struct qbt_touch_slot *slot;
	unsigned long flags;
	bool report;
	int tw, dw;

	if (type != EV_SYN && type != EV_ABS)
		return;

	spin_lock_irqsave(&fd->lock, flags);

	if (code == SYN_REPORT) {
		report = fd->config.touch_fd_enable || fd->config.intr2_enable;
		spin_unlock_irqrestore(&fd->lock, flags);
		if (report && drvdata->fd_enabled) {
			__pm_wakeup_event(drvdata->ws,
					  QBT_TOUCH_WAKELOCK_TIMEOUT_MS);
			schedule_work(&fd->work);
		}
		return;
	}

	if (code == ABS_MT_SLOT) {
		fd->current_slot = value;
		goto out;
	}

	if (fd->current_slot < 0 || fd->current_slot >= QBT_MT_MAX_SLOTS)
		goto out;
	slot = &fd->current_events[fd->current_slot];

	switch (code) {
	case ABS_MT_TRACKING_ID:
		slot->id = value;
		slot->updated = true;
		break;
	case ABS_MT_POSITION_X:
	case ABS_MT_POSITION_Y:
		/*
		 * Map raw touch units onto the display coordinate space the HAL
		 * programmed the AoI in (both axes scaled by the X range, as the
		 * reference driver does).
		 */
		tw = input_abs_get_max(handle->dev, ABS_MT_POSITION_X) + 1;
		dw = fd->config.left + fd->config.right;
		if (tw > 0 && dw > 0)
			value = value * dw / tw;
		if (code == ABS_MT_POSITION_X)
			slot->x = abs(value);
		else
			slot->y = abs(value);
		slot->updated = true;
		break;
	default:
		break;
	}

out:
	spin_unlock_irqrestore(&fd->lock, flags);
}

static int qbt_touch_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct qbt_drvdata *drvdata = handler->private;
	struct input_handle *handle;
	int rc;

	/* Only snoop a real (direct) multitouch touchscreen, and only one. */
	if (!test_bit(INPUT_PROP_DIRECT, dev->propbit) ||
	    !test_bit(ABS_MT_POSITION_X, dev->absbit))
		return -ENODEV;
	if (drvdata->touch_dev)
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "qbt_touch";
	handle->private = drvdata;

	rc = input_register_handle(handle);
	if (rc)
		goto err_free;

	rc = input_open_device(handle);
	if (rc)
		goto err_unregister;

	drvdata->touch_dev = dev;
	dev_dbg(drvdata->dev, "snooping touchscreen %s\n", dev_name(&dev->dev));
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return rc;
}

static void qbt_touch_disconnect(struct input_handle *handle)
{
	struct qbt_drvdata *drvdata = handle->private;

	if (drvdata->touch_dev == handle->dev)
		drvdata->touch_dev = NULL;
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id qbt_touch_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
	},
	{ }
};
MODULE_DEVICE_TABLE(input, qbt_touch_ids);

static struct input_handler qbt_touch_handler = {
	.event		= qbt_touch_event,
	.connect	= qbt_touch_connect,
	.disconnect	= qbt_touch_disconnect,
	.name		= "qbt_touch",
	.id_table	= qbt_touch_ids,
};

static void qbt_touch_handler_unregister(void *data)
{
	struct qbt_drvdata *drvdata = data;

	input_unregister_handler(&qbt_touch_handler);
	cancel_work_sync(&drvdata->touch.work);
}

static irqreturn_t qbt_ipc_irq_handler(int irq, void *dev_id)
{
	struct qbt_drvdata *drvdata = dev_id;

	/* keep the system awake long enough for userspace to drain the event */
	__pm_wakeup_event(drvdata->ws, QBT_IPC_WAKELOCK_TIMEOUT_MS);
	qbt_queue_ipc_event(drvdata, FW_EVENT_IPC);

	return IRQ_HANDLED;
}

static irqreturn_t qbt_fd_irq_handler(int irq, void *dev_id)
{
	struct qbt_drvdata *drvdata = dev_id;
	enum qbt_fw_event ev;

	__pm_wakeup_event(drvdata->ws, QBT_IPC_WAKELOCK_TIMEOUT_MS);
	ev = gpiod_get_value(drvdata->fd_gpio) ? FW_EVENT_FINGER_DOWN
					       : FW_EVENT_FINGER_UP;
	qbt_queue_ipc_event(drvdata, ev);

	return IRQ_HANDLED;
}

static int qbt_open(struct inode *inode, struct file *file)
{
	struct qbt_drvdata *drvdata;
	int minor = iminor(inode);

	if (minor == QBT_MINOR_FD)
		drvdata = container_of(inode->i_cdev, struct qbt_drvdata,
				       fd_cdev);
	else if (minor == QBT_MINOR_IPC)
		drvdata = container_of(inode->i_cdev, struct qbt_drvdata,
				       ipc_cdev);
	else
		return -EINVAL;

	file->private_data = drvdata;

	/* disallow concurrent opens of the same node */
	if (minor == QBT_MINOR_FD) {
		if (!atomic_dec_and_test(&drvdata->fd_available)) {
			atomic_inc(&drvdata->fd_available);
			return -EBUSY;
		}
	} else {
		if (!atomic_dec_and_test(&drvdata->ipc_available)) {
			atomic_inc(&drvdata->ipc_available);
			return -EBUSY;
		}
	}

	return 0;
}

static int qbt_release(struct inode *inode, struct file *file)
{
	struct qbt_drvdata *drvdata = file->private_data;

	if (iminor(inode) == QBT_MINOR_FD)
		atomic_inc(&drvdata->fd_available);
	else
		atomic_inc(&drvdata->ipc_available);

	return 0;
}

static int qbt_fifo_len(struct qbt_drvdata *drvdata, int minor)
{
	int len;

	if (minor == QBT_MINOR_FD) {
		mutex_lock(&drvdata->fd_events_mutex);
		len = kfifo_len(&drvdata->fd_events);
		mutex_unlock(&drvdata->fd_events_mutex);
	} else {
		mutex_lock(&drvdata->ipc_events_mutex);
		len = kfifo_len(&drvdata->ipc_events);
		mutex_unlock(&drvdata->ipc_events_mutex);
	}

	return len;
}

static ssize_t qbt_read(struct file *file, char __user *ubuf, size_t cnt,
			loff_t *ppos)
{
	struct qbt_drvdata *drvdata = file->private_data;
	int minor = iminor(file_inode(file));
	wait_queue_head_t *wq;
	enum qbt_fw_event ev;

	if (minor == QBT_MINOR_FD) {
		if (cnt < sizeof(*drvdata->fd_scratch))
			return -EINVAL;
		wq = &drvdata->fd_wait;
	} else {
		if (cnt < sizeof(ev))
			return -EINVAL;
		wq = &drvdata->ipc_wait;
	}

	while (qbt_fifo_len(drvdata, minor) == 0) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(*wq,
					     qbt_fifo_len(drvdata, minor) > 0))
			return -ERESTARTSYS;
	}

	if (minor == QBT_MINOR_FD) {
		struct qbt_fd_userspace_buf *buf = drvdata->fd_scratch;
		unsigned int i;

		memset(buf, 0, sizeof(*buf));
		mutex_lock(&drvdata->fd_events_mutex);
		buf->num_events = kfifo_len(&drvdata->fd_events);
		for (i = 0; i < buf->num_events; i++) {
			if (!kfifo_get(&drvdata->fd_events, &buf->fd_events[i])) {
				buf->num_events = i;
				break;
			}
		}
		mutex_unlock(&drvdata->fd_events_mutex);

		if (copy_to_user(ubuf, buf, sizeof(*buf)))
			return -EFAULT;
		return sizeof(*buf);
	}

	mutex_lock(&drvdata->ipc_events_mutex);
	if (!kfifo_get(&drvdata->ipc_events, &ev)) {
		mutex_unlock(&drvdata->ipc_events_mutex);
		return 0;
	}
	mutex_unlock(&drvdata->ipc_events_mutex);

	if (copy_to_user(ubuf, &ev, sizeof(ev)))
		return -EFAULT;
	return sizeof(ev);
}

static __poll_t qbt_poll(struct file *file, struct poll_table_struct *wait)
{
	struct qbt_drvdata *drvdata = file->private_data;
	int minor = iminor(file_inode(file));

	if (minor == QBT_MINOR_FD) {
		poll_wait(file, &drvdata->fd_wait, wait);
		if (qbt_fifo_len(drvdata, minor) > 0)
			return EPOLLIN | EPOLLRDNORM;
	} else {
		poll_wait(file, &drvdata->ipc_wait, wait);
		if (qbt_fifo_len(drvdata, minor) > 0)
			return EPOLLIN | EPOLLRDNORM;
	}

	return 0;
}

static int qbt_ioctl_configure_touch_fd(struct qbt_drvdata *drvdata,
					unsigned int cmd, void __user *arg)
{
	struct qbt_touch_config_v3 cfg = {}, up;
	unsigned long flags;
	size_t copy_len;
	int w, h;

	if (copy_from_user(&cfg.version, arg, sizeof(cfg.version)))
		return -EFAULT;

	if (cfg.version.version != QBT_TOUCH_FD_VERSION_2 &&
	    cfg.version.version != QBT_TOUCH_FD_VERSION_3)
		return -EINVAL;

	/* v2 has no intr2_enable field */
	if (cfg.version.version == QBT_TOUCH_FD_VERSION_2)
		copy_len = offsetof(struct qbt_touch_config_v3, intr2_enable);
	else
		copy_len = sizeof(cfg);

	if (copy_from_user(&cfg, arg, copy_len))
		return -EFAULT;

	if (cfg.version.version == QBT_TOUCH_FD_VERSION_2)
		cfg.intr2_enable = cfg.touch_fd_enable;

	/*
	 * Grow each side of the AoI by half its width/height to form the
	 * finger-up hysteresis region (see struct qbt_touch_fd).
	 */
	up = cfg;
	w = cfg.right - cfg.left;
	h = cfg.bottom - cfg.top;
	up.left   -= w / 2;
	up.right  += w / 2;
	up.top    -= h / 2;
	up.bottom += h / 2;

	spin_lock_irqsave(&drvdata->touch.lock, flags);
	drvdata->touch.config = cfg;
	drvdata->touch.up_config = up;
	spin_unlock_irqrestore(&drvdata->touch.lock, flags);

	return 0;
}

static long qbt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct qbt_drvdata *drvdata = file->private_data;
	void __user *uarg = (void __user *)arg;
	int rc = 0;

	mutex_lock(&drvdata->mutex);

	switch (cmd) {
	case QBT_ENABLE_IPC:
		if (!drvdata->ipc_irq_enabled) {
			enable_irq(drvdata->ipc_irq);
			drvdata->ipc_irq_enabled = true;
		}
		break;
	case QBT_DISABLE_IPC:
		if (drvdata->ipc_irq_enabled) {
			disable_irq(drvdata->ipc_irq);
			drvdata->ipc_irq_enabled = false;
		}
		break;
	case QBT_ENABLE_FD:
		/* Arm touch-derived finger detect (and the GPIO line if wired). */
		drvdata->fd_enabled = true;
		if (drvdata->fd_gpio && !drvdata->fd_irq_enabled) {
			enable_irq(drvdata->fd_irq);
			drvdata->fd_irq_enabled = true;
		}
		break;
	case QBT_DISABLE_FD:
		drvdata->fd_enabled = false;
		if (drvdata->fd_gpio && drvdata->fd_irq_enabled) {
			disable_irq(drvdata->fd_irq);
			drvdata->fd_irq_enabled = false;
		}
		break;
	case QBT_IS_WUHB_CONNECTED: {
		struct qbt_wuhb_connected_status status = {
			.is_wuhb_connected = drvdata->is_wuhb_connected,
		};

		if (copy_to_user(uarg, &status, sizeof(status)))
			rc = -EFAULT;
		break;
	}
	case QBT_SEND_KEY_EVENT: {
		struct qbt_key_event ev;

		if (copy_from_user(&ev, uarg, sizeof(ev))) {
			rc = -EFAULT;
			break;
		}
		input_event(drvdata->in_dev, EV_KEY, ev.key, ev.value);
		input_sync(drvdata->in_dev);
		break;
	}
	case QBT_GET_TOUCH_FD_VERSION: {
		struct qbt_touch_fd_version ver = {
			.version = QBT_TOUCH_FD_VERSION_3,
		};

		if (copy_to_user(uarg, &ver, sizeof(ver)))
			rc = -EFAULT;
		break;
	}
	case QBT_CONFIGURE_TOUCH_FD_V2:
	case QBT_CONFIGURE_TOUCH_FD_V3:
		rc = qbt_ioctl_configure_touch_fd(drvdata, cmd, uarg);
		break;
	case QBT_INTR2_TEST: {
		struct qbt_intr2_test test;

		if (copy_from_user(&test, uarg, sizeof(test))) {
			rc = -EFAULT;
			break;
		}
		if (drvdata->intr2_gpio)
			gpiod_set_value(drvdata->intr2_gpio, test.state);
		else
			rc = -ENODEV;
		break;
	}
	case QBT_LPTW_TOUCH: {
		struct qbt_test_touch touch;
		struct qbt_fd_event evt = {};
		struct timespec64 ts;

		if (copy_from_user(&touch, uarg, sizeof(touch))) {
			rc = -EFAULT;
			break;
		}
		ktime_get_ts64(&ts);
		evt.timestamp.tv_sec = ts.tv_sec;
		evt.timestamp.tv_nsec = ts.tv_nsec;
		evt.x = touch.x;
		evt.y = touch.y;
		evt.id = QBT_LPTW_FINGER_ID;
		evt.state = touch.state;
		evt.touch_valid = true;
		qbt_queue_fd_event(drvdata, &evt);
		break;
	}
	case QBT_ACQUIRE_WAKELOCK: {
		struct qbt_wakelock_timeout to;

		if (copy_from_user(&to, uarg, sizeof(to)))
			to.timeout_ms = QBT_IOCTL_WAKELOCK_TIMEOUT_MS;
		__pm_wakeup_event(drvdata->ws,
				  to.timeout_ms ? to.timeout_ms
						: QBT_IOCTL_WAKELOCK_TIMEOUT_MS);
		break;
	}
	case QBT_RELEASE_WAKELOCK:
		__pm_relax(drvdata->ws);
		break;
	case QBT_CONFIGURE_TOUCH_FD:
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&drvdata->mutex);
	return rc;
}

static const struct file_operations qbt_fops = {
	.owner		= THIS_MODULE,
	.open		= qbt_open,
	.release	= qbt_release,
	.read		= qbt_read,
	.poll		= qbt_poll,
	.unlocked_ioctl	= qbt_ioctl,
};

static void qbt_ws_unregister(void *ws)
{
	wakeup_source_unregister(ws);
}

static int qbt_create_devices(struct qbt_drvdata *drvdata)
{
	struct device *dev = drvdata->dev;
	struct device *cdev;
	int rc;

	rc = alloc_chrdev_region(&drvdata->devt, 0, QBT_NUM_DEVS, QBT_DEV);
	if (rc)
		return dev_err_probe(dev, rc, "failed to alloc chrdev region\n");

	drvdata->class = class_create(QBT_DEV);
	if (IS_ERR(drvdata->class)) {
		rc = PTR_ERR(drvdata->class);
		goto err_region;
	}

	cdev_init(&drvdata->fd_cdev, &qbt_fops);
	drvdata->fd_cdev.owner = THIS_MODULE;
	rc = cdev_add(&drvdata->fd_cdev,
		      MKDEV(MAJOR(drvdata->devt), QBT_MINOR_FD), 1);
	if (rc)
		goto err_class;

	cdev_init(&drvdata->ipc_cdev, &qbt_fops);
	drvdata->ipc_cdev.owner = THIS_MODULE;
	rc = cdev_add(&drvdata->ipc_cdev,
		      MKDEV(MAJOR(drvdata->devt), QBT_MINOR_IPC), 1);
	if (rc)
		goto err_fd_cdev;

	cdev = device_create(drvdata->class, dev,
			     MKDEV(MAJOR(drvdata->devt), QBT_MINOR_FD), NULL,
			     "%s_fd", QBT_DEV);
	if (IS_ERR(cdev)) {
		rc = PTR_ERR(cdev);
		goto err_ipc_cdev;
	}

	cdev = device_create(drvdata->class, dev,
			     MKDEV(MAJOR(drvdata->devt), QBT_MINOR_IPC), NULL,
			     "%s_ipc", QBT_DEV);
	if (IS_ERR(cdev)) {
		rc = PTR_ERR(cdev);
		goto err_fd_dev;
	}

	return 0;

err_fd_dev:
	device_destroy(drvdata->class, MKDEV(MAJOR(drvdata->devt), QBT_MINOR_FD));
err_ipc_cdev:
	cdev_del(&drvdata->ipc_cdev);
err_fd_cdev:
	cdev_del(&drvdata->fd_cdev);
err_class:
	class_destroy(drvdata->class);
err_region:
	unregister_chrdev_region(drvdata->devt, QBT_NUM_DEVS);
	return rc;
}

static void qbt_destroy_devices(struct qbt_drvdata *drvdata)
{
	device_destroy(drvdata->class, MKDEV(MAJOR(drvdata->devt), QBT_MINOR_IPC));
	device_destroy(drvdata->class, MKDEV(MAJOR(drvdata->devt), QBT_MINOR_FD));
	cdev_del(&drvdata->ipc_cdev);
	cdev_del(&drvdata->fd_cdev);
	class_destroy(drvdata->class);
	unregister_chrdev_region(drvdata->devt, QBT_NUM_DEVS);
}

static int qbt_register_input(struct qbt_drvdata *drvdata)
{
	struct input_dev *in_dev;

	in_dev = devm_input_allocate_device(drvdata->dev);
	if (!in_dev)
		return -ENOMEM;

	in_dev->name = "qbt_key_input";
	in_dev->phys = "qbt/input0";
	in_dev->id.bustype = BUS_HOST;

	/*
	 * The sensor has no physical keys. QBT_SEND_KEY_EVENT only exists to
	 * emit a wake event on a fingerprint tap, so advertise just KEY_WAKEUP
	 * rather than the whole keymap (which otherwise makes input classifiers
	 * mistake this for a keyboard/pointer device).
	 */
	input_set_capability(in_dev, EV_KEY, KEY_WAKEUP);

	drvdata->in_dev = in_dev;
	return input_register_device(in_dev);
}

static int qbt_parse_dt(struct qbt_drvdata *drvdata)
{
	struct device *dev = drvdata->dev;

	drvdata->fd_gpio = devm_gpiod_get_optional(dev, "qcom,finger-detect",
						   GPIOD_IN);
	if (IS_ERR(drvdata->fd_gpio))
		return dev_err_probe(dev, PTR_ERR(drvdata->fd_gpio),
				     "failed to get finger-detect gpio\n");
	drvdata->is_wuhb_connected = !!drvdata->fd_gpio;

	drvdata->intr2_gpio = devm_gpiod_get_optional(dev, "qcom,intr2",
						      GPIOD_OUT_LOW);
	if (IS_ERR(drvdata->intr2_gpio))
		return dev_err_probe(dev, PTR_ERR(drvdata->intr2_gpio),
				     "failed to get intr2 gpio\n");

	return 0;
}

static int qbt_setup_irqs(struct platform_device *pdev,
			  struct qbt_drvdata *drvdata)
{
	struct device *dev = drvdata->dev;
	int rc;

	/* The IPC doorbell line's trigger type comes from the DT interrupt. */
	drvdata->ipc_irq = platform_get_irq(pdev, 0);
	if (drvdata->ipc_irq < 0)
		return drvdata->ipc_irq;

	/* start disabled; userspace enables via QBT_ENABLE_IPC */
	rc = devm_request_threaded_irq(dev, drvdata->ipc_irq, NULL,
				       qbt_ipc_irq_handler,
				       IRQF_ONESHOT | IRQF_NO_AUTOEN,
				       "qbt_ipc", drvdata);
	if (rc)
		return dev_err_probe(dev, rc, "failed to request IPC irq\n");

	if (!drvdata->fd_gpio)
		return 0;

	drvdata->fd_irq = gpiod_to_irq(drvdata->fd_gpio);
	if (drvdata->fd_irq < 0)
		return dev_err_probe(dev, drvdata->fd_irq,
				     "failed to map finger-detect irq\n");

	rc = devm_request_threaded_irq(dev, drvdata->fd_irq, NULL,
				       qbt_fd_irq_handler,
				       IRQF_ONESHOT | IRQF_TRIGGER_RISING |
				       IRQF_TRIGGER_FALLING | IRQF_NO_AUTOEN,
				       "qbt_fd", drvdata);
	if (rc)
		return dev_err_probe(dev, rc,
				     "failed to request finger-detect irq\n");

	return 0;
}

static int qbt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qbt_drvdata *drvdata;
	int rc;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = dev;
	platform_set_drvdata(pdev, drvdata);

	drvdata->fd_scratch = devm_kzalloc(dev, sizeof(*drvdata->fd_scratch),
					   GFP_KERNEL);
	if (!drvdata->fd_scratch)
		return -ENOMEM;

	mutex_init(&drvdata->mutex);
	mutex_init(&drvdata->ipc_events_mutex);
	mutex_init(&drvdata->fd_events_mutex);
	INIT_KFIFO(drvdata->ipc_events);
	INIT_KFIFO(drvdata->fd_events);
	init_waitqueue_head(&drvdata->ipc_wait);
	init_waitqueue_head(&drvdata->fd_wait);
	atomic_set(&drvdata->ipc_available, 1);
	atomic_set(&drvdata->fd_available, 1);
	spin_lock_init(&drvdata->touch.lock);
	INIT_WORK(&drvdata->touch.work, qbt_touch_work_func);

	rc = qbt_parse_dt(drvdata);
	if (rc)
		return rc;

	drvdata->ws = wakeup_source_register(dev, QBT_DEV);
	if (!drvdata->ws)
		return -ENOMEM;
	rc = devm_add_action_or_reset(dev, qbt_ws_unregister, drvdata->ws);
	if (rc)
		return rc;

	rc = qbt_register_input(drvdata);
	if (rc)
		return dev_err_probe(dev, rc, "failed to register input\n");

	/* Snoop the touchscreen to derive finger-detect events for the HAL. */
	qbt_touch_handler.private = drvdata;
	rc = input_register_handler(&qbt_touch_handler);
	if (rc)
		return dev_err_probe(dev, rc,
				     "failed to register touch handler\n");
	rc = devm_add_action_or_reset(dev, qbt_touch_handler_unregister, drvdata);
	if (rc)
		return rc;

	rc = qbt_setup_irqs(pdev, drvdata);
	if (rc)
		return rc;

	rc = qbt_create_devices(drvdata);
	if (rc)
		return rc;

	device_init_wakeup(dev, true);
	dev_info(dev, "Qualcomm ultrasonic fingerprint IPC handler ready%s\n",
		 drvdata->is_wuhb_connected ? " (WUHB connected)" : "");

	return 0;
}

static void qbt_remove(struct platform_device *pdev)
{
	struct qbt_drvdata *drvdata = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, false);
	qbt_destroy_devices(drvdata);
}

static int qbt_suspend(struct device *dev)
{
	struct qbt_drvdata *drvdata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev) && drvdata->ipc_irq_enabled)
		enable_irq_wake(drvdata->ipc_irq);

	return 0;
}

static int qbt_resume(struct device *dev)
{
	struct qbt_drvdata *drvdata = dev_get_drvdata(dev);

	if (device_may_wakeup(dev) && drvdata->ipc_irq_enabled)
		disable_irq_wake(drvdata->ipc_irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qbt_pm_ops, qbt_suspend, qbt_resume);

static const struct of_device_id qbt_match[] = {
	{ .compatible = "qcom,qbt-handler" },
	{ }
};
MODULE_DEVICE_TABLE(of, qbt_match);

static struct platform_driver qbt_driver = {
	.driver = {
		.name		= "qbt_handler",
		.of_match_table	= qbt_match,
		.pm		= pm_sleep_ptr(&qbt_pm_ops),
	},
	.probe	= qbt_probe,
	.remove	= qbt_remove,
};
module_platform_driver(qbt_driver);

MODULE_DESCRIPTION("Qualcomm ultrasonic fingerprint IPC handler");
MODULE_LICENSE("GPL");
