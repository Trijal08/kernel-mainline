/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Userspace ABI for the Qualcomm ultrasonic fingerprint IPC handler.
 *
 * The AP-side driver is only a doorbell/relay: the fingerprint sensor itself
 * lives on a TrustZone-owned SPI bus that is invisible to Linux. The driver
 * exposes two character devices:
 *
 *   /dev/qbt_ipc - read() returns one enum qbt_fw_event (4 bytes) per call,
 *                  signalling that the secure firmware has data ready.
 *   /dev/qbt_fd  - read() returns a struct qbt_fd_userspace_buf describing
 *                  finger-detect (touch) events.
 *
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#ifndef _UAPI_LINUX_QBT_HANDLER_H_
#define _UAPI_LINUX_QBT_HANDLER_H_

#include <linux/types.h>
#include <linux/time_types.h>

/* ioctl request codes (historical plain integers, kept for ABI stability) */
#define QBT_IS_WUHB_CONNECTED		100
#define QBT_SEND_KEY_EVENT		101
#define QBT_ENABLE_IPC			102
#define QBT_DISABLE_IPC			103
#define QBT_ENABLE_FD			104
#define QBT_DISABLE_FD			105
#define QBT_CONFIGURE_TOUCH_FD		106
#define QBT_ACQUIRE_WAKELOCK		107
#define QBT_RELEASE_WAKELOCK		108
#define QBT_GET_TOUCH_FD_VERSION	109
#define QBT_CONFIGURE_TOUCH_FD_V2	110
#define QBT_INTR2_TEST			111
#define QBT_CONFIGURE_TOUCH_FD_V3	112
#define QBT_LPTW_TOUCH			1000

#define QBT_LPTW_FINGER_ID		(-1)

/*
 * enum qbt_finger_events - finger-detect event states reported in
 * struct qbt_fd_event::state
 */
enum qbt_finger_events {
	QBT_EVENT_FINGER_UP,
	QBT_EVENT_FINGER_DOWN,
	QBT_EVENT_FINGER_MOVE,
};

/*
 * enum qbt_fw_event - value delivered by a read() on /dev/qbt_ipc
 * @FW_EVENT_FINGER_DOWN: finger down detected
 * @FW_EVENT_FINGER_UP:   finger up detected
 * @FW_EVENT_IPC:         an IPC doorbell from the secure firmware is pending
 */
enum qbt_fw_event {
	FW_EVENT_FINGER_DOWN = 1,
	FW_EVENT_FINGER_UP = 2,
	FW_EVENT_IPC = 3,
};

/* QBT_IS_WUHB_CONNECTED argument */
struct qbt_wuhb_connected_status {
	__u8 is_wuhb_connected;
};

/* QBT_SEND_KEY_EVENT argument */
struct qbt_key_event {
	__s32 key;
	__s32 value;
};

/* QBT_GET_TOUCH_FD_VERSION argument */
struct qbt_touch_fd_version {
	__s32 version;
};

/* QBT_CONFIGURE_TOUCH_FD_V2 argument */
struct qbt_touch_config_v2 {
	struct qbt_touch_fd_version version;
	__u8 touch_fd_enable;
	__u8 rad_filter_enable;
	__s32 left;
	__s32 top;
	__s32 right;
	__s32 bottom;
	__s32 rad_x;
	__s32 rad_y;
};

/* QBT_CONFIGURE_TOUCH_FD_V3 argument (adds intr2_enable) */
struct qbt_touch_config_v3 {
	struct qbt_touch_fd_version version;
	__u8 touch_fd_enable;
	__u8 rad_filter_enable;
	__s32 left;
	__s32 top;
	__s32 right;
	__s32 bottom;
	__s32 rad_x;
	__s32 rad_y;
	__u8 intr2_enable;
};

/* QBT_INTR2_TEST argument */
struct qbt_intr2_test {
	__s32 state;
};

/* QBT_LPTW_TOUCH argument: inject a low-power-touch-wake gesture */
struct qbt_test_touch {
	__s32 x;
	__s32 y;
	__s32 state;	/* 0 = up, 1 = down, 2 = move */
};

/* QBT_ACQUIRE_WAKELOCK argument */
struct qbt_wakelock_timeout {
	__u32 timeout_ms;
};

/*
 * struct qbt_fd_event - one finger-detect event, as returned in the
 * qbt_fd_userspace_buf array by a read() on /dev/qbt_fd.
 */
struct qbt_fd_event {
	struct __kernel_timespec timestamp;
	__s32 x;
	__s32 y;
	__s32 id;
	__s32 state;		/* enum qbt_finger_events */
	__u8 touch_valid;
};

#define QBT_MAX_FD_EVENTS 128

/* read() payload for /dev/qbt_fd */
struct qbt_fd_userspace_buf {
	__u32 num_events;
	struct qbt_fd_event fd_events[QBT_MAX_FD_EVENTS];
};

#endif /* _UAPI_LINUX_QBT_HANDLER_H_ */
