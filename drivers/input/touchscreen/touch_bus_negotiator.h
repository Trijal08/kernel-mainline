/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Touch Bus Negotiator (TBN) client for Google Pixel devices.
 *
 * Ported from the downstream google-modules/touch touch_bus_negotiator.
 * Only the AoC-channel arbitration mode (tbn,mode = 2, TBN_MODE_AOC_CHANNEL)
 * used by tegu is implemented; the reference GPIO and MOCK modes are dropped.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#ifndef _TOUCH_BUS_NEGOTIATOR_H
#define _TOUCH_BUS_NEGOTIATOR_H

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define TBN_REQUEST_BUS_TIMEOUT_MS 500
#define TBN_RELEASE_BUS_TIMEOUT_MS 500

enum tbn_mode {
	TBN_MODE_DISABLED = 0,
	TBN_MODE_GPIO,
	TBN_MODE_AOC_CHANNEL,
	TBN_MODE_MOCK,
};

enum tbn_bus_owner {
	TBN_BUS_OWNER_AP = 0,
	TBN_BUS_OWNER_AOC = 1,
};

/*
 * Wire protocol shared with the AoC TBN service.  The operation field is a
 * 4-byte little-endian value on the wire; a plain enum is int-sized on the
 * arm64 LP64 ABI, so it is layout-compatible with the downstream
 * "enum TbnOperation : __u32" without relying on the fixed-underlying-type
 * enum extension.
 */
enum TbnOperation {
	TBN_OPERATION_IDLE = 0,
	TBN_OPERATION_AP_RELEASE_BUS,
	TBN_OPERATION_AP_REQUEST_BUS,
	TBN_OPERATION_AOC_RESET,
	TBN_OPERATION_AOC_SEND_LPTW_EVENT,
};

struct TbnEvent {
	__u32 id;
	enum TbnOperation operation;
} __packed;

struct TbnEventHeader {
	__u32 id;
	__s32 err;
	enum TbnOperation operation;
	__u8 data[52];
} __packed;

struct TbnEventResponse {
	__u32 id;
	__s32 err;
	enum TbnOperation operation;
	bool lptw_triggered;
} __packed;

struct TbnLptwEvent {
	__u32 id;
	__s32 err;
	enum TbnOperation operation;
	__u16 x;
	__u16 y;
	__u16 major;
	__u16 minor;
	__s16 angle;
} __packed;

struct tbn_context {
	struct device *dev;
	struct completion bus_requested;
	struct completion bus_released;
	struct mutex dev_mask_mutex;
	u32 mode;
	u32 max_devices;
	u32 registered_mask;
	u32 requested_dev_mask;
	struct task_struct *aoc_channel_task;

	/* event management */
	struct TbnEventResponse event_resp;
	struct TbnEvent event;
	struct mutex event_lock;
	struct work_struct aoc_reset_work;
	struct workqueue_struct *event_wq;

	void (*lptw_event_cb)(struct TbnLptwEvent *lptw, void *user_data);
	void *lptw_event_cbdata;
};

/*
 * A touch driver may be built regardless of whether the negotiator is
 * reachable from it (TBN off, or =m while the touch driver is =y).  Provide
 * no-op stubs so the consumer always builds and links; with the stubs
 * register_tbn() yields mask 0 and tbn_ready() is false, so the consumer keeps
 * its plain AP-owns-the-bus suspend path.
 */
#if IS_REACHABLE(CONFIG_TOUCHSCREEN_TBN)
bool tbn_ready(void);
int register_tbn(u32 *output);
void unregister_tbn(u32 *output);
void register_tbn_lptw_callback(void (*callback)(struct TbnLptwEvent *lptw,
						 void *user_data),
				void *cbdata);
int tbn_request_bus_with_result(u32 dev_mask, bool *lptw_triggered);
int tbn_request_bus(u32 dev_mask);
int tbn_release_bus(u32 dev_mask);
#else
static inline bool tbn_ready(void)
{
	return false;
}
static inline int register_tbn(u32 *output)
{
	*output = 0;
	return 0;
}
static inline void unregister_tbn(u32 *output)
{
	*output = 0;
}
static inline void
register_tbn_lptw_callback(void (*callback)(struct TbnLptwEvent *lptw,
					    void *user_data),
			   void *cbdata)
{
}
static inline int tbn_request_bus_with_result(u32 dev_mask, bool *lptw_triggered)
{
	return -ENODEV;
}
static inline int tbn_request_bus(u32 dev_mask)
{
	return -ENODEV;
}
static inline int tbn_release_bus(u32 dev_mask)
{
	return -ENODEV;
}
#endif /* IS_REACHABLE(CONFIG_TOUCHSCREEN_TBN) */

#endif /* _TOUCH_BUS_NEGOTIATOR_H */
