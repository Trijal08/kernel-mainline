/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Client session for the AoC USF (Unified Sensor Framework) firmware.
 *
 * Wraps the AOCC transport and the request/response half of the USF protocol:
 * open the channels, resolve the server handles, enumerate sensors, and
 * start/stop sampling. Consumers add the policy -- what a sensor means and
 * where its samples go (IIO buffers, input events).
 *
 * The USF servers stay dormant until the sensor registry has been uploaded, and
 * usf_session_bootstrap() returns -EAGAIN while they are, so a consumer should
 * retry rather than fail probe. The registry is loaded in-kernel out of the
 * firmware search path; see usf_registry_load().
 */
#ifndef _SOC_GOOGLE_USF_H_
#define _SOC_GOOGLE_USF_H_

#include <linux/types.h>
#include <linux/soc/google/usf-proto.h>

struct device;
struct device_node;
struct usf_session;

#define USF_NAME_MAX	64	/* longest sensor name we keep */

/**
 * usf_session_alloc() - allocate a USF client session
 * @dev: owning device; the session is devm-managed against it
 * @aoc_dev: the AOC platform device carrying the USF services
 * @sample: callback for every type-9 sample batch on either channel. It runs in
 *          the AOCC demux kthread under an internal lock: it must not sleep.
 * @ctx: opaque context for @sample
 *
 * Does not touch the AoC. Return: session or ERR_PTR.
 */
struct usf_session *usf_session_alloc(struct device *dev,
				      struct device *aoc_dev,
				      void (*sample)(void *ctx, const u8 *pay,
						     u32 plen),
				      void *ctx);

/**
 * usf_session_open() - open the wake and non-wake AOCC channels
 *
 * Return: 0, or -ENODEV if the AOCC service is not probed yet (retry).
 */
int usf_session_open(struct usf_session *s);

/** usf_session_close() - close both channels (idempotent) */
void usf_session_close(struct usf_session *s);

/** usf_session_dev() - the device the session was allocated against */
struct device *usf_session_dev(struct usf_session *s);

/**
 * usf_session_registry_open() - resolve the Registry server handle
 *
 * The Registry server answers before the registry itself is loaded, which is
 * what makes loading it from here possible at all.
 *
 * Return: 0, or -ENODEV.
 */
int usf_session_registry_open(struct usf_session *s);

/**
 * usf_session_load_script() - upload one registry script
 * @script: `.reg` text; NUL-terminated on the wire, so @len counts the NUL
 * @len: length of @script including its terminator
 * @cdt: device CDT the AoC evaluates the script's ?+/?- directives against
 *
 * Fragmented automatically when it exceeds the AOCC MTU.  Return: 0 or errno.
 */
int usf_session_load_script(struct usf_session *s, const u8 *script,
			    size_t len, u32 cdt);

/**
 * usf_session_set_loaded() - set /.loaded = 1
 *
 * Fires the AoC's one-shot USF startup, after which the sensors probe and
 * usf_session_bootstrap() can resolve the remaining servers.  Call once, after
 * every script has been uploaded.  Return: 0 or errno.
 */
int usf_session_set_loaded(struct usf_session *s);

/**
 * usf_registry_load() - load and upload the whole sensor registry
 * @s: an open session
 * @np: device node naming the registry scripts and the CDT
 *
 * Reads each script through request_firmware(), uploads it, and finishes with
 * usf_session_set_loaded().  Return: 0, or a negative errno.
 */
int usf_registry_load(struct usf_session *s, struct device_node *np);

/**
 * usf_session_bootstrap() - resolve the USF server handles
 *
 * Return: 0 once SensorMgr and SampleChannel answer, -EAGAIN while they are
 * dormant (registry not loaded yet), or another negative errno.
 */
int usf_session_bootstrap(struct usf_session *s);

/**
 * usf_session_sensor_list() - enumerate sensor handles
 * @handles: caller array, @max entries
 *
 * Retries internally while the AoC's asynchronous chip probe settles.
 * Return: number of handles (0 if none settled) or -errno.
 */
int usf_session_sensor_list(struct usf_session *s, u32 *handles, int max);

/**
 * usf_session_sensor_info() - fetch a sensor's name and resolution
 * @name: filled with the USF name (USF_NAME_MAX)
 * @res_bits: f32 bits of the reported resolution, 0 if absent; may be NULL
 *
 * Sensor handles are device-specific, so match sensors by name, never by a
 * hardcoded handle.
 */
int usf_session_sensor_info(struct usf_session *s, u32 handle, char *name,
			    size_t namesz, u32 *res_bits);

/**
 * usf_session_start_sampling() - CreateSampling on a sensor
 * @mode: see enum usf_sampling_mode; picks the delivery service
 * @client_id: AP-chosen opaque id, echoed in every sample batch
 * @sampling_id: out, the firmware's stream id
 */
int usf_session_start_sampling(struct usf_session *s, u32 handle,
			       enum usf_sampling_mode mode, s64 period_ns,
			       u32 client_id, u32 *sampling_id);

/**
 * usf_session_reconfig() - retune a live stream
 * @enable: false does NOT stop the stream -- the firmware ignores it. Use
 *          usf_session_stop_sampling().
 *
 * Fire-and-forget: sent without waiting for a response, since a dropped retune
 * costs only the old rate.
 */
int usf_session_reconfig(struct usf_session *s, u32 handle, u32 sampling_id,
			 s64 period_ns, s64 max_latency_ns, bool enable);

/**
 * usf_session_stop_sampling() - StopSampling on a live stream
 *
 * Must be this request and its response must be checked: the firmware silently
 * ignores ReconfigSampling(enable=0), and leaked streams accumulate into a
 * multi-kHz mailbox interrupt storm.
 */
int usf_session_stop_sampling(struct usf_session *s, u32 handle,
			      u32 sampling_id);

#endif /* _SOC_GOOGLE_USF_H_ */
