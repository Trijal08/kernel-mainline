/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google AOC (Always-On Compute) coprocessor - in-kernel service API.
 *
 * The AOC exposes a table of named services, each a bidirectional pair of
 * rings or message queues in the shared carveout.  A consumer (for example the
 * ASoC card that streams audio through the AOC) looks a service up by name and
 * then reads, writes and takes a doorbell notification on it.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */
#ifndef __LINUX_SOC_GOOGLE_AOC_H
#define __LINUX_SOC_GOOGLE_AOC_H

#include <linux/types.h>

struct device;
struct aoc_service;

/**
 * aoc_service_find() - look up an AOC service by name
 * @aoc_dev: the AOC platform device (from a DT phandle)
 * @name: the service name, e.g. "audio_playback0"
 *
 * Returns an opaque service handle, or NULL if the AOC is not online or has no
 * such service.  Valid until the AOC device is removed.
 */
struct aoc_service *aoc_service_find(struct device *aoc_dev, const char *name);

/* Read one message / drain the ring (AOC->AP).  Length, or a negative errno. */
int aoc_service_read(struct aoc_service *svc, void *buf, size_t len);

/* Write one message / fill the ring (AP->AOC) and ring the doorbell. */
int aoc_service_write(struct aoc_service *svc, const void *buf, size_t len);

/* Whether a read / write would make progress right now. */
bool aoc_service_can_read(struct aoc_service *svc);
bool aoc_service_can_write(struct aoc_service *svc);

/*
 * The AOC's running byte position in the service's data stream, for a PCM
 * hardware pointer: bytes consumed for playback, bytes produced for capture.
 */
u32 aoc_service_progress(struct aoc_service *svc, bool playback);

/**
 * aoc_service_set_handler() - register a doorbell callback for a service
 * @svc: the service handle
 * @handler: called from the mailbox interrupt when the AOC signals this
 *           service's channel, or NULL to unregister
 * @priv: passed back to the handler
 *
 * The handler runs in interrupt context.
 */
void aoc_service_set_handler(struct aoc_service *svc,
			     void (*handler)(struct aoc_service *svc, void *priv),
			     void *priv);

/**
 * aoc_ts_to_boottime_ns() - map an AOC-domain timestamp to CLOCK_BOOTTIME
 * @aoc_dev: the AOC platform device
 * @aoc_ts: nanoseconds in the AOC's clock domain, as it stamps its samples
 *
 * The AOC counts from the architected counter that CLOCK_BOOTTIME is derived
 * from, so the conversion is exact.  Safe from atomic context.
 *
 * Returns CLOCK_BOOTTIME nanoseconds, or 0 if the AOC is not up or the stamp
 * is too far from now to trust, in which case the caller should substitute
 * the current boottime.
 */
u64 aoc_ts_to_boottime_ns(struct device *aoc_dev, u64 aoc_ts);

#endif /* __LINUX_SOC_GOOGLE_AOC_H */
