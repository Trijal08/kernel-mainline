/* SPDX-License-Identifier: GPL-2.0 */
/*
 * In-kernel client for AoC Channelized Comms (AOCC).
 *
 * AOCC multiplexes many logical channels over a single AOC service, so that
 * several independent clients can share one ring.  Every message carries a
 * 4-byte header naming the channel it belongs to; channel 0 is reserved for
 * the control messages that open and close the others.
 *
 * Downstream this lives inside a char driver that also exports each service to
 * userspace as /dev/acd-<service>.  Here only the in-kernel side exists: the
 * sensor stack runs entirely in the kernel, so there is no userspace peer to
 * share the channel index space with.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */
#ifndef __LINUX_SOC_GOOGLE_AOC_CHANNEL_H
#define __LINUX_SOC_GOOGLE_AOC_CHANNEL_H

#include <linux/types.h>

struct device;
struct aocc_channel;

/**
 * aocc_kernel_open_channel() - open an in-kernel AOCC channel on a service
 * @aoc_dev: the AOC platform device the service belongs to
 * @service_name: AOCC service name, e.g. "com.google.usf"
 * @rx: called for every message demultiplexed to this channel.  @payload and
 *      @len exclude the 4-byte channel header.  It runs in the transport's
 *      demux worker under an internal lock, so it must not sleep and must not
 *      re-enter aocc_kernel_open_channel() or aocc_kernel_close_channel().
 * @ctx: opaque context handed back to @rx.
 *
 * Return: a channel handle, or ERR_PTR(-ENODEV) if the AOC is not up or has no
 * such service, in which case the caller may defer and retry.
 */
struct aocc_channel *
aocc_kernel_open_channel(struct device *aoc_dev, const char *service_name,
			 void (*rx)(void *ctx, const void *payload, size_t len),
			 void *ctx);

/**
 * aocc_kernel_listen() - attach the demux to a service without opening a channel
 * @aoc_dev: the AOC platform device the service belongs to
 * @service_name: AOCC service name, e.g. "com.google.usf.non_wake_up"
 *
 * The AOC delivers a stream on whichever service matches the sensor's wakeup
 * class, tagged with the channel index that created it -- so a channel opened
 * on the wake service can be answered on its non-wake sibling.  Demux matches
 * on the channel index alone, so the sibling only has to be drained.
 *
 * Return: 0, or a negative errno.  Balanced by aocc_kernel_unlisten().
 */
int aocc_kernel_listen(struct device *aoc_dev, const char *service_name);

/** aocc_kernel_unlisten() - drop a reference taken by aocc_kernel_listen() */
void aocc_kernel_unlisten(struct device *aoc_dev, const char *service_name);

/**
 * aocc_kernel_write() - send one message on an in-kernel AOCC channel
 * @chan: handle from aocc_kernel_open_channel()
 * @payload: message bytes; the channel header is prepended internally
 * @len: length of @payload, at most AOCC_MAX_PAYLOAD
 *
 * May sleep.  Return: bytes accepted, or a negative errno.
 */
int aocc_kernel_write(struct aocc_channel *chan, const void *payload,
		      size_t len);

/** aocc_kernel_close_channel() - close a channel (NULL / ERR_PTR tolerated) */
void aocc_kernel_close_channel(struct aocc_channel *chan);

/* Largest message the AOC accepts, header included. */
#define AOCC_MAX_MSG_SIZE	1024
#define AOCC_MAX_PAYLOAD	(AOCC_MAX_MSG_SIZE - sizeof(u32))

#endif /* __LINUX_SOC_GOOGLE_AOC_CHANNEL_H */
