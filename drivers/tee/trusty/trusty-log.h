/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2015 Google, Inc.
 *
 * Trusty also has a copy of this header.  Please keep the copies in sync.
 */
#ifndef _TRUSTY_LOG_H_
#define _TRUSTY_LOG_H_

#include <linux/trusty/smcall.h>

/*
 * Ring buffer shared with the secure world: one secure producer, one
 * normal-world consumer.  @alloc/@put are free-running (unwrapped) indices;
 * @sz (a power of two) is the size of @data.
 */
struct log_rb {
	volatile u32 alloc;
	volatile u32 put;
	u32 sz;
	volatile char data[];
} __packed;

#define SMC_SC_SHARED_LOG_VERSION	SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 0)
#define SMC_SC_SHARED_LOG_ADD		SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 1)
#define SMC_SC_SHARED_LOG_RM		SMC_STDCALL_NR(SMC_ENTITY_LOGGING, 2)

#define TRUSTY_LOG_API_VERSION		1

#endif /* _TRUSTY_LOG_H_ */
