/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2013-2014 Google Inc. All rights reserved
 *
 * Trusty and TF-A also have a copy of this header.
 * Please keep the copies in sync.
 */
#ifndef __LINUX_TRUSTY_SMCALL_H
#define __LINUX_TRUSTY_SMCALL_H

#define SMC_NUM_ENTITIES	64
#define SMC_NUM_ARGS		4
#define SMC_NUM_PARAMS		(SMC_NUM_ARGS - 1)

#define SMC_IS_FASTCALL(smc_nr)	((smc_nr) & 0x80000000)
#define SMC_IS_SMC64(smc_nr)	((smc_nr) & 0x40000000)
#define SMC_ENTITY(smc_nr)	(((smc_nr) & 0x3F000000) >> 24)
#define SMC_FUNCTION(smc_nr)	((smc_nr) & 0x0000FFFF)

#define SMC_NR(entity, fn, fastcall, smc64) ((((fastcall) & 0x1U) << 31) | \
					     (((smc64) & 0x1U) << 30) | \
					     (((entity) & 0x3FU) << 24) | \
					     ((fn) & 0xFFFFU) \
					    )

#define SMC_FASTCALL_NR(entity, fn)	SMC_NR((entity), (fn), 1, 0)
#define SMC_STDCALL_NR(entity, fn)	SMC_NR((entity), (fn), 0, 0)
#define SMC_FASTCALL64_NR(entity, fn)	SMC_NR((entity), (fn), 1, 1)
#define SMC_STDCALL64_NR(entity, fn)	SMC_NR((entity), (fn), 0, 1)

#define SMC_ENTITY_ARCH			0	/* ARM Architecture calls */
#define SMC_ENTITY_CPU			1	/* CPU Service calls */
#define SMC_ENTITY_SIP			2	/* SIP Service calls */
#define SMC_ENTITY_OEM			3	/* OEM Service calls */
#define SMC_ENTITY_STD			4	/* Standard Service calls */
#define SMC_ENTITY_RESERVED		5	/* Reserved for future use */
#define SMC_ENTITY_TRUSTED_APP		48	/* Trusted Application calls */
#define SMC_ENTITY_TRUSTED_OS		50	/* Trusted OS calls */
#define SMC_ENTITY_LOGGING		51	/* Used for secure -> nonsecure logging */
#define SMC_ENTITY_SECURE_MONITOR	60	/* Trusted OS calls internal to secure monitor */

/* FC = Fast call, SC = Standard call */
#define SMC_SC_RESTART_LAST	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 0)
#define SMC_SC_LOCKED_NOP	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 1)
#define SMC_SC_RESTART_FIQ	SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 2)
#define SMC_SC_NOP		SMC_STDCALL_NR(SMC_ENTITY_SECURE_MONITOR, 3)

#define SMC_FC_RESERVED		SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 0)
#define SMC_FC_FIQ_EXIT		SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 1)
#define SMC_FC_REQUEST_FIQ	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 2)
#define SMC_FC_GET_NEXT_IRQ	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 3)

#define SMC_FC_CPU_SUSPEND	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 7)
#define SMC_FC_CPU_RESUME	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 8)

#define SMC_FC_AARCH_SWITCH	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 9)
#define SMC_FC_GET_VERSION_STR	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 10)

/*
 * SMC_FC_API_VERSION - Find and select supported API version.
 *
 * Returns the API version supported by Trusty.  Must be called before any
 * call whose behaviour depends on the API version.
 */
#define TRUSTY_API_VERSION_RESTART_FIQ	(1)
#define TRUSTY_API_VERSION_SMP		(2)
#define TRUSTY_API_VERSION_SMP_NOP	(3)
#define TRUSTY_API_VERSION_PHYS_MEM_OBJ	(4)
#define TRUSTY_API_VERSION_MEM_OBJ	(5)
#define TRUSTY_API_VERSION_CURRENT	(5)
#define SMC_FC_API_VERSION	SMC_FASTCALL_NR(SMC_ENTITY_SECURE_MONITOR, 11)

/* TRUSTED_OS entity calls, used by the virtio transport */
#define SMC_SC_VIRTIO_GET_DESCR	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 20)
#define SMC_SC_VIRTIO_START	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 21)
#define SMC_SC_VIRTIO_STOP	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 22)

#define SMC_SC_VDEV_RESET	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 23)
#define SMC_SC_VDEV_KICK_VQ	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 24)
#define SMC_NC_VDEV_KICK_VQ	SMC_STDCALL_NR(SMC_ENTITY_TRUSTED_OS, 25)

#endif /* __LINUX_TRUSTY_SMCALL_H */
