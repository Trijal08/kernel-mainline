/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 Google, Inc.
 */
#ifndef __LINUX_TRUSTY_TRUSTY_H
#define __LINUX_TRUSTY_TRUSTY_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

/**
 * trusty_std_call32() - issue a standard (yielding) call to Trusty
 * @dev:   the Trusty device
 * @smcnr: SMC function number (an SMC_SC_* value)
 * @a0, @a1, @a2: arguments
 *
 * Standard calls can be preempted and are resumed transparently by the
 * driver.  Must not be called from atomic context.
 *
 * Return: the secure-monitor result, or a negative SM_ERR_* code.
 */
s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);

/**
 * trusty_fast_call32() - issue a fast (atomic, non-yielding) call to Trusty
 * @dev:   the Trusty device
 * @smcnr: SMC function number (an SMC_FC_* value)
 * @a0, @a1, @a2: arguments
 *
 * Return: the secure-monitor result, or a negative SM_ERR_* code.
 */
s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2);

#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2);
#endif

/**
 * trusty_get_api_version() - the negotiated Trusty API version
 * @dev: the Trusty device
 */
u32 trusty_get_api_version(struct device *dev);

/**
 * trusty_version_str_get() - the Trusty build version string
 * @dev: the Trusty device
 */
const char *trusty_version_str_get(struct device *dev);

/*
 * Memory sharing with the secure world.  On this API version (>= MEM_OBJ) the
 * transfer uses the FF-A memory-object protocol; the returned id is an FF-A
 * handle the secure world uses to map the memory.
 */
typedef u64 trusty_shared_mem_id_t;

struct ns_mem_page_info {
	u64 paddr;
	u8 ffa_mem_attr;
	u8 ffa_mem_perm;
	u64 compat_attr;
};

int trusty_encode_page_info(struct ns_mem_page_info *inf,
			    struct page *page, pgprot_t pgprot);

int trusty_share_memory(struct device *dev, trusty_shared_mem_id_t *id,
			struct scatterlist *sglist, unsigned int nents,
			pgprot_t pgprot);
int trusty_share_memory_compat(struct device *dev, trusty_shared_mem_id_t *id,
			       struct scatterlist *sglist, unsigned int nents,
			       pgprot_t pgprot);
int trusty_transfer_memory(struct device *dev, u64 *id,
			   struct scatterlist *sglist, unsigned int nents,
			   pgprot_t pgprot, u64 tag, bool lend);
int trusty_reclaim_memory(struct device *dev, trusty_shared_mem_id_t id,
			  struct scatterlist *sglist, unsigned int nents);

struct dma_buf;
void trusty_register_func_for_dma_buf(
	u64 (*get_ffa_tag)(struct dma_buf *dma_buf),
	int (*get_shared_mem_id)(struct dma_buf *dma_buf,
				 trusty_shared_mem_id_t *id));

/*
 * Events reported to trusty_call_notifier callbacks around every standard call.
 * @TRUSTY_CALL_PREPARE:  fired before entering the secure world
 * @TRUSTY_CALL_RETURNED: fired after the call returns (used e.g. by the virtio
 *                        transport to poll its virtqueues for new activity)
 */
enum {
	TRUSTY_CALL_PREPARE,
	TRUSTY_CALL_RETURNED,
};

int trusty_call_notifier_register(struct device *dev, struct notifier_block *n);
int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n);

/**
 * struct trusty_nop - a piece of work handed to Trusty on the next idle call
 * @node: queue linkage, owned by the Trusty core
 * @args: the three SMC arguments passed with SMC_SC_NOP
 *
 * A nop makes the driver enter Trusty (via SMC_SC_NOP) so its scheduler can run
 * pending secure-world work; @args select a non-default nop handler.
 */
struct trusty_nop {
	struct list_head node;
	u32 args[3];
};

static inline void trusty_nop_init(struct trusty_nop *nop,
				   u32 arg0, u32 arg1, u32 arg2)
{
	INIT_LIST_HEAD(&nop->node);
	nop->args[0] = arg0;
	nop->args[1] = arg1;
	nop->args[2] = arg2;
}

void trusty_enqueue_nop(struct device *dev, struct trusty_nop *nop);
void trusty_dequeue_nop(struct device *dev, struct trusty_nop *nop);

#endif /* __LINUX_TRUSTY_TRUSTY_H */
