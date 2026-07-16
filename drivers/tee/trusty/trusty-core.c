// SPDX-License-Identifier: GPL-2.0-only
/*
 * Trusty secure OS driver core: the SMC call interface to the Trusty TEE.
 *
 * Trusty runs in the secure world (below TF-A) and is reached through SMC
 * calls.  This driver owns the "android,trusty-smc-v1" device and exposes the
 * fast- and standard-call primitives that the higher Trusty layers (IRQ, log,
 * virtio/IPC) build on.
 *
 * Copyright (C) 2013 Google, Inc.
 */

#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <linux/trusty/arm_ffa.h>
#include <linux/trusty/sm_err.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>

#if IS_ENABLED(CONFIG_ARM64)
#include <asm/daifflags.h>
#endif

static struct platform_driver trusty_driver;

struct trusty_state {
	struct device *dev;
	struct mutex smc_lock;		/* serialises standard calls */
	struct completion cpu_idle_completion;
	struct atomic_notifier_head notifier;
	u32 api_version;
	char *version_str;
	bool trusty_panicked;

	/* nop machinery: lets the secure world run its own pending work */
	struct task_struct *nop_thread;
	wait_queue_head_t nop_event_wait;
	struct list_head nop_queue;
	spinlock_t nop_lock;		/* protects nop_queue and nop_signaled */
	bool nop_signaled;

	/* FF-A memory sharing (api >= TRUSTY_API_VERSION_MEM_OBJ) */
	void *ffa_tx;
	void *ffa_rx;
	u16 ffa_local_id;
	u16 ffa_remote_id;
	struct mutex share_memory_msg_lock;	/* serialises FF-A tx buffer use */
	struct device_dma_parameters dma_parms;
};

/*
 * Issue the SMC.  Trusty uses the SMCCC 1.2 eight-register convention; only the
 * first result register carries the return value for the calls this driver
 * makes.
 */
static unsigned long trusty_smc(unsigned long r0, unsigned long r1,
				unsigned long r2, unsigned long r3)
{
	struct arm_smccc_1_2_regs args = {
		.a0 = r0, .a1 = r1, .a2 = r2, .a3 = r3,
	};
	struct arm_smccc_1_2_regs res = { };

	arm_smccc_1_2_smc(&args, &res);
	return res.a0;
}

/* Eight-register SMC, for the FF-A calls whose results span r0..r3. */
static struct arm_smccc_1_2_regs trusty_smc8(unsigned long r0, unsigned long r1,
					     unsigned long r2, unsigned long r3,
					     unsigned long r4, unsigned long r5,
					     unsigned long r6, unsigned long r7)
{
	struct arm_smccc_1_2_regs args = {
		.a0 = r0, .a1 = r1, .a2 = r2, .a3 = r3,
		.a4 = r4, .a5 = r5, .a6 = r6, .a7 = r7,
	};
	struct arm_smccc_1_2_regs res = { };

	arm_smccc_1_2_smc(&args, &res);
	return res;
}

s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(!s))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(!SMC_IS_FASTCALL(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(SMC_IS_SMC64(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	return trusty_smc(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL_GPL(trusty_fast_call32);

#ifdef CONFIG_64BIT
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(!s))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(!SMC_IS_FASTCALL(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(!SMC_IS_SMC64(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	return trusty_smc(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL_GPL(trusty_fast_call64);
#endif

/*
 * Trusty runs standard calls with interrupts disabled on this CPU and yields
 * back with SM_ERR_INTERRUPTED whenever the normal world has pending work; the
 * caller re-enables interrupts and resumes with SMC_SC_RESTART_LAST.  On arm64
 * all DAIF exceptions are masked across the call so that FIQs stay routed to
 * the secure world.
 */
static void trusty_irq_disable_before_smc(void)
{
#if IS_ENABLED(CONFIG_ARM64)
	local_daif_mask();
#else
	local_irq_disable();
#endif
}

static void trusty_irq_enable_after_smc(void)
{
#if IS_ENABLED(CONFIG_ARM64)
	local_daif_restore(DAIF_PROCCTX);
#else
	local_irq_enable();
#endif
}

static unsigned long trusty_std_call_inner(struct device *dev,
					   unsigned long smcnr,
					   unsigned long a0, unsigned long a1,
					   unsigned long a2)
{
	unsigned long ret;
	int retry = 5;

	while (true) {
		ret = trusty_smc(smcnr, a0, a1, a2);
		while ((s32)ret == SM_ERR_FIQ_INTERRUPTED)
			ret = trusty_smc(SMC_SC_RESTART_FIQ, 0, 0, 0);
		if ((int)ret != SM_ERR_BUSY || !retry)
			break;
		retry--;
	}

	return ret;
}

static unsigned long trusty_std_call_helper(struct device *dev,
					    unsigned long smcnr,
					    unsigned long a0, unsigned long a1,
					    unsigned long a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	unsigned long ret;
	int sleep_time = 1;

	while (true) {
		trusty_irq_disable_before_smc();
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_PREPARE,
					   NULL);
		ret = trusty_std_call_inner(dev, smcnr, a0, a1, a2);
		if (ret == SM_ERR_PANIC) {
			s->trusty_panicked = true;
			WARN_ONCE(1, "trusty crashed");
		}
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_RETURNED,
					   NULL);
		trusty_irq_enable_after_smc();

		if ((int)ret != SM_ERR_BUSY)
			break;

		msleep(sleep_time);
		if (sleep_time < 1000)
			sleep_time <<= 1;
	}

	return ret;
}

s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	int ret;

	if (WARN_ON(SMC_IS_FASTCALL(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;
	if (WARN_ON(SMC_IS_SMC64(smcnr)))
		return SM_ERR_INVALID_PARAMETERS;

	if (s->trusty_panicked)
		return SM_ERR_PANIC;

	if (smcnr != SMC_SC_NOP) {
		mutex_lock(&s->smc_lock);
		reinit_completion(&s->cpu_idle_completion);
	}

	ret = trusty_std_call_helper(dev, smcnr, a0, a1, a2);
	while (ret == SM_ERR_INTERRUPTED || ret == SM_ERR_CPU_IDLE) {
		if (ret == SM_ERR_CPU_IDLE)
			wait_for_completion_timeout(&s->cpu_idle_completion,
						    HZ * 10);
		ret = trusty_std_call_helper(dev, SMC_SC_RESTART_LAST, 0, 0, 0);
	}

	if (smcnr == SMC_SC_NOP)
		complete(&s->cpu_idle_completion);
	else
		mutex_unlock(&s->smc_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(trusty_std_call32);

u32 trusty_get_api_version(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return s->api_version;
}
EXPORT_SYMBOL_GPL(trusty_get_api_version);

const char *trusty_version_str_get(struct device *dev)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return s->version_str;
}
EXPORT_SYMBOL_GPL(trusty_version_str_get);

int trusty_call_notifier_register(struct device *dev, struct notifier_block *n)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return atomic_notifier_chain_register(&s->notifier, n);
}
EXPORT_SYMBOL_GPL(trusty_call_notifier_register);

int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return atomic_notifier_chain_unregister(&s->notifier, n);
}
EXPORT_SYMBOL_GPL(trusty_call_notifier_unregister);

/*
 * The nop mechanism.  When the secure world has pending work it cannot do on
 * its own (it has no timer tick in the normal world), a consumer enqueues a nop
 * and the nop thread enters Trusty with SMC_SC_NOP until it reports idle.  A
 * single thread is enough here; the SMP fan-out the vendor driver adds is a
 * throughput optimisation, not a correctness requirement.
 */
static bool trusty_dequeue_nop_locked(struct trusty_state *s, u32 *args)
{
	struct trusty_nop *nop;
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&s->nop_lock, flags);
	if (!list_empty(&s->nop_queue)) {
		nop = list_first_entry(&s->nop_queue, struct trusty_nop, node);
		list_del_init(&nop->node);
		args[0] = nop->args[0];
		args[1] = nop->args[1];
		args[2] = nop->args[2];
		ret = true;
	} else {
		args[0] = 0;
		args[1] = 0;
		args[2] = 0;
		ret = s->nop_signaled;
	}
	s->nop_signaled = false;
	spin_unlock_irqrestore(&s->nop_lock, flags);

	return ret;
}

static int trusty_nop_thread(void *data)
{
	struct trusty_state *s = data;
	u32 args[3];

	while (!kthread_should_stop()) {
		wait_event_interruptible(s->nop_event_wait,
					 s->nop_signaled ||
					 !list_empty_careful(&s->nop_queue) ||
					 kthread_should_stop());

		while (trusty_dequeue_nop_locked(s, args)) {
			int ret;

			if (kthread_should_stop())
				return 0;
			do {
				ret = trusty_std_call32(s->dev, SMC_SC_NOP,
							args[0], args[1],
							args[2]);
			} while (ret == SM_ERR_NOP_INTERRUPTED);
			if (ret != SM_ERR_NOP_DONE)
				dev_err(s->dev, "SMC_SC_NOP failed %d\n", ret);
		}
	}

	return 0;
}

void trusty_enqueue_nop(struct device *dev, struct trusty_nop *nop)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;

	spin_lock_irqsave(&s->nop_lock, flags);
	if (nop && list_empty(&nop->node))
		list_add_tail(&nop->node, &s->nop_queue);
	s->nop_signaled = true;
	spin_unlock_irqrestore(&s->nop_lock, flags);

	wake_up_interruptible(&s->nop_event_wait);
}
EXPORT_SYMBOL_GPL(trusty_enqueue_nop);

void trusty_dequeue_nop(struct device *dev, struct trusty_nop *nop)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;

	if (WARN_ON(!nop))
		return;

	spin_lock_irqsave(&s->nop_lock, flags);
	if (!list_empty(&nop->node))
		list_del_init(&nop->node);
	spin_unlock_irqrestore(&s->nop_lock, flags);
}
EXPORT_SYMBOL_GPL(trusty_dequeue_nop);

/*
 * Transfer a scatter-gather list of pages to the secure world via the FF-A
 * memory-object protocol (share, or lend if @lend).  Returns an FF-A handle in
 * @id that the secure world uses to retrieve and map the memory.  A single
 * endpoint (the secure OS) and a single memory-access descriptor are used,
 * which is all our callers need; the descriptor is sent from the mapped FF-A tx
 * buffer, fragmenting if it does not fit in one page.
 */
int trusty_transfer_memory(struct device *dev, u64 *id,
			   struct scatterlist *sglist, unsigned int nents,
			   pgprot_t pgprot, u64 tag, bool lend)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	struct ns_mem_page_info pg_inf;
	struct scatterlist *sg;
	size_t count, i, len = 0;
	u64 ffa_handle = 0;
	size_t total_len;
	size_t endpoint_count = 1;
	struct ffa_mtd *mtd = s->ffa_tx;
	size_t comp_mrd_offset = offsetof(struct ffa_mtd, emad[endpoint_count]);
	struct ffa_comp_mrd *comp_mrd = s->ffa_tx + comp_mrd_offset;
	struct ffa_cons_mrd *cons_mrd = comp_mrd->address_range_array;
	size_t cons_mrd_offset = (void *)cons_mrd - s->ffa_tx;
	struct arm_smccc_1_2_regs res;
	u32 cookie_low = 0, cookie_high = 0;
	int ret;

	if (WARN_ON(dev->driver != &trusty_driver.driver))
		return -EINVAL;
	if (WARN_ON(nents < 1))
		return -EINVAL;
	if (nents != 1 && s->api_version < TRUSTY_API_VERSION_MEM_OBJ) {
		dev_err(s->dev, "non-contiguous memory objects need a newer Trusty\n");
		return -EOPNOTSUPP;
	}

	count = dma_map_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);
	if (count != nents) {
		dev_err(s->dev, "failed to dma map sg_table\n");
		return -EINVAL;
	}

	sg = sglist;
	ret = trusty_encode_page_info(&pg_inf, phys_to_page(sg_dma_address(sg)),
				      pgprot);
	if (ret) {
		dev_err(s->dev, "trusty_encode_page_info failed\n");
		goto err_encode;
	}

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ) {
		*id = pg_inf.compat_attr;
		return 0;
	}

	for_each_sg(sglist, sg, nents, i)
		len += sg_dma_len(sg);

	mutex_lock(&s->share_memory_msg_lock);

	mtd->sender_id = s->ffa_local_id;
	mtd->memory_region_attributes = pg_inf.ffa_mem_attr;
	mtd->reserved_3 = 0;
	mtd->flags = 0;
	mtd->handle = 0;
	mtd->tag = tag;
	mtd->reserved_24_27 = 0;
	mtd->emad_count = endpoint_count;
	for (i = 0; i < endpoint_count; i++) {
		struct ffa_emad *emad = &mtd->emad[i];

		emad->mapd.endpoint_id = s->ffa_remote_id;
		emad->mapd.memory_access_permissions = pg_inf.ffa_mem_perm;
		emad->mapd.flags = 0;
		emad->comp_mrd_offset = comp_mrd_offset;
		emad->reserved_8_15 = 0;
	}
	comp_mrd->total_page_count = len / FFA_PAGE_SIZE;
	comp_mrd->address_range_count = nents;
	comp_mrd->reserved_8_15 = 0;

	total_len = cons_mrd_offset + nents * sizeof(*cons_mrd);
	sg = sglist;
	while (count) {
		size_t lcount = min_t(size_t, count,
				      (PAGE_SIZE - cons_mrd_offset) /
				      sizeof(*cons_mrd));
		size_t fragment_len = lcount * sizeof(*cons_mrd) + cons_mrd_offset;

		for (i = 0; i < lcount; i++) {
			cons_mrd[i].address = sg_dma_address(sg);
			cons_mrd[i].page_count = sg_dma_len(sg) / FFA_PAGE_SIZE;
			cons_mrd[i].reserved_12_15 = 0;
			sg = sg_next(sg);
		}
		count -= lcount;
		if (cons_mrd_offset) {
			u32 smcnr = lend ? SMC_FC_FFA_MEM_LEND :
					   SMC_FC_FFA_MEM_SHARE;

			res = trusty_smc8(smcnr, total_len, fragment_len,
					  0, 0, 0, 0, 0);
		} else {
			res = trusty_smc8(SMC_FC_FFA_MEM_FRAG_TX, cookie_low,
					  cookie_high, fragment_len, 0, 0, 0, 0);
		}

		if (res.a0 == SMC_FC_FFA_MEM_FRAG_RX) {
			cookie_low = res.a1;
			cookie_high = res.a2;
			if (!count) {
				dev_err(s->dev, "unexpected MEM_FRAG_RX after last fragment\n");
				ret = -EIO;
				break;
			}
		} else if (res.a0 == SMC_FC_FFA_SUCCESS) {
			ffa_handle = res.a2 | (u64)res.a3 << 32;
			if (count) {
				dev_err(s->dev, "unexpected SUCCESS before last fragment\n");
				ret = -EIO;
				break;
			}
		} else {
			dev_err(s->dev, "FFA_MEM_SHARE failed 0x%lx 0x%lx 0x%lx\n",
				res.a0, res.a1, res.a2);
			ret = -EIO;
			break;
		}

		cons_mrd = s->ffa_tx;
		cons_mrd_offset = 0;
	}

	mutex_unlock(&s->share_memory_msg_lock);

	if (!ret) {
		*id = ffa_handle;
		return 0;
	}

err_encode:
	dma_unmap_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);
	return ret;
}
EXPORT_SYMBOL_GPL(trusty_transfer_memory);

int trusty_share_memory(struct device *dev, trusty_shared_mem_id_t *id,
			struct scatterlist *sglist, unsigned int nents,
			pgprot_t pgprot)
{
	return trusty_transfer_memory(dev, id, sglist, nents, pgprot, 0, false);
}
EXPORT_SYMBOL_GPL(trusty_share_memory);

/*
 * Like trusty_share_memory(), but masks off the memory attributes for old APIs
 * that expected a bare physical address as the id.
 */
int trusty_share_memory_compat(struct device *dev, trusty_shared_mem_id_t *id,
			       struct scatterlist *sglist, unsigned int nents,
			       pgprot_t pgprot)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	int ret;

	ret = trusty_share_memory(dev, id, sglist, nents, pgprot);
	if (!ret && s->api_version < TRUSTY_API_VERSION_PHYS_MEM_OBJ)
		*id &= 0x0000FFFFFFFFF000ull;

	return ret;
}
EXPORT_SYMBOL_GPL(trusty_share_memory_compat);

int trusty_reclaim_memory(struct device *dev, trusty_shared_mem_id_t id,
			  struct scatterlist *sglist, unsigned int nents)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	struct arm_smccc_1_2_regs res;
	int ret = 0;

	if (WARN_ON(dev->driver != &trusty_driver.driver))
		return -EINVAL;
	if (WARN_ON(nents < 1))
		return -EINVAL;

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ) {
		if (nents != 1) {
			dev_err(s->dev, "non-contiguous reclaim needs a newer Trusty\n");
			return -EOPNOTSUPP;
		}
		dma_unmap_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);
		return 0;
	}

	mutex_lock(&s->share_memory_msg_lock);
	res = trusty_smc8(SMC_FC_FFA_MEM_RECLAIM, (u32)id, id >> 32, 0, 0,
			  0, 0, 0);
	if (res.a0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "FFA_MEM_RECLAIM failed 0x%lx 0x%lx 0x%lx\n",
			res.a0, res.a1, res.a2);
		if (res.a0 == SMC_FC_FFA_ERROR && res.a2 == FFA_ERROR_DENIED)
			ret = -EBUSY;
		else
			ret = -EIO;
	}
	mutex_unlock(&s->share_memory_msg_lock);

	if (ret)
		return ret;

	dma_unmap_sg(dev, sglist, nents, DMA_BIDIRECTIONAL);
	return 0;
}
EXPORT_SYMBOL_GPL(trusty_reclaim_memory);

/*
 * The API version we negotiate with Trusty.  This build refuses anything below
 * MEM_OBJ (5), so memory sharing goes through the FF-A memory-object protocol
 * (see trusty-mem.c and trusty_share_memory()).
 */
#define TRUSTY_API_VERSION_REQUESTED	TRUSTY_API_VERSION_CURRENT

static int trusty_init_api_version(struct trusty_state *s, struct device *dev)
{
	u32 api_version;

	api_version = trusty_fast_call32(dev, SMC_FC_API_VERSION,
					 TRUSTY_API_VERSION_REQUESTED, 0, 0);
	if (api_version == SM_ERR_UNDEFINED_SMC)
		api_version = 0;

	if (api_version > TRUSTY_API_VERSION_REQUESTED) {
		dev_err(dev, "unsupported Trusty api version %u > %u\n",
			api_version, TRUSTY_API_VERSION_REQUESTED);
		return -EINVAL;
	}

	dev_info(dev, "selected Trusty api version: %u (requested %u)\n",
		 api_version, TRUSTY_API_VERSION_REQUESTED);
	s->api_version = api_version;
	return 0;
}

/*
 * Read the Trusty build version string, one character per fast call.  A failure
 * here is how we detect that no Trusty secure OS is actually present behind the
 * SMC interface.
 */
static int trusty_init_version_str(struct trusty_state *s, struct device *dev)
{
	char *str;
	int len, i, ret;

	ret = trusty_fast_call32(dev, SMC_FC_GET_VERSION_STR, -1, 0, 0);
	if (ret <= 0) {
		dev_err(dev, "Trusty not detected (version query returned %d)\n",
			ret);
		return -ENODEV;
	}
	len = ret;

	str = kmalloc(len + 1, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	for (i = 0; i < len; i++) {
		ret = trusty_fast_call32(dev, SMC_FC_GET_VERSION_STR, i, 0, 0);
		if (ret < 0)
			goto err;
		str[i] = ret;
	}
	str[i] = '\0';

	s->version_str = str;
	dev_info(dev, "Trusty version: %s\n", str);
	return 0;

err:
	kfree(str);
	return ret;
}

/*
 * Set up FF-A for memory sharing: negotiate the FF-A version, confirm MEM_SHARE
 * is available, learn our endpoint id, and map the tx/rx message buffers.  Only
 * needed once the API version reaches MEM_OBJ.
 */
static int trusty_init_msg_buf(struct trusty_state *s, struct device *dev)
{
	phys_addr_t tx_paddr, rx_paddr;
	struct arm_smccc_1_2_regs res;
	int ret;

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ)
		return 0;

	res = trusty_smc8(SMC_FC_FFA_VERSION, FFA_CURRENT_VERSION, 0, 0,
			  0, 0, 0, 0);
	if (FFA_VERSION_TO_MAJOR(res.a0) != FFA_CURRENT_VERSION_MAJOR) {
		dev_err(dev, "unsupported FF-A version 0x%lx, expected major %u\n",
			res.a0, FFA_CURRENT_VERSION_MAJOR);
		return -EIO;
	}

	res = trusty_smc8(SMC_FC_FFA_FEATURES, SMC_FC_FFA_MEM_SHARE, 0, 0,
			  0, 0, 0, 0);
	if (res.a0 != SMC_FC_FFA_SUCCESS) {
		dev_err(dev, "FF-A does not implement MEM_SHARE (0x%lx)\n", res.a0);
		return -EIO;
	}

	res = trusty_smc8(SMC_FC_FFA_ID_GET, 0, 0, 0, 0, 0, 0, 0);
	if (res.a0 != SMC_FC_FFA_SUCCESS) {
		dev_err(dev, "FFA_ID_GET failed (0x%lx)\n", res.a0);
		return -EIO;
	}
	s->ffa_local_id = res.a2;
	s->ffa_remote_id = 0x8000;	/* the secure OS endpoint */

	s->ffa_tx = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s->ffa_tx)
		return -ENOMEM;
	tx_paddr = virt_to_phys(s->ffa_tx);
	if (WARN_ON(tx_paddr & (PAGE_SIZE - 1))) {
		ret = -EINVAL;
		goto err_tx;
	}

	s->ffa_rx = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s->ffa_rx) {
		ret = -ENOMEM;
		goto err_tx;
	}
	rx_paddr = virt_to_phys(s->ffa_rx);
	if (WARN_ON(rx_paddr & (PAGE_SIZE - 1))) {
		ret = -EINVAL;
		goto err_rx;
	}

	res = trusty_smc8(SMC_FCZ_FFA_RXTX_MAP, tx_paddr, rx_paddr,
			  PAGE_SIZE / FFA_PAGE_SIZE, 0, 0, 0, 0);
	if (res.a0 != SMC_FC_FFA_SUCCESS) {
		dev_err(dev, "FFA_RXTX_MAP failed (0x%lx)\n", res.a0);
		ret = -EIO;
		goto err_rx;
	}

	return 0;

err_rx:
	kfree(s->ffa_rx);
	s->ffa_rx = NULL;
err_tx:
	kfree(s->ffa_tx);
	s->ffa_tx = NULL;
	return ret;
}

static void trusty_free_msg_buf(struct trusty_state *s, struct device *dev)
{
	struct arm_smccc_1_2_regs res;

	if (s->api_version < TRUSTY_API_VERSION_MEM_OBJ)
		return;

	res = trusty_smc8(SMC_FC_FFA_RXTX_UNMAP, 0, 0, 0, 0, 0, 0, 0);
	if (res.a0 != SMC_FC_FFA_SUCCESS)
		dev_err(dev, "FFA_RXTX_UNMAP failed (0x%lx)\n", res.a0);
	kfree(s->ffa_rx);
	kfree(s->ffa_tx);
}

static int trusty_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct trusty_state *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	mutex_init(&s->smc_lock);
	mutex_init(&s->share_memory_msg_lock);
	init_completion(&s->cpu_idle_completion);
	ATOMIC_INIT_NOTIFIER_HEAD(&s->notifier);
	init_waitqueue_head(&s->nop_event_wait);
	INIT_LIST_HEAD(&s->nop_queue);
	spin_lock_init(&s->nop_lock);
	platform_set_drvdata(pdev, s);

	/*
	 * Memory shared with the secure world is dma-mapped on this device; give
	 * it a wide mask so the FF-A descriptors carry the real physical address
	 * rather than a bounced one the secure world cannot follow.
	 */
	s->dev->dma_parms = &s->dma_parms;
	dma_set_max_seg_size(s->dev, 0xfffff000);
	ret = dma_coerce_mask_and_coherent(s->dev, DMA_BIT_MASK(48));
	if (ret)
		return ret;

	ret = trusty_init_api_version(s, dev);
	if (ret)
		return ret;

	ret = trusty_init_version_str(s, dev);
	if (ret)
		return ret;

	ret = trusty_init_msg_buf(s, dev);
	if (ret)
		goto err_free_version;

	s->nop_thread = kthread_run(trusty_nop_thread, s, "trusty-nop");
	if (IS_ERR(s->nop_thread)) {
		ret = PTR_ERR(s->nop_thread);
		goto err_free_msg_buf;
	}

	/* Bring up the child nodes (irq, log, virtio) once they exist. */
	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret)
		dev_warn(dev, "failed to add Trusty child devices: %d\n", ret);

	return 0;

err_free_msg_buf:
	trusty_free_msg_buf(s, dev);
err_free_version:
	kfree(s->version_str);
	return ret;
}

static void trusty_remove(struct platform_device *pdev)
{
	struct trusty_state *s = platform_get_drvdata(pdev);

	of_platform_depopulate(&pdev->dev);
	kthread_stop(s->nop_thread);
	trusty_free_msg_buf(s, &pdev->dev);
	s->dev->dma_parms = NULL;
	mutex_destroy(&s->share_memory_msg_lock);
	mutex_destroy(&s->smc_lock);
	kfree(s->version_str);
}

static const struct of_device_id trusty_of_match[] = {
	{ .compatible = "android,trusty-smc-v1", },
	{ }
};
MODULE_DEVICE_TABLE(of, trusty_of_match);

static struct platform_driver trusty_driver = {
	.probe = trusty_probe,
	.remove = trusty_remove,
	.driver = {
		.name = "trusty",
		.of_match_table = trusty_of_match,
	},
};
module_platform_driver(trusty_driver);

MODULE_DESCRIPTION("Trusty TEE core (SMC interface)");
MODULE_LICENSE("GPL");
