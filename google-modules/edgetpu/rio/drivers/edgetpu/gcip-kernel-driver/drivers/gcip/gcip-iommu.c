// SPDX-License-Identifier: GPL-2.0-only
/*
 * Manages GCIP IOMMU domains and allocates/maps IOVAs.
 *
 * Copyright (C) 2023-2025 Google LLC
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/dma-resv.h>
#include <linux/genalloc.h>
#include <linux/iova.h>
#include <linux/limits.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/scatterlist.h>
#include <linux/sched/mm.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/swap.h>

#include <gcip/gcip-config.h>
#include <gcip/gcip-domain-pool.h>
#include <gcip/gcip-iommu.h>
#include <gcip/gcip-mem-pool.h>

/* Restricted IOVA ceiling is for components with 32-bit DMA windows */
#define GCIP_RESTRICT_IOVA_CEILING UINT_MAX

#define to_buffer_mapping(mapping) container_of(mapping, struct gcip_iommu_buffer_mapping, mapping)
#define to_dmabuf_mapping(mapping) container_of(mapping, struct gcip_iommu_dmabuf_mapping, mapping)

/**
 * struct gcip_iommu_buffer_mapping - Contains the information about buffer mapping.
 * @mapping: Stores the mapping information to the IOMMU domain.
 * @host_address: Start address of buffer in the virtual address space of the mapping process.
 * @owning_mm: The mm_struct to maintain pinned_vm.
 * @sync_lock: The mutex lock to make sure the mapping only be synced by one thread at a time.
 */
struct gcip_iommu_buffer_mapping {
	struct gcip_iommu_mapping mapping;
	u64 host_address;
	struct mm_struct *owning_mm;
	struct mutex sync_lock;
};

/* Contains the information about dma-buf mapping. */
struct gcip_iommu_dmabuf_mapping {
	/* Stores the mapping information to the IOMMU domain. */
	struct gcip_iommu_mapping mapping;

	/* Following fields store the mapping information to the default domain. */

	/* Scatter-gather table which contains the mapping information. */
	struct sg_table *sgt_default;
	/* Shared dma-buf object. */
	struct dma_buf *dma_buf;
	/* Device attachment of dma-buf. */
	struct dma_buf_attachment *dma_buf_attachment;
};

/**
 * dma_info_to_prot - Translate DMA API directions and attributes to IOMMU API
 *                    page flags.
 * @dir: Direction of DMA transfer
 * @coherent: If true, create coherent mappings of the scatterlist.
 * @attrs: DMA attributes for the mapping
 *
 * See v5.15.94/source/drivers/iommu/dma-iommu.c#L418
 *
 * Return: corresponding IOMMU API page protection flags
 */
static int dma_info_to_prot(enum dma_data_direction dir, bool coherent, unsigned long attrs)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	if (attrs & DMA_ATTR_PRIVILEGED)
		prot |= IOMMU_PRIV;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

/*
 * Allocates an IOVA for the scatterlist and maps it to @domain.
 *
 * @domain: GCIP IOMMU domain which manages IOVA addresses.
 * @sgl: Scatterlist to be mapped.
 * @nents: The number of entries in @sgl.
 * @iova: Target IOVA to map @sgl. If it is 0, this function allocates an IOVA space.
 * @gcip_map_flags: Flags indicating mapping attributes, which should be encoded with
 *                  gcip_iommu_encode_gcip_map_flags().
 *
 * Returns the number of entries which are mapped to @domain. Returns 0 if it fails.
 */
static unsigned int gcip_iommu_domain_map_sg(struct gcip_iommu_domain *domain,
					     struct scatterlist *sgl, int nents, dma_addr_t iova,
					     u64 gcip_map_flags)
{
	enum dma_data_direction dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);
	bool coherent = GCIP_MAP_FLAGS_GET_DMA_COHERENT(gcip_map_flags);
	unsigned long attrs = GCIP_MAP_FLAGS_GET_DMA_ATTR(gcip_map_flags);
	int i, prot = dma_info_to_prot(dir, coherent, attrs);
	struct scatterlist *sg;
	size_t iova_len = 0;
	ssize_t map_size;
	int ret;
	bool allocated = false;

	/* Calculates how much IOVA space we need. */
	for_each_sg(sgl, sg, nents, i)
		iova_len += sg->length;

	if (!iova) {
		/* Allocates one continuous IOVA. */
		iova = gcip_iommu_alloc_iova(domain, iova_len, gcip_map_flags);
		if (!iova)
			return 0;
		allocated = true;
	}

	/*
	 * Maps scatterlist to the allocated IOVA.
	 *
	 * It will iterate each scatter list segment in order and map them to the IOMMU domain
	 * as amount of the size of each segment successively.
	 * Returns an error on failure or the total length of mapped segments on success.
	 */
#if GCIP_IOMMU_MAP_HAS_GFP
	map_size = iommu_map_sg(domain->domain, iova, sgl, nents, prot, GFP_KERNEL);
#else
	map_size = iommu_map_sg(domain->domain, iova, sgl, nents, prot);
#endif
	if (map_size < 0 || map_size < iova_len)
		goto err_free_iova;

	/*
	 * Fills out the mapping information. Each entry can be max UINT_MAX bytes, floored
	 * to the pool granule size.
	 */
	ret = 0;
	sg = sgl;
	while (iova_len) {
		size_t segment_len = min_t(size_t, iova_len,
					   UINT_MAX & ~(gcip_iommu_domain_granule(domain) - 1));

		sg_dma_address(sg) = iova;
		sg_dma_len(sg) = segment_len;
		iova += segment_len;
		iova_len -= segment_len;
		ret++;
		sg = sg_next(sg);
	}

	/* Return # of sg entries filled out above. */
	return ret;

err_free_iova:
	if (allocated)
		gcip_iommu_free_iova(domain, iova, iova_len);
	return 0;
}

/*
 * Unmaps an IOVA which was mapped for the scatterlist.
 *
 * @domain: GCIP IOMMU domain which manages IOVA addresses.
 * @sgl: Scatterlist to be unmapped.
 * @nents: The number of sg elements.
 * @free_iova: Set to true if the IOVA space was allocated internally while mapping @sgl by the
 *             `gcip_iommu_domain_map_sg` function. (i.e., @iova argument of the function was 0.)
 */
static void gcip_iommu_domain_unmap_sg(struct gcip_iommu_domain *domain, struct scatterlist *sgl,
				       int nents, bool free_iova)
{
	dma_addr_t iova = sg_dma_address(sgl);
	size_t iova_len = 0;
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nents, i) {
		uint s_len = sg_dma_len(sg);

		if (!s_len)
			break;
		iova_len += s_len;
	}

	iommu_unmap(domain->domain, iova, iova_len);
	if (free_iova)
		gcip_iommu_free_iova(domain, iova, iova_len);
}

static inline unsigned long gcip_iommu_domain_shift(struct gcip_iommu_domain *domain)
{
	return __ffs(gcip_iommu_domain_granule(domain));
}

static inline unsigned long gcip_iommu_domain_pfn(struct gcip_iommu_domain *domain, dma_addr_t iova)
{
	return iova >> gcip_iommu_domain_shift(domain);
}

static inline size_t gcip_iommu_domain_align(struct gcip_iommu_domain *domain, size_t size)
{
	return ALIGN(size, gcip_iommu_domain_granule(domain));
}

static int iovad_initialize_domain(struct gcip_iommu_domain *domain)
{
	struct gcip_iommu_domain_pool *dpool = domain->domain_pool;
	int ret;

	init_iova_domain(&domain->iova_space.iovad, dpool->granule,
			 max_t(unsigned long, 1, dpool->base_daddr >> ilog2(dpool->granule)));

	if (dpool->reserved_size) {
		unsigned long shift = gcip_iommu_domain_shift(domain);
		unsigned long pfn_lo = dpool->reserved_base_daddr >> shift;
		unsigned long pfn_hi = (dpool->reserved_base_daddr + dpool->reserved_size) >> shift;

		reserve_iova(&domain->iova_space.iovad, pfn_lo, pfn_hi);
	}

	ret = iova_domain_init_rcaches(&domain->iova_space.iovad);
	if (ret)
		put_iova_domain(&domain->iova_space.iovad);

	return ret;
}

static void iovad_finalize_domain(struct gcip_iommu_domain *domain)
{
	put_iova_domain(&domain->iova_space.iovad);
}

static void iovad_enable_best_fit_algo(struct gcip_iommu_domain *domain)
{
}

static dma_addr_t iovad_alloc_iova_space(struct gcip_iommu_domain *domain, size_t size,
					 bool restrict_iova)
{
	unsigned long iova_pfn, shift = gcip_iommu_domain_shift(domain);
	dma_addr_t iova_ceiling = restrict_iova ? min_t(dma_addr_t, GCIP_RESTRICT_IOVA_CEILING,
							domain->domain_pool->last_daddr) :
						  domain->domain_pool->last_daddr;

	size = size >> shift;
	iova_pfn = alloc_iova_fast(&domain->iova_space.iovad, size, iova_ceiling >> shift, true);
	return (dma_addr_t)iova_pfn << shift;
}

static void iovad_free_iova_space(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size)
{
	free_iova_fast(&domain->iova_space.iovad, gcip_iommu_domain_pfn(domain, iova),
		       size >> gcip_iommu_domain_shift(domain));
}

static const struct gcip_iommu_domain_ops iovad_ops = {
	.initialize_domain = iovad_initialize_domain,
	.finalize_domain = iovad_finalize_domain,
	.enable_best_fit_algo = iovad_enable_best_fit_algo,
	.alloc_iova_space = iovad_alloc_iova_space,
	.free_iova_space = iovad_free_iova_space,
};

static int mem_pool_initialize_domain(struct gcip_iommu_domain *domain)
{
	struct gcip_iommu_domain_pool *dpool = domain->domain_pool;
	size_t size = dpool->size;
	int ret;

	/*
	 * Use separate gen_pools for 32-bit vs. unrestricted IOVAs.  Must have a non-empty 32-bit
	 * space.
	 */
	if (dpool->base_daddr > UINT_MAX)
		return -EINVAL;
	if (dpool->base_daddr + size + 1 > UINT_MAX) {
		size = dpool->size - ((unsigned long long)UINT_MAX - dpool->base_daddr + 1);
		ret = gcip_mem_pool_init(&domain->iova_space.mem_pool.pool64, dpool->dev,
					 (unsigned long long)UINT_MAX + 1, size, dpool->granule);
		if (ret)
			return ret;

		domain->iova_space.mem_pool.pool64_valid = true;
		size = UINT_MAX - dpool->base_daddr + 1;
	}
	ret = gcip_mem_pool_init(&domain->iova_space.mem_pool.pool32, dpool->dev,
				 dpool->base_daddr, size, dpool->granule);
	if (ret) {
		if (domain->iova_space.mem_pool.pool64_valid)
			gcip_mem_pool_exit(&domain->iova_space.mem_pool.pool64);
		return ret;
	}

	if (dpool->reserved_size)
		dev_warn(domain->dev, "gcip-reserved-map is not supported in mem_pool mode.");

	return 0;
}

static void mem_pool_finalize_domain(struct gcip_iommu_domain *domain)
{
	gcip_mem_pool_exit(&domain->iova_space.mem_pool.pool32);
	if (domain->iova_space.mem_pool.pool64_valid)
		gcip_mem_pool_exit(&domain->iova_space.mem_pool.pool64);
}

static void mem_pool_enable_best_fit_algo(struct gcip_iommu_domain *domain)
{
	gen_pool_set_algo(domain->iova_space.mem_pool.pool32.gen_pool, gen_pool_best_fit, NULL);
	if (domain->iova_space.mem_pool.pool64_valid)
		gen_pool_set_algo(domain->iova_space.mem_pool.pool64.gen_pool, gen_pool_best_fit,
				  NULL);
}

static dma_addr_t mem_pool_alloc_iova_space(struct gcip_iommu_domain *domain, size_t size,
					    bool restrict_iova)
{
	if (restrict_iova || !domain->iova_space.mem_pool.pool64_valid)
		return (dma_addr_t)gcip_mem_pool_alloc(&domain->iova_space.mem_pool.pool32, size);
	return (dma_addr_t)gcip_mem_pool_alloc(&domain->iova_space.mem_pool.pool64, size);
}


static void mem_pool_free_iova_space(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size)
{
	if (iova <= UINT_MAX)
		gcip_mem_pool_free(&domain->iova_space.mem_pool.pool32, iova, size);
	else
		gcip_mem_pool_free(&domain->iova_space.mem_pool.pool64, iova, size);
}

static const struct gcip_iommu_domain_ops mem_pool_ops = {
	.initialize_domain = mem_pool_initialize_domain,
	.finalize_domain = mem_pool_finalize_domain,
	.enable_best_fit_algo = mem_pool_enable_best_fit_algo,
	.alloc_iova_space = mem_pool_alloc_iova_space,
	.free_iova_space = mem_pool_free_iova_space,
};

/**
 * get_window_config() - Retrieve base address and size from device tree.
 * @dev: The device struct to get the device tree.
 * @name: The name of the target window.
 * @n_addr: The required number of cells to read the value of @addr.
 * @n_size: The required number of cells to read the value of @size.
 * @addr: The pointer of the base address to output the value. Set to 0 on failure.
 * @size: The pointer of the size to output the value. Set to 0 on failure.
 *
 * Return: 0 on success, negative errno on failure.
 */
static int get_window_config(struct device *dev, char *name, int n_addr, int n_size,
			     dma_addr_t *addr, size_t *size)
{
	const __be32 *window;

	window = of_get_property(dev->of_node, name, NULL);
	if (!window) {
		*addr = *size = 0;
		return -ENODATA;
	}

	*addr = of_read_number(window, n_addr);
	*size = of_read_number(window + n_addr, n_size);

	return 0;
}

/**
 * copy_alloc_sg_table(): Allocates a new sgt and copies the data from the old one.
 * @sgt_src: The source sg_table whose data will be copied to the new one.
 *
 * We will only copy the page information to the new sg_table, so the new sg_table will have the
 * same orig_nents and page information as the old one.
 *
 * Return: The new allocated sg_table with data copied from sgt_src or an error pointer on failure.
 */
static struct sg_table *copy_alloc_sg_table(struct sg_table *sgt_src)
{
	struct sg_table *sgt_dst;
	struct scatterlist *sgl_src, *sgl_dst;
	int ret, i;

	sgt_dst = kzalloc(sizeof(*sgt_dst), GFP_KERNEL);
	if (!sgt_dst) {
		ret = -ENOMEM;
		goto err_alloc_sgt;
	}

	ret = sg_alloc_table(sgt_dst, sgt_src->orig_nents, GFP_KERNEL);
	if (ret)
		goto err_alloc_sgl;

	sgl_dst = sgt_dst->sgl;
	for_each_sg(sgt_src->sgl, sgl_src, sgt_src->orig_nents, i) {
		sg_set_page(sgl_dst, sg_page(sgl_src), sgl_src->length, 0);
		sgl_dst = sg_next(sgl_dst);
	}

	return sgt_dst;

err_alloc_sgl:
	kfree(sgt_dst);
err_alloc_sgt:
	return ERR_PTR(ret);
}

static inline void sync_sg_if_needed(struct device *dev, struct sg_table *sgt, u64 gcip_map_flags,
				     bool for_device)
{
	enum dma_data_direction dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);

	if (GCIP_MAP_FLAGS_GET_DMA_ATTR(gcip_map_flags) & DMA_ATTR_SKIP_CPU_SYNC)
		return;

	if (for_device)
		dma_sync_sg_for_device(dev, sgt->sgl, sgt->orig_nents, dir);
	else
		dma_sync_sg_for_cpu(dev, sgt->sgl, sgt->orig_nents, dir);
}

/* Maps @sgt to @iova. If @iova is 0, this function allocates an IOVA space internally. */
unsigned int gcip_iommu_domain_map_sgt_to_iova(struct gcip_iommu_domain *domain,
					       struct sg_table *sgt, dma_addr_t iova,
					       u64 *gcip_map_flags)
{
	struct scatterlist *sgl = sgt->sgl;
	uint orig_nents = sgt->orig_nents;
	enum dma_data_direction dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(*gcip_map_flags);
	uint nents_mapped;

	if (dir != DMA_BIDIRECTIONAL && dir != DMA_TO_DEVICE) {
		dev_err(domain->dev, "Invalid DMA direction: %d", dir);
		return -EINVAL;
	}

	nents_mapped = gcip_iommu_domain_map_sg(domain, sgl, orig_nents, iova, *gcip_map_flags);

	sgt->nents = nents_mapped;

	sync_sg_if_needed(domain->dev, sgt, *gcip_map_flags, true);

	return nents_mapped;
}

unsigned int gcip_iommu_domain_map_sgt(struct gcip_iommu_domain *domain, struct sg_table *sgt,
				       u64 *gcip_map_flags)
{
	return gcip_iommu_domain_map_sgt_to_iova(domain, sgt, 0, gcip_map_flags);
}

/*
 * Unmaps @sgt from @domain. If @free_iova is true, the IOVA region which was allocated by the
 * `gcip_iommu_domain_map_sgt_to_iova` function will be freed.
 */
static void gcip_iommu_domain_unmap_sgt_free_iova(struct gcip_iommu_domain *domain,
						  struct sg_table *sgt, bool free_iova,
						  u64 gcip_map_flags)
{
	sync_sg_if_needed(domain->dev, sgt, gcip_map_flags, false);
	gcip_iommu_domain_unmap_sg(domain, sgt->sgl, sgt->orig_nents, free_iova);
}

void gcip_iommu_domain_unmap_sgt(struct gcip_iommu_domain *domain, struct sg_table *sgt,
				 u64 gcip_map_flags)
{
	return gcip_iommu_domain_unmap_sgt_free_iova(domain, sgt, true, gcip_map_flags);
}

void gcip_iommu_domain_unmap_sgt_from_iova(struct gcip_iommu_domain *domain, struct sg_table *sgt,
					   u64 gcip_map_flags)
{
	gcip_iommu_domain_unmap_sgt_free_iova(domain, sgt, false, gcip_map_flags);
}

/**
 * gcip_iommu_dmabuf_sgt_destroy() - Reverts gcip_iommu_dmabuf_sgt_create().
 * @sgt_default: The sg_table to unmap.
 * @dmabuf: The dma_buf to detach.
 * @attachment: The dma_buf_attachment to unmap and detach.
 * @dir: The DMA direction of the mapping.
 */
static void gcip_iommu_dmabuf_sgt_destroy(struct sg_table *sgt_default, struct dma_buf *dmabuf,
					  struct dma_buf_attachment *attachment,
					  enum dma_data_direction dir)
{
	dma_buf_unmap_attachment(attachment, sgt_default, dir);

	dma_buf_detach(dmabuf, attachment);
}

/**
 * gcip_iommu_mapping_unmap_dma_buf() - Unmaps the dma buf mapping.
 * @mapping: The pointer of the mapping instance to be unmapped.
 *
 * Reverting gcip_iommu_domain_map_dma_buf()
 */
static void gcip_iommu_mapping_unmap_dma_buf(struct gcip_iommu_mapping *mapping)
{
	struct gcip_iommu_dmabuf_mapping *dmabuf_mapping = to_dmabuf_mapping(mapping);

	if (!mapping->domain->default_domain) {
		gcip_iommu_domain_unmap_sgt_free_iova(mapping->domain, mapping->sgt,
						      !mapping->user_specified_daddr,
						      mapping->gcip_map_flags);
		sg_free_table(mapping->sgt);
		kfree(mapping->sgt);
	}

	gcip_iommu_dmabuf_sgt_destroy(dmabuf_mapping->sgt_default, dmabuf_mapping->dma_buf,
				      dmabuf_mapping->dma_buf_attachment, mapping->dir);
	dma_buf_put(dmabuf_mapping->dma_buf);
	kfree(dmabuf_mapping);
}

/**
 * gcip_iommu_buffer_sgt_destroy() - Reverts gcip_iommu_buffer_sgt_create().
 * @sgt: The scatter-gather table to destroy.
 * @dir: The DMA direction of the mapping.
 * @mm: The mm_struct to maintain pinned_vm.
 *
 * If the @sgt has never been mapped, pass DMA_NONE for @dir to skip set_page_dirty().
 */
static void gcip_iommu_buffer_sgt_destroy(struct sg_table *sgt, enum dma_data_direction dir,
					  struct mm_struct *mm)
{
	struct sg_page_iter sg_iter;
	struct page *page;
	unsigned long num_pages = 0;

	for_each_sg_page(sgt->sgl, &sg_iter, sgt->orig_nents, 0) {
		page = sg_page_iter_page(&sg_iter);
		if (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL)
			set_page_dirty(page);
		unpin_user_page(page);
		num_pages++;
	}

	atomic64_sub(num_pages, &mm->pinned_vm);
	sg_free_table(sgt);
	kfree(sgt);
}

/*
 * For buffer unmap and trim operations, sync, unpin, and free the sgt.
 */
static void gcip_iommu_mapping_buffer_flush_sgt(struct gcip_iommu_buffer_mapping *buffer_mapping)
{
	struct gcip_iommu_mapping *mapping = &buffer_mapping->mapping;
	enum dma_data_direction dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(mapping->gcip_map_flags);

	if (!mapping->sgt)
		return;

	gcip_iommu_domain_unmap_sgt_free_iova(mapping->domain, mapping->sgt, false,
					      mapping->gcip_map_flags);
	gcip_iommu_buffer_sgt_destroy(mapping->sgt, dir, buffer_mapping->owning_mm);
	mapping->sgt = NULL;
}

/**
 * gcip_iommu_mapping_unmap_buffer() - Revert gcip_iommu_domain_map_buffer
 * @mapping: The target mapping that should be unmapped.
 */
static void gcip_iommu_mapping_unmap_buffer(struct gcip_iommu_mapping *mapping)
{
	struct gcip_iommu_buffer_mapping *buffer_mapping = to_buffer_mapping(mapping);

	gcip_iommu_mapping_buffer_flush_sgt(buffer_mapping);

	if (!mapping->user_specified_daddr)
		gcip_iommu_free_iova(mapping->domain, mapping->alloced_iova, mapping->size);

	mmdrop(buffer_mapping->owning_mm);
	mutex_destroy(&buffer_mapping->sync_lock);
	kfree(buffer_mapping);
}

/**
 * gcip_pin_user_pages_fast() - Tries pin_user_pages_fast and returns success only if all pages are
 *                              pinned.
 * @pages: The allocated pages to be pinned.
 * @start_addr: The starting user address, must be page-aligned.
 * @num_pages: Same as gcip_iommu_alloc_and_pin_user_pages.
 * @gup_flags: The gup_flags used to pin user pages.
 * @pin_user_pages_lock: Same as gcip_iommu_alloc_and_pin_user_pages.
 *
 * The function will try pin_user_pages_fast.
 * If its return value equals @num_pages, returns @num_pages.
 * If only partial pages are pinned, unpins all pages and return 0.
 * Returns the error code otherwise.
 */
static int gcip_pin_user_pages_fast(struct page **pages, unsigned long start_addr, uint num_pages,
				    unsigned int gup_flags, struct mutex *pin_user_pages_lock)
{
	int ret, i;

	/*
	 * Provide protection around `pin_user_pages_fast` since it fails if called by more than one
	 * thread simultaneously.
	 */
	if (pin_user_pages_lock)
		mutex_lock(pin_user_pages_lock);

	ret = pin_user_pages_fast(start_addr, num_pages, gup_flags, pages);

	if (pin_user_pages_lock)
		mutex_unlock(pin_user_pages_lock);

	if (ret < num_pages) {
		for (i = 0; i < ret; i++)
			unpin_user_page(pages[i]);
		ret = 0;
	}

	return ret;
}

/**
 * gcip_pin_user_pages() - Try pin_user_pages_fast and try again with pin_user_pages if failed.
 * @dev: device for which the pages are being pinned, for logs.
 * @pages: The allocated pages to be pinned.
 * @start_addr: The starting user address, must be page-aligned.
 * @num_pages: Same as gcip_iommu_alloc_and_pin_user_pages.
 * @gup_flags: The gup_flags used to pin user pages.
 * @pin_user_pages_lock: Same as gcip_iommu_alloc_and_pin_user_pages.
 *
 * The return value and the partial pinned cases is handled the same as @gcip_pin_user_pages_fast.
 */
static int gcip_pin_user_pages(struct device *dev, struct page **pages, unsigned long start_addr,
			       uint num_pages, unsigned int gup_flags,
			       struct mutex *pin_user_pages_lock)
{
	int ret, i;
	int tried;

	ret = gcip_pin_user_pages_fast(pages, start_addr, num_pages, gup_flags,
				       pin_user_pages_lock);
	if (ret == num_pages)
		return ret;

	dev_dbg(dev, "Failed to pin user pages in fast mode (ret=%d, addr=%lu, num_pages=%d)", ret,
		start_addr, num_pages);

	/*
	 * pin_user_pages may fail due to temporary page reference counts held
	 * in various areas. Retry under lru_cache_disable to release additional
	 * reference counts from the LRU cache.
	 */
	for (tried = 0; tried < 5; tried++) {
		if (tried > 0) {
			dev_info(dev, "Retrying mapping with LRU cache disabled (tries=%d)", tried);
			lru_cache_disable();
		}
		if (pin_user_pages_lock)
			mutex_lock(pin_user_pages_lock);
		mmap_read_lock(current->mm);

		/* NOTE(mainline): pin_user_pages() dropped the vmas out-param. */
		ret = pin_user_pages(start_addr, num_pages, gup_flags, pages);

		mmap_read_unlock(current->mm);
		if (pin_user_pages_lock)
			mutex_unlock(pin_user_pages_lock);

		if (tried > 0)
			lru_cache_enable();

		if (ret == num_pages)
			break;

		if (ret >= 0) {
			dev_err(dev, "Can only pin %u of %u pages requested", ret, num_pages);
			for (i = 0; i < ret; i++)
				unpin_user_page(pages[i]);
		}
		ret = 0;
	}

	return ret;
}

int gcip_iommu_domain_pool_init(struct gcip_iommu_domain_pool *pool, struct device *dev,
				dma_addr_t base_daddr, size_t iova_space_size, size_t granule,
				unsigned int num_domains, enum gcip_iommu_domain_type domain_type)
{
	int ret;

	ret = gcip_domain_pool_init(dev, &pool->domain_pool, num_domains);
	if (ret)
		return ret;

	pool->dev = dev;
	pool->base_daddr = base_daddr;
	pool->size = iova_space_size;
	pool->granule = granule;
	pool->best_fit = false;
	pool->domain_type = domain_type;

	if (dev->of_node && (!base_daddr || !iova_space_size)) {
		const __be32 *prop;
		u32 n_addr, n_size;

		prop = of_get_property(dev->of_node, "#dma-address-cells", NULL);
		n_addr = max_t(u32, 1, prop ? be32_to_cpup(prop) : of_n_addr_cells(dev->of_node));

		prop = of_get_property(dev->of_node, "#dma-size-cells", NULL);
		n_size = max_t(u32, 1, prop ? be32_to_cpup(prop) : of_n_size_cells(dev->of_node));

		ret = get_window_config(dev, "gcip-dma-window", n_addr, n_size, &pool->base_daddr,
					&pool->size);
		if (ret)
			dev_warn(dev, "Failed to find gcip-dma-window property");

		get_window_config(dev, "gcip-reserved-map", n_addr, n_size,
				  &pool->reserved_base_daddr, &pool->reserved_size);
	}

	if (!pool->base_daddr || !pool->size) {
		gcip_domain_pool_destroy(&pool->domain_pool);
		return -EINVAL;
	}

	pool->last_daddr = pool->base_daddr + pool->size - 1;
	pool->min_pasid = 0;
	pool->max_pasid = 0;
	ida_init(&pool->pasid_pool);

	dev_dbg(dev, "Init GCIP IOMMU domain pool, base_daddr=%#llx, size=%#zx", pool->base_daddr,
		pool->size);

	return 0;
}

void gcip_iommu_domain_pool_destroy(struct gcip_iommu_domain_pool *pool)
{
	gcip_domain_pool_destroy(&pool->domain_pool);
	ida_destroy(&pool->pasid_pool);
}

void gcip_iommu_domain_pool_enable_best_fit_algo(struct gcip_iommu_domain_pool *pool)
{
	if (pool->domain_type == GCIP_IOMMU_DOMAIN_TYPE_IOVAD) {
		dev_warn(pool->dev, "This env doesn't support best-fit algorithm with IOVAD");
		pool->best_fit = false;
	} else {
		pool->best_fit = true;
	}
}

struct gcip_iommu_domain *gcip_iommu_domain_pool_alloc_domain(struct gcip_iommu_domain_pool *pool)
{
	struct gcip_iommu_domain *gdomain;
	int ret;

	gdomain = devm_kzalloc(pool->dev, sizeof(*gdomain), GFP_KERNEL);
	if (!gdomain)
		return ERR_PTR(-ENOMEM);

	gdomain->dev = pool->dev;
	gdomain->domain_pool = pool;
	gdomain->pasid = IOMMU_PASID_INVALID;
	gdomain->domain = gcip_domain_pool_alloc(&pool->domain_pool);
	if (IS_ERR_OR_NULL(gdomain->domain)) {
		ret = -ENOMEM;
		goto err_free_gdomain;
	}

	switch (pool->domain_type) {
	case GCIP_IOMMU_DOMAIN_TYPE_IOVAD:
		gdomain->ops = &iovad_ops;
		break;
	case GCIP_IOMMU_DOMAIN_TYPE_MEM_POOL:
		gdomain->ops = &mem_pool_ops;
		break;
	default:
		ret = -EINVAL;
		goto err_free_domain_pool;
	}

	ret = gdomain->ops->initialize_domain(gdomain);
	if (ret)
		goto err_free_domain_pool;

	if (pool->best_fit)
		gdomain->ops->enable_best_fit_algo(gdomain);

	return gdomain;

err_free_domain_pool:
	gcip_domain_pool_free(&pool->domain_pool, gdomain->domain);
err_free_gdomain:
	devm_kfree(pool->dev, gdomain);
	return ERR_PTR(ret);
}

void gcip_iommu_domain_pool_free_domain(struct gcip_iommu_domain_pool *pool,
					struct gcip_iommu_domain *domain)
{
	domain->ops->finalize_domain(domain);
	gcip_domain_pool_free(&pool->domain_pool, domain->domain);
	devm_kfree(pool->dev, domain);
}

void gcip_iommu_domain_pool_set_pasid_range(struct gcip_iommu_domain_pool *pool, ioasid_t min,
					    ioasid_t max)
{
	pool->min_pasid = min;
	pool->max_pasid = max;
}

static int _gcip_iommu_domain_pool_attach_domain(struct gcip_iommu_domain_pool *pool,
						 struct gcip_iommu_domain *domain)
{
	int ret = -EOPNOTSUPP, pasid = IOMMU_PASID_INVALID;

	pasid = ida_alloc_range(&pool->pasid_pool, pool->min_pasid, pool->max_pasid, GFP_KERNEL);
	if (pasid < 0)
		return pasid;

	/* NOTE(mainline): iommu_attach_device_pasid() gained a handle out-param. */
	ret = iommu_attach_device_pasid(domain->domain, pool->dev, pasid, NULL);
	if (ret) {
		ida_free(&pool->pasid_pool, pasid);
		return ret;
	}

	domain->pasid = pasid;
	return ret;
}

int gcip_iommu_domain_pool_attach_domain(struct gcip_iommu_domain_pool *pool,
					 struct gcip_iommu_domain *domain)
{
	if (domain->pasid != IOMMU_PASID_INVALID)
		/* Already attached. */
		return domain->pasid;

	return _gcip_iommu_domain_pool_attach_domain(pool, domain);
}

void gcip_iommu_domain_pool_detach_domain(struct gcip_iommu_domain_pool *pool,
					  struct gcip_iommu_domain *domain)
{
	if (domain->pasid == IOMMU_PASID_INVALID)
		return;
	iommu_detach_device_pasid(domain->domain, pool->dev, domain->pasid);
	ida_free(&pool->pasid_pool, domain->pasid);
	domain->pasid = IOMMU_PASID_INVALID;
}

struct gcip_iommu_domain *gcip_iommu_get_domain_for_dev(struct device *dev)
{
	struct gcip_iommu_domain *gdomain;

	gdomain = devm_kzalloc(dev, sizeof(*gdomain), GFP_KERNEL);
	if (!gdomain)
		return ERR_PTR(-ENOMEM);

	gdomain->domain = iommu_get_domain_for_dev(dev);
	if (!gdomain->domain) {
		devm_kfree(dev, gdomain);
		return ERR_PTR(-ENODEV);
	}

	gdomain->dev = dev;
	gdomain->default_domain = true;
	gdomain->pasid = 0;

	return gdomain;
}

struct gcip_iommu_domain *
gcip_iommu_get_domain_for_dev_from_pool(struct device *dev, struct gcip_iommu_domain_pool *pool)
{
	struct gcip_iommu_domain *gdomain;
	int ret;

	gdomain = gcip_iommu_get_domain_for_dev(dev);
	if (IS_ERR(gdomain))
		return gdomain;

	gdomain->domain_pool = pool;
	switch (pool->domain_type) {
	case GCIP_IOMMU_DOMAIN_TYPE_IOVAD:
		gdomain->ops = &iovad_ops;
		break;
	case GCIP_IOMMU_DOMAIN_TYPE_MEM_POOL:
		gdomain->ops = &mem_pool_ops;
		break;
	default:
		ret = -EINVAL;
		goto err_free_domain;
	}

	ret = gdomain->ops->initialize_domain(gdomain);
	if (ret)
		goto err_free_domain;

	if (pool->best_fit)
		gdomain->ops->enable_best_fit_algo(gdomain);

	return gdomain;

err_free_domain:
	devm_kfree(dev, gdomain);
	return ERR_PTR(ret);
}

u64 gcip_iommu_encode_gcip_map_flags(enum dma_data_direction dir, bool coherent,
				     unsigned long dma_attrs, bool restrict_iova, bool mmio)
{
	if (dir == DMA_FROM_DEVICE)
		dir = DMA_BIDIRECTIONAL;

	return (dir << GCIP_MAP_FLAGS_DMA_DIRECTION_OFFSET) |
	       (coherent << GCIP_MAP_FLAGS_DMA_COHERENT_OFFSET) |
	       (dma_attrs << GCIP_MAP_FLAGS_DMA_ATTR_OFFSET) |
	       (restrict_iova << GCIP_MAP_FLAGS_RESTRICT_IOVA_OFFSET) |
	       (mmio << GCIP_MAP_FLAGS_MMIO_OFFSET);
}

/* The helper function of gcip_iommu_dmabuf_map_show for printing multi-entry mappings. */
static void entry_show_dma_addrs(struct gcip_iommu_mapping *mapping, struct seq_file *s)
{
	struct sg_table *sgt = mapping->sgt;
	struct scatterlist *sg;
	uint i;

	if (sgt && sgt->nents > 1) {
		sg = sgt->sgl;
		seq_puts(s, " dma=[");
		for (i = 0; i < sgt->nents; i++) {
			if (i)
				seq_puts(s, ", ");
			seq_printf(s, "%pad", &sg_dma_address(sg));
			sg = sg_next(sg);
		}
		seq_puts(s, "]");
	}
	seq_puts(s, "\n");
}

void gcip_iommu_dmabuf_map_show(struct gcip_iommu_mapping *mapping, struct seq_file *s)
{
	static const char *dma_dir_tbl[4] = { "rw", "r", "w", "?" };
	struct gcip_iommu_dmabuf_mapping *dmabuf_mapping = to_dmabuf_mapping(mapping);

	seq_printf(s, "  %pad %lu %s %s %pad", &mapping->device_address,
		   DIV_ROUND_UP(mapping->size, PAGE_SIZE), dma_dir_tbl[mapping->dir],
		   dmabuf_mapping->dma_buf->exp_name,
		   &sg_dma_address(dmabuf_mapping->sgt_default->sgl));
	entry_show_dma_addrs(mapping, s);
}

size_t gcip_iommu_dmabuf_hiorder_size(struct gcip_iommu_mapping *mapping)
{
	struct gcip_iommu_dmabuf_mapping *dmabuf_mapping = to_dmabuf_mapping(mapping);
	struct scatterlist *sgl;
	int i;
	size_t ret = 0;

	for_each_sg(dmabuf_mapping->sgt_default->sgl, sgl, dmabuf_mapping->sgt_default->orig_nents,
		    i)
		if (sgl->length >= SZ_2M)
			ret += sgl->length;

	return ret;
}

/**
 * gcip_iommu_get_buffer_npages() - Calculates the number of pages from given host address and size.
 * @dev: The device pointer for printing debug message.
 * @host_address: The host address passed by user.
 * @size: The size passed by user.
 * @n_pg_ptr: The pointer used to output the number of pages.
 *
 * Return: Error code or 0 on success.
 */
static int gcip_iommu_get_buffer_npages(struct device *dev, u64 host_address, size_t size,
					uint *n_pg_ptr)
{
	ulong offset = offset_in_page(host_address);
	uint num_pages;

	if (unlikely(offset + size < offset)) {
		dev_dbg(dev, "Overflow: offset(%lu) + size(%lu) < offset(%lu)", offset, size,
			offset);
		return -EFAULT;
	}

	num_pages = DIV_ROUND_UP((size + offset), PAGE_SIZE);
	if (unlikely(num_pages * PAGE_SIZE < size + offset)) {
		dev_dbg(dev, "Overflow: num_pages(%u) * PAGE_SIZE(%lu) < size(%lu) + offset(%lu)",
			num_pages, PAGE_SIZE, offset, size);
		return -EFAULT;
	}

	*n_pg_ptr = num_pages;

	return 0;
}

/**
 * gcip_iommu_get_gup_flags() - Checks the access mode of the given address with VMA.
 * @host_addr: The target host_addr for checking the access.
 * @dev: The device struct used to print messages.
 * @map_debug_flags: Pointer to initialized gcip_map_debug_flags, bits may be set for any findings
 *                   here to be noted
 *
 * Checks and returns the read/write permission of address @host_addr.
 * If the target address can not be found in current->mm, assuming it is RW.
 *
 * Return: The encoded gup_flags of target host_addr.
 */
static unsigned int gcip_iommu_get_gup_flags(u64 host_addr, struct device *dev,
					     enum gcip_map_debug_flags *map_debug_flags)
{
	struct vm_area_struct *vma;
	unsigned int gup_flags;
	vm_flags_t vm_flags;

	mmap_read_lock(current->mm);
	vma = vma_lookup(current->mm, host_addr & PAGE_MASK);
	if (vma)
		vm_flags = vma->vm_flags;
	mmap_read_unlock(current->mm);

	if (!vma) {
		dev_dbg(dev, "unable to find address in VMA, assuming buffer writable");
		gup_flags = FOLL_LONGTERM | FOLL_WRITE;
		*map_debug_flags |= GCIP_MAP_DEBUG_VMA_NF;
	} else if (vm_flags & VM_WRITE) {
		gup_flags = FOLL_LONGTERM | FOLL_WRITE;
	} else {
		gup_flags = FOLL_LONGTERM;
	}

	if (vma && is_cow_mapping(vm_flags) && (gup_flags & FOLL_WRITE)) {
		dev_dbg(dev, "%#llx maps copy-on-write (vm_flags %#lx)", host_addr, vm_flags);
		*map_debug_flags |= GCIP_MAP_DEBUG_COW;
	}

	return gup_flags;
}

/**
 * gcip_iommu_alloc_and_pin_user_pages() - Pins the user pages and returns an array of struct page
 *                                         pointers for the pinned pages.
 * @dev: The device pointer used to print messages.
 * @host_address: The requested host address.
 * @num_pages: The requested number of pages.
 * @gup_flags: The pointer gup_flags for pinning user pages.
 *             The flag FOLL_WRITE in gup_flags may be reomved if the user pages cannot be pinned
 *             with write access.
 * @pin_user_pages_lock: The lock to protect pin_user_page
 * @map_debug_flags: Pointer to initialized gcip_map_debug_flags, bits may be set for any findings
 *                   here to be noted
 *
 * This function tries to pin the user pages with `pin_user_page_fast` first and will try
 * `pin_user_page` on failure.
 * If both of above functions failed, it will retry with read-only mode.
 *
 * Return: Pinned user pages or error pointer on failure.
 */
static struct page **gcip_iommu_alloc_and_pin_user_pages(struct device *dev, u64 host_address,
							 uint num_pages, unsigned int *gup_flags,
							 struct mutex *pin_user_pages_lock,
							 enum gcip_map_debug_flags *map_debug_flags)
{
	unsigned long start_addr = host_address & PAGE_MASK;
	struct page **pages;
	int ret;

	/*
	 * "num_pages" is decided from user-space arguments, don't show warnings
	 * when facing malicious input.
	 */
	pages = kvmalloc_array(num_pages, sizeof(*pages), GFP_KERNEL | __GFP_NOWARN);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	ret = gcip_pin_user_pages(dev, pages, start_addr, num_pages, *gup_flags,
				  pin_user_pages_lock);
	if (ret == num_pages)
		return pages;

	if (!(*gup_flags & FOLL_WRITE))
		goto err_free_pages;

	dev_warn_ratelimited(dev, "pin failed (ret=%d), assuming buffer is read-only", ret);
	*gup_flags &= ~FOLL_WRITE;
	*map_debug_flags |= GCIP_MAP_DEBUG_ASSUME_RDONLY;

	ret = gcip_pin_user_pages(dev, pages, start_addr, num_pages, *gup_flags,
				  pin_user_pages_lock);
	if (ret != num_pages)
		goto err_free_pages;

	atomic64_add(num_pages, &current->mm->pinned_vm);

	return pages;

err_free_pages:
	kvfree(pages);
	dev_err(dev, "Pin user pages failed: user_add=%#llx, num_pages=%u, %s, ret=%d\n",
		host_address, num_pages, ((*gup_flags & FOLL_WRITE) ? "writeable" : "read-only"),
		ret);

	return ERR_PTR(ret >= 0 ? -EFAULT : ret);
}

/**
 * gcip_iommu_buffer_sgt_create() - Pins user pages and creates a scatter-gather table.
 * @dev: The device pointer used to print messages.
 * @host_address: The requested host address.
 * @size: The size of the buffer.
 * @map_flags_ptr: The pointer to the mapping flags.
 * @debug_flags_ptr: The pointer to the debug flags.
 * @pin_user_pages_lock: The lock to protect pin_user_page.
 *
 * The @map_flags_ptr and @debug_flags_ptr are passed by pointer because they can be modified by
 * this function.
 *
 * Return: The pointer to the created sg_table, or the pointer to a negative errno otherwise.
 */
static struct sg_table *gcip_iommu_buffer_sgt_create(struct device *dev, u64 host_address,
						     size_t size, u64 *map_flags_ptr,
						     enum gcip_map_debug_flags *debug_flags_ptr,
						     struct mutex *pin_user_pages_lock)
{
	u64 gcip_map_flags = *map_flags_ptr;
	enum gcip_map_debug_flags map_debug_flags = *debug_flags_ptr;
	enum dma_data_direction orig_dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);
	uint num_pages;
	struct page **pages;
	int ret, i;
	struct sg_table *sgt;
	uint gup_flags;

	if (orig_dir != DMA_BIDIRECTIONAL && orig_dir != DMA_TO_DEVICE) {
		dev_err(dev, "Invalid DMA direction: %d", orig_dir);
		return ERR_PTR(-EINVAL);
	}

	if (size == 0)
		return ERR_PTR(-EINVAL);

	if (!access_ok((const void *)host_address, size)) {
		dev_err(dev, "invalid address range in buffer map request");
		return ERR_PTR(-EFAULT);
	}

	gup_flags = gcip_iommu_get_gup_flags(host_address, dev, &map_debug_flags);

	ret = gcip_iommu_get_buffer_npages(dev, host_address, size, &num_pages);
	if (ret) {
		dev_err(dev, "Buffer size overflow: size=%#zx", size);
		return ERR_PTR(ret);
	}

	pages = gcip_iommu_alloc_and_pin_user_pages(dev, host_address, num_pages, &gup_flags,
						    pin_user_pages_lock, &map_debug_flags);
	if (IS_ERR(pages)) {
		dev_err(dev, "Failed to pin user pages (ret=%ld)\n", PTR_ERR(pages));
		return ERR_CAST(pages);
	}

	if (!(gup_flags & FOLL_WRITE) && orig_dir != DMA_TO_DEVICE) {
		gcip_map_flags &= ~GCIP_MAP_MASK_DMA_DIRECTION;
		gcip_map_flags |= ((u64)(DMA_TO_DEVICE) << GCIP_MAP_FLAGS_DMA_DIRECTION_OFFSET);
		map_debug_flags |= GCIP_MAP_DEBUG_OVRRD_RDDIR;
	}

	/* If mapping a writeable VMA read-only, clear CoW debug flag if set. */
	if (GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags) == DMA_TO_DEVICE)
		map_debug_flags &= ~GCIP_MAP_DEBUG_COW;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		goto err_unpin_page;
	}

	ret = sg_alloc_table_from_pages(sgt, pages, num_pages, 0, num_pages * PAGE_SIZE,
					GFP_KERNEL);
	if (ret) {
		dev_err(dev, "Failed to alloc sgt for mapping (ret=%d)\n", ret);
		goto err_free_table;
	}

	*map_flags_ptr = gcip_map_flags;
	*debug_flags_ptr = map_debug_flags;
	kvfree(pages);

	return sgt;

err_free_table:
	/*
	 * The caller must call sg_free_table to clean up any leftover allocations if
	 * sg_alloc_table_from_pages returns non-zero values.
	 */
	sg_free_table(sgt);
	kfree(sgt);
err_unpin_page:
	for (i = 0; i < num_pages; i++)
		unpin_user_page(pages[i]);
	atomic64_sub(num_pages, &current->mm->pinned_vm);
	kvfree(pages);

	return ERR_PTR(ret);
}

/**
 * gcip_iommu_domain_map_buffer_sgt_to_iova() - Prepare the sg_table of a buffer and map it to the
 *                                              domain at given IOVA.
 * @domain: The GCIP IOMMU domain the buffer should be mapped to.
 * @host_address: The starting virtual address of the user-space buffer.
 * @size: The size of the buffer in bytes.
 * @iova: The target IOVA to map the buffer to.
 * @map_flags_ptr: The pointer to the gcip_map_flags.
 * @debug_flags_ptr: The pointer to the gcip_map_debug_flags.
 * @pin_user_pages_lock: A mutex to protect the pin_user_pages calls.
 *
 * The @map_flags_ptr and @debug_flags_ptr are passed by pointer because they can be modified by
 * this function.
 *
 * If @iova is 0, a new IOVA will be allocated from the pool.
 *
 * Return: The pointer to the sg_table on success, or the pointer to a negative errno otherwise.
 */
static struct sg_table *
gcip_iommu_domain_map_buffer_sgt_to_iova(struct gcip_iommu_domain *domain, u64 host_address,
					 size_t size, dma_addr_t iova, u64 *map_flags_ptr,
					 enum gcip_map_debug_flags *debug_flags_ptr,
					 struct mutex *pin_user_pages_lock)
{
	u64 gcip_map_flags = *map_flags_ptr;
	enum gcip_map_debug_flags map_debug_flags = *debug_flags_ptr;
	struct sg_table *sgt;
	int ret;

	sgt = gcip_iommu_buffer_sgt_create(domain->dev, host_address, size, &gcip_map_flags,
					   &map_debug_flags, pin_user_pages_lock);
	if (IS_ERR(sgt))
		return ERR_CAST(sgt);

	ret = gcip_iommu_domain_map_sgt_to_iova(domain, sgt, iova, &gcip_map_flags);
	if (!ret) {
		ret = -ENOSPC;
		dev_err(domain->dev, "Failed to map sgt to domain (ret=%d)\n", ret);
		goto err_destroy_sgt;
	}

	*map_flags_ptr = gcip_map_flags;
	*debug_flags_ptr = map_debug_flags;

	return sgt;

err_destroy_sgt:
	gcip_iommu_buffer_sgt_destroy(sgt, DMA_NONE, current->mm);

	return ERR_PTR(ret);
}

/**
 * gcip_iommu_dmabuf_sgt_create() - Attach and map dma-buf to the default domain.
 * @dev: The device to attach the dma-buf to.
 * @dmabuf: The dma_buf to attach and map.
 * @map_flags_ptr: The pointer to the mapping flags.
 * @attachment_ptr: Pointer to return the dma_buf_attachment.
 *
 * The @map_flags_ptr is passed by pointer because it can be modified by this function.
 *
 * Return: The sg_table of the mapped dma-buf, or the pointer to a negative errno otherwise.
 */
static struct sg_table *gcip_iommu_dmabuf_sgt_create(struct device *dev, struct dma_buf *dmabuf,
						     u64 *map_flags_ptr,
						     struct dma_buf_attachment **attachment_ptr)
{
	u64 gcip_map_flags = *map_flags_ptr;
	enum dma_data_direction dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt_default;
	int ret;

	if (dir != DMA_BIDIRECTIONAL && dir != DMA_TO_DEVICE) {
		dev_err(dev, "Invalid DMA direction: %d", dir);
		return ERR_PTR(-EINVAL);
	}

	attachment = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(attachment)) {
		dev_err(dev, "Failed to attach dma-buf (ret=%ld, name=%s)\n", PTR_ERR(attachment),
			dmabuf->name);
		return ERR_CAST(attachment);
	}

	/*
	 * NOTE(mainline): struct dma_buf_attachment::dma_map_attrs was removed; there is no
	 * longer a per-attachment way to forward DMA_ATTR_* flags to dma_buf_map_attachment().
	 * The requested attrs are still preserved in gcip_map_flags for the IOMMU mapping step.
	 */

	/* Map the attachment into the default domain. */
	sgt_default = dma_buf_map_attachment(attachment, dir);
	if (IS_ERR(sgt_default)) {
		ret = PTR_ERR(sgt_default);
		dev_err(dev, "Failed to get sgt from attachment (ret=%d, name=%s, size=%lu)\n", ret,
			dmabuf->name, dmabuf->size);
		goto err_detach_dmabuf;
	}

	*map_flags_ptr = gcip_map_flags;
	*attachment_ptr = attachment;

	return sgt_default;

err_detach_dmabuf:
	dma_buf_detach(dmabuf, attachment);

	return ERR_PTR(ret);
}

/**
 * gcip_iommu_domain_map_dma_buf_sgt_to_iova() - Prepare the sg_table of a dmabuf and map it to the
 *                                               domain at given IOVA.
 * @domain: The desired IOMMU domain where the sgt should be mapped.
 * @dmabuf: The shared dma-buf object.
 * @iova: The target IOVA to map @sgt.
 * @map_flags_ptr: The pointer to the gcip_map_flags.
 * @attach_ptr: The pointer to return the device attachment of @dmabuf.
 * @sgt_default_ptr: The pointer to return the default sg_table.
 *
 * The @gcip_map_flags is passed by pointer because it is possible to be modified by this function.
 *
 * If @iova is 0, a new IOVA will be allocated from the pool.
 *
 * Return: The pointer to the sg_table on success, or the pointer to a negative errno otherwise.
 */
static struct sg_table *
gcip_iommu_domain_map_dma_buf_sgt_to_iova(struct gcip_iommu_domain *domain, struct dma_buf *dmabuf,
					  dma_addr_t iova, u64 *map_flags_ptr,
					  struct dma_buf_attachment **attach_ptr,
					  struct sg_table **sgt_default_ptr)
{
	struct device *dev = domain->dev;
	u64 gcip_map_flags = *map_flags_ptr;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt_default, *sgt_ret;
	int nents_mapped;
	int ret;

	sgt_default =
		gcip_iommu_dmabuf_sgt_create(domain->dev, dmabuf, &gcip_map_flags, &attachment);
	if (IS_ERR(sgt_default)) {
		ret = PTR_ERR(sgt_default);
		dev_err(dev, "Failed to create sg_table for dmabuf (%d)", ret);
		return ERR_CAST(sgt_default);
	}

	if (domain->default_domain) {
		sgt_ret = sgt_default;
		goto out;
	}

	sgt_ret = copy_alloc_sg_table(sgt_default);
	if (IS_ERR(sgt_ret)) {
		ret = PTR_ERR(sgt_ret);
		dev_err(domain->dev, "Failed to copy sg_table (ret=%d)\n", ret);
		goto err_destroy_sgt;
	}

	nents_mapped = gcip_iommu_domain_map_sgt_to_iova(domain, sgt_ret, iova, &gcip_map_flags);
	if (!nents_mapped) {
		ret = -ENOSPC;
		dev_err(domain->dev, "Failed to map dmabuf to IOMMU domain (ret=%d)\n", ret);
		goto err_free_sgt_ret;
	}

out:
	*attach_ptr = attachment;
	*sgt_default_ptr = sgt_default;
	*map_flags_ptr = gcip_map_flags;

	return sgt_ret;

err_free_sgt_ret:
	sg_free_table(sgt_ret);
	kfree(sgt_ret);
err_destroy_sgt:
	gcip_iommu_dmabuf_sgt_destroy(sgt_default, dmabuf, attachment,
				      GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags));

	return ERR_PTR(ret);
}

struct gcip_iommu_mapping *gcip_iommu_domain_map_buffer_to_iova(struct gcip_iommu_domain *domain,
								u64 host_address, size_t size,
								dma_addr_t iova, u64 gcip_map_flags,
								struct mutex *pin_user_pages_lock)
{
	struct gcip_iommu_buffer_mapping *buffer_mapping;
	struct gcip_iommu_mapping *mapping;
	struct sg_table *sgt;
	enum gcip_map_debug_flags map_debug_flags = 0;
	struct scatterlist *sl;
	int ret;
	int i;

	buffer_mapping = kzalloc(sizeof(*buffer_mapping), GFP_KERNEL);
	if (!buffer_mapping)
		return ERR_PTR(-ENOMEM);

	sgt = gcip_iommu_domain_map_buffer_sgt_to_iova(domain, host_address, size, iova,
						       &gcip_map_flags, &map_debug_flags,
						       pin_user_pages_lock);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_free_mapping;
	}

	buffer_mapping->host_address = host_address;
	mutex_init(&buffer_mapping->sync_lock);

	/* Grab a reference for owning_mm.  */
	mmgrab(current->mm);
	buffer_mapping->owning_mm = current->mm;

	mapping = &buffer_mapping->mapping;
	mapping->domain = domain;
	mapping->type = GCIP_IOMMU_MAPPING_BUFFER;
	mapping->sgt = sgt;
	mapping->device_address = sg_dma_address(sgt->sgl) + offset_in_page(host_address);
	mapping->user_specified_daddr = iova;
	mapping->alloced_iova = iova ? 0 : sg_dma_address(sgt->sgl);
	mapping->gcip_map_flags = gcip_map_flags;
	mapping->dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);
	mapping->map_debug_flags = map_debug_flags;

	/* Calculate the mapped size. */
	mapping->size = 0;
	for_each_sg(sgt->sgl, sl, sgt->nents, i)
		mapping->size += sg_dma_len(sl);

	return mapping;

err_free_mapping:
	kfree(buffer_mapping);

	return ERR_PTR(ret);
}

struct gcip_iommu_mapping *gcip_iommu_domain_map_buffer(struct gcip_iommu_domain *domain,
							u64 host_address, size_t size,
							u64 gcip_map_flags,
							struct mutex *pin_user_pages_lock)
{
	return gcip_iommu_domain_map_buffer_to_iova(domain, host_address, size, 0, gcip_map_flags,
						    pin_user_pages_lock);
}

static int gcip_iommu_mapping_remap_buffer(struct gcip_iommu_mapping *mapping,
					   struct mutex *pin_user_pages_lock)
{
	struct gcip_iommu_buffer_mapping *buffer_mapping;
	struct sg_table *sgt;
	enum gcip_map_debug_flags map_debug_flags;

	if (mapping->type != GCIP_IOMMU_MAPPING_BUFFER)
		return -EINVAL;

	if (mapping->sgt) {
		dev_err(mapping->domain->dev, "remap buffer iova %pad not trimmed",
			&mapping->device_address);
		return -EBUSY;
	}

	buffer_mapping = to_buffer_mapping(mapping);
	sgt = gcip_iommu_domain_map_buffer_sgt_to_iova(mapping->domain,
						       buffer_mapping->host_address, mapping->size,
						       mapping->device_address,
						       &mapping->gcip_map_flags, &map_debug_flags,
						       pin_user_pages_lock);
	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	mapping->sgt = sgt;
	mapping->map_debug_flags = map_debug_flags;

	return 0;
}

struct gcip_iommu_mapping *gcip_iommu_domain_map_dma_buf_to_iova(struct gcip_iommu_domain *domain,
								 struct dma_buf *dmabuf,
								 dma_addr_t iova,
								 u64 gcip_map_flags)
{
	struct gcip_iommu_dmabuf_mapping *dmabuf_mapping;
	struct gcip_iommu_mapping *mapping;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt, *sgt_default;
	int ret;

	dmabuf_mapping = kzalloc(sizeof(*dmabuf_mapping), GFP_KERNEL);
	if (!dmabuf_mapping)
		return ERR_PTR(-ENOMEM);

	sgt = gcip_iommu_domain_map_dma_buf_sgt_to_iova(domain, dmabuf, iova, &gcip_map_flags,
							&attachment, &sgt_default);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_free_mapping;
	}

	get_dma_buf(dmabuf);
	dmabuf_mapping->dma_buf = dmabuf;
	dmabuf_mapping->dma_buf_attachment = attachment;
	dmabuf_mapping->sgt_default = sgt_default;

	mapping = &dmabuf_mapping->mapping;
	mapping->domain = domain;
	mapping->size = dmabuf->size;
	mapping->type = GCIP_IOMMU_MAPPING_DMA_BUF;
	mapping->user_specified_daddr = iova;
	mapping->sgt = sgt;
	mapping->device_address = sg_dma_address(sgt->sgl);
	mapping->gcip_map_flags = gcip_map_flags;
	mapping->dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);

	return mapping;

err_free_mapping:
	kfree(dmabuf_mapping);

	return ERR_PTR(ret);
}

struct gcip_iommu_mapping *gcip_iommu_domain_map_dma_buf(struct gcip_iommu_domain *domain,
							 struct dma_buf *dmabuf, u64 gcip_map_flags)
{
	return gcip_iommu_domain_map_dma_buf_to_iova(domain, dmabuf, 0, gcip_map_flags);
}

void gcip_iommu_mapping_unmap(struct gcip_iommu_mapping *mapping)
{
	void *data = mapping->data;
	const struct gcip_iommu_mapping_ops *ops = mapping->ops;

	if (mapping->type == GCIP_IOMMU_MAPPING_BUFFER)
		gcip_iommu_mapping_unmap_buffer(mapping);
	else if (mapping->type == GCIP_IOMMU_MAPPING_DMA_BUF)
		gcip_iommu_mapping_unmap_dma_buf(mapping);

	/* From now on, @mapping is released and must not be accessed. */

	if (ops && ops->after_unmap)
		ops->after_unmap(data);
}

int gcip_iommu_mapping_sync(struct gcip_iommu_mapping *mapping, struct device *dev, u64 offset,
			    u64 size, bool for_cpu)
{
	struct gcip_iommu_buffer_mapping *buffer_mapping;
	struct scatterlist *cur_sg, *start_sg = NULL, *end_sg = NULL;
	int nelems = 0, ret = 0, i;
	size_t cur_offset = 0;
	u64 start, end;
	unsigned int start_diff = 0, end_diff = 0;

	if (mapping->type != GCIP_IOMMU_MAPPING_BUFFER)
		return -EINVAL;

	buffer_mapping = to_buffer_mapping(mapping);

	if (!mapping->sgt)
		return -EINVAL;

	/*
	 * Valid input requires:
	 * - size > 0 (offset + size != offset)
	 * - offset + size does not overflow (offset + size > offset)
	 * - the mapped range falls within [0 : mapping->size]
	 */
	if (offset + size <= offset || offset + size > mapping->size)
		return -EINVAL;

	/* Ensure only one sync at a time as the scatterlist will be modified. */
	mutex_lock(&buffer_mapping->sync_lock);

	/*
	 * Mappings are created at a PAGE_SIZE granularity, however other data which is not part of
	 * the mapped buffer may be present in the first and last pages of the buffer's
	 * scattergather list.
	 *
	 * To ensure only the intended data is actually synced, iterate through the scattergather
	 * list, to find the first and last `scatterlist`s which contain the range of the buffer to
	 * sync.
	 *
	 * After those links are found, change their offset/lengths so that `dma_map_sg_for_*()`
	 * will only sync the requested region.
	 */
	start = offset_in_page(buffer_mapping->host_address) + offset;
	end = start + size;

	for_each_sg(mapping->sgt->sgl, cur_sg, mapping->sgt->orig_nents, i) {
		/* Check if the scatterlist contains the start of the range to sync. */
		if (cur_offset <= start && start < cur_offset + cur_sg->length) {
			start_sg = cur_sg;
			start_diff = start - cur_offset;
		}

		if (start_sg)
			nelems++;

		/* Check if the scatterlist contains the end of the range to sync. */
		if (cur_offset < end && end <= cur_offset + cur_sg->length) {
			end_sg = cur_sg;
			end_diff = cur_offset + cur_sg->length - end;
			break;
		}

		cur_offset += cur_sg->length;
	}

	/* Make sure the valid start/end SGs were found. */
	if (!start_sg || !end_sg) {
		ret = -EINVAL;
		goto out_unlock;
	}

	start_sg->offset += start_diff;
	start_sg->dma_address += start_diff;
	start_sg->length -= start_diff;
	start_sg->dma_length -= start_diff;
	end_sg->length -= end_diff;
	end_sg->dma_length -= end_diff;

	if (for_cpu)
		dma_sync_sg_for_cpu(dev, start_sg, nelems, mapping->dir);
	else
		dma_sync_sg_for_device(dev, start_sg, nelems, mapping->dir);

	/* Revert the start and end scatterlist list. */
	end_sg->length += end_diff;
	end_sg->dma_length += end_diff;
	start_sg->offset -= start_diff;
	start_sg->dma_address -= start_diff;
	start_sg->length += start_diff;
	start_sg->dma_length += start_diff;

out_unlock:
	mutex_unlock(&buffer_mapping->sync_lock);

	return ret;
}

void gcip_iommu_mapping_trim(struct gcip_iommu_mapping *mapping)
{
	struct gcip_iommu_buffer_mapping *buffer_mapping;

	if (mapping->type != GCIP_IOMMU_MAPPING_BUFFER) {
		dev_err(mapping->domain->dev, "Only buffer mappings can be trimmed");
		return;
	}

	buffer_mapping = to_buffer_mapping(mapping);
	gcip_iommu_mapping_buffer_flush_sgt(buffer_mapping);
}

int gcip_iommu_mapping_remap(struct gcip_iommu_mapping *mapping, struct mutex *pin_user_pages_lock)
{
	if (mapping->type == GCIP_IOMMU_MAPPING_BUFFER)
		return gcip_iommu_mapping_remap_buffer(mapping, pin_user_pages_lock);
	return -EINVAL;
}

dma_addr_t gcip_iommu_alloc_iova(struct gcip_iommu_domain *domain, size_t size, u64 gcip_map_flags)
{
	bool restrict_iova = GCIP_MAP_FLAGS_GET_RESTRICT_IOVA(gcip_map_flags);
	dma_addr_t iova;

	iova = domain->ops->alloc_iova_space(domain, gcip_iommu_domain_align(domain, size),
					     restrict_iova);
	if (!iova)
		dev_err(domain->dev, "%siova alloc size %zu failed",
			restrict_iova ? "32-bit " : "", size);
	return iova;
}

void gcip_iommu_free_iova(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size)
{
	domain->ops->free_iova_space(domain, iova, gcip_iommu_domain_align(domain, size));
}

int gcip_iommu_map(struct gcip_iommu_domain *domain, dma_addr_t iova, phys_addr_t paddr,
		   size_t size, u64 gcip_map_flags)
{
	enum dma_data_direction dir = GCIP_MAP_FLAGS_GET_DMA_DIRECTION(gcip_map_flags);
	bool coherent = GCIP_MAP_FLAGS_GET_DMA_COHERENT(gcip_map_flags);
	bool mmio = GCIP_MAP_FLAGS_GET_MMIO(gcip_map_flags);
	unsigned long attrs = GCIP_MAP_FLAGS_GET_DMA_ATTR(gcip_map_flags);
	int prot = dma_info_to_prot(dir, coherent, attrs);

	if (mmio)
		prot |= IOMMU_MMIO;

#if GCIP_IOMMU_MAP_HAS_GFP
	return iommu_map(domain->domain, iova, paddr, size, prot, GFP_KERNEL);
#else
	return iommu_map(domain->domain, iova, paddr, size, prot);
#endif /* GCIP_IOMMU_MAP_HAS_GFP */
}

/* Reverts gcip_iommu_map(). */
void gcip_iommu_unmap(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size)
{
	size_t unmapped = iommu_unmap(domain->domain, iova, size);

	if (unlikely(unmapped != size))
		dev_warn(domain->dev, "Unmapping IOVA %pad, size (%#zx) only unmapped %#zx", &iova,
			 size, unmapped);
}

MODULE_IMPORT_NS("DMA_BUF");
