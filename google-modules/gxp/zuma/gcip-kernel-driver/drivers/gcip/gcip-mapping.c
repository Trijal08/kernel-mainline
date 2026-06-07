// SPDX-License-Identifier: GPL-2.0-only
/*
 * Definitions of GCIP mapping structs and interfaces.
 *
 * Copyright (C) 2025 Google LLC
 */

#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-resv.h>
#include <linux/err.h>
#include <linux/gfp_types.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <gcip/gcip-iommu.h>
#include <gcip/gcip-mapping.h>

#define to_dmabuf_mapping(mapping) container_of(mapping, struct gcip_iommu_dmabuf_mapping, mapping)

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
		if (mapping->user_specified_daddr)
			gcip_iommu_domain_unmap_sgt_from_iova(mapping->domain, mapping->sgt,
							      mapping->gcip_map_flags);
		else
			gcip_iommu_domain_unmap_sgt(mapping->domain, mapping->sgt,
						    mapping->gcip_map_flags);
		sg_free_table(mapping->sgt);
		kfree(mapping->sgt);
	}

	gcip_iommu_dmabuf_sgt_destroy(dmabuf_mapping->sgt_default, dmabuf_mapping->dma_buf,
				      dmabuf_mapping->dma_buf_attachment, mapping->dir);
	dma_buf_put(dmabuf_mapping->dma_buf);
	kfree(dmabuf_mapping);
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

MODULE_IMPORT_NS("DMA_BUF");
