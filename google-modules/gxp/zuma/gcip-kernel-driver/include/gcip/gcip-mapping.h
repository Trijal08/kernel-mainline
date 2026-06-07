/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Declarations of GCIP mapping structs and interfaces.
 *
 * Copyright (C) 2025 Google LLC
 */

#ifndef __GCIP_MAPPING_H__
#define __GCIP_MAPPING_H__

#include <linux/dma-buf.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include <gcip/gcip-iommu.h>

/*
 * This function basically works the same as the `gcip_iommu_domain_map_dma_buf` function but
 * receives the target @iova to map the dma-buf. If @iova is 0, there will be no difference.
 *
 * Note that the passed @iova won't be freed if it was non-zero when the returned mapping is going
 * to be unmapped. The life cycle of the given @iova must be managed by the user.
 */
struct gcip_iommu_mapping *gcip_iommu_domain_map_dma_buf_to_iova(struct gcip_iommu_domain *domain,
								 struct dma_buf *dmabuf,
								 dma_addr_t iova,
								 u64 gcip_map_flags);

/**
 * gcip_iommu_domain_map_dma_buf() - Maps the DMA buffer to the target IOMMU domain.
 * @domain: The desired IOMMU domain where the DMA buffer should be mapped.
 * @dmabuf: The dma_buf to map to @domain.
 * @gcip_map_flags: The flags used to create the mapping, which should be encoded with
 *                  gcip_iommu_encode_gcip_map_flags().
 *
 * The DMA buffer will be mapped to the default domain first to get a scatter-gather table.
 * The received sgt will be copied to a new sgt and the new one will be mapped to the target domain.
 * The IOVAs of those domains may be different and the mappings will be released at once by calling
 * `gcip_iommu_mapping_unmap`.
 *
 * Return: The mapping of the desired DMA buffer with type GCIP_IOMMU_MAPPING_DMA_BUF
 *         or an error pointer on failure.
 */
struct gcip_iommu_mapping *gcip_iommu_domain_map_dma_buf(struct gcip_iommu_domain *domain,
							 struct dma_buf *dmabuf,
							 u64 gcip_map_flags);

/**
 * gcip_iommu_dmabuf_map_show() - Writes the dma-buf mapping information to the seq_file.
 * @mapping: The container of the mapping info.
 * @s: The seq_file that the mapping info should be written to.
 *
 * Following information will be written to the seq_file:
 * 1. Device addresses of the related domains.
 * 2. Number of pages.
 * 3. DMA data direction.
 * 4. The name of the dmabuf.
 */
void gcip_iommu_dmabuf_map_show(struct gcip_iommu_mapping *mapping, struct seq_file *s);

/**
 * gcip_iommu_dmabuf_hiorder_size() - Returns the number of bytes mapped by high-order (>=2MB)
 *                                    scatter-gather list segments for a dma-buf mapping.
 * @mapping: The container of the mapping info.
 */
size_t gcip_iommu_dmabuf_hiorder_size(struct gcip_iommu_mapping *mapping);

/**
 * gcip_iommu_mapping_unmap() - Unmaps the mapping depends on its type.
 * @mapping: The pointer of the mapping instance to be unmapped.
 *
 * Reverting either gcip_iommu_domain_map_dma_buf() or gcip_iommu_domain_map_buffer().
 *
 * The @mapping->gcip_map_flags will be used for unmapping the buffer, it can be modified if
 * necessary such as adding DMA_ATTR_SKIP_CPU_SYNC flag.
 * In most scenarios the we should use the same flag which we used while mapping especially for
 * direction, coherent, and iova_restrict.
 */
void gcip_iommu_mapping_unmap(struct gcip_iommu_mapping *mapping);

#endif /* __GCIP_MAPPING_H__ */
