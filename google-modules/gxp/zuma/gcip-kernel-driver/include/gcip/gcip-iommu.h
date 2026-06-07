/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Manages GCIP IOMMU domains and allocates/maps IOVAs.
 *
 * One can replace allocating IOVAs via Linux DMA interface which will allocate and map them to
 * the default IOMMU domain with this framework. This framework will allocate and map IOVAs to the
 * specific IOMMU domain directly. This has following two advantages:
 *
 * - Can remove the mapping time by once as it maps to the target IOMMU domain directly.
 * - IOMMU domains don't have to share the total capacity.
 *
 * GCIP IOMMU domain is implemented by utilizing multiple kinds of IOVA space pool:
 * - struct iova_domain
 * - struct gcip_mem_pool
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_IOMMU_H__
#define __GCIP_IOMMU_H__

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/idr.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>

#include <gcip/gcip-config.h>
#include <gcip/gcip-domain-pool.h>
#include <gcip/gcip-mem-pool.h>

/* Helpers to get/set @gcip_map_flags of the `gcip_iommu_domain_{map,unmap}_sg` functions. */

/* Bitfield sizes of gcip_map_flags. */
#define GCIP_MAP_FLAGS_DMA_DIRECTION_BIT_SIZE 2
#define GCIP_MAP_FLAGS_DMA_COHERENT_BIT_SIZE 1
#define GCIP_MAP_FLAGS_RESTRICT_IOVA_BIT_SIZE 1
#define GCIP_MAP_FLAGS_DMA_ATTR_BIT_SIZE 10
#define GCIP_MAP_FLAGS_MMIO_BIT_SIZE 1

/* Offsets of gcip_map_flags. */
#define GCIP_MAP_FLAGS_DMA_DIRECTION_OFFSET 0
#define GCIP_MAP_FLAGS_DMA_COHERENT_OFFSET \
	(GCIP_MAP_FLAGS_DMA_DIRECTION_OFFSET + GCIP_MAP_FLAGS_DMA_DIRECTION_BIT_SIZE)
#define GCIP_MAP_FLAGS_DMA_ATTR_OFFSET \
	(GCIP_MAP_FLAGS_DMA_COHERENT_OFFSET + GCIP_MAP_FLAGS_DMA_COHERENT_BIT_SIZE)
#define GCIP_MAP_FLAGS_RESTRICT_IOVA_OFFSET \
	(GCIP_MAP_FLAGS_DMA_ATTR_OFFSET + GCIP_MAP_FLAGS_DMA_ATTR_BIT_SIZE)
#define GCIP_MAP_FLAGS_MMIO_OFFSET \
	(GCIP_MAP_FLAGS_RESTRICT_IOVA_OFFSET + GCIP_MAP_FLAGS_RESTRICT_IOVA_BIT_SIZE)

/* Masks of gcip_map_flags. */
#define GCIP_MAP_MASK(ATTR) \
	((BIT_ULL(GCIP_MAP_FLAGS_##ATTR##_BIT_SIZE) - 1) << (GCIP_MAP_FLAGS_##ATTR##_OFFSET))
#define GCIP_MAP_MASK_DMA_DIRECTION GCIP_MAP_MASK(DMA_DIRECTION)
#define GCIP_MAP_MASK_DMA_COHERENT GCIP_MAP_MASK(DMA_COHERENT)
#define GCIP_MAP_MASK_DMA_ATTR GCIP_MAP_MASK(DMA_ATTR)
#define GCIP_MAP_MASK_RESTRICT_IOVA GCIP_MAP_MASK(RESTRICT_IOVA)
#define GCIP_MAP_MASK_MMIO GCIP_MAP_MASK(MMIO)

/* Get functions of gcip_map_flags. */
#define GCIP_MAP_FLAGS_GET_VALUE(ATTR, flags) \
	(((flags) & GCIP_MAP_MASK(ATTR)) >> (GCIP_MAP_FLAGS_##ATTR##_OFFSET))
#define GCIP_MAP_FLAGS_GET_DMA_DIRECTION(flags) GCIP_MAP_FLAGS_GET_VALUE(DMA_DIRECTION, flags)
#define GCIP_MAP_FLAGS_GET_DMA_COHERENT(flags) GCIP_MAP_FLAGS_GET_VALUE(DMA_COHERENT, flags)
#define GCIP_MAP_FLAGS_GET_DMA_ATTR(flags) GCIP_MAP_FLAGS_GET_VALUE(DMA_ATTR, flags)
#define GCIP_MAP_FLAGS_GET_RESTRICT_IOVA(flags) GCIP_MAP_FLAGS_GET_VALUE(RESTRICT_IOVA, flags)
#define GCIP_MAP_FLAGS_GET_MMIO(flags) GCIP_MAP_FLAGS_GET_VALUE(MMIO, flags)

/*
 * Bitfields of @gcip_map_flags:
 *   [1:0]   - DMA_DIRECTION:
 *               00 = DMA_BIDIRECTIONAL (host/device can write buffer)
 *               01 = DMA_TO_DEVICE     (host can write buffer)
 *               10 = DMA_FROM_DEVICE   (device can write buffer)
 *               (See [REDACTED]
 *   [2:2]   - Coherent Mapping:
 *               0 = Create non-coherent mappings of the buffer.
 *               1 = Create coherent mappings of the buffer.
 *   [12:3]  - DMA_ATTR:
 *               (See [REDACTED]
 *   [13:13] - RESTRICT_IOVA:
 *               Restrict the IOVA assignment to 32 bit address window.
 *   [14:14] - MMIO:
 *               Mapping is for device memory, use IOMMU_MMIO flag.
 *   [63:15] - RESERVED
 *               Set RESERVED bits to 0 to ensure backwards compatibility.
 *
 * One should use gcip_iommu_encode_gcip_map_flags to generate the gcip_map_flags.
 */

struct gcip_iommu_domain_ops;

/**
 * enum gcip_iommu_mapping_type - Indicates the type of the gcip_iommu_mapping.
 * GCIP_IOMMU_MAPPING_BUFFER: The mapping of a normal buffer that mapped to the domain directly.
 * GCIP_IOMMU_MAPPING_DMA_BUF: The mapping of a DMA buffer that mapped to domain with 2 steps.
 */
enum gcip_iommu_mapping_type {
	GCIP_IOMMU_MAPPING_BUFFER,
	GCIP_IOMMU_MAPPING_DMA_BUF,
};

/**
 * enum gcip_map_debug_flags - Mapping status flags for debugging, noting various attributes of the
 *                             mapping used for diagnosis of access problems.
 * GCIP_MAP_DEBUG_COW: VMA is copy-on-write, writeable mappings may have made a copy of pages
 * GCIP_MAP_DEBUG_OVRRD_RDDIR: map direction override to read-only, writable page pin failed
 * GCIP_MAP_DEBUG_VMA_NF: VMA for host addr not found, so initially assumed writeable by default
 * GCIP_MAP_DEBUG_ASSUME_RDONLY: writable page pin failed, assuming read-only
 */
enum gcip_map_debug_flags {
	GCIP_MAP_DEBUG_COW = 0x1,
	GCIP_MAP_DEBUG_OVRRD_RDDIR = 0x2,
	GCIP_MAP_DEBUG_VMA_NF = 0x4,
	GCIP_MAP_DEBUG_ASSUME_RDONLY = 0x8,
};

/* Operaters for `struct gcip_iommu_mapping`. */
struct gcip_iommu_mapping_ops {
	/*
	 * Called after the corresponding mapping of @data is unmapped and released. Since its
	 * `struct gcip_iommu_mapping` instance is released, it won't be passed to the callback.
	 *
	 * This callback is optional.
	 */
	void (*after_unmap)(void *data);
};

/**
 * struct gcip_iommu_mapping - Contains the information of sgt mapping to the domain.
 * @type: Type of the mapping.
 * @domain: IOMMU domain where the @sgt is mapped.
 * @device_address: Assigned device address.
 * @alloced_iova: Allocated IOVA.
 * @size: Size of mapped buffer.
 * @sgt: This pointer will be set to a new allocated Scatter-gather table which contains the mapping
 *       information to the given domain received from the custom IOVA allocator.
 *       If the given domain is the default domain, the pointer will be set to the sgt received from
 *       default allocator.
 *       If NULL then the mapping has no pages resident due to being trimmed.
 * @dir: The dma data direction may be adjusted due to the system or hardware limit.
 *       This value is the real one that was used for mapping and should be the same as the one
 *       encoded in gcip_map_flags.
 *       This field should be used in revert functions and dma sync functions.
 * @gcip_map_flags: The flags used to create the mapping, which should be encoded with
 *                  gcip_iommu_encode_gcip_map_flags().
 * @map_debug_flags: debug flags for reporting and diagnosis purposes.
 * @user_specified_daddr: If true, its IOVA address was specified by the user from the `*_to_iova`
 *                        mapping functions and it won't free that when it's going to be unmapped.
 *                        It's user's responsibility to manage the IOVA region.
 * @ops: User defined operators.
 * @data: User defined data.
 */
struct gcip_iommu_mapping {
	enum gcip_iommu_mapping_type type;
	struct gcip_iommu_domain *domain;
	dma_addr_t device_address;
	dma_addr_t alloced_iova;
	size_t size;
	struct sg_table *sgt;
	enum dma_data_direction dir;
	u64 gcip_map_flags;
	enum gcip_map_debug_flags map_debug_flags;
	bool user_specified_daddr;
	const struct gcip_iommu_mapping_ops *ops;
	void *data;
};

/*
 * Type of IOVA space pool that IOMMU domain will utilize.
 * Regardless of the type, its functionality will be the same. However, its implementation might be
 * different. For example, iova_domain uses red-black tree for the memory management, but gen_pool
 * uses bitmap. Therefore, their performance might be different and the kernel drivers can choose
 * which one to use according to its real use cases and the performance.
 */
enum gcip_iommu_domain_type {
	/* Uses iova_domain. */
	GCIP_IOMMU_DOMAIN_TYPE_IOVAD,
	/* Uses gcip_mem_pool which is based on gen_pool. */
	GCIP_IOMMU_DOMAIN_TYPE_MEM_POOL,
};

/*
 * IOMMU domain pool.
 *
 * It manages the pool of IOMMU domains. Also, it specifies the base address and the size of IOMMU
 * domains. Also, one can choose the data structure and algorithm of IOVA space management.
 */
struct gcip_iommu_domain_pool {
	struct device *dev;
	struct gcip_domain_pool domain_pool;
	dma_addr_t base_daddr;
	/* Will hold (base_daddr + size - 1) to prevent calculating it every IOVAD mappings. */
	dma_addr_t last_daddr;
	size_t size;
	dma_addr_t reserved_base_daddr;
	size_t reserved_size;
	size_t granule;
	bool best_fit;
	enum gcip_iommu_domain_type domain_type;
	ioasid_t min_pasid;
	ioasid_t max_pasid;
	struct ida pasid_pool;
};

/* For GCIP_IOMMU_DOMAIN_TYPE_MEM_POOL, gen_pools for 32-bit and > 32-bit spaces. */
struct gcip_iommu_domain_iova_mem_pools {
	struct gcip_mem_pool pool32;
	struct gcip_mem_pool pool64;
	/* If true then pool64 is valid, else this is a 32-bit-only pool. */
	bool pool64_valid;
};

/*
 * Wrapper of iommu_domain.
 * It has its own IOVA space pool based on iova_domain or gcip_mem_pool. One can choose one of them
 * when calling the `gcip_iommu_domain_pool_init` function. See `enum gcip_iommu_domain_type`
 * for details.
 */
struct gcip_iommu_domain {
	struct device *dev;
	struct gcip_iommu_domain_pool *domain_pool;
	struct iommu_domain *domain;
	bool default_domain;
	union {
		struct iova_domain iovad;
		struct gcip_iommu_domain_iova_mem_pools mem_pool;
	} iova_space;
	const struct gcip_iommu_domain_ops *ops;
	ioasid_t pasid; /* Only valid if attached */
};

/*
 * Holds operators which will be set according to the @domain_type.
 * These callbacks will be filled automatically when a `struct gcip_iommu_domain` is allocated.
 */
struct gcip_iommu_domain_ops {
	/* Initializes pool of @domain. */
	int (*initialize_domain)(struct gcip_iommu_domain *domain);
	/* Destroyes pool of @domain */
	void (*finalize_domain)(struct gcip_iommu_domain *domain);
	/*
	 * Enables best-fit algorithm for the memory management.
	 * Only domains which are allocated after calling this callback will be affected.
	 */
	void (*enable_best_fit_algo)(struct gcip_iommu_domain *domain);
	/* Allocates @size of IOVA space, optionally restricted to 32 bits, returns start IOVA. */
	dma_addr_t (*alloc_iova_space)(struct gcip_iommu_domain *domain, size_t size,
				       bool restrict_iova);
	/* Releases @size of buffer which was allocated to @iova. */
	void (*free_iova_space)(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size);
};

/*
 * Initializes an IOMMU domain pool.
 *
 * One can specify the base DMA address and IOVA space size via @base_daddr and @iova_space_size
 * parameters. If any of them is 0, it will try to parse "gcip-dma-window" property from the device
 * tree of @dev.
 *
 * If the base DMA address and IOVA space size are set successfully (i.e., larger than 0), IOMMU
 * domains allocated by this domain pool will have their own IOVA space pool and will map buffers
 * to their own IOMMU domain directly.
 * If either DMA address or IOVA space size are not set correctly, returns -EINVAL.
 *
 * @pool: IOMMU domain pool to be initialized.
 * @dev: Device where to parse "gcip-dma-window" property.
 * @base_addr: The base address of IOVA space. Must be greater than 0 and a multiple of @granule.
 * @iova_space_size: The size of the IOVA space. @size must be a multiple of @granule.
 * @granule: The granule when invoking the IOMMU domain pool. Must be a power of 2.
 * @num_domains: The number of IOMMU domains.
 * @domain_type: Type of the IOMMU domain.
 *
 * Returns 0 on success or negative error value.
 */
int gcip_iommu_domain_pool_init(struct gcip_iommu_domain_pool *pool, struct device *dev,
				dma_addr_t base_daddr, size_t iova_space_size, size_t granule,
				unsigned int num_domains, enum gcip_iommu_domain_type domain_type);

/*
 * Destroys an IOMMU domain pool.
 *
 * @pool: IOMMU domain pool to be destroyed.
 */
void gcip_iommu_domain_pool_destroy(struct gcip_iommu_domain_pool *pool);

/*
 * Enables the best fit algorithm for allocating an IOVA space.
 * It affects domains which are allocated after calling this function only.
 *
 * @pool: IOMMU domain pool to be enabled.
 */
void gcip_iommu_domain_pool_enable_best_fit_algo(struct gcip_iommu_domain_pool *pool);

/*
 * Allocates a GCIP IOMMU domain.
 *
 * @pool: IOMMU domain pool.
 *
 * Returns a pointer of allocated domain on success or an error pointer on failure.
 */
struct gcip_iommu_domain *gcip_iommu_domain_pool_alloc_domain(struct gcip_iommu_domain_pool *pool);

/*
 * Releases a GCIP IOMMU domain.
 *
 * Before calling this function, you must unmap all IOVAs by calling `gcip_iommu_domain_unmap{_sg}`
 * functions.
 *
 * @pool: IOMMU domain pool.
 * @domain: GCIP IOMMU domain to be released.
 */
void gcip_iommu_domain_pool_free_domain(struct gcip_iommu_domain_pool *pool,
					struct gcip_iommu_domain *domain);

/*
 * Sets the range of valid PASIDs to be used when attaching a domain
 *
 * @min: The smallest acceptable value to be assigned to an attached domain
 * @max: The largest acceptable value to be assigned to an attached domain
 */
void gcip_iommu_domain_pool_set_pasid_range(struct gcip_iommu_domain_pool *pool, ioasid_t min,
					    ioasid_t max);

/*
 * Returns the number of PASIDs can be used previously set by
 * gcip_iommu_domain_pool_set_pasid_range().
 *
 * @pool: IOMMU domain pool.
 */
static inline int gcip_iommu_domain_pool_get_num_pasid(struct gcip_iommu_domain_pool *pool)
{
	return pool->max_pasid - pool->min_pasid + 1;
}

/*
 * Returns the size of IOVA space of this pool. Does not consider reserved size.
 *
 * @pool: IOMMU domain pool.
 */
static inline size_t gcip_iommu_domain_pool_get_size(struct gcip_iommu_domain_pool *pool)
{
	return pool->size;
}

/*
 * Attaches a GCIP IOMMU domain and sets the obtained PASID
 *
 * Before calling this function, you must set the valid PASID range by calling
 * `gcip_iommu_domain_pool_set_pasid_range()`.
 *
 * @pool: IOMMU domain pool @domain was allocated from
 * @domain: The GCIP IOMMU domain to attach
 *
 * On success, @domain->pasid will be set to obtained PASID
 *
 * Returns:
 * * 0 - Domain successfully attached with a PASID
 * * -ENOSYS - This device does not support attaching multiple domains
 * * other   - Failed to attach the domain or obtain a PASID for it
 */
int gcip_iommu_domain_pool_attach_domain(struct gcip_iommu_domain_pool *pool,
					 struct gcip_iommu_domain *domain);

/*
 * Detaches a GCIP IOMMU domain
 *
 * @pool: IOMMU domain pool @domain was allocated from and attached by
 * @domain: The GCIP IOMMU domain to detach
 */
void gcip_iommu_domain_pool_detach_domain(struct gcip_iommu_domain_pool *pool,
					  struct gcip_iommu_domain *domain);

/**
 * gcip_iommu_domain_map_sgt(): Maps the scatter-gather table to the target IOMMU domain.
 * @domain: The domain that the sgt will be mapped to.
 * @sgt: The scatter-gather table to be mapped.
 * @gcip_map_flags: The gcip flags used to map the @sgt.
 *
 * This function will allocate an IOVA space and map the scatter-gather table to the address of the
 * allocated space in the target IOMMU domain. @sgt->nents will be updated to the number of mapped
 * chunks. Also, @sgt will be synced for the device.
 *
 * Return: The number of the entries that are mapped successfully.
 */
unsigned int gcip_iommu_domain_map_sgt(struct gcip_iommu_domain *domain, struct sg_table *sgt,
				       u64 *gcip_map_flags);

/**
 * gcip_iommu_domain_unmap_sgt() - Unmaps the scatter-gather table from the given domain.
 * @domain: The domain that the sgt will be unmapped from.
 * @sgt: The scatter-gather table to be unmapped.
 * @gcip_map_flags: The gcip flags used to unmap the @sgt.
 *
 * The scatter-gather table will be unmapped from @domain and synced for cpu. Also, the IOVA space
 * which was allocated from the `gcip_iommu_domain_map_sgt` function will be released.
 */
void gcip_iommu_domain_unmap_sgt(struct gcip_iommu_domain *domain, struct sg_table *sgt,
				 u64 gcip_map_flags);

/**
 * gcip_iommu_domain_map_sgt_to_iova(): Maps the scatter-gather table with specified IOVA to the
 *                                      target domain.
 *
 * @domain: The domain that the sgt will be mapped to.
 * @sgt: The scatter-gather table to be mapped.
 * @iova: The specified device address.
 * @gcip_map_flags: The gcip flags used to map the @sgt.
 *
 * This function is almost identical to gcip_iommu_domain_map_sgt() except this function maps with
 * the specified device address instead of allocating one internally.
 *
 * Note the used device address is NOT reserved by the domain, it's caller's responsibility to
 * ensure @iova does not overlap with the domain's IOVA space.
 *
 * Return: The number of the entries that are mapped successfully.
 */
unsigned int gcip_iommu_domain_map_sgt_to_iova(struct gcip_iommu_domain *domain,
					       struct sg_table *sgt, dma_addr_t iova,
					       u64 *gcip_map_flags);
/**
 * gcip_iommu_domain_unmap_sgt_from_iova(): Reverts gcip_iommu_domain_map_sgt_to_iova().
 * @domain: The domain that the sgt will be unmapped from.
 * @sgt: The scatter-gather table to be unmapped.
 * @gcip_map_flags: The gcip flags used to unmap @sgt.
 *
 * There is no @iova parameter because it is recorded in @sgt as done by
 * gcip_iommu_domain_map_sgt_to_iova().
 */
void gcip_iommu_domain_unmap_sgt_from_iova(struct gcip_iommu_domain *domain, struct sg_table *sgt,
					   u64 gcip_map_flags);

/*
 * Returns a default GCIP IOMMU domain.
 *
 * @dev: Device where to fetch the default IOMMU domain.
 */
struct gcip_iommu_domain *gcip_iommu_get_domain_for_dev(struct device *dev);

/*
 * Returns a default GCIP IOMMU domain associated with the domain pool.
 *
 * @dev: Device where to fetch the default IOMMU domain.
 * @pool: IOMMU domain pool.
 *
 * Since this domain is associated with the domain pool, it supports to be called with
 * gcip_iommu_domain_map_buffer() to map a buffer on a iova allocated by the domain pool.
 */
struct gcip_iommu_domain *
gcip_iommu_get_domain_for_dev_from_pool(struct device *dev, struct gcip_iommu_domain_pool *pool);

/**
 * gcip_iommu_encode_gcip_map_flags() - Encodes the gcip_map_flags from given arguments.
 * @dir: The DMA_DIRECTION used for mapping.
 * @coherent: Whether it is a coherent buffer or not.
 * @dma_attrs: The DMA attributes used for mapping.
 * @restrict_iova: Whether to restrict the IOVA assignment to 32 bit address window.
 * @mmio: Whether to use IOMMU_MMIO flag.
 *
 * If the direction is DMA_FROM_DEVICE(WO), it will be adjusted to DMA_BIDIRECTIONAL(RW).
 * If the direction is DMA_NONE, it will be adjusted to DMA_TO_DEVICE(RO).
 *
 * Return: The encoded gcip_map_flags.
 */
u64 gcip_iommu_encode_gcip_map_flags(enum dma_data_direction dir, bool coherent,
				     unsigned long dma_attrs, bool restrict_iova, bool mmio);

/**
 * gcip_iommu_map_flags_dma_rw() - Encodes gcip_map_flags with DMA_BIDIRECTIONAL and default values.
 *
 * Return: The encoded value.
 */
static inline u64 gcip_iommu_map_flags_dma_rw(void)
{
	return gcip_iommu_encode_gcip_map_flags(DMA_BIDIRECTIONAL, false, 0, false, false);
}

/**
 * gcip_iommu_map_flags_dma_rw() - Encodes gcip_map_flags with DMA_TO_DEVICE and default values.
 *
 * Return: The encoded value.
 */
static inline u64 gcip_iommu_map_flags_dma_ro(void)
{
	return gcip_iommu_encode_gcip_map_flags(DMA_TO_DEVICE, false, 0, false, false);
}

/**
 * gcip_iommu_domain_map_buffer() - Maps the buffer to the target IOMMU domain.
 * @domain: The desired IOMMU domain where the buffer should be mapped.
 * @host_address: The starting address of the buffer.
 * @size: The size of the buffer.
 * @gcip_map_flags: The flags used to create the mapping, which should be encoded with
 *                  gcip_iommu_encode_gcip_map_flags().
 * @pin_user_pages_lock: The lock for pinning user pages, or NULL if none.
 *
 * Following things are done in this function:
 * 1. Pin user pages.
 * 2. Allocate corresponding sg_table.
 * 3. Map the sg_table to the target domain.
 * 4. Create the desired mapping.
 *
 * Return: The mapping of the desired buffer with type GCIP_IOMMU_MAPPING_BUFFER or an error pointer
 *         on failure.
 */
struct gcip_iommu_mapping *gcip_iommu_domain_map_buffer(struct gcip_iommu_domain *domain,
							u64 host_address, size_t size,
							u64 gcip_map_flags,
							struct mutex *pin_user_pages_lock);

/*
 * This function basically works the same as the `gcip_iommu_domain_map_buffer` function but
 * receives the target @iova to map the buffer. If @iova is 0, there will be no difference.
 *
 * Note that the passed @iova won't be freed if it was non-zero when the returned mapping is going
 * to be unmapped. The life cycle of the given @iova must be managed by the user.
 */
struct gcip_iommu_mapping *gcip_iommu_domain_map_buffer_to_iova(struct gcip_iommu_domain *domain,
								u64 host_address, size_t size,
								dma_addr_t iova, u64 gcip_map_flags,
								struct mutex *pin_user_pages_lock);

/**
 * gcip_iommu_mapping_sync() - Sync a mapped buffer for either CPU or device.
 * @mapping: The pointer of the mapping instance to be synced.
 * @dev: The device that the mapping belongs to.
 * @offset: The offset, in bytes, into the mapped buffer where the region to be synced begins.
 * @size: The size, in bytes, of the region to be synced.
 * @for_cpu: True to sync for CPU access, false to sync for device access.
 *
 * This function only supports mappings with type GCIP_IOMMU_MAPPING_BUFFER.
 *
 * Return: 0 on success, or a negative errno otherwise.
 */
int gcip_iommu_mapping_sync(struct gcip_iommu_mapping *mapping, struct device *dev, u64 offset,
			    u64 size, bool for_cpu);

/**
 * gcip_iommu_mapping_trim() - Trim a buffer mapping, unpinning pages and unmapping from TPU,
 *                             but leaving the IOVA allocation and mapping metadata in place.
 * @mapping: The mapping instance to be trimmed.
 *
 * @mapping->sgt is set to NULL, indicating no pages currently mapped to TPU (or pinned).
 * The full mapping may be restored via gcip_iommu_mapping_remap().
 *
 * Only implemented for buffer, not dma-buf, mappings.
 */
void gcip_iommu_mapping_trim(struct gcip_iommu_mapping *mapping);

/**
 * gcip_iommu_mapping_remap() - Remap a previously trimmed buffer mapping, re-pinning pages and
 *                              remapping to the TPU at the same IOVA as previous.
 * @mapping: The mapping instance to be remapped.
 * @pin_user_pages_lock: The lock for pinning user pages, or NULL if none.
 *
 * @mapping->sgt is set to the new scatter-gather list.
 *
 * Only implemented for buffer, not dma-buf, mappings.
 */
int gcip_iommu_mapping_remap(struct gcip_iommu_mapping *mapping, struct mutex *pin_user_pages_lock);

/**
 * gcip_iommu_alloc_iova() - Allocates IOVA with size @size.
 * @domain: The GCIP domain to allocate IOVA.
 * @size: Size in bytes.
 * @gcip_map_flags: The flags used to create the mapping, which should be encoded with
 *                  gcip_iommu_encode_gcip_map_flags().
 *
 * Return: The allocated IOVA. Returns 0 on failure.
 */
dma_addr_t gcip_iommu_alloc_iova(struct gcip_iommu_domain *domain, size_t size, u64 gcip_map_flags);

/**
 * gcip_iommu_free_iova() - Frees IOVA allocated by gcip_iommu_alloc_iova().
 * @domain: The GCIP domain @iova allocated from.
 * @iova: The IOVA returned by gcip_iommu_alloc_iova().
 * @size: Size in bytes.
 */
void gcip_iommu_free_iova(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size);

static inline void gcip_iommu_mapping_set_ops(struct gcip_iommu_mapping *mapping,
					      const struct gcip_iommu_mapping_ops *ops)
{
	mapping->ops = ops;
}

static inline void gcip_iommu_mapping_set_data(struct gcip_iommu_mapping *mapping, void *data)
{
	mapping->data = data;
}

static inline size_t gcip_iommu_domain_granule(struct gcip_iommu_domain *domain)
{
	if (unlikely(domain->default_domain))
		return PAGE_SIZE;
	return domain->domain_pool->granule;
}

/**
 * gcip_iommu_map() - Maps the desired mappings to the domain.
 * @domain: The GCIP domain to be mapped to.
 * @iova: The device address.
 * @paddr: The target address to be mapped to.
 * @size: Map size in bytes.
 * @gcip_map_flags: The flags used to create the mapping, which should be encoded with
 *                  gcip_iommu_encode_gcip_map_flags().
 *
 * Return: 0 on success, otherwise a negative errno.
 */
int gcip_iommu_map(struct gcip_iommu_domain *domain, dma_addr_t iova, phys_addr_t paddr,
		   size_t size, u64 gcip_map_flags);
/* Reverts gcip_iommu_map(). */
void gcip_iommu_unmap(struct gcip_iommu_domain *domain, dma_addr_t iova, size_t size);

/* TODO(b/455299283): Hide the following functions to static after refactor. */

/**
 * gcip_iommu_mapping_unmap_buffer() - Reverts gcip_iommu_domain_map_buffer
 * @mapping: The target mapping that should be unmapped.
 */
void gcip_iommu_mapping_unmap_buffer(struct gcip_iommu_mapping *mapping);

#endif /* __GCIP_IOMMU_H__ */
