/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NOTE(mainline): minimal in-module reimplementation of the Android dma-buf
 * heaps page-pool helper library, whose sources are absent from the mainline
 * tree.  It is a simple per-order free list of pre-allocated (and pre-zeroed)
 * pages used to avoid the cost of high-order page allocation on the hot path.
 *
 * Only the subset of the API used by the Samsung dma-buf heaps is implemented.
 */
#ifndef DMABUF_PAGE_POOL_H
#define DMABUF_PAGE_POOL_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/mm_types.h>
#include <linux/types.h>

/* high-order pages are kept on a separate (smaller) watermark than order-0 */
enum {
	POOL_LOWPAGE,	/* movable / reclaimable order-0 style pages */
	POOL_HIGHPAGE,
	POOL_TYPE_SIZE,
};

/**
 * struct dmabuf_page_pool - a per-order pool of free pages
 * @count:	number of cached pages per pool type
 * @items:	cached page free lists per pool type
 * @mutex:	protects @count and @items
 * @gfp_mask:	gfp flags used when refilling the pool
 * @order:	allocation order of pages kept in this pool
 * @list:	linkage into the global list of pools (for shrinking)
 */
struct dmabuf_page_pool {
	int count[POOL_TYPE_SIZE];
	struct list_head items[POOL_TYPE_SIZE];
	struct mutex mutex;
	gfp_t gfp_mask;
	unsigned int order;
	struct list_head list;
};

struct dmabuf_page_pool *dmabuf_page_pool_create(gfp_t gfp_mask,
						 unsigned int order);
void dmabuf_page_pool_destroy(struct dmabuf_page_pool *pool);
struct page *dmabuf_page_pool_alloc(struct dmabuf_page_pool *pool);
void dmabuf_page_pool_free(struct dmabuf_page_pool *pool, struct page *page);

/* total size, in bytes, of the pages currently cached in @pool */
unsigned long dmabuf_page_pool_get_size(struct dmabuf_page_pool *pool);

#endif /* DMABUF_PAGE_POOL_H */
