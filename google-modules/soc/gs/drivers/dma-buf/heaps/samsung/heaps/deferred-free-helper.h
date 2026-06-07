/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NOTE(mainline): minimal in-module reimplementation of the Android dma-buf
 * heaps "deferred free" helper library, whose sources are absent from the
 * mainline tree.  It provides a worker thread that drains a list of freed
 * buffers, calling each buffer's free callback off the hot free path.  Under
 * memory pressure a shrinker drives the same list and passes DF_UNDER_PRESSURE
 * so the callback can skip page zeroing / pool caching.
 *
 * Only the subset of the API used by the Samsung dma-buf heaps is implemented.
 */
#ifndef DEFERRED_FREE_HELPER_H
#define DEFERRED_FREE_HELPER_H

#include <linux/list.h>

/**
 * enum df_reason - context a deferred-free callback is invoked in
 * @DF_NORMAL:		drained by the deferred-free worker thread
 * @DF_UNDER_PRESSURE:	drained by the shrinker due to memory pressure
 */
enum df_reason {
	DF_NORMAL,
	DF_UNDER_PRESSURE,
};

struct deferred_freelist_item;

typedef void (*free_callback)(struct deferred_freelist_item *item,
			      enum df_reason reason);

/**
 * struct deferred_freelist_item - an entry on the deferred free list
 * @nr_pages:	number of pages owned by this item (accounted by the shrinker)
 * @free:	callback that performs the actual free
 * @list:	linkage into the global free list
 */
struct deferred_freelist_item {
	size_t nr_pages;
	free_callback free;
	struct list_head list;
};

/**
 * deferred_free - queue an item to be freed by the deferred-free worker
 * @item:	caller-owned freelist item (embedded in the buffer)
 * @free:	callback invoked to perform the free
 * @nr_pages:	number of pages this item accounts for
 */
void deferred_free(struct deferred_freelist_item *item,
		   free_callback free,
		   size_t nr_pages);

#endif /* DEFERRED_FREE_HELPER_H */
