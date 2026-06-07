// SPDX-License-Identifier: GPL-2.0
/*
 * NOTE(mainline): minimal in-module reimplementation of the Android dma-buf
 * heaps helper libraries (deferred-free-helper and page_pool) whose sources
 * are absent from the mainline tree.  The Samsung dma-buf heaps depend on
 * these helpers; this file provides just the subset of their APIs that the
 * Samsung heaps actually call.
 *
 * deferred-free-helper:
 *   A single worker thread drains a global list of freed buffers, invoking each
 *   buffer's free callback with DF_NORMAL.  A shrinker drains the same list
 *   under memory pressure, passing DF_UNDER_PRESSURE so callbacks can skip
 *   page zeroing / pool caching.
 *
 * page_pool:
 *   A simple per-order free list of pre-zeroed pages, registered with the same
 *   shrinker so cached pages are reclaimed first under pressure.
 *
 * This is functionally equivalent to the upstream Android helpers for the
 * Samsung heaps' usage, but is intentionally minimal and should be reviewed
 * before relying on it for production.
 */

#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/shrinker.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/swap.h>
#include <linux/wait.h>

#include <heaps/deferred-free-helper.h>
#include <heaps/page_pool.h>

/* ------------------------------------------------------------------ */
/* deferred-free helper                                               */
/* ------------------------------------------------------------------ */

static LIST_HEAD(free_list);
static size_t list_nr_pages;
static DEFINE_SPINLOCK(free_list_lock);
static DECLARE_WAIT_QUEUE_HEAD(freelist_waitqueue);
static struct task_struct *freelist_task;

void deferred_free(struct deferred_freelist_item *item,
		   free_callback free,
		   size_t nr_pages)
{
	unsigned long flags;

	item->free = free;
	item->nr_pages = nr_pages;

	spin_lock_irqsave(&free_list_lock, flags);
	list_add_tail(&item->list, &free_list);
	list_nr_pages += nr_pages;
	spin_unlock_irqrestore(&free_list_lock, flags);

	wake_up(&freelist_waitqueue);
}
EXPORT_SYMBOL_GPL(deferred_free);

/*
 * Pop one item off the free list and run its callback. Returns the number of
 * pages freed, or 0 if the list was empty.
 */
static size_t free_one_item(enum df_reason reason)
{
	unsigned long flags;
	size_t nr_pages;
	struct deferred_freelist_item *item;

	spin_lock_irqsave(&free_list_lock, flags);
	if (list_empty(&free_list)) {
		spin_unlock_irqrestore(&free_list_lock, flags);
		return 0;
	}
	item = list_first_entry(&free_list, struct deferred_freelist_item, list);
	list_del(&item->list);
	nr_pages = item->nr_pages;
	list_nr_pages -= nr_pages;
	spin_unlock_irqrestore(&free_list_lock, flags);

	item->free(item, reason);
	return nr_pages;
}

static unsigned long get_freelist_nr_pages(void)
{
	unsigned long nr_pages;
	unsigned long flags;

	spin_lock_irqsave(&free_list_lock, flags);
	nr_pages = list_nr_pages;
	spin_unlock_irqrestore(&free_list_lock, flags);
	return nr_pages;
}

static int deferred_free_thread(void *data)
{
	while (!kthread_should_stop()) {
		wait_event_freezable(freelist_waitqueue,
				     get_freelist_nr_pages() > 0 ||
				     kthread_should_stop());

		while (free_one_item(DF_NORMAL))
			;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* page pool                                                          */
/* ------------------------------------------------------------------ */

static LIST_HEAD(pool_list);
static DEFINE_MUTEX(pool_list_lock);

static inline unsigned int pool_type(struct dmabuf_page_pool *pool)
{
	return pool->order ? POOL_HIGHPAGE : POOL_LOWPAGE;
}

static struct page *dmabuf_page_pool_alloc_pages(struct dmabuf_page_pool *pool)
{
	return alloc_pages(pool->gfp_mask, pool->order);
}

static void dmabuf_page_pool_free_pages(struct dmabuf_page_pool *pool,
					struct page *page)
{
	__free_pages(page, pool->order);
}

static void dmabuf_page_pool_add(struct dmabuf_page_pool *pool,
				 struct page *page)
{
	unsigned int type = pool_type(pool);

	mutex_lock(&pool->mutex);
	list_add_tail(&page->lru, &pool->items[type]);
	pool->count[type]++;
	mutex_unlock(&pool->mutex);
}

static struct page *dmabuf_page_pool_remove(struct dmabuf_page_pool *pool,
					    unsigned int type)
{
	struct page *page;

	mutex_lock(&pool->mutex);
	page = list_first_entry_or_null(&pool->items[type], struct page, lru);
	if (page) {
		pool->count[type]--;
		list_del(&page->lru);
	}
	mutex_unlock(&pool->mutex);

	return page;
}

static struct page *dmabuf_page_pool_fetch(struct dmabuf_page_pool *pool)
{
	struct page *page;

	page = dmabuf_page_pool_remove(pool, POOL_HIGHPAGE);
	if (!page)
		page = dmabuf_page_pool_remove(pool, POOL_LOWPAGE);

	return page;
}

struct page *dmabuf_page_pool_alloc(struct dmabuf_page_pool *pool)
{
	struct page *page;

	if (WARN_ON(!pool))
		return NULL;

	page = dmabuf_page_pool_fetch(pool);
	if (!page)
		page = dmabuf_page_pool_alloc_pages(pool);

	return page;
}
EXPORT_SYMBOL_GPL(dmabuf_page_pool_alloc);

void dmabuf_page_pool_free(struct dmabuf_page_pool *pool, struct page *page)
{
	if (WARN_ON(pool->order != compound_order(page)))
		return;

	dmabuf_page_pool_add(pool, page);
}
EXPORT_SYMBOL_GPL(dmabuf_page_pool_free);

unsigned long dmabuf_page_pool_get_size(struct dmabuf_page_pool *pool)
{
	int i;
	unsigned long num_pages = 0;

	mutex_lock(&pool->mutex);
	for (i = 0; i < POOL_TYPE_SIZE; i++)
		num_pages += pool->count[i];
	mutex_unlock(&pool->mutex);

	/* size in bytes */
	return num_pages * (PAGE_SIZE << pool->order);
}
EXPORT_SYMBOL_GPL(dmabuf_page_pool_get_size);

struct dmabuf_page_pool *dmabuf_page_pool_create(gfp_t gfp_mask,
						 unsigned int order)
{
	struct dmabuf_page_pool *pool;
	int i;

	pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return NULL;

	for (i = 0; i < POOL_TYPE_SIZE; i++) {
		pool->count[i] = 0;
		INIT_LIST_HEAD(&pool->items[i]);
	}
	pool->gfp_mask = gfp_mask | __GFP_COMP;
	pool->order = order;
	mutex_init(&pool->mutex);

	mutex_lock(&pool_list_lock);
	list_add(&pool->list, &pool_list);
	mutex_unlock(&pool_list_lock);

	return pool;
}
EXPORT_SYMBOL_GPL(dmabuf_page_pool_create);

void dmabuf_page_pool_destroy(struct dmabuf_page_pool *pool)
{
	struct page *page;
	int i;

	mutex_lock(&pool_list_lock);
	list_del(&pool->list);
	mutex_unlock(&pool_list_lock);

	for (i = 0; i < POOL_TYPE_SIZE; i++) {
		while ((page = dmabuf_page_pool_remove(pool, i)))
			dmabuf_page_pool_free_pages(pool, page);
	}

	kfree(pool);
}
EXPORT_SYMBOL_GPL(dmabuf_page_pool_destroy);

/* ------------------------------------------------------------------ */
/* shared shrinker (page pool + deferred free list)                   */
/* ------------------------------------------------------------------ */

static unsigned long pool_total_pages(void)
{
	struct dmabuf_page_pool *pool;
	unsigned long total = 0;

	mutex_lock(&pool_list_lock);
	list_for_each_entry(pool, &pool_list, list) {
		int i;

		for (i = 0; i < POOL_TYPE_SIZE; i++)
			total += pool->count[i] << pool->order;
	}
	mutex_unlock(&pool_list_lock);

	return total;
}

static unsigned long dma_heap_shrink_count(struct shrinker *shrinker,
					   struct shrink_control *sc)
{
	unsigned long count = pool_total_pages() + get_freelist_nr_pages();

	return count ? count : SHRINK_EMPTY;
}

static unsigned long pool_shrink_one(struct dmabuf_page_pool *pool)
{
	struct page *page;
	int i;

	for (i = 0; i < POOL_TYPE_SIZE; i++) {
		page = dmabuf_page_pool_remove(pool, i);
		if (page) {
			dmabuf_page_pool_free_pages(pool, page);
			return 1UL << pool->order;
		}
	}

	return 0;
}

static unsigned long dma_heap_shrink_scan(struct shrinker *shrinker,
					  struct shrink_control *sc)
{
	struct dmabuf_page_pool *pool;
	unsigned long freed = 0;

	/* First, drain the deferred free list (frees whole buffers). */
	while (freed < sc->nr_to_scan) {
		size_t nr = free_one_item(DF_UNDER_PRESSURE);

		if (!nr)
			break;
		freed += nr;
	}

	/* Then trim cached pages from the pools. */
	mutex_lock(&pool_list_lock);
	while (freed < sc->nr_to_scan) {
		unsigned long progress = 0;

		list_for_each_entry(pool, &pool_list, list) {
			progress += pool_shrink_one(pool);
			if (freed + progress >= sc->nr_to_scan)
				break;
		}
		if (!progress)
			break;
		freed += progress;
	}
	mutex_unlock(&pool_list_lock);

	return freed ? freed : SHRINK_STOP;
}

static struct shrinker *dma_heap_shrinker;

/*
 * Wired into the Samsung dma-buf heap module's init/exit (samsung_heap.c)
 * rather than a separate module_init, since this file is linked into the same
 * samsung_dma_heap module.
 */
int dma_heap_helpers_init(void)
{
	freelist_task = kthread_run(deferred_free_thread, NULL,
				    "%s", "dmabuf-deferred-free-worker");
	if (IS_ERR(freelist_task)) {
		pr_err("%s: failed to start deferred free worker\n", __func__);
		return PTR_ERR(freelist_task);
	}

	dma_heap_shrinker = shrinker_alloc(0, "dmabuf-heap-helpers");
	if (!dma_heap_shrinker) {
		kthread_stop(freelist_task);
		freelist_task = NULL;
		return -ENOMEM;
	}
	dma_heap_shrinker->count_objects = dma_heap_shrink_count;
	dma_heap_shrinker->scan_objects = dma_heap_shrink_scan;
	dma_heap_shrinker->seeks = DEFAULT_SEEKS;
	dma_heap_shrinker->batch = 0;
	shrinker_register(dma_heap_shrinker);

	return 0;
}

void dma_heap_helpers_exit(void)
{
	if (dma_heap_shrinker) {
		shrinker_free(dma_heap_shrinker);
		dma_heap_shrinker = NULL;
	}
	if (!IS_ERR_OR_NULL(freelist_task)) {
		kthread_stop(freelist_task);
		freelist_task = NULL;
	}
}
