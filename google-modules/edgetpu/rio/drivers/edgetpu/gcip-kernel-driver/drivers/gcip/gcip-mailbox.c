// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP Mailbox Interface.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <asm/barrier.h>

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h> /* memcpy */
#include <linux/wait.h>

#include <gcip/gcip-mailbox.h>

#if IS_ENABLED(CONFIG_GCIP_TEST)
#include "unittests/helper/gcip-mailbox-controller.h"

#define TEST_TRIGGER_TIMEOUT_RACE(awaiter, lock) \
	gcip_mailbox_controller_trigger_timeout_race(awaiter, lock)
#define TEST_FLUSH_TIMEOUT_RACE(awaiter) gcip_mailbox_controller_flush_timeout_race(awaiter)
#define TEST_WAIT_FIRMWARE_WORK() gcip_mailbox_controller_wait_firmware_work()
#define TEST_NOTIFY_TIMEOUT_HANDLER_START() gcip_mailbox_controller_notify_timeout_handler_start()
#else
#define TEST_TRIGGER_TIMEOUT_RACE(...)
#define TEST_FLUSH_TIMEOUT_RACE(...)
#define TEST_WAIT_FIRMWARE_WORK(...)
#define TEST_NOTIFY_TIMEOUT_HANDLER_START(...)
#endif

#define GET_TX_QUEUE_TAIL() mailbox->ops->get_tx_queue_tail(mailbox)
#define INC_TX_QUEUE_TAIL(inc) mailbox->ops->inc_tx_queue_tail(mailbox, inc)
#define ACQUIRE_TX_QUEUE_LOCK(try, atomic) mailbox->ops->acquire_tx_queue_lock(mailbox, try, atomic)
#define RELEASE_TX_QUEUE_LOCK() mailbox->ops->release_tx_queue_lock(mailbox)

#define GET_CMD_ELEM_SEQ(cmd) mailbox->ops->get_cmd_elem_seq(mailbox, cmd)
#define SET_CMD_ELEM_SEQ(cmd, seq) mailbox->ops->set_cmd_elem_seq(mailbox, cmd, seq)

#define GET_RX_QUEUE_SIZE() mailbox->ops->get_rx_queue_size(mailbox)
#define GET_RX_QUEUE_HEAD() mailbox->ops->get_rx_queue_head(mailbox)
#define INC_RX_QUEUE_HEAD(inc) mailbox->ops->inc_rx_queue_head(mailbox, inc)
#define GET_RX_QUEUE_TAIL() mailbox->ops->get_rx_queue_tail(mailbox)
#define ACQUIRE_RX_QUEUE_LOCK(try, atomic) mailbox->ops->acquire_rx_queue_lock(mailbox, try, atomic)
#define RELEASE_RX_QUEUE_LOCK() mailbox->ops->release_rx_queue_lock(mailbox)

#define GET_RESP_ELEM_SEQ(resp) mailbox->ops->get_resp_elem_seq(mailbox, resp)
#define SET_RESP_ELEM_SEQ(resp, seq) mailbox->ops->set_resp_elem_seq(mailbox, resp, seq)

#define IS_BLOCK_OFF() (mailbox->ops->is_block_off ? mailbox->ops->is_block_off(mailbox) : false)

struct gcip_mailbox_wait_list_elem {
	struct list_head list;
	struct gcip_mailbox_async_resp *async_resp;
	struct gcip_mailbox_resp_awaiter *awaiter;
};

/**
 * gcip_mailbox_awaiter_create() - Creates the awaiter of the response.
 * @mailbox: The pointer to the gcip mailbox to interact with its interfaces.
 * @resp: The pointer to the response to be filled.
 * @data: The user-defined data.
 *
 * The created awaiter is expected to be released with the gcip_mailbox_awaiter_put().
 *
 * Return: The pointer to the awaiter on success, or a negative errno otherwise.
 */
static struct gcip_mailbox_resp_awaiter *gcip_mailbox_awaiter_create(struct gcip_mailbox *mailbox,
								     void *resp, void *data)
{
	struct gcip_mailbox_resp_awaiter *awaiter;

	awaiter = kzalloc(sizeof(*awaiter), GFP_KERNEL);
	if (!awaiter)
		return ERR_PTR(-ENOMEM);

	awaiter->async_resp.resp = resp;
	awaiter->mailbox = mailbox;
	awaiter->data = data;
	awaiter->release_data = mailbox->ops->release_awaiter_data;
	kref_init(&awaiter->kref);
	init_completion(&awaiter->handled);

	return awaiter;
}

/**
 * gcip_mailbox_awaiter_destroy() - Destroys the awaiter of the response.
 * @kref: The pointer to the kref of the awaiter to be destroyed.
 *
 * This function will call the `release_data` callback if the provided.
 * This function should only be called by gcip_mailbox_awaiter_put().
 */
static void gcip_mailbox_awaiter_destroy(struct kref *kref)
{
	struct gcip_mailbox_resp_awaiter *awaiter =
		container_of(kref, struct gcip_mailbox_resp_awaiter, kref);

	if (awaiter->release_data)
		awaiter->release_data(awaiter->data);

	kfree(awaiter);
}

/**
 * gcip_mailbox_awaiter_get() - Gets the reference count of the awaiter.
 * @awaiter: The awaiter to be got.
 *
 * Return: The pointer to the awaiter.
 */
static inline struct gcip_mailbox_resp_awaiter *
gcip_mailbox_awaiter_get(struct gcip_mailbox_resp_awaiter *awaiter)
{
	kref_get(&awaiter->kref);

	return awaiter;
}

void gcip_mailbox_awaiter_put(struct gcip_mailbox_resp_awaiter *awaiter)
{
	kref_put(&awaiter->kref, gcip_mailbox_awaiter_destroy);
}

/**
 * does_response_match_waiter() - Checks if the incoming response matches the waiter.
 * @mailbox: The pointer to the gcip mailbox to interact with its interfaces.
 * @incoming_resp: The pointer to the incoming response.
 * @waiting_resp: The pointer to the waiting response.
 *
 * Return: Whether or not the incoming response matches the waiter.
 */
static inline bool does_response_match_waiter(struct gcip_mailbox *mailbox, void *incoming_resp,
					      void *waiting_resp)
{
	if (mailbox->ops->does_response_match_waiter)
		return mailbox->ops->does_response_match_waiter(mailbox, incoming_resp,
								waiting_resp);

	return GET_RESP_ELEM_SEQ(incoming_resp) == GET_RESP_ELEM_SEQ(waiting_resp);
}

/**
 * gcip_mailbox_del_wait_resp() - Deletes the response pushed with gcip_mailbox_push_wait_resp().
 * @mailbox: The pointer to the gcip mailbox to interact with its interfaces.
 * @async_resp: The pointer to the async response to be removed.
 *
 * This is used when the kernel want to cancel the awaiter of the response.
 *
 * There are 3 cases that the awaiter can be removed from the wait list:
 * 1. Response arrived, gcip_mailbox_handle_response() will delete the awaiter form wait list
 * 2. Response timed out, gcip_mailbox_async_cmd_timeout_work() will trigger this function.
 * 3. Response canceled, gcip_mailbox_cancel_awaiter() and gcip_mailbox_cancel_awaiter_all() will
 *    trigger this function.
 *
 * Return: Whether or not the awaiter of @async_resp is deleted from the wait list successfully.
 */
static bool gcip_mailbox_del_wait_resp(struct gcip_mailbox *mailbox,
				       struct gcip_mailbox_async_resp *async_resp)
{
	struct gcip_mailbox_wait_list_elem *cur;
	unsigned long flags;
	bool removed = false;

	spin_lock_irqsave(&mailbox->wait_list->list_lock, flags);
	list_for_each_entry(cur, &mailbox->wait_list->list, list) {
		if (!does_response_match_waiter(mailbox, async_resp->resp, cur->async_resp->resp))
			continue;

		list_del(&cur->list);

		/*
		 * If the awaiter is not NULL, it means that the response is asynchronous.
		 * Decrease the reference count of the awaiter acquired by the wait list.
		 */
		if (cur->awaiter)
			gcip_mailbox_awaiter_put(cur->awaiter);

		kfree(cur);
		removed = true;

		break;
	}
	spin_unlock_irqrestore(&mailbox->wait_list->list_lock, flags);

	return removed;
}

/**
 * gcip_mailbox_push_wait_resp() - Adds @resp to @mailbox->wait_list.
 * @mailbox: The pointer to the gcip mailbox to interact with its interfaces.
 * @async_resp: The pointer to the async response to be added.
 * @awaiter: The pointer to the awaiter of the async response.
 * @atomic: Whether or not the function is run under atomic context.
 *
 * If @awaiter is not NULL, the @resp is asynchronous. Otherwise, the @resp is synchronous.
 *
 * Context: Depends on @atomic.
 * Return: 0 on success, or a negative errno otherwise.
 */
static int gcip_mailbox_push_wait_resp(struct gcip_mailbox *mailbox,
				       struct gcip_mailbox_async_resp *async_resp,
				       struct gcip_mailbox_resp_awaiter *awaiter, bool atomic)
{
	struct gcip_mailbox_wait_list_elem *entry;
	unsigned long flags;
	int ret;

	entry = kzalloc(sizeof(*entry), atomic ? GFP_ATOMIC : GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	if (mailbox->ops->before_enqueue_wait_list) {
		ret = mailbox->ops->before_enqueue_wait_list(mailbox, async_resp->resp, awaiter);
		if (ret) {
			kfree(entry);
			return ret;
		}
	}

	/* Increase a reference of the awaiter for the wait list. */
	if (awaiter)
		entry->awaiter = gcip_mailbox_awaiter_get(awaiter);

	entry->async_resp = async_resp;
	spin_lock_irqsave(&mailbox->wait_list->list_lock, flags);
	list_add_tail(&entry->list, &mailbox->wait_list->list);
	spin_unlock_irqrestore(&mailbox->wait_list->list_lock, flags);

	return 0;
}

/**
 * should_maintain_seq_num() - Checks if the sequence number of the command should be maintained.
 * @mode: The operating mode of the mailbox.
 *
 * Return: Whether or not the sequence number of the command should be maintained.
 */
static inline bool should_maintain_seq_num(u8 mode)
{
	if (mode & GCIP_MAILBOX_MODE_SEQ_EXTERNAL)
		return false;

	return ((mode & GCIP_MAILBOX_MODE_TX_CMD) && (mode & GCIP_MAILBOX_MODE_RX_RSP));
}

/*
 * Pushes @cmd to the command queue of mailbox and returns. @resp should be passed if the request
 * is synchronous and want to get the response. If @resp is NULL even though the request is
 * synchronous, the @cmd will be put into the queue, but the caller may not wait the response and
 * ignore it. If the request is async, @awaiter should be passed too.
 */
static int gcip_mailbox_enqueue_cmd(struct gcip_mailbox *mailbox, void *cmd,
				    struct gcip_mailbox_async_resp *async_resp,
				    struct gcip_mailbox_resp_awaiter *awaiter,
				    u32 gcip_mailbox_cmd_flags)
{
	void *cmd_ptr;
	int idx;
	int ret = 0;
	bool atomic = false;

	ACQUIRE_TX_QUEUE_LOCK(false, &atomic);

	if (should_maintain_seq_num(mailbox->mode))
		SET_CMD_ELEM_SEQ(cmd, atomic64_read(&mailbox->cur_seq));

	/* Wait until the cmd queue has a space for putting cmd. */
	ret = mailbox->ops->wait_for_tx_queue_not_full(mailbox);
	if (ret)
		goto out;

	if (async_resp->resp) {
		/* Adds @resp to the wait_list only if the cmd can be pushed successfully. */
		SET_RESP_ELEM_SEQ(async_resp->resp, GET_CMD_ELEM_SEQ(cmd));
		async_resp->waiting = true;
		ret = gcip_mailbox_push_wait_resp(mailbox, async_resp, awaiter, atomic);
		if (ret)
			goto out;
	}

	/* Calculate the address of the command in the command queue. */
	idx = CIRC_QUEUE_REAL_INDEX(GET_TX_QUEUE_TAIL(), mailbox->queue_wrap_bit);
	cmd_ptr = mailbox->tx_queue + mailbox->tx_elem_size * idx;
	memcpy(cmd_ptr, cmd, mailbox->tx_elem_size);

	INC_TX_QUEUE_TAIL(1);

	if (mailbox->ops->after_enqueue_cmd) {
		ret = mailbox->ops->after_enqueue_cmd(mailbox, cmd);
		if (ret) {
			/*
			 * Currently, as both DSP and EdgeTPU never return errors, do nothing
			 * here. We can decide later how to rollback the status such as
			 * `cmd_queue_tail` when the possibility of returning an error is raised.
			 */
			dev_warn(mailbox->dev,
				 "after_enqueue_cmd returned an error, but not handled: ret=%d",
				 ret);
			goto out;
		}
	}

	if (should_maintain_seq_num(mailbox->mode))
		atomic64_inc(&mailbox->cur_seq);

out:
	RELEASE_TX_QUEUE_LOCK();
	if (ret)
		dev_dbg(mailbox->dev, "%s: ret=%d", __func__, ret);

	return ret;
}

/*
 * Handler of a response.
 * Pops the wait_list until the sequence number of @resp is found, and copies @resp to the found
 * entry.
 */
static void gcip_mailbox_handle_response(struct gcip_mailbox *mailbox, void *resp)
{
	struct gcip_mailbox_wait_list_elem *cur, *nxt;
	struct gcip_mailbox_resp_awaiter *awaiter = NULL;
	unsigned long flags;

	spin_lock_irqsave(&mailbox->wait_list->list_lock, flags);

	list_for_each_entry_safe(cur, nxt, &mailbox->wait_list->list, list) {
		if (!does_response_match_waiter(mailbox, resp, cur->async_resp->resp))
			continue;

		memcpy(cur->async_resp->resp, resp, mailbox->rx_elem_size);

		/*
		 * Paired with smp_rmb() in gcip_mailbox_send_cmd().  Ensure all writes to
		 * *cur->async_resp->resp are complete before setting cur->async_resp->status,
		 * which tells waiters the async response is complete.
		 */
		smp_wmb();
		cur->async_resp->waiting = false;
		list_del(&cur->list);
		awaiter = cur->awaiter;
		if (awaiter) {
			/*
			 * The timedout handler will be fired, but pended by waiting for acquiring
			 * the wait_list->list_lock.
			 */
			TEST_TRIGGER_TIMEOUT_RACE(awaiter, &mailbox->wait_list->list_lock);
		}
		kfree(cur);
		break;
	}

	spin_unlock_irqrestore(&mailbox->wait_list->list_lock, flags);

	if (!awaiter)
		return;

	/* The reference acquired by the timeout work will be released implicitly. */
	gcip_mailbox_cancel_timeout_work(awaiter);

	if (mailbox->ops->handle_awaiter_arrived)
		mailbox->ops->handle_awaiter_arrived(mailbox, awaiter);

	complete_all(&awaiter->handled);

	/* Make sure the timedout handler is finished before decreasing the ref count. */
	TEST_FLUSH_TIMEOUT_RACE(awaiter);

	/* Remove the reference of the arrived handler. */
	gcip_mailbox_awaiter_put(awaiter);
}

/**
 * gcip_mailbox_handle_rx_elem() - Handles the received element according to its type.
 * @mailbox: The pointer to the gcip mailbox to interact with its interfaces.
 * @elem: The received element to be handled.
 *
 * If both GCIP_MAILBOX_MODE_RX_CMD and GCIP_MAILBOX_MODE_RX_RSP are on, the mailboix ops
 * is_rx_elem_reversed must be defined and used here.
 *
 * If only GCIP_MAILBOX_MODE_RX_CMD or GCIP_MAILBOX_MODE_RX_RSP is on, we can assign the handler
 * according to the operating mode.
 *
 * Context: normal and in_interrupt().
 */
static void gcip_mailbox_handle_rx_elem(struct gcip_mailbox *mailbox, void *elem)
{
	bool is_reversed_cmd = mailbox->ops->is_rx_elem_reversed ?
				       mailbox->ops->is_rx_elem_reversed(mailbox, elem) :
				       (mailbox->mode & GCIP_MAILBOX_MODE_RX_CMD);

	if (is_reversed_cmd)
		mailbox->ops->handle_reversed_command(mailbox, elem);
	else
		gcip_mailbox_handle_response(mailbox, elem);
}

/*
 * Fetches elements in the response queue.
 *
 * Returns the pointer of fetched response elements.
 * @total_ptr will be the number of elements fetched.
 *
 * If @trylock is true, the function will return right away if the lock is held by others which
 * means that the response queue is being consumed by other threads. Otherwise, it will use the
 * normal lock to guarantee that all responses have been handled when the function returns.
 *
 * Returns -ENOMEM if failed on memory allocation.
 * Returns NULL if the response queue is empty or there is another worker fetching responses.
 */
static void *gcip_mailbox_fetch_responses(struct gcip_mailbox *mailbox, u32 *total_ptr,
					  bool trylock)
{
	u32 head;
	u32 tail;
	u32 count;
	u32 i;
	u32 j;
	u32 total = 0;
	const u32 wrap_bit = mailbox->queue_wrap_bit;
	const u32 size = GET_RX_QUEUE_SIZE();
	const u32 elem_size = mailbox->rx_elem_size;
	void *ret = NULL; /* Array of responses. */
	void *prev_ptr = NULL; /* Temporary pointer to realloc ret. */
	bool atomic = false;

	/* The block is off or someone is working on consuming - we can leave early. */
	if (IS_BLOCK_OFF() || !ACQUIRE_RX_QUEUE_LOCK(trylock, &atomic))
		goto out;

	head = GET_RX_QUEUE_HEAD();
	/* Loops until our head equals to CSR tail. */
	while (1) {
		tail = GET_RX_QUEUE_TAIL();
		/*
		 * Make sure the CSR is read and reported properly by checking if any bit higher
		 * than wrap_bit is set and if the tail exceeds resp_queue size.
		 */
		if (unlikely(tail & ~CIRC_QUEUE_VALID_MASK(wrap_bit) ||
			     CIRC_QUEUE_REAL_INDEX(tail, wrap_bit) >= size)) {
			dev_err_ratelimited(mailbox->dev, "Invalid response queue tail: %#x", tail);
			break;
		}

		count = gcip_circ_queue_cnt(head, tail, size, wrap_bit);
		if (count == 0)
			break;

		prev_ptr = ret;
		ret = krealloc(prev_ptr, (total + count) * elem_size,
			       atomic ? GFP_ATOMIC : GFP_KERNEL);
		/*
		 * Out-of-memory, we can return the previously fetched responses if any, or ENOMEM
		 * otherwise.
		 */
		if (!ret) {
			if (!prev_ptr)
				ret = ERR_PTR(-ENOMEM);
			else
				ret = prev_ptr;
			break;
		}
		/* Copies responses. */
		j = CIRC_QUEUE_REAL_INDEX(head, wrap_bit);
		for (i = 0; i < count; i++) {
			memcpy(ret + elem_size * total, mailbox->rx_queue + elem_size * j,
			       elem_size);
			j = (j + 1) % size;
			total++;
		}
		head = gcip_circ_queue_inc(head, count, size, wrap_bit);
	}
	INC_RX_QUEUE_HEAD(total);

	RELEASE_RX_QUEUE_LOCK();

	if (mailbox->ops->after_fetch_resps)
		mailbox->ops->after_fetch_resps(mailbox, total);
out:
	*total_ptr = total;
	return ret;
}

/* Fetches one response from the response queue. */
static int gcip_mailbox_fetch_one_response(struct gcip_mailbox *mailbox, void *resp)
{
	u32 head;
	u32 tail;
	bool atomic;

	if (IS_BLOCK_OFF() || !ACQUIRE_RX_QUEUE_LOCK(true, &atomic))
		return 0;

	head = GET_RX_QUEUE_HEAD();
	tail = GET_RX_QUEUE_TAIL();
	/* Queue empty. */
	if (head == tail) {
		RELEASE_RX_QUEUE_LOCK();
		return 0;
	}

	memcpy(resp,
	       mailbox->rx_queue +
		       CIRC_QUEUE_REAL_INDEX(head, mailbox->queue_wrap_bit) * mailbox->rx_elem_size,
	       mailbox->rx_elem_size);
	INC_RX_QUEUE_HEAD(1);

	RELEASE_RX_QUEUE_LOCK();

	if (mailbox->ops->after_fetch_resps)
		mailbox->ops->after_fetch_resps(mailbox, 1);

	return 1;
}

/* Handles the timed out asynchronous commands. */
static void gcip_mailbox_async_cmd_timeout_work(struct work_struct *work)
{
	struct gcip_mailbox_resp_awaiter *awaiter =
		container_of(work, struct gcip_mailbox_resp_awaiter, timeout_work.work);
	struct gcip_mailbox *mailbox = awaiter->mailbox;
	bool removed;

	TEST_NOTIFY_TIMEOUT_HANDLER_START();

	/*
	 * This function returns true if @awaiter has been removed from the wait list successfully.
	 * It means that it is safe to process @awaiter as timeout. (i.e., there won't be any race
	 * cases that @awaiter has been processed as arrived or canceled at the same time.)
	 */
	removed = gcip_mailbox_del_wait_resp(mailbox, &awaiter->async_resp);
	if (removed) {
		if (mailbox->ops->handle_awaiter_timedout)
			mailbox->ops->handle_awaiter_timedout(mailbox, awaiter);

		complete_all(&awaiter->handled);
	}

	/* Remove the reference of the timedout handler. */
	gcip_mailbox_awaiter_put(awaiter);
}

/**
 * gcip_mailbox_cancel_awaiter_all() - Cancels the unhandled awaiters in the wait list.
 * @mailbox: The mailbox to cancel the unhandled awaiters.
 *
 * This function will delete all the awaiters in the wait list and cancel their timeout workers.
 * It is used when the mailbox is about to be released.
 */
static void gcip_mailbox_cancel_awaiter_all(struct gcip_mailbox *mailbox)
{
	struct gcip_mailbox_wait_list_elem *cur, *nxt;
	struct gcip_mailbox_resp_awaiter *awaiter;
	struct list_head cancel_list;
	unsigned long flags;

	/* Tests cases that responses arrived or timedout while flushing awaiters. */
	TEST_WAIT_FIRMWARE_WORK();

	/*
	 * At this point only async responses should be pending.
	 * Remove them all from the `wait_list` at once to prevent them from being handled by
	 * the arrived or timedout handlers.
	 */
	INIT_LIST_HEAD(&cancel_list);
	spin_lock_irqsave(&mailbox->wait_list->list_lock, flags);
	list_splice_init(&mailbox->wait_list->list, &cancel_list);
	spin_unlock_irqrestore(&mailbox->wait_list->list_lock, flags);

	list_for_each_entry_safe(cur, nxt, &cancel_list, list) {
		awaiter = cur->awaiter;

		if (!awaiter) {
			dev_err(mailbox->dev, "Unexpected synchronous cmd during mailbox release");
			kfree(cur);
			continue;
		}

		/* Remove the reference of the awaiter acquired by the wait list. */
		gcip_mailbox_awaiter_put(awaiter);

		/*  This will implicitly decrease the reference acquired by the timeout work. */
		gcip_mailbox_cancel_timeout_work_sync(awaiter);

		/*
		 * If the operator is defined, @awaiter will be released on the implementation side.
		 * Otherwise, it should be freed from here.
		 */
		if (mailbox->ops->handle_awaiter_flushed)
			mailbox->ops->handle_awaiter_flushed(mailbox, awaiter);
		else
			gcip_mailbox_awaiter_put(cur->awaiter);

		kfree(cur);
	}
}

/* Verifies the mailbox operators. */
static int gcip_mailbox_ops_verify(const struct gcip_mailbox_ops *ops, u8 mode, struct device *dev)
{
	if (!ops) {
		dev_err(dev, "Mailbox ops should not be NULL.");
		return -EINVAL;
	}

	if ((mode & GCIP_MAILBOX_MODE_TX_CMD) || (mode & GCIP_MAILBOX_MODE_TX_RSP)) {
		if (!ops->get_tx_queue_tail || !ops->inc_tx_queue_tail ||
		    !ops->acquire_tx_queue_lock || !ops->release_tx_queue_lock ||
		    !ops->wait_for_tx_queue_not_full) {
			dev_err(dev, "Incomplete mailbox CMD queue ops.");
			return -EINVAL;
		}
	}

	if ((mode & GCIP_MAILBOX_MODE_RX_RSP) || (mode & GCIP_MAILBOX_MODE_RX_CMD)) {
		if (!ops->get_rx_queue_size || !ops->get_rx_queue_head || !ops->get_rx_queue_tail ||
		    !ops->inc_rx_queue_head || !ops->acquire_rx_queue_lock ||
		    !ops->release_rx_queue_lock) {
			dev_err(dev, "Incomplete mailbox RESP queue ops.");
			return -EINVAL;
		}
	}

	if (should_maintain_seq_num(mode)) {
		if (!ops->get_cmd_elem_seq || !ops->set_cmd_elem_seq || !ops->get_resp_elem_seq ||
		    !ops->set_resp_elem_seq) {
			dev_err(dev, "Incomplete mailbox sequence number ops.");
			return -EINVAL;
		}
	}

	if (mode & GCIP_MAILBOX_MODE_RX_CMD) {
		if (!ops->handle_reversed_command) {
			dev_err(dev, "Incomplete mailbox reversed CMD element ops.");
			return -EINVAL;
		}
	}

	if ((mode & GCIP_MAILBOX_MODE_RX_RSP) && (mode & GCIP_MAILBOX_MODE_RX_CMD)) {
		if (!ops->is_rx_elem_reversed) {
			dev_err(dev, "Incomplete mailbox RX element ops.");
			return -EINVAL;
		}
	}

	return 0;
}

int gcip_mailbox_init(struct gcip_mailbox *mailbox, const struct gcip_mailbox_args *args)
{
	int ret;

	if (!args->mode) {
		dev_err(args->dev, "Mailbox mode cannot be NULL.");
		return -EINVAL;
	}

	ret = gcip_mailbox_ops_verify(args->ops, args->mode, args->dev);
	if (ret)
		return ret;

	mailbox->dev = args->dev;
	mailbox->mode = args->mode;
	mailbox->queue_wrap_bit = args->queue_wrap_bit;
	mailbox->tx_queue = args->tx_queue;
	mailbox->tx_elem_size = args->tx_elem_size;
	mailbox->rx_queue = args->rx_queue;
	mailbox->rx_elem_size = args->rx_elem_size;
	mailbox->timeout = args->timeout;
	mailbox->ops = args->ops;
	mailbox->data = args->data;

	atomic64_set(&mailbox->cur_seq, 0);

	if (args->wait_list_external) {
		mailbox->wait_list = args->wait_list_external;
	} else {
		gcip_mailbox_wait_list_init(&mailbox->wait_list_internal);
		mailbox->wait_list = &mailbox->wait_list_internal;
	}

	return 0;
}

void gcip_mailbox_release(struct gcip_mailbox *mailbox)
{
	gcip_mailbox_cancel_awaiter_all(mailbox);
	mailbox->ops = NULL;
	mailbox->data = NULL;
}

void gcip_mailbox_wait_list_init(struct gcip_mailbox_wait_list *wait_list)
{
	spin_lock_init(&wait_list->list_lock);
	INIT_LIST_HEAD(&wait_list->list);
	init_waitqueue_head(&wait_list->waitq);
}

static void gcip_mailbox_do_consume_responses(struct gcip_mailbox *mailbox, bool trylock)
{
	void *responses;
	u32 i;
	u32 count = 0;

	/* Fetches responses and bumps resp_queue head. */
	responses = gcip_mailbox_fetch_responses(mailbox, &count, trylock);
	if (count == 0)
		return;
	if (IS_ERR(responses)) {
		dev_err(mailbox->dev, "GCIP mailbox failed on fetching responses: %ld",
			PTR_ERR(responses));
		return;
	}

	for (i = 0; i < count; i++)
		gcip_mailbox_handle_rx_elem(mailbox, responses + mailbox->rx_elem_size * i);

	/* Responses handled, wake up threads that are waiting for a response. */
	wake_up(&mailbox->wait_list->waitq);
	kfree(responses);
}

void gcip_mailbox_consume_responses_work(struct gcip_mailbox *mailbox)
{
	gcip_mailbox_do_consume_responses(mailbox, true);
}

void gcip_mailbox_consume_responses(struct gcip_mailbox *mailbox)
{
	gcip_mailbox_do_consume_responses(mailbox, false);
}

int gcip_mailbox_send_cmd(struct gcip_mailbox *mailbox, void *cmd, void *resp,
			  u32 gcip_mailbox_cmd_flags)
{
	struct gcip_mailbox_async_resp async_resp = {
		.resp = resp,
	};
	int ret;

	ret = gcip_mailbox_enqueue_cmd(mailbox, cmd, &async_resp, NULL, gcip_mailbox_cmd_flags);
	if (ret)
		goto err;

	/*
	 * If @resp is NULL, it will not enqueue the response into the waiting list. Therefore, it
	 * is fine to release @async_resp.
	 */
	if (!resp)
		return 0;

	ret = wait_event_timeout(mailbox->wait_list->waitq, !async_resp.waiting,
				 msecs_to_jiffies(mailbox->timeout));
	if (!ret) {
		dev_dbg(mailbox->dev, "event wait timeout");
		gcip_mailbox_del_wait_resp(mailbox, &async_resp);
		ret = -ETIMEDOUT;
		goto err;
	}

	/*
	 * Paired with write barrier in gcip_mailbox_handle_response.  Access to other fields in
	 * async_resp, plus access to *resp (by caller), must occur after observing
	 * !async_resp.waiting above.
	 */
	smp_rmb();

	return 0;

err:
	if (mailbox->ops->on_error)
		mailbox->ops->on_error(mailbox, ret);

	return ret;
}

struct gcip_mailbox_resp_awaiter *gcip_mailbox_put_cmd_flags(struct gcip_mailbox *mailbox,
							     void *cmd, void *resp, void *data,
							     u32 gcip_mailbox_cmd_flags)
{
	struct gcip_mailbox_resp_awaiter *awaiter;
	int ret;

	/* The ownership of the data will be transferred to the awaiter */
	awaiter = gcip_mailbox_awaiter_create(mailbox, resp, data);
	if (IS_ERR(awaiter))
		return awaiter;

	INIT_DELAYED_WORK(&awaiter->timeout_work, gcip_mailbox_async_cmd_timeout_work);
	if (!(gcip_mailbox_cmd_flags & GCIP_MAILBOX_CMD_FLAGS_NO_TIMEOUT)) {
		u32 timeout;

		if (mailbox->ops->get_cmd_timeout)
			timeout = mailbox->ops->get_cmd_timeout(mailbox, cmd, resp, data);
		else
			timeout = mailbox->timeout;

		/* The pending timeout worker needs a reference as well. */
		gcip_mailbox_awaiter_get(awaiter);

		schedule_delayed_work(&awaiter->timeout_work, msecs_to_jiffies(timeout));
	}

	ret = gcip_mailbox_enqueue_cmd(mailbox, cmd, &awaiter->async_resp, awaiter,
				       gcip_mailbox_cmd_flags);
	if (ret)
		goto err_free_resp;

	return awaiter;

err_free_resp:
	gcip_mailbox_cancel_timeout_work_sync(awaiter);
	/*
	 * Use kfree instead of gcip_mailbox_awaiter_put() when error occurred.
	 * The @data should not be released here as the caller should handle it.
	 */
	kfree(awaiter);
	return ERR_PTR(ret);
}

struct gcip_mailbox_resp_awaiter *gcip_mailbox_put_cmd(struct gcip_mailbox *mailbox, void *cmd,
						       void *resp, void *data)
{
	return gcip_mailbox_put_cmd_flags(mailbox, cmd, resp, data, 0);
}

bool gcip_mailbox_cancel_awaiter(struct gcip_mailbox_resp_awaiter *awaiter)
{
	bool removed;

	/* Cancel the timeout work of the awaiter if it is still pending. */
	gcip_mailbox_cancel_timeout_work(awaiter);

	/*
	 * If @removed is true, it means that the awaiter is now taken by this cancel handler.
	 * The arrived handler will skip the awaiter when they are executed.
	 *
	 * If @removed is false, it means either the arrived handler or the timeout handler has
	 * already started, wait until they are completed to ensure that no other threads will
	 * access the awaiter. The caller is the only owner that can access the awaiter after this
	 * function returns.
	 */
	removed = gcip_mailbox_del_wait_resp(awaiter->mailbox, &awaiter->async_resp);
	if (!removed)
		wait_for_completion(&awaiter->handled);

	return removed;
}

void gcip_mailbox_cancel_timeout_work(struct gcip_mailbox_resp_awaiter *awaiter)
{
	/*
	 * If the timeout work is canceled successfully, we have to decrease the reference count
	 * which was acquired by the timeout work.
	 */
	if (cancel_delayed_work(&awaiter->timeout_work))
		gcip_mailbox_awaiter_put(awaiter);
}

void gcip_mailbox_cancel_timeout_work_sync(struct gcip_mailbox_resp_awaiter *awaiter)
{
	/*
	 * If the timeout work is canceled successfully, we have to decrease the reference count
	 * which was acquired by the timeout work.
	 */
	if (cancel_delayed_work_sync(&awaiter->timeout_work))
		gcip_mailbox_awaiter_put(awaiter);
}

void gcip_mailbox_consume_one_response(struct gcip_mailbox *mailbox, void *resp)
{
	int ret;

	/* Fetches (at most) one response. */
	ret = gcip_mailbox_fetch_one_response(mailbox, resp);
	if (!ret)
		return;

	gcip_mailbox_handle_rx_elem(mailbox, resp);

	/* Responses handled, wakes up threads that are waiting for a response. */
	wake_up(&mailbox->wait_list->waitq);
}
