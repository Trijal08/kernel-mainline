// SPDX-License-Identifier: GPL-2.0-only
/*
 * Trusty secure-world log: shares a ring buffer with Trusty and drains it into
 * the kernel log.  Minimal consumer - just enough to surface Trusty's own
 * messages (crash dumps, driver diagnostics) in dmesg.
 *
 * Copyright (C) 2015 Google, Inc.
 */

#include <linux/log2.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <linux/trusty/smcall.h>
#include <linux/trusty/sm_err.h>
#include <linux/trusty/trusty.h>

#include "trusty-log.h"

/* Contiguous (single FF-A range) so the share matches the paths that work. */
#define TRUSTY_LOG_SIZE		(32 * 1024)
#define TRUSTY_LINE_MAX		256

struct trusty_log_state {
	struct device *dev;
	struct device *trusty_dev;
	struct log_rb *log;
	trusty_shared_mem_id_t mem_id;
	struct scatterlist sg;
	u32 get;
	struct notifier_block call_notifier;
	char line[TRUSTY_LINE_MAX];
};

/* Drain [get, put) from the ring, emitting complete lines to the kernel log. */
static void trusty_log_drain(struct trusty_log_state *s)
{
	struct log_rb *log = s->log;
	u32 get, put, sz = log->sz;
	unsigned int n = 0;

	if (!sz || !is_power_of_2(sz))
		return;

	get = s->get;
	put = log->put;
	rmb();				/* read put before the data it guards */

	while (get != put) {
		char c = log->data[get & (sz - 1)];

		get++;
		if (c == '\n' || n == TRUSTY_LINE_MAX - 1) {
			s->line[n] = '\0';
			if (n)
				dev_info(s->dev, "TZ: %s\n", s->line);
			n = 0;
			if (c != '\n')
				s->line[n++] = c;
		} else {
			s->line[n++] = c;
		}
	}
	if (n) {
		s->line[n] = '\0';
		dev_info(s->dev, "TZ: %s\n", s->line);
	}

	s->get = get;
}

static int trusty_log_call_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct trusty_log_state *s;

	if (action != TRUSTY_CALL_RETURNED)
		return NOTIFY_DONE;

	s = container_of(nb, struct trusty_log_state, call_notifier);
	trusty_log_drain(s);
	return NOTIFY_OK;
}

static int trusty_log_probe(struct platform_device *pdev)
{
	struct trusty_log_state *s;
	int ret;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = &pdev->dev;
	s->trusty_dev = s->dev->parent;

	ret = trusty_std_call32(s->trusty_dev, SMC_SC_SHARED_LOG_VERSION,
				TRUSTY_LOG_API_VERSION, 0, 0);
	if (ret != TRUSTY_LOG_API_VERSION) {
		dev_info(s->dev, "trusty-log unsupported (version %d)\n", ret);
		ret = -ENOTSUPP;
		goto err_free_state;
	}

	s->log = alloc_pages_exact(TRUSTY_LOG_SIZE, GFP_KERNEL | __GFP_ZERO);
	if (!s->log) {
		ret = -ENOMEM;
		goto err_free_state;
	}
	sg_init_one(&s->sg, s->log, TRUSTY_LOG_SIZE);

	ret = trusty_share_memory_compat(s->trusty_dev, &s->mem_id, &s->sg, 1,
					 PAGE_KERNEL);
	if (ret) {
		dev_err(s->dev, "failed to share log buffer: %d\n", ret);
		goto err_free_log;
	}

	ret = trusty_std_call32(s->trusty_dev, SMC_SC_SHARED_LOG_ADD,
				(u32)s->mem_id, (u32)(s->mem_id >> 32),
				TRUSTY_LOG_SIZE);
	if (ret < 0) {
		dev_err(s->dev, "SMC_SC_SHARED_LOG_ADD failed: %d\n", ret);
		goto err_reclaim;
	}

	s->call_notifier.notifier_call = trusty_log_call_notify;
	ret = trusty_call_notifier_register(s->trusty_dev, &s->call_notifier);
	if (ret < 0)
		goto err_log_rm;

	platform_set_drvdata(pdev, s);
	dev_info(s->dev, "trusty-log online\n");
	return 0;

err_log_rm:
	trusty_std_call32(s->trusty_dev, SMC_SC_SHARED_LOG_RM, (u32)s->mem_id,
			  (u32)(s->mem_id >> 32), 0);
err_reclaim:
	trusty_reclaim_memory(s->trusty_dev, s->mem_id, &s->sg, 1);
err_free_log:
	free_pages_exact(s->log, TRUSTY_LOG_SIZE);
err_free_state:
	kfree(s);
	return ret;
}

static void trusty_log_remove(struct platform_device *pdev)
{
	struct trusty_log_state *s = platform_get_drvdata(pdev);

	trusty_call_notifier_unregister(s->trusty_dev, &s->call_notifier);
	trusty_std_call32(s->trusty_dev, SMC_SC_SHARED_LOG_RM, (u32)s->mem_id,
			  (u32)(s->mem_id >> 32), 0);
	trusty_reclaim_memory(s->trusty_dev, s->mem_id, &s->sg, 1);
	free_pages_exact(s->log, TRUSTY_LOG_SIZE);
	kfree(s);
}

static const struct of_device_id trusty_log_of_match[] = {
	{ .compatible = "android,trusty-log-v1", },
	{ }
};
MODULE_DEVICE_TABLE(of, trusty_log_of_match);

static struct platform_driver trusty_log_driver = {
	.probe = trusty_log_probe,
	.remove = trusty_log_remove,
	.driver = {
		.name = "trusty-log",
		.of_match_table = trusty_log_of_match,
	},
};
module_platform_driver(trusty_log_driver);

MODULE_DESCRIPTION("Trusty secure-world log");
MODULE_LICENSE("GPL");
