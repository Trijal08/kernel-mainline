// SPDX-License-Identifier: GPL-2.0
/*
 * Touch Bus Negotiator call-in point.
 *
 * The negotiator itself needs the AoC, so it can only load once the AoC driver
 * has; a touch driver, by contrast, wants to be up from the initramfs.  If the
 * touch driver called into the negotiator directly it would carry a module
 * dependency on it, and through it on the AoC -- loading touch early would drag
 * the whole AoC stack along.
 *
 * So the dependency runs the other way.  This object is always built in, holds
 * the operations the negotiator publishes when it probes, and answers the touch
 * driver's calls from whatever is registered at the time.  Nothing is
 * registered until the negotiator loads, which is exactly the "AoC is not
 * available" case its callers already handle: tbn_ready() is false and the
 * touch driver keeps the bus across suspend.
 *
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/rcupdate.h>
#include <linux/spinlock.h>

#include "touch_bus_negotiator.h"

static const struct tbn_ops __rcu *tbn_current;
static DEFINE_SPINLOCK(tbn_ops_lock);

/**
 * tbn_register_ops() - publish the negotiator's operations
 * @ops: operations table, valid until tbn_unregister_ops()
 *
 * Called by the negotiator once it has attached to the AoC.  Return: 0, or
 * -EBUSY if a negotiator is already registered.
 */
int tbn_register_ops(const struct tbn_ops *ops)
{
	int ret = 0;

	if (!ops)
		return -EINVAL;

	spin_lock(&tbn_ops_lock);
	if (rcu_dereference_protected(tbn_current,
				      lockdep_is_held(&tbn_ops_lock)))
		ret = -EBUSY;
	else
		rcu_assign_pointer(tbn_current, ops);
	spin_unlock(&tbn_ops_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_register_ops);

/**
 * tbn_unregister_ops() - withdraw the negotiator's operations
 *
 * Waits for in-flight callers to leave before returning, so the negotiator can
 * be unloaded safely afterwards.
 */
void tbn_unregister_ops(void)
{
	spin_lock(&tbn_ops_lock);
	rcu_assign_pointer(tbn_current, NULL);
	spin_unlock(&tbn_ops_lock);

	synchronize_rcu();
}
EXPORT_SYMBOL_GPL(tbn_unregister_ops);

/*
 * The calls below run under rcu_read_lock() so the negotiator cannot go away
 * mid-call.  They are not hot: a handful per display transition.
 */
bool tbn_ready(void)
{
	const struct tbn_ops *ops;
	bool ready = false;

	rcu_read_lock();
	ops = rcu_dereference(tbn_current);
	if (ops && ops->ready)
		ready = ops->ready();
	rcu_read_unlock();

	return ready;
}
EXPORT_SYMBOL_GPL(tbn_ready);

/*
 * No negotiator means no bus to share, which is not an error: the caller keeps
 * the bus and takes its own suspend path.  Hence mask 0 rather than a failure.
 */
int register_tbn(u32 *output)
{
	const struct tbn_ops *ops;
	int ret = 0;

	*output = 0;

	rcu_read_lock();
	ops = rcu_dereference(tbn_current);
	if (ops && ops->register_tbn)
		ret = ops->register_tbn(output);
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(register_tbn);

void unregister_tbn(u32 *output)
{
	const struct tbn_ops *ops;

	rcu_read_lock();
	ops = rcu_dereference(tbn_current);
	if (ops && ops->unregister_tbn)
		ops->unregister_tbn(output);
	rcu_read_unlock();

	*output = 0;
}
EXPORT_SYMBOL_GPL(unregister_tbn);

void register_tbn_lptw_callback(void (*callback)(struct TbnLptwEvent *lptw,
						 void *user_data),
				void *cbdata)
{
	const struct tbn_ops *ops;

	rcu_read_lock();
	ops = rcu_dereference(tbn_current);
	if (ops && ops->register_lptw_callback)
		ops->register_lptw_callback(callback, cbdata);
	rcu_read_unlock();
}
EXPORT_SYMBOL_GPL(register_tbn_lptw_callback);

int tbn_request_bus_with_result(u32 dev_mask, bool *lptw_triggered)
{
	const struct tbn_ops *ops;
	int ret = -ENODEV;

	rcu_read_lock();
	ops = rcu_dereference(tbn_current);
	if (ops && ops->request_bus)
		ret = ops->request_bus(dev_mask, lptw_triggered);
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_request_bus_with_result);

int tbn_request_bus(u32 dev_mask)
{
	return tbn_request_bus_with_result(dev_mask, NULL);
}
EXPORT_SYMBOL_GPL(tbn_request_bus);

int tbn_release_bus(u32 dev_mask)
{
	const struct tbn_ops *ops;
	int ret = -ENODEV;

	rcu_read_lock();
	ops = rcu_dereference(tbn_current);
	if (ops && ops->release_bus)
		ret = ops->release_bus(dev_mask);
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(tbn_release_bus);
