/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compatibility shim for <linux/of_gpio.h>.
 *
 * Upstream removed include/linux/of_gpio.h along with the legacy
 * of_get_named_gpio()/of_get_gpio()/of_gpio_count() helpers, requiring all
 * consumers to move to the gpiod descriptor API. The out-of-tree Google
 * modules still use the integer-GPIO helpers extensively, so this header
 * re-implements them on top of the surviving descriptor/fwnode API.
 *
 * It lives under google-modules/soc/gs/include (which is on every module's
 * include path) so the in-tree kernel headers stay untouched. Once the
 * modules are converted to the descriptor API this file can be deleted.
 *
 * Note on semantics: fwnode_gpiod_get_index() *requests* the line, whereas
 * the old of_get_named_gpio() only resolved its global number. We therefore
 * release the descriptor again immediately (gpiod_put) so the common
 * "n = of_get_named_gpio(...); gpio_request(n);" pattern keeps working. This
 * is safe at driver-probe time, which is where these helpers are used.
 */
#ifndef __COMPAT_LINUX_OF_GPIO_H
#define __COMPAT_LINUX_OF_GPIO_H

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/string.h>

/* Legacy flags returned by of_get_named_gpio_flags(). */
enum of_gpio_flags {
	OF_GPIO_ACTIVE_LOW	= 0x1,
	OF_GPIO_SINGLE_ENDED	= 0x2,
	OF_GPIO_OPEN_DRAIN	= 0x4,
	OF_GPIO_TRANSITORY	= 0x8,
	OF_GPIO_PULL_UP		= 0x10,
	OF_GPIO_PULL_DOWN	= 0x20,
	OF_GPIO_PULL_DISABLE	= 0x40,
};

/*
 * Derive a gpiod con_id from a legacy DT property name by stripping the
 * trailing "-gpios"/"-gpio"/"gpios"/"gpio" suffix. fwnode_gpiod_get_index()
 * re-appends "-gpios" (and falls back to "-gpio"), matching the original
 * property lookup. Returns an empty string for the bare "gpios" property,
 * which the caller maps to a NULL con_id.
 */
static inline void __of_gpio_propname_to_conid(const char *propname,
					       char *buf, size_t buflen)
{
	size_t len;

	if (!propname) {
		buf[0] = '\0';
		return;
	}
	strscpy(buf, propname, buflen);
	len = strlen(buf);
	if (len >= 6 && !strcmp(buf + len - 6, "-gpios"))
		buf[len - 6] = '\0';
	else if (len >= 5 && !strcmp(buf + len - 5, "-gpio"))
		buf[len - 5] = '\0';
	else if (len == 5 && !strcmp(buf, "gpios"))
		buf[0] = '\0';
	else if (len == 4 && !strcmp(buf, "gpio"))
		buf[0] = '\0';
}

static inline int of_get_named_gpio_flags(const struct device_node *np,
					  const char *propname, int index,
					  enum of_gpio_flags *flags)
{
	struct gpio_desc *desc;
	char conid[64];
	const char *con;
	int gpio;

	__of_gpio_propname_to_conid(propname, conid, sizeof(conid));
	con = conid[0] ? conid : NULL;

	desc = fwnode_gpiod_get_index(of_fwnode_handle((struct device_node *)np),
				      con, index, GPIOD_ASIS, "of_get_named_gpio");
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	if (flags)
		*flags = gpiod_is_active_low(desc) ? OF_GPIO_ACTIVE_LOW : 0;

	gpio = desc_to_gpio(desc);
	gpiod_put(desc);
	return gpio;
}

static inline int of_get_named_gpio(const struct device_node *np,
				    const char *propname, int index)
{
	return of_get_named_gpio_flags(np, propname, index, NULL);
}

static inline int of_get_gpio_flags(const struct device_node *np, int index,
				    enum of_gpio_flags *flags)
{
	return of_get_named_gpio_flags(np, "gpios", index, flags);
}

static inline int of_get_gpio(const struct device_node *np, int index)
{
	return of_get_named_gpio_flags(np, "gpios", index, NULL);
}

static inline int of_gpio_named_count(const struct device_node *np,
				      const char *propname)
{
	return of_count_phandle_with_args(np, propname, "#gpio-cells");
}

static inline int of_gpio_count(const struct device_node *np)
{
	return of_gpio_named_count(np, "gpios");
}

#endif /* __COMPAT_LINUX_OF_GPIO_H */
