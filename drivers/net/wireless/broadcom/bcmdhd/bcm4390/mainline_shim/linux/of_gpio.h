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
#include <linux/gpio/driver.h>
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
 * Resolve a legacy named GPIO property to its global GPIO number the way the
 * removed of_get_named_gpio() did: read the property by its *exact* name as a
 * GPIO specifier (phandle + #gpio-cells args) and look up the descriptor on the
 * referenced gpiochip. This works for raw property names (e.g.
 * "usbpd,usbpd_int", "gpio_gnss2ap_spi") as well as the "<name>-gpios"
 * convention, unlike fwnode_gpiod_get_index() which only accepts the suffixed
 * con_id form and therefore mistranslates raw-named properties to -ENOENT.
 */
static inline int of_get_named_gpio_flags(const struct device_node *np,
					  const char *propname, int index,
					  enum of_gpio_flags *flags)
{
	struct of_phandle_args gpiospec;
	struct gpio_device *gdev;
	struct gpio_desc *desc;
	int gpio, ret;

	ret = of_parse_phandle_with_args(np, propname, "#gpio-cells", index,
					 &gpiospec);
	if (ret)
		return ret;

	gdev = gpio_device_find_by_fwnode(of_fwnode_handle(gpiospec.np));
	of_node_put(gpiospec.np);
	if (!gdev)
		return -EPROBE_DEFER;

	/*
	 * Standard GPIO specifiers encode <line flags>; the line number is the
	 * chip-relative hwnum. This matches every controller these modules use
	 * (exynos pinctrl banks, PMIC GPIOs, etc.).
	 */
	desc = gpio_device_get_desc(gdev, gpiospec.args[0]);
	if (IS_ERR(desc)) {
		gpio_device_put(gdev);
		return PTR_ERR(desc);
	}

	if (flags)
		*flags = (gpiospec.args_count > 1 && (gpiospec.args[1] & 1)) ?
			 OF_GPIO_ACTIVE_LOW : 0;

	gpio = desc_to_gpio(desc);
	gpio_device_put(gdev);
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
