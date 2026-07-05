/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Bring-up hooks the Zuma/Zumapro PCIe root-complex driver (pci-exynos)
 * exports for the Samsung Exynos Modem 5300 boot driver. The modem boot
 * protocol needs two things no other endpoint does: the RC's MSI target
 * address moved into the modem's MSI carveout (the mask ROM derives where to
 * report boot progress from its MSI capability address), and a mid-boot link
 * bounce (the CP bootloader re-links after the first-stage download). Both
 * callers pass the RC's platform device resolved from a DT phandle.
 */
#ifndef _LINUX_PCIE_ZUMAPRO_H
#define _LINUX_PCIE_ZUMAPRO_H

#include <linux/types.h>

struct device;

#if IS_ENABLED(CONFIG_PCI_EXYNOS)
int zumapro_pcie_set_msi_target(struct device *rc_dev, phys_addr_t target);
int zumapro_pcie_modem_link_down(struct device *rc_dev);
int zumapro_pcie_modem_link_up(struct device *rc_dev);
#else
static inline int zumapro_pcie_set_msi_target(struct device *rc_dev,
					      phys_addr_t target)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_link_down(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_link_up(struct device *rc_dev)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_PCIE_ZUMAPRO_H */
