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
struct phy;

/*
 * Switch the modem PCIe PHY sub-block clock to the safe OSC (or back to the
 * external PLL) during a runtime link bounce (implemented in phy-exynos-pcie.c).
 */
#if IS_ENABLED(CONFIG_PHY_EXYNOS_PCIE)
void exynos_pcie_phy_safe_clk(struct phy *phy, bool safe);
void exynos_pcie_phy_keep_refclk(struct phy *phy, bool keep);
#else
static inline void exynos_pcie_phy_safe_clk(struct phy *phy, bool safe) { }
static inline void exynos_pcie_phy_keep_refclk(struct phy *phy, bool keep) { }
#endif

#if IS_ENABLED(CONFIG_PCI_EXYNOS)
int zumapro_pcie_set_msi_target(struct device *rc_dev, phys_addr_t target);
int zumapro_pcie_reserve_msi_base(struct device *rc_dev, unsigned int count);
int zumapro_pcie_modem_link_down(struct device *rc_dev);
int zumapro_pcie_modem_link_up(struct device *rc_dev);
int zumapro_pcie_modem_gen3(struct device *rc_dev);
int zumapro_pcie_modem_wake(struct device *rc_dev);
int zumapro_pcie_modem_reset(struct device *rc_dev);
int zumapro_pcie_modem_power_cycle(struct device *rc_dev);
int zumapro_pcie_modem_link_speed(struct device *rc_dev);
#else
static inline int zumapro_pcie_set_msi_target(struct device *rc_dev,
					      phys_addr_t target)
{
	return -ENODEV;
}
static inline int zumapro_pcie_reserve_msi_base(struct device *rc_dev,
						unsigned int count)
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
static inline int zumapro_pcie_modem_gen3(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_wake(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_reset(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_link_speed(struct device *rc_dev)
{
	return -ENODEV;
}
static inline int zumapro_pcie_modem_power_cycle(struct device *rc_dev)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_PCIE_ZUMAPRO_H */
