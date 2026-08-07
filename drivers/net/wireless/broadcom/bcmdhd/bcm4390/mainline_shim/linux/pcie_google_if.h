/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline integration for the downstream google_pcie_* PCIe-RC glue.
 *
 * komodo (zumapro) uses the mainline pci-exynos host controller. The downstream
 * Exynos/Google PCIe root-complex power API is not ported wholesale; instead
 * these thin wrappers map dhd_custom_google.c's RC coupling onto pci-exynos's
 * exported Wi-Fi runtime-relink API (zumapro_pcie_wifi_*), so dhd's D3 PCIe
 * power management (suspend parks the link, resume must retrain it) actually
 * works. A stubbed resume (the earlier no-op) left the endpoint stuck after the
 * BCM4390 firmware parked its link on D3_INFORM: "Unable to change power state
 * from D3hot to D0, device inaccessible" -> synthesized deauth.
 *
 * ch_num is the PCIe domain of the Wi-Fi endpoint; the EP lives at domain:1:00.0
 * and its RC controller is the host bridge's parent (the pci-exynos platform
 * device that zumapro_pcie_from_dev() keys on). See dhd-bcm4390-integration.
 */
#ifndef _MAINLINE_PCIE_GOOGLE_IF_STUB_H_
#define _MAINLINE_PCIE_GOOGLE_IF_STUB_H_

#include <linux/pci.h>
#include <linux/device.h>
#include <linux/errno.h>

/* pci-exynos exported Wi-Fi runtime relink API (drivers/pci/controller/dwc). */
extern int zumapro_pcie_wifi_link_up(struct device *rc_dev);
extern int zumapro_pcie_wifi_link_down(struct device *rc_dev);
extern int zumapro_pcie_wifi_link_status(struct device *rc_dev);
extern int zumapro_pcie_wifi_l1ss_enable(struct device *rc_dev);
extern int zumapro_pcie_wifi_l1ss_disable(struct device *rc_dev);
extern int zumapro_pcie_wifi_l1_exit(struct device *rc_dev);

enum google_pcie_callback_type {
	GPCIE_CB_LINK_DOWN = 0,
	GPCIE_CB_CPL_TIMEOUT,
	GPCIE_CB_MAX,
};

/* Resolve the Wi-Fi RC controller device from the endpoint's PCIe domain. */
static inline struct device *google_pcie_wifi_rc_dev(int ch_num)
{
	/* Resolve by the BCM4390 vendor:device, NOT by ch_num/domain. The DT
	 * ch-num (=1) maps to the wrong PCIe controller -- pci_find_bus(1,1)
	 * lands on the MODEM's domain (12100000.pcie), not the Wi-Fi's
	 * (13120000.pcie, domain 0). Find the actual Wi-Fi endpoint (14e4:4438,
	 * OTP-alt 14e4:4390) and return its root complex device. */
	struct pci_dev *ep = pci_get_device(0x14e4, 0x4438, NULL);
	struct pci_host_bridge *hb;
	struct device *rc = NULL;

	(void)ch_num;
	if (!ep)
		ep = pci_get_device(0x14e4, 0x4390, NULL);
	if (!ep)
		return NULL;
	hb = pci_find_host_bridge(ep->bus);
	if (hb)
		rc = hb->dev.parent;
	pci_dev_put(ep);
	return rc;
}

/* Enable/disable the full downstream Wi-Fi L1SS+LTR+L1.2 sequence in pci-exynos.
 * Must run after the EP is enumerated + firmware up (dhd_plat_l1ss_ctrl). */
static inline int google_pcie_wifi_l1ss(int ch_num, int enable)
{
	struct device *rc = google_pcie_wifi_rc_dev(ch_num);

	if (!rc)
		return -ENODEV;
	return enable ? zumapro_pcie_wifi_l1ss_enable(rc)
		      : zumapro_pcie_wifi_l1ss_disable(rc);
}

/* Transient SW L1-exit (downstream exynos_pcie_l1_exit): force LTSSM to L0,
 * leaving L1SS armed. Recovery/diagnostic parity hook -- downstream komodo
 * exports but never invokes it (DHD_PCIE_L1_EXIT_DURING_IO is defined in no
 * downstream Makefile and dhd_plat_l1_exit has no callers). No-op (ret 0)
 * while L1SS is not armed. */
static inline int google_pcie_wifi_l1_exit(int ch_num)
{
	struct device *rc = google_pcie_wifi_rc_dev(ch_num);

	if (!rc)
		return -ENODEV;
	return zumapro_pcie_wifi_l1_exit(rc);
}

/* Link-down / completion-timeout callbacks: pci-exynos surfaces link-down via
 * its own IRQ; dhd's software recovery is not wired here. Harmless no-ops. */
static inline int google_pcie_register_callback(int domain_nr,
		void (*cb)(enum google_pcie_callback_type, void *), void *priv)
{
	(void)domain_nr; (void)cb; (void)priv;
	return 0;
}

static inline void google_pcie_unregister_callback(int domain_nr)
{
	(void)domain_nr;
}

/* nonzero == link up */
static inline int google_pcie_link_status(int ch_num)
{
	struct device *rc = google_pcie_wifi_rc_dev(ch_num);

	return rc ? zumapro_pcie_wifi_link_status(rc) : 0;
}

/* 0 == success. Retrain the parked link back to L0 (dhd D3 resume). */
static inline int google_pcie_rc_poweron(int ch_num)
{
	struct device *rc = google_pcie_wifi_rc_dev(ch_num);

	return rc ? zumapro_pcie_wifi_link_up(rc) : -ENODEV;
}

static inline void google_pcie_rc_poweroff(int ch_num)
{
	struct device *rc = google_pcie_wifi_rc_dev(ch_num);

	if (rc)
		zumapro_pcie_wifi_link_down(rc);
}

/* The warm retrain preserves the endpoint's trained link speed, so the speed
 * argument (already-known EP max speed) needs no separate handling here. */
static inline int google_pcie_poweron_withspeed(int ch_num, int speed)
{
	struct device *rc = google_pcie_wifi_rc_dev(ch_num);

	(void)speed;
	return rc ? zumapro_pcie_wifi_link_up(rc) : -ENODEV;
}

static inline void google_pcie_dump_debug(int num)
{
	(void)num;
}

#endif /* _MAINLINE_PCIE_GOOGLE_IF_STUB_H_ */
