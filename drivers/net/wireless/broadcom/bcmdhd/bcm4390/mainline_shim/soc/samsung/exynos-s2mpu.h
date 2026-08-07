/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline integration stub for the downstream Exynos S2MPU fault-detect
 * notifier API (soc/samsung/exynos-s2mpu.h is not present in mainline).
 *
 * komodo's BCM4390 WiFi DMA path has no S2MPU wall (100% MMIO, no secure
 * memory-protection unit on the Wi-Fi DMA), so dhd's S2MPU registration is a
 * no-op here and the fault callback is never invoked. See the
 * dhd-bcm4390-integration plan.
 */
#ifndef _MAINLINE_EXYNOS_S2MPU_STUB_H_
#define _MAINLINE_EXYNOS_S2MPU_STUB_H_

#define S2MPUFD_NOTIFY_OK	0
#define S2MPUFD_NOTIFY_BAD	1

struct s2mpufd_notifier_info {
	unsigned long fault_addr;
	int fault_rw;
	int fault_len;
	int fault_type;
};

struct s2mpufd_notifier_block {
	const char *subsystem;
	int (*notifier_call)(struct s2mpufd_notifier_block *block,
			     struct s2mpufd_notifier_info *info);
};

static inline int
s2mpufd_notifier_call_register(struct s2mpufd_notifier_block *nb)
{
	(void)nb;
	return 0;
}

#endif /* _MAINLINE_EXYNOS_S2MPU_STUB_H_ */
