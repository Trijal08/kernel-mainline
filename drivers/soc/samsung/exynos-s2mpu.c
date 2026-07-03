// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung/Google Exynos Stage-2 Memory Protection Unit (S2MPU), v9.
 *
 * The S2MPU is a bus-master firewall in front of a device's DMA path (GPU,
 * codecs, camera, ...). Out of reset it blocks all traffic until its Memory
 * Protection Table (MPT) is programmed. On production Android this is done by
 * the pKVM hypervisor at EL2; here we program the same registers from EL1.
 *
 * The v9 programming order matters: a context id must be allocated to a VID
 * (CONTEXT_CFG_VALID_VID) before any L1ENTRY* register for that VID may be
 * written - otherwise the write is rejected on the bus and raises an SError.
 * The unit is then enabled per-VID via CTRL_PROT_EN_PER_VID_SET (CTRL0 stays
 * 0 on v9).
 *
 * For bring-up this installs a fully-permissive MPT: every 1GB granule is
 * marked read/write for every context-backed VID. Covering the whole map at
 * the 1GB L1 granule means the S2MPU never walks an L2 table, so no
 * walk-attribute/SLC tuning is needed. A future extension can install a
 * tighter, per-buffer allow list.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define S2MPU_CTRL0				0x0
#define S2MPU_V9_CFG_MPTW_ATTRIBUTE		0x10
#define S2MPU_INTERRUPT_ENABLE_PER_VID_SET	0x20
#define S2MPU_V9_CTRL_PROT_EN_PER_VID_SET	0x50
#define S2MPU_V9_CTRL_PROT_EN_PER_VID_CLR	0x54
#define S2MPU_VERSION				0x60
#define S2MPU_V9_CTRL_ERR_RESP_T_PER_VID_SET	0x70
#define S2MPU_NUM_CONTEXT			0x100
#define S2MPU_NUM_CONTEXT_MASK			GENMASK(3, 0)
#define S2MPU_CONTEXT_CFG_VALID_VID		0x104
#define S2MPU_ALL_INVALIDATION			0x1000
#define S2MPU_ALL_INVALIDATION_GO		BIT(0)
#define S2MPU_L1ENTRY_ATTR(vid, gb)		(0x4004 + ((vid) * 0x200) + ((gb) * 0x8))

#define S2MPU_NR_VIDS				8
#define S2MPU_NR_GB_GRANULES			64
#define S2MPU_ALL_VIDS_BITMAP			GENMASK(S2MPU_NR_VIDS - 1, 0)

/* L1 attribute: 1GB granule (L2TABLE disabled), MPT_PROT in bits [2:1]. */
#define S2MPU_MPT_PROT_RW			3
#define S2MPU_L1ENTRY_ATTR_1G_RW		FIELD_PREP(GENMASK(2, 1), S2MPU_MPT_PROT_RW)

/* CONTEXT_CFG_VALID_VID: 4 bits per context - [vid in low 3, valid in bit 3]. */
#define S2MPU_CTX_CFG_VALID(ctx)		BIT((4 * (ctx)) + 3)
#define S2MPU_CTX_CFG_VID(ctx, vid)		(((u32)(vid) & 0x7) << (4 * (ctx)))

static int exynos_s2mpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u8 ctx_vid[S2MPU_NR_VIDS] = { };
	unsigned int num_ctx, ctx, gb;
	unsigned int vid_bmap;
	void __iomem *base;
	u32 ctx_cfg = 0;
	u32 version;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	version = readl_relaxed(base + S2MPU_VERSION);
	num_ctx = readl_relaxed(base + S2MPU_NUM_CONTEXT) & S2MPU_NUM_CONTEXT_MASK;
	if (!num_ctx) {
		dev_warn(dev, "no MPT contexts (version %#x); leaving as-is\n",
			 version);
		return 0;
	}

	/*
	 * Allocate a context id to each VID (in order), building the
	 * CONTEXT_CFG_VALID_VID value. This MUST be written before any L1ENTRY
	 * register for the corresponding VID, or the L1 write bus-errors.
	 */
	vid_bmap = S2MPU_ALL_VIDS_BITMAP;
	for (ctx = 0; ctx < num_ctx && vid_bmap; ctx++) {
		unsigned int vid = __ffs(vid_bmap);

		vid_bmap &= ~BIT(vid);
		ctx_vid[ctx] = vid;
		ctx_cfg |= S2MPU_CTX_CFG_VID(ctx, vid) | S2MPU_CTX_CFG_VALID(ctx);
	}
	writel_relaxed(ctx_cfg, base + S2MPU_CONTEXT_CFG_VALID_VID);

	/* Grant RW to the whole address map at the 1GB granule for each VID. */
	for (ctx = 0; ctx < num_ctx; ctx++)
		for (gb = 0; gb < S2MPU_NR_GB_GRANULES; gb++)
			writel_relaxed(S2MPU_L1ENTRY_ATTR_1G_RW,
				       base + S2MPU_L1ENTRY_ATTR(ctx_vid[ctx], gb));

	/*
	 * Make the unit fully transparent, mirroring the vendor pKVM EL1 resume
	 * path (which CLEARs per-VID enforcement and leaves CTRL0=0 before the
	 * hypervisor installs its own MPT). Leaving per-VID protection ENABLED,
	 * even with an all-permissive MPT, can make a v9 unit silently drop a
	 * transaction whose context is not the one it expects - without latching
	 * a fault - which looks exactly like our GPU_BUS_FAULT + FAULT_STATUS=0.
	 * Disabling enforcement outright avoids that class of failure.
	 */
	writel_relaxed(0, base + S2MPU_INTERRUPT_ENABLE_PER_VID_SET);
	writel_relaxed(S2MPU_ALL_VIDS_BITMAP,
		       base + S2MPU_V9_CTRL_PROT_EN_PER_VID_CLR);
	writel_relaxed(0, base + S2MPU_CTRL0);

	/* Flush the MPT cache and make all of the above visible. */
	writel_relaxed(S2MPU_ALL_INVALIDATION_GO, base + S2MPU_ALL_INVALIDATION);
	wmb();

	dev_info(dev, "v9 allow-all installed (version %#x, %u contexts)\n",
		 version, num_ctx);

	return 0;
}

static const struct of_device_id exynos_s2mpu_of_match[] = {
	{ .compatible = "google,s2mpu-v9" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_s2mpu_of_match);

static struct platform_driver exynos_s2mpu_driver = {
	.probe	= exynos_s2mpu_probe,
	.driver	= {
		.name		= "exynos-s2mpu",
		.of_match_table	= exynos_s2mpu_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(exynos_s2mpu_driver);

MODULE_DESCRIPTION("Exynos/Google S2MPU v9 permissive driver");
MODULE_LICENSE("GPL");
