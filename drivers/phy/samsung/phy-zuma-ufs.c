// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Google Tensor G4 / Zuma.
 *
 * PMA register values are converted from the downstream Zuma UFS CAL table:
 * private/google-modules/soc/gs/drivers/ufs/zuma/ufs-cal.h.
 */

#include <linux/io.h>
#include <linux/iopoll.h>

#include "phy-samsung-ufs.h"

/*
 * zuma/zumapro insert extra PMU_ALIVE registers ahead of the PHY isolation
 * controls, so the UFS PHY_CTRL sits one register lower than on gs101. Using
 * the gs101 offset here clears a bit in an unrelated PMU_ALIVE register on
 * phy_power_off(); as PMU_ALIVE survives a warm reset, that wedges the next
 * boot until VDD_ALIVE is cut.
 */
#define TENSOR_ZUMA_PHY_CTRL		0x3ec0
#define TENSOR_ZUMA_PHY_CTRL_MASK	0x1
#define TENSOR_ZUMA_PHY_CTRL_EN	BIT(0)
#define PHY_ZUMA_LANE_OFFSET	0x200
#define PHY_ZUMA_EMB_CAL_DONE	0x31d
#define PHY_ZUMA_EMB_CAL_DONE_BIT	BIT(0)

#define PHY_PMA_TRSV_ADDR(reg, lane)	(PHY_APB_ADDR((reg) + \
					((lane) * PHY_ZUMA_LANE_OFFSET)))
#define PHY_TRSV_REG_CFG_ZUMA(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, PHY_ZUMA_LANE_OFFSET)

static const struct samsung_ufs_phy_cfg tensor_zuma_pre_init_cfg[] = {
	PHY_COMN_REG_CFG(0x50, 0x08, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x05, 0x19, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0b, 0x44, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0c, 0xc4, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0d, 0xc3, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0f, 0x88, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x16, 0x1a, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x19, 0x04, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x54, 0x88, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x67, 0x4c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x68, 0x4c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x201, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x202, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x203, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x204, 0x18, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x205, 0xc0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x207, 0x1c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2ec, 0x8c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x27c, 0xd0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x288, 0xfa, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x289, 0x60, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x234, 0x30, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x239, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x23d, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x24d, 0x1a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x24e, 0x12, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x24f, 0x5e, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x259, 0x2a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x260, 0x54, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x266, 0x54, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x273, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x274, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2ab, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2ac, 0x02, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x50, 0x0c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x50, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *tensor_zuma_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT] = tensor_zuma_pre_init_cfg,
};

/*
 * Manual (DME) hibern8 needs the PMA parked and woken by software; without
 * this every runtime-resume hibern8 exit fails with PWR_FATAL_ERROR.
 * Downstream zuma ufs-cal.h post_h8_enter / pre_h8_exit, byte offsets / 4.
 */
static const struct samsung_ufs_phy_cfg tensor_zuma_post_h8_enter[] = {
	PHY_TRSV_REG_CFG_ZUMA(0x27d, 0x08, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x280, 0x3a, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x00, 0x51, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2d9, 0x30, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2d9, 0x33, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg tensor_zuma_pre_h8_exit[] = {
	PHY_COMN_REG_CFG(0x00, 0x11, PWR_MODE_ANY),
	PHY_WAIT_CFG(10, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x27d, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x280, 0x30, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2d9, 0x32, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMA(0x2d9, 0x22, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *tensor_zuma_hibern8_cfgs[] = {
	[CFG_POST_HIBERN8_ENTER]	= tensor_zuma_post_h8_enter,
	[CFG_PRE_HIBERN8_EXIT]		= tensor_zuma_pre_h8_exit,
};

static const char * const tensor_zuma_ufs_phy_clks[] = {
	"ref_clk",
};

static int zuma_phy_wait_for_calibration(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	const unsigned int timeout_us = 4000;
	const unsigned int sleep_us = 40;
	u32 val;
	u32 off;
	int err;

	off = PHY_PMA_TRSV_ADDR(PHY_ZUMA_EMB_CAL_DONE, lane);
	err = readl_poll_timeout(ufs_phy->reg_pma + off,
				 val, (val & PHY_ZUMA_EMB_CAL_DONE_BIT),
				 sleep_us, timeout_us);
	if (err)
		dev_warn(ufs_phy->dev,
			 "lane %u embedded phy cal done bit not set (continuing)\n",
			 lane);

	return 0;
}

const struct samsung_ufs_phy_drvdata tensor_zuma_ufs_phy = {
	.cfgs = tensor_zuma_ufs_phy_cfgs,
	.cfgs_hibern8 = tensor_zuma_hibern8_cfgs,
	.isol = {
		.offset = TENSOR_ZUMA_PHY_CTRL,
		.mask = TENSOR_ZUMA_PHY_CTRL_MASK,
		.en = TENSOR_ZUMA_PHY_CTRL_EN,
	},
	.clk_list = tensor_zuma_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(tensor_zuma_ufs_phy_clks),
	.explicit_calibration = true,
	.wait_for_cal = zuma_phy_wait_for_calibration,
};
