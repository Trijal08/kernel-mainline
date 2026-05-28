// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Google Tensor G4 / Zumapro.
 *
 * PMA register values are converted from the downstream Zuma UFS CAL table:
 * private/google-modules/soc/gs/drivers/ufs/zuma/ufs-cal.h.
 */

#include <linux/io.h>
#include <linux/iopoll.h>

#include "phy-samsung-ufs.h"

#define TENSOR_ZUMAPRO_PHY_CTRL		0x3ec0
#define TENSOR_ZUMAPRO_PHY_CTRL_MASK	0x1
#define TENSOR_ZUMAPRO_PHY_CTRL_EN	BIT(0)
#define PHY_ZUMAPRO_LANE_OFFSET	0x200
#define PHY_ZUMAPRO_EMB_CAL_DONE	0x31d
#define PHY_ZUMAPRO_EMB_CAL_DONE_BIT	BIT(0)

#define PHY_PMA_TRSV_ADDR(reg, lane)	(PHY_APB_ADDR((reg) + \
					((lane) * PHY_ZUMAPRO_LANE_OFFSET)))
#define PHY_TRSV_REG_CFG_ZUMAPRO(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, PHY_ZUMAPRO_LANE_OFFSET)

static const struct samsung_ufs_phy_cfg tensor_zumapro_pre_init_cfg[] = {
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
	PHY_TRSV_REG_CFG_ZUMAPRO(0x201, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x202, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x203, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x204, 0x18, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x205, 0xc0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x207, 0x1c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x2ec, 0x8c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x27c, 0xd0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x288, 0xfa, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x289, 0x60, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x234, 0x30, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x239, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x23d, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x24d, 0x1a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x24e, 0x12, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x24f, 0x5e, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x259, 0x2a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x260, 0x54, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x266, 0x54, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x273, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x274, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x2ab, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_ZUMAPRO(0x2ac, 0x02, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x50, 0x0c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x50, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *tensor_zumapro_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT] = tensor_zumapro_pre_init_cfg,
};

static const char * const tensor_zumapro_ufs_phy_clks[] = {
	"ref_clk",
};

static int zumapro_phy_wait_for_calibration(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	const unsigned int timeout_us = 4000;
	const unsigned int sleep_us = 40;
	u32 val;
	u32 off;
	int err;

	off = PHY_PMA_TRSV_ADDR(PHY_ZUMAPRO_EMB_CAL_DONE, lane);
	err = readl_poll_timeout(ufs_phy->reg_pma + off,
				 val, (val & PHY_ZUMAPRO_EMB_CAL_DONE_BIT),
				 sleep_us, timeout_us);
	if (err)
		dev_warn(ufs_phy->dev,
			 "lane %u embedded phy cal done bit not set (continuing)\n",
			 lane);

	return 0;
}

const struct samsung_ufs_phy_drvdata tensor_zumapro_ufs_phy = {
	.cfgs = tensor_zumapro_ufs_phy_cfgs,
	.isol = {
		.offset = TENSOR_ZUMAPRO_PHY_CTRL,
		.mask = TENSOR_ZUMAPRO_PHY_CTRL_MASK,
		.en = TENSOR_ZUMAPRO_PHY_CTRL_EN,
	},
	.clk_list = tensor_zumapro_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(tensor_zumapro_ufs_phy_clks),
	.wait_for_cal = zumapro_phy_wait_for_calibration,
};
