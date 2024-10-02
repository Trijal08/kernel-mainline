// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Samsung ExynosAuto DRM Display Port Link_Training driver.
 *
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 */

#include <linux/err.h>

#include "exynos_drm_dp.h"

static void exynos_dp_dump_symbol_error(struct exynos_dp_subdev *dp)
{
	int i;
	struct device *dev = dp->dev;
	u8 info[MAX_LANE_CNT * 2] = {0, };
	unsigned int offset = DP_SINK_COUNT + 0x10;
	size_t size = sizeof(info);

	drm_dp_dpcd_read(&dp->aux, offset, info, size);
	for (i = 0; i < size; i = i + 2)
		dp_log_info(dev, "SYMBOL_ERROR_COUNT(addr:%#4x), %02x, %02x\n",
			    offset, info[i], info[i + 1]);
};

static void exynos_dp_set_phy_training_lane_set(u32 id,
		u8 *dpcd_buf, struct exynos_dp_lt_info *lt_info)
{
	int i;
	u8 *drive_current = lt_info->voltage_swing;
	u8 *pre_emphasis = lt_info->pre_emphasis;
	u8 lane_cnt = lt_info->lane_cnt;
	u8 max_reach_value = 0;

	for (i = 0; i < lane_cnt; i++) {
		dp_reg_set_phy_tune(id, i, drive_current[i],
				pre_emphasis[i] >> DP_TRAIN_PRE_EMPHASIS_SHIFT);

		if (drive_current[i] >= DP_TRAIN_VOLTAGE_SWING_LEVEL_2)
			max_reach_value |= (DP_TRAIN_MAX_SWING_REACHED);
		else
			max_reach_value &= ~(DP_TRAIN_MAX_SWING_REACHED);

		if (pre_emphasis[i] >= DP_TRAIN_PRE_EMPH_LEVEL_2)
			max_reach_value |= (DP_TRAIN_MAX_PRE_EMPHASIS_REACHED);
		else
			max_reach_value &= ~(DP_TRAIN_MAX_PRE_EMPHASIS_REACHED);

		dpcd_buf[i] = drive_current[i] | pre_emphasis[i] | max_reach_value;
	}
}

static void exynos_dp_dsc_prepare(struct exynos_dp_subdev *dp, bool enable)
{
	int ret;

	if (!drm_dp_sink_supports_dsc(dp->dsc_dpcd))
		return;

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_DSC_ENABLE,
				enable ? DP_DECOMPRESSION_EN : 0);
	if (ret < 0) {
		dp_log_err(dp->dev,
				"Failed to %s sink decompression\n",
				enable ? "enable" : "disable");
		dp->dsc_dpcd[0] = 0; /* deactivate DSC/FEC capable */
		return;
	} else {
		dp_log_info(dp->dev,
				"Success to %s sink decompression\n",
				enable ? "enable" : "disable");
	}

	/* Set FEC ready of DP source device */
	if (drm_dp_sink_supports_fec(dp->fec_capable)) {
		/* Write 1 to clear the FEC_STATUS bit */
		drm_dp_dpcd_writeb(&dp->aux, DP_FEC_STATUS,
			DP_FEC_DECODE_EN_DETECTED | DP_FEC_DECODE_DIS_DETECTED);

		ret = drm_dp_dpcd_writeb(&dp->aux, DP_FEC_CONFIGURATION,
				enable ? DP_FEC_READY : 0);
		if (ret < 0) {
			dp_log_err(dp->dev,
				"Failed to %s sink fec\n",
				enable ? "enable" : "disable");
			return;
		}
		dp_log_info(dp->dev,
				"success to %s sink fec\n",
				enable ? "enable" : "disable");
		dp_reg_set_dsc_fec(dp->id, enable);
	}
}

static void exynos_dp_phy_init(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	u32 id = dp->id;
	u8 dpcd_val[2];

	dpcd_val[0] = dp->lt_info.link_rate;
	dpcd_val[1] = dp->lt_info.lane_cnt;

	dp_reg_phy_reset(id, 1);
	dp_reg_phy_init_setting(id);

	dp_reg_phy_set_link_bw(id, dpcd_val[0]);

	dp_reg_phy_mode_setting(id);

	dp_reg_set_lane_count(id, dpcd_val[1]);

	dp_log_info(dev, "link_rate = %d Mbps, lane_cnt = %x\n",
			drm_dp_bw_code_to_link_rate(dpcd_val[0])/100, dpcd_val[1]);

	if (dp->lt_info.enhanced_frame_cap) {
		dp_reg_set_enhanced_mode(id, 1);
		dpcd_val[1] |= DP_ENHANCED_FRAME_CAP;
	}

	/* wait for 60us - Exynosauto9 DPTX PHY spec */
	udelay(60);

	dp_reg_phy_reset(id, 0);

	drm_dp_dpcd_write(&dp->aux, DP_LINK_BW_SET, dpcd_val, 2);

	dp_reg_wait_phy_pll_lock(id);

	/* SCRAMBLING_DISABLE, TRAINING_PATTERN_1 */
	dp_reg_set_training_pattern(id, TRAINING_PATTERN_1);
	dp_reg_scrambling_enable(id, 0);

	dpcd_val[0] = DP_LINK_SCRAMBLING_DISABLE | DP_TRAINING_PATTERN_1;
	drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET, dpcd_val[0]);
}

static int exynos_dp_reduced_link_rate(u8 link_rate, u8 test_mode)
{
	if (test_mode == DEBUG_LT_BW_NO_STEPDOWN)
		link_rate = DP_LINK_BW_1_62;

	switch (link_rate) {
	case DP_LINK_BW_8_1:
		return DP_LINK_BW_5_4;
	case DP_LINK_BW_5_4:
		return DP_LINK_BW_2_7;
	case DP_LINK_BW_2_7:
		return DP_LINK_BW_1_62;
	case DP_LINK_BW_1_62:
	default:
		return -EINVAL;
	}
}

static bool exynos_drm_dp_get_lt_info(struct exynos_dp_subdev *dp)
{
	int link_rate;
	u8 lane_cnt = drm_dp_max_lane_count(dp->dpcd);
	u8 enhanced_frame_cap;

	link_rate = dp->dpcd[DP_MAX_LINK_RATE];
	lane_cnt = min(lane_cnt, dp->lt_info.max_link_lane);

	if (!link_rate || !lane_cnt)
		return false;

	if (dp->dp_debug.debug_lt == DEBUG_LT_BW_LOWER) {
		link_rate = exynos_dp_reduced_link_rate(link_rate, dp->dp_debug.debug_lt);

		if (link_rate < 0)
			return false;
	}

	if (!exynos_dp_mst_cap(dp))
		enhanced_frame_cap = drm_dp_enhanced_frame_cap(dp->dpcd);
	else
		enhanced_frame_cap = 0;

	dp->lt_info.link_rate = (u8)link_rate;
	dp->lt_info.lane_cnt = lane_cnt;
	dp->lt_info.enhanced_frame_cap = enhanced_frame_cap;

	return true;
}

/*
 * About retry times.
 * The DP 1.4 spec defines retry times in Figure 3-20 : Clock Recovery Sequence of Link Training.
 * Any one of the following true: - LANEx_CR_DONE?
 * #1: Maximum voltage swing reached?
 * #2: AUX w/o LANEx_CR_DONE with the same ADJ_REQ 'five times'? - voltage_retry_no
 * #3: AUX_ACK w/o LANEx_CR_DONE '10 times'? - cr_retry_no
 * LT_CR_RETRY_CNT = Clock Recovery Retry
 * LT_VS_RETRY_CNT = Voltage_Swing Retry
 */
#define LT_CR_RETRY_CNT	10
#define LT_VS_RETRY_CNT	5
static bool
exynos_drm_dp_lt_clock_recovery(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	struct exynos_dp_lt_info lt_info;
	u32 id = dp->id;

	u8 link_status[DP_LINK_STATUS_SIZE];
	u8 lane_cnt = dp->lt_info.lane_cnt;
	u8 dpcd_buf[MAX_LANE_CNT] = {0, };

	int cr_retry_no;
	int voltage_retry_no = 1;
	int i;

	dp_log_info(dev, "Start Clock Recovery(CR)\n");

	for (i = 0; i < MAX_LANE_CNT; i++) {
		lt_info.voltage_swing[i] = 0;
		lt_info.pre_emphasis[i] = 0;
	}
	lt_info.lane_cnt = lane_cnt;

	for (cr_retry_no = 0;
			cr_retry_no < LT_CR_RETRY_CNT; cr_retry_no++) {

		exynos_dp_set_phy_training_lane_set(id, dpcd_buf, &lt_info);

		dp_log_dbg(dev, "(CR) Try TRAINING_LANEx_SET: %02x %02x %02x %02x\n",
				dpcd_buf[0], dpcd_buf[1], dpcd_buf[2], dpcd_buf[3]);
		drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET, dpcd_buf, lane_cnt);

		drm_dp_link_train_clock_recovery_delay(&dp->aux, dp->dpcd);

		drm_dp_dpcd_read_link_status(&dp->aux, link_status);

		if (drm_dp_clock_recovery_ok(link_status, lane_cnt)) {
			goto LT_CR_DONE;
		}

		for (i = 0; i < lane_cnt; i++) {
			if (lt_info.voltage_swing[i] == DP_TRAIN_VOLTAGE_SWING_LEVEL_3) {
				dp_log_info(dev, "LANE_CR_FAIL - Maximum Voltage swing reached(%02x)\n",
						lt_info.voltage_swing[i]);
				goto LT_CR_FAIL;
			}

			lt_info.voltage_swing[i] = drm_dp_get_adjust_request_voltage(link_status, i);
			lt_info.pre_emphasis[i] = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		}

		for (i = 0; i < lane_cnt; i++) {
			if (lt_info.voltage_swing[i] == dp->lt_info.voltage_swing[i]) {
				if (voltage_retry_no == LT_VS_RETRY_CNT) {
					dp_log_info(dev, "LANE_CR_FAIL - Same ADJ_REQ_Voltage %d times(%02x)\n",
							voltage_retry_no, lt_info.voltage_swing[i]);
					goto LT_CR_FAIL;
				} else {
					voltage_retry_no++;
					break;
				}
			} else if (i == lane_cnt-1) {
				voltage_retry_no = 1;
			}
		}
		memcpy(&dp->lt_info.voltage_swing, lt_info.voltage_swing,
				sizeof(dp->lt_info.voltage_swing));
	}

	dp_log_info(dev, "LANE_CR_FAIL - Maximum Retry %d times\n", cr_retry_no);

LT_CR_FAIL:
	dp_log_info(dev, "TRAINING_LANEx_SET: %02x %02x %02x %02x\n",
			dpcd_buf[0], dpcd_buf[1], dpcd_buf[2], dpcd_buf[3]);
	return false;

LT_CR_DONE:
	memcpy(&dp->lt_info.voltage_swing, lt_info.voltage_swing,
			sizeof(dp->lt_info.voltage_swing));
	memcpy(&dp->lt_info.pre_emphasis, lt_info.pre_emphasis,
			sizeof(dp->lt_info.pre_emphasis));
	dp_log_info(dev, "LANE_CR_DONE - TRAINING_LANEx_SET : %02x %02x %02x %02x\n",
			dpcd_buf[0], dpcd_buf[1], dpcd_buf[2], dpcd_buf[3]);
	return true;
}

static u8
exynos_drm_dp_lt_training_pattern(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	u32 id = dp->id;
	u8 link_rate = dp->dpcd[DP_MAX_LINK_RATE];

	if (drm_dp_tps4_supported(dp->dpcd) && link_rate == DP_LINK_BW_8_1) {
		dp_reg_set_training_pattern(id, TRAINING_PATTERN_4);
		dp_reg_scrambling_enable(id, 1);
		dp_log_dbg(dev, "TPS4_supported\n");
		return DP_TRAINING_PATTERN_4;
	}

	dp_reg_scrambling_enable(id, 0);

	if (drm_dp_tps3_supported(dp->dpcd) && link_rate >= DP_LINK_BW_5_4) {
		dp_reg_set_training_pattern(id, TRAINING_PATTERN_3);
		dp_log_dbg(dev, "TPS3_supported\n");
		return DP_TRAINING_PATTERN_3;
	}

	dp_reg_set_training_pattern(id, TRAINING_PATTERN_2);
	dp_log_dbg(dev, "TPS2_supported\n");
	return DP_TRAINING_PATTERN_2;
}

#define EQ_RETRY_CNT	5
static bool
exynos_drm_dp_lt_equalization(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	struct exynos_dp_lt_info *lt_info = &dp->lt_info;
	u32 id = dp->id;

	u8 link_status[DP_LINK_STATUS_SIZE];
	u8 lane_cnt = dp->lt_info.lane_cnt;
	u8 dpcd_buf[MAX_LANE_CNT] = {0, };
	u8 training_pattern;

	int i;
	int eq_retry_no;
	bool cr_done, eq_done;

	training_pattern = exynos_drm_dp_lt_training_pattern(dp);

	dp_log_info(dev, "Start Equalization(EQ) - TPS%d supported\n",
			(training_pattern > DP_TRAINING_PATTERN_3) ? 4 : training_pattern);

	if (training_pattern != DP_TRAINING_PATTERN_4)
		training_pattern |= DP_LINK_SCRAMBLING_DISABLE;

	drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET, training_pattern);

	for (eq_retry_no = 0; eq_retry_no < EQ_RETRY_CNT; eq_retry_no++) {

		exynos_dp_set_phy_training_lane_set(id, dpcd_buf, lt_info);

		dp_log_dbg(dev, "(EQ) Try TRAINING_LANEx_SET: %02x %02x %02x %02x\n",
				dpcd_buf[0], dpcd_buf[1], dpcd_buf[2], dpcd_buf[3]);
		drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET, dpcd_buf, lane_cnt);

		drm_dp_link_train_channel_eq_delay(&dp->aux, dp->dpcd);

		drm_dp_dpcd_read_link_status(&dp->aux, link_status);

		cr_done = drm_dp_clock_recovery_ok(link_status, lane_cnt);

		if (!cr_done) {
			dp_log_info(dev, "LANE_CR_FAIL in LT_EQ\n");
			goto LT_EQ_FAIL;
		}

		eq_done = drm_dp_channel_eq_ok(link_status, lane_cnt);

		if (cr_done && eq_done)
			goto LT_EQ_DONE;

		exynos_dp_dump_symbol_error(dp);

		for (i = 0; i < lane_cnt; i++)
			lt_info->pre_emphasis[i] = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
	}

	dp_log_info(dev, "LANE_EQ_FAIL, Maximum Retry %d times\n", eq_retry_no);

LT_EQ_FAIL:
	dp_log_info(dev, "TRAINING_LANEx_SET: %02x %02x %02x %02x\n",
			dpcd_buf[0], dpcd_buf[1], dpcd_buf[2], dpcd_buf[3]);
	return false;

LT_EQ_DONE:
	dp_log_info(dev, "LANE_EQ_DONE - TRAINING_LANEx_SET: %02x %02x %02x %02x\n",
			dpcd_buf[0], dpcd_buf[1], dpcd_buf[2], dpcd_buf[3]);
	return true;
}

static bool exynos_dp_link_training_skip_reduce_bw_parse(struct device *dev)
{
	return of_property_read_bool(dev->of_node,
			"samsung,lt_skip_reduce_bw");
}

#define LT_RETRY_CNT	4
/**
 * @cnotice
 * @prdcode
 * @unit_name{Exynos_dp_drv}
 * @purpose Operate DP Full link training with RX
 * @logic Operate DP Full link training with DP RX
 * @params
 * @param{in, dp, struct* ::exynos_dp_subdev, None}
 * @endparam
 * @retval{ret, int, 0, <= 0, < 0}
 */
static int exynos_dp_full_link_training(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	u32 id = dp->id;
	int lt_retry_cnt;
	int link_rate;
	bool debug_lt =
		((dp->dp_debug.debug_lt == DEBUG_LT_FAIL_FIXED_BW) ||
		 (dp->dp_debug.debug_lt == DEBUG_LT_FAIL_TRY_BW_DOWN)) ? true : false;
	bool cr_done = false;
	bool eq_done = false;
	bool lt_done = false;

	if (!dp->hpd_state) {
		dp_log_err(dev, "HPD is Low in Full Link Training\n");
		return -EBUSY;
	}

	if (!exynos_drm_dp_get_lt_info(dp)) {
		dp_log_err(dev, "Invalid values of link_rate(%x) or lane_cnt(%x)\n",
				dp->dpcd[DP_MAX_LINK_RATE], drm_dp_max_lane_count(dp->dpcd));
		return -EINVAL;
	}
	dp_log_info(dev, "Start Full Link Training + : DP_REV%02x\n", dp->dpcd[DP_DPCD_REV]);

	/*
	 * This is work-around code when using DP MST serializer.
	 * Link-training succedds only in SST mode in DP MST with TPS4.
	 * Added this sequence for Link-Training called by HPD_IRQ event.
	 */
	if (drm_dp_tps4_supported(dp->dpcd))
		dp_reg_set_mst_en(id, 0);

	for (lt_retry_cnt = 0; lt_retry_cnt < LT_RETRY_CNT; lt_retry_cnt++) {
		if (!dp->hpd_state) {
			dp_log_err(dev, "HPD is Low in Full Link_Training\n");
			return -EINVAL;
		}

		exynos_dp_phy_init(dp);

		cr_done = exynos_drm_dp_lt_clock_recovery(dp);

		if (cr_done) {
			eq_done = exynos_drm_dp_lt_equalization(dp);
			if (eq_done && !debug_lt)
				goto LINK_TRAINING_END;
		}

		if (exynos_dp_link_training_skip_reduce_bw_parse(dev) ||
			(drm_dp->skip_messaging_aux_client == SKIP_MSG_AUX_CLIENT) ||
			(dp->dp_debug.debug_lt == DEBUG_LT_FAIL_FIXED_BW))
			goto LINK_TRAINING_END;

		link_rate = exynos_dp_reduced_link_rate(dp->lt_info.link_rate, dp->dp_debug.debug_lt);

		if (link_rate < 0)
			goto LINK_TRAINING_END;
		else
			dp->lt_info.link_rate = link_rate;
	}

LINK_TRAINING_END:
	lt_done = cr_done && eq_done && !debug_lt;
	dp_log_info(dev, "%s Full Link Training -\n", lt_done ? "Finished" : "Failed");
	drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET, 0);

	return lt_done ? 0 : -EINVAL;
}

static void exynos_dp_mst_configure(struct exynos_dp_subdev *dp, bool is_mst)
{
	struct device *dev = dp->dev;

	if (dp->mst_config && is_mst)
		dp->mst_config(dev, is_mst);
	dp_log_info(dev, "DP link use %s protocol\n", is_mst ? "MST" : "SST");
}

static int exynos_dp_get_dpcd_receiver_capability(struct exynos_dp_subdev *dp)
{
	u8 extend_val[DP_RECEIVER_CAP_SIZE] = {0,};
	struct device_node *np = dp->dev->of_node;
	int ret = 0;

	ret = drm_dp_dpcd_read(&dp->aux, DP_DPCD_REV, dp->dpcd, sizeof(dp->dpcd));

	if (ret < 0)
		return ret;

	if (dp->dp_debug.debug_lt == DEBUG_LT_DPCD_READ_FAIL)
		return -EBUSY;

	if (dp->dpcd[DP_TRAINING_AUX_RD_INTERVAL] & DP_EXTENDED_RECEIVER_CAP_FIELD_PRESENT) {
		drm_dp_dpcd_read(&dp->aux, DP_DP13_DPCD_REV, extend_val, sizeof(extend_val));
		if (extend_val[DP_DPCD_REV] > dp->dpcd[DP_DPCD_REV])
			memcpy(dp->dpcd, extend_val, sizeof(extend_val));
	}

	/* clear first before read */
	dp->dsc_dpcd[0] = 0;
	dp->fec_capable = 0;
	/* DSC and FEC DPCD if DP rev >= 1.4 but some MST_HUB uses 1.2 */
	if (dp->dpcd[DP_DPCD_REV] >= DP_DPCD_REV_12) {
		dp->force_dsc_dis =
			of_property_read_bool(np, "samsung,force-dsc-dis");
		dp->mst_dsc_en =
			of_property_read_bool(np, "samsung,mst-dsc-en");
		/* skip read DSC capability */
		if (dp->force_dsc_dis)
			return ret;

		drm_dp_dpcd_read(&dp->aux, DP_DSC_SUPPORT, dp->dsc_dpcd,
				sizeof(dp->dsc_dpcd));
		drm_dp_dpcd_readb(&dp->aux, DP_FEC_CAPABILITY, &dp->fec_capable);
	}
	return ret;
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{Exynos_dp_drv}
 * @purpose DP link training
 * @logic Check the HPD state and disable DP irq<br>
 * and than running the DP link_training<br>
 * If the link_training is successful and in MST mode, excute MST opeation.
 * @params
 * @param{in, dp, struct* ::exynos_dp_subdev, None}
 * @endparam
 * @retval{ret, int, 0, <= 0, < 0}
 */
int exynos_drm_dp_link_training(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	bool is_mst;
	int ret = 0;

	if (!dp->hpd_state) {
		dp_log_info(dev, "hpd is low in link training\n");
		return 0;
	}

	dp_log_info(dev, "exynos_drm_dp_link_training\n");
	dp_reg_set_plug_interrupt(dp->id, 0);

	dp_log_info(dev, "exynos_drm_dp_link_training1\n");

	ret = exynos_dp_get_dpcd_receiver_capability(dp);

	if (ret < 0) {
		dp_log_err(dev, "DPCD read error(ret=%d)\n", ret);
		goto LT_END;
	}

	dp_log_info(dev, "exynos_drm_dp_link_training2\n");
	mutex_lock(&dp->lock);
	ret = exynos_dp_full_link_training(dp);
	mutex_unlock(&dp->lock);

	if (ret < 0) {
		exynos_drm_dp_dpcd_status_dump(dp);
		goto LT_END;
	}

	is_mst = exynos_dp_mst_cap(dp);
	exynos_dp_dsc_prepare(dp, true);
	dp_log_info(dev, "exynos_drm_dp_link_training3\n");

	if (is_mst) {
		exynos_drm_dp_set_normal_data(dp);

		drm_dp_dpcd_read(&dp->aux, DP_DOWNSTREAM_PORT_0,
				dp->downstream_ports,
				sizeof(dp->downstream_ports));
		dp_reg_set_mst_en(dp->id, 1);
	}

	if (dp->hpd_state == HPD_CHECK)
		exynos_dp_mst_configure(dp, is_mst);

LT_END:
	dp_reg_set_plug_interrupt(dp->id, 1);
	return ret;
}

