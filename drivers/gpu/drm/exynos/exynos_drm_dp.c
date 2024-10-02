// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Samsung ExynosAuto DRM Display Port driver
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 */
#include <linux/console.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/of_device.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <drm/drm_atomic_helper.h>	/* drm_atomic_helperxxx */

#include <drm/drm_edid.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dp_mst_helper.h>
#include <drm/display/drm_dsc_helper.h>

#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_dp.h"

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 Samsung Electronics Co.Ltd
 */
#include <linux/delay.h> /* udelay */
#include <linux/err.h> /* EBUSY, EINVAL */
#include <linux/export.h> /* EXPORT_SYMBOL */
#include <linux/io.h> /* for use 'readl()', 'writel()' functions */
#include <linux/kernel.h> /* DIV_ROUND_CLOSEST */
#include <linux/printk.h> /* pr_xxx */
#include <linux/types.h> /* uint32_t, __iomem, ... */
#include <video/mipi_display.h>
#include <drm/drm_mipi_dsi.h> /* MIPI_DSI_* */
#include <linux/iopoll.h> /* readx_poll_timeout_atomic */

#define MAX_LANE				4

#define SYSTEM_DEVICE_VERSION			(0x0000)
#define DEVICE_VERSION				(0xFFFFFFFF << 0)

#define SYSTEM_SW_RESET_CONTROL			(0x0004)
#define SW_RESET				(0x01 << 0)

#define SYSTEM_CLK_CONTROL			(0x0008)
#define GFMUX_STATUS_TXCLK			(0x03 << 8)
#define GFMUX_STATUS_TXCLK_ON			(0x02)
#define TXCLK_SEL				(0x01 << 5)
#define TXCLK_SEL_MODE				(0x01 << 4)
#define OSC_CLK_SEL				(0x01 << 0)

#define SYSTEM_MAIN_LINK_BANDWIDTH		(0x000C)
#define LINK_BW_SET			        (0x1F << 0)

#define SYSTEM_MAIN_LINK_LANE_COUNT		(0x0010)
#define LANE_COUNT_SET			        (0x07 << 0)

#define SYSTEM_SW_FUNCTION_ENABLE		(0x0014)
#define SW_FUNC_EN			        (0x01 << 0)

#define SYSTEM_COMMON_FUNCTION_ENABLE		(0x0018)
#define HDCP22_FUNC_EN				(0x01 << 4)
#define HDCP13_FUNC_EN				(0x01 << 3)
#define GTC_FUNC_EN				(0x01 << 2)
#define PCS_FUNC_EN				(0x01 << 1)
#define AUX_FUNC_EN				(0x01 << 0)

#define SYSTEM_SST1_FUNCTION_ENABLE		(0x001C)
#define SST1_LH_PWR_ON_STATUS			(0x01 << 5)
#define SST1_LH_PWR_ON				(0x01 << 4)
#define SST1_AUDIO_FIFO_FUNC_EN			(0x01 << 2)
#define SST1_AUDIO_FUNC_EN			(0x01 << 1)
#define SST1_VIDEO_FUNC_EN			(0x01 << 0)

#define SYSTEM_SST2_FUNCTION_ENABLE		(0x0020)

#define SYSTEM_SST3_FUNCTION_ENABLE		(0x0030)

#define SYSTEM_SST4_FUNCTION_ENABLE		(0x0034)

#define SYSTEM_MISC_CONTROL			(0x0024)
#define MISC_CTRL_EN				(0x01 << 1)
#define HDCP_HPD_RST				(0x01 << 0)

#define SYSTEM_HPD_CONTROL			(0x0028)
#define HPD_HDCP				(0x01 << 30)
#define HPD_DEGLITCH_COUNT			(0x3FFF << 16)
#define HPD_STATUS				(0x01 << 8)
#define HPD_EVENT_UNPLUG			(0x01 << 7)
#define HPD_EVENT_PLUG				(0x01 << 6)
#define HPD_EVENT_IRQ				(0x01 << 5)
#define HPD_EVENT_CTRL_EN			(0x01 << 4)
#define HPD_FORCE				(0x01 << 1)
#define HPD_FORCE_EN				(0x01 << 0)

#define SYSTEM_PLL_LOCK_CONTROL			(0x002C)
#define PLL_LOCK_STATUS				(0x01 << 4)
#define PLL_LOCK_FORCE				(0x01 << 3)
#define PLL_LOCK_FORCE_EN			(0x01 << 2)

#define SYSTEM_INTERRUPT_CONTROL		(0x0100)
#define SW_INTR_CTRL				(0x01 << 1)
#define INTR_POL				(0x01 << 0)

#define SYSTEM_INTERRUPT_REQUEST		(0x0104)
#define IRQ_MST					(0x01 << 3)
#define IRQ_STR2				(0x01 << 2)
#define IRQ_STR1				(0x01 << 1)
#define IRQ_COMMON				(0x01 << 0)

#define SYSTEM_IRQ_COMMON_STATUS		(0x0108)
#define HDCP_ENC_EN_CHG				(0x01 << 22)
#define HDCP_LINK_CHK_FAIL			(0x01 << 21)
#define HDCP_R0_CHECK_FLAG			(0x01 << 20)
#define HDCP_BKSV_RDY				(0x01 << 19)
#define HDCP_SHA_DONE				(0x01 << 18)
#define HDCP_AUTH_STATE_CHG			(0x01 << 17)
#define HDCP_AUTH_DONE				(0x01 << 16)
#define HPD_IRQ_FLAG				(0x01 << 11)
#define HPD_CHG					(0x01 << 10)
#define HPD_LOST				(0x01 << 9)
#define HPD_PLUG_INT				(0x01 << 8)
#define AUX_REPLY_RECEIVED			(0x01 << 5)
#define AUX_ERR					(0x01 << 4)
#define PLL_LOCK_CHG				(0x01 << 1)
#define SW_INTR					(0x01 << 0)

#define SYSTEM_IRQ_COMMON_STATUS_MASK		(0x010C)
#define MST_BUF_OVERFLOW_MASK			(0x01 << 23)
#define HDCP_ENC_EN_CHG_MASK			(0x01 << 22)
#define HDCP_LINK_CHK_FAIL_MASK			(0x01 << 21)
#define HDCP_R0_CHECK_FLAG_MASK			(0x01 << 20)
#define HDCP_BKSV_RDY_MASK			(0x01 << 19)
#define HDCP_SHA_DONE_MASK			(0x01 << 18)
#define HDCP_AUTH_STATE_CHG_MASK		(0x01 << 17)
#define HDCP_AUTH_DONE_MASK			(0x01 << 16)
#define HPD_IRQ_MASK				(0x01 << 11)
#define HPD_CHG_MASK				(0x01 << 10)
#define HPD_LOST_MASK				(0x01 << 9)
#define HPD_PLUG_MASK				(0x01 << 8)
#define AUX_REPLY_RECEIVED_MASK			(0x01 << 5)
#define AUX_ERR_MASK				(0x01 << 4)
#define PLL_LOCK_CHG_MASK			(0x01 << 1)
#define SW_INTR_MASK				(0x01 << 0)

#define SYSTEM_DEBUG				(0x0200)
#define AUX_HPD_CONTROL				(0x01 << 2)
#define CLKGATE_DISABLE				(0x01 << 1)

#define SYSTEM_DEBUG_LH_PCH			(0x0204)
#define SST4_LH_PSTATE				(0x01 << 28)
#define SST4_LH_PCH_FSM_STATE			(0x0F << 24)
#define SST3_LH_PSTATE				(0x01 << 20)
#define SST3_LH_PCH_FSM_STATE			(0x0F << 16)
#define SST2_LH_PSTATE				(0x01 << 12)
#define SST2_LH_PCH_FSM_STATE			(0x0F << 8)
#define SST1_LH_PSTATE				(0x01 << 4)
#define SST1_LH_PCH_FSM_STATE			(0x0F << 0)

#define AUX_CONTROL				(0x1000)
#define AUX_POWER_DOWN				(0x01 << 16)
#define AUX_REPLY_TIMER_MODE			(0x03 << 12)
#define AUX_REPLY_TIMER_VAL(_v_)		((_v_) << 12)
#define AUX_RETRY_TIMER				(0x07 << 8)
#define AUX_PN_INV				(0x01 << 1)
#define REG_MODE_SEL				(0x01 << 0)

#define AUX_TRANSACTION_START			(0x1004)
#define AUX_TRAN_START				(0x01 << 0)

#define AUX_BUFFER_CLEAR			(0x1008)
#define AUX_BUF_CLR				(0x01 << 0)

#define AUX_ADDR_ONLY_COMMAND			(0x100C)
#define ADDR_ONLY_CMD				(0x01 << 0)

#define AUX_REQUEST_CONTROL			(0x1010)
#define REQ_COMM				(0x0F << 28)
#define REQ_COMM_VAL(_v_)			((_v_) << 28)
#define REQ_ADDR				(0xFFFFF << 8)
#define REQ_ADDR_VAL(_v_)			((_v_) << 8)
#define REQ_LENGTH				(0x3F << 0)

#define AUX_COMMAND_CONTROL			(0x1014)
#define DEFER_CTRL_EN				(0x01 << 8)
#define DEFER_COUNT				(0x7F << 0)

#define AUX_MONITOR_1				(0x1018)
#define AUX_BUF_DATA_COUNT			(0x7F << 24)
#define AUX_DETECTED_PERIOD_MON			(0x1FF << 12)
#define AUX_CMD_STATUS				(0x0F << 8)
#define AUX_RX_COMM				(0x0F << 4)
#define AUX_LAST_MODE				(0x01 << 3)
#define AUX_BUSY				(0x01 << 2)
#define AUX_REQ_WAIT_GRANT			(0x01 << 1)
#define AUX_REQ_SIGNAL				(0x01 << 0)

#define AUX_MONITOR_2				(0x101C)
#define AUX_ERROR_NUMBER			(0xFF << 0)

#define AUX_TX_DATA_SET0			(0x1030)
#define TX_DATA_3				(0xFF << 24)
#define TX_DATA_2				(0xFF << 16)
#define TX_DATA_1				(0xFF << 8)
#define TX_DATA_0				(0xFF << 0)

#define AUX_TX_DATA_SET1			(0x1034)
#define TX_DATA_7				(0xFF << 24)
#define TX_DATA_6				(0xFF << 16)
#define TX_DATA_5				(0xFF << 8)
#define TX_DATA_4				(0xFF << 0)

#define AUX_TX_DATA_SET2			(0x1038)
#define TX_DATA_11				(0xFF << 24)
#define TX_DATA_10				(0xFF << 16)
#define TX_DATA_9				(0xFF << 8)
#define TX_DATA_8				(0xFF << 0)

#define AUX_TX_DATA_SET3			(0x103C)
#define TX_DATA_15				(0xFF << 24)
#define TX_DATA_14				(0xFF << 16)
#define TX_DATA_13				(0xFF << 8)
#define TX_DATA_12				(0xFF << 0)

#define AUX_RX_DATA_SET0			(0x1040)
#define RX_DATA_3				(0xFF << 24)
#define RX_DATA_2				(0xFF << 16)
#define RX_DATA_1				(0xFF << 8)
#define RX_DATA_0				(0xFF << 0)

#define AUX_RX_DATA_SET1			(0x1044)
#define RX_DATA_7				(0xFF << 24)
#define RX_DATA_6				(0xFF << 16)
#define RX_DATA_5				(0xFF << 8)
#define RX_DATA_4				(0xFF << 0)

#define AUX_RX_DATA_SET2			(0x1048)
#define RX_DATA_11				(0xFF << 24)
#define RX_DATA_10				(0xFF << 16)
#define RX_DATA_9				(0xFF << 8)
#define RX_DATA_8				(0xFF << 0)

#define AUX_RX_DATA_SET3			(0x104C)
#define RX_DATA_15				(0xFF << 24)
#define RX_DATA_14				(0xFF << 16)
#define RX_DATA_13				(0xFF << 8)
#define RX_DATA_12				(0xFF << 0)

#define GTC_CONTROL				(0x1100)
#define GTC_DELTA_ADJ_EN			(0x01 << 2)
#define IMPL_OPTION				(0x01 << 1)
#define GTC_TX_MASTER				(0x01 << 0)

#define GTC_WORK_ENABLE				(0x1104)
#define GTC_WORK_EN				(0x01 << 0)

#define GTC_TIME_CONTROL			(0x1108)
#define TIME_PERIOD_SEL				(0x03 << 28)
#define TIME_PERIOD_FRACTIONAL			(0xFFFFF << 8)
#define TIME_PERIOD_INT1			(0x0F << 4)
#define TIME_PERIOD_INT2			(0x0F << 0)

#define GTC_ATTEMPT_CONTROL			(0x110C)
#define GTC_STATE_CHANGE_CTRL			(0x01 << 8)
#define NUM_GTC_ATTEMPT				(0xFF << 0)

#define GTC_TX_VALUE_MONITOR			(0x1110)
#define GTC_TX_VAL				(0xFFFFFFFF << 0)

#define GTC_RX_VALUE_MONITOR			(0x1114)
#define GTC_RX_VAL				(0xFFFFFFFF << 0)

#define GTC_STATUS_MONITOR			(0x1118)
#define FREQ_ADJ_10_3				(0xFF << 8)
#define FREQ_ADJ_2_0				(0x07 << 5)
#define TXGTC_LOCK_DONE_FLAG			(0x01 << 1)
#define RXGTC_LOCK_DONE_FLAG			(0x01 << 0)

#define AUX_GTC_DEBUG				(0x1200)
#define DEBUG_STATE_SEL				(0xFF << 8)
#define DEBUG_STATE				(0xFF << 0)

#define MST_ENABLE				(0x2000)
#define MST_EN					(0x01 << 0)

#define MST_VC_PAYLOAD_UPDATE_FLAG		(0x2004)
#define VC_PAYLOAD_UPDATE_FLAG			(0x01 << 0)

#define MST_STREAM_1_ENABLE			(0x2010)
#define STRM_1_EN				(0x01 << 0)

#define MST_STREAM_1_HDCP_CTRL			(0x2014)
#define STRM_1_HDCP22_TYPE			(0x01 << 1)
#define STRM_1_HDCP_EN				(0x01 << 0)

#define MST_STREAM_1_X_VALUE			(0x2018)
#define MST_STREAM_X_VALUE(sst_id)		(MST_STREAM_1_X_VALUE + (0x10 * (sst_id)))
#define STRM_VALUE				(0xFF << 0)

#define MST_STREAM_1_Y_VALUE			(0x201C)
#define MST_STREAM_Y_VALUE(sst_id)		(MST_STREAM_1_Y_VALUE + (0x10 * (sst_id)))

#define MST_VC_PAYLOAD_ID_TIMESLOT_01_08	(0x2050)
#define VC_PAYLOAD_ID_TIMESLOT_01		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_02		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_03		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_04		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_05		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_06		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_07		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_08		(0x03 << 0)
#define VC_PAYLOAD_ID_MASK(mask)		(~(0x07 << (mask)))

#define MST_VC_PAYLOAD_ID_TIMESLOT_09_16	(0x2054)
#define VC_PAYLOAD_ID_TIMESLOT_09		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_10		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_11		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_12		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_13		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_14		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_15		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_16		(0x03 << 0)

#define MST_VC_PAYLOAD_ID_TIMESLOT_17_24	(0x2058)
#define VC_PAYLOAD_ID_TIMESLOT_17		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_18		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_19		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_20		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_21		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_22		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_23		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_24		(0x03 << 0)

#define MST_VC_PAYLOAD_ID_TIMESLOT_25_32	(0x205C)
#define VC_PAYLOAD_ID_TIMESLOT_25		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_26		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_27		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_28		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_29		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_30		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_31		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_32		(0x03 << 0)

#define MST_VC_PAYLOAD_ID_TIMESLOT_33_40	(0x2060)
#define VC_PAYLOAD_ID_TIMESLOT_33		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_34		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_35		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_36		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_37		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_38		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_39		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_40		(0x03 << 0)

#define MST_VC_PAYLOAD_ID_TIMESLOT_41_48	(0x2064)
#define VC_PAYLOAD_ID_TIMESLOT_41		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_42		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_43		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_44		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_45		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_46		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_47		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_48		(0x03 << 0)

#define MST_VC_PAYLOAD_ID_TIMESLOT_49_56	(0x2068)
#define VC_PAYLOAD_ID_TIMESLOT_49		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_50		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_51		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_52		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_53		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_54		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_55		(0x03 << 4)
#define VC_PAYLOAD_ID_TIMESLOT_56		(0x03 << 0)

#define MST_VC_PAYLOAD_ID_TIMESLOT_57_63	(0x206C)
#define VC_PAYLOAD_ID_TIMESLOT_57		(0x03 << 28)
#define VC_PAYLOAD_ID_TIMESLOT_58		(0x03 << 24)
#define VC_PAYLOAD_ID_TIMESLOT_59		(0x03 << 20)
#define VC_PAYLOAD_ID_TIMESLOT_60		(0x03 << 16)
#define VC_PAYLOAD_ID_TIMESLOT_61		(0x03 << 12)
#define VC_PAYLOAD_ID_TIMESLOT_62		(0x03 << 8)
#define VC_PAYLOAD_ID_TIMESLOT_63		(0x03 << 4)

#define MST_ETC_OPTION				(0x2070)
#define ALLOCATE_TIMESLOT_CHECK_TO_ACT		(0x01 << 1)
#define HDCP22_LVP_TYPE				(0x01 << 0)

#define MST_BUF_STATUS				(0x2064)
#define STRM_2_BUF_OVERFLOW			(0x01 << 5)
#define STRM_1_BUF_OVERFLOW			(0x01 << 4)
#define STRM_2_BUF_OVERFLOW_CLEAR		(0x01 << 1)
#define STRM_1_BUF_OVERFLOW_CLEAR		(0x01 << 0)

#define PCS_CONTROL				(0x3000)
#define FEC_READY				(0x01 << 9)
#define FEC_FUNC_EN				(0x01 << 8)
#define LINK_TRAINING_PATTERN_SET		(0x07 << 4)
#define LINK_TRAINING_PATTERN_SET_VAL(_v_)	((_v_) << 4)
#define BYTE_SWAP				(0x01 << 3)
#define BIT_SWAP				(0x01 << 2)
#define SCRAMBLE_RESET_VALUE			(0x01 << 1)
#define SCRAMBLE_BYPASS				(0x01 << 0)

#define PCS_LANE_CONTROL			(0x3004)
#define LANE_DATA_INV_EN			(0x01 << 20)
#define LANE3_DATA_INV				(0x01 << 19)
#define LANE2_DATA_INV				(0x01 << 18)
#define LANE1_DATA_INV				(0x01 << 17)
#define LANE0_DATA_INV				(0x01 << 16)
#define LANE3_MAP				(0x03 << 12)
#define LANE3_MAP_VAL(_v_)			((_v_) << 12)
#define LANE2_MAP				(0x03 << 8)
#define LANE2_MAP_VAL(_v_)			((_v_) << 8)
#define LANE1_MAP				(0x03 << 4)
#define LANE1_MAP_VAL(_v_)			((_v_) << 4)
#define LANE0_MAP				(0x03 << 0)

#define PCS_TEST_PATTERN_CONTROL		(0x3008)
#define TEST_PATTERN				(0x3F << 8)
#define LINK_QUALITY_PATTERN_SET		(0x07 << 0)

#define PCS_TEST_PATTERN_SET0			(0x300C)
#define TEST_80BIT_PATTERN_SET0			(0xFFFFFFFF << 0)

#define PCS_TEST_PATTERN_SET1			(0x3010)
#define TEST_80BIT_PATTERN_SET1			(0xFFFFFFFF << 0)

#define PCS_TEST_PATTERN_SET2			(0x3014)
#define TEST_80BIT_PATTERN_SET2			(0xFFFF << 0)

#define PCS_DEBUG_CONTROL			(0x3018)
#define FEC_FLIP_CDADJ_CODES_CASE4		(0x01 << 6)
#define FEC_FLIP_CDADJ_CODES_CASE2		(0x01 << 5)
#define FEC_DECODE_EN_4TH_SEL			(0x01 << 4)
#define FEC_DECODE_DIS_4TH_SEL			(0x01 << 3)
#define PRBS7_OPTION				(0x01 << 2)
#define DISABLE_AUTO_RESET_ENCODE		(0x01 << 1)
#define PRBS31_EN				(0x01 << 0)

#define PCS_HBR2_EYE_SR_CONTROL			(0x3020)
#define HBR2_EYE_SR_CTRL			(0x03 << 16)
#define HBR2_EYE_SR_COUNT			(0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_1			(0x3100)
#define SA_CRC_CLEAR				(0x01 << 13)
#define SA_CRC_SW_COMPARE			(0x01 << 12)
#define SA_CRC_LN3_PASS				(0x01 << 11)
#define SA_CRC_LN2_PASS				(0x01 << 10)
#define SA_CRC_LN1_PASS				(0x01 << 9)
#define SA_CRC_LN0_PASS				(0x01 << 8)
#define SA_CRC_LN3_FAIL				(0x01 << 7)
#define SA_CRC_LN2_FAIL				(0x01 << 6)
#define SA_CRC_LN1_FAIL				(0x01 << 5)
#define SA_CRC_LN0_FAIL				(0x01 << 4)
#define SA_CRC_LANE_3_ENABLE			(0x01 << 3)
#define SA_CRC_LANE_2_ENABLE			(0x01 << 2)
#define SA_CRC_LANE_1_ENABLE			(0x01 << 1)
#define SA_CRC_LANE_0_ENABLE			(0x01 << 0)

#define PCS_SA_CRC_CONTROL_2			(0x3104)
#define SA_CRC_LN0_REF				(0xFFFF << 16)
#define SA_CRC_LN0_RESULT			(0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_3			(0x3108)
#define SA_CRC_LN1_REF				(0xFFFF << 16)
#define SA_CRC_LN1_RESULT			(0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_4			(0x310C)
#define SA_CRC_LN2_REF				(0xFFFF << 16)
#define SA_CRC_LN2_RESULT			(0xFFFF << 0)

#define PCS_SA_CRC_CONTROL_5			(0x3110)
#define SA_CRC_LN3_REF				(0xFFFF << 16)
#define SA_CRC_LN3_RESULT			(0xFFFF << 0)

#define HDCP13_STATUS				(0x4000)
#define REAUTH_REQUEST				(0x01 << 7)
#define AUTH_FAIL				(0x01 << 6)
#define HW_1ST_AUTHEN_PASS			(0x01 << 5)
#define BKSV_VALID				(0x01 << 3)
#define ENCRYPT					(0x01 << 2)
#define HW_AUTHEN_PASS				(0x01 << 1)
#define AKSV_VALID				(0x01 << 0)

#define HDCP13_CONTROL_0			(0x4004)
#define SW_STORE_AN				(0x01 << 7)
#define SW_RX_REPEATER				(0x01 << 6)
#define HW_RE_AUTHEN				(0x01 << 5)
#define SW_AUTH_OK				(0x01 << 4)
#define HW_AUTH_EN				(0x01 << 3)
#define HDCP13_ENC_EN				(0x01 << 2)
#define HW_1ST_PART_ATHENTICATION_EN		(0x01 << 1)
#define HW_2ND_PART_ATHENTICATION_EN		(0x01 << 0)

#define HDCP13_CONTROL_1			(0x4008)
#define DPCD_REV_1_2				(0x01 << 3)
#define HW_AUTH_POLLING_MODE			(0x01 << 1)
#define HDCP_INT				(0x01 << 0)

#define HDCP13_AKSV_0				(0x4010)
#define AKSV0					(0xFFFFFFFF << 0)

#define HDCP13_AKSV_1				(0x4014)
#define AKSV1					(0xFF << 0)

#define HDCP13_AN_0				(0x4018)
#define AN0					(0xFFFFFFFF << 0)

#define HDCP13_AN_1				(0x401C)
#define AN1					(0xFFFFFFFF << 0)

#define HDCP13_BKSV_0				(0x4020)
#define BKSV0					(0xFFFFFFFF << 0)

#define HDCP13_BKSV_1				(0x4024)
#define BKSV1					(0xFF << 0)

#define HDCP13_R0_REG				(0x4028)
#define R0					(0xFFFF << 0)

#define HDCP13_BCAPS				(0x4030)
#define BCAPS					(0xFF << 0)

#define HDCP13_BINFO_REG			(0x4034)
#define BINFO					(0xFF << 0)

#define HDCP13_DEBUG_CONTROL			(0x4038)
#define CHECK_KSV				(0x01 << 2)
#define REVOCATION_CHK_DONE			(0x01 << 1)
#define HW_SKIP_RPT_ZERO_DEV			(0x01 << 0)

#define HDCP13_AUTH_DBG				(0x4040)
#define DDC_STATE				(0x07 << 5)
#define AUTH_STATE				(0x1F << 0)

#define HDCP13_ENC_DBG				(0x4044)
#define ENC_STATE				(0x07 << 3)

#define HDCP13_AM0_0				(0x4048)
#define AM0_0					(0xFFFFFFFF << 0)

#define HDCP13_AM0_1				(0x404C)
#define AM0_1					(0xFFFFFFFF << 0)

#define HDCP13_WAIT_R0_TIME			(0x4054)
#define HW_WRITE_AKSV_WAIT			(0xFF << 0)

#define HDCP13_LINK_CHK_TIME			(0x4058)
#define LINK_CHK_TIMER				(0xFF << 0)

#define HDCP13_REPEATER_READY_WAIT_TIME		(0x405C)
#define HW_RPTR_RDY_TIMER			(0xFF << 0)

#define HDCP13_READY_POLL_TIME			(0x4060)
#define POLLING_TIMER_TH			(0xFF << 0)

#define HDCP13_STREAM_ID_ENCRYPTION_CONTROL	(0x4068)
#define STRM_ID_ENC_UPDATE			(0x01 << 7)
#define STRM_ID_ENC				(0x7F << 0)

#define HDCP22_SYS_EN				(0x4400)
#define SYSTEM_ENABLE				(0x01 << 0)

#define HDCP22_CONTROL				(0x4404)
#define HDCP22_BYPASS_MODE			(0x01 << 1)
#define HDCP22_ENC_EN				(0x01 << 0)

#define HDCP22_CONTROL				(0x4404)
#define HDCP22_BYPASS_MODE			(0x01 << 1)
#define HDCP22_ENC_EN				(0x01 << 0)

#define HDCP22_STREAM_TYPE			(0x4454)
#define STREAM_TYPE				(0x01 << 0)

#define HDCP22_LVP				(0x4460)
#define LINK_VERIFICATION_PATTERN		(0xFFFF << 0)

#define HDCP22_LVP_GEN				(0x4464)
#define LVP_GEN					(0x01 << 0)

#define HDCP22_LVP_CNT_KEEP			(0x4468)
#define LVP_COUNT_KEEP_ENABLE			(0x01 << 0)

#define HDCP22_LANE_DECODE_CTRL			(0x4470)
#define ENHANCED_FRAMING_MODE			(0x01 << 3)
#define LVP_EN_DECODE_ENABLE			(0x01 << 2)
#define ENCRYPTION_SIGNAL_DECODE_ENABLE		(0x01 << 1)
#define LANE_DECODE_ENABLE			(0x01 << 0)

#define HDCP22_SR_VALUE				(0x4480)
#define SR_VALUE				(0xFF << 0)

#define HDCP22_CP_VALUE				(0x4484)
#define CP_VALUE				(0xFF << 0)

#define HDCP22_BF_VALUE				(0x4488)
#define BF_VALUE				(0xFF << 0)

#define HDCP22_BS_VALUE				(0x448C)
#define BS_VALUE				(0xFF << 0)

#define HDCP22_RIV_XOR				(0x4490)
#define RIV_XOR_LOCATION			(0x01 << 0)

#define HDCP22_RIV_0				(0x4500)
#define RIV_KEY_0				(0xFFFFFFFF << 0)

#define HDCP22_RIV_1				(0x4504)
#define RIV_KEY_1				(0xFFFFFFFF << 0)

#define SST1_MAIN_CONTROL			(0x5000)
#define MVID_MODE				(0x01 << 11)
#define MAUD_MODE				(0x01 << 10)
#define MVID_UPDATE_RATE			(0x03 << 8)
#define VIDEO_MODE				(0x01 << 6)
#define ENHANCED_MODE				(0x01 << 5)
#define ODD_TU_CONTROL				(0x01 << 4)

#define SST1_MAIN_FIFO_CONTROL			(0x5004)
#define CLEAR_AUDIO_FIFO			(0x01 << 3)
#define CLEAR_PIXEL_MAPPING_FIFO		(0x01 << 2)
#define CLEAR_MAPI_FIFO				(0x01 << 1)
#define CLEAR_GL_DATA_FIFO			(0x01 << 0)

#define SST1_GNS_CONTROL			(0x5008)
#define RS_CTRL					(0x01 << 0)

#define SST1_SR_CONTROL				(0x500C)
#define SR_COUNT_RESET_VALUE			(0x1FF << 16)
#define SR_REPLACE_BS_COUNT			(0x1F << 4)
#define SR_START_CTRL				(0x01 << 1)
#define SR_REPLACE_BS_EN			(0x01 << 0)

#define SST1_INTERRUPT_MONITOR			(0x5020)
#define INT_STATE				(0x01 << 0)

#define SST1_INTERRUPT_STATUS_SET0		(0x5024)
#define VSC_SDP_TX_INCOMPLETE			(0x01 << 9)
#define MAPI_FIFO_UNDER_FLOW			(0x01 << 8)
#define VSYNC_DET				(0x01 << 7)

#define SST1_INTERRUPT_STATUS_SET1		(0x5028)
#define AFIFO_UNDER				(0x01 << 7)
#define AFIFO_OVER				(0x01 << 6)

#define SST1_INTERRUPT_MASK_SET0		(0x502C)
#define VSC_SDP_TX_INCOMPLETE_MASK		(0x01 << 9)
#define MAPI_FIFO_UNDER_FLOW_MASK		(0x01 << 8)
#define VSYNC_DET_MASK				(0x01 << 7)

#define SST1_INTERRUPT_MASK_SET1		(0x5030)
#define AFIFO_UNDER_MASK			(0x01 << 7)
#define AFIFO_OVER_MASK				(0x01 << 6)

#define SST1_MVID_CALCULATION_CONTROL		(0x5040)
#define MVID_GEN_FILTER_TH			(0xFF << 8)
#define MVID_GEN_FILTER_EN			(0x01 << 0)

#define SST1_MVID_MASTER_MODE			(0x5044)
#define MVID_MASTER				(0xFFFFFFFF << 0)

#define SST1_NVID_MASTER_MODE			(0x5048)
#define NVID_MASTER				(0xFFFFFFFF << 0)

#define SST1_MVID_SFR_CONFIGURE			(0x504C)
#define MVID_SFR_CONFIG				(0xFFFFFF << 0)

#define SST1_NVID_SFR_CONFIGURE			(0x5050)
#define NVID_SFR_CONFIG				(0xFFFFFF << 0)

#define SST1_MVID_MONITOR			(0x5054)
#define MVID_MON				(0xFFFFFF << 0)

#define SST1_MAUD_CALCULATION_CONTROL		(0x5058)
#define M_AUD_GEN_FILTER_TH			(0xFF << 8)
#define M_AUD_GEN_FILTER_EN			(0x01 << 0)

#define SST1_MAUD_MASTER_MODE			(0x505C)
#define MAUD_MASTER				(0xFFFFFFFF << 0)

#define SST1_NAUD_MASTER_MODE			(0x5060)
#define NAUD_MASTER				(0xFFFFFF << 0)

#define SST1_MAUD_SFR_CONFIGURE			(0x5064)
#define MAUD_SFR_CONFIG				(0xFFFFFF << 0)

#define SST1_NAUD_SFR_CONFIGURE			(0x5068)
#define NAUD_SFR_CONFIG				(0xFFFFFF << 0)

#define SST1_NARROW_BLANK_CONTROL		(0x506C)
#define NARROW_BLANK_EN				(0x01 << 1)
#define VIDEO_FIFO_FLUSH_DISABLE		(0x01 << 0)

#define SST1_LOW_TU_CONTROL			(0x5070)
#define NULL_TU_CONTROL				(0x01 << 1)
#define HALF_FREQUENCY_CONTROL			(0x01 << 0)

#define SST1_ACTIVE_SYMBOL_INTEGER_FEC_OFF	(0x5080)
#define ACTIVE_SYMBOL_INTEGER_FEC_OFF		(0x3F << 0)

#define SST1_ACTIVE_SYMBOL_FRACTION_FEC_OFF	(0x5084)
#define ACTIVE_SYMBOL_FRACTION_FEC_OFF		(0x3FFFFFFF << 0)

#define SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_OFF	(0x5088)
#define ACTIVE_SYMBOL_THRESHOLD_FEC_OFF		(0x0F << 0)

#define SST1_ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_OFF	(0x508C)
#define ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_OFF	(0x01 << 0)

#define SST1_ACTIVE_SYMBOL_INTEGER_FEC_ON	(0x5090)
#define ACTIVE_SYMBOL_INTEGER_FEC_ON		(0x3F << 0)

#define SST1_ACTIVE_SYMBOL_FRACTION_FEC_ON	(0x5094)
#define ACTIVE_SYMBOL_FRACTION_FEC_ON		(0x3FFFFFFF << 0)

#define SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_ON	(0x5098)
#define ACTIVE_SYMBOL_THRESHOLD_FEC_ON		(0x0F << 0)

#define SST1_ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_ON	(0x509C)
#define ACTIVE_SYMBOL_THRESHOLD_SEL_FEC_ON	(0x01 << 0)

#define SST1_FEC_DISABLE_SEND_CONTROL		(0x5100)
#define FEC_DISABLE_SEND_CONTROL		(0x01 << 0)

#define SST1_VIDEO_CONTROL			(0x5400)
#define STRM_VALID_MON				(0x01 << 10)
#define STRM_VALID_FORCE			(0x01 << 9)
#define STRM_VALID_CTRL				(0x01 << 8)
#define DYNAMIC_RANGE_MODE			(0x01 << 7)
#define DYNAMIC_RANGE_VAL(_v_)			((_v_) << 7)
#define BPC					(0x07 << 4)
#define BPC_VAL(_v_)				((_v_) << 4)
#define COLOR_FORMAT				(0x03 << 2)
#define VSYNC_POLARITY				(0x01 << 1)
#define VSYNC_POLARITY_VAL(_v_)			((_v_) << 1)
#define HSYNC_POLARITY				(0x01 << 0)

#define SST1_VIDEO_ENABLE			(0x5404)
#define VIDEO_EN				(0x01 << 0)

#define SST1_VIDEO_MASTER_TIMING_GEN		(0x5408)
#define VIDEO_MASTER_TIME_GEN			(0x01 << 0)

#define SST1_VIDEO_MUTE				(0x540C)
#define VIDEO_MUTE				(0x01 << 0)

#define SST1_VIDEO_FIFO_THRESHOLD_CONTROL	(0x5410)
#define GL_FIFO_TH_CTRL				(0x01 << 5)
#define GL_FIFO_TH_VALUE			(0x1F << 0)

#define SST1_VIDEO_HORIZONTAL_TOTAL_PIXELS	(0x5414)
#define H_TOTAL_MASTER				(0xFFFFFFFF << 0)

#define SST1_VIDEO_VERTICAL_TOTAL_PIXELS	(0x5418)
#define V_TOTAL_MASTER				(0xFFFFFFFF << 0)

#define SST1_VIDEO_HORIZONTAL_FRONT_PORCH	(0x541C)
#define H_F_PORCH_MASTER			(0xFFFFFFFF << 0)

#define SST1_VIDEO_HORIZONTAL_BACK_PORCH	(0x5420)
#define H_B_PORCH_MASTER			(0xFFFFFFFF << 0)

#define SST1_VIDEO_HORIZONTAL_ACTIVE		(0x5424)
#define H_ACTIVE_MASTER				(0xFFFFFFFF << 0)

#define SST1_VIDEO_VERTICAL_FRONT_PORCH		(0x5428)
#define V_F_PORCH_MASTER			(0xFFFFFFFF << 0)

#define SST1_VIDEO_VERTICAL_BACK_PORCH		(0x542C)
#define V_B_PORCH_MASTER			(0xFFFFFFFF << 0)

#define SST1_VIDEO_VERTICAL_ACTIVE		(0x5430)
#define V_ACTIVE_MASTER				(0xFFFFFFFF << 0)

#define SST1_VIDEO_DSC_STREAM_CONTROL_0		(0x5434)
#define DSC_ENABLE				(0x01 << 4)
#define SLICE_COUNT_PER_LINE			(0x07 << 0)

#define SST1_VIDEO_DSC_STREAM_CONTROL_1		(0x5438)
#define CHUNK_SIZE_1				(0xFFFF << 16)
#define CHUNK_SIZE_0				(0xFFFF << 0)

#define SST1_VIDEO_DSC_STREAM_CONTROL_2		(0x543C)
#define CHUNK_SIZE_3				(0xFFFF << 16)
#define CHUNK_SIZE_2				(0xFFFF << 0)

#define SST1_VIDEO_BIST_CONTROL			(0x5450)
#define CTS_BIST_EN				(0x01 << 17)
#define CTS_BIST_TYPE				(0x03 << 15)
#define CTS_BIST_TYPE_VAL(_v_)			((_v_) << 15)
#define BIST_PRBS7_SEED				(0x7F << 8)
#define BIST_USER_DATA_EN			(0x01 << 4)
#define BIST_EN					(0x01 << 3)
#define BIST_WIDTH				(0x01 << 2)
#define BIST_TYPE				(0x03 << 0)

#define SST1_VIDEO_BIST_USER_DATA_R		(0x5454)
#define BIST_USER_DATA_R			(0x3FF << 0)

#define SST1_VIDEO_BIST_USER_DATA_G		(0x5458)
#define BIST_USER_DATA_G			(0x3FF << 0)

#define SST1_VIDEO_BIST_USER_DATA_B		(0x545C)
#define BIST_USER_DATA_B			(0x3FF << 0)

#define SST1_VIDEO_DEBUG_FSM_STATE		(0x5460)
#define DATA_PACK_FSM_STATE			(0x3F << 16)
#define LINE_FSM_STATE				(0x07 << 8)
#define PIXEL_FSM_STATE				(0x07 << 0)

#define SST1_VIDEO_DEBUG_MAPI			(0x5464)
#define MAPI_UNDERFLOW_STATUS			(0x01 << 0)

#define SST1_VIDEO_DEBUG_ACTV_SYM_STEP_CNTL	(0x5468)
#define ACTV_SYM_STEP_CNT_VAL			(0x3FF << 1)
#define ACTV_SYM_STEP_CNT_EN			(0x01 << 0)

#define SST1_VIDEO_DEBUG_HOR_BLANK_AUD_BW_ADJ	(0x546C)
#define HOR_BLANK_AUD_BW_ADJ			(0x01 << 0)

#define SST1_AUDIO_CONTROL			(0x5800)
#define DMA_REQ_GEN_EN				(0x01 << 30)
#define SW_AUD_CODING_TYPE			(0x07 << 27)
#define AUD_DMA_IF_LTNCY_TRG_MODE		(0x01 << 26)
#define AUD_DMA_IF_MODE_CONFIG			(0x01 << 25)
#define AUD_ODD_CHANNEL_DUMMY			(0x01 << 24)
#define AUD_M_VALUE_CMP_SPD_MASTER		(0x07 << 21)
#define DMA_BURST_SEL				(0x07 << 18)
#define AUDIO_BIT_MAPPING_TYPE			(0x03 << 16)
#define PCM_SIZE				(0x03 << 13)
#define AUDIO_CH_STATUS_SAME			(0x01 << 5)
#define AUD_GTC_CHST_EN				(0x01 << 1)

#define SST1_AUDIO_ENABLE			(0x5804)
#define AUDIO_EN				(0x01 << 0)

#define SST1_AUDIO_MASTER_TIMING_GEN		(0x5808)
#define AUDIO_MASTER_TIME_GEN			(0x01 << 0)

#define SST1_AUDIO_DMA_REQUEST_LATENCY_CONFIG	(0x580C)
#define AUD_DMA_ACK_STATUS			(0x01 << 21)
#define AUD_DMA_FORCE_ACK			(0x01 << 20)
#define AUD_DMA_FORCE_ACK_SEL			(0x01 << 19)
#define AUD_DMA_REQ_STATUS			(0x01 << 18)
#define AUD_DMA_FORCE_REQ_VAL			(0x01 << 17)
#define AUD_DMA_FORCE_REQ_SEL			(0x01 << 16)
#define MASTER_DMA_REQ_LTNCY_CONFIG		(0xFF << 0)

#define SST1_AUDIO_MUTE_CONTROL			(0x5810)
#define AUD_MUTE_UNDRUN_EN			(0x01 << 5)
#define AUD_MUTE_OVFLOW_EN			(0x01 << 4)
#define AUD_MUTE_CLKCHG_EN			(0x01 << 1)

#define SST1_AUDIO_MARGIN_CONTROL		(0x5814)
#define FORCE_AUDIO_MARGIN			(0x01 << 16)
#define AUDIO_MARGIN				(0x1FFF << 0)

#define SST1_AUDIO_DATA_WRITE_FIFO		(0x5818)
#define AUDIO_DATA_FIFO				(0xFFFFFFFF << 0)

#define SST1_AUDIO_GTC_CONTROL			(0x5824)
#define AUD_GTC_DELTA				(0xFFFFFFFF << 0)

#define SST1_AUDIO_GTC_VALID_BIT_CONTROL	(0x5828)
#define AUDIO_GTC_VALID_CONTROL			(0x01 << 1)
#define AUDIO_GTC_VALID				(0x01 << 0)

#define SST1_AUDIO_3DLPCM_PACKET_WAIT_TIMER	(0x582C)
#define AUDIO_3D_PKT_WAIT_TIMER			(0x3F << 0)

#define SST1_AUDIO_BIST_CONTROL			(0x5830)
#define SIN_AMPL				(0x0F << 4)
#define AUD_BIST_EN				(0x01 << 0)

#define SST1_AUDIO_BIST_CHANNEL_STATUS_SET0	(0x5834)
#define CHNL_BIT1				(0x03 << 30)
#define CLK_ACCUR				(0x03 << 28)
#define FS_FREQ					(0x0F << 24)
#define CH_NUM					(0x0F << 20)
#define SOURCE_NUM				(0x0F << 16)
#define CAT_CODE				(0xFF << 8)
#define MODE					(0x03 << 6)
#define PCM_MODE				(0x07 << 3)
#define SW_CPRGT				(0x01 << 2)
#define NON_PCM					(0x01 << 1)
#define PROF_APP				(0x01 << 0)

#define SST1_AUDIO_BIST_CHANNEL_STATUS_SET1	(0x5838)
#define CHNL_BIT2				(0x0F << 4)
#define WORD_LENGTH				(0x07 << 1)
#define WORD_MAX				(0x01 << 0)

#define SST1_AUDIO_BUFFER_CONTROL		(0x583C)
#define MASTER_AUDIO_INIT_BUFFER_THRD		(0x7F << 24)
#define MASTER_AUDIO_BUFFER_THRD		(0x3F << 18)
#define MASTER_AUDIO_BUFFER_EMPTY_INT_MASK	(0x01 << 17)
#define MASTER_AUDIO_CHANNEL_COUNT		(0x1F << 12)
#define MASTER_AUDIO_BUFFER_LEVEL		(0x7F << 5)
#define MASTER_AUDIO_BUFFER_LEVEL_BIT_POS	(5)
#define AUD_DMA_NOISE_INT_MASK			(0x01 << 4)
#define AUD_DMA_NOISE_INT			(0x01 << 3)
#define AUD_DMA_NOISE_INT_EN			(0x01 << 2)
#define MASTER_AUDIO_BUFFER_EMPTY_INT		(0x01 << 1)
#define MASTER_AUDIO_BUFFER_EMPTY_INT_EN	(0x01 << 0)

#define SST1_AUDIO_CHANNEL_1_4_REMAP		(0x5840)
#define AUD_CH_04_REMAP				(0x3F << 24)
#define AUD_CH_03_REMAP				(0x3F << 16)
#define AUD_CH_02_REMAP				(0x3F << 8)
#define AUD_CH_01_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_5_8_REMAP		(0x5844)
#define AUD_CH_08_REMAP				(0x3F << 24)
#define AUD_CH_07_REMAP				(0x3F << 16)
#define AUD_CH_06_REMAP				(0x3F << 8)
#define AUD_CH_05_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_9_12_REMAP		(0x5848)
#define AUD_CH_12_REMAP				(0x3F << 24)
#define AUD_CH_11_REMAP				(0x3F << 16)
#define AUD_CH_10_REMAP				(0x3F << 8)
#define AUD_CH_09_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_13_16_REMAP		(0x584C)
#define AUD_CH_16_REMAP				(0x3F << 24)
#define AUD_CH_15_REMAP				(0x3F << 16)
#define AUD_CH_14_REMAP				(0x3F << 8)
#define AUD_CH_13_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_17_20_REMAP		(0x5850)
#define AUD_CH_20_REMAP				(0x3F << 24)
#define AUD_CH_19_REMAP				(0x3F << 16)
#define AUD_CH_18_REMAP				(0x3F << 8)
#define AUD_CH_17_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_21_24_REMAP		(0x5854)
#define AUD_CH_24_REMAP				(0x3F << 24)
#define AUD_CH_23_REMAP				(0x3F << 16)
#define AUD_CH_22_REMAP				(0x3F << 8)
#define AUD_CH_21_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_25_28_REMAP		(0x5858)
#define AUD_CH_28_REMAP				(0x3F << 24)
#define AUD_CH_27_REMAP				(0x3F << 16)
#define AUD_CH_26_REMAP				(0x3F << 8)
#define AUD_CH_25_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_29_32_REMAP		(0x585C)
#define AUD_CH_32_REMAP				(0x3F << 24)
#define AUD_CH_31_REMAP				(0x3F << 16)
#define AUD_CH_30_REMAP				(0x3F << 8)
#define AUD_CH_29_REMAP				(0x3F << 0)

#define SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_0	(0x5860)
#define MASTER_AUD_GP0_STA_3			(0xFF << 24)
#define MASTER_AUD_GP0_STA_2			(0xFF << 16)
#define MASTER_AUD_GP0_STA_1			(0xFF << 8)
#define MASTER_AUD_GP0_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_1_2_STATUS_CTRL_1	(0x5864)
#define MASTER_AUD_GP0_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_3_4_STATUS_CTRL_0	(0x5868)
#define MASTER_AUD_GP1_STA_3			(0xFF << 24)
#define MASTER_AUD_GP1_STA_2			(0xFF << 16)
#define MASTER_AUD_GP1_STA_1			(0xFF << 8)
#define MASTER_AUD_GP1_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_3_4_STATUS_CTRL_1	(0x586C)
#define MASTER_AUD_GP1_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_5_6_STATUS_CTRL_0	(0x5870)
#define MASTER_AUD_GP2_STA_3			(0xFF << 24)
#define MASTER_AUD_GP2_STA_2			(0xFF << 16)
#define MASTER_AUD_GP2_STA_1			(0xFF << 8)
#define MASTER_AUD_GP2_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_5_6_STATUS_CTRL_1	(0x5874)
#define MASTER_AUD_GP2_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_7_8_STATUS_CTRL_0	(0x5878)
#define MASTER_AUD_GP3_STA_3			(0xFF << 24)
#define MASTER_AUD_GP3_STA_2			(0xFF << 16)
#define MASTER_AUD_GP3_STA_1			(0xFF << 8)
#define MASTER_AUD_GP3_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_7_8_STATUS_CTRL_1	(0x587C)
#define MASTER_AUD_GP3_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_9_10_STATUS_CTRL_0	(0x5880)
#define MASTER_AUD_GP4_STA_3			(0xFF << 24)
#define MASTER_AUD_GP4_STA_2			(0xFF << 16)
#define MASTER_AUD_GP4_STA_1			(0xFF << 8)
#define MASTER_AUD_GP4_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_9_10_STATUS_CTRL_1	(0x5884)
#define MASTER_AUD_GP4_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_11_12_STATUS_CTRL_0	(0x5888)
#define MASTER_AUD_GP5_STA_3			(0xFF << 24)
#define MASTER_AUD_GP5_STA_2			(0xFF << 16)
#define MASTER_AUD_GP5_STA_1			(0xFF << 8)
#define MASTER_AUD_GP5_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_11_12_STATUS_CTRL_1	(0x588C)
#define MASTER_AUD_GP5_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_13_14_STATUS_CTRL_0	(0x5890)
#define MASTER_AUD_GP6_STA_3			(0xFF << 24)
#define MASTER_AUD_GP6_STA_2			(0xFF << 16)
#define MASTER_AUD_GP6_STA_1			(0xFF << 8)
#define MASTER_AUD_GP6_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_13_14_STATUS_CTRL_1	(0x5894)
#define MASTER_AUD_GP6_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_15_16_STATUS_CTRL_0	(0x5898)
#define MASTER_AUD_GP7_STA_3			(0xFF << 24)
#define MASTER_AUD_GP7_STA_2			(0xFF << 16)
#define MASTER_AUD_GP7_STA_1			(0xFF << 8)
#define MASTER_AUD_GP7_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_15_16_STATUS_CTRL_1	(0x589C)
#define MASTER_AUD_GP7_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_17_18_STATUS_CTRL_0	(0x58A0)
#define MASTER_AUD_GP8_STA_3			(0xFF << 24)
#define MASTER_AUD_GP8_STA_2			(0xFF << 16)
#define MASTER_AUD_GP8_STA_1			(0xFF << 8)
#define MASTER_AUD_GP8_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_17_18_STATUS_CTRL_1	(0x58A4)
#define MASTER_AUD_GP8_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_19_20_STATUS_CTRL_0	(0x58A8)
#define MASTER_AUD_GP9_STA_3			(0xFF << 24)
#define MASTER_AUD_GP9_STA_2			(0xFF << 16)
#define MASTER_AUD_GP9_STA_1			(0xFF << 8)
#define MASTER_AUD_GP9_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_19_20_STATUS_CTRL_1	(0x58AC)
#define MASTER_AUD_GP9_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_21_22_STATUS_CTRL_0	(0x58B0)
#define MASTER_AUD_GP10_STA_3			(0xFF << 24)
#define MASTER_AUD_GP10_STA_2			(0xFF << 16)
#define MASTER_AUD_GP10_STA_1			(0xFF << 8)
#define MASTER_AUD_GP10_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_21_22_STATUS_CTRL_1	(0x58B4)
#define MASTER_AUD_GP10_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_23_24_STATUS_CTRL_0	(0x58B8)
#define MASTER_AUD_GP11_STA_3			(0xFF << 24)
#define MASTER_AUD_GP11_STA_2			(0xFF << 16)
#define MASTER_AUD_GP11_STA_1			(0xFF << 8)
#define MASTER_AUD_GP11_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_23_24_STATUS_CTRL_1	(0x58BC)
#define MASTER_AUD_GP11_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_25_26_STATUS_CTRL_0	(0x58C0)
#define MASTER_AUD_GP12_STA_3			(0xFF << 24)
#define MASTER_AUD_GP12_STA_2			(0xFF << 16)
#define MASTER_AUD_GP12_STA_1			(0xFF << 8)
#define MASTER_AUD_GP12_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_25_26_STATUS_CTRL_1	(0x58C4)
#define MASTER_AUD_GP12_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_27_28_STATUS_CTRL_0	(0x58C8)
#define MASTER_AUD_GP13_STA_3			(0xFF << 24)
#define MASTER_AUD_GP13_STA_2			(0xFF << 16)
#define MASTER_AUD_GP13_STA_1			(0xFF << 8)
#define MASTER_AUD_GP13_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_27_28_STATUS_CTRL_1	(0x58CC)
#define MASTER_AUD_GP13_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_29_30_STATUS_CTRL_0	(0x58D0)
#define MASTER_AUD_GP14_STA_3			(0xFF << 24)
#define MASTER_AUD_GP14_STA_2			(0xFF << 16)
#define MASTER_AUD_GP14_STA_1			(0xFF << 8)
#define MASTER_AUD_GP14_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_29_30_STATUS_CTRL_1	(0x58D4)
#define MASTER_AUD_GP14_STA_4			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_31_32_STATUS_CTRL_0	(0x58D8)
#define MASTER_AUD_GP15_STA_3			(0xFF << 24)
#define MASTER_AUD_GP15_STA_2			(0xFF << 16)
#define MASTER_AUD_GP15_STA_1			(0xFF << 8)
#define MASTER_AUD_GP15_STA_0			(0xFF << 0)

#define SST1_AUDIO_CHANNEL_31_32_STATUS_CTRL_1	(0x58DC)
#define MASTER_AUD_GP15_STA_4			(0xFF << 0)

#define SST1_STREAM_IF_CRC_CONTROL_1		(0x58E0)
#define IF_CRC_CLEAR				(0x01 << 13)
#define IF_CRC_PASS				(0x01 << 12)
#define IF_CRC_FAIL				(0x01 << 8)
#define IF_CRC_SW_COMPARE			(0x01 << 4)
#define IF_CRC_EN				(0x01 << 0)

#define SST1_STREAM_IF_CRC_CONTROL_2		(0x58E4)
#define IF_CRC_R_REF				(0xFF << 16)
#define IF_CRC_R_RESULT				(0xFF << 0)

#define SST1_STREAM_IF_CRC_CONTROL_3		(0x58E8)
#define IF_CRC_G_REF				(0xFF << 16)
#define IF_CRC_G_RESULT				(0xFF << 0)

#define SST1_STREAM_IF_CRC_CONTROL_4		(0x58EC)
#define IF_CRC_B_REF				(0xFF << 16)
#define IF_CRC_B_RESULT				(0xFF << 0)

#define SST1_AUDIO_DEBUG_MARGIN_CONTROL		(0x5900)
#define AUDIO_DEBUG_MARGIN_EN			(0x01 << 6)
#define AUDIO_DEBUG_MARGIN_VAL			(0x3F << 0)

#define SST1_SDP_SPLITTING_CONTROL		(0x5C00)
#define SDP_SPLITTING_EN			(0x01 << 0)

#define SST1_INFOFRAME_UPDATE_CONTROL		(0x5C04)
#define HDR_INFO_UPDATE				(0x01 << 4)
#define AUDIO_INFO_UPDATE			(0x01 << 3)
#define AVI_INFO_UPDATE				(0x01 << 2)
#define MPEG_INFO_UPDATE			(0x01 << 1)
#define SPD_INFO_UPDATE				(0x01 << 0)

#define SST1_INFOFRAME_SEND_CONTROL		(0x5C08)
#define HDR_INFO_SEND				(0x01 << 4)
#define AUDIO_INFO_SEND				(0x01 << 3)
#define AVI_INFO_SEND				(0x01 << 2)
#define MPEG_INFO_SEND				(0x01 << 1)
#define SPD_INFO_SEND				(0x01 << 0)

#define SST1_INFOFRAME_SDP_VERSION_CONTROL	(0x5C0C)
#define INFOFRAME_SDP_HB3_SEL			(0x01 << 1)
#define INFOFRAME_SDP_VERSION_SEL		(0x01 << 0)

#define SST1_INFOFRAME_SPD_PACKET_TYPE		(0x5C10)
#define SPD_TYPE				(0xFF << 0)

#define SST1_INFOFRAME_SPD_REUSE_PACKET_CONTROL	(0x5C14)
#define SPD_REUSE_EN				(0x01 << 0)

#define SST1_PPS_SDP_CONTROL			(0x5C20)
#define PPS_SDP_CHANGE_STATUS			(0x01 << 2)
#define PPS_SDP_FRAME_SEND_ENABLE		(0x01 << 1)
#define PPS_SDP_UPDATE				(0x01 << 0)

#define SST1_VSC_SDP_CONTROL_1			(0x5C24)
#define VSC_TOTAL_BYTES_IN_SDP			(0xFFF << 8)
#define VSC_CHANGE_STATUS			(0x01 << 5)
#define VSC_FORCE_PACKET_MARGIN			(0x01 << 4)
#define VSC_FIX_PACKET_SEQUENCE			(0x01 << 3)
#define VSC_EXT_VESA_CEA			(0x01 << 2)
#define VSC_SDP_FRAME_SEND_ENABLE		(0x01 << 1)
#define VSC_SDP_UPDATE				(0x01 << 0)

#define SST1_VSC_SDP_CONTROL_2			(0x5C28)
#define VSC_SETUP_TIME				(0xFFF << 20)
#define VSC_PACKET_MARGIN			(0x1FFF << 0)

#define SST1_MST_WAIT_TIMER_CONTROL_1		(0x5C2C)
#define AUDIO_WAIT_TIMER			(0x1FFF << 16)
#define INFOFRAME_WAIT_TIMER			(0x1FFF << 0)

#define SST1_MST_WAIT_TIMER_CONTROL_2		(0x5C30)
#define PPS_WAIT_TIMER				(0x1FFF << 16)
#define VSC_PACKET_WAIT_TIMER			(0x1FFF << 0)

#define SST1_INFOFRAME_AVI_PACKET_DATA_SET0	(0x5C40)
#define AVI_DB4					(0xFF << 24)
#define AVI_DB3					(0xFF << 16)
#define AVI_DB2					(0xFF << 8)
#define AVI_DB1					(0xFF << 0)

#define SST1_INFOFRAME_AVI_PACKET_DATA_SET1	(0x5C44)
#define AVI_DB8					(0xFF << 24)
#define AVI_DB7					(0xFF << 16)
#define AVI_DB6					(0xFF << 8)
#define AVI_DB5					(0xFF << 0)

#define SST1_INFOFRAME_AVI_PACKET_DATA_SET2	(0x5C48)
#define AVI_DB12				(0xFF << 24)
#define AVI_DB11				(0xFF << 16)
#define AVI_DB10				(0xFF << 8)
#define AVI_DB9					(0xFF << 0)

#define SST1_INFOFRAME_AVI_PACKET_DATA_SET3	(0x5C4C)
#define AVI_DB13				(0xFF << 0)

#define SST1_INFOFRAME_AUDIO_PACKET_DATA_SET0	(0x5C50)
#define AUDIO_DB4				(0xFF << 24)
#define AUDIO_DB3				(0xFF << 16)
#define AUDIO_DB2				(0xFF << 8)
#define AUDIO_DB1				(0xFF << 0)

#define SST1_INFOFRAME_AUDIO_PACKET_DATA_SET1	(0x5C54)
#define AUDIO_DB8				(0xFF << 24)
#define AUDIO_DB7				(0xFF << 16)
#define AUDIO_DB6				(0xFF << 8)
#define AUDIO_DB5				(0xFF << 0)

#define SST1_INFOFRAME_AUDIO_PACKET_DATA_SET2	(0x5C58)
#define AVI_DB10				(0xFF << 8)
#define AVI_DB9					(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET0	(0x5C60)
#define SPD_DB4					(0xFF << 24)
#define SPD_DB3					(0xFF << 16)
#define SPD_DB2					(0xFF << 8)
#define SPD_DB1					(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET1	(0x5C64)
#define SPD_DB8					(0xFF << 24)
#define SPD_DB7					(0xFF << 16)
#define SPD_DB6					(0xFF << 8)
#define SPD_DB5					(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET2	(0x5C68)
#define SPD_DB12				(0xFF << 24)
#define SPD_DB11				(0xFF << 16)
#define SPD_DB10				(0xFF << 8)
#define SPD_DB9					(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET3	(0x5C6C)
#define SPD_DB16				(0xFF << 24)
#define SPD_DB15				(0xFF << 16)
#define SPD_DB14				(0xFF << 8)
#define SPD_DB13				(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET4	(0x5C70)
#define SPD_DB20				(0xFF << 24)
#define SPD_DB19				(0xFF << 16)
#define SPD_DB18				(0xFF << 8)
#define SPD_DB17				(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET5	(0x5C74)
#define SPD_DB24				(0xFF << 24)
#define SPD_DB23				(0xFF << 16)
#define SPD_DB22				(0xFF << 8)
#define SPD_DB21				(0xFF << 0)

#define SST1_INFOFRAME_SPD_PACKET_DATA_SET6	(0x5C78)
#define SPD_DB25				(0xFF << 0)

#define SST1_INFOFRAME_MPEG_PACKET_DATA_SET0	(0x5C80)
#define MPEG_DB4				(0xFF << 24)
#define MPEG_DB3				(0xFF << 16)
#define MPEG_DB2				(0xFF << 8)
#define MPEG_DB1				(0xFF << 0)

#define SST1_INFOFRAME_MPEG_PACKET_DATA_SET1	(0x5C84)
#define MPEG_DB8				(0xFF << 24)
#define MPEG_DB7				(0xFF << 16)
#define MPEG_DB6				(0xFF << 8)
#define MPEG_DB5				(0xFF << 0)

#define SST1_INFOFRAME_MPEG_PACKET_DATA_SET2	(0x5C88)
#define MPEG_DB10				(0xFF << 8)
#define MPEG_DB9				(0xFF << 0)

#define SST1_INFOFRAME_SPD_REUSE_PACKET_HEADER_SET	(0x5C90)
#define SPD_REUSE_HB3				(0xFF << 24)
#define SPD_REUSE_HB2				(0xFF << 16)
#define SPD_REUSE_HB1				(0xFF << 8)
#define SPD_REUSE_HB0				(0xFF << 0)

#define SST1_INFOFRAME_SPD_REUSE_PACKET_PARITY_SET	(0x5C94)
#define SPD_REUSE_PB3				(0xFF << 24)
#define SPD_REUSE_PB2				(0xFF << 16)
#define SPD_REUSE_PB1				(0xFF << 8)
#define SPD_REUSE_PB0				(0xFF << 0)

#define SST1_HDR_PACKET_DATA_SET_0		(0x5CA0)
#define HDR_INFOFRAME_DATA_0			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_1		(0x5CA4)
#define HDR_INFOFRAME_DATA_1			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_2		(0x5CA8)
#define HDR_INFOFRAME_DATA_2			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_3		(0x5CAC)
#define HDR_INFOFRAME_DATA_3			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_4		(0x5CB0)
#define HDR_INFOFRAME_DATA_4			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_5		(0x5CB4)
#define HDR_INFOFRAME_DATA_5			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_6		(0x5CB8)
#define HDR_INFOFRAME_DATA_6			(0xFFFFFFFF << 0)

#define SST1_HDR_PACKET_DATA_SET_7		(0x5CBC)
#define HDR_INFOFRAME_DATA_7			(0xFFFFFFFF << 0)

/* PPS03[31:24], PPS02[23:16], PPS01[15:8], PPS00[7:0] */
#define PPS0xN(val)				(val)
#define PPS1xN(val)				(val << 8)
#define PPS2xN(val)				(val << 16)
#define PPS3xN(val)				(val << 24)

#define SST1_PPS_SDP_PAYLOAD_0			(0x5D00)
#define PPS_SDP_DATA_0				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_1			(0x5D04)
#define PPS_SDP_DATA_1				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_2			(0x5D08)
#define PPS_SDP_DATA_2				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_3			(0x5D0C)
#define PPS_SDP_DATA_3				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_4			(0x5D10)
#define PPS_SDP_DATA_4				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_5			(0x5D14)
#define PPS_SDP_DATA_5				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_6			(0x5D18)
#define PPS_SDP_DATA_6				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_7			(0x5D1C)
#define PPS_SDP_DATA_7				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_8			(0x5D20)
#define PPS_SDP_DATA_8				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_9			(0x5D24)
#define PPS_SDP_DATA_9				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_10			(0x5D28)
#define PPS_SDP_DATA_10				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_11			(0x5D2C)
#define PPS_SDP_DATA_11				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_12			(0x5D30)
#define PPS_SDP_DATA_12				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_13			(0x5D34)
#define PPS_SDP_DATA_13				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_14			(0x5D38)
#define PPS_SDP_DATA_14				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_15			(0x5D3C)
#define PPS_SDP_DATA_15				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_16			(0x5D40)
#define PPS_SDP_DATA_16				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_17			(0x5D44)
#define PPS_SDP_DATA_17				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_18			(0x5D48)
#define PPS_SDP_DATA_18				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_19			(0x5D4C)
#define PPS_SDP_DATA_19				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_20			(0x5D50)
#define PPS_SDP_DATA_20				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_21			(0x5D54)
#define PPS_SDP_DATA_21				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_22			(0x5D58)
#define PPS_SDP_DATA_22				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_23			(0x5D5C)
#define PPS_SDP_DATA_23				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_24			(0x5D60)
#define PPS_SDP_DATA_24				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_25			(0x5D64)
#define PPS_SDP_DATA_25				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_26			(0x5D68)
#define PPS_SDP_DATA_26				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_27			(0x5D6C)
#define PPS_SDP_DATA_27				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_28			(0x5D70)
#define PPS_SDP_DATA_28				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_29			(0x5D74)
#define PPS_SDP_DATA_29				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_30			(0x5D78)
#define PPS_SDP_DATA_30				(0xFFFFFFFF << 0)

#define SST1_PPS_SDP_PAYLOAD_31			(0x5D7C)
#define PPS_SDP_DATA_31				(0xFFFFFFFF << 0)

#define SST1_VSC_SDP_DATA_PAYLOAD_FIFO		(0x5D80)
#define VSC_SDP_DATA_PAYLOAD_FIFO		(0xFFFFFFFF << 0)

/* PHY register */
#define DEFAULT_SFR_CNT				54

#define CMN_REG0009				(0x0024)
#define ANA_AUX_TX_LVL_CTRL			(0x0F << 3)
#define ANA_AUX_TX_LVL_CTRL_VAL(_v_)		((_v_) << 3)

#define CMN_REG00A2				(0x0288)
#define LANE_MUX_SEL_DP_LN3			(0x01 << 7)
#define LANE_MUX_SEL_DP_LN2			(0x01 << 6)
#define LANE_MUX_SEL_DP_LN1			(0x01 << 5)
#define LANE_MUX_SEL_DP_LN0			(0x01 << 4)
#define DP_LANE_EN_LN3				(0x01 << 3)
#define DP_LANE_EN_LN2				(0x01 << 2)
#define DP_LANE_EN_LN1				(0x01 << 1)
#define DP_LANE_EN_LN0				(0x01 << 0)

#define CMN_REG00A3				(0x028C)
#define DP_TX_LINK_BW				(0x03 << 5)
#define DP_TX_LINK_BW_VAL(_v_)			((_v_) << 5)
#define OVRD_RX_CDR_DATA_MODE_EXIT		(0x01 << 4)

#define CMN_REG00B4				(0x02D0)
#define ROPLL_SSC_EN				(0x01 << 1)

#define CMN_REG00E3				(0x038C)
#define DP_INIT_RSTN				(0x01 << 3)
#define DP_CMN_RSTN				(0x01 << 2)
#define DP_CMN_RSTN_VAL(_v_)			((_v_) << 2)
#define CDR_WATCHDOG_EN				(0x01 << 1)

#define TRSV_REG0204				(0x0810)
#define LN0_TX_DRV_LVL_CTRL			(0x1F << 0)

#define TRSV_REG0215				(0x0854)
#define LN0_ANA_TX_SER_TXCLK_INV	(0x01 << 1)

#define TRSV_REG0400				(0x1000)
#define OVRD_LN1_TX_DRV_BECON_LFPS_OUT_EN	(0x01 << 5)

#define TRSV_REG0404				(0x1010)
#define LN1_TX_DRV_LVL_CTRL			(0x1F << 0)

#define TRSV_REG0415				(0x1054)
#define LN1_ANA_TX_SER_TXCLK_INV	(0x01 << 1)

#define TRSV_REG0604				(0x1810)
#define LN2_TX_DRV_LVL_CTRL			(0x1F << 0)

#define TRSV_REG0615				(0x1854)
#define LN2_ANA_TX_SER_TXCLK_INV	(0x01 << 1)

#define TRSV_REG0800				(0x2000)
#define OVRD_LN3_TX_DRV_BECON_LFPS_OUT_EN	(0x01 << 5)

#define TRSV_REG0804				(0x2010)
#define LN3_TX_DRV_LVL_CTRL			(0x1F << 0)

#define TRSV_REG0815				(0x2054)
#define LN3_ANA_TX_SER_TXCLK_INV	(0x01 << 1)

#define MAX_SLICE_COUNT 4
#define SST_REG(reg) (reg + (0x1000 * sst_id))

DEFINE_CAL_REGS_FUNCS(dp_link, MAX_DP_CNT);
DEFINE_CAL_REGS_FUNCS(dp_phy, MAX_DP_CNT);

const u32 phy_default_value[DEFAULT_SFR_CNT][2] = {
	{ 0x0830, 0x07 }, { 0x085C, 0x80 }, { 0x1030, 0x07 }, { 0x105C, 0x80 },
	{ 0x1830, 0x07 }, { 0x185C, 0x80 }, { 0x2030, 0x07 }, { 0x205C, 0x80 },
	{ 0x0228, 0x38 }, { 0x0104, 0x44 }, { 0x0248, 0x44 }, { 0x038C, 0x02 },
	{ 0x0878, 0x04 }, { 0x1878, 0x04 }, { 0x0898, 0x77 }, { 0x1898, 0x77 },
	{ 0x0054, 0x01 }, { 0x00E0, 0x38 }, { 0x0060, 0x24 }, { 0x0064, 0x77 },
	{ 0x0070, 0x76 }, { 0x0234, 0xE8 }, { 0x0AF4, 0x15 }, { 0x1AF4, 0x15 },
	{ 0x081C, 0xE5 }, { 0x181C, 0xE5 }, { 0x091C, 0x1F }, { 0x191C, 0x1F },
	{ 0x0928, 0x7C }, { 0x1928, 0x7C }, { 0x0994, 0x14 }, { 0x1994, 0x14 },
	{ 0x099C, 0x48 }, { 0x199C, 0x48 }, { 0x09A4, 0x0B }, { 0x09A8, 0x62 },
	{ 0x19A4, 0x0B }, { 0x19A8, 0x62 }, { 0x09B8, 0x3E }, { 0x19B8, 0x3E },
	{ 0x09E4, 0x05 }, { 0x09E8, 0x02 }, { 0x09EC, 0x02 }, { 0x19E4, 0x05 },
	{ 0x19E8, 0x02 }, { 0x19EC, 0x02 }, { 0x0A34, 0x1C }, { 0x1A34, 0x1C },
	{ 0x0A98, 0x2F }, { 0x1A98, 0x2F }, { 0x0C30, 0x0C }, { 0x0C48, 0x08 },
	{ 0x1C30, 0x0C }, { 0x1C48, 0x08 },
};

const u32 phy_tune_parameters[4][4][4] = {
	/* {amp, post, pre, idrv} */
	{
		/* Swing Level_0 */
		{ 0x21, 0x10, 0x42, 0xE5 }, /* Pre-emphasis Level_0 */
		{ 0x25, 0x14, 0x42, 0xE5 }, /* Pre-emphasis Level_1 */
		{ 0x26, 0x17, 0x43, 0xE5 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x1C, 0x43, 0xE7 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_1 */
		{ 0x26, 0x10, 0x42, 0xE7 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x15, 0x42, 0xE7 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x18, 0x43, 0xE7 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x18, 0x43, 0xE7 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_2 */
		{ 0x2A, 0x10, 0x42, 0xE7 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x15, 0x43, 0xE7 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x15, 0x43, 0xE7 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x15, 0x43, 0xE7 }, /* Pre-emphasis Level_3 */
	},
	{
		/* Swing Level_3 */
		{ 0x2B, 0x10, 0x43, 0xE7 }, /* Pre-emphasis Level_0 */
		{ 0x2B, 0x10, 0x43, 0xE7 }, /* Pre-emphasis Level_1 */
		{ 0x2B, 0x10, 0x43, 0xE7 }, /* Pre-emphasis Level_2 */
		{ 0x2B, 0x10, 0x43, 0xE7 }, /* Pre-emphasis Level_3 */
	},
};

const struct dp_support_video support_videos[] = {
	/*               width,     hsync,    height,   vsync, vbackporch,
	 *                   hfrontporch,hbackporch, vfrontporch, pixel_clk, v_sync_pol,    h_sync_pol,    vic, name
	 */
	{ V640X480P60, 640, 16, 96, 48, 480, 10, 2, 33, 25200000, SYNC_NEGATIVE,
	  SYNC_NEGATIVE, 1, "V640X480P60" },
	{ V640X480P30, 640, 16, 96, 48, 480, 10, 2, 33, 12600000, SYNC_NEGATIVE,
	  SYNC_NEGATIVE, 1, "V640X480P30" },
	{ V720X480P60, 720, 16, 62, 60, 480, 9, 6, 30, 27027000, SYNC_NEGATIVE,
	  SYNC_NEGATIVE, 2, "V720X480P60" },
	{ V720X576P50, 720, 12, 64, 68, 576, 5, 5, 39, 27000000, SYNC_NEGATIVE,
	  SYNC_NEGATIVE, 17, "V720X576P50" },
	{ V1280X800P60RB, 1280, 48, 32, 80, 800, 3, 6, 14, 71000000,
	  SYNC_NEGATIVE, SYNC_POSITIVE, 0, "V1280X800P60RB" },
	{ V1280X720P60, 1280, 110, 40, 220, 720, 5, 5, 20, 74250000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 4, "V1280X720P60" },
	{ V1366X768P60, 1366, 70, 143, 213, 768, 3, 3, 24, 85500000,
	  SYNC_POSITIVE, SYNC_NEGATIVE, 0, "V1366X768P60" },
	{ V1280X1024P60, 1280, 48, 112, 248, 1024, 1, 3, 38, 108000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 0, "V1280X1024P60" },
	{ V1920X1080P24, 1920, 638, 44, 148, 1080, 4, 5, 36, 74250000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 32, "V1920X1080P24" },
	{ V1920X1080P25, 1920, 528, 44, 148, 1080, 4, 5, 36, 74250000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 33, "V1920X1080P25" },
	{ V1920X1080P30, 1920, 88, 44, 148, 1080, 4, 5, 36, 74250000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 34, "V1920X1080P30" },
	{ V1600X900P60RB, 1600, 24, 80, 96, 900, 1, 3, 96, 108000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 0, "V1600X900P60RB" },
	{ V1920X1080P60, 1920, 88, 44, 148, 1080, 4, 5, 36, 148500000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 16, "V1920X1080P60" },
	{ V1920X1200P60, 1920, 48, 32, 80, 1200, 3, 6, 26, 154000000,
	  SYNC_NEGATIVE, SYNC_POSITIVE, 16, "V1920X1200P60" },
	{ V1920X1200P60P, 1920, 48, 32, 80, 1200, 3, 6, 26, 154128000,
	  SYNC_NEGATIVE, SYNC_POSITIVE, 16, "V1920X1200P60P" },
	{ V1920X1200P30, 1920, 48, 32, 80, 1200, 3, 6, 26, 77000000,
	  SYNC_NEGATIVE, SYNC_POSITIVE, 16, "V1920X1200P30" },
	{ V1920X1200P30P, 1920, 48, 32, 80, 1200, 3, 6, 26, 77064000,
	  SYNC_NEGATIVE, SYNC_POSITIVE, 16, "V1920X1200P30P" },
	{ V3840X2160P24, 3840, 1276, 88, 296, 2160, 8, 10, 72, 297000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 93, "V3840X2160P24" },
	{ V3840X2160P25, 3840, 1056, 88, 296, 2160, 8, 10, 72, 297000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 94, "V3840X2160P25" },
	{ V3840X2160P30, 3840, 176, 88, 296, 2160, 8, 10, 72, 297000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 95, "V3840X2160P30" },
	{ V4096X2160P24, 4096, 1020, 88, 296, 2160, 8, 10, 72, 297000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 98, "V4096X2160P24" },
	{ V4096X2160P25, 4096, 968, 88, 128, 2160, 8, 10, 72, 297000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 99, "V4096X2160P25" },
	{ V4096X2160P30, 4096, 88, 88, 128, 2160, 8, 10, 72, 297000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 100, "V4096X2160P30" },
	{ V3840X2160P50, 3840, 1056, 88, 296, 2160, 8, 10, 72, 594000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 96, "V3840X2160P50" },
	{ V3840X2160P60, 3840, 176, 88, 296, 2160, 8, 10, 72, 594000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 97, "V3840X2160P60" },
	{ V4096X2160P50, 4096, 968, 88, 128, 2160, 8, 10, 72, 594000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 101, "V4096X2160P50" },
	{ V4096X2160P60, 4096, 88, 88, 128, 2160, 8, 10, 72, 594000000,
	  SYNC_POSITIVE, SYNC_POSITIVE, 102, "V4096X2160P60" },
	/* Used for ICAS */
	{ V1560X700P60, 1560, 30, 40, 70, 700, 9, 2, 20, 74562000,
	  SYNC_NEGATIVE, SYNC_NEGATIVE, 20, "V1560X700P60" },
	{ V800X400P60, 800, 112, 128, 136, 400, 10, 10, 25, 31399200,
	  SYNC_NEGATIVE, SYNC_NEGATIVE, 20, "V800X400P60" },
	{ V1120X780P60, 1120, 125, 1, 16, 780, 77, 1, 10, 65600000,
	  SYNC_NEGATIVE, SYNC_NEGATIVE, 20, "V1120X780P60" },
	/* Used for AR HUD */
	{ V1440X720P60, 1440, 32, 8, 32, 720, 16, 22, 2, 66000000,
	  SYNC_POSITIVE, SYNC_NEGATIVE, 20, "V1440X720P60" },
	{ V1440X2560P60, 1440, 96, 64, 160, 2560, 27, 24, 6, 276300000,
	  SYNC_POSITIVE, SYNC_NEGATIVE, 20, "V1440X2560P60" },
	{ V1800X900P60, 1800, 166, 101, 67, 900, 12, 10, 10, 119400000,
	  SYNC_POSITIVE, SYNC_NEGATIVE, 20, "V1800X900P60" },
};

int dp_regs_desc_init(u32 dp_id, struct dp_regs *regs)
{
	cal_regs_desc_check(dp_id, MAX_DP_CNT, __func__);

	cal_regs_desc_save(dp_link, regs->link_addr, __func__, REGS_DP, dp_id);
	cal_regs_desc_save(dp_phy, regs->phy_addr, __func__, REGS_DPPHY, dp_id);

	return 0;
}
void dp_reg_sw_reset(u32 id)
{
	u32 cnt = 10;
	u32 state;

	dp_link_write_mask(id, SYSTEM_SW_RESET_CONTROL, ~0, SW_RESET);

	do {
		state = dp_link_read(id, SYSTEM_SW_RESET_CONTROL) & SW_RESET;
		cnt--;
		udelay(1);
	} while (state && cnt);

	if (!cnt)
		cal_log_err(id, "%s is timeout.\n", __func__);
}
void dp_reg_phy_reset(u32 id, u32 en)
{
	if (en)
		dp_phy_write_mask(id, CMN_REG00E3, DP_CMN_RSTN_VAL(0),
				  DP_CMN_RSTN);
	else {
		dp_phy_write(id, CMN_REG00E3, DP_INIT_RSTN | CDR_WATCHDOG_EN);

		udelay(1);

		dp_phy_write(id, CMN_REG00E3,
			     DP_INIT_RSTN | DP_CMN_RSTN | CDR_WATCHDOG_EN);
	}
}

void dp_reg_phy_init_setting(u32 id)
{
	int i;

	for (i = 0; i < DEFAULT_SFR_CNT; i++)
		dp_phy_write(id, phy_default_value[i][0],
			     phy_default_value[i][1]);

	dp_phy_write_mask(id, CMN_REG0009, DP_CMN_RSTN_VAL(0xD),
			  ANA_AUX_TX_LVL_CTRL);
}

u32 dp_reg_phy_get_link_bw(u32 id)
{
	u32 val = 0;

	val = dp_phy_read_mask(id, CMN_REG00A3, DP_TX_LINK_BW) >> 5;

	switch (val) {
	case 0x03:
		val = LINK_RATE_8_1Gbps;
		break;
	case 0x02:
	default:
		val = LINK_RATE_5_4Gbps;
		break;
	case 0x01:
		val = LINK_RATE_2_7Gbps;
		break;
	case 0x00:
		val = LINK_RATE_1_62Gbps;
		break;
	}

	return val;
}

void dp_reg_phy_set_link_bw(u32 id, u8 link_rate)
{
	u32 val = 0;

	switch (link_rate) {
	case LINK_RATE_8_1Gbps:
		val = 0x03;
		break;
	case LINK_RATE_5_4Gbps:
		val = 0x02;
		break;
	case LINK_RATE_2_7Gbps:
		val = 0x01;
		break;
	case LINK_RATE_1_62Gbps:
		val = 0x00;
		break;
	default:
		val = 0x02;
	}

	dp_phy_write_mask(id, CMN_REG00A3, DP_TX_LINK_BW_VAL(val),
			  DP_TX_LINK_BW);
}

void dp_reg_phy_mode_setting(u32 id)
{
	u32 lane_config_val = 0;
	u32 lane_en_val = 0;

	lane_config_val = LANE_MUX_SEL_DP_LN3 | LANE_MUX_SEL_DP_LN2 |
			  LANE_MUX_SEL_DP_LN1 | LANE_MUX_SEL_DP_LN0;
	lane_en_val = DP_LANE_EN_LN3 | DP_LANE_EN_LN2 | DP_LANE_EN_LN1 |
		      DP_LANE_EN_LN0;

	if (dp_reg_phy_get_link_bw(id) < LINK_RATE_5_4Gbps) {
		dp_phy_write_mask(id, TRSV_REG0215, ~0,
				  LN0_ANA_TX_SER_TXCLK_INV);
		dp_phy_write_mask(id, TRSV_REG0415, ~0,
				  LN1_ANA_TX_SER_TXCLK_INV);
		dp_phy_write_mask(id, TRSV_REG0615, ~0,
				  LN2_ANA_TX_SER_TXCLK_INV);
		dp_phy_write_mask(id, TRSV_REG0815, ~0,
				  LN3_ANA_TX_SER_TXCLK_INV);
	} else {
		dp_phy_write_mask(id, TRSV_REG0215, 0,
				  LN0_ANA_TX_SER_TXCLK_INV);
		dp_phy_write_mask(id, TRSV_REG0415, 0,
				  LN1_ANA_TX_SER_TXCLK_INV);
		dp_phy_write_mask(id, TRSV_REG0615, 0,
				  LN2_ANA_TX_SER_TXCLK_INV);
		dp_phy_write_mask(id, TRSV_REG0815, 0,
				  LN3_ANA_TX_SER_TXCLK_INV);
	}

	dp_phy_write(id, CMN_REG00A2, lane_config_val | lane_en_val);
}

static void dp_reg_phy_ssc_enable(u32 id, u32 en)
{
	u32 val;

	val = en ? ~0 : 0;
	dp_phy_write_mask(id, CMN_REG00B4, val, ROPLL_SSC_EN);
}

void dp_reg_wait_phy_pll_lock(u32 id)
{
	u32 cnt = 165; /* wait for 150us + 10% margin */
	u32 state;

	do {
		state = dp_link_read(id, SYSTEM_PLL_LOCK_CONTROL) &
			PLL_LOCK_STATUS;
		cnt--;
		udelay(1);
	} while (!state && cnt);

	if (!cnt)
		cal_log_err(id, "%s is timeout.\n", __func__);
}

void dp_reg_set_lane_count(u32 id, u8 lane_cnt)
{
	dp_link_write(id, SYSTEM_MAIN_LINK_LANE_COUNT, lane_cnt);
}

u32 dp_reg_get_lane_count(u32 id)
{
	return dp_link_read(id, SYSTEM_MAIN_LINK_LANE_COUNT);
}

void dp_reg_set_enhanced_mode(u32 id, u32 en)
{
	int i = 0;
	u32 val;

	val = en ? ~0 : 0;

	for (i = 0; i < MAX_SST_CNT; i++)
		dp_link_write_mask(id, SST1_MAIN_CONTROL + 0x1000 * i, val,
				   ENHANCED_MODE);
}

void dp_reg_set_training_pattern(u32 id, dp_training_pattern pattern)
{
	dp_link_write_mask(id, PCS_TEST_PATTERN_CONTROL, 0,
			   LINK_QUALITY_PATTERN_SET);
	dp_link_write_mask(id, PCS_CONTROL,
			   LINK_TRAINING_PATTERN_SET_VAL(pattern),
			   LINK_TRAINING_PATTERN_SET);
}

void dp_reg_scrambling_enable(u32 id, bool enable)
{
	dp_link_write_mask(id, PCS_CONTROL, !enable, SCRAMBLE_BYPASS);
}

void dp_reg_set_phy_tune(u32 id, u32 phy_lane_num, u32 amp_lvl, u32 pre_emp_lvl)
{
	u32 addr = 0;
	u32 val = 0;
	int i;

	switch (phy_lane_num) {
	case 0:
		addr = TRSV_REG0204;
		break;
	case 1:
		addr = TRSV_REG0404;
		break;
	case 2:
		addr = TRSV_REG0604;
		break;
	case 3:
		addr = TRSV_REG0804;
		break;
	default:
		addr = TRSV_REG0204;
		break;
	}

	for (i = AMP; i <= IDRV; i++) {
		val = phy_tune_parameters[amp_lvl][pre_emp_lvl][i];
		dp_phy_write(id, addr + i * 4, val);
	}
}

static void dp_reg_set_phy_voltage_and_pre_emphasis(u32 id, u8 *voltage,
						    u8 *pre_emphasis)
{
	dp_reg_set_phy_tune(id, 0, voltage[0], pre_emphasis[0]);
	dp_reg_set_phy_tune(id, 1, voltage[1], pre_emphasis[1]);
	dp_reg_set_phy_tune(id, 2, voltage[2], pre_emphasis[2]);
	dp_reg_set_phy_tune(id, 3, voltage[3], pre_emphasis[3]);
}

void dp_reg_set_voltage_and_pre_emphasis(u32 id, u8 *voltage, u8 *pre_emphasis)
{
	dp_reg_set_phy_voltage_and_pre_emphasis(id, voltage, pre_emphasis);
}

static void dp_reg_common_function_enable(u32 id, u32 en)
{
	u32 val;

	val = en ? ~0 : 0;

	dp_link_write_mask(id, SYSTEM_COMMON_FUNCTION_ENABLE, val, PCS_FUNC_EN);
	dp_link_write_mask(id, SYSTEM_COMMON_FUNCTION_ENABLE, val, AUX_FUNC_EN);
}

static void dp_reg_sst_function_enable(u32 id, u32 sst_id, u32 en)
{
	u32 reg_offset = 0;

	switch (sst_id) {
	case 0:
		reg_offset = SYSTEM_SST1_FUNCTION_ENABLE;
		break;
	case 1:
		reg_offset = SYSTEM_SST2_FUNCTION_ENABLE;
		break;
	case 2:
		reg_offset = SYSTEM_SST3_FUNCTION_ENABLE;
		break;
	case 3:
		reg_offset = SYSTEM_SST4_FUNCTION_ENABLE;
		break;
	default:
		reg_offset = SYSTEM_SST1_FUNCTION_ENABLE;
	}

	dp_link_write_mask(id, reg_offset, en, SST1_VIDEO_FUNC_EN);
}

static void dp_reg_set_sst_stream_enable(u32 id, u32 sst_id, u32 en)
{
	dp_link_write_mask(id, MST_STREAM_1_ENABLE + 0x10 * sst_id, en,
			   STRM_1_EN);
}

static void
dp_reg_set_common_interrupt_mask(u32 id, enum dp_interrupt_mask param, u8 set)
{
	u32 val = set ? ~0 : 0;

	switch (param) {
	case HOTPLUG_CHG_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HPD_CHG_MASK);
		break;
	case HPD_LOST_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HPD_LOST_MASK);
		break;
	case PLUG_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HPD_PLUG_MASK);
		break;
	case HPD_IRQ_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HPD_IRQ_MASK);
		break;
	case RPLY_RECEIV_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   AUX_REPLY_RECEIVED_MASK);
		break;
	case AUX_ERR_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   AUX_ERR_MASK);
		break;
	case HDCP_LINK_CHECK_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HDCP_R0_CHECK_FLAG_MASK);
		break;
	case HDCP_LINK_FAIL_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HDCP_LINK_CHK_FAIL_MASK);
		break;
	case HDCP_R0_READY_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   HDCP_R0_CHECK_FLAG_MASK);
		break;
	case PLL_LOCK_CHG_INT_MASK:
		dp_link_write_mask(id, SYSTEM_IRQ_COMMON_STATUS_MASK, val,
				   PLL_LOCK_CHG_MASK);
		break;
	case ALL_INT_MASK:
		dp_link_write(id, SYSTEM_IRQ_COMMON_STATUS_MASK, 0xFF);
		break;
	default:
		break;
	}
}

static void dp_reg_set_sst_interrupt_mask(u32 id, u32 sst_id,
					  enum dp_interrupt_mask param, u8 set)
{
	u32 val = set ? ~0 : 0;

	switch (param) {
	case VIDEO_FIFO_UNDER_FLOW_MASK:
		dp_link_write_mask(id,
				   SST1_INTERRUPT_MASK_SET0 + 0x1000 * sst_id,
				   val, MAPI_FIFO_UNDER_FLOW_MASK);
		break;
	case VSYNC_DET_INT_MASK:
		dp_link_write_mask(id,
				   SST1_INTERRUPT_MASK_SET0 + 0x1000 * sst_id,
				   val, VSYNC_DET_MASK);
		break;
	case ALL_INT_MASK:
		dp_link_write(id, SST1_INTERRUPT_MASK_SET0 + 0x1000 * sst_id,
			      0xFF);
		dp_link_write(id, SST1_INTERRUPT_STATUS_SET1 + 0x1000 * sst_id,
			      0xFF);
		break;
	default:
		break;
	}
}

void dp_reg_set_hpd_interrupt(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_link_write(id, SYSTEM_IRQ_COMMON_STATUS, ~0);
	/*
	 * TODO : Check the PLUG_IN Interrupt of DP_LINK
	 * The role of PLUG_IN is by HPD_GPIO.
	 * So, Enabling DP's PLUG_IN IRQ is redundant.
	 * The Problem occurred because the PLUG_IN IRQ of DP_LINK
	 * was working in the reset sequence when boot.
	 * (The DP suspend and Link_training are overlapped.)
	 */

	dp_reg_set_common_interrupt_mask(id, HPD_IRQ_INT_MASK, val);
	/* dp_reg_set_common_interrupt_mask(id, HOTPLUG_CHG_INT_MASK, val); */
	dp_reg_set_common_interrupt_mask(id, HPD_LOST_INT_MASK, val);
	/* dp_reg_set_common_interrupt_mask(id, PLUG_INT_MASK, val); */
}

void dp_reg_set_plug_interrupt(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_link_write(id, SYSTEM_IRQ_COMMON_STATUS, ~0);
	/*
	 * TODO : Check the PLUG_IN Interrupt of DP_LINK
	 * DP plug_in/Out is abnormal work after S2R without this code.
	 */

	dp_reg_set_common_interrupt_mask(id, HPD_IRQ_INT_MASK, val);
	dp_reg_set_common_interrupt_mask(id, HOTPLUG_CHG_INT_MASK, val);
	dp_reg_set_common_interrupt_mask(id, HPD_LOST_INT_MASK, val);
	dp_reg_set_common_interrupt_mask(id, PLUG_INT_MASK, val);
}

#define FIN_MHZ 24
#define DEGLITCH_UDELAY 10

u32 dp_reg_get_hpd_status(u32 id)
{
	u32 val, cnt = 2; /* minimum retry count of double */

	/* try to wait until HPD_DEGLITCH_COUNT which is counted at 24 MHz */
	val = dp_link_read_mask(id, SYSTEM_HPD_CONTROL, HPD_DEGLITCH_COUNT) >>
	      16;
	cnt *= (val / (FIN_MHZ * DEGLITCH_UDELAY));

	do {
		val = dp_link_read_mask(id, SYSTEM_HPD_CONTROL, HPD_STATUS);
		if (val)
			break;

		udelay(DEGLITCH_UDELAY);
		cnt--;
	} while (!val && cnt);

	if (!cnt && !val)
		cal_log_err(id, "HPD_STATUS waiting timeout(%#x)\n",
			    dp_link_read(id, SYSTEM_HPD_CONTROL));

	return val;
}

u32 dp_reg_get_int_and_clear(u32 id, dp_irq_reg_type_t irq_reg)
{
	u32 val = 0;
	u32 irq_status_reg = 0;

	switch (irq_reg) {
	case DP_IRQ_REG_SST1_SET0:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET0;
		break;
	case DP_IRQ_REG_SST2_SET0:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET0 + 0x1000;
		break;
	case DP_IRQ_REG_SST3_SET0:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET0 + 0x2000;
		break;
	case DP_IRQ_REG_SST4_SET0:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET0 + 0x3000;
		break;
	case DP_IRQ_REG_SST1_SET1:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET1;
		break;
	case DP_IRQ_REG_SST2_SET1:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET1 + 0x1000;
		break;
	case DP_IRQ_REG_SST3_SET1:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET1 + 0x2000;
		break;
	case DP_IRQ_REG_SST4_SET1:
		irq_status_reg = SST1_INTERRUPT_STATUS_SET1 + 0x3000;
		break;
	case DP_IRQ_REG_SYSTEM:
	default:
		irq_status_reg = SYSTEM_IRQ_COMMON_STATUS;
		break;
	}

	val = dp_link_read(id, irq_status_reg);
	dp_link_write(id, irq_status_reg, ~0);

	return val;
}

static void dp_reg_set_daynamic_range(u32 id, u32 sst_id,
				      enum dynamic_range_type dynamic_range)
{
	dp_link_write_mask(id, SST1_VIDEO_CONTROL + 0x1000 * sst_id,
			   DYNAMIC_RANGE_VAL(dynamic_range),
			   DYNAMIC_RANGE_MODE);
}

static void dp_reg_set_video_bist_mode(u32 id, u32 sst_id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_link_write_mask(id, SST1_VIDEO_CONTROL + 0x1000 * sst_id, val,
			   STRM_VALID_FORCE | STRM_VALID_CTRL);
	dp_link_write_mask(id, SST1_VIDEO_BIST_CONTROL + 0x1000 * sst_id, val,
			   BIST_EN);
}

static void dp_reg_video_format_setting(u32 id,
					struct dp_video_info dp_video_info)
{
	u32 val = 0;
	struct dp_support_video *vm = &dp_video_info.vm;

	u32 sst_id = dp_video_info.sst_id;

	val += vm->vactive;
	val += vm->vfront_porch;
	val += vm->vsync_len;
	val += vm->vback_porch;
	dp_link_write(id, SST1_VIDEO_VERTICAL_TOTAL_PIXELS + 0x1000 * sst_id,
		      val);

	val = 0;
	val += vm->hactive;
	val += vm->hfront_porch;
	val += vm->hsync_len;
	val += vm->hback_porch;
	dp_link_write(id, SST1_VIDEO_HORIZONTAL_TOTAL_PIXELS + 0x1000 * sst_id,
		      val);

	val = vm->vactive;
	dp_link_write(id, SST1_VIDEO_VERTICAL_ACTIVE + 0x1000 * sst_id, val);

	val = vm->vfront_porch;
	dp_link_write(id, SST1_VIDEO_VERTICAL_FRONT_PORCH + 0x1000 * sst_id,
		      val);

	val = vm->vback_porch;
	dp_link_write(id, SST1_VIDEO_VERTICAL_BACK_PORCH + 0x1000 * sst_id,
		      val);

	val = vm->hactive;
	dp_link_write(id, SST1_VIDEO_HORIZONTAL_ACTIVE + 0x1000 * sst_id, val);

	val = vm->hfront_porch;
	dp_link_write(id, SST1_VIDEO_HORIZONTAL_FRONT_PORCH + 0x1000 * sst_id,
		      val);

	val = vm->hback_porch;
	dp_link_write(id, SST1_VIDEO_HORIZONTAL_BACK_PORCH + 0x1000 * sst_id,
		      val);

	val = vm->vsync_pol;
	dp_link_write_mask(id, SST1_VIDEO_CONTROL + 0x1000 * sst_id,
			   VSYNC_POLARITY_VAL(val), VSYNC_POLARITY);

	val = vm->hsync_pol;
	dp_link_write_mask(id, SST1_VIDEO_CONTROL + 0x1000 * sst_id, val,
			   HSYNC_POLARITY);
}

static bool is_dsc_en(u32 id, u32 sst_id)
{
	u32 val = dp_link_read_mask(
		id, SST_REG(SST1_VIDEO_DSC_STREAM_CONTROL_0), DSC_ENABLE);

	return val ? true : false;
}

static void dp_reg_set_dsc_pps_sdp_control(u32 id, u32 sst_id)
{
	u32 val;

	val = PPS_SDP_UPDATE;
	dp_link_write(id, SST_REG(SST1_PPS_SDP_CONTROL), val);

	val |= PPS_SDP_FRAME_SEND_ENABLE;
	dp_link_write(id, SST_REG(SST1_PPS_SDP_CONTROL), val);
}

static void dp_reg_set_dsc_pps(u32 id, u32 sst_id, u8 *pps, u32 en)
{
	u32 pps_val;
	int i;

	/* DSC pps */
	for (i = 0; i < MAX_PPS_NUM; i += SZ_4) {
		/* Clear PPS */
		if (!en) {
			dp_link_write(id, SST_REG(SST1_PPS_SDP_PAYLOAD_0) + i,
				      0);
			continue;
		}

		pps_val = 0;
		pps_val |= PPS0xN(pps[i + 0]);
		pps_val |= PPS1xN(pps[i + 1]);
		pps_val |= PPS2xN(pps[i + 2]);
		pps_val |= PPS3xN(pps[i + 3]);

		dp_link_write(id, SST_REG(SST1_PPS_SDP_PAYLOAD_0) + i, pps_val);
		cal_log_debug(id, "PPS_SDP_PAYLOAD_%02d(%03d~%03d) 0x%.8X\n",
			      i / SZ_4, (i + SZ_4 - 1), i, pps_val);
	}

	if (en)
		dp_reg_set_dsc_pps_sdp_control(id, sst_id);
}

static void dp_reg_set_dsc_en(u32 id, u32 sst_id, u32 slice_cnt, u32 en)
{
	u32 val = 0;

	if (slice_cnt > MAX_SLICE_COUNT && en) {
		cal_log_err(id, "slice count(%u) not supported\n", slice_cnt);
		slice_cnt = MAX_SLICE_COUNT;
	}

	if (en) {
		val |= DSC_ENABLE;
		val |= slice_cnt & SLICE_COUNT_PER_LINE;
	}

	dp_link_write(id, SST_REG(SST1_VIDEO_DSC_STREAM_CONTROL_0), val);
	cal_log_debug(id, "SST%u DSC_STREAM_CONTROL_0: %#x\n", sst_id + 1, val);

	/* If FEC is not ready then ignore it */
	if (!dp_link_read_mask(id, PCS_CONTROL, FEC_READY))
		return;

	val = en ? ~0 : 0;
	dp_link_write_mask(id, PCS_DEBUG_CONTROL, ~val, FEC_DECODE_DIS_4TH_SEL);
	dp_link_write_mask(id, PCS_DEBUG_CONTROL, val, FEC_DECODE_EN_4TH_SEL);
	dp_link_write_mask(id, PCS_CONTROL, val, FEC_FUNC_EN);
}

static void dp_reg_set_dsc_stream(u32 id, u32 sst_id, struct dp_dsc dsc, u32 en)
{
	u32 val[2] = {
		0,
	};
	int i;
	u8 slice_cnt = dsc.slice_count;
	u16 chunk_size = dsc.chunk_size;

	/* request disable but already disabled */
	if (!en && !is_dsc_en(id, sst_id))
		return;

	/* request enable but dsc is not required */
	if (en && !dsc.enable)
		return;

	for (i = 0; i < slice_cnt && en; i++) {
		if (i < MAX_SLICE_COUNT / 2)
			val[0] |= (chunk_size << (i % 2) * 16);
		else
			val[1] |= (chunk_size << (i % 2) * 16);
	}
	dp_link_write(id, SST_REG(SST1_VIDEO_DSC_STREAM_CONTROL_1), val[0]);
	dp_link_write(id, SST_REG(SST1_VIDEO_DSC_STREAM_CONTROL_2), val[1]);

	if (en) {
		cal_log_info(id, "SST%u, slice count(%u), chunk_size(%u)\n",
			     sst_id + 1, slice_cnt, chunk_size);

		cal_log_debug(id, "DSC_STREAM_CONTROL_1: %#x, CONTROL_2: %#x\n",
			      val[0], val[1]);
	}

	dp_reg_set_dsc_pps(id, sst_id, dsc.pps, en);
	dp_reg_set_dsc_en(id, sst_id, slice_cnt, en);
}

void dp_reg_set_dsc_fec(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_link_write_mask(id, PCS_CONTROL, val, FEC_READY);
}

static u32 dp_reg_get_ls_clk(u32 id)
{
	u32 val;
	u32 ls_clk;

	val = dp_reg_phy_get_link_bw(id);

	if (val == LINK_RATE_8_1Gbps)
		ls_clk = 810000000;
	else if (val == LINK_RATE_5_4Gbps)
		ls_clk = 540000000;
	else if (val == LINK_RATE_2_7Gbps)
		ls_clk = 270000000;
	else /* LINK_RATE_1_62Gbps */
		ls_clk = 162000000;

	return ls_clk;
}

static void dp_reg_set_video_clock(u32 id, u32 sst_id, u32 pixelclock)
{
	u32 stream_clk = 0;
	u32 ls_clk = 0;
	u32 mvid_master = 0;
	u32 nvid_master = 0;

	stream_clk = pixelclock / 1000;
	ls_clk = dp_reg_get_ls_clk(id) / 1000;

	mvid_master = stream_clk >> 1;
	nvid_master = ls_clk;

	dp_link_write(id, SST1_MVID_MASTER_MODE + 0x1000 * sst_id, mvid_master);
	dp_link_write(id, SST1_NVID_MASTER_MODE + 0x1000 * sst_id, nvid_master);

	dp_link_write(id, SST1_MVID_SFR_CONFIGURE + 0x1000 * sst_id,
		      stream_clk);
	dp_link_write(id, SST1_NVID_SFR_CONFIGURE + 0x1000 * sst_id, ls_clk);
}

static void dp_reg_set_active_symbol(u32 id, u32 sst_id, u32 pixelclock, u8 bpc)
{
	u64 TU_off = 0; /* TU Size when FEC is off*/
	u64 TU_on = 0; /* TU Size when FEC is on*/
	u32 bpp = 0; /* Bit Per Pixel */
	u32 lanecount = 0;
	u32 bandwidth = 0;
	u32 integer_fec_off = 0;
	u32 fraction_fec_off = 0;
	u32 threshold_fec_off = 0;
	u32 integer_fec_on = 0;
	u32 fraction_fec_on = 0;
	u32 threshold_fec_on = 0;
	u32 clk = 0;

	switch (bpc) {
	case BPC_8:
		bpp = 24;
		break;
	case BPC_10:
		bpp = 30;
		break;
	default:
		bpp = 18;
		break;
	}

	/* if DSC on, bpp / 3 */
	if (is_dsc_en(id, sst_id))
		bpp /= 3;

	/* change to Mbps from bps of pixel clock*/
	clk = pixelclock / 1000;

	bandwidth = dp_reg_get_ls_clk(id) / 1000;
	if (dp_link_read(id, MST_ENABLE) & MST_EN)
		lanecount =
			MAX_LANE; /* In MST mode, number of lanes become 4 */
	else
		lanecount = dp_reg_get_lane_count(id);

	TU_off = ((clk * bpp * 32) * 10000000000) / (lanecount * bandwidth * 8);
	TU_on = (TU_off * 1000) / 976;

	integer_fec_off = (u32)(TU_off / 10000000000);
	fraction_fec_off =
		(u32)((TU_off - (integer_fec_off * 10000000000)) / 10);
	integer_fec_on = (u32)(TU_on / 10000000000);
	fraction_fec_on = (u32)((TU_on - (integer_fec_on * 10000000000)) / 10);

	if (integer_fec_off <= 2)
		threshold_fec_off = 7;
	else if (integer_fec_off > 2 && integer_fec_off <= 5)
		threshold_fec_off = 8;
	else /* integer_fec_off > 5 */
		threshold_fec_off = 9;

	if (integer_fec_on <= 2)
		threshold_fec_on = 7;
	else if (integer_fec_on > 2 && integer_fec_on <= 5)
		threshold_fec_on = 8;
	else /* integer_fec_on > 5 */
		threshold_fec_on = 9;

	cal_log_debug(
		id,
		"fec_off(int: %d, frac: %d, thr: %d), fec_on(int: %d, frac: %d, thr: %d)\n",
		integer_fec_off, fraction_fec_off, threshold_fec_off,
		integer_fec_on, fraction_fec_on, threshold_fec_on);

	dp_link_write_mask(id,
			   SST1_ACTIVE_SYMBOL_INTEGER_FEC_OFF + 0x1000 * sst_id,
			   integer_fec_off, ACTIVE_SYMBOL_INTEGER_FEC_OFF);
	dp_link_write_mask(
		id, SST1_ACTIVE_SYMBOL_FRACTION_FEC_OFF + 0x1000 * sst_id,
		fraction_fec_off, ACTIVE_SYMBOL_FRACTION_FEC_OFF);
	dp_link_write_mask(
		id, SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_OFF + 0x1000 * sst_id,
		threshold_fec_off, ACTIVE_SYMBOL_FRACTION_FEC_OFF);

	dp_link_write_mask(id,
			   SST1_ACTIVE_SYMBOL_INTEGER_FEC_ON + 0x1000 * sst_id,
			   integer_fec_on, ACTIVE_SYMBOL_INTEGER_FEC_ON);
	dp_link_write_mask(id,
			   SST1_ACTIVE_SYMBOL_FRACTION_FEC_ON + 0x1000 * sst_id,
			   fraction_fec_on, ACTIVE_SYMBOL_FRACTION_FEC_OFF);
	dp_link_write_mask(
		id, SST1_ACTIVE_SYMBOL_THRESHOLD_FEC_ON + 0x1000 * sst_id,
		threshold_fec_on, ACTIVE_SYMBOL_THRESHOLD_FEC_ON);
}

static void dp_reg_aux_ch_buf_clr(u32 id)
{
	dp_link_write_mask(id, AUX_BUFFER_CLEAR, 1, AUX_BUF_CLR);
}

static void dp_reg_aux_defer_ctrl(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;

	dp_link_write_mask(id, AUX_COMMAND_CONTROL, val, DEFER_CTRL_EN);
}

static void dp_reg_set_aux_reply_timeout(u32 id)
{
	dp_link_write_mask(id, AUX_CONTROL,
			   AUX_REPLY_TIMER_VAL(AUX_TIMEOUT_1800us),
			   AUX_REPLY_TIMER_MODE);
}

static void dp_reg_set_aux_ch_command(u32 id,
				      enum aux_ch_command_type aux_ch_mode)
{
	dp_link_write_mask(id, AUX_REQUEST_CONTROL, REQ_COMM_VAL(aux_ch_mode),
			   REQ_COMM);
}

static void dp_reg_set_aux_ch_address(u32 id, u32 aux_ch_address)
{
	dp_link_write_mask(id, AUX_REQUEST_CONTROL,
			   REQ_ADDR_VAL(aux_ch_address), REQ_ADDR);
}

static void dp_reg_set_aux_ch_length(u32 id, u32 aux_ch_length)
{
	dp_link_write_mask(id, AUX_REQUEST_CONTROL, aux_ch_length - 1,
			   REQ_LENGTH);
}

static void dp_reg_aux_ch_send_buf(u32 id, u8 *aux_ch_send_buf,
				   u32 aux_ch_length)
{
	int i;

	for (i = 0; i < aux_ch_length; i++) {
		dp_link_write_mask(id, AUX_TX_DATA_SET0 + ((i / 4) * 4),
				   (aux_ch_send_buf[i] << ((i % 4) * 8)),
				   (0x000000FF << ((i % 4) * 8)));
	}
}

static void dp_reg_aux_ch_received_buf(u32 id, u8 *aux_ch_received_buf,
				       u32 aux_ch_length)
{
	int i;

	for (i = 0; i < aux_ch_length; i++) {
		aux_ch_received_buf[i] =
			(dp_link_read_mask(id, AUX_RX_DATA_SET0 + ((i / 4) * 4),
					   0xFF << ((i % 4) * 8)) >>
			 (i % 4) * 8);
	}
}

static int dp_reg_set_aux_ch_operation_enable(u32 id)
{
	u32 cnt = 5000;
	u32 state;
	u32 val0, val1;

	dp_link_write_mask(id, AUX_TRANSACTION_START, 1, AUX_TRAN_START);

	do {
		state = dp_link_read(id, AUX_TRANSACTION_START) &
			AUX_TRAN_START;
		cnt--;
		udelay(10);
	} while (state && cnt);

	if (!cnt) {
		cal_log_err(id, "AUX_TRAN_START waiting timeout.\n");
		return -ETIME;
	}

	val0 = dp_link_read(id, AUX_MONITOR_1);
	val1 = dp_link_read(id, AUX_MONITOR_2);

	if ((val0 & AUX_CMD_STATUS) != 0x00 || val1 != 0x00) {
		cal_log_err(id, "AUX_MONITOR_1 : 0x%X, AUX_MONITOR_2 : 0x%X\n",
			    val0, val1);
		cal_log_err(
			id,
			"AUX_CONTROL : 0x%X, AUX_REQUEST_CONTROL : 0x%X, AUX_COMMAND_CONTROL : 0x%X\n",
			dp_link_read(id, AUX_CONTROL),
			dp_link_read(id, AUX_REQUEST_CONTROL),
			dp_link_read(id, AUX_COMMAND_CONTROL));

		udelay(400);
		return -EIO;
	}

	return 0;
}

int dp_reg_aux_write(u32 id, u32 comm, u32 address, u32 length, u8 *data)
{
	int ret = 0;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_reg_aux_ch_buf_clr(id);
		dp_reg_aux_defer_ctrl(id, 1);
		dp_reg_set_aux_reply_timeout(id);
		dp_reg_set_aux_ch_command(id, comm);
		dp_reg_set_aux_ch_address(id, address);
		dp_reg_set_aux_ch_length(id, length);
		if (comm == DPCD_WRITE)
			dp_reg_aux_ch_send_buf(id, data, length);

		ret = dp_reg_set_aux_ch_operation_enable(id);
		if (ret == 0) {
			ret = length;
			break;
		}
		retry_cnt--;
	}

	return ret;
}

int dp_reg_aux_read(u32 id, u32 comm, u32 address, u32 length, u8 *data)
{
	int ret = 0;
	int retry_cnt = AUX_RETRY_COUNT;

	while (retry_cnt > 0) {
		dp_link_write(id, AUX_REQUEST_CONTROL, 0);
		dp_reg_set_aux_ch_command(id, comm);
		dp_reg_set_aux_ch_address(id, address);
		if (length)
			dp_reg_set_aux_ch_length(id, length);
		dp_reg_aux_ch_buf_clr(id);
		dp_reg_aux_defer_ctrl(id, 1);
		dp_reg_set_aux_reply_timeout(id);

		ret = dp_reg_set_aux_ch_operation_enable(id);
		if (ret == 0) {
			dp_reg_aux_ch_received_buf(id, data, length);
			ret = length;
			break;
		}
		retry_cnt--;
	}

	return ret;
}

int dp_reg_dpcd_write_burst(u32 id, u32 address, u32 length, u8 *data)
{
	int ret = 0, transferred = 0;
	u32 i, buf_length, length_calculation;

	length_calculation = length;
	for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
		if (length_calculation >= AUX_DATA_BUF_COUNT) {
			buf_length = AUX_DATA_BUF_COUNT;
			length_calculation -= AUX_DATA_BUF_COUNT;
		} else {
			buf_length = length % AUX_DATA_BUF_COUNT;
			length_calculation = 0;
		}

		ret = dp_reg_aux_write(id, DPCD_WRITE, address + i, buf_length,
				       data + i);
		if (ret <= 0) {
			cal_log_err(id, "DPCD write burst fail\n");
			break;
		}

		transferred += ret;
	}

	return transferred;
}

int dp_reg_dpcd_read_burst(u32 id, u32 address, u32 length, u8 *data)
{
	int ret = 0, transferred = 0;
	u32 i, buf_length, length_calculation;

	length_calculation = length;

	for (i = 0; i < length; i += AUX_DATA_BUF_COUNT) {
		if (length_calculation >= AUX_DATA_BUF_COUNT) {
			buf_length = AUX_DATA_BUF_COUNT;
			length_calculation -= AUX_DATA_BUF_COUNT;
		} else {
			buf_length = length % AUX_DATA_BUF_COUNT;
			length_calculation = 0;
		}

		ret = dp_reg_aux_read(id, DPCD_READ, address + i, buf_length,
				      data + i);
		if (ret <= 0) {
			cal_log_err(id, "DPCD read burst fail\n");
			break;
		}

		transferred += ret;
	}

	return transferred;
}

static void dp_reg_set_lane_map(u32 id, u32 lane0, u32 lane1, u32 lane2,
				u32 lane3)
{
	dp_link_write_mask(id, PCS_LANE_CONTROL, lane0, LANE0_MAP);
	dp_link_write_mask(id, PCS_LANE_CONTROL, LANE1_MAP_VAL(lane1),
			   LANE1_MAP);
	dp_link_write_mask(id, PCS_LANE_CONTROL, LANE2_MAP_VAL(lane2),
			   LANE2_MAP);
	dp_link_write_mask(id, PCS_LANE_CONTROL, LANE3_MAP_VAL(lane3),
			   LANE3_MAP);
}

static void dp_reg_set_lane_map_config(u32 id)
{
	dp_reg_set_lane_map(id, 0, 1, 2, 3);
}

static void dp_reg_lh_p_ch_power(u32 id, u32 sst_id, u32 en)
{
	u32 cnt = 20 * 1000; /* wait 1ms */
	u32 state;
	u32 reg_offset = 0;
	u32 val;

	val = en ? ~0 : 0;

	switch (sst_id) {
	case 3:
		reg_offset = SYSTEM_SST4_FUNCTION_ENABLE;
		break;
	case 2:
		reg_offset = SYSTEM_SST3_FUNCTION_ENABLE;
		break;
	case 1:
		reg_offset = SYSTEM_SST2_FUNCTION_ENABLE;
		break;
	case 0:
	default:
		reg_offset = SYSTEM_SST1_FUNCTION_ENABLE;
	}

	dp_link_write_mask(id, reg_offset, val, SST1_LH_PWR_ON);

	do {
		state = dp_link_read_mask(id, reg_offset,
					  SST1_LH_PWR_ON_STATUS);
		cnt--;
		udelay(1);
	} while ((en ^ state) && cnt);
	/*
	 * en xor state
	 * if (en), do while (!state) = en(1), state(0)
	 * else, do while(state) = en(0), state(1)
	 */

	if (!(en ^ state) && !cnt) {
		cal_log_err(id, "%s on is timeout[%d].\n", __func__, state);
		cal_log_err(id, "SYSTEM_CLK_CONTROL[0x%08x]\n",
			    dp_link_read(id, SYSTEM_CLK_CONTROL));
		cal_log_err(id, "SYSTEM_PLL_LOCK_CONTROL[0x%08x]\n",
			    dp_link_read(id, SYSTEM_PLL_LOCK_CONTROL));
		cal_log_err(id, "SYSTEM_DEBUG[0x%08x]\n",
			    dp_link_read(id, SYSTEM_DEBUG));
		cal_log_err(id, "SYSTEM_DEBUG_LH_PCH[0x%08x]\n",
			    dp_link_read(id, SYSTEM_DEBUG_LH_PCH));
		cal_log_err(id, "SST%d_VIDEO_CONTROL[0x%08x]\n", sst_id + 1,
			    dp_link_read(id,
					 SST1_VIDEO_CONTROL + 0x1000 * sst_id));
		cal_log_err(id, "SST%d_VIDEO_DEBUG_FSM_STATE[0x%08x]\n",
			    sst_id + 1,
			    dp_link_read(id, SST1_VIDEO_DEBUG_FSM_STATE +
						     0x1000 * sst_id));
		cal_log_err(id, "SST%d_VIDEO_DEBUG_MAPI[0x%08x]\n", sst_id + 1,
			    dp_link_read(id, SST1_VIDEO_DEBUG_MAPI +
						     0x1000 * sst_id));
		cal_log_err(id, "SYSTEM_SW_FUNCTION_ENABLE[0x%08x]\n",
			    dp_link_read(id, SYSTEM_SW_FUNCTION_ENABLE));
		cal_log_err(id, "SYSTEM_COMMON_FUNCTION_ENABLE[0x%08x]\n",
			    dp_link_read(id, SYSTEM_COMMON_FUNCTION_ENABLE));
		cal_log_err(id, "SYSTEM_SST%d_FUNCTION_ENABLE[0x%08x]\n",
			    sst_id + 1, dp_link_read(id, reg_offset));
	}
}

static void dp_reg_sw_function_en(u32 id, u32 en)
{
	dp_link_write_mask(id, SYSTEM_SW_FUNCTION_ENABLE, en, SW_FUNC_EN);
}

int dp_reg_get_link_clock(u32 id)
{
	int val = 0;

	val = dp_link_read_mask(id, SYSTEM_CLK_CONTROL, GFMUX_STATUS_TXCLK);

	/* If PHY_PLL goes to lock and video data is transferred from DECON
	   then GFMUX_STATUS_TXCLK changes from OSC_CLK to TXCLK */
	return val & GFMUX_STATUS_TXCLK_ON;
}

bool dp_reg_get_sst_pstate(u32 id)
{
	u32 val = 0;

	u32 mask = SST1_LH_PSTATE | SST2_LH_PSTATE | SST3_LH_PSTATE |
		   SST4_LH_PSTATE;
	val = dp_link_read_mask(id, SYSTEM_DEBUG_LH_PCH, mask);

	if (val)
		return true;
	else
		return false;
}

static u32 dp_reg_get_lh_power_status(u32 id, u32 sst_id)
{
	u32 val = 0;
	u32 reg_offset = 0;

	switch (sst_id) {
	case 0:
		reg_offset = SYSTEM_SST1_FUNCTION_ENABLE;
		break;
	case 1:
		reg_offset = SYSTEM_SST2_FUNCTION_ENABLE;
		break;
	case 2:
		reg_offset = SYSTEM_SST3_FUNCTION_ENABLE;
		break;
	case 3:
		reg_offset = SYSTEM_SST4_FUNCTION_ENABLE;
		break;
	default:
		reg_offset = SYSTEM_SST1_FUNCTION_ENABLE;
	}

	val = dp_link_read_mask(id, reg_offset, SST1_LH_PWR_ON_STATUS);
	val |= dp_link_read_mask(id, reg_offset, SST1_VIDEO_FUNC_EN);

	return val;
}

static void dp_reg_phy_init(u32 id)
{
	dp_reg_phy_reset(id, 1);
	dp_reg_phy_init_setting(id);
	dp_reg_phy_mode_setting(id);
	dp_reg_phy_ssc_enable(id, 0);
	dp_reg_phy_reset(id, 0);
	dp_reg_wait_phy_pll_lock(id);
}

void dp_reg_phy_disable(u32 id)
{
	dp_reg_phy_reset(id, 1);
	dp_phy_write(id, CMN_REG00A2, 0x00);
}

void dp_reg_init(u32 id)
{
	dp_reg_phy_init(id);
	dp_reg_common_function_enable(id, 1);
	dp_reg_sw_function_en(id, 1);
	dp_reg_set_lane_map_config(id);
}

void dp_reg_deinit(u32 id)
{
	dp_reg_common_function_enable(id, 0);
	dp_reg_sw_function_en(id, 0);
}

static u8 dp_reg_find_hw_bpc_value(u8 bpc)
{
	u8 ret;
	switch (bpc) {
	case 6:
		ret = BPC_6;
		break;
	case 8:
		ret = BPC_8;
		break;
	case 10:
		ret = BPC_10;
		break;
	default:
		ret = BPC_8;
		break;
	}

	return ret;
}

void dp_reg_set_video_config(u32 id, struct dp_video_info dp_video_info)
{
	u32 sst_id = -1;
	u8 bpc = 0;
	u8 range = 0;

	sst_id = dp_video_info.sst_id;
	bpc = dp_reg_find_hw_bpc_value(dp_video_info.bpc);
	range = dp_video_info.dyn_range;

	dp_reg_set_daynamic_range(id, sst_id, (range) ? CEA_RANGE : VESA_RANGE);
	dp_link_write_mask(id, SST1_VIDEO_CONTROL + 0x1000 * sst_id,
			   BPC_VAL(bpc), BPC); /* 0 : 6bits, 1 : 8bits */
	dp_link_write_mask(id, SST1_VIDEO_CONTROL + 0x1000 * sst_id, 0,
			   COLOR_FORMAT); /* RGB */
	dp_reg_video_format_setting(id, dp_video_info);
	dp_reg_set_video_clock(id, sst_id, dp_video_info.vm.pixelclock);
	/* DSC: Note that dsc_en should be called first to set active_symbol properly */
	dp_reg_set_dsc_stream(id, sst_id, dp_video_info.dsc, 1);
	dp_reg_set_active_symbol(id, sst_id, dp_video_info.vm.pixelclock, bpc);
	dp_link_write_mask(id, SST1_VIDEO_MASTER_TIMING_GEN + 0x1000 * sst_id,
			   ~0, VIDEO_MASTER_TIME_GEN);
	dp_link_write_mask(id, SST1_MAIN_CONTROL + 0x1000 * sst_id, 0,
			   VIDEO_MODE);
}

void dp_reg_set_bist_video_config(u32 id, struct dp_video_info dp_video_info,
				  u8 type)
{
	u32 sst_id = -1;
	u8 bpc = 0;
	u8 range = 0;

	sst_id = dp_video_info.sst_id;
	bpc = dp_reg_find_hw_bpc_value(dp_video_info.bpc);
	range = dp_video_info.dyn_range;

	if (type < CTS_COLOR_RAMP) {
		dp_reg_set_video_config(id, dp_video_info);
		dp_link_write_mask(id,
				   SST1_VIDEO_BIST_CONTROL + 0x1000 * sst_id,
				   type, BIST_TYPE);
		dp_link_write_mask(id,
				   SST1_VIDEO_BIST_CONTROL + 0x1000 * sst_id, 0,
				   CTS_BIST_EN);
	} else {
		if (type == CTS_COLOR_SQUARE_CEA)
			dp_video_info.dyn_range = CEA_RANGE;
		else
			dp_video_info.dyn_range = VESA_RANGE;
		dp_reg_set_video_config(id, dp_video_info);

		dp_link_write_mask(id,
				   SST1_VIDEO_BIST_CONTROL + 0x1000 * sst_id,
				   CTS_BIST_TYPE_VAL(type - CTS_COLOR_RAMP),
				   CTS_BIST_TYPE);
		dp_link_write_mask(id,
				   SST1_VIDEO_BIST_CONTROL + 0x1000 * sst_id,
				   ~0, CTS_BIST_EN);
	}

	dp_reg_set_video_bist_mode(id, sst_id, 1);

	cal_log_info(
		id, "set bist video config res:%dx%d range:%d bpc:%d type:%d\n",
		dp_video_info.vm.hactive, dp_video_info.vm.vactive,
		(range) ? 1 : 0, (bpc) ? 1 : 0, type);
}

void dp_reg_start(u32 id, u32 sst_id)
{
	dp_reg_sst_function_enable(id, sst_id, 1);
	dp_reg_set_sst_stream_enable(id, sst_id, 1);
	dp_reg_set_sst_interrupt_mask(id, sst_id, VIDEO_FIFO_UNDER_FLOW_MASK,
				      0);
	dp_link_write_mask(id, SST1_VIDEO_ENABLE + 0x1000 * sst_id, 1,
			   VIDEO_EN);
	dp_reg_lh_p_ch_power(id, sst_id, 1);
}

static void dp_reg_wait_vsync_interrupt_status(u32 id, u32 sst_id)
{
	u32 cnt = 500; /* waiting time:50ms */
	u32 state;
	u32 irq_status_reg = SST1_INTERRUPT_STATUS_SET0 + (0x1000 * sst_id);

	do {
		state = dp_link_read_mask(id, irq_status_reg, VSYNC_DET_MASK);
		cnt--;
		udelay(100);
	} while (!state && cnt);

	if (!cnt)
		cal_log_err(id, "%s is timeout.\n", __func__);
}

void dp_reg_stop(u32 id, u32 sst_id)
{
	u32 offset = 0x1000 * sst_id;
	struct dp_video_info dp_vi;

	memset(&dp_vi, 0, sizeof(struct dp_video_info));

	if (!dp_reg_get_lh_power_status(id, sst_id)) {
		cal_log_info(id, "already IDLE status(sst=%d)\n", sst_id);
		return;
	}

	dp_reg_set_sst_interrupt_mask(id, sst_id, VIDEO_FIFO_UNDER_FLOW_MASK,
				      0);
	dp_link_write_mask(id, SST1_VIDEO_ENABLE + offset, 0, VIDEO_EN);
	dp_reg_get_int_and_clear(id, DP_IRQ_REG_SST1_SET0 + sst_id);
	dp_reg_wait_vsync_interrupt_status(id, sst_id);
	dp_link_write_mask(id, SST1_VIDEO_MASTER_TIMING_GEN + offset, 0,
			   VIDEO_MASTER_TIME_GEN);
	dp_link_write_mask(id, SST1_VIDEO_ENABLE + offset, 0, VIDEO_EN);
	dp_reg_lh_p_ch_power(id, sst_id, 0);

	/* DSC: DSC disable should be called after checking check LH_PWR_ON_STATUS[5] = 1'b0 */
	dp_reg_set_dsc_stream(id, sst_id, dp_vi.dsc, 0);

	dp_link_write_mask(id, SST1_MAIN_FIFO_CONTROL + offset, ~0,
			   CLEAR_MAPI_FIFO);
	dp_reg_get_int_and_clear(id, DP_IRQ_REG_SST1_SET0 + sst_id);
	dp_reg_set_sst_stream_enable(id, sst_id, 0);
}

void dp_reg_set_mst_en(u32 id, u32 en)
{
	u32 val = en ? ~0 : 0;
	dp_link_write_mask(id, MST_ENABLE, val, MST_EN);
}

void dp_reg_set_strm_x_y(u32 id, u32 sst_id, u32 x_val, u32 y_val)
{
	cal_log_info(id, "SST:%d, x:%d, y:%d\n", sst_id + 1, x_val, y_val);
	dp_link_write_mask(id, MST_STREAM_X_VALUE(sst_id), x_val, STRM_VALUE);
	dp_link_write_mask(id, MST_STREAM_Y_VALUE(sst_id), y_val, STRM_VALUE);
}

void dp_reg_set_vcpi_timeslot(u32 id, u32 sst_id, u32 start, u32 size)
{
	int i;
	u32 val[8] = {0, }, shift_val;
	int timeslot_start = start - 1;
	int timeslot_end = timeslot_start + size;
	int start_offset = timeslot_start / 8;
	int end_offset = timeslot_end / 8;

	if (timeslot_end <= MAX_VC_PAYLOAD_TIMESLOT) {
		val[start_offset] =
			dp_link_read(id, MST_VC_PAYLOAD_ID_TIMESLOT_01_08 +
						 4 * start_offset);

		for (i = timeslot_start; i < timeslot_end; i++) {
			/* bit masking first */
			shift_val = 28 - ((i * 4) % 32);
			val[i / 8] &= VC_PAYLOAD_ID_MASK(shift_val);
			/* save sst_id into timeslot bit */
			val[i / 8] |= sst_id << shift_val;
		}

		for (i = start_offset; i <= end_offset; i++)
			dp_link_write(id,
				      MST_VC_PAYLOAD_ID_TIMESLOT_01_08 + 4 * i,
				      val[i]);
	} else {
		cal_log_err(id, "Over MAX_VC_PAYLOAD_TIMESLOT\n");
	}

	for (i = start_offset; i <= end_offset; i++)
		cal_log_info(id, "MST_VC_PAYLOAD_ID_TIMESLOT_%d = 0x%8x\n", i,
			     dp_link_read(id, MST_VC_PAYLOAD_ID_TIMESLOT_01_08 +
						      4 * i));
}

static void dp_reg_reset_vcpi_timeslot(u32 id)
{
	u32 i;

	for (i = MST_VC_PAYLOAD_ID_TIMESLOT_01_08;
	     i <= MST_VC_PAYLOAD_ID_TIMESLOT_57_63; i += 4)
		dp_link_write(id, i, 0);
}

void dp_reg_remove_vcpi_timeslot(u32 id, u32 sst_id)
{
	int i;
	int idx = 0;
	int vcpi_idx[5] = {
		0,
	};
	int vcpi_num[5] = {
		0,
	};
	int vcpi_start = 0;
	u32 tmp;
	u32 sst_idx = sst_id + 1;

	if (!dp_link_read_mask(id, MST_ENABLE, MST_EN))
		return;

	for (i = 0; i <= MAX_VC_PAYLOAD_TIMESLOT; i++) {
		tmp = dp_link_read_mask(id,
					MST_VC_PAYLOAD_ID_TIMESLOT_01_08 +
						(4 * (int)(i / 8)),
					0x7 << (28 - (i % 8) * 4)) >>
		      (28 - (i % 8) * 4);
		if (sst_idx != tmp)
			vcpi_num[tmp]++;

		if (i == 0)
			vcpi_idx[idx] = tmp;
		else if (vcpi_idx[idx] != tmp) {
			idx++;
			vcpi_idx[idx] = tmp;
		}
	}

	dp_reg_reset_vcpi_timeslot(id);
	for (i = 0; i <= MAX_SST_CNT; i++) {
		if (i > 0)
			vcpi_start += vcpi_num[vcpi_idx[i - 1]];

		if (vcpi_num[vcpi_idx[i]] && vcpi_idx[i]) {
			dp_reg_set_vcpi_timeslot(id, vcpi_idx[i],
						 vcpi_start + 1,
						 vcpi_num[vcpi_idx[i]]);
		}
	}
}

int dp_reg_wait_for_vcpi_update(u32 id)
{
	u32 cnt = 3000;
	u32 val;

	dp_link_write(id, MST_VC_PAYLOAD_UPDATE_FLAG, VC_PAYLOAD_UPDATE_FLAG);

	do {
		val = dp_link_read(id, MST_VC_PAYLOAD_UPDATE_FLAG) &
		      VC_PAYLOAD_UPDATE_FLAG;
		cnt--;
		udelay(10);
	} while (val && cnt);

	if (!cnt) {
		cal_log_err(id, "MST_VC_PAYLOAD_UPDATE_FLAG wait timeout.\n");
		return -ETIMEDOUT;
	}

	return 0;
}

void dp_reg_set_mst_always_sent_act(u32 id, u32 en)
{
	u32 val;

	val = en ? ~0 : 0;
	/*
	 * MST_ETC_OPTION : 0x2070 : ALLOCATE_TIMESLOT_CHECK_TO_ACT[1]
	 * L : if VC_PAYLOAD Table is not updated, ACT is not sent.
	 * H : 'Always sent ACT' after ECF change
	 */
	dp_link_write_mask(id, MST_ETC_OPTION, val,
			   ALLOCATE_TIMESLOT_CHECK_TO_ACT);
}

u32 dp_reg_read_vcpi_timeslot(u32 id, u32 index)
{
	if (index >= NUM_VC_PAYLOAD_SLOT)
		index = NUM_VC_PAYLOAD_SLOT - 1;

	return dp_link_read(id, MST_VC_PAYLOAD_ID_TIMESLOT_01_08 + 4 * index);
}


#define DEV_NAME "exynos_dp"
#define DEFAULT_BPC 8 /* DP supports at least 8 bpc */

struct dp_dev_data {
	const enum chip_version version;
};

// static void __exynos910_drm_dp_dump_sfr(struct exynos_dp_subdev *subdev)
// {
// 	struct device *dev = subdev->dev;
// 	struct dp_regs *regs = &subdev->regs;
// 	int i;

// 	if (regs->link_addr) {
// 		print_dump_info("DP_LINK BASE(%s) +", dev_name(dev));
// 		print_hexdump(regs->link_addr,		0x210);
// 		print_hexdump(regs->link_addr + 0x1000,	0x60);
// 		print_hexdump(regs->link_addr + 0x1100,	0x20);
// 		print_hexdump(regs->link_addr + 0x2000,	0x80);
// 		print_hexdump(regs->link_addr + 0x3000,	0x120);
// 		/* TODO: Should be added SSTx SFR */
// 		print_dump_info("DP_LINK BASE(%s) -", dev_name(dev));
// 		for (i = 0; i < DP_SST_MAX; i++) {
// 			struct exynos_dp_video_info *vi = &subdev->vi[i];
// 			print_dump_info("DP_LINK SST%d +", i + 1);
// 			print_hexdump(regs->link_addr + 0x5000 + (0x1000 * i), 0x108);
// 			print_hexdump(regs->link_addr + 0x5400 + (0x1000 * i), 0x110);
// 			if (vi->dsc.enable) {
// 				print_hexdump(regs->link_addr + 0x5C20 + (0x1000 * i), 0x4);
// 				print_hexdump(regs->link_addr + 0x5D00 + (0x1000 * i), 0x80);
// 			}

// 			print_dump_info("DP_LINK SST%d -", i + 1);
// 		}
// 	}

// 	if (regs->phy_addr) {
// 		print_dump_info("DP__PHY BASE(%s) +", dev_name(dev));
// 		print_hexdump(regs->phy_addr,		0x428);
// 		print_hexdump(regs->phy_addr + 0x800,	0x4D4);
// 		print_hexdump(regs->phy_addr + 0x1000,	0xD0);
// 		print_hexdump(regs->phy_addr + 0x10D4,	0x4);
// 		print_hexdump(regs->phy_addr + 0x1800,	0x4D4);
// 		print_hexdump(regs->phy_addr + 0x2000,	0xD0);
// 		print_hexdump(regs->phy_addr + 0x20D4,	0x4);
// 		print_dump_info("DP__PHY BASE(%s) -", dev_name(dev));
// 	}
// }

int exynos_drm_dp_dump_sfr(struct exynos_dp_subdev *subdev)
{
	int acquired;

	if (!subdev || !subdev->dev)
		return -ENXIO;

	if (subdev->state == DP_STATE_OFF)
		return -EBUSY;

	acquired = console_trylock();

	if (subdev->version == V910)
		// __exynos910_drm_dp_dump_sfr(subdev);

	if (acquired)
		console_unlock();

	return 0;
}

void exynos_drm_dp_dpcd_status_dump(struct exynos_dp_subdev *dp)
{
	u8 max_lane;
	u8 link_lane = dp->lt_info.lane_cnt;
	int max_rate;
	int link_rate = drm_dp_bw_code_to_link_rate(dp->lt_info.link_rate);
	struct device *dev = dp->dev;
	bool enhance, tps3, tps4, dsc;
	u8 info[DP_LINK_STATUS_SIZE];

	max_rate	= drm_dp_max_link_rate(dp->dpcd);
	max_lane	= drm_dp_max_lane_count(dp->dpcd);
	enhance		= drm_dp_enhanced_frame_cap(dp->dpcd);
	tps3		= drm_dp_tps3_supported(dp->dpcd);
	tps4		= drm_dp_tps4_supported(dp->dpcd);
	dsc		= drm_dp_sink_supports_dsc(dp->dsc_dpcd);

	drm_dp_dpcd_read(&dp->aux, DP_SINK_COUNT, info, DP_LINK_STATUS_SIZE);
	dp_log_info(dev, "======= DP%d DPCD Link/Sink Dump =======\n", dp->id);
	dp_log_info(dev, "Link:: RATE(%d) LANE(%d) ENHANCED(%d)\n",
			max_rate, max_lane, enhance);
	dp_log_info(dev, "       TPS3(%d) TPS4(%d)\n",
			tps3, tps4);
	dp_log_info(dev, "SINK:: RATE(%d) LANE(%d)\n",
			link_rate, link_lane);
	dp_log_info(dev, "DPCD(0x200~205) %*ph\n", DP_LINK_STATUS_SIZE, info);

	if (dsc) {
		drm_dp_dpcd_readb(&dp->aux, 0x20F, &info[0]);
		dp_log_info(dev, "DSC_STATUS(0x20f):: %#2x\n", info[0]);

		drm_dp_dpcd_read(&dp->aux, DP_FEC_STATUS, info,
				DP_FEC_ERROR_COUNT_MSB - DP_FEC_STATUS);
		dp_log_info(dev, "FEC_STATUS(0x280~281):: %#2x, %#2x\n",
				info[0], info[1]);
	}
	dp_log_info(dev, "======= DP%d                     =======\n", dp->id);
}

static bool exynos_drm_dp_check_dpcd_status(struct exynos_dp_subdev *dp)
{
	u8 link_status[DP_LINK_STATUS_SIZE];
	int link_rate = drm_dp_bw_code_to_link_rate(dp->lt_info.link_rate);
	u8 link_lane = dp->lt_info.lane_cnt;
	bool status_err = false;

	drm_dp_dpcd_read_link_status(&dp->aux, link_status);

	if (!drm_dp_clock_recovery_ok(link_status, link_lane)) {
		dp_log_err(dp->dev, "failed w/ LANE_CR_DONE(0x202~0x203)\n");
		status_err = true;
	}

	if (!drm_dp_channel_eq_ok(link_status, link_lane)) {
		dp_log_err(dp->dev, "failed w/ INTERLANE_ALIGN_DONE(0x204)\n");
		status_err = true;
	}

	if (link_rate > drm_dp_max_link_rate(dp->dpcd)) {
		dp_log_err(dp->dev, "failed w/ MAX_LINK_RATE\n");
		status_err = true;
	}

	if (link_lane > drm_dp_max_lane_count(dp->dpcd)) {
		dp_log_err(dp->dev, "failed w/ MAX_LINK_LANE\n");
		status_err = true;
	}

	if (status_err)
		exynos_drm_dp_dpcd_status_dump(dp);

	if (dp->dp_debug.debug_hpd_irq)
		return true;

	return status_err;
}

void exynos_drm_dp_set_normal_data(struct exynos_dp_subdev *dp)
{
	dp_reg_set_training_pattern(dp->id, NORAMAL_DATA);
	dp_reg_scrambling_enable(dp->id, 1);
}

static void exynos_drm_dp_hpd_changed(struct exynos_dp_subdev *dp, int state)
{
	struct device *dev = dp->dev;
	int ret;

	if (dp->hpd_state == state) {
		dp_log_info(dev, "hpd same state skip %x\n", state);
		return;
	}

	if (dp->hpd_state == HPD_CHECK) {
		dp_log_info(dev, "hpd is being checked\n");
		return;
	}

	dp_log_info(dev, "hpd state cur:%d -> new:%d\n", dp->hpd_state, state);

	disable_irq(dp->hpd_gpio_irq);
	if (state) {
		dp->hpd_state = HPD_CHECK;
		if (!dp->training_state) {
			ret = exynos_drm_dp_link_training(dp);
			if (ret < 0)
				goto HPD_FAIL;
		}
		dp->training_state = true;
		dp->hpd_state = HPD_PLUG;

		if (state == HPD_PLUG && dp->detect)
			dp->detect(dev);
	} else {
		dp->hpd_state = HPD_UNPLUG;
		dp->training_state = false;

		if (dp->detect)
			dp->detect(dev);
	}

	dp_log_info(dp->dev, "hpd state goto %s\n", state ? "plug" : "unplug");
	enable_irq(dp->hpd_gpio_irq);
	return;

HPD_FAIL:
	dp_log_err(dev, "link training is failed\n");
	dp->training_state = false;
	dp->hpd_state = HPD_LT_FAILED;

	enable_irq(dp->hpd_gpio_irq);
	return;
}

static irqreturn_t exynos_drm_dp_irq(int irq, void *dev_id)
{
	struct exynos_dp_subdev *dp = dev_id;
	struct device *dev = dp->dev;
	u32 irq_type = 0, val;
	int i = 0;
	bool valid_gpio = gpio_is_valid(dp->hpd_gpio);
	unsigned long flags;

	spin_lock_irqsave(&dp->slock, flags);

	/* common interrupt */
	irq_type = dp_reg_get_int_and_clear(dp->id, DP_IRQ_REG_SYSTEM);

	/* The priority for PLUG and UNPLUG behavior is HPD_GPIO than DPTX_IRQ.*/
	if (!valid_gpio) {
		if (irq_type & DP_IRQ_HPD_CHG)
			dp_log_dbg(dev, "HPD_CHG detect\n");

		if (irq_type & DP_IRQ_HPD_PLUG_INT) {
			queue_delayed_work(dp->dp_wq, &dp->hpd_plug_work, 0);
			dp_log_info(dev, "HPD_PLUG detect\n");
		}
	} else {
		if (irq_type & DP_IRQ_HPD_LOST) {
			queue_delayed_work(dp->dp_wq, &dp->hpd_unplug_work, 0);
			dp_log_info(dev, "HPD_LOST detect\n");
		}

		if (irq_type & DP_IRQ_HPD_IRQ_FLAG) {
			queue_delayed_work(dp->dp_wq, &dp->hpd_irq_work, 0);
			dp_log_info(dev, "HPD_IRQ detect\n");
		}

		if (irq_type & DP_IRQ_HPD_PLUG_INT) {
			queue_delayed_work(dp->dp_wq, &dp->hpd_plug_work, 0);
			dp_log_info(dev, "HPD_PLUG detect\n");
		}
	}

	if (irq_type)
		dp_log_dbg(dev, "DP_IRQ_REG_SYSTEM=%#x\n", irq_type);

	/* SST1~4 interrupt */
	for (i = 0; i < MAX_SST_CNT; i++) {
		val = dp_reg_get_int_and_clear(dp->id, DP_IRQ_REG_SST1_SET0 + i);

		if ((val & DP_IRQ_MAPI_FIFO_UNDER_FLOW) && !irq_type)
			dp_log_errl(dev, "SST%d FIFO_UNDER_FLOW=%#x\n",
					i + 1, val);
	}

	spin_unlock_irqrestore(&dp->slock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_drm_dp_hpd_gpio_irq_thread(int irq, void *dev_id)
{
	struct exynos_dp_subdev *dp = dev_id;
	struct device *dev = dp->dev;
	unsigned long flags;

	spin_lock_irqsave(&dp->slock, flags);

	if ((gpio_get_value(dp->hpd_gpio) == HPD_PLUG) &&
			dp->hpd_state == HPD_UNPLUG) {
		queue_delayed_work(dp->dp_wq, &dp->hpd_plug_work, 0);
		dp_log_info(dev, "GPIO_HPD_PLUG detect\n");
	}
	spin_unlock_irqrestore(&dp->slock, flags);

	return IRQ_HANDLED;
}

void exynos_drm_dp_hpd_en(struct exynos_dp_subdev *dp)
{
	if (dp_reg_get_hpd_status(dp->id) && !dp->training_state)
		exynos_drm_dp_hpd_changed(dp, HPD_CHECK);
}

static void exynos_drm_dp_hpd_plug_work(struct work_struct *work)
{
	struct delayed_work *delay_work =
		container_of(work, struct delayed_work, work);
	struct exynos_dp_subdev *dp =
		container_of(delay_work, struct exynos_dp_subdev, hpd_plug_work);

	flush_delayed_work(&dp->hpd_unplug_work);
	exynos_drm_dp_hpd_changed(dp, HPD_PLUG);
}

static void exynos_drm_dp_hpd_unplug_work(struct work_struct *work)
{
	struct delayed_work *delay_work =
		container_of(work, struct delayed_work, work);
	struct exynos_dp_subdev *dp =
		container_of(delay_work, struct exynos_dp_subdev, hpd_unplug_work);

	if (!dp->hpd_state) {
		dp_log_err(dp->dev, "HPD is low(%d)\n", dp->hpd_state);
		return;
	}

	exynos_drm_dp_hpd_changed(dp, HPD_UNPLUG);
}

static void exynos_drm_dp_hpd_irq_work(struct work_struct *work)
{
	struct delayed_work *delay_work =
		container_of(work, struct delayed_work, work);
	struct exynos_dp_subdev *dp =
		container_of(delay_work, struct exynos_dp_subdev, hpd_irq_work);

	flush_delayed_work(&dp->hpd_unplug_work);

	if (dp->state == DP_STATE_OFF || dp->hpd_state == HPD_UNPLUG)
		return;

	if (exynos_drm_dp_check_dpcd_status(dp)) {
		dp_log_info(dp->dev, "link training in HPD_IRQ work\n");
		if (exynos_drm_dp_link_training(dp) < 0) {
			dp_log_err(dp->dev, "failed link_training in HPD_IRQ work\n");
			dp->hpd_state = HPD_IRQ;
			dp->training_state = false;
			if (dp->detect)
				dp->detect(dp->dev);
			dp_reg_set_plug_interrupt(dp->id, 1);
			return;
		}
		if (dp->hpd_state == HPD_IRQ) {
			dp->hpd_state = HPD_PLUG;
			dp->training_state = true;
			if (dp->detect)
				dp->detect(dp->dev);
			return;
		}
		if (dp_reg_get_sst_pstate(dp->id))
			exynos_drm_dp_set_normal_data(dp);

		if (dp->detect)
			dp->detect(dp->dev);
	}

	if (dp->mst_irq)
		dp->mst_irq(dp->dev);
}

static int exynos_drm_dp_init_resources(struct exynos_dp_subdev *dp, struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = dp->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "link_base");
	if (res == NULL) {
		dp_log_err(dev, "failed to find dp link memory resource for dp\n");
		return -EINVAL;
	}
	dp->regs.link_addr = devm_ioremap_resource(dp->dev, res);

	if (IS_ERR(dp->regs.link_addr)) {
		dp_log_err(dev, "failed to remap io region\n");
		return PTR_ERR(dp->regs.link_addr);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy_base");
	if (res == NULL) {
		dp_log_err(dev, "failed to find dp phy memory resource for dp\n");
		return -EINVAL;
	}
	dp->regs.phy_addr = devm_ioremap_resource(dp->dev, res);

	if (IS_ERR(dp->regs.phy_addr)) {
		dp_log_err(dev, "failed to remap io region\n");
		return PTR_ERR(dp->regs.phy_addr);
	}

	dp_regs_desc_init(dp->id, &dp->regs);

	return 0;
}

static ssize_t exynos_drm_dp_show_sfr(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct exynos_drm_dp *dp = dev_get_drvdata(dev);

	exynos_drm_dp_dump_sfr(dp->subdev);

	return 0;
}

static DEVICE_ATTR(dump_sfr, S_IRUGO, exynos_drm_dp_show_sfr, NULL);

static ssize_t exynos_drm_dp_show_link_lane(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;
	int rc;

	rc = sprintf(buf, "%d\n", dp->lt_info.max_link_lane);

	return rc;
}

static ssize_t exynos_drm_dp_store_link_lane(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;
	int rc;

	rc = kstrtou8(buf, 0, &dp->lt_info.max_link_lane);
	if (rc < 0)
		return rc;

	return len;
}

static DEVICE_ATTR(link_lane, 0644,
		exynos_drm_dp_show_link_lane, exynos_drm_dp_store_link_lane);

static ssize_t exynos_drm_dp_show_hpd_irq(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;
	int np;

	np = sprintf(buf, "debug_hpd_irq = %d\n", dp->dp_debug.debug_hpd_irq);
	np += sprintf(buf + np, "0: normal_op\n");
	np += sprintf(buf + np, "1: forced occur HPD_IRQ with Link_training\n");
	return np;
}

static ssize_t exynos_drm_dp_store_hpd_irq(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;

	if (dp->hpd_state == HPD_UNPLUG) {
		dp_log_err(dp->dev, "hpd state unplug\n");
		return -EACCES;
	}

	kstrtobool(buf, &dp->dp_debug.debug_hpd_irq);

	queue_delayed_work(dp->dp_wq, &dp->hpd_irq_work, 0);

	return len;
}

static DEVICE_ATTR(debug_hpd_irq, 0644,
		exynos_drm_dp_show_hpd_irq, exynos_drm_dp_store_hpd_irq);

static ssize_t exynos_drm_dp_show_set_lt(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;
	int np;

	np  = sprintf(buf, "debug_lt = %d\n", dp->dp_debug.debug_lt);
	np += sprintf(buf + np, "0: normal_op\n");
	np += sprintf(buf + np, "1: link_training - dpcd_read fail\n");
	np += sprintf(buf + np, "2: link_training - EQ and CR fail, fixed BW by DP_Rx\n");
	np += sprintf(buf + np, "3: link_training - EQ and CR fail, Try Step Down BW\n");
	np += sprintf(buf + np, "4: link_training - Start Bandwidth one-step Lower from DP_Rx\n");
	np += sprintf(buf + np, "\t\t  (eq : DP_Rx(DPCD = 8.1Gbps) -> Start BW 5.4Gbps)\n");
	np += sprintf(buf + np, "5: link_training - No Stepdown BW from DP_Rx\n");
	return np;
}

static ssize_t exynos_drm_dp_store_set_lt(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;

	if (dp->hpd_state == HPD_UNPLUG) {
		dp_log_err(dp->dev, "hpd state unplug\n");
		return -EACCES;
	}

	kstrtou32(buf, 0, &dp->dp_debug.debug_lt);

	return len;
}

static DEVICE_ATTR(debug_lt, 0644,
		exynos_drm_dp_show_set_lt, exynos_drm_dp_store_set_lt);

static ssize_t sysfs_show_bist_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{

	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;

	return sprintf(buf, "%#x(0x01.Color 0x11.Gray 0x21.Moving 0x31.PRBS)\n",
			dp->bist_use);
}

static int bist_mode_set_config(struct exynos_drm_dp *drm_dp)
{
	struct drm_connector *connector = drm_dp->connector;
	const struct drm_connector_helper_funcs *funcs
				= connector->helper_private;
	struct drm_display_mode *mode;
	struct list_head *modes;
	struct exynos_dp_video_info vi;

	mutex_lock(&connector->dev->mode_config.mutex);

	/* Get a list_head with modes */
	modes = list_empty(&connector->probed_modes) ?
			&connector->modes : &connector->probed_modes;
	if (list_empty(modes)) {
		/* Get a probed_modes via get_modes */
		if (funcs && funcs->get_modes)
			funcs->get_modes(connector);
		modes = &connector->probed_modes;
	}

	list_for_each_entry(mode, modes, head)
		if (mode->type & DRM_MODE_TYPE_PREFERRED)
			break;

	mutex_unlock(&connector->dev->mode_config.mutex);

	exynos_drm_dp_to_videoinfo(drm_dp->encoder, mode, &vi);

	exynos_drm_dp_videomode_set(drm_dp->subdev, &vi, DP_SST_1);

	return 0;
}

static ssize_t sysfs_store_bist_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct exynos_drm_dp *drm_dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *dp = drm_dp->subdev;
	u8 used = dp->bist_use;
	int rc;

	rc = kstrtou32(buf, 0, &dp->bist_use);
	if (rc < 0)
		return rc;

	/* already disabled */
	if (!used && !dp->bist_use)
		return len;

	exynos_drm_dp_hpd_en(dp);
	if (dp->hpd_state == HPD_UNPLUG) {
		dp_log_err(dp->dev, "hpd state unplug\n");
		return -EACCES;
	}

	/* Disable used sst setting */
	if (used)
		exynos_drm_dp_stream_disable(dp, DP_SST_1);

	dp->bist_type = dp->bist_use >> DP_SST_4;

	if (dp->bist_use) {
		if (bist_mode_set_config(drm_dp) < 0)
			return -EINVAL;

		exynos_drm_dp_stream_enable(dp, DP_SST_1);
	}

	dp_log_info(dp->dev, "DP_SST_1 %s w/ bist_type(%#x)\n",
			dp->bist_use ? "on" : "off", dp->bist_type);
	return len;
}

static DEVICE_ATTR(bist_mode, 0644,
		sysfs_show_bist_mode, sysfs_store_bist_mode);

/*** Exposed functions for exynos_drm_dp ***/
static int exynos_drm_dp_subdev_probe(u32 id, struct device *dev,
		struct drm_device *drm_dev, struct exynos_dp_subdev **subdev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_dp_subdev *dp = NULL;
	const struct dp_dev_data *match;
	int ret = 0;

	dp = devm_kzalloc(dev, sizeof(struct exynos_dp_subdev), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	match = of_device_get_match_data(dev);
	if (!match) {
		dp_log_err(dev, "Failed to get match data\n");
		ret = -ENODEV;
		goto err_devm_alloc;
	}
	dp->version = match->version;

	dp->dev = &pdev->dev;
	dp->drm_dev = drm_dev;
	*subdev = dp;
	dp->id = id;
	dp->state = DP_STATE_OFF;
	dp->training_state = false;

	ret = exynos_drm_dp_init_resources(dp, pdev);
	if (ret)
		goto err_devm_alloc;

	dp->phy = devm_phy_get(dp->dev, "dp_phy");
	if (IS_ERR(dp->phy)) {
		dp_log_err(dev, "failed to get dp phy\n");
		dp->phy = NULL;
		ret = -EPROBE_DEFER;
		goto err_devm_alloc;
	}

	spin_lock_init(&dp->slock);

	dp->hpd_gpio = of_get_named_gpio(dev->of_node, "samsung,hpd-gpio", 0);

	if (gpio_is_valid(dp->hpd_gpio)) {
		ret = devm_gpio_request_one(dp->dev, dp->hpd_gpio, GPIOF_IN, "hpd_gpio");
		if (ret) {
			dp_log_err(dev, "failed to get HPD_GPIO\n");
			goto err_devm_alloc;
		}

		dp->hpd_gpio_irq = gpio_to_irq(dp->hpd_gpio);

		dp_log_dbg(dev, "GPIO_HPD has been registered. GPIO(%d), irq(%d)\n",
				dp->hpd_gpio, dp->hpd_gpio_irq);

		irq_set_status_flags(dp->hpd_gpio_irq, IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dp->dev, dp->hpd_gpio_irq,
					NULL, exynos_drm_dp_hpd_gpio_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(dp->dev), dp);
		if (ret) {
			dp_log_err(dev, "failed to request dp hpd_gpio_irq\n");
			goto err_devm_alloc;
		}
	}

	dp->irq = platform_get_irq(pdev, 0);
	if (dp->irq < 0) {
		dp_log_err(dev, "failed to request dp irq resource\n");
		ret = dp->irq;
		goto err_devm_alloc;
	}

	irq_set_status_flags(dp->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dp->dev, dp->irq, NULL,
					exynos_drm_dp_irq, IRQF_ONESHOT,
					dev_name(dp->dev), dp);
	if (ret) {
		dp_log_err(dev, "failed to request dp irq\n");
		goto err_devm_alloc;
	}

	dp->dp_wq = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!dp->dp_wq) {
		dp_log_err(dev, "create dp_wq failed.\n");
		goto err_devm_alloc;
	}

	device_create_file(dev, &dev_attr_dump_sfr);
	device_create_file(dev, &dev_attr_link_lane);
	device_create_file(dev, &dev_attr_bist_mode);
	device_create_file(dev, &dev_attr_debug_hpd_irq);
	device_create_file(dev, &dev_attr_debug_lt);
	dp->lt_info.max_link_lane = MAX_LANE_CNT;
	dp->dp_debug.debug_hpd_irq = 0;
	dp->dp_debug.debug_lt = DEBUG_LT_NORMAL;

	INIT_DELAYED_WORK(&dp->hpd_plug_work, exynos_drm_dp_hpd_plug_work);
	INIT_DELAYED_WORK(&dp->hpd_unplug_work, exynos_drm_dp_hpd_unplug_work);
	INIT_DELAYED_WORK(&dp->hpd_irq_work, exynos_drm_dp_hpd_irq_work);
	INIT_LIST_HEAD(&dp->mst_list);
	mutex_init(&dp->lock);
	mutex_init(&dp->pwlock);

	dp_log_info(dev, "is successfully\n");
	return ret;

err_devm_alloc:
	dp_log_err(dev, "Failed to alloc err(%d)\n", ret);
	devm_kfree(dev, dp);
	return ret;
}

/* H/W goes to stop or reset state when OS restarting during h/w operation */
static void exynos_drm_dp_reset(struct exynos_dp_subdev *dp)
{
	int i;

	for (i = DP_SST_1; i <= DP_SST_MAX; i++)
		dp_reg_stop(dp->id, i);
}

static void exynos_drm_dp_mst_pre_disable(struct exynos_dp_subdev *dp,
		dp_sst_idx_t sst_idx)
{
	struct device *dev = dp->dev;
	struct dp_mst_config *cfg, *tmp_cfg;
	u32 start_slot = MST_SLOT_INIT_NUM, del_num_slot;
	bool find = false;
	bool hpd_en = (dp->hpd_state == HPD_UNPLUG) ? false : true;

	if (list_empty(&dp->mst_list))
		return;

	/* 1. find list that has sst_idx and rewrite for new timeslot */
	list_for_each_entry_safe(cfg, tmp_cfg, &dp->mst_list, list) {
		if (cfg->sst_id == sst_idx) {
			del_num_slot = cfg->num_slots;
			list_del(&cfg->list);
			kfree(cfg);
			find = true;
			continue;
		}
		if (find && hpd_en)
			dp_reg_set_vcpi_timeslot(dp->id, cfg->sst_id,
						start_slot, cfg->num_slots);
		start_slot += cfg->num_slots;
	}

	if (!hpd_en)
		return;

	/* 2. delete of vc timeslot */
	if (find)
		dp_reg_set_vcpi_timeslot(dp->id, 0, start_slot, del_num_slot);
	else
		dp_log_err(dev, "failed to delete payload of SST%d\n", sst_idx);

	/* 3. wait for generating ACT sequence */
	dp_reg_wait_for_vcpi_update(dp->id);
}

static void exynos_drm_dp_dt_to_dp_video_info(struct exynos_dp_video_info *vi,
		struct dp_video_info *dp_video_info)
{
	struct videomode *vm = &vi->vm;

	dp_video_info->vm.pixelclock = vm->pixelclock;
	dp_video_info->vm.hfront_porch = vm->hfront_porch;
	dp_video_info->vm.hactive = vm->hactive;
	dp_video_info->vm.hback_porch = vm->hback_porch;
	dp_video_info->vm.hsync_len = vm->hsync_len;

	dp_video_info->vm.vfront_porch = vm->vfront_porch;
	dp_video_info->vm.vactive = vm->vactive;
	dp_video_info->vm.vback_porch = vm->vback_porch;
	dp_video_info->vm.vsync_len = vm->vsync_len;

	dp_video_info->vm.hsync_pol = vi->hsync_pol;
	dp_video_info->vm.vsync_pol = vi->vsync_pol;

	dp_video_info->bpc = vi->bpc;
	dp_video_info->sst_id = vi->sst_id;
	dp_video_info->dyn_range = vi->dyn_range;
}

/* DSC */
static void exynos_drm_dp_dsc_set(struct exynos_dp_subdev *dp,
		struct exynos_dp_video_info *vi,
		struct dp_video_info *dp_vi)
{
	/* DSC */
	if (!drm_dp_sink_supports_dsc(dp->dsc_dpcd)) {
		dp_vi->dsc.enable = false;
		return;
	}

	dp_vi->dsc.enable = vi->dsc.enable;
	if (vi->dsc.enable) {
		dp_vi->dsc.slice_count = vi->dsc.slice_count;
		dp_vi->dsc.chunk_size = vi->dsc.chunk_size;
		memcpy(dp_vi->dsc.pps, &vi->dsc.pps, sizeof(dp_vi->dsc.pps));
		drm_dp_dpcd_writeb(&dp->aux, DP_DSC_ENABLE,
				DP_DECOMPRESSION_EN);
	}
}

static void exynos_drm_dp_dsc_unset(struct exynos_dp_subdev *dp,
		struct exynos_dp_video_info *vi)
{
	vi->dsc.enable = false;
	/* consider DSC mode switching without HPD unplug */
	if (drm_dp_sink_supports_dsc(dp->dsc_dpcd))
		drm_dp_dpcd_writeb(&dp->aux, DP_DSC_ENABLE, 0);
}

void exynos_drm_dp_stream_enable(struct exynos_dp_subdev *dp, dp_sst_idx_t sst_idx)
{
	struct device *dev = dp->dev;
	struct exynos_dp_video_info *vi;
	struct dp_video_info dp_video_info;

	if (!dp->hpd_state)
		dp_log_err(dev, "HPD is low(%d)\n", dp->hpd_state);

	/* Set default sst_idx */
	if (sst_idx == DP_SST_UNKNOWN) {
		dp_log_err(dp->dev, "sst idx is unknown\n");
		return;
	}

	vi = &dp->vi[sst_idx - 1];

	exynos_drm_dp_dt_to_dp_video_info(vi, &dp_video_info);

	dp_log_info(dev, "SST%d cur_video = %dx%d-%dHz(%dbpc) in displayport_enable!!!\n",
			vi->sst_id + 1, vi->vm.hactive, vi->vm.vactive, vi->vrefresh, vi->bpc);

	exynos_drm_dp_dsc_set(dp, vi, &dp_video_info);

	if (dp->bist_use)
		dp_reg_set_bist_video_config(dp->id, dp_video_info, dp->bist_type);
	else
		dp_reg_set_video_config(dp->id, dp_video_info);

	dp_reg_start(dp->id, vi->sst_id);

	exynos_drm_dp_set_normal_data(dp);
}

void exynos_drm_dp_stream_disable(struct exynos_dp_subdev *dp, dp_sst_idx_t sst_idx)
{
	struct device *dev = dp->dev;
	struct exynos_dp_video_info *vi;

	dp_log_info(dev, "+, state: %d, SST%d\n", dp->state, sst_idx);

	/* Set default sst_idx */
	if (sst_idx == DP_SST_UNKNOWN) {
		dp_log_err(dp->dev, "sst idx is unknown\n");
		return;
	}

	vi = &dp->vi[sst_idx - 1];

	exynos_drm_dp_mst_pre_disable(dp, sst_idx);

	dp_reg_stop(dp->id, vi->sst_id);
	exynos_drm_dp_dsc_unset(dp, vi);

	dp_log_info(dev, "-, state: %d, SST%d\n", dp->state, sst_idx);
}

/**
 * @cnotice
 * @prdcode
 * @unit_name{Exynos_dp_drv}
 * @purpose Trigering of DP work
 * @logic Operation the DP according to the DP and HPD status
 * @params
 * @param{in, dp, struct* ::exynos_dp_subdev, None}
 * @endparam
 * @retval{dp->hpd_state, bool, None, [false:true], None}
 */
bool exynos_drm_dp_is_hpd_connected(struct exynos_dp_subdev *dp)
{
	int old_hpd = dp->hpd_state;

	mutex_lock(&dp->lock);
	if (gpio_get_value(dp->hpd_gpio)) {
		mutex_unlock(&dp->lock);
		if (dp->state == DP_STATE_LINKED)
			exynos_drm_dp_hpd_en(dp);
	} else {
		mutex_unlock(&dp->lock);
		dp->hpd_state = HPD_UNPLUG;
	}

	if (old_hpd != dp->hpd_state)
		dp_log_info(dp->dev, "DP%d hpd = %d\n", dp->id, dp->hpd_state);

	return dp->hpd_state & HPD_PLUG;
}

void exynos_drm_dp_videomode_set(struct exynos_dp_subdev *dp,
		struct exynos_dp_video_info *vi, dp_sst_idx_t sst_idx)
{
	struct exynos_dp_video_info *cur_vi;

	/* Set default sst_idx */
	if (sst_idx == DP_SST_UNKNOWN) {
		dp_log_err(dp->dev, "sst idx is unknown\n");
		return;
	}

	cur_vi = &dp->vi[sst_idx - 1];

	cur_vi->sst_id = sst_idx - 1;
	cur_vi->dyn_range = vi->dyn_range;
	cur_vi->bpc = vi->bpc;
	cur_vi->vrefresh = vi->vrefresh;
	cur_vi->hsync_pol = vi->hsync_pol;
	cur_vi->vsync_pol = vi->vsync_pol;
	memcpy(&cur_vi->vm, &vi->vm, sizeof(struct videomode));

	/* DSC */
	cur_vi->dsc.enable = vi->dsc.enable;
	if (vi->dsc.enable) {
		cur_vi->dsc.slice_count = vi->dsc.slice_count;
		cur_vi->dsc.chunk_size = vi->dsc.chunk_size;
		memcpy(&cur_vi->dsc.pps, &vi->dsc.pps, sizeof(vi->dsc.pps));
	}
}

static int exynos_drm_dp_start(struct exynos_dp_subdev *dp)
{
	struct device *dev = dp->dev;
	int ret = 0;

	dp_log_info(dev, "+, state: %d\n", dp->state);
	if (gpio_get_value(dp->hpd_gpio) == HPD_UNPLUG) {
		dp_log_err(dev, "DP(%d) is unplug\n", dp->id);
		return 0;
	}

	if (dp->state == DP_STATE_LINKED) {
		dp_log_info(dev, "DP(%d), state : %d\n", dp->id, dp->state);
		return 0;
	}
	mutex_lock(&dp->pwlock);

	ret = phy_power_on(dp->phy);
	if (ret < 0) {
		dp_log_err(dev, "cannot enable DP_PHY[%d], %d\n", dp->id, ret);
		mutex_unlock(&dp->pwlock);
		return ret;
	}

	dp_reg_sw_reset(dp->id);
	dp_reg_init(dp->id);

	dp_reg_set_hpd_interrupt(dp->id, 1);

	dp->training_state = false;

	enable_irq(dp->irq);
	disable_irq(dp->hpd_gpio_irq);

	dp->state = DP_STATE_LINKED;

	mutex_unlock(&dp->pwlock);

	dp_log_info(dev, "-, state: %d\n", dp->state);

	return ret;
}

/*
 * Functions below are from exynos_dp_drv.c.
 * It would be refactored more later.
 */
uint32_t exynos_drm_dp_find_possible_crtc(struct exynos_drm_dp *dp, int sst_idx)
{
	int possible_crtcs = 0;
	struct device_node *dp_node = dp->dev->of_node;
	struct device_node *endp, *remote_port;

	endp = of_graph_get_endpoint_by_regs(dp_node, 0, sst_idx);
	if (endp) {
		remote_port = of_graph_get_remote_port(endp);
		possible_crtcs = drm_of_crtc_port_mask(dp->drm_dev, remote_port);
		of_node_put(remote_port);
	}

	of_node_put(endp);

	return possible_crtcs;
}

dp_sst_idx_t exynos_drm_dp_get_sst_idx(struct drm_encoder *encoder)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct drm_crtc *crtc = encoder->crtc;
	struct device_node *dp_node = dp->dev->of_node;
	struct device_node *endp, *crtc_node;
	struct of_endpoint of_endpoint;

	for_each_endpoint_of_node(dp_node, endp) {
		crtc_node = of_graph_get_remote_port(endp);
		if (crtc->port == crtc_node) {
			of_node_put(crtc_node);
			goto out;
		}
		of_node_put(crtc_node);
	}
out:
	of_graph_parse_endpoint(endp, &of_endpoint);
	of_node_put(endp);
	return of_endpoint.id;
}

/* DSC */
static bool exynos_drm_dp_supports_dsc(struct exynos_drm_dp *dp)
{
	struct exynos_dp_subdev *subdev = dp->subdev;

	/* This version returns true regardless of FEC capability */
	return drm_dp_sink_supports_dsc(subdev->dsc_dpcd);
}

#define MAX_SOURCE_SLICE_COUNT		4
#define INVALID_SOURCE_SLICE_COUNT	3
static u8 exynos_drm_dsc_get_slice_count(struct exynos_drm_dp *dp,
		struct drm_dsc_config *dsc_cfg)
{
	struct exynos_dp_subdev *subdev = dp->subdev;
	u32 max_slice_width;
	u8 max_slice_count, source_slice_count;

	max_slice_width = drm_dp_dsc_sink_max_slice_width(subdev->dsc_dpcd);
	if (max_slice_width < DP_DSC_MIN_SLICE_WIDTH_VALUE) {
		dp_log_info(dp->dev, "max_slice_width %d is out of range\n",
				max_slice_width);
		return 0;
	}

	max_slice_count =
		drm_dp_dsc_sink_max_slice_count(subdev->dsc_dpcd, false);


	/* FIXME: divide by 2 to use 4-slice in UHD picture */
	source_slice_count = min_t(u8, MAX_SOURCE_SLICE_COUNT,
				DIV_ROUND_UP(dsc_cfg->pic_width,
					     max_slice_width / 2));

	if (source_slice_count == INVALID_SOURCE_SLICE_COUNT)
		source_slice_count = MAX_SOURCE_SLICE_COUNT;

	dp_log_kms(dp->dev, "max_slice_width %d count %d, src_slice_count %d\n",
			max_slice_width, max_slice_count, source_slice_count);

	return min_t(u8, source_slice_count, max_slice_count);
}

#define MIN_SOURCE_SLICE_HEIGHT	8
static u16 exynos_drm_dsc_get_slice_height(struct exynos_drm_dp *dp,
		struct drm_dsc_config *dsc_cfg)
{
	struct device *dev = dp->dev;
	u16 slice_height;

	if (dsc_cfg->pic_height <= 0) {
		dp_log_info(dev, "invalid pic_height\n");
		return 0;
	}

	/*
	 * Guideline for slice_height
	 * must be an integral divisor of picture height
	 * slice_height * slice_width >=  15,000 pixels
	 * slice_height * slice_width <= 100,000 pixels
	 */
	if (dsc_cfg->pic_height % 16 == 0)
		slice_height = dsc_cfg->pic_height / 16;
	if (dsc_cfg->pic_height % 10 == 0)
		slice_height = dsc_cfg->pic_height / 10;
	else
		slice_height = dsc_cfg->pic_height / 4;

	return max_t(u16, slice_height, MIN_SOURCE_SLICE_HEIGHT);
}

static int exynos_drm_dp_dsc_compute(struct drm_encoder *encoder,
		struct exynos_dp_video_info *vi,
		dp_sst_idx_t sst_idx)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct exynos_dp_subdev *subdev = dp->subdev;
	u8 *dsc_dpcd = subdev->dsc_dpcd;
	struct drm_dsc_picture_parameter_set *pps;
	struct drm_dsc_config dsc_cfg;

	/* Initialize */
	memset(&dsc_cfg, 0x0, sizeof(dsc_cfg));

	dsc_cfg.dsc_version_major =
		(dsc_dpcd[DP_DSC_REV - DP_DSC_SUPPORT] &
		 DP_DSC_MAJOR_MASK) >> DP_DSC_MAJOR_SHIFT;
	dsc_cfg.dsc_version_minor =
		(dsc_dpcd[DP_DSC_REV - DP_DSC_SUPPORT] &
		 DP_DSC_MINOR_MASK) >> DP_DSC_MINOR_SHIFT;

	dsc_cfg.line_buf_depth =
		drm_dp_dsc_sink_line_buf_depth(dsc_dpcd);
	dsc_cfg.convert_rgb =
		dsc_dpcd[DP_DSC_DEC_COLOR_FORMAT_CAP - DP_DSC_SUPPORT]
			& DP_DSC_RGB;
	dsc_cfg.simple_422 =
		dsc_dpcd[DP_DSC_DEC_COLOR_FORMAT_CAP - DP_DSC_SUPPORT]
			& DP_DSC_YCbCr422_Simple;
	dsc_cfg.native_420 =
		dsc_dpcd[DP_DSC_DEC_COLOR_FORMAT_CAP - DP_DSC_SUPPORT]
			& DP_DSC_YCbCr420_Native;
	dsc_cfg.native_422 =
		dsc_dpcd[DP_DSC_DEC_COLOR_FORMAT_CAP - DP_DSC_SUPPORT]
			& DP_DSC_YCbCr422_Native;
	dsc_cfg.block_pred_enable =
		dsc_dpcd[DP_DSC_BLK_PREDICTION_SUPPORT - DP_DSC_SUPPORT]
			& DP_DSC_BLK_PREDICTION_IS_SUPPORTED;

	dsc_cfg.vbr_enable = false;
	dsc_cfg.pic_width = vi->vm.hactive;
	dsc_cfg.pic_height = vi->vm.vactive;
	dsc_cfg.bits_per_component = vi->bpc;

	/* TODO: This version only covers case of "BPC == BPP/3". */
	/* Compressed BPP format is multiply by 16 */
	dsc_cfg.bits_per_pixel = vi->bpc << 4;

	dsc_cfg.slice_count = exynos_drm_dsc_get_slice_count(dp, &dsc_cfg);
	if (dsc_cfg.slice_count == 0) {
		dp_log_info(dp->dev, "slice_count is invalid\n");
		return -EINVAL;
	}

	dsc_cfg.slice_height = exynos_drm_dsc_get_slice_height(dp, &dsc_cfg);

	dsc_cfg.slice_width =
		DIV_ROUND_UP(dsc_cfg.pic_width, dsc_cfg.slice_count);

	vi->dsc.slice_count = dsc_cfg.slice_count;
	vi->dsc.chunk_size = dsc_cfg.slice_chunk_size;

	pps = &vi->dsc.pps;
	drm_dsc_pps_payload_pack(pps, &dsc_cfg);

	return 0;
}

bool exynos_drm_dp_dsc_enable(struct drm_encoder *encoder,
		struct exynos_dp_video_info *vi, dp_sst_idx_t sst_idx)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);

	if (!vi)
		return false;

	if (!exynos_drm_dp_supports_dsc(dp))
		return false;

	if (exynos_drm_dp_dsc_compute(encoder, vi, sst_idx) < 0)
		return false;

	return true;
}

void exynos_drm_dp_dsc_disable(struct drm_encoder *encoder,  dp_sst_idx_t sst_idx)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);

	/* Set default sst_idx */
	if (sst_idx == DP_SST_UNKNOWN) {
		dp_log_err(dp->dev, "sst idx is unknown\n");
		return;
	}
}

static void exynos_drm_dp_enable(struct drm_encoder *encoder)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct exynos_dp_subdev *subdev = dp->subdev;
	struct device *dev = dp->dev;
	dp_sst_idx_t sst_idx;

	dp_log_kms(dev, "exynos_drm_dp_enable\n");

	if (subdev->state == DP_STATE_ON) {
		dp_log_info(dev, "DP(%d), state : %d\n", dp->id, subdev->state);
		return;
	}

	exynos_drm_dp_hpd_en(subdev);

	sst_idx = exynos_drm_dp_get_sst_idx(encoder);

	exynos_drm_dp_stream_enable(subdev, sst_idx);

	subdev->state = DP_STATE_ON;
}

static void exynos_drm_dp_disable(struct drm_encoder *encoder)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct exynos_dp_subdev *subdev = dp->subdev;
	struct device *dev = dp->dev;
	dp_sst_idx_t sst_idx;

	dp_log_kms(dev, "\n");

	mutex_lock(&subdev->pwlock);

	if (subdev->state == DP_STATE_LINKED) {
		dp_log_info(dev, "DP(%d), state : ON\n", dp->id);
		mutex_unlock(&subdev->pwlock);
		return;
	}

	sst_idx = exynos_drm_dp_get_sst_idx(encoder);
	/* DSC */
	exynos_drm_dp_dsc_disable(encoder, sst_idx);
	exynos_drm_dp_stream_disable(subdev, sst_idx);

	subdev->state = DP_STATE_LINKED;
	mutex_unlock(&subdev->pwlock);
}

void exynos_drm_dp_to_videoinfo(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct exynos_dp_video_info *vi)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct device *dev = dp->dev;
	struct dp_encoder *dp_encoder = to_encoder(encoder);
	struct drm_display_info *dp_info =
				&dp_encoder->dp_connector->base.display_info;

	memset(vi, 0x0, sizeof(*vi));

	drm_display_mode_to_videomode(mode, &vi->vm);
	vi->vrefresh = drm_mode_vrefresh(mode);

	/*
	 * The polarity of DP is set to the inverted value of DRM.
	 * DRM:	DRM_MODE_FLAG_PHSYNC -> DISPLAY_FLAGS_HSYNC_HIGH
	 *	DRM_MODE_FLAG_PVSYNC -> DISPLAY_FLAGS_VSYNC_HIGH
	 * DisplayPort: H/VSyncPolarity in HSP, VSP
	 * 	0 = Active high pulse.
	 * 	1 = Active low pulse.
	 */
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		vi->hsync_pol = false;
	else
		vi->hsync_pol = true;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vi->vsync_pol = false;
	else
		vi->vsync_pol = true;

	if (!dp_info->bpc)
		dp_info->bpc = DEFAULT_BPC;

	vi->bpc = dp_info->bpc;

	dp_log_kms(dev, "%s, vrefresh(%d) pclock(%d khz) \
			hsync_pol(%d) vsync_pol(%d) bpc(%d)\n",
			mode->name, vi->vrefresh, mode->clock,
			vi->hsync_pol, vi->vsync_pol, vi->bpc);
}

static void exynos_drm_dp_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct exynos_dp_subdev *subdev = dp->subdev;
	struct exynos_dp_video_info vi;
	struct device *dev = dp->dev;
	dp_sst_idx_t sst_idx;

	sst_idx = exynos_drm_dp_get_sst_idx(encoder);
	exynos_drm_dp_to_videoinfo(encoder, adjusted_mode, &vi);
	/* DSC */
	vi.dsc.enable = exynos_drm_dp_dsc_enable(encoder, &vi, sst_idx);
	exynos_drm_dp_videomode_set(subdev, &vi, sst_idx);

	drm_mode_copy(&dp->cur_mode, adjusted_mode);
	dp_log_info(dev, "SST%u %s = %s-%dHz dsc %s\n",	sst_idx,
			encoder->name, mode->name, drm_mode_vrefresh(mode),
			vi.dsc.enable ? "compression" : "bypass");
}

static bool exynos_drm_dp_connector_set_link_status(struct drm_device *dev,
		struct exynos_drm_dp *dp)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;
	struct exynos_dp_subdev *subdev = dp->subdev;
	struct exynos_drm_dp *drm_dp;
	struct dp_connector *dp_connector;
	enum drm_link_status set_link_status = DRM_LINK_STATUS_GOOD;
	enum drm_link_status old_link_status;
	int max_link_rate = 0;
	int root_id = dp->connector->base.id;
	bool changed = false;

	if (subdev->dpcd[DP_MAX_LINK_RATE])
		max_link_rate =	subdev->dpcd[DP_MAX_LINK_RATE];

	if ((subdev->hpd_state) &&
		((max_link_rate != subdev->lt_info.link_rate) || !subdev->training_state))
		set_link_status = DRM_LINK_STATUS_BAD;

	drm_connector_list_iter_begin(dev, &conn_iter);

	drm_for_each_connector_iter(connector, &conn_iter) {
		if (connector->connector_type != DRM_MODE_CONNECTOR_DisplayPort)
			continue;

		if (IS_ERR_OR_NULL(connector->state))
			continue;

		drm_dp = connector_to_dp(connector);
		if (drm_dp->output_type != dp->output_type)
			continue;

		/* This is case of DP UNPLUG */
		if (set_link_status == DRM_LINK_STATUS_GOOD)
			goto CHECK_LINK_STATUS;

		/* pdt = peer-device-type
		 * link_status is controled sink device in mst mode.
		 * pdt = 0 (no device connected
		 * ptd = 1 (Source device or SST branch device to UFP)
		 * pdt = 2 (Device with MST branch or SST branch device to DFP)
		 * pdt = 3 (SST Sink device or stream sink in MST)
		 * pdt = 4 (DP-to-Legacy converter)
		 */
		dp_connector = to_connector(connector);
		if (dp->is_mst && (!dp_connector->port ||
					(dp_connector->port->pdt < DP_PEER_DEVICE_SST_SINK)))
			continue;

		if (!dp->is_mst && (connector->base.id != root_id))
			continue;

CHECK_LINK_STATUS:
		old_link_status = connector->state->link_status;

		if (set_link_status != old_link_status) {
			dp_log_kms(dp->dev, "[CONNECTOR:%d:%s] link_status changed(%d) -> (%d)\n",
					connector->base.id, connector->name,
					old_link_status, set_link_status);
			connector->state->link_status = set_link_status;
			changed = true;
		}
	}

	drm_connector_list_iter_end(&conn_iter);

	return changed;
}

/*** connector functions ***/
static enum drm_connector_status
exynos_drm_dp_connector_detect(struct drm_connector *connector, bool force)
{
	enum drm_connector_status status = connector_status_disconnected;
	enum drm_connector_status old_status = connector->status;
	struct exynos_drm_dp *dp = connector_to_dp(connector);
	struct drm_device *drm_dev = dp->drm_dev;
	struct device *dev = dp->dev;
	bool link_status_changed = false;

	dp_log_kms(dev, "\n");

	if (exynos_drm_dp_is_hpd_connected(dp->subdev))
		status = connector_status_connected;

	/* In case of MST, SST connector should be disconnected */
	if (dp->is_mst)
		status = connector_status_disconnected;

	link_status_changed =
		exynos_drm_dp_connector_set_link_status(drm_dev, dp);

	if ((status == old_status) && link_status_changed)
		drm_kms_helper_hotplug_event(dp->drm_dev);

	dp_log_kms(dev, "status is %s\n",
			status == connector_status_connected ? \
			"connected" : "disconnected");
	return status;
};

/* If native only is true, only native mode is used */
#define for_each_dt_timings(num_timings, native_only, native_mode, __i) \
	for ((__i) = (native_only) ? (native_mode) : 0; 		\
	     (__i) < (num_timings) && 					\
		     (!(native_only) || (__i) == (native_mode));	\
	     (__i)++)	\

static int exynos_drm_dp_add_mode_from_dt(struct drm_connector *connector,
					  struct display_timings *timings)
{
	struct dp_connector *dp_connector = to_connector(connector);
	struct drm_device *drm_dev = connector->dev;
	struct videomode vm;
	struct drm_display_mode *mode;
	int native_mode;
	bool native_only;
	int mode_num = 0;
	int index;

	if (!timings)
		return mode_num;

	/* Check a out of timings->num_timings */
	native_mode = dp_connector->native_mode;
	if (native_mode < 0 || native_mode >= timings->num_timings)
		native_mode = timings->native_mode;

	native_only = dp_connector->native_only;

	for_each_dt_timings(timings->num_timings, native_only, native_mode, index) {
		if (videomode_from_timings(timings, &vm, index)) {
			dp_log_err(drm_dev->dev, "index(%d) is invalid\n",
					index);
			continue;
		}

		mode = drm_mode_create(drm_dev);
		if (!mode) {
			dp_log_err(drm_dev->dev, "failed to add mode %ux%u\n",
					vm.hactive, vm.vactive);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type = DRM_MODE_TYPE_DRIVER;

		if (index == native_mode)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		dp_log_dbg(drm_dev->dev, "%s, vrefresh(%d), pclock(%d khz)\n",
				mode->name, drm_mode_vrefresh(mode), mode->clock);

		drm_mode_probed_add(connector, mode);
		mode_num++;
	}

	return mode_num;
}

int exynos_drm_dp_add_timings(struct exynos_drm_dp *dp,
		struct drm_connector *connector)
{
	struct display_timings *timings = dp->timings;
	struct device *dev = dp->dev;
	int num_modes;

	num_modes = exynos_drm_dp_add_mode_from_dt(connector, timings);
	if (!num_modes)
		dp_log_dbg(dev, "failed to get modes from device tree");

	return num_modes;
}

static int exynos_drm_dp_get_modes(struct drm_connector *connector)
{
	struct exynos_drm_dp *dp = connector_to_dp(connector);
	struct exynos_dp_subdev *subdev = dp->subdev;
	struct device *dev = dp->dev;
	struct edid *edid = NULL;
	int num_modes = 0;
	u8 support_edid;

	num_modes = exynos_drm_dp_add_timings(dp, connector);
	if (num_modes)
		goto ret;

	drm_dp_dpcd_readb(&subdev->aux, DP_RECEIVE_PORT_0_CAP_0, &support_edid);

	if (support_edid & DP_LOCAL_EDID_PRESENT)
		edid = drm_get_edid(connector, &subdev->aux.ddc);

	if (edid) {
		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

ret:
	dp_log_kms(dev, "num_modes %d\n", num_modes);
	return num_modes;
}
static enum drm_mode_status exynos_drm_dp_mode_valid(struct drm_connector *connector,
			struct drm_display_mode *mode)
{
	struct exynos_drm_dp *dp = connector_to_dp(connector);
	struct device *dev = dp->dev;

	dp_log_kms(dev, "hdisplay=%d, vdisplay=%d, vrefresh=%d, clock=%d\n",
		mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode),
		mode->clock * 1000);

	/* TODO: return MODE_BAD if exynos_dp_drv can't support it */

	return MODE_OK;
}

static void exynos_drm_dp_destroy_encoder(struct drm_encoder *encoder)
{
	struct exynos_drm_dp *dp = encoder_to_dp(encoder);
	struct dp_encoder *dp_encoder = to_encoder(encoder);
	struct device *dev = dp->dev;

	dp_log_kms(dev, "encoder %d\n", encoder->base.id);
	drm_encoder_cleanup(encoder);

	kfree(dp_encoder);
}

static void exynos_drm_dp_destroy_connector(struct drm_connector *connector)
{
	struct dp_connector *dp_connector = to_connector(connector);
	struct exynos_drm_dp *dp = connector_to_dp(connector);
	struct device *dev = dp->dev;

	dp_log_kms(dev, "connector %d\n", connector->base.id);
	drm_connector_cleanup(connector);
	kfree(dp_connector);
}

static const struct drm_encoder_funcs exynos_drm_dp_encoder_funcs = {
	.destroy = exynos_drm_dp_destroy_encoder,
};

static const struct drm_encoder_helper_funcs
exynos_drm_dp_encoder_helper_funcs = {
	.enable = exynos_drm_dp_enable,
	.disable = exynos_drm_dp_disable,
	.mode_fixup = NULL,
	.mode_set = exynos_drm_dp_mode_set,
};

static const struct drm_connector_funcs exynos_drm_dp_connector_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.detect = exynos_drm_dp_connector_detect,
	.destroy = exynos_drm_dp_destroy_connector,
	.dpms = drm_helper_connector_dpms,
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
};

static const struct drm_connector_helper_funcs
exynos_drm_dp_connector_helper_funcs = {
	.get_modes = exynos_drm_dp_get_modes,
	.mode_valid = exynos_drm_dp_mode_valid,
};

static struct dp_encoder *dp_encoder_alloc(struct exynos_drm_dp *dp)
{
	struct dp_encoder *dp_encoder;

	dp_encoder = kzalloc(sizeof(*dp_encoder), GFP_KERNEL);
	if (!dp_encoder)
		return NULL;

	dp_encoder->dp = dp;
	dp_encoder->output_type = dp->output_type;
	dp_encoder->sst_idx = DP_SST_UNKNOWN;

	return dp_encoder;
}

static struct dp_connector *dp_connector_alloc(struct exynos_drm_dp *dp,
					       struct dp_encoder *dp_encoder)
{
	struct dp_connector *dp_connector;

	dp_connector = kzalloc(sizeof(*dp_connector), GFP_KERNEL);
	if (!dp_connector)
		return NULL;

	dp_connector->dp = dp;
	dp_connector->dp_encoder = dp_encoder;
	dp_encoder->dp_connector = dp_connector;

	return dp_connector;
}

static int exynos_drm_dp_init_encoder(struct dp_encoder *dp_encoder)
{
	struct exynos_drm_dp *dp = dp_encoder->dp;
	struct drm_device *drm_dev = dp->drm_dev;
	struct drm_encoder *encoder = &dp_encoder->base;
	dp_sst_idx_t sst_id;

	drm_encoder_init(drm_dev, encoder, &exynos_drm_dp_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, "exynos_dp%u", dp->id);

	drm_encoder_helper_add(encoder, &exynos_drm_dp_encoder_helper_funcs);

	for (sst_id = DP_SST_1; sst_id <= DP_SST_MAX; sst_id++)
		encoder->possible_crtcs |= exynos_drm_dp_find_possible_crtc(dp, sst_id);

	dp_log_info(dp->dev, "Success to create %s(possible_crtcs:%#x)\n",
		    encoder->name, encoder->possible_crtcs);

	return 0;
}

static int exynos_drm_dp_init_connector(struct dp_connector *dp_connector)
{
	struct exynos_drm_dp *dp = dp_connector->dp;
	struct drm_connector *connector = &dp_connector->base;
	struct drm_encoder *encoder = &dp_connector->dp_encoder->base;
	struct device *dev = dp->dev;
	int ret = 0;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(dp->drm_dev, connector,
				 &exynos_drm_dp_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		dp_log_err(dev, "Failed to create connector %d\n", ret);
		drm_connector_cleanup(connector);
		return ret;
	}

	drm_connector_helper_add(connector,
				 &exynos_drm_dp_connector_helper_funcs);
	drm_connector_attach_encoder(connector, encoder);

	dp_log_info(dev, "Success to create connector %s\n", connector->name);

	return 0;
}

static int exynos_drm_dp_get_timings(struct device *dev)
{
	struct exynos_drm_dp *dp = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	struct device_node *timing_np;

	timing_np = of_parse_phandle(np, "samsung,display-timings", 0);
	if (!timing_np) {
		dp_log_info(dev, "could not get display timings\n");
		return 0;
	}

	dp->timings = of_get_display_timings(timing_np);
	if (IS_ERR_OR_NULL(dp->timings)) {
		dp_log_err(dev, "could not get native display timing\n");
		of_node_put(timing_np);
		return -EINVAL;
	}
	of_node_put(timing_np);

	return 0;
}

static void exynos_drm_dp_get_native_mode(struct device *dev,
					  struct dp_connector *dp_connector)
{
	struct device_node *np = dev->of_node;
	int idx;

	if (of_property_read_s32(np, "samsung,native-mode,idx", &idx) < 0)
		dp_connector->native_mode = -1;
	else
		dp_connector->native_mode = idx;

	dp_connector->native_only =
		of_property_read_bool(np, "samsung,native-only");

	of_property_read_u32(np, "samsung,native-mode,bpc",
				&dp_connector->base.display_info.bpc);
}

static ssize_t exynos_drm_dp_aux_transfer(struct drm_dp_aux *aux,
		struct drm_dp_aux_msg *msg)
{
	struct exynos_dp_subdev *subdev = to_subdev(aux);
	struct device *dev = subdev->dev;
	u8 *buffer = msg->buffer;
	int transferred = 0;
	u32 id = subdev->id;

	dp_log_enter(dev);

	if (subdev->state == DP_STATE_OFF)
		return -ETIMEDOUT;

	if (subdev->hpd_state == HPD_UNPLUG)
		return -ETIMEDOUT;

	if (!dp_reg_get_hpd_status(subdev->id))
		return -ETIMEDOUT;

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_WRITE:
		transferred = dp_reg_dpcd_write_burst(id, msg->address,
				msg->size, buffer);
		break;
	case DP_AUX_NATIVE_READ:
		transferred = dp_reg_dpcd_read_burst(id, msg->address,
				msg->size, buffer);
		break;
	case DP_AUX_I2C_WRITE:
	case DP_AUX_I2C_WRITE_STATUS_UPDATE:
		return dp_reg_aux_write(id, I2C_WRITE, msg->address,
				msg->size, buffer);
	case DP_AUX_I2C_READ:
		return dp_reg_aux_read(id, I2C_READ, msg->address,
				msg->size, buffer);
	default:
		dp_log_err(dev, "invalid msg request(%#x)\n", msg->request);
		return -EINVAL;
	}

	return transferred > 0 ? transferred : -EBUSY;
}

static void exynos_drm_dp_subdev_detect(struct device *dev)
{
	struct exynos_drm_dp *dp = dev_get_drvdata(dev);
	struct drm_device *drm_dev = dp->drm_dev;

	dp_log_kms(dev, "\n");
	drm_helper_hpd_irq_event(drm_dev);
}

static int exynos_drm_dp_subdev_bind(struct device *dev,
		struct exynos_drm_dp *dp)
{
	struct exynos_dp_subdev *subdev = NULL;
	struct drm_device *drm_dev = dp->drm_dev;
	int ret = 0;

	ret = exynos_drm_dp_subdev_probe(dp->id, dev, drm_dev, &subdev);
	if (ret < 0)
		return ret;

	dp->subdev = subdev;
	subdev->aux.name = kasprintf(GFP_KERNEL, "%s%d-aux", DEV_NAME, dp->id);
	subdev->aux.transfer = exynos_drm_dp_aux_transfer;
	subdev->aux.dev = dev;
	subdev->aux.drm_dev = drm_dev;
	subdev->detect = exynos_drm_dp_subdev_detect;
	subdev->state = DP_STATE_OFF;

	ret = drm_dp_aux_register(&subdev->aux);
	if (ret) {
		dp_log_err(dev, "failed to aux_register, %d\n", ret);
		goto err_aux_register;
	}

	if (gpio_get_value(subdev->hpd_gpio))
		exynos_drm_dp_reset(subdev);

	dp_log_dbg(dev, "%s registered for aux_transfer\n", subdev->aux.name);

	return 0;

err_aux_register:
	return ret;
}

static int exynos_drm_dp_bind(struct device *dev,
		struct device *master, void *data)
{
	struct exynos_drm_dp *dp = dev_get_drvdata(dev);
	struct dp_encoder *dp_encoder;
	struct dp_connector *dp_connector;
	struct drm_device *drm_dev = data;
	int ret = 0;

	dp_log_enter(dev);
	dp->drm_dev = drm_dev;

	dp_encoder = dp_encoder_alloc(dp);
	if (IS_ERR_OR_NULL(dp_encoder))
		goto err_encoder_init;
	ret = exynos_drm_dp_init_encoder(dp_encoder);
	if (ret < 0)
		goto err_encoder_init;

	dp_connector = dp_connector_alloc(dp, dp_encoder);
	if (IS_ERR_OR_NULL(dp_connector))
		goto err_connector_init;
	ret = exynos_drm_dp_init_connector(dp_connector);
	if (ret < 0)
		goto err_connector_init;

	/* Set base encoder/connector */
	dp->encoder = &dp_encoder->base;
	dp->connector = &dp_connector->base;

	ret = exynos_drm_dp_get_timings(dev);
	if (ret < 0)
		goto err_connector_init;

	exynos_drm_dp_get_native_mode(dev, dp_connector);

	ret = exynos_drm_dp_subdev_bind(dev, dp);
	if (ret < 0)
		goto err_connector_init;

	exynos_drm_dp_mst_init(dp_connector);

	/* enable_irq end of binding */
	if (dp->subdev)
		enable_irq(dp->subdev->hpd_gpio_irq);
	
	ret = exynos_drm_dp_start(dp->subdev);
	if (ret < 0)
		dp_log_err(dp->dev, "cannot start DP[%d], %d\n", dp->id, ret);

	dp->is_mst = false;

	dp_log_exit(dev);

	return 0;

err_connector_init:
	if (dp_connector)
		exynos_drm_dp_destroy_connector(&dp_connector->base);
err_encoder_init:
	if (dp_encoder)
		exynos_drm_dp_destroy_encoder(&dp_encoder->base);
	dp_log_err(dev, "failed: drm_encoder/connector_init() : %d\n", ret);
	return ret;
}

static void exynos_drm_dp_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct exynos_drm_dp *dp = dev_get_drvdata(dev);
	struct exynos_dp_subdev *subdev = dp->subdev;

	dp_log_enter(dev);

	kfree(subdev->aux.name);

	destroy_workqueue(subdev->dp_wq);
	mutex_destroy(&subdev->lock);
	mutex_destroy(&subdev->pwlock);

	drm_dp_aux_unregister(&subdev->aux);

	display_timings_release(dp->timings);

	flush_delayed_work(&subdev->hpd_irq_work);

	disable_irq(subdev->irq);
	disable_irq(subdev->hpd_gpio_irq);


	dp_log_exit(dev);
	return;
}

static const struct component_ops exynos_drm_dp_ops = {
	.bind	= exynos_drm_dp_bind,
	.unbind	= exynos_drm_dp_unbind,
};

static int exynos_drm_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_drm_dp *dp;

	struct clk *aclk, *pclk;
	int ret = 0;

	dp = devm_kzalloc(dev, sizeof(struct exynos_drm_dp),
			  GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dev = dev;
	dp->bridge.of_node = dev->of_node;
	dp->bridge.driver_private = dp;
	drm_bridge_add(&dp->bridge);

	dp->id = 0;
	// TODO: make it proper value
	dp->output_type = EXYNOS_DISPLAY_TYPE_NONE;

	platform_set_drvdata(pdev, dp);

	aclk = devm_clk_get_enabled(dp->dev, "aclk");
	if (IS_ERR(aclk))
		return dev_err_probe(dp->dev, PTR_ERR(aclk),
				     "Could not get aclk clock\n");
	pclk = devm_clk_get_enabled(dp->dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dp->dev, PTR_ERR(pclk),
				     "Could not get pclk clock\n");

	dp_log_info(dev, "is successfully\n");

	ret = component_add(dev, &exynos_drm_dp_ops);
	if (ret)
		goto fail_probe;

	return 0;

fail_probe:
	dp_log_err(dev, "probe failed");
	devm_kfree(dev, dp);

	return ret;
}

static void
exynos_drm_dp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_drm_dp *dp = dev_get_drvdata(dev);

	device_remove_file(dev, &dev_attr_dump_sfr);
	device_remove_file(dev, &dev_attr_link_lane);
	device_remove_file(dev, &dev_attr_bist_mode);
	device_remove_file(dev, &dev_attr_debug_hpd_irq);
	device_remove_file(dev, &dev_attr_debug_lt);

	drm_bridge_remove(&dp->bridge);

	component_del(&pdev->dev, &exynos_drm_dp_ops);

}

static const struct dp_dev_data exynos910_dp = {
	.version = V910,
};

static const struct of_device_id exynos_drm_dp_match[] = {
	{ .compatible = "samsung,exynos910-dp", .data = (void *)&exynos910_dp },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_drm_dp_match);

struct platform_driver dp_driver = {
	.probe		= exynos_drm_dp_probe,
	.remove_new	= exynos_drm_dp_remove,
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = exynos_drm_dp_match,
	},
};

MODULE_DESCRIPTION("Samsung Specific DisplayPort Driver");
MODULE_LICENSE("GPL v2");
