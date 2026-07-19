// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Google Pixel 9 (tokay) TK4C MIPI-DSI panel
 *
 * The command sequence and display parameters are derived from Google's
 * downstream panel-gs-tk4c driver.  Only the normal 60/120 Hz modes and DCS
 * brightness are implemented here; LP/HBM and seamless switching remain out
 * of scope for this bring-up driver.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct tokay_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_connector *connector;
	struct gpio_desc *reset_gpio;
};

/* Production TK4C PPS configuration (DSC 1.2a, two 540-pixel slices). */
static const struct drm_dsc_config tokay_tk4c_dsc = {
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_width = 540,
	.slice_height = 101,
	.slice_count = 2,
	.simple_422 = false,
	.pic_width = 1080,
	.pic_height = 2424,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.initial_dec_delay = 526,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126
	},
	.rc_range_params = {
		{ 0, 4, 2 }, { 0, 4, 0 }, { 1, 5, 0 }, { 1, 6, 62 },
		{ 3, 7, 60 }, { 3, 7, 58 }, { 3, 7, 56 }, { 3, 8, 56 },
		{ 3, 9, 56 }, { 3, 10, 54 }, { 5, 11, 54 }, { 5, 12, 52 },
		{ 5, 13, 52 }, { 7, 13, 52 }, { 13, 15, 52 }
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 7,
	.scale_increment_interval = 2517,
	.nfl_bpg_offset = 246,
	.slice_bpg_offset = 258,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 540,
	.dsc_version_minor = 2,
	.dsc_version_major = 1,
};

static inline struct tokay_panel *to_tokay_panel(struct drm_panel *panel)
{
	return container_of(panel, struct tokay_panel, panel);
}

#define TOKAY_MODE(hz, flags)						\
	{								\
		.clock = (1080 + 32 + 12 + 16) *			\
			 (2424 + 8 + 2 + 16) * (hz) / 1000,		\
		.hdisplay = 1080,					\
		.hsync_start = 1080 + 32,				\
		.hsync_end = 1080 + 32 + 12,				\
		.htotal = 1080 + 32 + 12 + 16,				\
		.vdisplay = 2424,					\
		.vsync_start = 2424 + 8,				\
		.vsync_end = 2424 + 8 + 2,				\
		.vtotal = 2424 + 8 + 2 + 16,				\
		.width_mm = 65,						\
		.height_mm = 146,					\
		.type = DRM_MODE_TYPE_DRIVER | (flags),			\
	}

/* Keep 60 Hz preferred to match the bootloader-selected TK4C mode. */
static const struct drm_display_mode tokay_modes[] = {
	TOKAY_MODE(60, DRM_MODE_TYPE_PREFERRED),
	TOKAY_MODE(120, 0),
};

static int tokay_panel_unprepare(struct drm_panel *panel)
{
	return 0;
}

static int tokay_panel_prepare(struct drm_panel *panel)
{
	struct tokay_panel *ctx = to_tokay_panel(panel);

	/* Downstream TK4C reset timing: low 1 ms, high 1 ms. */
	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		usleep_range(1000, 1100);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(1000, 1100);
	}

	return 0;
}

static int tokay_panel_disable(struct drm_panel *panel)
{
	struct tokay_panel *ctx = to_tokay_panel(panel);

	mipi_dsi_dcs_set_display_off(ctx->dsi);
	msleep(120);
	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);

	return 0;
}

#define tk4c_cmd(dsi, seq...)						\
	mipi_dsi_dcs_write_buffer((dsi), (const u8[]){ seq },		\
				  sizeof((const u8[]){ seq }))

static int tokay_cur_vrefresh(struct tokay_panel *ctx)
{
	struct drm_connector *conn = ctx->connector;

	if (conn && conn->state && conn->state->crtc &&
	    conn->state->crtc->state)
		return drm_mode_vrefresh(&conn->state->crtc->state->adjusted_mode);

	return 60;
}

static int tokay_panel_enable(struct drm_panel *panel)
{
	struct tokay_panel *ctx = to_tokay_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;
	msleep(120);

	/* TK4C normal-mode init, retaining MTP-programmed TE/FGZ values. */
	tk4c_cmd(dsi, MIPI_DCS_SET_TEAR_ON);
	tk4c_cmd(dsi, MIPI_DCS_SET_COLUMN_ADDRESS,
		 0x00, 0x00, 0x04, 0x37);
	tk4c_cmd(dsi, MIPI_DCS_SET_PAGE_ADDRESS,
		 0x00, 0x00, 0x09, 0x77);

	/* FFC-off setup for the 756 Mbps link. */
	tk4c_cmd(dsi, 0xF0, 0x5A, 0x5A);
	tk4c_cmd(dsi, 0xFC, 0x5A, 0x5A);
	tk4c_cmd(dsi, 0xB0, 0x00, 0x3A, 0xC5);
	tk4c_cmd(dsi, 0xC5, 0x6C, 0x5C);
	tk4c_cmd(dsi, 0xB0, 0x00, 0x36, 0xC5);
	tk4c_cmd(dsi, 0xC5, 0x10);
	tk4c_cmd(dsi, 0xF0, 0xA5, 0xA5);
	tk4c_cmd(dsi, 0xFC, 0xA5, 0xA5);

	/* Select 60 or 120 Hz and latch the LTPS update. */
	tk4c_cmd(dsi, 0xF0, 0x5A, 0x5A);
	tk4c_cmd(dsi, 0x83, tokay_cur_vrefresh(ctx) == 60 ? 0x08 : 0x00);
	tk4c_cmd(dsi, 0xF7, 0x2F);
	tk4c_cmd(dsi, 0xF0, 0xA5, 0xA5);

	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to enable DSC: %d\n", ret);

	drm_dsc_pps_payload_pack(&pps, &tokay_tk4c_dsc);
	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to send DSC PPS: %d\n", ret);

	tk4c_cmd(dsi, 0x9D, 0x01);
	tk4c_cmd(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);

	return mipi_dsi_dcs_set_display_on(dsi);
}

static int tokay_panel_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct tokay_panel *ctx = to_tokay_panel(panel);
	unsigned int i;

	ctx->connector = connector;

	for (i = 0; i < ARRAY_SIZE(tokay_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &tokay_modes[i]);
		if (!mode)
			return i ? i : -ENOMEM;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = tokay_modes[0].width_mm;
	connector->display_info.height_mm = tokay_modes[0].height_mm;

	return ARRAY_SIZE(tokay_modes);
}

static const struct drm_panel_funcs tokay_panel_funcs = {
	.prepare = tokay_panel_prepare,
	.unprepare = tokay_panel_unprepare,
	.enable = tokay_panel_enable,
	.disable = tokay_panel_disable,
	.get_modes = tokay_panel_get_modes,
};

#define TOKAY_BRIGHTNESS_MAX		4095
#define TOKAY_BRIGHTNESS_DEFAULT	1290

static int tokay_bl_update_status(struct backlight_device *bl)
{
	struct tokay_panel *ctx = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);

	return mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, brightness);
}

static const struct backlight_ops tokay_bl_ops = {
	.update_status = tokay_bl_update_status,
};

static int tokay_panel_backlight_init(struct tokay_panel *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = TOKAY_BRIGHTNESS_DEFAULT,
		.max_brightness = TOKAY_BRIGHTNESS_MAX,
	};
	struct backlight_device *bl;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					    &tokay_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	ctx->panel.backlight = bl;
	return 0;
}

static int tokay_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct tokay_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct tokay_panel, panel,
				   &tokay_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset gpio\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->hs_rate = 756000000;
	dsi->dsc = (struct drm_dsc_config *)&tokay_tk4c_dsc;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ret = tokay_panel_backlight_init(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void tokay_panel_remove(struct mipi_dsi_device *dsi)
{
	struct tokay_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id tokay_panel_of_match[] = {
	{ .compatible = "google,gs-tk4c" },
	{ }
};
MODULE_DEVICE_TABLE(of, tokay_panel_of_match);

static struct mipi_dsi_driver tokay_panel_driver = {
	.probe = tokay_panel_probe,
	.remove = tokay_panel_remove,
	.driver = {
		.name = "panel-google-tokay-tk4c",
		.of_match_table = tokay_panel_of_match,
	},
};
module_mipi_dsi_driver(tokay_panel_driver);

MODULE_DESCRIPTION("Google Tokay TK4C MIPI-DSI panel driver");
MODULE_LICENSE("GPL");
