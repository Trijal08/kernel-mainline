// SPDX-License-Identifier: GPL-2.0
/*
 * CS40L26 Boosted Haptic Driver with Integrated DSP and Waveform Memory
 *
 * Copyright 2024 Cirrus Logic, Inc.
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 *
 * Modelled on the CS40L50 driver by James Ogletree <james.ogletree@cirrus.com>.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/firmware/cirrus/cs_dsp.h>
#include <linux/firmware/cirrus/wmfw.h>
#include <linux/mfd/core.h>
#include <linux/mfd/cs40l26.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

static const struct mfd_cell cs40l26_devs[] = {
	{ .name = "cs40l26-vibra", },
};

const struct regmap_config cs40l26_regmap = {
	.reg_bits =		32,
	.reg_stride =		4,
	.val_bits =		32,
	.reg_format_endian =	REGMAP_ENDIAN_BIG,
	.val_format_endian =	REGMAP_ENDIAN_BIG,
};
EXPORT_SYMBOL_GPL(cs40l26_regmap);

static const char * const cs40l26_supplies[] = {
	"vdd-a", "vdd-p",
};

/*
 * The ROM image differs between silicon revisions, so the addresses of its
 * state variables move with it.
 */
static const struct cs40l26_rom_regs cs40l26_rom_regs_a1_b0_b1 = {
	.pm_cur_state =		0x02800370,
	.dsp_halo_state =	0x02800FA8,
};

static const struct cs40l26_rom_regs cs40l26_rom_regs_b2 = {
	.pm_cur_state =		0x02801F98,
	.dsp_halo_state =	0x02806AF8,
};

int cs40l26_dsp_write(struct device *dev, struct regmap *regmap, u32 val)
{
	int i, ret;
	u32 ack;

	/* Device NAKs if hibernating, so optionally retry */
	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = regmap_write(regmap, CS40L26_DSP_VIRTUAL1_MBOX_1, val);
		if (!ret)
			break;

		usleep_range(CS40L26_DSP_POLL_US, CS40L26_DSP_POLL_US + 100);
	}

	/* If the write never took place, no need to check for the ACK */
	if (i == CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out writing %#X to DSP: %d\n", val, ret);
		return ret;
	}

	ret = regmap_read_poll_timeout(regmap, CS40L26_DSP_VIRTUAL1_MBOX_1, ack, !ack,
				       CS40L26_DSP_POLL_US,
				       CS40L26_DSP_POLL_US * CS40L26_DSP_TIMEOUT_COUNT);
	if (ret)
		dev_err(dev, "DSP failed to ACK %#X: %d\n", val, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(cs40l26_dsp_write);

static const struct cs_dsp_region cs40l26_dsp_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = CS40L26_PMEM_0 },
	{ .type = WMFW_HALO_XM_PACKED, .base = CS40L26_XMEM_PACKED_0 },
	{ .type = WMFW_HALO_YM_PACKED, .base = CS40L26_YMEM_PACKED_0 },
	{ .type = WMFW_ADSP2_XM, .base = CS40L26_XMEM_UNPACKED24_0 },
	{ .type = WMFW_ADSP2_YM, .base = CS40L26_YMEM_UNPACKED24_0 },
};

/*
 * The patched firmware runs from RAM, which must be powered and initialised
 * before the core is started: the hardware readiness bit first, then the
 * firmware's own initialisation entry point.  This runs as the cs_dsp post_run
 * hook, where the controls the firmware just brought with it are resolvable
 * and reach the device directly.
 */
static int cs40l26_dsp_post_run(struct cs_dsp *dsp)
{
	struct cs40l26 *cs40l26 = container_of(dsp, struct cs40l26, dsp);
	struct cs_dsp_coeff_ctl *ctl;
	__be32 val = cpu_to_be32(1);
	int ret;

	ret = regmap_update_bits(cs40l26->regmap, CS40L26_PWRMGT_CTL,
				 CS40L26_MEM_RDY, CS40L26_MEM_RDY);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to ready DSP memory: %d\n", ret);
		return ret;
	}

	ctl = cs_dsp_get_ctl(dsp, "CALL_RAM_INIT", WMFW_ADSP2_XM, dsp->fw_id);
	if (!ctl) {
		dev_err(cs40l26->dev, "Control not found for RAM init\n");
		return -ENOENT;
	}

	/* A write reports whether the value changed, which is not an error */
	ret = cs_dsp_coeff_write_ctrl(ctl, 0, &val, sizeof(val));

	return ret < 0 ? ret : 0;
}

static const struct cs_dsp_client_ops cs40l26_client_ops = {
	.post_run = cs40l26_dsp_post_run,
};

static void cs40l26_dsp_remove(void *data)
{
	cs_dsp_remove(data);
}

static int cs40l26_dsp_init(struct cs40l26 *cs40l26)
{
	int ret;

	cs40l26->dsp.num = 1;
	cs40l26->dsp.type = WMFW_HALO;
	cs40l26->dsp.dev = cs40l26->dev;
	cs40l26->dsp.regmap = cs40l26->regmap;
	cs40l26->dsp.base = CS40L26_CORE_BASE;
	cs40l26->dsp.base_sysinfo = CS40L26_SYS_INFO_ID;
	cs40l26->dsp.mem = cs40l26_dsp_regions;
	cs40l26->dsp.num_mems = ARRAY_SIZE(cs40l26_dsp_regions);
	/* The core is started below, with the extra bit this part requires */
	cs40l26->dsp.no_core_startstop = true;
	cs40l26->dsp.client_ops = &cs40l26_client_ops;

	ret = cs_dsp_halo_init(&cs40l26->dsp);
	if (ret)
		return ret;

	return devm_add_action_or_reset(cs40l26->dev, cs40l26_dsp_remove,
					&cs40l26->dsp);
}

/*
 * Before the RAM patch is written the DSP must be running the ROM firmware and
 * parked somewhere it is safe to stop: killing the core mid-playback leaves
 * the amplifier driving the LRA.
 */
static int cs40l26_dsp_stop_core(struct cs40l26 *cs40l26)
{
	u32 halo_state, state;
	int i, ret;

	ret = regmap_read(cs40l26->regmap, cs40l26->rom_regs->dsp_halo_state,
			  &halo_state);
	if (ret)
		return ret;

	if (halo_state != CS40L26_DSP_HALO_STATE_RUN) {
		dev_err(cs40l26->dev, "ROM firmware not running: HALO_STATE %#X\n",
			halo_state);
		return -ENODEV;
	}

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = regmap_read(cs40l26->regmap, cs40l26->rom_regs->pm_cur_state,
				  &state);
		if (ret)
			return ret;

		state = FIELD_GET(CS40L26_DSP_STATE_MASK, state);
		if (state == CS40L26_DSP_STATE_SHUTDOWN ||
		    state == CS40L26_DSP_STATE_STANDBY)
			break;

		usleep_range(CS40L26_DSP_POLL_US, CS40L26_DSP_POLL_US + 100);
	}

	if (i == CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(cs40l26->dev, "DSP not safe to stop: state %u\n", state);
		return -EBUSY;
	}

	return regmap_write(cs40l26->regmap, CS40L26_CCM_CORE_CONTROL,
			    CS40L26_CCM_CORE_KILL);
}

static int cs40l26_reset_dsp(struct cs40l26 *cs40l26)
{
	int ret;

	guard(mutex)(&cs40l26->lock);

	if (cs40l26->dsp.running)
		cs_dsp_stop(&cs40l26->dsp);

	if (cs40l26->dsp.booted)
		cs_dsp_power_down(&cs40l26->dsp);

	/* Hold the part awake for the duration of the patch */
	ret = cs40l26_dsp_write(cs40l26->dev, cs40l26->regmap, CS40L26_PREVENT_HIBER);
	if (ret)
		return ret;

	ret = cs40l26_dsp_stop_core(cs40l26);
	if (ret)
		return ret;

	ret = cs_dsp_power_up(&cs40l26->dsp, cs40l26->fw, CS40L26_FW,
			      cs40l26->bin, CS40L26_WT, "cs40l26");
	if (ret)
		return ret;

	/* Readies the DSP's RAM through the post_run hook above */
	ret = cs_dsp_run(&cs40l26->dsp);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_CCM_CORE_CONTROL,
			   CS40L26_CCM_CORE_RESET);
	if (ret)
		return ret;

	/*
	 * The patched firmware owns the mailbox from here on, so acknowledging
	 * a command is what proves it came up.
	 */
	return cs40l26_dsp_write(cs40l26->dev, cs40l26->regmap, CS40L26_PREVENT_HIBER);
}

static void cs40l26_dsp_power_down(void *data)
{
	cs_dsp_power_down(data);
}

static void cs40l26_dsp_stop(void *data)
{
	cs_dsp_stop(data);
}

static void cs40l26_dsp_bringup(const struct firmware *bin, void *context)
{
	struct cs40l26 *cs40l26 = context;
	int ret;

	/* Wavetable is optional; bring the DSP up regardless */
	cs40l26->bin = bin;

	ret = cs40l26_reset_dsp(cs40l26);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to reset DSP: %d\n", ret);
		goto err_fw;
	}

	/* Add teardown actions for first-time bringup */
	ret = devm_add_action_or_reset(cs40l26->dev, cs40l26_dsp_power_down,
				       &cs40l26->dsp);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to add power down action: %d\n", ret);
		goto err_fw;
	}

	ret = devm_add_action_or_reset(cs40l26->dev, cs40l26_dsp_stop, &cs40l26->dsp);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to add stop action: %d\n", ret);
		goto err_fw;
	}

	ret = devm_mfd_add_devices(cs40l26->dev, PLATFORM_DEVID_NONE, cs40l26_devs,
				   ARRAY_SIZE(cs40l26_devs), NULL, 0, NULL);
	if (ret)
		dev_err(cs40l26->dev, "Failed to add child devices: %d\n", ret);
err_fw:
	release_firmware(cs40l26->bin);
	release_firmware(cs40l26->fw);
}

static void cs40l26_request_firmware(const struct firmware *fw, void *context)
{
	struct cs40l26 *cs40l26 = context;
	int ret;

	if (!fw) {
		dev_err(cs40l26->dev, "No firmware file found\n");
		return;
	}

	cs40l26->fw = fw;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT, CS40L26_WT,
				      cs40l26->dev, GFP_KERNEL, cs40l26,
				      cs40l26_dsp_bringup);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to request %s: %d\n", CS40L26_WT, ret);
		release_firmware(cs40l26->fw);
	}
}

static int cs40l26_get_model(struct cs40l26 *cs40l26)
{
	int ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_DEVID, &cs40l26->devid);
	if (ret)
		return ret;

	cs40l26->devid &= CS40L26_DEVID_MASK;

	switch (cs40l26->devid) {
	case CS40L26_DEVID_L26_A:
	case CS40L26_DEVID_L26_B:
	case CS40L26_DEVID_L27_A:
	case CS40L26_DEVID_L27_B:
		break;
	default:
		dev_err(cs40l26->dev, "Unrecognised device ID %#X\n", cs40l26->devid);
		return -ENODEV;
	}

	ret = regmap_read(cs40l26->regmap, CS40L26_REVID, &cs40l26->revid);
	if (ret)
		return ret;

	cs40l26->revid &= CS40L26_REVID_MASK;

	switch (cs40l26->revid) {
	case CS40L26_REVID_A1:
	case CS40L26_REVID_B0:
	case CS40L26_REVID_B1:
		cs40l26->rom_regs = &cs40l26_rom_regs_a1_b0_b1;
		break;
	case CS40L26_REVID_B2:
		cs40l26->rom_regs = &cs40l26_rom_regs_b2;
		break;
	default:
		dev_err(cs40l26->dev, "Unsupported revision %02X\n", cs40l26->revid);
		return -ENODEV;
	}

	dev_dbg(cs40l26->dev, "Cirrus Logic CS40L%02X rev. %02X\n",
		(cs40l26->devid >> 4) & 0xFF, cs40l26->revid);

	return 0;
}

static int cs40l26_pm_runtime_setup(struct device *dev)
{
	int ret;

	pm_runtime_set_autosuspend_delay(dev, CS40L26_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_get_noresume(dev);
	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;

	return devm_pm_runtime_enable(dev);
}

int cs40l26_probe(struct cs40l26 *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret;

	ret = devm_mutex_init(dev, &cs40l26->lock);
	if (ret)
		return ret;

	cs40l26->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cs40l26->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(cs40l26->reset_gpio),
				     "Failed getting reset GPIO\n");

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(cs40l26_supplies),
					     cs40l26_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed getting supplies\n");

	/* Ensure minimum reset pulse width */
	usleep_range(CS40L26_RESET_PULSE_US, CS40L26_RESET_PULSE_US + 100);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, 0);

	/* Wait for control port to be ready */
	usleep_range(CS40L26_CP_READY_US, CS40L26_CP_READY_US + 100);

	ret = cs40l26_get_model(cs40l26);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get part number\n");

	ret = cs40l26_dsp_init(cs40l26);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize DSP\n");

	ret = cs40l26_pm_runtime_setup(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize runtime PM\n");

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT, CS40L26_FW,
				      dev, GFP_KERNEL, cs40l26,
				      cs40l26_request_firmware);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request %s\n", CS40L26_FW);

	pm_runtime_put_autosuspend(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(cs40l26_probe);

static int cs40l26_runtime_suspend(struct device *dev)
{
	struct cs40l26 *cs40l26 = dev_get_drvdata(dev);

	/* No ACK: the part is free to stop answering as soon as this lands */
	return regmap_write(cs40l26->regmap, CS40L26_DSP_VIRTUAL1_MBOX_1,
			    CS40L26_ALLOW_HIBER);
}

static int cs40l26_runtime_resume(struct device *dev)
{
	struct cs40l26 *cs40l26 = dev_get_drvdata(dev);

	return cs40l26_dsp_write(dev, cs40l26->regmap, CS40L26_PREVENT_HIBER);
}

EXPORT_GPL_DEV_PM_OPS(cs40l26_pm_ops) = {
	RUNTIME_PM_OPS(cs40l26_runtime_suspend, cs40l26_runtime_resume, NULL)
};

MODULE_DESCRIPTION("CS40L26 Boosted Haptic Driver");
MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("FW_CS_DSP");
