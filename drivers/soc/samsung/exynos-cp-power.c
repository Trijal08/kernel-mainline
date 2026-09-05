// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos modem (CP) power/reset sequencer
 *
 * Copyright 2026 Trijal Saha
 *
 * Owns the AP2CP power/reset rail GPIOs of the Exynos modem and the two SPMI
 * slaves the reset sequence has to touch -- the CP PMIC (SID 0) and the S5910
 * DCXO clock buffer (SID 5), reached over the bit-banged SPMI bus.  The PCIe RC
 * that hosts the modem endpoint drives link training but delegates the CP
 * power/reset mechanics here: warm reset (MAIN survives in self-refresh DRAM)
 * and a full cold power cycle.  A faithful port of the downstream
 * power_reset_warm_cp() / power_on_cp() sequences.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-cp-power.h>
#include <linux/spmi.h>

struct exynos_cp_power {
	struct device		*dev;
	/* AP2CP rail-sequencing lines (see exynos_cp_power_warm_reset()) */
	struct gpio_desc	*cp_pwr;
	struct gpio_desc	*cp_nreset;
	struct gpio_desc	*cp_wrst;
	struct gpio_desc	*cp_pm_wrst;
	struct gpio_desc	*cp_pda_active;
	struct gpio_desc	*cp_dump_noti;	/* AP2CP_DUMP_NOTI (out) */
	struct gpio_desc	*cp2ap_cp_wrst;	/* CP warm-reset status (in) */
	/* SPMI slaves patched after a PMIC reset; NULL if the bus is not wired */
	struct regmap		*pmic;		/* CP PMIC, SID 0 */
	struct regmap		*dcxo;		/* S5910 DCXO buffer, SID 5 */
	bool			spmi_ready;	/* slaves resolved (lazily) */
	bool			ever_powered_on;	/* gate the S5910 re-arm */
};

/* 16-bit register address, one data byte: SPMI Extended Register R/W Long. */
static const struct regmap_config exynos_cp_spmi_regmap_cfg = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
};

/*
 * Resolve one SPMI slave (child @name phandle) to a regmap, or NULL if it is
 * not wired or the SPMI controller has not bound yet (the caller retries).
 */
static struct regmap *exynos_cp_power_get_spmi(struct exynos_cp_power *cp,
					       const char *name)
{
	struct spmi_device *sdev;
	struct device_node *np;
	struct regmap *regmap;

	np = of_parse_phandle(cp->dev->of_node, name, 0);
	if (!np)
		return NULL;

	sdev = spmi_find_device_by_of_node(np);
	of_node_put(np);
	if (!sdev)
		return NULL;

	regmap = devm_regmap_init_spmi_ext(sdev, &exynos_cp_spmi_regmap_cfg);
	put_device(&sdev->dev);
	return IS_ERR(regmap) ? NULL : regmap;
}

/*
 * Resolve the CP PMIC and S5910 SPMI slaves.  Done lazily on the first reset
 * rather than at probe, so cp-power -- and the PCIe RC that must reset the CP
 * before it can train the link -- is not held behind the whole SPMI bus
 * binding; a CP reset that runs late leaves the endpoint in a state a warm
 * reset cannot revive.  This runs in process context, so wait briefly for the
 * SPMI controller if it is still coming up; absent slaves only cost the
 * post-reset PMIC/DCXO patch, not the reset itself.
 */
static void exynos_cp_power_setup_spmi(struct exynos_cp_power *cp)
{
	int i;

	if (cp->spmi_ready)
		return;

	for (i = 0; i < 20; i++) {		/* up to ~200 ms */
		if (!cp->pmic)
			cp->pmic = exynos_cp_power_get_spmi(cp, "google,cp-pmic");
		if (!cp->dcxo)
			cp->dcxo = exynos_cp_power_get_spmi(cp, "google,cp-dcxo");
		if (cp->pmic && cp->dcxo)
			break;
		msleep(10);
	}

	cp->spmi_ready = true;
	if (!cp->pmic)
		dev_warn(cp->dev, "CP PMIC SPMI unavailable; skipping its patch\n");
	if (!cp->dcxo)
		dev_warn(cp->dev, "S5910 SPMI unavailable; skipping its patch\n");
}

/*
 * S5910 DCXO clock buffer (SID 5).  The CP releases its refclk request when it
 * sleeps; the buffer's LPM configuration decides whether it survives that.  It
 * loses its state on a full power-off and only the on_seq programs it, so a
 * board that last ran the stock kernel sits at hardware defaults and the CP's
 * first autonomous sleep (~125 ms after ONLINE when idle) kills it.  Dump the
 * pre-state for diagnosis, then apply the vendor on_seq while the CP is held in
 * reset (downstream gpio_power_off_cp_with_s5910_on()).
 */
static void exynos_cp_power_s5910_on_seq(struct exynos_cp_power *cp)
{
	/* komodo fdt s5910,on_seq */
	static const struct { u16 reg; u8 val; } on_seq[] = {
		{ 0x211, 0x47 },
		{ 0x236, 0x0e },
		{ 0x215, 0x7f },
		{ 0x20e, 0x00 },	/* DCXO LPM mode off */
	};
	static const u16 dump_regs[] = { 0x20e, 0x211, 0x215, 0x236, 0x240 };
	struct device *dev = cp->dev;
	unsigned int val;
	int i, retry, ret;

	if (!cp->dcxo)
		return;

	for (i = 0; i < ARRAY_SIZE(dump_regs); i++) {
		if (regmap_read(cp->dcxo, dump_regs[i], &val))
			dev_warn(dev, "s5910 reg %#x read fail\n", dump_regs[i]);
		else
			dev_info(dev, "s5910 reg %#x = %#02x\n", dump_regs[i], val);
	}

	for (i = 0; i < ARRAY_SIZE(on_seq); i++) {
		for (retry = 0; retry < 2; retry++) {
			ret = regmap_write(cp->dcxo, on_seq[i].reg, on_seq[i].val);
			if (!ret)
				break;
			msleep(2);
		}
		if (ret) {
			dev_err(dev, "s5910 write %#x=%#x failed\n",
				on_seq[i].reg, on_seq[i].val);
			return;
		}
	}
	dev_info(dev, "s5910 turn-on sequence completed (DCXO LPM off)\n");
}

/*
 * CP PMIC (SID 0).  The CP_PMIC_WRST pulse in the warm reset returns the PMIC
 * to its OTP defaults; patch it (downstream pmic_warm_reset_sequence), or the
 * CP boots but dies at its first autonomous low-power transition (~120 ms after
 * ONLINE when idle).  Log the OTP version first (doubles as a bus liveness
 * check).
 */
static void exynos_cp_power_pmic_warm_reset_seq(struct exynos_cp_power *cp)
{
	static const struct { u16 reg; u8 val; } seq[] = {
		{ 0x675, 0xa1 },
		{ 0x67d, 0xbe },
	};
	struct device *dev = cp->dev;
	unsigned int otp;
	int i, retry, ret;

	if (!cp->pmic) {
		dev_warn(dev, "no CP PMIC SPMI bus; skipping warm reset sequence\n");
		return;
	}

	for (retry = 0; retry < 2; retry++) {
		ret = regmap_read(cp->pmic, 0x30e, &otp);
		if (!ret)
			break;
		msleep(2);
	}
	if (ret)
		dev_warn(dev, "CP PMIC OTP version read fail\n");
	else
		dev_info(dev, "CP PMIC OTP version %#x\n", otp);

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		for (retry = 0; retry < 2; retry++) {
			ret = regmap_write(cp->pmic, seq[i].reg, seq[i].val);
			if (!ret)
				break;
			msleep(2);
		}
		if (ret) {
			dev_err(dev, "CP PMIC write %#x=%#x failed\n",
				seq[i].reg, seq[i].val);
			return;
		}
	}
	dev_info(dev, "CP PMIC warm reset sequence completed\n");
}

int exynos_cp_power_warm_reset(struct exynos_cp_power *cp, bool dump)
{
	struct device *dev = cp->dev;
	int val = -1, i;

	dev_info(dev, "warm-resetting the CP (modem) endpoint%s\n",
		 dump ? " into dump mode" : "");

	exynos_cp_power_setup_spmi(cp);

	/*
	 * WARM reset only: the CP's DRAM is self-powered, so the MAIN image from
	 * a previous boot survives -- the bootloader just re-runs the resident
	 * image.  cp_pwr and cp_nreset go high WITHOUT a low pulse so an
	 * already-powered CP keeps its DRAM.
	 */
	gpiod_direction_output(cp->cp_pwr, 1);
	gpiod_direction_output(cp->cp_nreset, 1);
	/*
	 * AP2CP_DUMP_NOTI latched across the reset is what makes the ROM start
	 * its minidump agent instead of a normal boot (downstream
	 * power_reset_dump_cp): the agent decodes the CP's encoded crash record
	 * into srinfo as ASCII, which is the only way to read the assert reason.
	 */
	if (cp->cp_dump_noti)
		gpiod_direction_output(cp->cp_dump_noti, dump);
	/* The caller (RC) drives AP2CP_WAKEUP low before this; it is a link signal. */

	/*
	 * gpio_power_wreset_cp(): frame the CP_WRST_N toggle inside a PM_WRST_N
	 * (CP_PMIC_WRST) pulse, waiting for the CP to acknowledge on
	 * CP2AP_CP_WRST_N.  Releasing PM_WRST before CP_WRST is load-bearing: the
	 * CP must leave warm reset with its PMIC reset already de-asserted.
	 */
	if (cp->cp2ap_cp_wrst) {
		val = gpiod_get_value_cansleep(cp->cp2ap_cp_wrst);
		if (!val)
			dev_warn(dev, "cp2ap_cp_wrst low before warm reset\n");
	}

	gpiod_direction_output(cp->cp_pm_wrst, 1);
	msleep(5);
	gpiod_direction_output(cp->cp_wrst, 0);
	if (val == 0)
		udelay(1000);

	for (i = 0; cp->cp2ap_cp_wrst && i < 20; i++) {
		if (!gpiod_get_value_cansleep(cp->cp2ap_cp_wrst))
			break;
		usleep_range(1000, 1100);
	}

	gpiod_set_value_cansleep(cp->cp_pm_wrst, 0);
	msleep(5);

	/* CP core still held in reset: program the S5910 DCXO buffer. */
	exynos_cp_power_s5910_on_seq(cp);

	gpiod_set_value_cansleep(cp->cp_wrst, 1);
	msleep(45);

	/* Patch the PMIC that the CP_PMIC_WRST pulse reset to OTP defaults. */
	exynos_cp_power_pmic_warm_reset_seq(cp);

	gpiod_direction_output(cp->cp_pda_active, 1);

	/*
	 * The CP has now been powered, so a later cold cycle must re-arm the
	 * S5910 (downstream sets this before its warm reset too); only the very
	 * first cold power-on, with nothing yet to preserve, skips it.
	 */
	cp->ever_powered_on = true;

	msleep(200);				/* ROM settle before link training */
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_cp_power_warm_reset);

int exynos_cp_power_cold_cycle(struct exynos_cp_power *cp)
{
	dev_info(cp->dev, "CP cold power cycle\n");

	exynos_cp_power_setup_spmi(cp);

	/*
	 * --- gpio_power_off_cp_with_s5910_on() ---
	 * The caller (RC) has already pulsed AP2CP_WAKEUP (a link signal it owns)
	 * before entering here.
	 */
	gpiod_set_value_cansleep(cp->cp_nreset, 0);
	/* Re-arm the S5910 while the CP is held in reset (skipped first power-on). */
	if (cp->ever_powered_on) {
		udelay(10);
		exynos_cp_power_s5910_on_seq(cp);
		udelay(200);
	}
	gpiod_set_value_cansleep(cp->cp_wrst, 0);
	gpiod_set_value_cansleep(cp->cp_pwr, 0);
	msleep(30);
	gpiod_set_value_cansleep(cp->cp_pm_wrst, 0);
	msleep(50);

	/* --- gpio_power_offon_cp() power-on half --- */
	cp->ever_powered_on = true;
	gpiod_set_value_cansleep(cp->cp_pm_wrst, 1);
	msleep(10);
	gpiod_set_value_cansleep(cp->cp_pwr, 1);
	msleep(10);
	gpiod_set_value_cansleep(cp->cp_nreset, 1);
	msleep(10);
	gpiod_set_value_cansleep(cp->cp_wrst, 1);

	msleep(200);				/* ROM settle before link training */
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_cp_power_cold_cycle);

struct exynos_cp_power *exynos_cp_power_get(struct device *consumer)
{
	struct platform_device *pdev;
	struct exynos_cp_power *cp;
	struct device_node *np;

	np = of_parse_phandle(consumer->of_node, "google,cp-power", 0);
	if (!np)
		return ERR_PTR(-ENOENT);

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	cp = platform_get_drvdata(pdev);
	/* The platform core keeps the device; drop our lookup reference. */
	platform_device_put(pdev);
	if (!cp)
		return ERR_PTR(-EPROBE_DEFER);

	return cp;
}
EXPORT_SYMBOL_GPL(exynos_cp_power_get);

static int exynos_cp_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_cp_power *cp;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	cp->dev = dev;

	/* Rail lines that must exist; requested in their reset-idle state. */
	cp->cp_pwr = devm_gpiod_get(dev, "cp-pwr", GPIOD_ASIS);
	cp->cp_nreset = devm_gpiod_get(dev, "cp-nreset", GPIOD_ASIS);
	cp->cp_pm_wrst = devm_gpiod_get(dev, "cp-pm-wrst", GPIOD_ASIS);
	cp->cp_wrst = devm_gpiod_get(dev, "cp-wrst", GPIOD_ASIS);
	cp->cp_pda_active = devm_gpiod_get(dev, "cp-pda-active", GPIOD_ASIS);
	if (IS_ERR(cp->cp_pwr) || IS_ERR(cp->cp_nreset) ||
	    IS_ERR(cp->cp_pm_wrst) || IS_ERR(cp->cp_wrst) ||
	    IS_ERR(cp->cp_pda_active))
		return dev_err_probe(dev, -EINVAL, "missing a CP rail gpio\n");

	/* Optional companions; each miss is a real gap in the vendor sequence. */
	cp->cp_dump_noti = devm_gpiod_get_optional(dev, "cp-dump-noti",
						   GPIOD_ASIS);
	cp->cp2ap_cp_wrst = devm_gpiod_get_optional(dev, "cp2ap-cp-wrst",
						    GPIOD_IN);
	if (IS_ERR(cp->cp_dump_noti) || IS_ERR(cp->cp2ap_cp_wrst))
		return dev_err_probe(dev, -EINVAL, "bad optional CP gpio\n");
	if (!cp->cp_dump_noti)
		dev_warn(dev, "no cp-dump-noti gpio (vendor clears it pre-reset)\n");
	if (!cp->cp2ap_cp_wrst)
		dev_warn(dev, "no cp2ap-cp-wrst gpio (reset unhandshaken)\n");

	/*
	 * The SPMI slaves are resolved lazily on the first reset, not here, so
	 * this driver binds immediately and does not delay the CP reset behind
	 * the SPMI bus coming up (see exynos_cp_power_setup_spmi()).
	 */
	platform_set_drvdata(pdev, cp);
	return 0;
}

static const struct of_device_id exynos_cp_power_of_match[] = {
	{ .compatible = "google,exynos-cp-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_cp_power_of_match);

static struct platform_driver exynos_cp_power_driver = {
	.probe = exynos_cp_power_probe,
	.driver = {
		.name = "exynos-cp-power",
		.of_match_table = exynos_cp_power_of_match,
	},
};
module_platform_driver(exynos_cp_power_driver);

MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_DESCRIPTION("Exynos modem (CP) power/reset sequencer");
MODULE_LICENSE("GPL");
