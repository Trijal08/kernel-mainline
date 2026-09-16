// SPDX-License-Identifier: GPL-2.0-only
/*
 * On-device power monitoring (ODPM) for the S2MPG14/S2MPG15 PMIC pair.
 *
 * Each PMIC carries twelve power meter channels. A channel is pointed at a
 * rail via its MUXSEL register and then accumulates power samples into a
 * 41-bit accumulator, with a shared 20-bit count of how many samples went in.
 * Average power over the window is therefore acc / count, scaled by a
 * per-rail resolution in mW/LSB -- no knowledge of the sampling frequency is
 * needed, which is why this driver does not configure or care about it.
 *
 * Writing ASYNC_RD latches the accumulators into readable registers and
 * restarts them, so each read of the debugfs file reports the average since
 * the previous read.
 *
 * Output is one debugfs file per PMIC, listing uW per rail. This is a
 * measurement tool, not a power supply: it deliberately does not register
 * with IIO or hwmon.
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define S2MPG1415_METER_CHANNELS	12

/* METER_CTRL1 */
#define METER_EN			BIT(0)
/* METER_CTRL2 */
#define ASYNC_RD			BIT(7)

/* Both registers occupy whole bytes but only the low bits carry the value. */
#define ACC_DATA_BYTES			6
#define ACC_DATA_MASK			GENMASK_ULL(40, 0)
#define ACC_COUNT_BYTES			3
#define ACC_COUNT_MASK			GENMASK(19, 0)
#define LPF_DATA_BYTES			3
#define LPF_DATA_MASK			GENMASK(20, 0)

/*
 * Resolutions are mW per LSB in unsigned Q30, straight from the vendor
 * tables. Q30 cannot hold a value >= 4.0, which no rail here approaches.
 */
#define IQ30(_val)			((u32)((_val) * (1U << 30)))

#define CMS_BUCK_POWER			IQ30(0.006868131868131)
#define CMD_BUCK_POWER			IQ30(0.013736263736263)
#define CMT_BUCK_POWER			IQ30(0.020604395604395)
#define VM_POWER			IQ30(0.013736263736263)
#define NLDO_POWER_150mA		IQ30(0.000457875457875)
#define PLDO_POWER_150mA		IQ30(0.000915750915750)
#define NLDO_POWER_1200mA		IQ30(0.001831501831501)

struct s2mpg1415_rail {
	const char *name;
	u8 muxsel;
	u32 resolution;
};

struct s2mpg1415_chip {
	const char *name;
	const struct s2mpg1415_rail *rails;
};

/*
 * Channel assignment is ours to choose -- nothing else programs MUXSEL in
 * mainline. These follow the caiman/komodo schematic names, and spend every
 * channel on an internal buck: the external VSEN rails (modem, WLAN/BT,
 * camera) need shunt-resistance scaling that this driver does not implement.
 */
static const struct s2mpg1415_rail s2mpg14_rails[S2MPG1415_METER_CHANNELS] = {
	{ "VDD_MIF",		0x1, CMS_BUCK_POWER },
	{ "VDD_CPUCL2",		0x2, CMD_BUCK_POWER },
	{ "VDD_CPUCL1",		0x3, CMT_BUCK_POWER },
	{ "VDD_CPUCL0",		0x4, CMD_BUCK_POWER },
	{ "VDD_INT",		0x5, CMT_BUCK_POWER },
	{ "LLDO1_M",		0x6, CMS_BUCK_POWER },
	{ "VDD_TPU",		0x7, CMT_BUCK_POWER },
	{ "LLDO2_M",		0x8, CMS_BUCK_POWER },
	{ "VDD_CPUCL0_M",	0x9, CMS_BUCK_POWER },
	{ }, { }, { },
};

static const struct s2mpg1415_rail s2mpg15_rails[S2MPG1415_METER_CHANNELS] = {
	{ "VDD_CAM",		0x1, CMD_BUCK_POWER },
	{ "VDD_G3D",		0x2, CMD_BUCK_POWER },
	{ "LLDO1_S",		0x3, CMS_BUCK_POWER },
	{ "VDD2H_MEM",		0x4, CMS_BUCK_POWER },
	{ "VDDQ_MEM",		0x5, CMS_BUCK_POWER },
	{ "LLDO2_S",		0x6, CMS_BUCK_POWER },
	{ "MLDO_S",		0x7, VM_POWER },
	{ "VDD_G3D_L2",		0x8, CMS_BUCK_POWER },
	{ "VDD_AOC",		0x9, CMS_BUCK_POWER },
	{ "VDD_SLC_M",		0xa, CMS_BUCK_POWER },
	{ "VDD_G3D_GLB",	0xb, CMS_BUCK_POWER },
	{ "VDD_AUR",		0xc, CMD_BUCK_POWER },
};

static const struct s2mpg1415_chip s2mpg14_chip = {
	.name = "s2mpg14",
	.rails = s2mpg14_rails,
};

static const struct s2mpg1415_chip s2mpg15_chip = {
	.name = "s2mpg15",
	.rails = s2mpg15_rails,
};

struct s2mpg1415_meter {
	struct regmap *regmap;
	const struct s2mpg1415_chip *chip;
	/* Serialises the latch-then-read sequence against concurrent readers. */
	struct mutex lock;
};

static u64 s2mpg1415_read_le(const u8 *buf, unsigned int bytes)
{
	u64 val = 0;
	int i;

	for (i = bytes - 1; i >= 0; i--)
		val = (val << 8) | buf[i];

	return val;
}

static int s2mpg1415_meter_show(struct seq_file *s, void *unused)
{
	struct s2mpg1415_meter *meter = s->private;
	u8 count_buf[ACC_COUNT_BYTES];
	u32 acc_count, ctrl2;
	int ch, ret;

	guard(mutex)(&meter->lock);

	/*
	 * Latch the accumulators and wait for the self-clear. The data and the
	 * sample count are not latched at the same instant, so reading during
	 * acquisition pairs an accumulator with a count from a different window
	 * and skews the average by whatever the mismatch happens to be.
	 */
	ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL2,
				 ASYNC_RD, ASYNC_RD);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(meter->regmap, S2MPG14_METER_CTRL2, ctrl2,
				       !(ctrl2 & ASYNC_RD), 1000, 200000);
	if (ret)
		return ret;

	ret = regmap_bulk_read(meter->regmap, S2MPG14_METER_ACC_COUNT_1,
			       count_buf, ACC_COUNT_BYTES);
	if (ret)
		return ret;

	acc_count = s2mpg1415_read_le(count_buf, ACC_COUNT_BYTES) & ACC_COUNT_MASK;
	if (!acc_count) {
		seq_puts(s, "no samples accumulated\n");
		return 0;
	}

	seq_printf(s, "%-16s %10s %10s\n", "rail", "uW", "uW_lpf");

	for (ch = 0; ch < S2MPG1415_METER_CHANNELS; ch++) {
		const struct s2mpg1415_rail *rail = &meter->chip->rails[ch];
		u8 acc_buf[ACC_DATA_BYTES], lpf_buf[LPF_DATA_BYTES];
		u64 acc, lpf, mw_iq30, lpf_mw_iq30;

		if (!rail->name)
			continue;

		ret = regmap_bulk_read(meter->regmap,
				       S2MPG14_METER_ACC_DATA_CH0_1 +
				       ch * ACC_DATA_BYTES,
				       acc_buf, ACC_DATA_BYTES);
		if (ret)
			return ret;

		ret = regmap_bulk_read(meter->regmap,
				       S2MPG14_METER_LPF_DATA_CH0_1 +
				       ch * LPF_DATA_BYTES,
				       lpf_buf, LPF_DATA_BYTES);
		if (ret)
			return ret;

		acc = s2mpg1415_read_le(acc_buf, ACC_DATA_BYTES) & ACC_DATA_MASK;
		lpf = s2mpg1415_read_le(lpf_buf, LPF_DATA_BYTES) & LPF_DATA_MASK;
		mw_iq30 = mul_u64_u64_div_u64(acc, rail->resolution, acc_count);
		lpf_mw_iq30 = lpf * rail->resolution;

		seq_printf(s, "%-16s %10llu %10llu\n", rail->name,
			   (mw_iq30 * 1000) >> 30, (lpf_mw_iq30 * 1000) >> 30);
	}

	seq_printf(s, "\nsamples in window: %u\n", acc_count);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(s2mpg1415_meter);

static int s2mpg1415_meter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg1415_meter *meter;
	int ch, ret;

	meter = devm_kzalloc(dev, sizeof(*meter), GFP_KERNEL);
	if (!meter)
		return -ENOMEM;

	meter->chip = (const void *)platform_get_device_id(pdev)->driver_data;

	meter->regmap = dev_get_regmap(dev->parent, "meter");
	if (!meter->regmap)
		return dev_err_probe(dev, -ENODEV, "no meter regmap on parent\n");

	ret = devm_mutex_init(dev, &meter->lock);
	if (ret)
		return ret;

	for (ch = 0; ch < S2MPG1415_METER_CHANNELS; ch++) {
		ret = regmap_write(meter->regmap, S2MPG14_METER_MUXSEL0 + ch,
				   meter->chip->rails[ch].muxsel);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot set MUXSEL%d\n", ch);
	}

	ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL1,
				 METER_EN, METER_EN);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable the meter\n");

	debugfs_create_file(meter->chip->name, 0400, NULL, meter,
			    &s2mpg1415_meter_fops);

	return 0;
}

static const struct platform_device_id s2mpg1415_meter_id[] = {
	{ "s2mpg14-meter", (kernel_ulong_t)&s2mpg14_chip },
	{ "s2mpg15-meter", (kernel_ulong_t)&s2mpg15_chip },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpg1415_meter_id);

static struct platform_driver s2mpg1415_meter_driver = {
	.probe	= s2mpg1415_meter_probe,
	.id_table = s2mpg1415_meter_id,
	.driver	= {
		.name = "s2mpg1415-meter",
	},
};
module_platform_driver(s2mpg1415_meter_driver);

MODULE_DESCRIPTION("S2MPG14/S2MPG15 on-device power monitoring");
MODULE_LICENSE("GPL");
