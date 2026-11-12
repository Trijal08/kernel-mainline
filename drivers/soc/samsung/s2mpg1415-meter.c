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

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/mod_devicetable.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#define S2MPG1415_METER_CHANNELS	12

/* METER_CTRL1 */
#define INT_SAMP_RATE			GENMASK(4, 2)
#define INT_SAMP_RATE_62P5HZ		3
#define EXT_METER_EN			BIT(1)
#define METER_EN			BIT(0)
/* METER_CTRL2 */
#define ASYNC_RD			BIT(7)
#define EXT_CHANNEL_EN			GENMASK(5, 3)
#define EXT_SAMP_RATE			GENMASK(2, 0)
#define EXT_SAMP_RATE_62P5HZ		3
/* METER_CTRL5 and METER_CTRL7 hold the top four channels' mode bits. */
#define METER_MODE_HI			GENMASK(3, 0)

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
#define DVS_NLDO_POWER_800mA_ON		IQ30(0.001221001221001)
#define DVS_NLDO_POWER_1200mA_ON	IQ30(0.001831501831501)

/*
 * A rail brought out to one of the three external sense inputs is measured
 * across a board shunt rather than inside the PMIC, so its resolution depends
 * on that shunt and has to be computed rather than looked up:
 *
 *	mW/LSB = V_rail/LSB * V_shunt/LSB * trim * 10 / shunt_uohms
 *
 * with the vendor's 2.1978021 and 0.7935698 and its 3-bit trim. Every external
 * rail on this board uses a 5 mOhm shunt.
 */
#define EXT_SHUNT_POWER(_uohms)						\
	IQ30(2.1978021 * 0.7935698 * 8 * 10 / (_uohms))
#define EXT_SHUNT_POWER_5MOHM		EXT_SHUNT_POWER(5000)

struct s2mpg1415_rail {
	const char *name;
	u8 muxsel;
	u32 resolution;
	/*
	 * For a rail on an external sense input, which of the three the board
	 * wired it to; zero for a rail measured inside the PMIC.
	 */
	u8 ext_channel_en;
};

struct s2mpg1415_chip {
	const char *name;
	const struct s2mpg1415_rail *rails;
	/* The thermistor engine and its NTC channels are S2MPG15-only. */
	bool has_ntc;
};

/*
 * Channel assignment is ours to choose -- nothing else programs MUXSEL in
 * mainline. These follow the zumapro schematic names, which reassign four
 * bucks relative to zuma (S2M/S3M swap, S9M, S11S). The external VSEN rails
 * (modem, WLAN/BT, camera) need shunt-resistance scaling that this driver does
 * not implement, so every channel goes to an internal buck or LDO.
 */
static const struct s2mpg1415_rail s2mpg14_rails[S2MPG1415_METER_CHANNELS] = {
	{ "VDD_MIF",		0x1, CMS_BUCK_POWER },
	{ "VDD_CPUCL1",		0x2, CMD_BUCK_POWER },
	{ "VDD_CPUCL2",		0x3, CMT_BUCK_POWER },
	{ "VDD_CPUCL0",		0x4, CMD_BUCK_POWER },
	{ "VDD_INT",		0x5, CMT_BUCK_POWER },
	{ "LLDO1_M",		0x6, CMS_BUCK_POWER },
	{ "VDD_TPU",		0x7, CMT_BUCK_POWER },
	{ "LLDO2_M",		0x8, CMS_BUCK_POWER },
	{ "VDD_CPUCL1_M",	0x9, CMS_BUCK_POWER },
	/*
	 * The display's supply, on the first external sense input. It is the
	 * largest single rail on the board at idle, and the only way to tell
	 * what the panel costs: there is no die sensor for it and its heat
	 * reaches the case rather than any thermal zone.
	 */
	{ "VSYS_PWR_DISPLAY",	0x5c, EXT_SHUNT_POWER_5MOHM, BIT(0) },
	/*
	 * Two of the clusters' SRAM supplies. LDOs, so whatever they drop is
	 * heat, and they are the only CPU rails not on a buck. The third,
	 * VDD_CPUCL2_HS_M, lost its channel to the display above.
	 */
	{ "VDD_CPUCL0_M",	0x2c, DVS_NLDO_POWER_1200mA_ON },
	{ "VDD_CPUCL2_HD_M",	0x31, DVS_NLDO_POWER_800mA_ON },
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
	{ "VDD_INT_M",		0xb, CMS_BUCK_POWER },
	{ "VDD_AUR",		0xc, CMD_BUCK_POWER },
};

static const struct s2mpg1415_chip s2mpg14_chip = {
	.name = "s2mpg14",
	.rails = s2mpg14_rails,
};

static const struct s2mpg1415_chip s2mpg15_chip = {
	.name = "s2mpg15",
	.rails = s2mpg15_rails,
	.has_ntc = true,
};

/* Also the width of the per-channel arrays below; see the NTC section. */
#define S2MPG1415_NTC_CHANNELS		8

struct s2mpg1415_ntc;

struct s2mpg1415_meter {
	struct regmap *regmap;
	const struct s2mpg1415_chip *chip;
	/* Serialises the latch-then-read sequence against concurrent readers. */
	struct mutex lock;
	/* NULL when the device tree gives the meter block no interrupt. */
	struct regmap_irq_chip_data *irq_data;
	/*
	 * Set once the NTC channels are enabled and have produced a real
	 * conversion.  Until then the data registers hold whatever the
	 * bootloader left behind, and a stale code is indistinguishable from
	 * a measured one.
	 */
	bool ntc_ready;
	/* Registered thermistor channels, and which of them are converting. */
	struct s2mpg1415_ntc *ntc[S2MPG1415_NTC_CHANNELS];
	unsigned int ntc_enabled;
	/* Stall detection, under @lock. */
	u16 ntc_code[S2MPG1415_NTC_CHANNELS];
	unsigned long ntc_changed;
};

/*
 * ---------------------------------------------------------------------------
 * NTC thermistors.
 *
 * The S2MPG15's meter block also drives eight NTC channels, which on this
 * board are the thermistors scattered around it -- including the one next to
 * the case that a skin-temperature policy wants to regulate on, rather than
 * the die sensors inside the SoC.
 *
 * Each channel's filtered reading is a 12-bit code, little-endian in two
 * bytes, that falls as the thermistor warms. The code is not linear in
 * temperature, so it is interpolated through a calibration table.
 * ---------------------------------------------------------------------------
 */

#define S2MPG1415_NTC_BUF		2	/* bytes per channel */
#define S2MPG1415_NTC_CODE_MASK		GENMASK(11, 0)
/* Writing this to a channel's LPF coefficient bypasses the filter. */
#define S2MPG1415_NTC_LPF_BYPASS	0x80
/*
 * The vendor's coefficient for every channel on caimito. Unfiltered, the
 * skin thermistor jumps several degrees in seconds under load.
 */
#define S2MPG1415_NTC_LPF_COEFF		0x0d
/* Reconfiguring the channels needs the meter engine reset first. */
#define S2MPG1415_NTC_METER_RESET	0x4f
#define S2MPG1415_NTC_SETTLE_US		100
#define S2MPG1415_NTC_SETTLE_MAX_US	10000
#define S2MPG1415_NTC_READY_MS		50
#define S2MPG1415_NTC_READY_TIMEOUT_MS	1000

/* The engine has been seen to stop converting; 0 disables the restart. */
static unsigned int ntc_stall_s = 600;
module_param(ntc_stall_s, uint, 0644);
MODULE_PARM_DESC(ntc_stall_s,
		 "Restart the meter after this many seconds without a new NTC code");

/*
 * Treat a channel's filter as running once its output agrees with the
 * conversion it is filtering. The band is wide because the reference is a
 * snapshot taken before the filter went in, and a thermistor next to
 * something that is powering up moves a few degrees while the rest of the
 * channels are brought up. What this has to catch is a filter left reading
 * the hot end of the calibration, which is wrong by thirty degrees or more.
 */
#define S2MPG1415_NTC_FILTER_TOL_MILLIC	10000
#define S2MPG1415_NTC_FILTER_TIMEOUT_MS	1000

/*
 * The thermistors sit against the case and are the slowest sensors on the
 * board: none of them moves several degrees between two samples. A jump that
 * large is the meter glitching, which the vendor filters as well
 * (filter_spurious_samples in its device tree). Hold the previous reading once
 * and believe the new one only if it repeats, so that a single bad sample
 * cannot trip a critical trip point.
 */
#define S2MPG1415_NTC_SLEW_MILLIC	5000

/* Falling code, rising temperature in millidegrees Celsius. */
struct s2mpg1415_ntc_pt {
	u16 code;
	int millic;
};

static const struct s2mpg1415_ntc_pt s2mpg15_ntc_map[] = {
	{ 0xf8d, -26428 }, { 0xf6a, -21922 }, { 0xf29, -15958 },
	{ 0xee4, -11060 }, { 0xe9d,  -6890 }, { 0xe3f,  -2264 },
	{ 0xdbf,   2961 }, { 0xd33,   7818 }, { 0xc97,  12525 },
	{ 0xbf5,  16945 }, { 0xb3a,  21623 }, { 0xa42,  27431 },
	{ 0x7f1,  40631 }, { 0x734,  44960 }, { 0x66b,  49757 },
	{ 0x5a3,  54854 }, { 0x4ee,  59898 }, { 0x446,  65076 },
	{ 0x43a,  65779 }, { 0x430,  65856 }, { 0x3c3,  69654 },
	{ 0x3bd,  69873 }, { 0x33b,  74910 }, { 0x2bb,  80691 },
	{ 0x259,  85844 }, { 0x206,  90915 }, { 0x1ce,  94873 },
	{ 0x191,  99720 }, { 0x160, 104216 }, { 0x12e, 109531 },
	{ 0x0f9, 116445 }, { 0x0d7, 121600 }, { 0x09f, 131839 },
};

struct s2mpg1415_ntc {
	struct s2mpg1415_meter *meter;
	struct thermal_zone_device *tz;
	unsigned int chan;
	/* Previous reading, and whether one was held back as suspect. */
	int last_millic;
	bool have_last;
	bool suspect;
};

/*
 * Each channel has an over- and an under-temperature comparator, and the
 * meter block raises an interrupt when the filtered code crosses one. The
 * threshold registers hold the top eight bits of the twelve-bit code.
 *
 * The two comparators are what let a zone be interrupt driven rather than
 * polled, which matters most for the thermistor next to the case: it is the
 * slowest sensor on the board and the one a skin policy regulates on, so it
 * should be the first to report a crossing, not something a timer notices up
 * to a poll interval later.
 */
#define S2MPG1415_NTC_WARN_SHIFT	4

/* Meter INT3 carries the eight over-temperature bits, INT4 the under. */
#define S2MPG1415_NTC_IRQ_OT(chan)	(chan)
#define S2MPG1415_NTC_IRQ_UT(chan)	(S2MPG1415_NTC_CHANNELS + (chan))

#define S2MPG1415_NTC_IRQ_OT_REG	2
#define S2MPG1415_NTC_IRQ_UT_REG	3

static const struct regmap_irq s2mpg1415_ntc_irqs[] = {
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(0), S2MPG1415_NTC_IRQ_OT_REG, BIT(0)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(1), S2MPG1415_NTC_IRQ_OT_REG, BIT(1)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(2), S2MPG1415_NTC_IRQ_OT_REG, BIT(2)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(3), S2MPG1415_NTC_IRQ_OT_REG, BIT(3)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(4), S2MPG1415_NTC_IRQ_OT_REG, BIT(4)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(5), S2MPG1415_NTC_IRQ_OT_REG, BIT(5)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(6), S2MPG1415_NTC_IRQ_OT_REG, BIT(6)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_OT(7), S2MPG1415_NTC_IRQ_OT_REG, BIT(7)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(0), S2MPG1415_NTC_IRQ_UT_REG, BIT(0)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(1), S2MPG1415_NTC_IRQ_UT_REG, BIT(1)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(2), S2MPG1415_NTC_IRQ_UT_REG, BIT(2)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(3), S2MPG1415_NTC_IRQ_UT_REG, BIT(3)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(4), S2MPG1415_NTC_IRQ_UT_REG, BIT(4)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(5), S2MPG1415_NTC_IRQ_UT_REG, BIT(5)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(6), S2MPG1415_NTC_IRQ_UT_REG, BIT(6)),
	REGMAP_IRQ_REG(S2MPG1415_NTC_IRQ_UT(7), S2MPG1415_NTC_IRQ_UT_REG, BIT(7)),
};

/*
 * The meter block's four interrupt registers live in its own regmap, so they
 * cannot be folded into the PMIC block's chip; both hang off the one interrupt
 * the SoC demultiplexes for this PMIC, which is why the line is requested
 * shared. Nothing is unmasked until a zone asks for a trip window.
 */
static const struct regmap_irq_chip s2mpg1415_meter_irq_chip = {
	.name = "s2mpg1415-meter",
	.status_base = S2MPG15_METER_INT1,
	.mask_base = S2MPG15_METER_INT1M,
	.num_regs = 4,
	.irqs = s2mpg1415_ntc_irqs,
	.num_irqs = ARRAY_SIZE(s2mpg1415_ntc_irqs),
};

/*
 * Interpolate a code into the table. A linear scan is plenty: the table has a
 * few dozen entries and a thermal zone reads at most a few times a second.
 */
static int s2mpg1415_ntc_millic(u16 code)
{
	const struct s2mpg1415_ntc_pt *t = s2mpg15_ntc_map;
	size_t n = ARRAY_SIZE(s2mpg15_ntc_map);
	size_t i;

	/* Off either end of the calibration, report its limit. */
	if (code >= t[0].code)
		return t[0].millic;
	if (code <= t[n - 1].code)
		return t[n - 1].millic;

	for (i = 1; i < n; i++)
		if (code >= t[i].code)
			break;

	/* Between t[i] (hotter, lower code) and t[i - 1] (colder, higher). */
	return t[i].millic + mult_frac(t[i - 1].millic - t[i].millic,
				       code - t[i].code,
				       t[i - 1].code - t[i].code);
}

static int s2mpg1415_ntc_read_code(struct s2mpg1415_meter *meter,
				   unsigned int chan, u16 *code)
{
	u8 buf[S2MPG1415_NTC_BUF];
	unsigned int reg;
	int ret;

	reg = S2MPG15_METER_LPF_DATA_NTC0_1 + S2MPG1415_NTC_BUF * chan;
	ret = regmap_bulk_read(meter->regmap, reg, buf, sizeof(buf));
	if (ret)
		return ret;

	*code = (buf[0] | (buf[1] << 8)) & S2MPG1415_NTC_CODE_MASK;

	/*
	 * The engine reads back all-zero until its first conversion lands, and
	 * anything below the calibration's hot end is not a measurement
	 * either: codes fall as the temperature rises, so every one of them
	 * clamps to the table's last point, 131 C, which is past the critical
	 * trip on every zone that reads this channel. Rejecting only zero
	 * leaves 1..0x9e decoding as a reading hot enough to shut the machine
	 * down. Treat the whole range as "not converting".
	 */
	if (*code < s2mpg15_ntc_map[ARRAY_SIZE(s2mpg15_ntc_map) - 1].code)
		return -EAGAIN;

	return 0;
}

/*
 * Return 0 once a channel's filtered output has reached @millic, -EAGAIN while
 * it is still on its way there.
 */
static int s2mpg1415_ntc_settled(struct s2mpg1415_meter *meter,
				 unsigned int chan, int millic)
{
	u16 code;
	int ret;

	ret = s2mpg1415_ntc_read_code(meter, chan, &code);
	if (ret)
		return ret;

	if (abs(s2mpg1415_ntc_millic(code) - millic) >
	    S2MPG1415_NTC_FILTER_TOL_MILLIC)
		return -EAGAIN;

	return 0;
}

static int s2mpg1415_meter_enable_ntc_lpf(struct device *dev,
					  struct s2mpg1415_meter *meter);

/*
 * Reset the meter engine, enable the thermistor channels with their filters
 * bypassed and wait for each to convert. The sequence is the vendor's,
 * including putting the power meter back. Called with @lock held.
 */
static int s2mpg1415_ntc_restart(struct device *dev,
				 struct s2mpg1415_meter *meter)
{
	unsigned int chan;
	int ret;

	for (chan = 0; chan < S2MPG1415_NTC_CHANNELS; chan++) {
		ret = regmap_write(meter->regmap,
				   S2MPG15_METER_NTC_LPF_C0_0 + chan,
				   S2MPG1415_NTC_LPF_BYPASS);
		if (ret)
			return ret;
	}
	meter->ntc_changed = jiffies;

	usleep_range(S2MPG1415_NTC_SETTLE_US, S2MPG1415_NTC_SETTLE_MAX_US);
	ret = regmap_write(meter->regmap, S2MPG15_METER_CTRL3, 0);
	if (ret)
		return ret;

	usleep_range(S2MPG1415_NTC_SETTLE_US, S2MPG1415_NTC_SETTLE_MAX_US);
	ret = regmap_write(meter->regmap, S2MPG15_METER_CTRL5,
			   S2MPG1415_NTC_METER_RESET);
	if (ret)
		return ret;

	usleep_range(S2MPG1415_NTC_SETTLE_US, S2MPG1415_NTC_SETTLE_MAX_US);
	ret = regmap_write(meter->regmap, S2MPG15_METER_CTRL3,
			   meter->ntc_enabled);
	if (ret)
		return ret;

	ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL5,
				 METER_MODE_HI, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(meter->regmap, S2MPG14_METER_CTRL1,
				METER_EN, METER_EN);
	if (ret)
		return ret;

	/* First conversions land a few tens of milliseconds later. */
	msleep(S2MPG1415_NTC_READY_MS);

	/*
	 * Wait for each channel to hold a real conversion, and keep what it
	 * reads. The channels stay unfiltered here, as the vendor leaves them:
	 * the filters go in once the meter is configured, which is the last
	 * thing the probe does. See s2mpg1415_meter_enable_ntc_lpf().
	 */
	for (chan = 0; chan < S2MPG1415_NTC_CHANNELS; chan++) {
		int err, millic;
		u16 code;

		if (!(meter->ntc_enabled & BIT(chan)))
			continue;

		ret = read_poll_timeout(s2mpg1415_ntc_read_code, err,
					err != -EAGAIN,
					S2MPG1415_NTC_READY_MS * USEC_PER_MSEC,
					S2MPG1415_NTC_READY_TIMEOUT_MS * USEC_PER_MSEC,
					false, meter, chan, &code);
		if (!ret)
			ret = err;
		if (ret) {
			dev_warn(dev, "NTC%u never converted: %d\n", chan, ret);
			continue;
		}

		/*
		 * What the channel reads with its filter still bypassed. This
		 * is the reading the zone would have got before the filter was
		 * programmed, so it is also the baseline the first filtered
		 * sample has to be plausible against.
		 */
		millic = s2mpg1415_ntc_millic(code);
		meter->ntc[chan]->last_millic = millic;
		meter->ntc[chan]->have_last = true;
		dev_dbg(dev, "NTC%u unfiltered code %#05x = %d mC\n",
			 chan, code, millic);
	}

	return 0;
}

static int s2mpg1415_ntc_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct s2mpg1415_ntc *ntc = thermal_zone_device_priv(tz);
	struct s2mpg1415_meter *meter = ntc->meter;
	unsigned int stall_s = READ_ONCE(ntc_stall_s);
	u16 code;
	int ret;

	guard(mutex)(&meter->lock);

	/*
	 * The zones are registered before the channels are enabled, and
	 * registration reads the temperature straight away.  A stale non-zero
	 * code from the bootloader decodes to the hot end of the scale, which
	 * is past the critical trip -- enough to shut the machine down during
	 * probe.  Report nothing until the engine is known to be converting.
	 */
	if (!meter->ntc_ready)
		return -EAGAIN;

	ret = s2mpg1415_ntc_read_code(meter, ntc->chan, &code);
	if (ret)
		return ret;

	if (code != meter->ntc_code[ntc->chan]) {
		meter->ntc_code[ntc->chan] = code;
		meter->ntc_changed = jiffies;
	} else if (stall_s &&
		   time_after(jiffies, meter->ntc_changed + stall_s * HZ)) {
		struct device *dev = regmap_get_device(meter->regmap);

		dev_warn(dev, "no new NTC code for %u s, restarting the meter\n",
			 stall_s);
		ret = s2mpg1415_ntc_restart(dev, meter);
		if (!ret)
			ret = s2mpg1415_meter_enable_ntc_lpf(dev, meter);
		return ret ?: -EAGAIN;
	}

	*temp = s2mpg1415_ntc_millic(code);

	/*
	 * Hold a lone jump upwards for one sample. If the heat is real the next
	 * reading confirms it, which costs one polling interval on a sensor
	 * that takes seconds to move; if it was a glitch, nothing downstream
	 * ever sees it.
	 */
	if (ntc->have_last && !ntc->suspect &&
	    *temp - ntc->last_millic > S2MPG1415_NTC_SLEW_MILLIC) {
		ntc->suspect = true;
		*temp = ntc->last_millic;
		return 0;
	}

	ntc->suspect = false;
	ntc->last_millic = *temp;
	ntc->have_last = true;

	return 0;
}

/*
 * The inverse of s2mpg1415_ntc_millic(), for programming the comparators.
 * Same table, same linear interpolation, walked the other way: the codes fall
 * as the temperature rises, so a temperature below the table's range maps to
 * its highest code and above it to its lowest.
 */
static u16 s2mpg1415_ntc_code(int millic)
{
	const struct s2mpg1415_ntc_pt *t = s2mpg15_ntc_map;
	size_t n = ARRAY_SIZE(s2mpg15_ntc_map);
	size_t i;

	if (millic <= t[0].millic)
		return t[0].code;
	if (millic >= t[n - 1].millic)
		return t[n - 1].code;

	for (i = 1; i < n; i++)
		if (millic <= t[i].millic)
			break;

	/* Between t[i - 1] (colder, higher code) and t[i] (hotter, lower). */
	return t[i].code + mult_frac(t[i - 1].code - t[i].code,
				     t[i].millic - millic,
				     t[i].millic - t[i - 1].millic);
}

/**
 * s2mpg1415_ntc_set_trips() - program a channel's comparators.
 * @tz:		the thermal zone.
 * @low:	temperature below which to report, in millidegrees.
 * @high:	temperature above which to report, in millidegrees.
 *
 * Crossing the window raises the channel's under- or over-temperature
 * interrupt. A bound the core does not want is passed as a saturated value,
 * which maps to the end of the calibration table and so to a comparator that
 * cannot fire.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int s2mpg1415_ntc_set_trips(struct thermal_zone_device *tz, int low,
				   int high)
{
	struct s2mpg1415_ntc *ntc = thermal_zone_device_priv(tz);
	u8 ut, ot;
	int ret;

	ut = s2mpg1415_ntc_code(low) >> S2MPG1415_NTC_WARN_SHIFT;
	ot = s2mpg1415_ntc_code(high) >> S2MPG1415_NTC_WARN_SHIFT;

	ret = regmap_write(ntc->meter->regmap,
			   S2MPG15_METER_NTC_UT_WARN0 + ntc->chan, ut);
	if (ret)
		return ret;

	return regmap_write(ntc->meter->regmap,
			    S2MPG15_METER_NTC_OT_WARN0 + ntc->chan, ot);
}

static irqreturn_t s2mpg1415_ntc_irq(int irq, void *data)
{
	struct s2mpg1415_ntc *ntc = data;

	/*
	 * The zone re-reads the channel and reprograms the window through
	 * ->set_trips(), which is what rearms the comparator that just fired.
	 */
	thermal_zone_device_update(ntc->tz, THERMAL_TRIP_VIOLATED);

	return IRQ_HANDLED;
}

static const struct thermal_zone_device_ops s2mpg1415_ntc_ops = {
	.get_temp = s2mpg1415_ntc_get_temp,
};

static const struct thermal_zone_device_ops s2mpg1415_ntc_ops_irq = {
	.get_temp = s2mpg1415_ntc_get_temp,
	.set_trips = s2mpg1415_ntc_set_trips,
};

/*
 * Register a zone per thermistor the device tree actually reads, then enable
 * exactly those channels.
 *
 * Changing the channel mask requires resetting the meter engine, which is why
 * this runs before the caller configures and enables it: the vendor driver
 * does the same work after ODPM is already running and so has to tear it down
 * and put it back afterwards. CTRL5 is clobbered by the reset, and the
 * caller's mode selection reprograms it.
 */
/**
 * s2mpg1415_meter_init_irq() - bring up the meter block's interrupts.
 * @dev:	the meter device.
 * @meter:	driver private data.
 *
 * Optional: a device tree that gives the meter node no interrupt leaves
 * @meter->irq_data NULL and the zones fall back to being polled, which is what
 * they did before this existed.
 *
 * Return: 0 on success or when there is no interrupt, -errno otherwise.
 */
static int s2mpg1415_meter_init_irq(struct device *dev,
				    struct s2mpg1415_meter *meter)
{
	int irq, ret;

	irq = of_irq_get(dev_of_node(dev), 0);
	if (irq == -EPROBE_DEFER)
		return irq;
	if (irq <= 0) {
		dev_dbg(dev, "No meter interrupt, NTC zones will be polled\n");
		return 0;
	}

	/*
	 * Shared because the PMIC block's chip is already on this line, and
	 * threaded one-shot because reaching these registers goes out over
	 * I3C.
	 */
	ret = devm_regmap_add_irq_chip(dev, meter->regmap, irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &s2mpg1415_meter_irq_chip,
				       &meter->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "cannot add meter irq chip\n");

	return 0;
}

/**
 * s2mpg1415_ntc_request_irqs() - take a channel's two comparator interrupts.
 * @dev:	the meter device.
 * @ntc:	the channel.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int s2mpg1415_ntc_request_irqs(struct device *dev,
				      struct s2mpg1415_ntc *ntc)
{
	static const char * const what[] = { "ot", "ut" };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(what); i++) {
		unsigned int hwirq = i ? S2MPG1415_NTC_IRQ_UT(ntc->chan)
				       : S2MPG1415_NTC_IRQ_OT(ntc->chan);
		const char *name;
		int virq;

		virq = regmap_irq_get_virq(ntc->meter->irq_data, hwirq);
		if (virq < 0)
			return virq;

		name = devm_kasprintf(dev, GFP_KERNEL, "ntc%u-%s", ntc->chan,
				      what[i]);
		if (!name)
			return -ENOMEM;

		ret = devm_request_threaded_irq(dev, virq, NULL,
						s2mpg1415_ntc_irq,
						IRQF_ONESHOT, name, ntc);
		if (ret)
			return ret;
	}

	return 0;
}

static int s2mpg1415_meter_init_ntc(struct device *dev,
				    struct s2mpg1415_meter *meter)
{
	unsigned int chan, enabled = 0;
	int ret;

	if (!meter->chip->has_ntc)
		return 0;

	ret = s2mpg1415_meter_init_irq(dev, meter);
	if (ret)
		return ret;

	for (chan = 0; chan < S2MPG1415_NTC_CHANNELS; chan++) {
		struct thermal_zone_device *tz;
		struct s2mpg1415_ntc *ntc;

		ntc = devm_kzalloc(dev, sizeof(*ntc), GFP_KERNEL);
		if (!ntc)
			return -ENOMEM;

		ntc->meter = meter;
		ntc->chan = chan;

		tz = devm_thermal_of_zone_register(dev, chan, ntc,
						   meter->irq_data ?
						   &s2mpg1415_ntc_ops_irq :
						   &s2mpg1415_ntc_ops);
		if (IS_ERR(tz)) {
			/*
			 * -ENODEV: no zone names this channel, so nothing
			 * reads that thermistor. -ENOTSUPP: the kernel has no
			 * CONFIG_THERMAL_OF, so no zone can name it. Either
			 * way leave the channel off rather than sample it for
			 * nobody, and keep the power meter working.
			 */
			if (PTR_ERR(tz) == -ENODEV || PTR_ERR(tz) == -ENOTSUPP)
				continue;
			return dev_err_probe(dev, PTR_ERR(tz),
					     "cannot register NTC%u\n", chan);
		}
		ntc->tz = tz;
		meter->ntc[chan] = ntc;

		if (meter->irq_data) {
			ret = s2mpg1415_ntc_request_irqs(dev, ntc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "cannot take NTC%u interrupts\n",
						     chan);
		}

		enabled |= BIT(chan);
	}

	if (!enabled)
		return 0;

	scoped_guard(mutex, &meter->lock) {
		meter->ntc_enabled = enabled;
		ret = s2mpg1415_ntc_restart(dev, meter);
	}
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot enable NTC channels %#x\n",
				     enabled);

	return 0;
}

/**
 * s2mpg1415_meter_enable_ntc_lpf() - put the thermistor filters in the path.
 * @dev:	the meter device.
 * @meter:	the meter.
 *
 * Programming a channel's filter coefficient has to come last, once the meter
 * engine is configured and running. Resetting the engine or toggling METER_EN
 * underneath a channel whose filter is already enabled corrupts that channel's
 * filtered output -- it comes back reading the hot end of the calibration,
 * which is past the critical trip on the skin zone -- and the vendor hits the
 * same thing (b/322027909 in its ODPM driver, which drops the filters around
 * every meter reset and restores them afterwards). Its bring-up is ordered the
 * same way: METER_EN first, filters last.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int s2mpg1415_meter_enable_ntc_lpf(struct device *dev,
					  struct s2mpg1415_meter *meter)
{
	unsigned int chan;
	int ret;

	if (!meter->ntc_enabled)
		return 0;

	for (chan = 0; chan < S2MPG1415_NTC_CHANNELS; chan++) {
		int err, millic;
		u16 code;

		if (!(meter->ntc_enabled & BIT(chan)))
			continue;

		millic = meter->ntc[chan]->last_millic;

		ret = regmap_write(meter->regmap,
				   S2MPG15_METER_NTC_LPF_C0_0 + chan,
				   S2MPG1415_NTC_LPF_COEFF);
		if (ret) {
			dev_warn(dev, "NTC%u left unfiltered: %d\n", chan, ret);
			continue;
		}

		/*
		 * Let the filter produce an output and check it against the
		 * conversion it is filtering before any zone is allowed to
		 * read it.
		 */
		ret = read_poll_timeout(s2mpg1415_ntc_settled, err,
					err != -EAGAIN,
					S2MPG1415_NTC_READY_MS * USEC_PER_MSEC,
					S2MPG1415_NTC_FILTER_TIMEOUT_MS * USEC_PER_MSEC,
					true, meter, chan, millic);
		if (!ret)
			ret = err;
		if (ret) {
			/*
			 * Put the channel back the way the engine resets it.
			 * An unfiltered thermistor is noisier than the vendor
			 * runs it, but it is a reading; a filter that never
			 * settled is not.
			 */
			dev_warn(dev, "NTC%u filter did not settle (%d), bypassed\n",
				 chan, ret);
			regmap_write(meter->regmap,
				     S2MPG15_METER_NTC_LPF_C0_0 + chan,
				     S2MPG1415_NTC_LPF_BYPASS);
		} else if (!s2mpg1415_ntc_read_code(meter, chan, &code)) {
			dev_dbg(dev, "NTC%u filtered code %#05x = %d mC\n",
				 chan, code, s2mpg1415_ntc_millic(code));
		}
	}

	meter->ntc_ready = true;

	dev_info(dev, "thermistor channels %#x enabled\n", meter->ntc_enabled);

	return 0;
}

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

static void s2mpg1415_meter_debugfs_remove(void *dentry)
{
	debugfs_remove(dentry);
}

static int s2mpg1415_meter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg1415_meter *meter;
	unsigned int acc_mode, ctrl1;
	u8 ext_en = 0;
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

	/*
	 * Before the meter is configured: enabling the thermistor channels
	 * resets the engine and clobbers CTRL5, which the mode selection
	 * below then reprograms.
	 */
	ret = s2mpg1415_meter_init_ntc(dev, meter);
	if (ret)
		return ret;

	ret = regmap_read(meter->regmap, S2MPG14_METER_CTRL4, &acc_mode);
	if (!ret)
		ret = regmap_read(meter->regmap, S2MPG14_METER_CTRL1, &ctrl1);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the meter config\n");

	dev_info(dev, "meter left in acc mode %#04x, rate selector %u\n",
		 acc_mode, (unsigned int)FIELD_GET(INT_SAMP_RATE, ctrl1));

	/*
	 * One bit per channel selects current rather than power, and the
	 * resolutions here are the power ones. CTRL4/5 cover the accumulators,
	 * CTRL6/7 the LPF.
	 */
	ret = regmap_write(meter->regmap, S2MPG14_METER_CTRL4, 0);
	if (!ret)
		ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL5,
					 METER_MODE_HI, 0);
	if (!ret)
		ret = regmap_write(meter->regmap, S2MPG14_METER_CTRL6, 0);
	if (!ret)
		ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL7,
					 METER_MODE_HI, 0);
	if (ret)
		return dev_err_probe(dev, ret, "cannot select power mode\n");

	/*
	 * Turn on whichever external sense inputs have a rail pointed at them,
	 * and give them the same rate as the internal channels.
	 *
	 * The rate matters here in a way it does not for the internal ones:
	 * acc/count is an average however often it sampled, but the count is
	 * shared, so an external channel sampling at a different rate would
	 * divide its own accumulator by the internal channels' sample count.
	 * The two rate fields take the same encoding, so this is just the same
	 * selector written twice.
	 */
	for (ch = 0; ch < S2MPG1415_METER_CHANNELS; ch++)
		ext_en |= meter->chip->rails[ch].ext_channel_en;

	if (ext_en) {
		ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL2,
					 EXT_CHANNEL_EN | EXT_SAMP_RATE,
					 FIELD_PREP(EXT_CHANNEL_EN, ext_en) |
					 FIELD_PREP(EXT_SAMP_RATE,
						    EXT_SAMP_RATE_62P5HZ));
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot enable external channels\n");
	}

	/*
	 * The rate is not in the arithmetic -- acc/count is an average however
	 * often it sampled -- but it has to be a known one: left alone, the two
	 * chips came up 60x apart and moved between boots.
	 */
	ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL1,
				 INT_SAMP_RATE | EXT_METER_EN | METER_EN,
				 FIELD_PREP(INT_SAMP_RATE,
					    INT_SAMP_RATE_62P5HZ) | METER_EN |
				 (ext_en ? EXT_METER_EN : 0));
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable the meter\n");

	/* Only now that the engine is running is it safe to filter the NTCs. */
	scoped_guard(mutex, &meter->lock)
		ret = s2mpg1415_meter_enable_ntc_lpf(dev, meter);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, s2mpg1415_meter_debugfs_remove,
					debugfs_create_file(meter->chip->name,
							    0400, NULL, meter,
							    &s2mpg1415_meter_fops));
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
