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
#define METER_EN			BIT(0)
/* METER_CTRL2 */
#define ASYNC_RD			BIT(7)
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

struct s2mpg1415_rail {
	const char *name;
	u8 muxsel;
	u32 resolution;
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
	 * The clusters' SRAM supplies. LDOs, so whatever they drop is heat,
	 * and they are the only CPU rails not on a buck.
	 */
	{ "VDD_CPUCL2_HS_M",	0x2b, DVS_NLDO_POWER_1200mA_ON },
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

struct s2mpg1415_meter {
	struct regmap *regmap;
	const struct s2mpg1415_chip *chip;
	/* Serialises the latch-then-read sequence against concurrent readers. */
	struct mutex lock;
	/* NULL when the device tree gives the meter block no interrupt. */
	struct regmap_irq_chip_data *irq_data;
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

#define S2MPG1415_NTC_CHANNELS		8
#define S2MPG1415_NTC_BUF		2	/* bytes per channel */
#define S2MPG1415_NTC_CODE_MASK		GENMASK(11, 0)
/* Writing this to a channel's LPF coefficient bypasses the filter. */
#define S2MPG1415_NTC_LPF_BYPASS	0x80
/* Reconfiguring the channels needs the meter engine reset first. */
#define S2MPG1415_NTC_METER_RESET	0x4f
#define S2MPG1415_NTC_SETTLE_US		100
#define S2MPG1415_NTC_SETTLE_MAX_US	10000
#define S2MPG1415_NTC_READY_MS		50

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

static int s2mpg1415_ntc_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct s2mpg1415_ntc *ntc = thermal_zone_device_priv(tz);
	u8 buf[S2MPG1415_NTC_BUF];
	unsigned int reg;
	u16 code;
	int ret;

	reg = S2MPG15_METER_LPF_DATA_NTC0_1 + S2MPG1415_NTC_BUF * ntc->chan;
	ret = regmap_bulk_read(ntc->meter->regmap, reg, buf, sizeof(buf));
	if (ret)
		return ret;

	code = (buf[0] | (buf[1] << 8)) & S2MPG1415_NTC_CODE_MASK;
	/* The engine reads back all-zero until its first conversion lands. */
	if (!code)
		return -EAGAIN;

	*temp = s2mpg1415_ntc_millic(code);
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

	/* The filter has to be out of the way across the reset. */
	for (chan = 0; chan < S2MPG1415_NTC_CHANNELS; chan++) {
		ret = regmap_write(meter->regmap,
				   S2MPG15_METER_NTC_LPF_C0_0 + chan,
				   S2MPG1415_NTC_LPF_BYPASS);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot bypass NTC%u filter\n",
					     chan);
	}

	usleep_range(S2MPG1415_NTC_SETTLE_US, S2MPG1415_NTC_SETTLE_MAX_US);
	ret = regmap_write(meter->regmap, S2MPG15_METER_CTRL3, 0);
	if (!ret) {
		usleep_range(S2MPG1415_NTC_SETTLE_US,
			     S2MPG1415_NTC_SETTLE_MAX_US);
		ret = regmap_write(meter->regmap, S2MPG15_METER_CTRL5,
				   S2MPG1415_NTC_METER_RESET);
	}
	if (!ret) {
		usleep_range(S2MPG1415_NTC_SETTLE_US,
			     S2MPG1415_NTC_SETTLE_MAX_US);
		ret = regmap_write(meter->regmap, S2MPG15_METER_CTRL3, enabled);
	}
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot enable NTC channels %#x\n",
				     enabled);

	/* First conversions land a few tens of milliseconds later. */
	msleep(S2MPG1415_NTC_READY_MS);

	dev_info(dev, "thermistor channels %#x enabled\n", enabled);
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

static int s2mpg1415_meter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg1415_meter *meter;
	unsigned int acc_mode, ctrl1;
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
	 * The rate is not in the arithmetic -- acc/count is an average however
	 * often it sampled -- but it has to be a known one: left alone, the two
	 * chips came up 60x apart and moved between boots.
	 */
	ret = regmap_update_bits(meter->regmap, S2MPG14_METER_CTRL1,
				 INT_SAMP_RATE | METER_EN,
				 FIELD_PREP(INT_SAMP_RATE,
					    INT_SAMP_RATE_62P5HZ) | METER_EN);
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
