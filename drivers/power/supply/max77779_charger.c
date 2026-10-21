// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779_charger.c - VBUS/OTG boost regulator for the MAX77779 charger.
 *
 * Derived from max77759_charger.c. The MAX77779 charger block is register
 * compatible with the MAX77759 for the CHG_CNFG_00.MODE control used here, but
 * boards such as zumapro/komodo source OTG VBUS through an *external* boost
 * enabled by a GPIO (vendor "extbst-ctl") rather than the internal reverse
 * boost. Both paths are supported.
 *
 * Copyright 2025 Google LLC.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/linear_range.h>
#include <linux/mfd/max77779.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/workqueue.h>

/* Defaults when the DT carries no battery node. */
#define CHG_CC_DEFAULT_UA			2266770
#define CHG_FV_DEFAULT_UV			4300000
/*
 * Rp-default source, and no get_current_limit on this TCPC, so tcpm reports
 * CURRENT_MAX = 0. 500 mA drains this board while plugged in until it resets;
 * fall back to the Rp-1.5A level instead. input_current_limit stays writeable
 * at runtime if a weak source ever browns out under it.
 */
#define CHG_ILIM_FALLBACK_UA			1500000

/*
 * How often to re-evaluate the charge tier while charging. The battery moves
 * between tiers over minutes, and each evaluation is two I2C reads.
 */
#define CHG_TIER_POLL_MS			30000
/* Three times the CHG_CV_PRM step, as the vendor policy uses. */
#define CHG_TIER_HEADROOM_UV			30000

#define MAX_NUM_RETRIES				3
#define PSY_WORK_RETRY_DELAY_MS			10

/* CHG_CNFG_02.CHGCC, in uA */
static const struct linear_range chgcc_limit_ranges[] = {
	LINEAR_RANGE(200000, 0x3, 0x3c, 66670),
};

/* CHG_CNFG_04.CHG_CV_PRM, in uV */
static const struct linear_range chg_cv_prm_ranges[] = {
	LINEAR_RANGE(4000000, 0x0, 0x37, 10000),
	LINEAR_RANGE(3800000, 0x38, 0x39, 100000),
};

/* CHG_CNFG_09.CHGIN_ILIM, in uA */
static const struct linear_range chgin_ilim_ranges[] = {
	LINEAR_RANGE(125000, 0x4, 0x7f, 25000),
};

/* CHG_DETAILS_01.CHG_DTLS */
#define CHG_DTLS_DEAD_BATTERY			0x0
#define CHG_DTLS_CC				0x1
#define CHG_DTLS_CV				0x2
#define CHG_DTLS_TOP_OFF			0x3
#define CHG_DTLS_DONE				0x4
#define CHG_DTLS_TIMER_FAULT			0x6
#define CHG_DTLS_DETBAT_HIGH_SUSPEND		0x7
#define CHG_DTLS_OFF				0x8
#define CHG_DTLS_OFF_HIGH_TEMP			0xa
#define CHG_DTLS_OFF_WATCHDOG			0xb
#define CHG_DTLS_OFF_JEITA			0xc
#define CHG_DTLS_OFF_TEMP			0xd

/*
 * The vendor charge table: a constant-voltage tier is picked from the battery
 * voltage and a temperature band from its temperature, and the two select the
 * charge current. A zero current means this tier is not allowed at this
 * temperature, which is how the profile keeps a cold cell off the high tiers.
 */
struct max77779_chg_profile {
	u32 *temp_limits;	/* deci-degrees C, n_temp of them */
	u32 *volt_limits;	/* uV, n_volt of them */
	u32 *cc_limits;		/* uA, (n_temp - 1) rows of n_volt */
	int n_temp;
	int n_volt;
};

struct max77779_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct notifier_block nb;
	struct power_supply *tcpm_psy;
	struct delayed_work psy_work;
	struct mutex retry_lock; /* protects psy_work_retry_cnt */
	u32 psy_work_retry_cnt;
	struct regulator_dev *otg_rdev;
	/* Optional external VBUS boost enable (e.g. komodo extbst-ctl). */
	struct gpio_desc *ext_bst_ctl;
	struct mutex lock; /* protects mode */
	enum max77779_chgr_mode mode;
	struct max77779_chg_profile profile;
	struct power_supply *fg_psy;
	struct delayed_work tier_work;
	int tier_temp_idx;
	int tier_volt_idx;
};

/* The CHG_CNFG_* registers CHGPROT gates drop writes silently while locked. */
static inline int unlock_prot_regs(struct max77779_charger *chg, bool unlock)
{
	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_06,
				  MAX77779_CHGR_REG_CHG_CNFG_06_CHGPROT,
				  unlock ? MAX77779_CHGR_REG_CHG_CNFG_06_CHGPROT
					 : 0);
}

static int charger_input_valid(struct max77779_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_INT_OK, &val);
	if (ret)
		return ret;

	return !!(val & MAX77779_CHGR_REG_CHG_INT_OK_CHGIN);
}

static int get_online(struct max77779_charger *chg)
{
	u32 val;
	int ret;

	ret = charger_input_valid(chg);
	if (ret <= 0)
		return ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_02, &val);
	if (ret)
		return ret;

	return !!(val & MAX77779_CHGR_REG_CHG_DETAILS_02_CHGIN_STS);
}

static int get_status(struct max77779_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_01, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_01_CHG_DTLS, val)) {
	case CHG_DTLS_DEAD_BATTERY:
	case CHG_DTLS_CC:
	case CHG_DTLS_CV:
	case CHG_DTLS_TOP_OFF:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHG_DTLS_DONE:
		return POWER_SUPPLY_STATUS_FULL;
	case CHG_DTLS_TIMER_FAULT:
	case CHG_DTLS_DETBAT_HIGH_SUSPEND:
	case CHG_DTLS_OFF_HIGH_TEMP:
	case CHG_DTLS_OFF_WATCHDOG:
	case CHG_DTLS_OFF_JEITA:
	case CHG_DTLS_OFF_TEMP:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case CHG_DTLS_OFF:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static int get_health(struct max77779_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_00, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_00_CHGIN_DTLS, val)) {
	case 0x0:
	case 0x1:
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	case 0x2:
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	case 0x3:
		return POWER_SUPPLY_HEALTH_GOOD;
	default:
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}
}

static int get_fast_charge_current(struct max77779_charger *chg)
{
	u32 regval, val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_02, &regval);
	if (ret)
		return ret;

	regval = FIELD_GET(MAX77779_CHGR_REG_CHG_CNFG_02_CHGCC, regval);
	ret = linear_range_get_value_array(chgcc_limit_ranges,
					   ARRAY_SIZE(chgcc_limit_ranges),
					   regval, &val);
	return ret ? ret : val;
}

/*
 * A battery profile states what the cell will take, which on this board is more
 * than the charge path can deliver -- 1.2C is 5.47A against a 4.0A ceiling. Clamp
 * rather than refuse, or asking for the cell's real limit fails the request.
 */
static int set_fast_charge_current(struct max77779_charger *chg, u32 cc_ua)
{
	const struct linear_range *last =
		&chgcc_limit_ranges[ARRAY_SIZE(chgcc_limit_ranges) - 1];
	u32 regval;
	bool found;

	cc_ua = clamp(cc_ua, chgcc_limit_ranges[0].min,
		      linear_range_get_max_value(last));

	linear_range_get_selector_high_array(chgcc_limit_ranges,
					     ARRAY_SIZE(chgcc_limit_ranges),
					     cc_ua, &regval, &found);
	if (!found)
		return -EINVAL;

	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_02,
				  MAX77779_CHGR_REG_CHG_CNFG_02_CHGCC, regval);
}

static int get_float_voltage(struct max77779_charger *chg)
{
	u32 regval, val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_04, &regval);
	if (ret)
		return ret;

	regval = FIELD_GET(MAX77779_CHGR_REG_CHG_CNFG_04_CHG_CV_PRM, regval);
	ret = linear_range_get_value_array(chg_cv_prm_ranges,
					   ARRAY_SIZE(chg_cv_prm_ranges),
					   regval, &val);
	return ret ? ret : val;
}

static int set_float_voltage(struct max77779_charger *chg, u32 fv_uv)
{
	u32 regval;
	bool found;

	linear_range_get_selector_high_array(chg_cv_prm_ranges,
					     ARRAY_SIZE(chg_cv_prm_ranges),
					     fv_uv, &regval, &found);
	if (!found)
		return -EINVAL;

	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_04,
				  MAX77779_CHGR_REG_CHG_CNFG_04_CHG_CV_PRM,
				  regval);
}

static int get_input_current_limit(struct max77779_charger *chg)
{
	u32 regval, val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_09, &regval);
	if (ret)
		return ret;

	regval = FIELD_GET(MAX77779_CHGR_REG_CHG_CNFG_09_CHGIN_ILIM, regval);
	regval = umax(regval, chgin_ilim_ranges[0].min_sel);
	ret = linear_range_get_value_array(chgin_ilim_ranges,
					   ARRAY_SIZE(chgin_ilim_ranges),
					   regval, &val);
	return ret ? ret : val;
}

/*
 * NO_AUTOIBUS pins the limit to what the driver programs. Left clear, the
 * hardware's BC1.2 autodetect re-arms CHGIN_ILIM on every attach -- typically
 * to 500 mA -- and quietly overrides anything set here.
 */
static int set_input_current_limit(struct max77779_charger *chg, int ilim_ua)
{
	u32 regval;

	if (ilim_ua < 0)
		return -EINVAL;

	linear_range_get_selector_within(chgin_ilim_ranges, ilim_ua, &regval);

	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_09,
				  MAX77779_CHGR_REG_CHG_CNFG_09_NO_AUTOIBUS |
				  MAX77779_CHGR_REG_CHG_CNFG_09_CHGIN_ILIM,
				  MAX77779_CHGR_REG_CHG_CNFG_09_NO_AUTOIBUS |
				  regval);
}

static int charger_set_mode(struct max77779_charger *chg,
			    enum max77779_chgr_mode mode)
{
	int ret;

	guard(mutex)(&chg->lock);

	if (chg->mode == mode)
		return 0;

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00,
				 MAX77779_CHGR_REG_CHG_CNFG_00_MODE, mode);
	if (ret)
		return ret;

	chg->mode = mode;
	return 0;
}

static const enum power_supply_property max77779_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int max77779_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *pval)
{
	struct max77779_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = get_online(chg);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = charger_input_valid(chg);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = get_status(chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = get_health(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_fast_charge_current(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = get_float_voltage(chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = get_input_current_limit(chg);
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	pval->intval = ret;
	return 0;
}

/*
 * Writable so a source that cannot advertise its real capability -- a BC1.2
 * wall charger behind an A-to-C cable, which can only present Rp-default --
 * can be given a usable limit by hand.
 */
static int max77779_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *pval)
{
	struct max77779_charger *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return set_fast_charge_current(chg, pval->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		return set_float_voltage(chg, pval->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return set_input_current_limit(chg, pval->intval);
	default:
		return -EINVAL;
	}
}

static int max77779_charger_prop_is_writeable(struct power_supply *psy,
					      enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		return 0;
	}
}

static const struct power_supply_desc max77779_charger_desc = {
	.name = "max77779-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77779_charger_props,
	.num_properties = ARRAY_SIZE(max77779_charger_props),
	.get_property = max77779_charger_get_property,
	.set_property = max77779_charger_set_property,
	.property_is_writeable = max77779_charger_prop_is_writeable,
};

/*
 * VBUS sourcing for USB OTG. On boards with an external boost (extbst-ctl
 * GPIO, e.g. zumapro/komodo) the charger MODE is left ALL_OFF and VBUS is
 * driven by the external boost; otherwise the internal reverse boost
 * (OTG_BOOST_ON) is used. Mirrors the vendor max77779_get_usecase()
 * GSU_MODE_USB_OTG path.
 */
static int max77779_otg_enable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	if (chg->ext_bst_ctl) {
		/*
		 * External boost sources VBUS; keep the charger off. Assert the
		 * boost-enable regardless of the mode write so VBUS comes up.
		 */
		ret = charger_set_mode(chg, MAX77779_CHGR_MODE_ALL_OFF);
		if (ret)
			dev_warn(chg->dev, "OTG: ALL_OFF failed: %d\n", ret);

		gpiod_set_value_cansleep(chg->ext_bst_ctl, 1);
		dev_info(chg->dev, "OTG VBUS on: external boost, ext_bst=%d\n",
			 gpiod_get_value_cansleep(chg->ext_bst_ctl));
		return 0;
	}

	dev_info(chg->dev, "OTG VBUS on: internal reverse boost\n");
	return charger_set_mode(chg, MAX77779_CHGR_MODE_OTG_BOOST_ON);
}

static int max77779_otg_disable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	if (chg->ext_bst_ctl)
		gpiod_set_value_cansleep(chg->ext_bst_ctl, 0);

	/*
	 * Deasserting the boost is what takes VBUS down, so the rail is off by
	 * here whatever the mode write does. Report success regardless: a
	 * failing disable() leaves the regulator core counting a user it no
	 * longer has, and it then skips every later enable() as redundant and
	 * never raises VBUS again.
	 */
	ret = charger_set_mode(chg, MAX77779_CHGR_MODE_ALL_OFF);
	if (ret)
		dev_warn(chg->dev, "OTG off: ALL_OFF failed: %d\n", ret);

	return 0;
}

static int max77779_otg_is_enabled(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);

	if (chg->ext_bst_ctl)
		return gpiod_get_value_cansleep(chg->ext_bst_ctl);

	guard(mutex)(&chg->lock);

	return chg->mode == MAX77779_CHGR_MODE_OTG_BOOST_ON;
}

static const struct regulator_ops max77779_otg_ops = {
	.enable = max77779_otg_enable,
	.disable = max77779_otg_disable,
	.is_enabled = max77779_otg_is_enabled,
};

static const struct regulator_desc max77779_otg_desc = {
	.name = "otg",
	.of_match = of_match_ptr("otg-regulator"),
	.owner = THIS_MODULE,
	.ops = &max77779_otg_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

/*
 * Both indices follow the vendor policy exactly. The tier climbs while the
 * battery is above a limit, and then one further if it is within headroom of
 * the next one, so that the tier can still deliver its current rather than
 * sitting in constant voltage at the bottom of it.
 */
static int tier_temp_idx(const struct max77779_chg_profile *p, int temp)
{
	int i = 0;

	while (i < p->n_temp - 2 && temp >= p->temp_limits[i + 1])
		i++;

	return i;
}

static int tier_volt_idx(const struct max77779_chg_profile *p, int vbatt)
{
	int i = 0;

	while (i < p->n_volt - 1 && vbatt > p->volt_limits[i])
		i++;

	if (i != p->n_volt - 1 &&
	    p->volt_limits[i] - vbatt < CHG_TIER_HEADROOM_UV)
		i++;

	return i;
}

static bool tier_temp_in_range(const struct max77779_chg_profile *p, int temp)
{
	return temp >= p->temp_limits[0] &&
	       temp < p->temp_limits[p->n_temp - 1];
}

/*
 * Only steer the charger when it is actually charging or idle on an input:
 * anything else means OTG is sourcing VBUS and the mode is not ours to touch.
 */
static bool tier_mode_is_ours(struct max77779_charger *chg)
{
	guard(mutex)(&chg->lock);

	return chg->mode == MAX77779_CHGR_MODE_CHGR_BUCK_ON ||
	       chg->mode == MAX77779_CHGR_MODE_BUCK_ON;
}

static void tier_work_item(struct work_struct *work)
{
	struct max77779_charger *chg =
		container_of(work, struct max77779_charger, tier_work.work);
	const struct max77779_chg_profile *p = &chg->profile;
	union power_supply_propval temp, vbatt;
	int ti, vi, cc_ua, ret;

	if (!tier_mode_is_ours(chg) || get_online(chg) <= 0)
		return;

	ret = power_supply_get_property(chg->fg_psy, POWER_SUPPLY_PROP_TEMP,
					&temp);
	if (!ret)
		ret = power_supply_get_property(chg->fg_psy,
						POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&vbatt);
	if (ret) {
		dev_warn_ratelimited(chg->dev,
				     "no battery temperature or voltage (%d); leaving the charge tier alone\n",
				     ret);
		goto rearm;
	}

	if (!tier_temp_in_range(p, temp.intval)) {
		if (chg->tier_temp_idx != -1)
			dev_info(chg->dev,
				 "battery at %d.%d C is outside the charge profile; charging off\n",
				 temp.intval / 10, abs(temp.intval % 10));
		chg->tier_temp_idx = -1;
		charger_set_mode(chg, MAX77779_CHGR_MODE_BUCK_ON);
		goto rearm;
	}

	ti = tier_temp_idx(p, temp.intval);
	vi = tier_volt_idx(p, vbatt.intval);
	cc_ua = p->cc_limits[ti * p->n_volt + vi];

	if (ti == chg->tier_temp_idx && vi == chg->tier_volt_idx)
		goto rearm;

	if (!cc_ua) {
		dev_info(chg->dev,
			 "charge tier %d.%03d V is not allowed at %d.%d C; charging off\n",
			 p->volt_limits[vi] / 1000000,
			 p->volt_limits[vi] % 1000000 / 1000,
			 temp.intval / 10, abs(temp.intval % 10));
		charger_set_mode(chg, MAX77779_CHGR_MODE_BUCK_ON);
	} else {
		ret = set_float_voltage(chg, p->volt_limits[vi]);
		if (!ret)
			ret = set_fast_charge_current(chg, cc_ua);
		if (ret) {
			dev_warn(chg->dev, "cannot apply charge tier: %d\n",
				 ret);
			goto rearm;
		}

		charger_set_mode(chg, MAX77779_CHGR_MODE_CHGR_BUCK_ON);
		dev_info(chg->dev,
			 "charge tier %d.%03d V at %d mA (%d.%d C)\n",
			 p->volt_limits[vi] / 1000000,
			 p->volt_limits[vi] % 1000000 / 1000, cc_ua / 1000,
			 temp.intval / 10, abs(temp.intval % 10));
	}

	chg->tier_temp_idx = ti;
	chg->tier_volt_idx = vi;

rearm:
	schedule_delayed_work(&chg->tier_work,
			      msecs_to_jiffies(CHG_TIER_POLL_MS));
}

/*
 * Charge currents are a percentage of the pack capacity in the device tree, as
 * in the vendor profile, so that one table describes a cell rather than a cell
 * in a particular phone.
 */
static int max77779_charger_parse_profile(struct max77779_charger *chg,
					  u32 capacity_ua)
{
	struct max77779_chg_profile *p = &chg->profile;
	struct fwnode_handle *node;
	int rows, i, ret;

	node = fwnode_find_reference(dev_fwnode(chg->dev), "monitored-battery",
				     0);
	if (IS_ERR(node))
		return -ENODEV;

	p->n_temp = fwnode_property_count_u32(node, "google,chg-temp-limits");
	p->n_volt = fwnode_property_count_u32(node, "google,chg-cv-limits");
	if (p->n_temp < 3 || p->n_volt < 1) {
		ret = -ENODEV;
		goto out;
	}

	rows = p->n_temp - 1;

	if (fwnode_property_count_u32(node, "google,chg-cc-limits") !=
	    rows * p->n_volt) {
		dev_err(chg->dev,
			"google,chg-cc-limits must be %d by %d\n", rows,
			p->n_volt);
		ret = -EINVAL;
		goto out;
	}

	p->temp_limits = devm_kcalloc(chg->dev, p->n_temp,
				      sizeof(*p->temp_limits), GFP_KERNEL);
	p->volt_limits = devm_kcalloc(chg->dev, p->n_volt,
				      sizeof(*p->volt_limits), GFP_KERNEL);
	p->cc_limits = devm_kcalloc(chg->dev, rows * p->n_volt,
				    sizeof(*p->cc_limits), GFP_KERNEL);
	if (!p->temp_limits || !p->volt_limits || !p->cc_limits) {
		ret = -ENOMEM;
		goto out;
	}

	ret = fwnode_property_read_u32_array(node, "google,chg-temp-limits",
					     p->temp_limits, p->n_temp);
	if (!ret)
		ret = fwnode_property_read_u32_array(node,
						     "google,chg-cv-limits",
						     p->volt_limits,
						     p->n_volt);
	if (!ret)
		ret = fwnode_property_read_u32_array(node,
						     "google,chg-cc-limits",
						     p->cc_limits,
						     rows * p->n_volt);
	if (ret)
		goto out;

	/* percent of the pack's mAh, to uA: pct * mAh * 10 */
	for (i = 0; i < rows * p->n_volt; i++)
		p->cc_limits[i] = p->cc_limits[i] * (capacity_ua / 1000) * 10;

out:
	fwnode_handle_put(node);
	return ret;
}

static void psy_work_item(struct work_struct *work)
{
	struct max77779_charger *chg =
		container_of(work, struct max77779_charger, psy_work.work);
	union power_supply_propval current_limit, online;
	int ret;

	ret = power_supply_get_property(chg->tcpm_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX,
					&current_limit);
	if (ret) {
		dev_err(chg->dev, "failed to get CURRENT_MAX: %d\n", ret);
		goto retry;
	}

	ret = power_supply_get_property(chg->tcpm_psy, POWER_SUPPLY_PROP_ONLINE,
					&online);
	if (ret) {
		dev_err(chg->dev, "failed to get ONLINE: %d\n", ret);
		goto retry;
	}

	if (online.intval) {
		int ilim_ua = current_limit.intval > 0 ? current_limit.intval
						      : CHG_ILIM_FALLBACK_UA;

		ret = set_input_current_limit(chg, ilim_ua);
		if (ret) {
			dev_err(chg->dev, "failed to set ILIM: %d\n", ret);
			goto retry;
		}

		charger_set_mode(chg, MAX77779_CHGR_MODE_CHGR_BUCK_ON);

		if (chg->fg_psy)
			mod_delayed_work(system_wq, &chg->tier_work, 0);
	} else {
		cancel_delayed_work(&chg->tier_work);
		chg->tier_temp_idx = -1;
		chg->tier_volt_idx = -1;
		charger_set_mode(chg, MAX77779_CHGR_MODE_ALL_OFF);
	}

	power_supply_changed(chg->psy);

	scoped_guard(mutex, &chg->retry_lock)
		chg->psy_work_retry_cnt = 0;

	return;

retry:
	scoped_guard(mutex, &chg->retry_lock) {
		if (chg->psy_work_retry_cnt >= MAX_NUM_RETRIES) {
			dev_err(chg->dev, "psy work failed, giving up\n");
			return;
		}

		++chg->psy_work_retry_cnt;
		schedule_delayed_work(&chg->psy_work,
				      msecs_to_jiffies(PSY_WORK_RETRY_DELAY_MS));
	}
}

static int max77779_match_tcpm_psy(struct power_supply *psy, void *data)
{
	struct max77779_charger *chg = data;

	if (!strnstr(psy->desc->name, "tcpm-source", strlen("tcpm-source")))
		return 0;

	chg->tcpm_psy = psy;
	return 1;
}

static int max77779_psy_changed(struct notifier_block *nb, unsigned long evt,
				void *data)
{
	struct max77779_charger *chg = container_of(nb, struct max77779_charger,
						    nb);
	struct power_supply *psy = data;

	if (evt != PSY_EVENT_PROP_CHANGED ||
	    !max77779_match_tcpm_psy(psy, chg))
		return NOTIFY_OK;

	scoped_guard(mutex, &chg->retry_lock)
		chg->psy_work_retry_cnt = 0;

	schedule_delayed_work(&chg->psy_work, 0);

	return NOTIFY_OK;
}

static void max77779_unreg_psy_notifier(void *nb)
{
	power_supply_unreg_notifier(nb);
}

static int max77779_charger_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	struct power_supply_config psy_cfg = { };
	struct power_supply_battery_info *info;
	struct device *dev = &pdev->dev;
	struct max77779_charger *chg;
	u32 fast_chg_curr, fv;
	u32 regval;
	int ret;

	device_set_of_node_from_dev(dev, dev->parent);

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->regmap = dev_get_regmap(dev->parent, "charger");
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "Missing charger regmap\n");

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize lock\n");

	ret = devm_mutex_init(dev, &chg->retry_lock);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize retry_lock\n");

	chg->ext_bst_ctl = devm_gpiod_get_optional(dev, "extbst", GPIOD_OUT_LOW);
	if (IS_ERR(chg->ext_bst_ctl))
		return dev_err_probe(dev, PTR_ERR(chg->ext_bst_ctl),
				     "Failed to get extbst GPIO\n");

	/* Cache the current MODE; the external boost starts disabled (above). */
	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00, &regval);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read CHG_CNFG_00\n");

	chg->mode = FIELD_GET(MAX77779_CHGR_REG_CHG_CNFG_00_MODE, regval);

	config.dev = dev;
	config.driver_data = chg;
	config.of_node = dev_of_node(dev);

	chg->otg_rdev = devm_regulator_register(dev, &max77779_otg_desc,
						&config);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->otg_rdev),
				     "Failed to register OTG regulator\n");

	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.drv_data = chg;
	chg->psy = devm_power_supply_register(dev, &max77779_charger_desc,
					      &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "Failed to register psy\n");

	if (power_supply_get_battery_info(chg->psy, &info)) {
		fv = CHG_FV_DEFAULT_UV;
		fast_chg_curr = CHG_CC_DEFAULT_UA;
	} else {
		fv = info->constant_charge_voltage_max_uv;
		fast_chg_curr = info->constant_charge_current_max_ua;

		/*
		 * With a charge table the limits above are only a ceiling:
		 * tier_work_item() walks the table and lowers both to what the
		 * battery's temperature and voltage allow. Without one, or
		 * without a gauge to read, they stand as the fixed setting.
		 */
		if (info->charge_full_design_uah > 0 &&
		    !max77779_charger_parse_profile(chg,
						    info->charge_full_design_uah)) {
			chg->fg_psy = devm_power_supply_get_by_reference(dev,
							       "fuel-gauge");
			if (IS_ERR(chg->fg_psy)) {
				dev_warn(dev,
					 "no fuel-gauge (%ld); charging at the profile ceiling\n",
					 PTR_ERR(chg->fg_psy));
				chg->fg_psy = NULL;
			}
		}
	}

	ret = set_fast_charge_current(chg, fast_chg_curr);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set fast charge current\n");

	ret = set_float_voltage(chg, fv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set float voltage\n");

	ret = unlock_prot_regs(chg, true);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to unlock CHGPROT\n");

	/* Charge from CHGIN only; there is no wireless input on this board. */
	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_12,
				 MAX77779_CHGR_REG_CHG_CNFG_12_WCINSEL, 0);
	if (ret)
		goto relock;

	ret = unlock_prot_regs(chg, false);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to relock CHGPROT\n");

	chg->tier_temp_idx = -1;
	chg->tier_volt_idx = -1;
	ret = devm_delayed_work_autocancel(dev, &chg->tier_work, tier_work_item);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &chg->psy_work, psy_work_item);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init psy work\n");

	chg->nb.notifier_call = max77779_psy_changed;
	ret = power_supply_reg_notifier(&chg->nb);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register notifier\n");

	ret = devm_add_action_or_reset(dev, max77779_unreg_psy_notifier,
				       &chg->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add notifier unregister\n");

	/*
	 * A source attached before this probe emits no further notifier event,
	 * so look for the tcpm psy once here.
	 */
	power_supply_for_each_psy(chg, max77779_match_tcpm_psy);
	if (chg->tcpm_psy)
		schedule_delayed_work(&chg->psy_work, 0);

	return 0;

relock:
	unlock_prot_regs(chg, false);
	return dev_err_probe(dev, ret, "Failed to configure charger\n");
}

static const struct platform_device_id max77779_charger_id[] = {
	{ .name = "max77779-charger", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77779_charger_id);

static struct platform_driver max77779_charger_driver = {
	.driver = {
		.name = "max77779-charger",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = max77779_charger_probe,
	.id_table = max77779_charger_id,
};
module_platform_driver(max77779_charger_driver);

MODULE_DESCRIPTION("Maxim MAX77779 charger OTG regulator driver");
MODULE_LICENSE("GPL");
