// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google Inc
 * Copyright 2025 Linaro Ltd.
 *
 * Core driver for Maxim MAX77779 companion PMIC for USB Type-C
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77779.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/regmap.h>

/* Chip ID as per MAX77779_PMIC_REG_PMIC_ID */
enum {
	MAX77779_CHIP_ID = 0x79,
};

enum max77779_i2c_subdev_id {
	/*
	 * These are arbitrary and simply used to match struct
	 * max77779_i2c_subdev entries to the regmap pointers in struct
	 * max77779 during probe().
	 */
	MAX77779_I2C_SUBDEV_ID_MAXQ,
	MAX77779_I2C_SUBDEV_ID_CHARGER,
};

struct max77779_i2c_subdev {
	enum max77779_i2c_subdev_id id;
	const struct regmap_config *cfg;
	u16 i2c_address;
};

static const struct regmap_range max77779_top_registers[] = {
	regmap_reg_range(0x00, 0x02), /* PMIC_ID / PMIC_REVISION / OTP_REVISION */
	regmap_reg_range(0x22, 0x24), /* INTSRC / INTSRCMASK / TOPSYS_INT */
	regmap_reg_range(0x26, 0x26), /* TOPSYS_INT_MASK */
	regmap_reg_range(0x40, 0x40), /* I2C_CNFG */
	regmap_reg_range(0x50, 0x51), /* SWRESET / CONTROL_FG */
};

static const struct regmap_range max77779_top_ro_registers[] = {
	regmap_reg_range(0x00, 0x02),
	regmap_reg_range(0x22, 0x22),
};

static const struct regmap_range max77779_top_volatile_registers[] = {
	regmap_reg_range(0x22, 0x22),
	regmap_reg_range(0x24, 0x24),
};

static const struct regmap_access_table max77779_top_wr_table = {
	.yes_ranges = max77779_top_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_top_registers),
	.no_ranges = max77779_top_ro_registers,
	.n_no_ranges = ARRAY_SIZE(max77779_top_ro_registers),
};

static const struct regmap_access_table max77779_top_rd_table = {
	.yes_ranges = max77779_top_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_top_registers),
};

static const struct regmap_access_table max77779_top_volatile_table = {
	.yes_ranges = max77779_top_volatile_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_top_volatile_registers),
};

static const struct regmap_config max77779_regmap_config_top = {
	.name = "top",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77779_PMIC_REG_CONTROL_FG,
	.wr_table = &max77779_top_wr_table,
	.rd_table = &max77779_top_rd_table,
	.volatile_table = &max77779_top_volatile_table,
	.num_reg_defaults_raw = MAX77779_PMIC_REG_CONTROL_FG + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range max77779_maxq_registers[] = {
	regmap_reg_range(0x60, 0x73), /* Device ID, Rev, INTx, STATUSx, MASKx */
	regmap_reg_range(0x81, 0xa1), /* AP_DATAOUTx */
	regmap_reg_range(0xb1, 0xd1), /* AP_DATAINx */
	regmap_reg_range(0xe0, 0xe0), /* UIC_SWRST */
};

static const struct regmap_range max77779_maxq_ro_registers[] = {
	regmap_reg_range(0x60, 0x63), /* Device ID, Rev */
	regmap_reg_range(0x68, 0x6f), /* STATUSx */
	regmap_reg_range(0xb1, 0xd1),
};

static const struct regmap_range max77779_maxq_volatile_registers[] = {
	regmap_reg_range(0x64, 0x6f), /* INTx, STATUSx */
	regmap_reg_range(0xb1, 0xd1),
	regmap_reg_range(0xe0, 0xe0),
};

static const struct regmap_access_table max77779_maxq_wr_table = {
	.yes_ranges = max77779_maxq_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_maxq_registers),
	.no_ranges = max77779_maxq_ro_registers,
	.n_no_ranges = ARRAY_SIZE(max77779_maxq_ro_registers),
};

static const struct regmap_access_table max77779_maxq_rd_table = {
	.yes_ranges = max77779_maxq_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_maxq_registers),
};

static const struct regmap_access_table max77779_maxq_volatile_table = {
	.yes_ranges = max77779_maxq_volatile_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_maxq_volatile_registers),
};

static const struct regmap_config max77779_regmap_config_maxq = {
	.name = "maxq",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77779_MAXQ_REG_UIC_SWRST,
	.wr_table = &max77779_maxq_wr_table,
	.rd_table = &max77779_maxq_rd_table,
	.volatile_table = &max77779_maxq_volatile_table,
	.num_reg_defaults_raw = MAX77779_MAXQ_REG_UIC_SWRST + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range max77779_charger_registers[] = {
	regmap_reg_range(0xb0, 0xcc),
};

static const struct regmap_range max77779_charger_ro_registers[] = {
	regmap_reg_range(0xb4, 0xb8), /* INT_OK, DETAILS_0x */
};

static const struct regmap_range max77779_charger_volatile_registers[] = {
	regmap_reg_range(0xb0, 0xb1), /* INTx */
	regmap_reg_range(0xb4, 0xb8),
};

static const struct regmap_access_table max77779_charger_wr_table = {
	.yes_ranges = max77779_charger_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_charger_registers),
	.no_ranges = max77779_charger_ro_registers,
	.n_no_ranges = ARRAY_SIZE(max77779_charger_ro_registers),
};

static const struct regmap_access_table max77779_charger_rd_table = {
	.yes_ranges = max77779_charger_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_charger_registers),
};

static const struct regmap_access_table max77779_charger_volatile_table = {
	.yes_ranges = max77779_charger_volatile_registers,
	.n_yes_ranges = ARRAY_SIZE(max77779_charger_volatile_registers),
};

static const struct regmap_config max77779_regmap_config_charger = {
	.name = "charger",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77779_CHGR_REG_CHG_CNFG_19,
	.wr_table = &max77779_charger_wr_table,
	.rd_table = &max77779_charger_rd_table,
	.volatile_table = &max77779_charger_volatile_table,
	.num_reg_defaults_raw = MAX77779_CHGR_REG_CHG_CNFG_19 + 1,
	.cache_type = REGCACHE_FLAT,
};

/*
 * Interrupts - with the following interrupt hierarchy:
 *   pmic IRQs (INTSRC)
 *     - MAXQ_INT: MaxQ IRQs
 *       - UIC_INT1
 *         - APCmdResI
 *         - SysMsgI
 *         - GPIOxI
 *     - TOPSYS_INT: topsys
 *       - TOPSYS_INT
 *         - TSHDN_INT
 *         - SYSOVLO_INT
 *         - SYSUVLO_INT
 *         - FSHIP_NOT_RD
 *     - CHGR_INT: charger
 *       - CHG_INT
 *       - CHG_INT2
 */
enum {
	MAX77779_INT_MAXQ,
	MAX77779_INT_TOPSYS,
	MAX77779_INT_CHGR,
};

enum {
	MAX77779_TOPSYS_INT_TSHDN,
	MAX77779_TOPSYS_INT_SYSOVLO,
	MAX77779_TOPSYS_INT_SYSUVLO,
	MAX77779_TOPSYS_INT_FSHIP_NOT_RD,
};

enum {
	MAX77779_MAXQ_INT_APCMDRESI,
	MAX77779_MAXQ_INT_SYSMSGI,
	MAX77779_MAXQ_INT_GPIO,
	MAX77779_MAXQ_INT_UIC1,
	MAX77779_MAXQ_INT_UIC2,
	MAX77779_MAXQ_INT_UIC3,
	MAX77779_MAXQ_INT_UIC4,
};

enum {
	MAX77779_CHARGER_INT_1,
	MAX77779_CHARGER_INT_2,
};

static const struct regmap_irq max77779_pmic_irqs[] = {
	REGMAP_IRQ_REG(MAX77779_INT_MAXQ, 0, MAX77779_PMIC_REG_INTSRC_MAXQ),
	REGMAP_IRQ_REG(MAX77779_INT_TOPSYS, 0, MAX77779_PMIC_REG_INTSRC_TOPSYS),
	REGMAP_IRQ_REG(MAX77779_INT_CHGR, 0, MAX77779_PMIC_REG_INTSRC_CHGR),
};

static const struct regmap_irq max77779_maxq_irqs[] = {
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_APCMDRESI, 0, MAX77779_MAXQ_REG_UIC_INT1_APCMDRESI),
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_SYSMSGI, 0, MAX77779_MAXQ_REG_UIC_INT1_SYSMSGI),
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_GPIO, 0, GENMASK(1, 0)),
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_UIC1, 0, GENMASK(5, 2)),
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_UIC2, 1, GENMASK(7, 0)),
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_UIC3, 2, GENMASK(7, 0)),
	REGMAP_IRQ_REG(MAX77779_MAXQ_INT_UIC4, 3, GENMASK(7, 0)),
};

static const struct regmap_irq max77779_topsys_irqs[] = {
	REGMAP_IRQ_REG(MAX77779_TOPSYS_INT_TSHDN, 0, MAX77779_PMIC_REG_TOPSYS_INT_TSHDN),
	REGMAP_IRQ_REG(MAX77779_TOPSYS_INT_SYSOVLO, 0, MAX77779_PMIC_REG_TOPSYS_INT_SYSOVLO),
	REGMAP_IRQ_REG(MAX77779_TOPSYS_INT_SYSUVLO, 0, MAX77779_PMIC_REG_TOPSYS_INT_SYSUVLO),
	REGMAP_IRQ_REG(MAX77779_TOPSYS_INT_FSHIP_NOT_RD, 0, MAX77779_PMIC_REG_TOPSYS_INT_FSHIP),
};

static const struct regmap_irq max77779_chgr_irqs[] = {
	REGMAP_IRQ_REG(MAX77779_CHARGER_INT_1, 0, GENMASK(7, 0)),
	REGMAP_IRQ_REG(MAX77779_CHARGER_INT_2, 1, GENMASK(7, 0)),
};

static const struct regmap_irq_chip max77779_pmic_irq_chip = {
	.name = "max77779-pmic",
	/* INTSRC is read-only and doesn't require clearing */
	.status_base = MAX77779_PMIC_REG_INTSRC,
	.mask_base = MAX77779_PMIC_REG_INTSRCMASK,
	.num_regs = 1,
	.irqs = max77779_pmic_irqs,
	.num_irqs = ARRAY_SIZE(max77779_pmic_irqs),
};

/*
 * We can let regmap-irq auto-ack the topsys interrupt bits as required, but
 * for all others the individual drivers need to know which interrupt bit
 * exactly is set inside their interrupt handlers, and therefore we can not set
 * .ack_base for those.
 */
static const struct regmap_irq_chip max77779_maxq_irq_chip = {
	.name = "max77779-maxq",
	.domain_suffix = "MAXQ",
	.status_base = MAX77779_MAXQ_REG_UIC_INT1,
	.mask_base = MAX77779_MAXQ_REG_UIC_INT1_M,
	.num_regs = 4,
	.irqs = max77779_maxq_irqs,
	.num_irqs = ARRAY_SIZE(max77779_maxq_irqs),
};

static const struct regmap_irq_chip max77779_topsys_irq_chip = {
	.name = "max77779-topsys",
	.domain_suffix = "TOPSYS",
	.status_base = MAX77779_PMIC_REG_TOPSYS_INT,
	.mask_base = MAX77779_PMIC_REG_TOPSYS_INT_MASK,
	.ack_base = MAX77779_PMIC_REG_TOPSYS_INT,
	.num_regs = 1,
	.irqs = max77779_topsys_irqs,
	.num_irqs = ARRAY_SIZE(max77779_topsys_irqs),
};

static const struct regmap_irq_chip max77779_chrg_irq_chip = {
	.name = "max77779-chgr",
	.domain_suffix = "CHGR",
	.status_base = MAX77779_CHGR_REG_CHG_INT,
	.mask_base = MAX77779_CHGR_REG_CHG_INT_MASK,
	.num_regs = 2,
	.irqs = max77779_chgr_irqs,
	.num_irqs = ARRAY_SIZE(max77779_chgr_irqs),
};

static const struct max77779_i2c_subdev max77779_i2c_subdevs[] = {
	{
		.id = MAX77779_I2C_SUBDEV_ID_MAXQ,
		.cfg = &max77779_regmap_config_maxq,
		/* I2C address is same as for sub-block 'top' */
	},
	{
		.id = MAX77779_I2C_SUBDEV_ID_CHARGER,
		.cfg = &max77779_regmap_config_charger,
		.i2c_address = 0x69,
	},
};

static const struct resource max77779_gpio_resources[] = {
	DEFINE_RES_IRQ_NAMED(MAX77779_MAXQ_INT_GPIO, "GPI"),
};

static const struct resource max77779_charger_resources[] = {
	DEFINE_RES_IRQ_NAMED(MAX77779_CHARGER_INT_1, "INT1"),
	DEFINE_RES_IRQ_NAMED(MAX77779_CHARGER_INT_2, "INT2"),
};

static const struct mfd_cell max77779_cells[] = {
	MFD_CELL_OF("max77779-nvmem", NULL, NULL, 0, 0,
		    "maxim,max77779-nvmem"),
};

static const struct mfd_cell max77779_maxq_cells[] = {
	MFD_CELL_OF("max77779-gpio", max77779_gpio_resources, NULL, 0, 0,
		    "maxim,max77779-gpio"),
};

static const struct mfd_cell max77779_charger_cells[] = {
	MFD_CELL_RES("max77779-charger", max77779_charger_resources),
};

int max77779_maxq_command(struct max77779 *max77779,
			  const struct max77779_maxq_command *cmd,
			  struct max77779_maxq_response *rsp)
{
	DEFINE_FLEX(struct max77779_maxq_response, _rsp, rsp, length, 1);
	struct device *dev = regmap_get_device(max77779->regmap_maxq);
	static const unsigned int timeout_ms = 200;
	int ret;

	if (cmd->length > MAX77779_MAXQ_OPCODE_MAXLENGTH)
		return -EINVAL;

	/*
	 * As a convenience for API users when issuing simple commands, rsp is
	 * allowed to be NULL. In that case we need a temporary here to write
	 * the response to, as we need to verify that the command was indeed
	 * completed correctly.
	 */
	if (!rsp)
		rsp = _rsp;

	if (!rsp->length || rsp->length > MAX77779_MAXQ_OPCODE_MAXLENGTH)
		return -EINVAL;

	guard(mutex)(&max77779->maxq_lock);

	reinit_completion(&max77779->cmd_done);

	/*
	 * MaxQ latches the message when the DATAOUT32 register is written. If
	 * cmd->length is shorter we still need to write 0 to it.
	 */
	ret = regmap_bulk_write(max77779->regmap_maxq,
				MAX77779_MAXQ_REG_AP_DATAOUT0, cmd->cmd,
				cmd->length);
	if (!ret && cmd->length < MAX77779_MAXQ_OPCODE_MAXLENGTH)
		ret = regmap_write(max77779->regmap_maxq,
				   MAX77779_MAXQ_REG_AP_DATAOUT32, 0);
	if (ret) {
		dev_err(dev, "writing command failed: %d\n", ret);
		return ret;
	}

	/* Wait for response from MaxQ */
	if (!wait_for_completion_timeout(&max77779->cmd_done,
					 msecs_to_jiffies(timeout_ms))) {
		dev_err(dev, "timed out waiting for response\n");
		return -ETIMEDOUT;
	}

	ret = regmap_bulk_read(max77779->regmap_maxq,
			       MAX77779_MAXQ_REG_AP_DATAIN0,
			       rsp->rsp, rsp->length);
	if (ret) {
		dev_err(dev, "reading response failed: %d\n", ret);
		return ret;
	}

	/*
	 * As per the protocol, the first byte of the reply will match the
	 * request.
	 */
	if (cmd->cmd[0] != rsp->rsp[0]) {
		dev_err(dev, "unexpected opcode response for %#.2x: %*ph\n",
			cmd->cmd[0], (int)rsp->length, rsp->rsp);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_maxq_command);

static irqreturn_t apcmdres_irq_handler(int irq, void *irq_data)
{
	struct max77779 *max77779 = irq_data;

	regmap_write(max77779->regmap_maxq, MAX77779_MAXQ_REG_UIC_INT1,
		     MAX77779_MAXQ_REG_UIC_INT1_APCMDRESI);

	complete(&max77779->cmd_done);

	return IRQ_HANDLED;
}

static int max77779_create_i2c_subdev(struct i2c_client *client,
				      struct max77779 *max77779,
				      const struct max77779_i2c_subdev *sd)
{
	struct i2c_client *sub;
	struct regmap *regmap;
	int ret;

	/*
	 * If 'sd' has an I2C address, 'sub' will be assigned a new 'dummy'
	 * device, otherwise use it as-is.
	 */
	sub = client;
	if (sd->i2c_address) {
		sub = devm_i2c_new_dummy_device(&client->dev,
						client->adapter,
						sd->i2c_address);

		if (IS_ERR(sub))
			return dev_err_probe(&client->dev, PTR_ERR(sub),
					"failed to claim I2C device %s\n",
					sd->cfg->name);
	}

	regmap = devm_regmap_init_i2c(sub, sd->cfg);
	if (IS_ERR(regmap))
		return dev_err_probe(&sub->dev, PTR_ERR(regmap),
				     "regmap init for '%s' failed\n",
				     sd->cfg->name);

	ret = regmap_attach_dev(&client->dev, regmap, sd->cfg);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "regmap attach of '%s' failed\n",
				     sd->cfg->name);

	if (sd->id == MAX77779_I2C_SUBDEV_ID_MAXQ)
		max77779->regmap_maxq = regmap;
	else if (sd->id == MAX77779_I2C_SUBDEV_ID_CHARGER)
		max77779->regmap_charger = regmap;

	return 0;
}

static int max77779_add_chained_irq_chip(struct device *dev,
					 struct regmap *regmap,
					 int pirq,
					 struct regmap_irq_chip_data *parent,
					 const struct regmap_irq_chip *chip,
					 struct regmap_irq_chip_data **data)
{
	int irq, ret;

	irq = regmap_irq_get_virq(parent, pirq);
	if (irq < 0)
		return dev_err_probe(dev, irq,
				     "failed to get parent vIRQ(%d) for chip %s\n",
				     pirq, chip->name);

	ret = devm_regmap_add_irq_chip(dev, regmap, irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0, chip,
				       data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add %s IRQ chip\n",
				     chip->name);

	return 0;
}

static int max77779_add_chained_maxq(struct i2c_client *client,
				     struct max77779 *max77779,
				     struct regmap_irq_chip_data *parent)
{
	struct regmap_irq_chip_data *irq_chip_data;
	int apcmdres_irq;
	int ret;

	ret = max77779_add_chained_irq_chip(&client->dev,
					    max77779->regmap_maxq,
					    MAX77779_INT_MAXQ,
					    parent,
					    &max77779_maxq_irq_chip,
					    &irq_chip_data);
	if (ret)
		return ret;

	init_completion(&max77779->cmd_done);
	apcmdres_irq = regmap_irq_get_virq(irq_chip_data,
					   MAX77779_MAXQ_INT_APCMDRESI);

	ret = devm_request_threaded_irq(&client->dev, apcmdres_irq,
					NULL, apcmdres_irq_handler,
					IRQF_ONESHOT | IRQF_SHARED,
					dev_name(&client->dev), max77779);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "MAX77779_MAXQ_INT_APCMDRESI failed\n");

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   max77779_maxq_cells,
				   ARRAY_SIZE(max77779_maxq_cells),
				   NULL, 0,
				   regmap_irq_get_domain(irq_chip_data));
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to add child devices (MaxQ)\n");

	return 0;
}

static int max77779_add_chained_topsys(struct i2c_client *client,
				       struct max77779 *max77779,
				       struct regmap_irq_chip_data *parent)
{
	struct regmap_irq_chip_data *irq_chip_data;
	int ret;

	ret = max77779_add_chained_irq_chip(&client->dev,
					    max77779->regmap_top,
					    MAX77779_INT_TOPSYS,
					    parent,
					    &max77779_topsys_irq_chip,
					    &irq_chip_data);
	if (ret)
		return ret;

	return 0;
}

static int max77779_add_chained_charger(struct i2c_client *client,
					struct max77779 *max77779,
					struct regmap_irq_chip_data *parent)
{
	struct regmap_irq_chip_data *irq_chip_data;
	int ret;

	ret = max77779_add_chained_irq_chip(&client->dev,
					    max77779->regmap_charger,
					    MAX77779_INT_CHGR,
					    parent,
					    &max77779_chrg_irq_chip,
					    &irq_chip_data);
	if (ret)
		return ret;

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   max77779_charger_cells,
				   ARRAY_SIZE(max77779_charger_cells),
				   NULL, 0,
				   regmap_irq_get_domain(irq_chip_data));
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to add child devices (charger)\n");

	return 0;
}

static int max77779_probe(struct i2c_client *client)
{
	struct regmap_irq_chip_data *irq_chip_data_pmic;
	struct max77779 *max77779;
	unsigned int pmic_id;
	int ret;

	max77779 = devm_kzalloc(&client->dev, sizeof(*max77779), GFP_KERNEL);
	if (!max77779)
		return -ENOMEM;

	i2c_set_clientdata(client, max77779);

	max77779->regmap_top = devm_regmap_init_i2c(client,
						    &max77779_regmap_config_top);
	if (IS_ERR(max77779->regmap_top))
		return dev_err_probe(&client->dev, PTR_ERR(max77779->regmap_top),
				     "regmap init for '%s' failed\n",
				     max77779_regmap_config_top.name);

	ret = regmap_read(max77779->regmap_top,
			  MAX77779_PMIC_REG_PMIC_ID, &pmic_id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "unable to read device ID\n");

	if (pmic_id != MAX77779_CHIP_ID)
		return dev_err_probe(&client->dev, -ENODEV,
				     "unsupported device ID %#.2x (%d)\n",
				     pmic_id, pmic_id);

	ret = devm_mutex_init(&client->dev, &max77779->maxq_lock);
	if (ret)
		return ret;

	for (int i = 0; i < ARRAY_SIZE(max77779_i2c_subdevs); i++) {
		ret = max77779_create_i2c_subdev(client, max77779,
						 &max77779_i2c_subdevs[i]);
		if (ret)
			return ret;
	}

	ret = devm_regmap_add_irq_chip(&client->dev, max77779->regmap_top,
				       client->irq, IRQF_ONESHOT | IRQF_SHARED, 0,
				       &max77779_pmic_irq_chip,
				       &irq_chip_data_pmic);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to add IRQ chip '%s'\n",
				     max77779_pmic_irq_chip.name);

	ret = max77779_add_chained_maxq(client, max77779, irq_chip_data_pmic);
	if (ret)
		return ret;

	ret = max77779_add_chained_topsys(client, max77779, irq_chip_data_pmic);
	if (ret)
		return ret;

	ret = max77779_add_chained_charger(client, max77779, irq_chip_data_pmic);
	if (ret)
		return ret;

	return devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				    max77779_cells, ARRAY_SIZE(max77779_cells),
				    NULL, 0,
				    regmap_irq_get_domain(irq_chip_data_pmic));
}

static const struct i2c_device_id max77779_i2c_id[] = {
	{ "max77779" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77779_i2c_id);

static const struct of_device_id max77779_of_id[] = {
	{ .compatible = "maxim,max77779", },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_of_id);

static struct i2c_driver max77779_i2c_driver = {
	.driver = {
		.name = "max77779",
		.of_match_table = max77779_of_id,
	},
	.probe = max77779_probe,
	.id_table = max77779_i2c_id,
};
module_i2c_driver(max77779_i2c_driver);

MODULE_AUTHOR("André Draszik <andre.draszik@linaro.org>");
MODULE_DESCRIPTION("Maxim MAX77779 core driver");
MODULE_LICENSE("GPL");
