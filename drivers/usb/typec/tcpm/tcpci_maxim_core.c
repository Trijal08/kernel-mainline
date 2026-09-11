// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 - 2022, Google LLC
 *
 * MAXIM TCPCI based TCPC driver
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>

#include "tcpci_maxim.h"

#define PD_ACTIVITY_TIMEOUT_MS				10000

#define TCPC_VENDOR_ALERT				0x80
#define TCPC_VENDOR_USBSW_CTRL				0x93
#define TCPC_VENDOR_USBSW_CTRL_ENABLE_USB_DATA		0x9
#define TCPC_VENDOR_USBSW_CTRL_DISABLE_USB_DATA		0

#define TCPC_RECEIVE_BUFFER_COUNT_OFFSET		0
#define TCPC_RECEIVE_BUFFER_FRAME_TYPE_OFFSET		1
#define TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET		2

/*
 * LongMessage not supported, hence 32 bytes for buf to be read from RECEIVE_BUFFER.
 * DEVICE_CAPABILITIES_2.LongMessage = 0, the value in READABLE_BYTE_COUNT reg shall be
 * less than or equal to 31. Since, RECEIVE_BUFFER len = 31 + 1(READABLE_BYTE_COUNT).
 */
#define TCPC_RECEIVE_BUFFER_LEN				32

static const struct regmap_range max_tcpci_tcpci_range[] = {
	regmap_reg_range(0x00, 0x95)
};

static const struct regmap_access_table max_tcpci_tcpci_write_table = {
	.yes_ranges = max_tcpci_tcpci_range,
	.n_yes_ranges = ARRAY_SIZE(max_tcpci_tcpci_range),
};

static const struct regmap_config max_tcpci_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x95,
	.wr_table = &max_tcpci_tcpci_write_table,
};

static struct max_tcpci_chip *tdata_to_max_tcpci(struct tcpci_data *tdata)
{
	return container_of(tdata, struct max_tcpci_chip, data);
}

static void max_tcpci_init_regs(struct max_tcpci_chip *chip)
{
	u16 alert_mask = 0;
	int ret;

	ret = max_tcpci_write16(chip, TCPC_ALERT, 0xffff);
	if (ret < 0) {
		dev_err(chip->dev, "Error writing to TCPC_ALERT ret:%d\n", ret);
		return;
	}

	ret = max_tcpci_write16(chip, TCPC_VENDOR_ALERT, 0xffff);
	if (ret < 0) {
		dev_err(chip->dev, "Error writing to TCPC_VENDOR_ALERT ret:%d\n", ret);
		return;
	}

	ret = max_tcpci_write8(chip, TCPC_ALERT_EXTENDED, 0xff);
	if (ret < 0) {
		dev_err(chip->dev, "Unable to clear TCPC_ALERT_EXTENDED ret:%d\n", ret);
		return;
	}

	/* Enable VSAFE0V detection */
	ret = max_tcpci_write8(chip, TCPC_EXTENDED_STATUS_MASK, TCPC_EXTENDED_STATUS_VSAFE0V);
	if (ret < 0) {
		dev_err(chip->dev, "Unable to unmask TCPC_EXTENDED_STATUS_VSAFE0V ret:%d\n", ret);
		return;
	}

	/* Vconn Over Current Protection */
	ret = max_tcpci_write8(chip, TCPC_FAULT_STATUS_MASK, TCPC_FAULT_STATUS_MASK_VCONN_OC);
	if (ret < 0)
		return;

	alert_mask = (TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_DISCARDED |
		      TCPC_ALERT_TX_FAILED | TCPC_ALERT_RX_HARD_RST |
		      TCPC_ALERT_RX_STATUS | TCPC_ALERT_POWER_STATUS |
		      TCPC_ALERT_CC_STATUS |
		      TCPC_ALERT_EXTND | TCPC_ALERT_EXTENDED_STATUS |
		      TCPC_ALERT_VBUS_DISCNCT | TCPC_ALERT_RX_BUF_OVF |
		      TCPC_ALERT_FAULT);

	ret = max_tcpci_write16(chip, TCPC_ALERT_MASK, alert_mask);
	if (ret < 0) {
		dev_err(chip->dev,
			"Error enabling TCPC_ALERT: TCPC_ALERT_MASK write failed ret:%d\n", ret);
		return;
	}

	/* Enable vbus voltage monitoring and voltage alerts */
	ret = max_tcpci_write8(chip, TCPC_POWER_CTRL, 0);
	if (ret < 0) {
		dev_err(chip->dev, "Error writing to TCPC_POWER_CTRL ret:%d\n", ret);
		return;
	}

	ret = max_tcpci_write8(chip, TCPC_ALERT_EXTENDED_MASK, TCPC_SINK_FAST_ROLE_SWAP);
	if (ret < 0)
		return;
}

static void process_rx(struct max_tcpci_chip *chip, u16 status)
{
	struct pd_message msg;
	u8 count, frame_type, rx_buf[TCPC_RECEIVE_BUFFER_LEN];
	int ret, payload_index;
	u8 *rx_buf_ptr;
	enum tcpm_transmit_type rx_type;

	/*
	 * READABLE_BYTE_COUNT: Indicates the number of bytes in the RX_BUF_BYTE_x registers
	 * plus one (for the RX_BUF_FRAME_TYPE) Table 4-36.
	 * Read the count and frame type.
	 */
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, 2);
	if (ret < 0) {
		dev_err(chip->dev, "TCPC_RX_BYTE_CNT read failed ret:%d\n", ret);
		return;
	}

	count = rx_buf[TCPC_RECEIVE_BUFFER_COUNT_OFFSET];
	frame_type = rx_buf[TCPC_RECEIVE_BUFFER_FRAME_TYPE_OFFSET];

	switch (frame_type) {
	case TCPC_RX_BUF_FRAME_TYPE_SOP1:
		rx_type = TCPC_TX_SOP_PRIME;
		break;
	case TCPC_RX_BUF_FRAME_TYPE_SOP:
		rx_type = TCPC_TX_SOP;
		break;
	default:
		rx_type = TCPC_TX_SOP;
		break;
	}

	if (count == 0 || (frame_type != TCPC_RX_BUF_FRAME_TYPE_SOP &&
	    frame_type != TCPC_RX_BUF_FRAME_TYPE_SOP1)) {
		max_tcpci_write16(chip, TCPC_ALERT, TCPC_ALERT_RX_STATUS);
		dev_err(chip->dev, "%s\n", count ==  0 ? "error: count is 0" :
			"error frame_type is not SOP/SOP'");
		return;
	}

	if (count > sizeof(struct pd_message) + 1 ||
	    count + 1 > TCPC_RECEIVE_BUFFER_LEN) {
		dev_err(chip->dev, "Invalid TCPC_RX_BYTE_CNT %d\n", count);
		return;
	}

	/*
	 * Read count + 1 as RX_BUF_BYTE_x is hidden and can only be read through
	 * TCPC_RX_BYTE_CNT
	 */
	count += 1;
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, count);
	if (ret < 0) {
		dev_err(chip->dev, "Error: TCPC_RX_BYTE_CNT read failed: %d\n", ret);
		return;
	}

	rx_buf_ptr = rx_buf + TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET;
	msg.header = cpu_to_le16(*(u16 *)rx_buf_ptr);
	rx_buf_ptr = rx_buf_ptr + sizeof(msg.header);

	if (count < TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET + sizeof(msg.header) +
		    pd_header_cnt_le(msg.header) * sizeof(msg.payload[0])) {
		max_tcpci_write16(chip, TCPC_ALERT, TCPC_ALERT_RX_STATUS);
		dev_err(chip->dev, "Invalid TCPC_RX_BYTE_CNT %d for header cnt %d\n",
			count, pd_header_cnt_le(msg.header));
		return;
	}

	for (payload_index = 0; payload_index < pd_header_cnt_le(msg.header); payload_index++,
	     rx_buf_ptr += sizeof(msg.payload[0]))
		msg.payload[payload_index] = cpu_to_le32(*(u32 *)rx_buf_ptr);

	/*
	 * Read complete, clear RX status alert bit.
	 * Clear overflow as well if set.
	 */
	ret = max_tcpci_write16(chip, TCPC_ALERT,
				TCPC_ALERT_RX_STATUS | (status & TCPC_ALERT_RX_BUF_OVF));
	if (ret < 0)
		return;

	tcpm_pd_receive(chip->port, &msg, rx_type);
}

static int get_vbus_regulator_handle(struct max_tcpci_chip *chip)
{
	if (IS_ERR_OR_NULL(chip->vbus_reg)) {
		chip->vbus_reg = devm_regulator_get_exclusive(chip->dev,
							      "vbus");
		if (IS_ERR_OR_NULL(chip->vbus_reg)) {
			dev_err(chip->dev,
				"Failed to get vbus regulator handle\n");
			return -ENODEV;
		}
	}

	return 0;
}

static void max_tcpci_sourcing_vbus_work(struct work_struct *work)
{
	struct max_tcpci_chip *chip = container_of(work, struct max_tcpci_chip,
						   sourcing_vbus_work);

	tcpm_sourcing_vbus(chip->port);
}

static void max_tcpci_cancel_sourcing_vbus_work(void *data)
{
	cancel_work_sync(data);
}

static int max_tcpci_set_vbus(struct tcpci *tcpci, struct tcpci_data *tdata, bool source, bool sink)
{
	struct max_tcpci_chip *chip = tdata_to_max_tcpci(tdata);
	int ret;

	if (source && sink) {
		dev_err(chip->dev, "Both source and sink set\n");
		return -EINVAL;
	}

	ret = get_vbus_regulator_handle(chip);
	if (ret) {
		/*
		 * Regulator is not necessary for sink only applications. Return
		 * success in cases where sink mode is being modified.
		 */
		return source ? ret : 1;
	}

	if (source) {
		if (!regulator_is_enabled(chip->vbus_reg))
			ret = regulator_enable(chip->vbus_reg);
		if (ret >= 0)
			/*
			 * The "vbus" regulator is an external boost the TCPC
			 * cannot sense, so its POWER_STATUS never reports
			 * SOURCING_VBUS and tcpm would otherwise time out the
			 * source attach. Tell tcpm explicitly that VBUS is up.
			 * Deferred to a work item: tcpm calls set_vbus() with
			 * its port lock held, and tcpm_sourcing_vbus() retakes
			 * it.
			 */
			schedule_work(&chip->sourcing_vbus_work);
	} else {
		if (regulator_is_enabled(chip->vbus_reg))
			ret = regulator_disable(chip->vbus_reg);
	}

	return ret < 0 ? ret : 1;
}

static void process_power_status(struct max_tcpci_chip *chip)
{
	u8 pwr_status;
	int ret;

	ret = max_tcpci_read8(chip, TCPC_POWER_STATUS, &pwr_status);
	if (ret < 0)
		return;

	if (pwr_status == 0xff)
		max_tcpci_init_regs(chip);
	else if (pwr_status & TCPC_POWER_STATUS_SOURCING_VBUS)
		tcpm_sourcing_vbus(chip->port);
	else
		tcpm_vbus_change(chip->port);
}

static void max_tcpci_frs_sourcing_vbus(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	/*
	 * For Fast Role Swap case, Boost turns on autonomously without
	 * AP intervention, but, needs AP to enable source mode explicitly
	 * for AP to regain control.
	 */
	max_tcpci_set_vbus(tcpci, tdata, true, false);
}

static void process_tx(struct max_tcpci_chip *chip, u16 status)
{
	if (status & TCPC_ALERT_TX_SUCCESS)
		tcpm_pd_transmit_complete(chip->port, TCPC_TX_SUCCESS);
	else if (status & TCPC_ALERT_TX_DISCARDED)
		tcpm_pd_transmit_complete(chip->port, TCPC_TX_DISCARDED);
	else if (status & TCPC_ALERT_TX_FAILED)
		tcpm_pd_transmit_complete(chip->port, TCPC_TX_FAILED);

	/* Reinit regs as Hard reset sets them to default value */
	if ((status & TCPC_ALERT_TX_SUCCESS) && (status & TCPC_ALERT_TX_FAILED))
		max_tcpci_init_regs(chip);
}

/* Enable USB switches when partner is USB communications capable */
static void max_tcpci_set_partner_usb_comm_capable(struct tcpci *tcpci, struct tcpci_data *data,
						   bool capable)
{
	struct max_tcpci_chip *chip = tdata_to_max_tcpci(data);
	int ret;

	ret = max_tcpci_write8(chip, TCPC_VENDOR_USBSW_CTRL, capable ?
			       TCPC_VENDOR_USBSW_CTRL_ENABLE_USB_DATA :
			       TCPC_VENDOR_USBSW_CTRL_DISABLE_USB_DATA);

	if (ret < 0)
		dev_err(chip->dev, "Failed to enable USB switches");
}

/*
 * The same USB switches as a connector state, for ports that never learn the
 * partner's USB_COMM bit.  That bit only arrives with a PD contract, so on a
 * pd-disable port the switches would be turned off at port reset and never
 * turned back on.  TCPM drives TYPEC_STATE_USB from tcpm_set_roles() on every
 * attach and TYPEC_STATE_SAFE from tcpm_reset_port() on every detach,
 * independently of PD, which is the signal a non-PD port does get.
 */
/*
 * Route the connector's SBU pair to the SoC's DP AUX lines, and raise the
 * pull-up rail to the 3.3V AUX wants while they are routed. The max77759 path
 * is already corrected for the plug orientation, so there is no polarity to
 * choose here.
 *
 * Without this AUX never reaches the sink, and a DP partner that is otherwise
 * fully negotiated -- mode entered, lanes muxed, HPD asserted -- can never be
 * link trained.
 */
static int max_tcpci_sbu_switch(struct max_tcpci_chip *chip, bool dp)
{
	int ret;

	if (chip->sbu_reg && dp != chip->sbu_reg_enabled) {
		if (dp) {
			regulator_set_voltage(chip->sbu_reg, SBU_PULLUP_UV,
					      SBU_PULLUP_UV);
			ret = regulator_enable(chip->sbu_reg);
		} else {
			ret = regulator_disable(chip->sbu_reg);
		}

		if (ret < 0)
			dev_err(chip->dev, "Failed to %s the SBU pull-up: %d\n",
				dp ? "enable" : "disable", ret);
		else
			chip->sbu_reg_enabled = dp;
	}

	ret = max_tcpci_write8(chip, TCPC_VENDOR_SBUSW_CTRL,
			       dp ? SBUSW_PATH_1 : SBUSW_OFF);
	if (ret < 0)
		dev_err(chip->dev, "Failed to %s the SBU switch: %d\n",
			dp ? "close" : "open", ret);

	return ret;
}

static int max_tcpci_mux_set(struct typec_mux_dev *mux,
			     struct typec_mux_state *state)
{
	struct max_tcpci_chip *chip = typec_mux_get_drvdata(mux);
	int ret;

	switch (state->mode) {
	case TYPEC_DP_STATE_A:
	case TYPEC_DP_STATE_B:
	case TYPEC_DP_STATE_C:
	case TYPEC_DP_STATE_D:
	case TYPEC_DP_STATE_E:
	case TYPEC_DP_STATE_F:
		/*
		 * Every DP pin assignment needs AUX, and the ones that keep USB
		 * alongside leave the D+/D- switches to the USB_COMM path.
		 */
		return max_tcpci_sbu_switch(chip, true);
	default:
		break;
	}

	/*
	 * Leave the remaining alternate and accessory modes alone: they arrive
	 * only through PD, where set_partner_usb_comm_capable() above already
	 * owns the switches.
	 */
	if (state->mode != TYPEC_STATE_USB && state->mode != TYPEC_STATE_SAFE)
		return 0;

	/* Back off DP: unroute SBU and drop the rail again. */
	max_tcpci_sbu_switch(chip, false);

	ret = max_tcpci_write8(chip, TCPC_VENDOR_USBSW_CTRL,
			       state->mode == TYPEC_STATE_USB ?
			       TCPC_VENDOR_USBSW_CTRL_ENABLE_USB_DATA :
			       TCPC_VENDOR_USBSW_CTRL_DISABLE_USB_DATA);
	if (ret < 0)
		dev_err(chip->dev, "Failed to set USB switches for mode %lu\n",
			state->mode);

	return ret;
}

static irqreturn_t _max_tcpci_irq(struct max_tcpci_chip *chip, u16 status)
{
	u16 mask;
	int ret;
	u8 reg_status;

	/*
	 * Clear alert status for everything except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 */
	if (status & ~TCPC_ALERT_RX_STATUS) {
		mask = status & ~(TCPC_ALERT_RX_STATUS
				  | (status & TCPC_ALERT_RX_BUF_OVF));
		ret = max_tcpci_write16(chip, TCPC_ALERT, mask);
		if (ret < 0) {
			dev_err(chip->dev, "ALERT clear failed\n");
			return ret;
		}
	}

	if (status & TCPC_ALERT_RX_BUF_OVF && !(status & TCPC_ALERT_RX_STATUS)) {
		ret = max_tcpci_write16(chip, TCPC_ALERT, (TCPC_ALERT_RX_STATUS |
							  TCPC_ALERT_RX_BUF_OVF));
		if (ret < 0) {
			dev_err(chip->dev, "ALERT clear failed\n");
			return ret;
		}
	}

	if (status & TCPC_ALERT_FAULT) {
		ret = max_tcpci_read8(chip, TCPC_FAULT_STATUS, &reg_status);
		if (ret < 0)
			return ret;

		ret = max_tcpci_write8(chip, TCPC_FAULT_STATUS, reg_status);
		if (ret < 0)
			return ret;

		if (reg_status & TCPC_FAULT_STATUS_VCONN_OC) {
			chip->veto_vconn_swap = true;
			tcpm_port_error_recovery(chip->port);
		}
	}

	if (status & TCPC_ALERT_EXTND) {
		ret = max_tcpci_read8(chip, TCPC_ALERT_EXTENDED, &reg_status);
		if (ret < 0)
			return ret;

		ret = max_tcpci_write8(chip, TCPC_ALERT_EXTENDED, reg_status);
		if (ret < 0)
			return ret;

		if (reg_status & TCPC_SINK_FAST_ROLE_SWAP) {
			dev_info(chip->dev, "FRS Signal\n");
			tcpm_sink_frs(chip->port);
		}
	}

	if (status & TCPC_ALERT_EXTENDED_STATUS) {
		ret = max_tcpci_read8(chip, TCPC_EXTENDED_STATUS, (u8 *)&reg_status);
		if (ret >= 0 && (reg_status & TCPC_EXTENDED_STATUS_VSAFE0V))
			tcpm_vbus_change(chip->port);
	}

	if (status & TCPC_ALERT_RX_STATUS)
		process_rx(chip, status);

	if (status & TCPC_ALERT_VBUS_DISCNCT)
		tcpm_vbus_change(chip->port);

	if (status & TCPC_ALERT_CC_STATUS) {
		bool cc_handled = false;

		if (chip->contaminant_state == DETECTED || tcpm_port_is_toggling(chip->port)) {
			if (!max_contaminant_is_contaminant(chip, false, &cc_handled))
				tcpm_port_clean(chip->port);
		}
		if (!cc_handled)
			tcpm_cc_change(chip->port);
	}

	if (status & TCPC_ALERT_POWER_STATUS)
		process_power_status(chip);

	if (status & TCPC_ALERT_RX_HARD_RST) {
		tcpm_pd_hard_reset(chip->port);
		max_tcpci_init_regs(chip);
	}

	if (status & TCPC_ALERT_TX_SUCCESS || status & TCPC_ALERT_TX_DISCARDED || status &
	    TCPC_ALERT_TX_FAILED)
		process_tx(chip, status);

	return IRQ_HANDLED;
}

static irqreturn_t max_tcpci_irq(int irq, void *dev_id)
{
	struct max_tcpci_chip *chip = dev_id;
	u16 status;
	irqreturn_t irq_return = IRQ_HANDLED;
	int ret;

	if (!chip->port)
		return IRQ_HANDLED;

	ret = max_tcpci_read16(chip, TCPC_ALERT, &status);
	if (ret < 0) {
		dev_err(chip->dev, "ALERT read failed\n");
		return ret;
	}
	while (status) {
		irq_return = _max_tcpci_irq(chip, status);
		/* Do not return if a (new) ALERT is set (again). */
		ret = max_tcpci_read16(chip, TCPC_ALERT, &status);
		if (ret < 0)
			break;
	}

	return irq_return;
}

static irqreturn_t max_tcpci_isr(int irq, void *dev_id)
{
	struct max_tcpci_chip *chip = dev_id;

	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

	if (!chip->port)
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static int max_tcpci_start_toggling(struct tcpci *tcpci, struct tcpci_data *tdata,
				    enum typec_cc_status cc)
{
	struct max_tcpci_chip *chip = tdata_to_max_tcpci(tdata);

	max_tcpci_init_regs(chip);

	return 0;
}

static int tcpci_init(struct tcpci *tcpci, struct tcpci_data *data)
{
	/*
	 * Generic TCPCI overwrites the regs once this driver initializes
	 * them. Prevent this by returning -1.
	 */
	return -1;
}

static void max_tcpci_check_contaminant(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	struct max_tcpci_chip *chip = tdata_to_max_tcpci(tdata);
	bool cc_handled;

	if (!max_contaminant_is_contaminant(chip, true, &cc_handled))
		tcpm_port_clean(chip->port);
}

static bool max_tcpci_attempt_vconn_swap_discovery(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	struct max_tcpci_chip *chip = tdata_to_max_tcpci(tdata);

	if (chip->veto_vconn_swap) {
		chip->veto_vconn_swap = false;
		return false;
	}

	return true;
}

static void max_tcpci_unregister_tcpci_port(void *tcpci)
{
	tcpci_unregister_port(tcpci);
}

static int max_tcpci_ext_bst_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int max_tcpci_ext_bst_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max_tcpci_chip *chip = gpiochip_get_data(gc);
	u8 val;
	int ret;

	ret = max_tcpci_read8(chip, TCPC_VENDOR_EXTBST_CTRL, &val);
	if (ret)
		return ret;

	return !!(val & EXT_BST_EN);
}

static int max_tcpci_ext_bst_set(struct gpio_chip *gc, unsigned int offset,
				 int value)
{
	struct max_tcpci_chip *chip = gpiochip_get_data(gc);

	return max_tcpci_write8(chip, TCPC_VENDOR_EXTBST_CTRL,
				value ? EXT_BST_EN : 0);
}

static int max_tcpci_ext_bst_direction_output(struct gpio_chip *gc,
					      unsigned int offset, int value)
{
	return max_tcpci_ext_bst_set(gc, offset, value);
}

/*
 * The MAX77759 TCPC exposes a single GPIO (EXT_BST_EN) that enables an external
 * VBUS boost. On boards that source OTG VBUS this way (e.g. zumapro/komodo), it
 * is consumed by the charger's OTG regulator. Register it so that consumer can
 * drive it; harmless on boards that don't reference it.
 */
static int max_tcpci_gpio_init(struct max_tcpci_chip *chip)
{
	if (!IS_ENABLED(CONFIG_GPIOLIB))
		return 0;

	chip->gpio.label = "max77759_tcpc_gpio";
	chip->gpio.parent = chip->dev;
	chip->gpio.owner = THIS_MODULE;
	chip->gpio.base = -1;
	chip->gpio.ngpio = 1;
	chip->gpio.can_sleep = true;
	chip->gpio.get_direction = max_tcpci_ext_bst_get_direction;
	chip->gpio.direction_output = max_tcpci_ext_bst_direction_output;
	chip->gpio.get = max_tcpci_ext_bst_get;
	chip->gpio.set = max_tcpci_ext_bst_set;

	return devm_gpiochip_add_data(chip->dev, &chip->gpio, chip);
}

static void max_tcpci_unregister_mux(void *mux)
{
	typec_mux_unregister(mux);
}

static int max_tcpci_register_mux(struct max_tcpci_chip *chip)
{
	struct typec_mux_desc mux_desc;

	/*
	 * Only boards that link the connector back to this device expect the
	 * switches to follow the connector state; leave the rest on the
	 * USB_COMM path alone.  Register before the port, or its mode-switch
	 * lookup defers on a mux this same probe has not created yet.
	 */
	if (!device_property_present(chip->dev, "mode-switch"))
		return 0;

	mux_desc = (struct typec_mux_desc){
		.fwnode = dev_fwnode(chip->dev),
		.set = max_tcpci_mux_set,
		.drvdata = chip,
		.name = dev_name(chip->dev),
	};

	chip->mux = typec_mux_register(chip->dev, &mux_desc);
	if (IS_ERR(chip->mux))
		return dev_err_probe(chip->dev, PTR_ERR(chip->mux),
				     "USB switch mode-switch registration failed\n");

	return devm_add_action_or_reset(chip->dev, max_tcpci_unregister_mux,
					chip->mux);
}

static int max_tcpci_probe(struct i2c_client *client)
{
	int ret;
	struct max_tcpci_chip *chip;
	u8 power_status;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->data.regmap = devm_regmap_init_i2c(client, &max_tcpci_regmap_config);
	if (IS_ERR(chip->data.regmap))
		return dev_err_probe(&client->dev, PTR_ERR(chip->data.regmap),
				     "Regmap init failed\n");

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	ret = max_tcpci_read8(chip, TCPC_POWER_STATUS, &power_status);
	if (ret < 0)
		return dev_err_probe(&client->dev, ret,
				     "Failed to read TCPC_POWER_STATUS\n");

	/* Chip level tcpci callbacks */
	chip->data.set_vbus = max_tcpci_set_vbus;
	chip->data.start_drp_toggling = max_tcpci_start_toggling;
	chip->data.TX_BUF_BYTE_x_hidden = true;
	chip->data.init = tcpci_init;
	chip->data.frs_sourcing_vbus = max_tcpci_frs_sourcing_vbus;
	chip->data.auto_discharge_disconnect = true;
	chip->data.vbus_vsafe0v = true;
	chip->data.set_partner_usb_comm_capable = max_tcpci_set_partner_usb_comm_capable;
	chip->data.check_contaminant = max_tcpci_check_contaminant;
	chip->data.cable_comm_capable = true;
	chip->data.attempt_vconn_swap_discovery = max_tcpci_attempt_vconn_swap_discovery;

	INIT_WORK(&chip->sourcing_vbus_work, max_tcpci_sourcing_vbus_work);

	max_tcpci_init_regs(chip);

	ret = max_tcpci_register_mux(chip);
	if (ret)
		return ret;

	/*
	 * Optional: boards that route SBU to DP AUX may gate the pair behind a
	 * pull-up rail. Without one the switch still closes, which is all a
	 * board with an always-on rail needs.
	 *
	 * A pullup-supply naming a regulator its provider never registers
	 * defers this port forever, taking PD and charging with it -- describe
	 * the rail only once the PMIC driver actually has a descriptor for it.
	 */
	chip->sbu_reg = devm_regulator_get_optional(chip->dev, "pullup");
	if (IS_ERR(chip->sbu_reg)) {
		if (PTR_ERR(chip->sbu_reg) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		chip->sbu_reg = NULL;
	}

	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR(chip->tcpci))
		return dev_err_probe(&client->dev, PTR_ERR(chip->tcpci),
				     "TCPCI port registration failed\n");

        ret = devm_add_action_or_reset(&client->dev,
				       max_tcpci_unregister_tcpci_port,
				       chip->tcpci);
        if (ret)
                return ret;

	ret = devm_add_action_or_reset(&client->dev,
				       max_tcpci_cancel_sourcing_vbus_work,
				       &chip->sourcing_vbus_work);
	if (ret)
		return ret;

	chip->port = tcpci_get_tcpm_port(chip->tcpci);

	ret = devm_request_threaded_irq(&client->dev, client->irq, max_tcpci_isr, max_tcpci_irq,
					(IRQF_TRIGGER_LOW | IRQF_ONESHOT), dev_name(chip->dev),
					chip);
	if (ret < 0)
		return dev_err_probe(&client->dev, ret,
				     "IRQ initialization failed\n");

	ret = devm_device_init_wakeup(chip->dev);
	if (ret)
		return dev_err_probe(chip->dev, ret, "Failed to init wakeup\n");

	ret = max_tcpci_gpio_init(chip);
	if (ret)
		return dev_err_probe(chip->dev, ret, "Failed to init GPIO\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max_tcpci_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	if (client->irq && device_may_wakeup(dev))
		ret = disable_irq_wake(client->irq);

	return ret;
}

static int max_tcpci_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	if (client->irq && device_may_wakeup(dev))
		ret = enable_irq_wake(client->irq);

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max_tcpci_pm_ops, max_tcpci_suspend, max_tcpci_resume);

static const struct i2c_device_id max_tcpci_id[] = {
	{ .name = "maxtcpc" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max_tcpci_id);

static const struct of_device_id max_tcpci_of_match[] = {
	{ .compatible = "maxim,max33359", },
	{},
};
MODULE_DEVICE_TABLE(of, max_tcpci_of_match);

static struct i2c_driver max_tcpci_i2c_driver = {
	.driver = {
		.name = "maxtcpc",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = max_tcpci_of_match,
		.pm = &max_tcpci_pm_ops,
	},
	.probe = max_tcpci_probe,
	.id_table = max_tcpci_id,
};
module_i2c_driver(max_tcpci_i2c_driver);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("Maxim TCPCI based USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL v2");
