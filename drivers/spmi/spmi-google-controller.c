// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google GPIO bit-banged SPMI controller
 *
 * Copyright 2026 Trijal Saha
 *
 * A minimal single-master SPMI controller that bit-bangs the two-wire bus
 * (SCLK, SDATA) over a pair of GPIOs.  It implements the Extended Register
 * Read/Write Long commands (16-bit register address) used to reach a PMIC
 * that hangs off a bare GPIO pair rather than a dedicated SPMI block -- e.g.
 * the Exynos modem CP PMIC.  Data is clocked out on the rising edge and
 * sampled by the slave on the falling edge; every frame carries one trailing
 * odd-parity bit.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spmi.h>
#include <linux/types.h>

#define SPMI_GOOGLE_DEFAULT_DELAY_US	10

struct spmi_google {
	struct gpio_desc	*sclk;
	struct gpio_desc	*sdata;
	u32			delay_us;	/* per clock phase, ~1/(2*delay) Hz */
};

static void spmi_google_delay(struct spmi_google *g)
{
	udelay(g->delay_us);
}

static bool spmi_google_parity(u32 data, u32 num_bits)
{
	bool parity = 1;			/* SPMI uses odd parity */

	while (num_bits--) {
		parity ^= data & 0x1;
		data >>= 1;
	}
	return parity;
}

/* Clock out @num_bits of @data MSB-first. */
static void spmi_google_send(struct spmi_google *g, u32 data, u32 num_bits)
{
	unsigned int mask = BIT(num_bits);	/* start one bit too high */

	gpiod_set_value_cansleep(g->sclk, 0);
	while (mask >>= 1) {
		gpiod_set_value_cansleep(g->sclk, 1);
		gpiod_set_value_cansleep(g->sdata, !!(data & mask));
		spmi_google_delay(g);
		gpiod_set_value_cansleep(g->sclk, 0);
		spmi_google_delay(g);
	}
}

/* Frame = payload + trailing odd-parity bit. */
static void spmi_google_send_frame(struct spmi_google *g, u32 data, u8 num_bits)
{
	spmi_google_send(g, data << 1 | spmi_google_parity(data, num_bits),
			 num_bits + 1);
}

/* Command frame: sid[3:0] << 8 | command[7:0], 12 bits + parity. */
static void spmi_google_send_cmd_frame(struct spmi_google *g, u8 sid, u8 cmd)
{
	spmi_google_send_frame(g, (u16)((sid & 0xf) << 8 | cmd), 12);
}

/* Read back one 8-bit frame; returns false on a parity mismatch. */
static bool spmi_google_recv_frame(struct spmi_google *g, u8 *payload)
{
	u32 num_bits = 9;			/* 8 data + parity */
	u32 data = 0;
	bool parity;

	gpiod_set_value_cansleep(g->sclk, 0);
	while (num_bits--) {
		gpiod_set_value_cansleep(g->sclk, 1);
		data <<= 1;
		data |= !!gpiod_get_value(g->sdata);
		spmi_google_delay(g);
		gpiod_set_value_cansleep(g->sclk, 0);
		spmi_google_delay(g);
	}
	parity = data & 0x1;
	data >>= 1;
	*payload = (u8)data;
	return parity == spmi_google_parity(data, 8);
}

/* Sequence start condition. */
static void spmi_google_ssc(struct spmi_google *g)
{
	gpiod_direction_output(g->sdata, 0);
	gpiod_set_value_cansleep(g->sclk, 0);
	gpiod_set_value_cansleep(g->sdata, 1);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sdata, 0);
	spmi_google_delay(g);
}

static void spmi_google_bus_park(struct spmi_google *g)
{
	gpiod_set_value_cansleep(g->sclk, 1);
	gpiod_set_value_cansleep(g->sdata, 0);
	spmi_google_delay(g);
	gpiod_direction_input(g->sdata);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);
}

/* true = the slave acknowledged. */
static bool spmi_google_recv_ack(struct spmi_google *g)
{
	bool bit;

	spmi_google_bus_park(g);
	gpiod_set_value_cansleep(g->sclk, 1);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sclk, 0);
	bit = !!gpiod_get_value(g->sdata);
	spmi_google_delay(g);
	spmi_google_bus_park(g);
	return bit;
}

/*
 * Bus arbitration preamble (SPMI master request): assert SDATA across five
 * phases, one clock, a bus-park, the 'C'/'A' pattern, then four
 * master-priority-level clocks with SDATA released.
 */
static void spmi_google_arbitrate(struct spmi_google *g)
{
	int i;

	gpiod_direction_output(g->sdata, 0);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);
	spmi_google_delay(g);

	gpiod_set_value_cansleep(g->sdata, 1);
	for (i = 0; i < 5; i++)
		spmi_google_delay(g);

	/* first clock */
	gpiod_set_value_cansleep(g->sclk, 1);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);

	/* bus park cycle */
	gpiod_set_value_cansleep(g->sclk, 1);
	gpiod_set_value_cansleep(g->sdata, 0);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);

	/* c */
	gpiod_set_value_cansleep(g->sclk, 1);
	gpiod_set_value_cansleep(g->sdata, 1);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);

	/* a */
	gpiod_set_value_cansleep(g->sclk, 1);
	gpiod_set_value_cansleep(g->sdata, 0);
	gpiod_direction_input(g->sdata);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);

	/* xtra clock + mpl0..mpl2 */
	for (i = 0; i < 4; i++) {
		gpiod_set_value_cansleep(g->sclk, 1);
		spmi_google_delay(g);
		gpiod_set_value_cansleep(g->sclk, 0);
		spmi_google_delay(g);
	}

	/* mpl3 */
	gpiod_set_value_cansleep(g->sclk, 1);
	gpiod_direction_output(g->sdata, 1);
	spmi_google_delay(g);
	gpiod_set_value_cansleep(g->sclk, 0);
	spmi_google_delay(g);

	gpiod_set_value_cansleep(g->sdata, 0);
}

/* Drive both lines (start of a transaction). */
static void spmi_google_bus_claim(struct spmi_google *g)
{
	gpiod_direction_output(g->sclk, 0);
	gpiod_direction_output(g->sdata, 0);
}

/* Release both lines back to the bus. */
static void spmi_google_bus_release(struct spmi_google *g)
{
	gpiod_direction_input(g->sclk);
	gpiod_direction_input(g->sdata);
}

static int spmi_google_write_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid,
				 u16 addr, const u8 *buf, size_t len)
{
	struct spmi_google *g = spmi_controller_get_drvdata(ctrl);
	bool ack;
	size_t i;

	if (opcode != SPMI_CMD_EXT_WRITEL || len < 1 || len > 8)
		return -EOPNOTSUPP;

	spmi_google_bus_claim(g);
	spmi_google_arbitrate(g);
	spmi_google_ssc(g);
	spmi_google_send_cmd_frame(g, sid, opcode | (len - 1));
	spmi_google_send_frame(g, addr >> 8, 8);
	spmi_google_send_frame(g, addr & 0xff, 8);
	for (i = 0; i < len; i++)
		spmi_google_send_frame(g, buf[i], 8);
	ack = spmi_google_recv_ack(g);
	spmi_google_bus_release(g);

	return ack ? 0 : -EIO;
}

static int spmi_google_read_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid,
				u16 addr, u8 *buf, size_t len)
{
	struct spmi_google *g = spmi_controller_get_drvdata(ctrl);
	int ret = 0;
	size_t i;

	if (opcode != SPMI_CMD_EXT_READL || len < 1 || len > 8)
		return -EOPNOTSUPP;

	spmi_google_bus_claim(g);
	spmi_google_arbitrate(g);
	spmi_google_ssc(g);
	spmi_google_send_cmd_frame(g, sid, opcode | (len - 1));
	spmi_google_send_frame(g, addr >> 8, 8);
	spmi_google_send_frame(g, addr & 0xff, 8);
	spmi_google_bus_park(g);		/* turn the bus around to the slave */
	for (i = 0; i < len; i++) {
		if (!spmi_google_recv_frame(g, &buf[i])) {
			ret = -EIO;
			break;
		}
	}
	spmi_google_bus_park(g);
	spmi_google_bus_release(g);

	return ret;
}

static int spmi_google_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spmi_controller *ctrl;
	struct spmi_google *g;

	ctrl = devm_spmi_controller_alloc(dev, sizeof(*g));
	if (IS_ERR(ctrl))
		return PTR_ERR(ctrl);
	g = spmi_controller_get_drvdata(ctrl);

	g->delay_us = SPMI_GOOGLE_DEFAULT_DELAY_US;
	device_property_read_u32(dev, "delay-us", &g->delay_us);

	/* Idle with both lines released; a transaction claims them as outputs. */
	g->sclk = devm_gpiod_get(dev, "sclk", GPIOD_IN);
	if (IS_ERR(g->sclk))
		return dev_err_probe(dev, PTR_ERR(g->sclk), "no sclk gpio\n");
	g->sdata = devm_gpiod_get(dev, "sdata", GPIOD_IN);
	if (IS_ERR(g->sdata))
		return dev_err_probe(dev, PTR_ERR(g->sdata), "no sdata gpio\n");

	ctrl->read_cmd = spmi_google_read_cmd;
	ctrl->write_cmd = spmi_google_write_cmd;

	return devm_spmi_controller_add(dev, ctrl);
}

static const struct of_device_id spmi_google_of_match[] = {
	{ .compatible = "google,bitbang-spmi-controller" },
	{ }
};
MODULE_DEVICE_TABLE(of, spmi_google_of_match);

static struct platform_driver spmi_google_driver = {
	.probe = spmi_google_probe,
	.driver = {
		.name = "spmi-google-controller",
		.of_match_table = spmi_google_of_match,
	},
};
module_platform_driver(spmi_google_driver);

MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_DESCRIPTION("Google GPIO bit-banged SPMI controller");
MODULE_LICENSE("GPL");
