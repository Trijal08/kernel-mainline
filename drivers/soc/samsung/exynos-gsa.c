// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google GSA (on-die security core) mailbox driver.
 *
 * The GSA is the always-running root of trust on Tensor SoCs.  The AP talks to
 * it over a Samsung-style MCU mailbox: a doorbell interrupt each way plus 16
 * shared data registers carrying a {cmd, argc, args...} request and a
 * {cmd|RSP, err, argc, args...} response.  The GSA firmware is booted by the
 * boot ROM, so this driver loads no firmware - it only opens the channel.
 *
 * This is a minimal bring-up port of the vendor google-modules GSA driver
 * (gsa_core.c + gsa_mbox.c): just enough to probe and round-trip a command.
 * The Trusty (HWMGR) service link, the external GSC/Titan proxy, KDN/SJTAG/TPU
 * helpers, the log region, the char device and the pKVM S2MPU wakelock dance
 * are all omitted; the GSA S2MPU is a permissive google,s2mpu-v9 handled by its
 * own driver and is not needed for MMIO mailbox traffic.
 *
 * Copyright (C) 2019-2020 Google LLC.
 * Mainline port 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/soc/samsung/exynos-gsa.h>

/* Mailbox registers (base = the gsa-ns MMIO window). */
#define MBOX_INTCR0_REG		0x0024	/* GSA->AP interrupt clear */
#define MBOX_INTMR0_REG		0x0028	/* GSA->AP interrupt mask */
#define MBOX_INTMSR0_REG	0x0030	/* GSA->AP masked status */
#define MBOX_INTGR1_REG		0x0040	/* AP->GSA interrupt generate (doorbell) */
#define MBOX_SR_BASE_REG	0x0080	/* shared data registers */
#define MBOX_SR_REG(n)		(MBOX_SR_BASE_REG + (n) * 4)
#define MBOX_SR_NUM		16

/* Bit 0 of each interrupt group carries the request / response. */
#define MBOX_HOST_REQ_IRQ	0
#define MBOX_CLIENT_RSP_IRQ	0

/* Most commands answer in well under a second, but an image-authenticate/load
 * makes the GSA hash a multi-MB body, so allow generous headroom.  The point of
 * the cap is only to fail instead of hanging the (uninterruptible) caller if the
 * GSA is wedged or absent; the firmware is far faster than this in practice.
 */
#define GSA_MBOX_TIMEOUT_MS	10000

/* Mailbox command opcodes (subset; full set in the vendor gsa_mbox.h). */
enum gsa_mbox_cmd {
	GSA_MB_CMD_LOAD_AOC_FW_IMG	= 90,
	GSA_MB_CMD_AOC_CMD		= 91,
	GSA_MB_CMD_UNLOAD_AOC_FW_IMG	= 92,
	GSA_MB_CMD_SJTAG_GET_CHIP_ID	= 101,
};

/* Arguments to GSA_MB_CMD_AOC_CMD. */
#define GSA_AOC_GET_STATE	0
#define GSA_AOC_START		1

/* Response cmd word = request cmd with this bit set. */
#define GSA_MB_CMD_RSP		BIT(31)

/* Mailbox error codes returned in the response err word. */
enum gsa_mb_error {
	GSA_MB_OK			= 0,
	GSA_MB_ERR_INVALID_ARGS		= 1,
	GSA_MB_ERR_AUTH_FAILED		= 2,
	GSA_MB_ERR_BUSY			= 3,
	GSA_MB_ERR_ALREADY_RUNNING	= 4,
	GSA_MB_ERR_OUT_OF_RESOURCES	= 5,
	GSA_MB_ERR_BAD_HANDLE		= 6,
	GSA_MB_ERR_GENERIC		= 128,
	GSA_MB_ERR_INTERNAL		= 129,
	GSA_MB_ERR_TIMED_OUT		= 130,
	GSA_MB_ERR_BAD_STATE		= 131,
};

struct gsa_dev_state {
	struct device *dev;
	void __iomem *base;
	int irq;
	spinlock_t slock;		/* RMW of the interrupt-mask register */
	struct mutex mbox_lock;		/* one command in flight at a time */
	struct completion rsp;
	u32 exp_intmr0;			/* expected mask, re-synced if GSA reset us */
};

static void gsa_mbox_mask_irq0(struct gsa_dev_state *s, u32 mask)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&s->slock, flags);
	v = readl(s->base + MBOX_INTMR0_REG) | mask;
	writel(v, s->base + MBOX_INTMR0_REG);
	s->exp_intmr0 = v;
	spin_unlock_irqrestore(&s->slock, flags);
}

static void gsa_mbox_unmask_irq0(struct gsa_dev_state *s, u32 mask)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&s->slock, flags);
	v = readl(s->base + MBOX_INTMR0_REG) & ~mask;
	writel(v, s->base + MBOX_INTMR0_REG);
	s->exp_intmr0 = v;
	spin_unlock_irqrestore(&s->slock, flags);
}

static void gsa_mbox_sync_irq0(struct gsa_dev_state *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->slock, flags);
	if (readl(s->base + MBOX_INTMR0_REG) != s->exp_intmr0)
		writel(s->exp_intmr0, s->base + MBOX_INTMR0_REG);
	spin_unlock_irqrestore(&s->slock, flags);
}

static void gsa_mbox_clr_irq0(struct gsa_dev_state *s, u32 mask)
{
	writel(mask, s->base + MBOX_INTCR0_REG);
}

static irqreturn_t gsa_mbox_irq(int irq, void *data)
{
	struct gsa_dev_state *s = data;

	/* Re-assert our mask in case the GSA reset the block under us. */
	gsa_mbox_sync_irq0(s);

	if (readl(s->base + MBOX_INTMSR0_REG) & BIT(MBOX_CLIENT_RSP_IRQ)) {
		gsa_mbox_mask_irq0(s, BIT(MBOX_CLIENT_RSP_IRQ));
		complete(&s->rsp);
	}

	return IRQ_HANDLED;
}

static int gsa_mb_err_to_errno(u32 err)
{
	switch (err) {
	case GSA_MB_OK:
		return 0;
	case GSA_MB_ERR_BAD_HANDLE:
	case GSA_MB_ERR_INVALID_ARGS:
		return -EINVAL;
	case GSA_MB_ERR_BUSY:
		return -EBUSY;
	case GSA_MB_ERR_AUTH_FAILED:
		return -EACCES;
	case GSA_MB_ERR_OUT_OF_RESOURCES:
		return -ENOMEM;
	case GSA_MB_ERR_ALREADY_RUNNING:
		return -EEXIST;
	case GSA_MB_ERR_TIMED_OUT:
		return -ETIMEDOUT;
	default:
		return -EIO;
	}
}

/*
 * Send one mailbox command and collect the response.  Returns the number of
 * response args on success, or a negative errno.  Serialised by the caller.
 */
static int gsa_send_mbox_cmd(struct gsa_dev_state *s, u32 cmd,
			     const u32 *req_args, u32 req_argc,
			     u32 *rsp_args, u32 rsp_max_argc)
{
	u32 rsp_cmd, rsp_err, rsp_argc;
	unsigned long timeleft;
	int ret;
	u32 i;

	if (req_argc + 2 > MBOX_SR_NUM)
		return -EINVAL;
	if (req_argc && !req_args)
		return -EINVAL;
	if (rsp_max_argc && !rsp_args)
		return -EINVAL;

	mutex_lock(&s->mbox_lock);

	/* stage the request in the shared registers */
	writel(cmd, s->base + MBOX_SR_REG(0));
	writel(req_argc, s->base + MBOX_SR_REG(1));
	for (i = 0; i < req_argc; i++)
		writel(req_args[i], s->base + MBOX_SR_REG(i + 2));

	reinit_completion(&s->rsp);
	gsa_mbox_clr_irq0(s, BIT(MBOX_CLIENT_RSP_IRQ));
	gsa_mbox_unmask_irq0(s, BIT(MBOX_CLIENT_RSP_IRQ));

	/* ring the doorbell */
	writel(BIT(MBOX_HOST_REQ_IRQ), s->base + MBOX_INTGR1_REG);

	timeleft = wait_for_completion_timeout(&s->rsp,
					       msecs_to_jiffies(GSA_MBOX_TIMEOUT_MS));
	if (!timeleft) {
		dev_err(s->dev, "mbox cmd 0x%x timed out\n", cmd);
		ret = -ETIMEDOUT;
		goto out;
	}

	rsp_cmd = readl(s->base + MBOX_SR_REG(0));
	rsp_err = readl(s->base + MBOX_SR_REG(1));
	rsp_argc = readl(s->base + MBOX_SR_REG(2));

	if (rsp_cmd != (cmd | GSA_MB_CMD_RSP)) {
		dev_err(s->dev, "mbox cmd 0x%x: bad response cmd 0x%x\n",
			cmd, rsp_cmd);
		ret = -EIO;
		goto out;
	}
	if (rsp_err != GSA_MB_OK) {
		dev_err(s->dev, "mbox cmd 0x%x returned err %u\n", cmd, rsp_err);
		ret = gsa_mb_err_to_errno(rsp_err);
		goto out;
	}
	if (rsp_argc + 3 > MBOX_SR_NUM) {
		dev_err(s->dev, "mbox cmd 0x%x: malformed argc %u\n",
			cmd, rsp_argc);
		ret = -EIO;
		goto out;
	}

	if (rsp_argc > rsp_max_argc)
		rsp_argc = rsp_max_argc;
	for (i = 0; i < rsp_argc; i++)
		rsp_args[i] = readl(s->base + MBOX_SR_REG(i + 3));

	ret = rsp_argc;
out:
	gsa_mbox_clr_irq0(s, BIT(MBOX_CLIENT_RSP_IRQ));
	gsa_mbox_mask_irq0(s, BIT(MBOX_CLIENT_RSP_IRQ));
	mutex_unlock(&s->mbox_lock);
	return ret;
}

/**
 * gsa_load_aoc_fw_image() - have the GSA authenticate and load an AOC image
 * @gsa:    the GSA device, resolved from the consumer's "gsa-device" phandle
 * @header: DMA address of the image's 4K authentication header (coherent memory
 *          the GSA reads by DMA)
 * @body:   physical address of the image body, already staged where the GSA can
 *          read it (the AOC DRAM carveout)
 *
 * The GSA reads the header, verifies it over the body and, on success, unlocks
 * the AOC firmware.  Returns 0 on success or a negative errno; -EACCES means the
 * image failed authentication.
 */
int gsa_load_aoc_fw_image(struct device *gsa, dma_addr_t header, phys_addr_t body)
{
	struct gsa_dev_state *s = dev_get_drvdata(gsa);
	u32 args[4] = {
		lower_32_bits(header), upper_32_bits(header),
		lower_32_bits(body), upper_32_bits(body),
	};

	if (!s)
		return -ENODEV;

	return gsa_send_mbox_cmd(s, GSA_MB_CMD_LOAD_AOC_FW_IMG, args,
				 ARRAY_SIZE(args), NULL, 0);
}
EXPORT_SYMBOL_GPL(gsa_load_aoc_fw_image);

/**
 * gsa_aoc_start() - ask the GSA to take the AOC out of reset
 * @gsa: the GSA device
 *
 * The AOC's reset is in the secure domain and cannot be released by the
 * non-secure AP; the GSA does it in response to this command.  Must be called
 * only after a successful gsa_load_aoc_fw_image().  Returns 0 on success or a
 * negative errno.
 */
int gsa_aoc_start(struct device *gsa)
{
	struct gsa_dev_state *s = dev_get_drvdata(gsa);
	u32 arg = GSA_AOC_START;

	if (!s)
		return -ENODEV;

	return gsa_send_mbox_cmd(s, GSA_MB_CMD_AOC_CMD, &arg, 1, NULL, 0);
}
EXPORT_SYMBOL_GPL(gsa_aoc_start);

/**
 * gsa_aoc_get_state() - query the AOC's state from the GSA
 * @gsa: the GSA device
 *
 * Return: the GSA-reported AOC state (>= 0), -ENODATA if the GSA returned no
 * state word, or another negative errno.
 */
int gsa_aoc_get_state(struct device *gsa)
{
	struct gsa_dev_state *s = dev_get_drvdata(gsa);
	u32 arg = GSA_AOC_GET_STATE;
	u32 state = 0;
	int ret;

	if (!s)
		return -ENODEV;

	ret = gsa_send_mbox_cmd(s, GSA_MB_CMD_AOC_CMD, &arg, 1, &state, 1);
	if (ret < 0)
		return ret;
	if (ret < 1)
		return -ENODATA;

	return state;
}
EXPORT_SYMBOL_GPL(gsa_aoc_get_state);

static int gsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gsa_dev_state *s;
	u32 chip_id[2];
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	spin_lock_init(&s->slock);
	mutex_init(&s->mbox_lock);
	init_completion(&s->rsp);
	platform_set_drvdata(pdev, s);

	/* GSA DMA (used by later data-transfer commands) is 36-bit. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	s->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->base))
		return PTR_ERR(s->base);

	/* mask everything, then wire up the response interrupt */
	s->exp_intmr0 = 0xffff;
	writel(s->exp_intmr0, s->base + MBOX_INTMR0_REG);

	s->irq = platform_get_irq(pdev, 0);
	if (s->irq < 0)
		return s->irq;

	ret = devm_request_irq(dev, s->irq, gsa_mbox_irq, 0, "gsa-mbox", s);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	/*
	 * Prove the channel: SJTAG_GET_CHIP_ID takes no arguments, does no DMA
	 * and returns the chip id in two inline response words, so a clean reply
	 * means the AP<->GSA mailbox works end to end.  A failure is logged but
	 * left non-fatal so the device stays bound for inspection.
	 */
	ret = gsa_send_mbox_cmd(s, GSA_MB_CMD_SJTAG_GET_CHIP_ID, NULL, 0,
				chip_id, ARRAY_SIZE(chip_id));
	if (ret < 0)
		dev_warn(dev, "GSA mailbox up but chip-id read failed (%d)\n", ret);
	else if (ret != ARRAY_SIZE(chip_id))
		dev_warn(dev, "GSA mailbox up, unexpected chip-id arg count %d\n", ret);
	else
		dev_info(dev, "GSA up: chip id %08x%08x\n", chip_id[1], chip_id[0]);

	return 0;
}

static const struct of_device_id gsa_of_match[] = {
	{ .compatible = "google,gs101-gsa-v1" },
	{}
};
MODULE_DEVICE_TABLE(of, gsa_of_match);

static struct platform_driver gsa_driver = {
	.probe = gsa_probe,
	.driver = {
		.name = "exynos-gsa",
		.of_match_table = gsa_of_match,
	},
};
module_platform_driver(gsa_driver);

MODULE_DESCRIPTION("Google GSA on-die security-core mailbox driver");
MODULE_LICENSE("GPL");
