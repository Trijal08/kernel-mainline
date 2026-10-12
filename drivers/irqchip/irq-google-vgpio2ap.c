// SPDX-License-Identifier: GPL-2.0
/*
 * VGPIO2AP interrupt demultiplexer for Google Tensor SoCs.
 *
 * The main and sub PMICs on these boards sit on an I3C bus and raise in-band
 * interrupts rather than driving a wire.  The SoC turns those into a single
 * shared interrupt: a latch (INTC0_IPEND, in the VGPIO2AP sysreg block) says
 * "an in-band interrupt arrived", and a monitor register in the I3C block says
 * which slave raised it.  Neither is a GPIO despite the name -- there is
 * nothing here to read as a line level or to use as an output.
 *
 * So this is a demultiplexer and nothing more.  It owns the shared parent
 * interrupt, acks the latch, and reports one child interrupt per slave.  What
 * each PMIC then has to say about itself is read back over the PMIC's own
 * (ACPM-mediated) register bus by its own driver, which is why the children
 * are handled as threaded interrupts and why there is no mask here: the real
 * masking lives in the PMIC's IBIM and INTxM registers.
 *
 * Copyright 2026 Trijal Saha
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/*
 * IBI0, the first byte of the monitor word: one bit per I3C slave.  The wider
 * word carries the per-block bits of each slave as further bytes, but which
 * block inside a PMIC asserted is the PMIC driver's business, not ours.
 */
#define VGPIO2AP_IBI0_PMIC_MAIN		BIT(0)
#define VGPIO2AP_IBI0_PMIC_SUB		BIT(1)
#define VGPIO2AP_IBI0_MASK		GENMASK(7, 0)

#define VGPIO2AP_IRQ_PMIC_MAIN		0
#define VGPIO2AP_IRQ_PMIC_SUB		1
#define VGPIO2AP_NR_IRQS		2

struct vgpio2ap {
	void __iomem		*ipend;
	void __iomem		*monitor;
	struct irq_domain	*domain;
	unsigned int		parent_irq;
};

/*
 * There is no per-slave mask in this block, only the shared latch.  genirq
 * still needs these for the IRQF_ONESHOT children, so they exist and do
 * nothing; a slave that wants to be quiet masks itself.
 */
static void vgpio2ap_irq_mask(struct irq_data *d)
{
}

static void vgpio2ap_irq_unmask(struct irq_data *d)
{
}

/*
 * Wakeup is a property of the shared parent: the PMICs are how this SoC wakes
 * up at all, so arming any child has to arm the line they come in on.
 */
static int vgpio2ap_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct vgpio2ap *v = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(v->parent_irq, on);
}

static struct irq_chip vgpio2ap_irq_chip = {
	.name			= "VGPIO2AP",
	.irq_mask		= vgpio2ap_irq_mask,
	.irq_unmask		= vgpio2ap_irq_unmask,
	.irq_set_wake		= vgpio2ap_irq_set_wake,
};

static void vgpio2ap_handle_irq(struct irq_desc *desc)
{
	struct vgpio2ap *v = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 pending, ibi;

	chained_irq_enter(chip, desc);

	/* Write-1-to-clear, and do it before reading who asserted: an in-band
	 * interrupt that arrives while we are working must latch again rather
	 * than be lost in the gap.
	 */
	pending = readl(v->ipend);
	writel(pending, v->ipend);

	ibi = readl(v->monitor) & VGPIO2AP_IBI0_MASK;

	if (ibi & VGPIO2AP_IBI0_PMIC_MAIN)
		generic_handle_domain_irq(v->domain, VGPIO2AP_IRQ_PMIC_MAIN);
	if (ibi & VGPIO2AP_IBI0_PMIC_SUB)
		generic_handle_domain_irq(v->domain, VGPIO2AP_IRQ_PMIC_SUB);

	chained_irq_exit(chip, desc);
}

static int vgpio2ap_domain_map(struct irq_domain *d, unsigned int virq,
			       irq_hw_number_t hw)
{
	struct vgpio2ap *v = d->host_data;

	irq_set_chip_and_handler(virq, &vgpio2ap_irq_chip, handle_simple_irq);
	irq_set_chip_data(virq, v);

	return 0;
}

static const struct irq_domain_ops vgpio2ap_domain_ops = {
	.map	= vgpio2ap_domain_map,
	.xlate	= irq_domain_xlate_twocell,
};

static int vgpio2ap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vgpio2ap *v;
	int ret;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;

	v->ipend = devm_platform_ioremap_resource_byname(pdev, "ipend");
	if (IS_ERR(v->ipend))
		return PTR_ERR(v->ipend);

	v->monitor = devm_platform_ioremap_resource_byname(pdev, "monitor");
	if (IS_ERR(v->monitor))
		return PTR_ERR(v->monitor);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	v->parent_irq = ret;

	v->domain = irq_domain_create_linear(dev_fwnode(dev), VGPIO2AP_NR_IRQS,
					     &vgpio2ap_domain_ops, v);
	if (!v->domain)
		return dev_err_probe(dev, -ENOMEM, "failed to create IRQ domain\n");

	irq_set_chained_handler_and_data(v->parent_irq, vgpio2ap_handle_irq, v);

	/*
	 * Clear anything the bootloader left latched, so the first real
	 * in-band interrupt is not mistaken for one of ours.
	 */
	writel(readl(v->ipend), v->ipend);

	platform_set_drvdata(pdev, v);

	return 0;
}

static void vgpio2ap_remove(struct platform_device *pdev)
{
	struct vgpio2ap *v = platform_get_drvdata(pdev);

	irq_set_chained_handler_and_data(v->parent_irq, NULL, NULL);
	irq_domain_remove(v->domain);
}

static const struct of_device_id vgpio2ap_of_match[] = {
	{ .compatible = "google,zuma-vgpio2ap" },
	{ }
};
MODULE_DEVICE_TABLE(of, vgpio2ap_of_match);

/*
 * An ordinary platform driver rather than an early IRQCHIP_PLATFORM_DRIVER:
 * the only consumers are PMICs reached over ACPM, which probe long after this
 * does, and of_irq_get() defers them until this domain exists.
 */
static struct platform_driver vgpio2ap_driver = {
	.probe	= vgpio2ap_probe,
	.remove	= vgpio2ap_remove,
	.driver	= {
		.name		= "google-vgpio2ap",
		.of_match_table	= vgpio2ap_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(vgpio2ap_driver);

MODULE_DESCRIPTION("VGPIO2AP interrupt demultiplexer for Google Tensor SoCs");
MODULE_LICENSE("GPL");
