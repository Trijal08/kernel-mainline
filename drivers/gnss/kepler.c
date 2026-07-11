// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung "KEPLER" GNSS receiver driver.
 *
 * The receiver is a companion to the Exynos CP (baseband) found on Google
 * Tensor boards.  Its firmware is staged into CP shared memory and booted by
 * the modem, but the runtime host link is an independent SPI bus with a
 * two-wire handshake:
 *
 *   gnss2ap  device -> host, level-high while the receiver has data to move
 *                      or (during a host-initiated write) once it is ready to
 *                      accept the transfer.  This line is the interrupt source.
 *   ap2gnss  host -> device, asserted by the host to signal "ready".
 *
 * Frames are moved in fixed 64-byte SPI bursts.  The transport is proprietary
 * (Samsung "BETP"); this driver is only the pipe -- it hands the raw byte
 * stream to the GNSS core and lets user space frame it.
 *
 * Ported from the Google/Samsung out-of-tree gnssif_spi driver.
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 * Copyright (C) 2026 Trijal Saha <trijalsaha2012@gmail.com>
 */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gnss.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#define KEPLER_RX_CHUNK		64		/* one SPI RX burst */
#define KEPLER_RX_FRAME_MAX	SZ_2K		/* deliver accumulated RX at this */
#define KEPLER_TX_MAX		SZ_4K
#define KEPLER_RDY_TIMEOUT_MS	5000
#define KEPLER_WAKE_MS		500

/*
 * The link is a plain byte stream. The vendor stack clocks it as 32-bit words
 * and then has the controller swap bytes and half-words within each word, which
 * is just a byte reversal that undoes the word being shifted out MSB first.
 * Eight-bit words with no swapping put the same bytes on the wire in the same
 * order, and need nothing beyond a stock SPI controller.
 */
#define KEPLER_BITS_PER_WORD	8

/* Frames are a whole number of 32-bit words even though the words are not. */
#define KEPLER_FRAME_ALIGN	4

struct kepler_gnss {
	struct spi_device	*spi;
	struct gnss_device	*gdev;

	struct gpio_desc	*gnss2ap;	/* device->host, IRQ source */
	struct gpio_desc	*ap2gnss;	/* host->device */
	int			irq;

	struct completion	ready;
	atomic_t		wait_ready;	/* a write is waiting for the ready edge */
	atomic_t		tx_active;
	atomic_t		rx_active;

	/*
	 * The gnss2ap line is level-triggered: the hard handler masks it and the
	 * RX thread (or the write path) unmasks it once the burst is drained.
	 * These come from several contexts, so guard the enable/disable state.
	 */
	spinlock_t		irq_lock;
	bool			irq_on;
	bool			irq_on_suspend;

	struct wakeup_source	*ws;

	u8			rx_buf[KEPLER_RX_FRAME_MAX] __aligned(4);
};

static void kepler_irq_enable(struct kepler_gnss *kp)
{
	unsigned long flags;

	spin_lock_irqsave(&kp->irq_lock, flags);
	if (!kp->irq_on) {
		enable_irq(kp->irq);
		kp->irq_on = true;
	}
	spin_unlock_irqrestore(&kp->irq_lock, flags);
}

/* Safe to call from the hard IRQ handler. */
static void kepler_irq_disable_nosync(struct kepler_gnss *kp)
{
	unsigned long flags;

	spin_lock_irqsave(&kp->irq_lock, flags);
	if (kp->irq_on) {
		disable_irq_nosync(kp->irq);
		kp->irq_on = false;
	}
	spin_unlock_irqrestore(&kp->irq_lock, flags);
}

static void kepler_irq_disable_sync(struct kepler_gnss *kp)
{
	bool was_on;

	spin_lock_irq(&kp->irq_lock);
	was_on = kp->irq_on;
	kp->irq_on = false;
	spin_unlock_irq(&kp->irq_lock);

	if (was_on)
		disable_irq(kp->irq);
}

static int kepler_spi_recv(struct kepler_gnss *kp, void *rx, unsigned int len)
{
	struct spi_transfer xfer = {
		.rx_buf		= rx,
		.len		= len,
		.bits_per_word	= KEPLER_BITS_PER_WORD,
	};

	return spi_sync_transfer(kp->spi, &xfer, 1);
}

static int kepler_spi_xfer(struct kepler_gnss *kp, const void *tx, void *rx,
			   unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf		= tx,
		.rx_buf		= rx,
		.len		= len,
		.bits_per_word	= KEPLER_BITS_PER_WORD,
	};

	return spi_sync_transfer(kp->spi, &xfer, 1);
}

static irqreturn_t kepler_irq_hard(int irq, void *data)
{
	struct kepler_gnss *kp = data;

	/* Mask the level line; RX thread / write path re-arms it. */
	kepler_irq_disable_nosync(kp);

	if (atomic_read(&kp->wait_ready)) {
		atomic_set(&kp->wait_ready, 0);
		complete_all(&kp->ready);
		return IRQ_HANDLED;
	}

	if (atomic_read(&kp->rx_active))
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t kepler_irq_thread(int irq, void *data)
{
	struct kepler_gnss *kp = data;
	unsigned int filled = 0;
	int ret;

	atomic_set(&kp->rx_active, 1);

	do {
		if (filled == 0) {
			/* Tell the receiver the host is ready to take a burst. */
			gpiod_set_value(kp->ap2gnss, 1);
			if (!atomic_read(&kp->tx_active)) {
				udelay(100);
				gpiod_set_value(kp->ap2gnss, 0);
			}
		}

		ret = kepler_spi_recv(kp, kp->rx_buf + filled, KEPLER_RX_CHUNK);
		if (ret) {
			dev_err_ratelimited(&kp->spi->dev, "RX failed: %d\n", ret);
			break;
		}
		filled += KEPLER_RX_CHUNK;

		if (filled >= KEPLER_RX_FRAME_MAX ||
		    atomic_read(&kp->wait_ready)) {
			gnss_insert_raw(kp->gdev, kp->rx_buf, filled);
			filled = 0;

			/* A write is waiting for the bus: hand it over. */
			if (atomic_read(&kp->wait_ready)) {
				atomic_set(&kp->wait_ready, 0);
				complete_all(&kp->ready);
				while (atomic_read(&kp->tx_active))
					udelay(10);
			}
		}
	} while (gpiod_get_value(kp->gnss2ap));

	if (filled)
		gnss_insert_raw(kp->gdev, kp->rx_buf, filled);

	atomic_set(&kp->rx_active, 0);
	if (!atomic_read(&kp->tx_active))
		gpiod_set_value(kp->ap2gnss, 0);

	kepler_irq_enable(kp);

	return IRQ_HANDLED;
}

static int kepler_open(struct gnss_device *gdev)
{
	struct kepler_gnss *kp = gnss_get_drvdata(gdev);

	kepler_irq_enable(kp);

	return 0;
}

static void kepler_close(struct gnss_device *gdev)
{
	struct kepler_gnss *kp = gnss_get_drvdata(gdev);

	kepler_irq_disable_sync(kp);
}

static int kepler_write_raw(struct gnss_device *gdev,
			    const unsigned char *buf, size_t count)
{
	struct kepler_gnss *kp = gnss_get_drvdata(gdev);
	unsigned int len;
	void *tx, *rx;
	int ret;

	count = min_t(size_t, count, KEPLER_TX_MAX);
	len = round_up(count, KEPLER_FRAME_ALIGN);

	tx = kzalloc(len, GFP_KERNEL);
	rx = kzalloc(len, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out_free;
	}
	memcpy(tx, buf, count);

	atomic_set(&kp->tx_active, 1);
	reinit_completion(&kp->ready);
	atomic_set(&kp->wait_ready, 1);

	__pm_wakeup_event(kp->ws, KEPLER_WAKE_MS);
	kepler_irq_enable(kp);

	/* Request the bus and wait for the receiver to signal it is ready. */
	gpiod_set_value(kp->ap2gnss, 1);
	if (!wait_for_completion_timeout(&kp->ready,
					 msecs_to_jiffies(KEPLER_RDY_TIMEOUT_MS))) {
		dev_err(&kp->spi->dev, "timed out waiting for GNSS ready\n");
		ret = -ETIMEDOUT;
		goto out_deassert;
	}

	ret = kepler_spi_xfer(kp, tx, rx, len);
	if (ret) {
		dev_err(&kp->spi->dev, "TX failed: %d\n", ret);
		goto out_deassert;
	}

	/* Bytes clocked out during the write are an inbound frame. */
	gnss_insert_raw(gdev, rx, len);
	ret = count;

out_deassert:
	atomic_set(&kp->wait_ready, 0);
	atomic_set(&kp->tx_active, 0);
	gpiod_set_value(kp->ap2gnss, 0);
	kepler_irq_enable(kp);
out_free:
	kfree(tx);
	kfree(rx);

	return ret;
}

static const struct gnss_operations kepler_gnss_ops = {
	.open		= kepler_open,
	.close		= kepler_close,
	.write_raw	= kepler_write_raw,
};

static int kepler_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct kepler_gnss *kp;
	struct gnss_device *gdev;
	int ret;

	kp = devm_kzalloc(dev, sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	kp->spi = spi;
	spin_lock_init(&kp->irq_lock);
	init_completion(&kp->ready);
	atomic_set(&kp->wait_ready, 0);
	atomic_set(&kp->tx_active, 0);
	atomic_set(&kp->rx_active, 0);

	spi->bits_per_word = KEPLER_BITS_PER_WORD;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	kp->gnss2ap = devm_gpiod_get(dev, "gnss2ap", GPIOD_IN);
	if (IS_ERR(kp->gnss2ap))
		return dev_err_probe(dev, PTR_ERR(kp->gnss2ap),
				     "failed to get gnss2ap gpio\n");

	kp->ap2gnss = devm_gpiod_get(dev, "ap2gnss", GPIOD_OUT_LOW);
	if (IS_ERR(kp->ap2gnss))
		return dev_err_probe(dev, PTR_ERR(kp->ap2gnss),
				     "failed to get ap2gnss gpio\n");

	kp->irq = gpiod_to_irq(kp->gnss2ap);
	if (kp->irq < 0)
		return dev_err_probe(dev, kp->irq, "no IRQ for gnss2ap\n");

	kp->ws = wakeup_source_register(dev, "gnss-kepler");
	if (!kp->ws)
		return -ENOMEM;

	/* Start masked; kepler_open() arms it when user space opens the port. */
	ret = devm_request_threaded_irq(dev, kp->irq, kepler_irq_hard,
					kepler_irq_thread,
					IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN,
					"gnss-kepler", kp);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request IRQ\n");
		goto err_ws;
	}
	enable_irq_wake(kp->irq);

	gdev = gnss_allocate_device(dev);
	if (!gdev) {
		ret = -ENOMEM;
		goto err_wake;
	}
	gdev->ops = &kepler_gnss_ops;
	gdev->type = GNSS_TYPE_NMEA;	/* nominal: BETP transport is proprietary */
	gnss_set_drvdata(gdev, kp);
	kp->gdev = gdev;
	spi_set_drvdata(spi, kp);

	ret = gnss_register_device(gdev);
	if (ret)
		goto err_put;

	return 0;

err_put:
	gnss_put_device(gdev);
err_wake:
	disable_irq_wake(kp->irq);
err_ws:
	wakeup_source_unregister(kp->ws);

	return ret;
}

static void kepler_remove(struct spi_device *spi)
{
	struct kepler_gnss *kp = spi_get_drvdata(spi);

	disable_irq_wake(kp->irq);
	kepler_irq_disable_sync(kp);
	gpiod_set_value(kp->ap2gnss, 0);
	gnss_deregister_device(kp->gdev);
	gnss_put_device(kp->gdev);
	wakeup_source_unregister(kp->ws);
}

static int kepler_suspend(struct device *dev)
{
	struct kepler_gnss *kp = dev_get_drvdata(dev);

	kp->irq_on_suspend = kp->irq_on;
	kepler_irq_disable_nosync(kp);

	return 0;
}

static int kepler_resume(struct device *dev)
{
	struct kepler_gnss *kp = dev_get_drvdata(dev);

	if (kp->irq_on_suspend)
		kepler_irq_enable(kp);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(kepler_pm_ops, kepler_suspend, kepler_resume);

static const struct of_device_id kepler_of_match[] = {
	{ .compatible = "samsung,kepler" },
	{ }
};
MODULE_DEVICE_TABLE(of, kepler_of_match);

static const struct spi_device_id kepler_spi_id[] = {
	{ "kepler" },
	{ }
};
MODULE_DEVICE_TABLE(spi, kepler_spi_id);

static struct spi_driver kepler_driver = {
	.driver = {
		.name		= "gnss-kepler",
		.of_match_table	= kepler_of_match,
		.pm		= pm_sleep_ptr(&kepler_pm_ops),
	},
	.probe		= kepler_probe,
	.remove		= kepler_remove,
	.id_table	= kepler_spi_id,
};
module_spi_driver(kepler_driver);

MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_DESCRIPTION("Samsung KEPLER SPI GNSS receiver driver");
MODULE_LICENSE("GPL");
