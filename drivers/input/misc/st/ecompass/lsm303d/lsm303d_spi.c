/*
 * STMicroelectronics lsm303d_spi.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/input.h>

#include "lsm303d.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold dev->lock */
static int lsm303d_spi_read(struct device *device, u8 addr, int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm303d_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = dev->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = dev->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, dev->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold dev->lock */
static int lsm303d_spi_write(struct device *device, u8 addr, int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm303d_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = dev->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM303D_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr;

	memcpy(&dev->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct lsm303d_transfer_function lsm303d_spi_tf = {
	.write = lsm303d_spi_write,
	.read = lsm303d_spi_read,
};

#ifdef CONFIG_PM
static int lsm303d_spi_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm303d_dev *dev = spi_get_drvdata(spi);

	return lsm303d_disable(dev);
}

static int lsm303d_spi_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm303d_dev *dev = spi_get_drvdata(spi);

	return lsm303d_enable(dev);
}

static const struct dev_pm_ops lsm303d_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303d_spi_suspend, lsm303d_spi_resume)
};
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id lsm303d_spi_id_table[] = {
	{ .compatible = "st,lsm303d", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303d_spi_id_table);
#endif /* CONFIG_OF */

static int lsm303d_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm303d_dev *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &spi->dev;
	dev->name = spi->modalias;
	dev->bus_type = BUS_SPI;
	dev->st.dev = dev;
	dev->tf = &lsm303d_spi_tf;

	spi_set_drvdata(spi, dev);

#ifdef CONFIG_OF
	dev->dev_id = lsm303d_spi_id_table;
#endif

	err = lsm303d_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

static int lsm303d_spi_remove(struct spi_device *spi)
{
	struct lsm303d_dev *dev = spi_get_drvdata(spi);

	lsm303d_remove(dev);
	kfree(dev);

	return 0;
}

static const struct spi_device_id lsm303d_spi_ids[] = {
	{ "lsm303d", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm303d_spi_ids);

static struct spi_driver lsm303d_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm303d",
#ifdef CONFIG_PM
		.pm = &lsm303d_spi_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = lsm303d_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe    = lsm303d_spi_probe,
	.remove   = lsm303d_spi_remove,
	.id_table = lsm303d_spi_ids,
};

module_spi_driver(lsm303d_spi_driver);

MODULE_DESCRIPTION("lsm303d spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

