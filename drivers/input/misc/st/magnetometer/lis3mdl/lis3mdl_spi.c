/*
 * STMicroelectronics lis3mdl_spi.c driver
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

#include "lis3mdl.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold dev->lock */
static int lis3mdl_spi_read(struct device *device, u8 addr, int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lis3mdl_dev *dev = spi_get_drvdata(spi);

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
static int lis3mdl_spi_write(struct device *device, u8 addr, int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lis3mdl_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = dev->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LIS3MDL_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr;

	memcpy(&dev->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static const struct lis3mdl_transfer_function lis3mdl_spi_tf = {
	.write = lis3mdl_spi_write,
	.read = lis3mdl_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int lis3mdl_spi_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lis3mdl_dev *dev = spi_get_drvdata(spi);

	return lis3mdl_mag_disable(dev);
}

static int lis3mdl_spi_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lis3mdl_dev *dev = spi_get_drvdata(spi);

	return lis3mdl_mag_enable(dev);
}

static SIMPLE_DEV_PM_OPS(lis3mdl_spi_pm_ops,
				lis3mdl_spi_suspend,
				lis3mdl_spi_resume);

#define LIS3MDL_PM_OPS		(&lis3mdl_spi_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS3MDL_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static int lis3mdl_spi_probe(struct spi_device *spi)
{
	int err;
	struct lis3mdl_dev *dev;

#ifdef LIS3MDL_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &spi->dev;
	dev->name = spi->modalias;
	dev->bus_type = BUS_SPI;
	dev->tf = &lis3mdl_spi_tf;
	spi_set_drvdata(spi, dev);

	mutex_init(&dev->lock);

	err = lis3mdl_mag_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

static int lis3mdl_spi_remove(struct spi_device *spi)
{
	struct lis3mdl_dev *dev = spi_get_drvdata(spi);

#ifdef LIS3MDL_DEBUG
	dev_info(&spi->dev, "driver removing\n");
#endif

	lis3mdl_mag_remove(dev);
	kfree(dev);

	return 0;
}

static const struct spi_device_id lis3mdl_spi_ids[] = {
	{ "lis3mdl", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lis3mdl_spi_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis3mdl_spi_id_table[] = {
	{ .compatible = "st,lis3mdl", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis3mdl_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lis3mdl_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lis3mdl_spi",
		.pm = LIS3MDL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis3mdl_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe    = lis3mdl_spi_probe,
	.remove   = lis3mdl_spi_remove,
	.id_table = lis3mdl_spi_ids,
};

module_spi_driver(lis3mdl_spi_driver);

MODULE_DESCRIPTION("lis3mdl i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

