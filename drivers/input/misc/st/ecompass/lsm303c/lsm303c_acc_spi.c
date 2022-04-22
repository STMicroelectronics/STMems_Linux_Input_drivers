/*
 * STMicroelectronics lsm303c_acc_spi driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/module.h>

#include "lsm303c.h"

#define SENSORS_SPI_READ	0x80

/* XXX: caller must hold cdata->lock */
static int lsm303c_acc_spi_read(struct device *device, u8 addr,
				int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm303c_acc_dev *dev = spi_get_drvdata(spi);

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

/* XXX: caller must hold cdata->lock */
static int lsm303c_acc_spi_write(struct device *device, u8 addr,
				 int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm303c_acc_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = dev->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM303C_TX_MAX_LENGTH)
		return -ENOMEM;

	dev->tb.tx_buf[0] = addr;

	memcpy(&dev->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct lsm303c_transfer_function lsm303c_acc_spi_tf = {
	.write = lsm303c_acc_spi_write,
	.read = lsm303c_acc_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int lsm303c_acc_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm303c_acc_dev *dev = spi_get_drvdata(spi);

	return lsm303c_acc_enable(dev);
}

static int lsm303c_acc_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm303c_acc_dev *dev = spi_get_drvdata(spi);

	return lsm303c_acc_disable(dev);
}

static SIMPLE_DEV_PM_OPS(lsm303c_acc_pm_ops,
			lsm303c_acc_suspend,
			lsm303c_acc_resume);

#define LSM303C_ACC_PM_OPS	(&lsm303c_acc_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM303C_ACC_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static int lsm303c_acc_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm303c_acc_dev *dev;

#ifdef lsm303c_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct lsm303c_acc_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->name = spi->modalias;
	dev->bus_type = BUS_SPI;
	dev->tf = &lsm303c_acc_spi_tf;
	dev->dev = &spi->dev;
	spi_set_drvdata(spi, dev);

	mutex_init(&dev->lock);

	err = lsm303c_acc_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

int lsm303c_acc_spi_remove(struct spi_device *spi)
{
	struct lsm303c_acc_dev *dev = spi_get_drvdata(spi);

#ifdef lsm303c_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm303c_acc_remove(dev);
	kfree(dev);

	return 0;
}

static const struct spi_device_id lsm303c_acc_spi_id[] = {
	{ "lsm303c_acc", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lsm303c_acc_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id lsm303c_acc_spi_id_table[] = {
	{ .compatible = "st,lsm303c_acc" },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303c_acc_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lsm303c_acc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm303c_acc_spi",
		.pm = LSM303C_ACC_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm303c_acc_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm303c_acc_spi_probe,
	.remove = lsm303c_acc_spi_remove,
	.id_table = lsm303c_acc_spi_id,
};

module_spi_driver(lsm303c_acc_spi_driver);

MODULE_DESCRIPTION("lsm303c acc spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

