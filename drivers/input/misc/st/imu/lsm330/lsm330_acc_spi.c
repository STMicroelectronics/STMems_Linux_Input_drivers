/*
 * STMicroelectronics lsm330_acc_spi driver
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

#include "lsm330.h"

#define SENSORS_SPI_READ	0x80

/* XXX: caller must hold stat->lock */
static int lsm330_acc_spi_read(struct device *device, u8 addr,
			       int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_acc_data *acc = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = acc->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = acc->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	acc->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, acc->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold stat->lock */
static int lsm330_acc_spi_write(struct device *device, u8 addr,
				     int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_acc_data *acc = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = acc->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM330_TX_MAX_LENGTH)
		return -ENOMEM;

	acc->tb.tx_buf[0] = addr;

	memcpy(&acc->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct lsm330_transfer_function lsm330_acc_spi_tf = {
	.write = lsm330_acc_spi_write,
	.read = lsm330_acc_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int lsm330_acc_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_acc_data *acc = spi_get_drvdata(spi);

	return lsm330_acc_enable(acc);
}

static int lsm330_acc_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_acc_data *acc = spi_get_drvdata(spi);

	return lsm330_acc_disable(acc);
}

static SIMPLE_DEV_PM_OPS(lsm330_acc_pm_ops,
				lsm330_acc_suspend,
				lsm330_acc_resume);

#define LSM330_ACC_PM_OPS		(&lsm330_acc_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM330_ACC_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static int lsm330_acc_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm330_acc_data *acc;

#ifdef LSM330_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
	if (!acc) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	acc->name = spi->modalias;
	acc->bus_type = BUS_SPI;
	acc->tf = &lsm330_acc_spi_tf;
	acc->dev = &spi->dev;
	spi_set_drvdata(spi, acc);

	mutex_init(&acc->lock);

	err = lsm330_acc_probe(acc);
	if (err < 0) {
		kfree(acc);
		return err;
	}

	return 0;
}

int lsm330_acc_spi_remove(struct spi_device *spi)
{
	struct lsm330_acc_data *acc = spi_get_drvdata(spi);

#ifdef LSM330_DEBUG
	dev_info(acc->dev, "driver removing\n");
#endif

	lsm330_acc_remove(acc);
	kfree(acc);

	return 0;
}

static const struct spi_device_id lsm330_acc_spi_id[] = {
	{ "lsm330_acc", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lsm330_acc_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id lsm330_acc_spi_id_table[] = {
	{ .compatible = "st,lsm330_acc" },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm330_acc_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lsm330_acc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm330_acc_spi",
		.pm = LSM330_ACC_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm330_acc_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm330_acc_spi_probe,
	.remove = lsm330_acc_spi_remove,
	.id_table = lsm330_acc_spi_id,
};

module_spi_driver(lsm330_acc_spi_driver);

MODULE_DESCRIPTION("lsm330 acc-mag spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

