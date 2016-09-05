/*
 * STMicroelectronics lsm330_gyr_spi driver
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
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold stat->lock */
static int lsm330_gyr_spi_read(struct device *device, u8 addr, int len,
				u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_gyr_status *stat = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = stat->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = stat->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	stat->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, stat->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold stat->lock */
static int lsm330_gyr_spi_write(struct device *device, u8 addr, int len,
				u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_gyr_status *stat = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = stat->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM330_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	stat->tb.tx_buf[0] = addr;

	memcpy(&stat->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct lsm330_transfer_function lsm330_gyr_spi_tf = {
	.write = lsm330_gyr_spi_write,
	.read = lsm330_gyr_spi_read,
};

#ifdef CONFIG_PM
static int lsm330_gyr_spi_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_gyr_status *stat = spi_get_drvdata(spi);

	return lsm330_gyr_enable(stat);
}

static int lsm330_gyr_spi_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct lsm330_gyr_status *stat = spi_get_drvdata(spi);

	return lsm330_gyr_disable(stat);
}

static const struct dev_pm_ops lsm330_gyr_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm330_gyr_spi_suspend,
				lsm330_gyr_spi_resume)
};
#endif /* CONFIG_PM */

static int lsm330_gyr_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm330_gyr_status *stat;

#ifdef LSM330_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	stat = kzalloc(sizeof(struct lsm330_gyr_status), GFP_KERNEL);
	if (!stat) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	stat->name = spi->modalias;
	stat->bus_type = BUS_SPI;
	stat->tf = &lsm330_gyr_spi_tf;
	stat->dev = &spi->dev;
	spi_set_drvdata(spi, stat);

	mutex_init(&stat->lock);

	err = lsm330_gyr_probe(stat);
	if (err < 0) {
		kfree(stat);
		return err;
	}

	return 0;
}

int lsm330_gyr_spi_remove(struct spi_device *spi)
{
	struct lsm330_gyr_status *stat = spi_get_drvdata(spi);

#ifdef LSM330_DEBUG
	dev_info(stat->dev, "driver removing\n");
#endif

	lsm330_gyr_remove(stat);
	kfree(stat);

	return 0;
}

static const struct spi_device_id lsm330_gyr_spi_id[] = {
	{ "lsm330_gyr", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lsm330_gyr_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id lsm330_gyr_spi_id_table[] = {
	{ .compatible = "st,lsm330_gyr" },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm330_gyr_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lsm330_gyr_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm330_gyr_spi",
#ifdef CONFIG_PM
		.pm = &lsm330_gyr_spi_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = lsm330_gyr_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm330_gyr_spi_probe,
	.remove = lsm330_gyr_spi_remove,
	.id_table = lsm330_gyr_spi_id,
};

module_spi_driver(lsm330_gyr_spi_driver);

MODULE_DESCRIPTION("lsm330 gyr spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

