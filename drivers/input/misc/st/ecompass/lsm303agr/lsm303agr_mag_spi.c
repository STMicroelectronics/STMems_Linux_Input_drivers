/*
 * STMicroelectronics lsm303agr_mag_spi.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
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

#include "lsm303agr_core.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold cdata->lock */
static int lsm303agr_mag_spi_read(struct device *dev, u8 reg_addr, int len,
				  u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(dev);
	struct lsm303agr_common_data *cdata = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = cdata->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = cdata->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len > 1)
		reg_addr |= SPI_AUTO_INCREMENT;

	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, cdata->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold cdata->lock */
static int lsm303agr_mag_spi_write(struct device *dev, u8 reg_addr, int len,
				   u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(dev);
	struct lsm303agr_common_data *cdata = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM303AGR_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		reg_addr |= SPI_AUTO_INCREMENT;

	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static const struct lsm303agr_transfer_function lsm303agr_mag_spi_tf = {
	.write = lsm303agr_mag_spi_write,
	.read = lsm303agr_mag_spi_read,
};

#ifdef CONFIG_PM
static int lsm303agr_mag_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lsm303agr_common_data *cdata = spi_get_drvdata(spi);

	cdata->on_before_suspend = atomic_read(&cdata->enabled);
	return lsm303agr_mag_disable(cdata);
}

static int lsm303agr_mag_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lsm303agr_common_data *cdata = spi_get_drvdata(spi);

	if (cdata->on_before_suspend)
		return lsm303agr_mag_enable(cdata);
	return 0;
}

static const struct dev_pm_ops lsm303agr_mag_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303agr_mag_spi_suspend,
				lsm303agr_mag_spi_resume)
};
#endif /* CONFIG_PM */

static int lsm303agr_mag_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm303agr_common_data *cdata;

	cdata = kzalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->sensor_num = LSM303AGR_MAX_SENSORS_NUM;
	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->bus_type = BUS_SPI;
	cdata->tf = &lsm303agr_mag_spi_tf;
	spi_set_drvdata(spi, cdata);

	mutex_init(&cdata->lock);

	err = lsm303agr_mag_probe(cdata);
	if (err < 0) {
		kfree(cdata);

		return err;
	}

	return 0;
}

static int lsm303agr_mag_spi_remove(struct spi_device *spi)
{
	struct lsm303agr_common_data *cdata = spi_get_drvdata(spi);

	lsm303agr_mag_remove(cdata);
	kfree(cdata);

	return 0;
}

static const struct spi_device_id lsm303agr_mag_spi_ids[] = {
	{ "lsm303agr_mag", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm303agr_mag_spi_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm303agr_mag_spi_id_table[] = {
	{ .compatible = "st,lsm303agr_mag", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303agr_mag_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lsm303agr_mag_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm303agr_mag",
#ifdef CONFIG_PM
		.pm = &lsm303agr_mag_spi_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = lsm303agr_mag_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe    = lsm303agr_mag_spi_probe,
	.remove   = lsm303agr_mag_spi_remove,
	.id_table = lsm303agr_mag_spi_ids,
};

module_spi_driver(lsm303agr_mag_spi_driver);


MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");

