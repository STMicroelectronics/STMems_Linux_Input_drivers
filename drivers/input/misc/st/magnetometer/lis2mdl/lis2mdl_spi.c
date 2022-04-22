/*
 * STMicroelectronics lis2mdl_spi.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Armando Visconti <armando.visconti@st.com>
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "lis2mdl.h"

#define LIS2MDL_WHO_AM_I_ADDR		(0x4F)
#define LIS2MDL_WHO_AM_I_VAL		(0x40)

#define SENSORS_SPI_READ	0x80

/* XXX: caller must hold cdata->lock */
static int lis2mdl_spi_read(struct st_common_data *cdata,
				 u8 addr, int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(cdata->dev);

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

	cdata->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

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
static int lis2mdl_spi_write(struct st_common_data *cdata,
				  u8 addr, int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(cdata->dev);

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LIS2MDL_TX_MAX_LENGTH)
		return -ENOMEM;

	cdata->tb.tx_buf[0] = addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct lis2mdl_transfer_function lis2mdl_spi_tf = {
	.write = lis2mdl_spi_write,
	.read = lis2mdl_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int lis2mdl_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct st_common_data *cdata = spi_get_drvdata(spi);

	return lis2mdl_enable(cdata);
}

static int lis2mdl_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct st_common_data *cdata = spi_get_drvdata(spi);

	return lis2mdl_disable(cdata);
}

static SIMPLE_DEV_PM_OPS(lis2mdl_pm_ops, lis2mdl_suspend, lis2mdl_resume);

#define LIS2MDL_PM_OPS		(&lis2mdl_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS2MDL_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id lis2mdl_spi_id_table[] = {
	{ .compatible = "st,lis2mdl", },
	{ .compatible = "st,iis2mdc", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2mdl_spi_id_table);
#endif /* CONFIG_OF */

static int lis2mdl_spi_probe(struct spi_device *spi)
{
	int err;
	struct st_common_data *cdata;

#ifdef LIS2MDL_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	cdata = kzalloc(sizeof(struct st_common_data), GFP_KERNEL);
	if (!cdata) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	cdata->sensors = (struct lis2mdl_data *) kmalloc(
						sizeof(struct lis2mdl_data),
						GFP_KERNEL);
	if (!cdata->sensors)
		return -ENOMEM;

	cdata->priv_data = (priv_data_t *) kmalloc(sizeof(priv_data_t),
						   GFP_KERNEL);
	if (!cdata->priv_data)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->irq = spi->irq;
	cdata->bus_type = BUS_SPI;
	cdata->tf = &lis2mdl_spi_tf;
	cdata->wai_addr = LIS2MDL_WHO_AM_I_ADDR;
	cdata->wai_val = LIS2MDL_WHO_AM_I_VAL;

	mutex_init(&cdata->lock);

	spi_set_drvdata(spi, cdata);

	err = lis2mdl_probe(cdata);
	if (err < 0) {
		kfree(cdata);

		return err;
	}

	return 0;
}

int lis2mdl_spi_remove(struct spi_device *spi)
{
	struct st_common_data *cdata = spi_get_drvdata(spi);

#ifdef LIS2MDL_DEBUG
	dev_info(cdata->dev, "driver removing\n");
#endif

	lis2mdl_remove(cdata);
	kfree(cdata);

	return 0;
}

static const struct spi_device_id lis2mdl_spi_id[] = {
	{ LIS2MDL_DEV_NAME, 0 },
	{ IIS2MDC_DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lis2mdl_spi_id);

static struct spi_driver lis2mdl_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2MDL_DEV_NAME,
		.pm = LIS2MDL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis2mdl_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lis2mdl_spi_probe,
	.remove = lis2mdl_spi_remove,
	.id_table = lis2mdl_spi_id,
};

module_spi_driver(lis2mdl_spi_driver);

MODULE_DESCRIPTION("lis2mdl spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

