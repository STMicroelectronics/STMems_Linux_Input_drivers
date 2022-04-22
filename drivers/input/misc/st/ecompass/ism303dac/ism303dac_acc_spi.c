/*
 * STMicroelectronics ism303dac_acc_spi.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
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
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "ism303dac_core.h"

#define ISM303DAC_WHO_AM_I_ADDR			0x0f
#define ISM303DAC_WHO_AM_I_DEF			0x43

#define SENSORS_SPI_READ	0x80

/* XXX: caller must hold cdata->lock */
static int ism303dac_acc_spi_read(struct st_common_data *cdata,
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
static int ism303dac_acc_spi_write(struct st_common_data *cdata,
				  u8 addr, int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(cdata->dev);

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= ISM303DAC_TX_MAX_LENGTH)
		return -ENOMEM;

	cdata->tb.tx_buf[0] = addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct st_sensor_transfer_function ism303dac_acc_spi_tf = {
	.write = ism303dac_acc_spi_write,
	.read = ism303dac_acc_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int ism303dac_acc_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct st_common_data *cdata = spi_get_drvdata(spi);

	return ism303dac_acc_enable(cdata);
}

static int ism303dac_acc_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct st_common_data *cdata = spi_get_drvdata(spi);

	return ism303dac_acc_disable(cdata);
}

static SIMPLE_DEV_PM_OPS(ism303dac_acc_pm_ops,
			ism303dac_acc_suspend,
			ism303dac_acc_resume);

#define ISM303DAC_ACC_PM_OPS	(&ism303dac_acc_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define ISM303DAC_ACC_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id ism303dac_acc_spi_id_table[] = {
	{ .compatible = "st,ism303dac_acc", },
	{ },
};
MODULE_DEVICE_TABLE(of, ism303dac_acc_spi_id_table);
#endif /* CONFIG_OF */

static int ism303dac_acc_spi_probe(struct spi_device *spi)
{
	int err;
	struct st_common_data *cdata;

#ifdef ISM303DAC_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	cdata = kzalloc(sizeof(struct st_common_data), GFP_KERNEL);
	if (!cdata) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	cdata->sensors = (struct st_sensor_data *) kmalloc(
						sizeof(struct st_sensor_data) *
						ISM303DAC_SENSORS_NUMB,
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
	cdata->tf = &ism303dac_acc_spi_tf;
	cdata->wai_addr = ISM303DAC_WHO_AM_I_ADDR;
	cdata->wai_val = ISM303DAC_WHO_AM_I_DEF;

	mutex_init(&cdata->lock);

	spi_set_drvdata(spi, cdata);

	err = ism303dac_acc_probe(cdata);
	if (err < 0) {
		kfree(cdata);

		return err;
	}

	return 0;
}

int ism303dac_acc_spi_remove(struct spi_device *spi)
{
	struct st_common_data *cdata = spi_get_drvdata(spi);

#ifdef ISM303DAC_DEBUG
	dev_info(cdata->dev, "driver removing\n");
#endif

	ism303dac_acc_remove(cdata);
	kfree(cdata);

	return 0;
}

static const struct spi_device_id ism303dac_acc_spi_id[] = {
	{ "ism303dac_acc", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ism303dac_acc_spi_id);

static struct spi_driver ism303dac_acc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ism303dac_acc_spi",
		.pm = ISM303DAC_ACC_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = ism303dac_acc_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = ism303dac_acc_spi_probe,
	.remove = ism303dac_acc_spi_remove,
	.id_table = ism303dac_acc_spi_id,
};

module_spi_driver(ism303dac_acc_spi_driver);

MODULE_DESCRIPTION("ism303dac_acc spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

