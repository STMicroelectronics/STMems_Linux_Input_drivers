/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
*
* File Name          : lis2ds_spi.c
* Authors            : AMS - VMU - Application Team
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.1.0
* Date               : 2015/Apr/17
* Description        : LIS2DS driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "lis2ds_core.h"

#define SENSORS_SPI_READ			0x80

static int lis2ds_spi_read(struct lis2ds_data *cdata, u8 reg_addr, int len,
			   u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;

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

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int lis2ds_spi_write(struct lis2ds_data *cdata, u8 reg_addr, int len,
			    u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LIS2DS_RX_MAX_LENGTH)
		return -ENOMEM;

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);

	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}


static const struct lis2ds_transfer_function lis2ds_tf_spi = {
	.write = lis2ds_spi_write,
	.read = lis2ds_spi_read,
};

static int lis2ds_spi_probe(struct spi_device *spi)
{
	int err;
	struct lis2ds_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &lis2ds_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = lis2ds_common_probe(cdata, spi->irq, BUS_SPI);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lis2ds_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lis2ds_data *cdata = spi_get_drvdata(spi);

	lis2ds_common_remove(cdata, spi->irq);
	dev_info(cdata->dev, "%s: removed\n", LIS2DS_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lis2ds_suspend(struct device *dev)
{
	struct lis2ds_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2ds_common_suspend(cdata);
}

static int lis2ds_resume(struct device *dev)
{
	struct lis2ds_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2ds_common_resume(cdata);
}

static const struct dev_pm_ops lis2ds_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis2ds_suspend, lis2ds_resume)
};

#define LIS2DS_PM_OPS		(&lis2ds_pm_ops)
#else /* CONFIG_PM */
#define LIS2DS_PM_OPS		NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct spi_device_id lis2ds_ids[] = {
	{ LIS2DS_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lis2ds_ids);

static const struct of_device_id lis2ds_id_table[] = {
	{ .compatible = "st,lis2ds", },
	{ .compatible = "st,lis2ds12", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2ds_id_table);
#endif

static struct spi_driver lis2ds_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2DS_DEV_NAME,
		.pm = LIS2DS_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis2ds_id_table,
#endif
	},
	.probe    = lis2ds_spi_probe,
	.remove   = lis2ds_spi_remove,
	.id_table = lis2ds_ids,
};

module_spi_driver(lis2ds_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2ds spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
