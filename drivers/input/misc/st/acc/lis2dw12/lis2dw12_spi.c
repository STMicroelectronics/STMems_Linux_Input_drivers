/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lis2dw12_spi.c
* Authors            : AMG - Application Team
*		     : Mario Tesi <mario.tesi@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.0
* Date               : 2016/Oct/18
* Description        : LIS2DW12 driver
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
#include <linux/input.h>
#include <linux/types.h>

#include "lis2dw12_core.h"

#define SENSORS_SPI_READ	0x80

static int lis2dw12_spi_read(struct lis2dw12_data *cdata, u8 reg_addr, int len,
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

static int lis2dw12_spi_write(struct lis2dw12_data *cdata, u8 reg_addr, int len,
			    u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;
	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LIS2DW12_RX_MAX_LENGTH)
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


static const struct lis2dw12_transfer_function lis2dw12_tf_spi = {
	.write = lis2dw12_spi_write,
	.read = lis2dw12_spi_read,
};

static int lis2dw12_spi_probe(struct spi_device *spi)
{
	int err;
	struct lis2dw12_data *cdata;

	cdata = devm_kzalloc(&spi->dev, sizeof(struct lis2dw12_data), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &lis2dw12_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = lis2dw12_common_probe(cdata, spi->irq, BUS_SPI);
	if (err < 0)
		return err;

	return 0;
}

static int lis2dw12_spi_remove(struct spi_device *spi)
{
	struct lis2dw12_data *cdata = spi_get_drvdata(spi);

	lis2dw12_common_remove(cdata, spi->irq);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lis2dw12_suspend(struct device *dev)
{
	struct lis2dw12_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2dw12_common_suspend(cdata);
}

static int lis2dw12_resume(struct device *dev)
{
	struct lis2dw12_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2dw12_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lis2dw12_pm_ops, lis2dw12_suspend, lis2dw12_resume);

#define LIS2DW12_PM_OPS		(&lis2dw12_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS2DW12_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lis2dw12_ids[] = {
	{ LIS2DW12_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lis2dw12_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis2dw12_id_table[] = {
	{ .compatible = "st,lis2dw12", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2dw12_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lis2dw12_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2DW12_DEV_NAME,
		.pm = LIS2DW12_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis2dw12_id_table,
#endif
	},
	.probe    = lis2dw12_spi_probe,
	.remove   = lis2dw12_spi_remove,
	.id_table = lis2dw12_ids,
};

module_spi_driver(lis2dw12_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2dw12 spi driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
