/******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
*
* File Name          : h3lis100dl_spi.c
* Authors            : AMS - VMU - Application Team
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.1.0
* Date               : 2016/Apr/26
* Description        : h3lis100dl spi driver
*
*******************************************************************************
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
*******************************************************************************/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "h3lis100dl.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* Set auto increment accordingly */
static inline u8 fill_autoinc(u8 reg_addr)
{
	return reg_addr | ((reg_addr & AUTO_INCREMENT) ? SPI_AUTO_INCREMENT : 0);
}

static int h3lis100dl_spi_read(struct h3lis100dl_data *cdata, u8 reg_addr, int len,
			       u8 *data)
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

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = fill_autoinc(reg_addr) | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);

	return err;
}

static int h3lis100dl_spi_write(struct h3lis100dl_data *cdata, u8 reg_addr, int len,
				u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= H3LIS100DL_RX_MAX_LENGTH)
		return -ENOMEM;

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = fill_autoinc(reg_addr);

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);
	mutex_unlock(&cdata->tb.buf_lock);

	return err;
}


static const struct h3lis100dl_transfer_function h3lis100dl_tf_spi = {
	.write = h3lis100dl_spi_write,
	.read = h3lis100dl_spi_read,
};

static int h3lis100dl_spi_probe(struct spi_device *spi)
{
	int err;
	struct h3lis100dl_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->bustype = BUS_SPI;
	cdata->tf = &h3lis100dl_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = h3lis100dl_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int h3lis100dl_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct h3lis100dl_data *cdata = spi_get_drvdata(spi);

	h3lis100dl_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", H3LIS100DL_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int h3lis100dl_suspend(struct device *dev)
{
	struct h3lis100dl_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return h3lis100dl_common_suspend(cdata);
}

static int h3lis100dl_resume(struct device *dev)
{
	struct h3lis100dl_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return h3lis100dl_common_resume(cdata);
}

static const struct dev_pm_ops h3lis100dl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(h3lis100dl_suspend, h3lis100dl_resume)
};

#define H3LIS100DL_PM_OPS		(&h3lis100dl_pm_ops)
#else /* CONFIG_PM */
#define H3LIS100DL_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id h3lis100dl_ids[] = {
	{H3LIS100DL_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(spi, h3lis100dl_ids);

#ifdef CONFIG_OF
static const struct of_device_id h3lis100dl_id_spi_table[] = {
	{.compatible = "st,h3lis100dl", },
	{ },
};
MODULE_DEVICE_TABLE(of, h3lis100dl_id_spi_table);
#endif

static struct spi_driver h3lis100dl_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = H3LIS100DL_DEV_NAME,
		.pm = H3LIS100DL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = h3lis100dl_id_spi_table,
#endif
	},
	.probe    = h3lis100dl_spi_probe,
	.remove   = h3lis100dl_spi_remove,
	.id_table = h3lis100dl_ids,
};

module_spi_driver(h3lis100dl_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics h3lis100dl spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
