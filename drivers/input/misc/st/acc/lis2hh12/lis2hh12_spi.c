/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis2hh12_spi.c
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Mario Tesi <mario.tesi@st.com>
*		     : Author is willing to be considered the contact and update
* Version	: V.1.0.14
* Date		: 2016/Apr/26
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
*******************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/input.h>

#include "lis2hh12.h"

#define SENSORS_SPI_READ	0x80

/* Not all devices has auto increment flags */
#ifdef SPI_HAS_AUTOINCREMENT
#define SPI_AUTO_INCREMENT	0x40
#endif

static int lis2hh12_spi_read(struct lis2hh12_status *stat, u8 reg_addr, int len,
			     u8 *data)
{
	int err;
	struct spi_message msg;
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

#ifdef SPI_HAS_AUTOINCREMENT
	stat->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ | ((len > 1) ? SPI_AUTO_INCREMENT : 0);
#else
	stat->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;
#endif

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	err = spi_sync(to_spi_device(stat->dev), &msg);
	if (err)
		return err;

	memcpy(data, stat->tb.rx_buf, len*sizeof(u8));

	return len;
}

static int lis2hh12_spi_write(struct lis2hh12_status *stat, u8 reg_addr, int len,
			      u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_transfer xfers = {
		.tx_buf = stat->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= BUFF_RX_MAX_LENGTH)
		return -ENOMEM;

#ifdef SPI_HAS_AUTOINCREMENT
	stat->tb.tx_buf[0] = reg_addr | ((len > 1) ? SPI_AUTO_INCREMENT : 0);
#else
	stat->tb.tx_buf[0] = reg_addr;
#endif

	memcpy(&stat->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(stat->dev), &msg);

	return err;
}

static struct lis2hh12_transfer_function lis2hh12_tf_spi = {
	.write = lis2hh12_spi_write,
	.read = lis2hh12_spi_read,
};

static int lis2hh12_spi_probe(struct spi_device *spi)
{
	int err;
	struct lis2hh12_status *stat;

	stat = kmalloc(sizeof(struct lis2hh12_status), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &spi->dev;
	stat->name = spi->modalias;
	stat->bustype = BUS_SPI;
	stat->tf = &lis2hh12_tf_spi;
	spi_set_drvdata(spi, stat);

	err = lis2hh12_common_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

static int lis2hh12_spi_remove(struct spi_device *spi)
{
	struct lis2hh12_status *stat = spi_get_drvdata(spi);

	lis2hh12_common_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lis2hh12_suspend(struct device *dev)
{
	struct lis2hh12_status *stat= spi_get_drvdata(to_spi_device(dev));

	return lis2hh12_common_suspend(stat);
}

static int lis2hh12_resume(struct device *dev)
{
	struct lis2hh12_status *stat = spi_get_drvdata(to_spi_device(dev));

	return lis2hh12_common_resume(stat);
}

static SIMPLE_DEV_PM_OPS(lis2hh12_pm_ops, lis2hh12_suspend, lis2hh12_resume);

#define LIS2HH12_PM_OPS	(&lis2hh12_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS2HH12_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lis2hh12_ids[] = {
	{ LIS2HH12_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lis2hh12_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis2hh12_id_table[] = {
	{ .compatible = "st,lis2hh12", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2hh12_id_table);
#endif

static struct spi_driver lis2hh12_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2HH12_ACC_DEV_NAME,
		.pm = LIS2HH12_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis2hh12_id_table,
#endif
	},
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,3,0)
	.remove = __devexit_p(lis2hh12_spi_remove),
#else
	.remove = lis2hh12_spi_remove,
#endif /* LINUX_VERSION_CODE <= KERNEL_VERSION(3,3,0) */
	.probe    = lis2hh12_spi_probe,
	.id_table = lis2hh12_ids,
};

module_spi_driver(lis2hh12_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2hh12 spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
