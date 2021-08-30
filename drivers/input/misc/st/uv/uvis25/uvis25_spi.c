/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: uvis25_spi.c
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		: Giuseppe Barba <giuseppe.barba@st.com>
*		: Mario Tesi <mario.tesi@st.com>
*		: Author is willing to be considered the contact and update
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

#include "uvis25.h"

#define SENSORS_SPI_READ	0x80

#ifdef HAS_IF_AUTO_INCREMENT
#define SPI_AUTO_INCREMENT	0x40
#endif

static int uvis25_spi_read(struct uvis25_data *stat, u8 reg_addr, int len,
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

#ifdef HAS_IF_AUTO_INCREMENT
	stat->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ | ((len > 1) ? SPI_AUTO_INCREMENT : 0);
#else
	stat->tb.tx_buf[0] = reg_addr;
#endif
	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	err = spi_sync(to_spi_device(stat->dev), &msg);
	if (err)
		return err;

	memcpy(data, stat->tb.rx_buf, len);

	return len;
}

static int uvis25_spi_write(struct uvis25_data *stat, u8 reg_addr, int len,
			    u8 *data)
{
	struct spi_message msg;
	struct spi_transfer xfers = {
		.tx_buf = stat->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= UVIS25_RX_MAX_LENGTH)
		return -ENOMEM;

#ifdef HAS_IF_AUTO_INCREMENT
	stat->tb.tx_buf[0] = reg_addr | ((len > 1) ? SPI_AUTO_INCREMENT : 0);
#else
	stat->tb.tx_buf[0] = reg_addr;
#endif
	memcpy(&stat->tb.tx_buf[1], data, len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);

	return spi_sync(to_spi_device(stat->dev), &msg);
}

static struct uvis25_transfer_function uvis25_tf_spi = {
	.write = uvis25_spi_write,
	.read = uvis25_spi_read,
};

static int uvis25_spi_probe(struct spi_device *spi)
{
	int err;
	struct uvis25_data *stat;

	stat = kzalloc(sizeof(struct uvis25_data), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &spi->dev;
	stat->name = spi->modalias;
	stat->bustype = BUS_SPI;
	stat->tf = &uvis25_tf_spi;
	spi_set_drvdata(spi, stat);

	err = uvis25_common_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

static int uvis25_spi_remove(struct spi_device *spi)
{
	struct uvis25_data *stat = spi_get_drvdata(spi);

	uvis25_common_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int uvis25_suspend(struct device *dev)
{
	struct uvis25_data *stat= spi_get_drvdata(to_spi_device(dev));

	return uvis25_common_suspend(stat);
}

static int uvis25_resume(struct device *dev)
{
	struct uvis25_data *stat = spi_get_drvdata(to_spi_device(dev));

	return uvis25_common_resume(stat);
}

static const struct dev_pm_ops uvis25_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(uvis25_suspend, uvis25_resume)
};

#define UVIS25_PM_OPS	(&uvis25_pm_ops)
#else /* CONFIG_PM */
#define UVIS25_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct spi_device_id uvis25_ids[] = {
	{ UVIS25_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, uvis25_ids);

#ifdef CONFIG_OF
static const struct of_device_id uvis25_id_table[] = {
	{ .compatible = "st,uvis25", },
	{ },
};
MODULE_DEVICE_TABLE(of, uvis25_id_table);
#endif

static struct spi_driver uvis25_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = UVIS25_DEV_NAME,
#ifdef CONFIG_PM
		.pm = UVIS25_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = uvis25_id_table,
#endif
	},
	.remove = uvis25_spi_remove,
	.probe    = uvis25_spi_probe,
	.id_table = uvis25_ids,
};

module_spi_driver(uvis25_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics uvis25 spi driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
