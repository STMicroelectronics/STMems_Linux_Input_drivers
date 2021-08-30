/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: l3g4200d_spi.c
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

#include "l3g4200d.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

static int l3g4200d_spi_read(struct l3g4200d_data *cdata, u8 reg_addr, int len,
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
	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ | ((len > 1) ? SPI_AUTO_INCREMENT : 0);

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

static int l3g4200d_spi_write(struct l3g4200d_data *cdata, u8 reg_addr, int len,
			       u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= BUFF_RX_MAX_LENGTH)
		return -ENOMEM;

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | ((len > 1) ? SPI_AUTO_INCREMENT : 0);

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);
	mutex_unlock(&cdata->tb.buf_lock);

	return err;
}

static struct l3g4200d_transfer_function l3g4200d_tf_spi = {
	.write = l3g4200d_spi_write,
	.read = l3g4200d_spi_read,
};

static int l3g4200d_spi_probe(struct spi_device *spi)
{
	int err;
	struct l3g4200d_data *cdata;

	cdata = kmalloc(sizeof(struct l3g4200d_data), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->bustype = BUS_SPI;
	cdata->tf = &l3g4200d_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = l3g4200d_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int l3g4200d_spi_remove(struct spi_device *spi)
{
	struct l3g4200d_data *cdata = spi_get_drvdata(spi);

	l3g4200d_common_remove(cdata);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int l3g4200d_suspend(struct device *dev)
{
	struct l3g4200d_data *cdata= spi_get_drvdata(to_spi_device(dev));

	return l3g4200d_common_suspend(cdata);
}

static int l3g4200d_resume(struct device *dev)
{
	struct l3g4200d_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return l3g4200d_common_resume(cdata);
}

static const struct dev_pm_ops l3g4200d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(l3g4200d_suspend, l3g4200d_resume)
};

#define L3G4200D_PM_OPS		(&l3g4200d_pm_ops)
#else /* CONFIG_PM */
#define L3G4200D_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id l3g4200d_ids[] = {
	{ L3G4200D_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, l3g4200d_ids);

#ifdef CONFIG_OF
static const struct of_device_id l3g4200d_id_table[] = {
	{ .compatible = "st,l3g4200d", },
	{ },
};
MODULE_DEVICE_TABLE(of, l3g4200d_id_table);
#endif

static struct spi_driver l3g4200d_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = L3G4200D_DEV_NAME,
#ifdef CONFIG_PM
		.pm = L3G4200D_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = l3g4200d_id_table,
#endif
	},
	.remove = l3g4200d_spi_remove,
	.probe = l3g4200d_spi_probe,
	.id_table = l3g4200d_ids,
};

module_spi_driver(l3g4200d_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics l3g4200d spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
