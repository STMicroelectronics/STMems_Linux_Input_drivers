/******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
*
* File Name          : h3lis100dl_i2c.c
* Authors            : AMS - VMU - Application Team
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.1.0
* Date               : 2016/Apr/26
* Description        : h3lis100dl i2c driver
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
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>

#include "h3lis100dl.h"

/* I2C conf. parameter */
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

static int h3lis100dl_i2c_read(struct h3lis100dl_data *cdata, u8 reg_addr, int len,
			       u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);
	int tries = 0;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	do {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	return err;
}

static int h3lis100dl_i2c_write(struct h3lis100dl_data *cdata, u8 reg_addr, int len,
				u8 *data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	err = i2c_transfer(client->adapter, &msg, 1);

	return err;
}

static const struct h3lis100dl_transfer_function h3lis100dl_tf_i2c = {
	.write = h3lis100dl_i2c_write,
	.read = h3lis100dl_i2c_read,
};

static int h3lis100dl_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct h3lis100dl_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->bustype = BUS_I2C;
	cdata->tf = &h3lis100dl_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = h3lis100dl_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int h3lis100dl_i2c_remove(struct i2c_client *client)
{
	struct h3lis100dl_data *cdata = i2c_get_clientdata(client);

	h3lis100dl_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", H3LIS100DL_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int h3lis100dl_suspend(struct device *dev)
{
	struct h3lis100dl_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return h3lis100dl_common_suspend(cdata);
}

static int h3lis100dl_resume(struct device *dev)
{
	struct h3lis100dl_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return h3lis100dl_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(h3lis100dl_pm_ops,
				h3lis100dl_suspend,
				h3lis100dl_resume);

#define H3LIS100DL_PM_OPS		(&h3lis100dl_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define H3LIS100DL_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */


static const struct i2c_device_id h3lis100dl_ids[] = {
	{H3LIS100DL_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, h3lis100dl_ids);

#ifdef CONFIG_OF
static const struct of_device_id h3lis100dl_id_i2c_table[] = {
	{ .compatible = "st,h3lis100dl", },
	{ },
};
MODULE_DEVICE_TABLE(of, h3lis100dl_id_i2c_table);
#endif

static struct i2c_driver h3lis100dl_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = H3LIS100DL_DEV_NAME,
		.pm = H3LIS100DL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = h3lis100dl_id_i2c_table,
#endif
	},
	.probe    = h3lis100dl_i2c_probe,
	.remove   = h3lis100dl_i2c_remove,
	.id_table = h3lis100dl_ids,
};

module_i2c_driver(h3lis100dl_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics h3lis100dl i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_LICENSE("GPL v2");
