/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: h3lis331dl_i2c.c
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
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/delay.h>

#include "h3lis331dl.h"

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5

#ifdef HAS_IF_AUTO_INCREMENT
#define	I2C_AUTO_INCREMENT	0x80
#endif

static int h3lis331dl_i2c_read(struct h3lis331dl_data *stat, u8 reg_addr,
			       int len, u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	int tries = 0;
	struct i2c_client *client = to_i2c_client(stat->dev);

#ifdef HAS_IF_AUTO_INCREMENT
	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
#endif
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

static int h3lis331dl_i2c_write(struct h3lis331dl_data *stat, u8 reg_addr,
				int len, u8 *data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	int tries = 0;
	struct i2c_client *client = to_i2c_client(stat->dev);

#ifdef HAS_IF_AUTO_INCREMENT
	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
#endif
	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1)
		return -EIO;

	return 0;
}

static struct h3lis331dl_transfer_function h3lis331dl_tf_i2c = {
	.write = h3lis331dl_i2c_write,
	.read = h3lis331dl_i2c_read,
};

static int h3lis331dl_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct h3lis331dl_data *stat;

	stat = kzalloc(sizeof(struct h3lis331dl_data), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bustype = BUS_I2C;
	stat->tf = &h3lis331dl_tf_i2c;
	i2c_set_clientdata(client, stat);

	err = h3lis331dl_common_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

static int h3lis331dl_i2c_remove(struct i2c_client *client)
{
	struct h3lis331dl_data *stat = i2c_get_clientdata(client);

	h3lis331dl_common_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int h3lis331dl_suspend(struct device *dev)
{
	struct h3lis331dl_data *stat = i2c_get_clientdata(to_i2c_client(dev));

	return h3lis331dl_common_suspend(stat);
}

static int h3lis331dl_resume(struct device *dev)
{
	struct h3lis331dl_data *stat = i2c_get_clientdata(to_i2c_client(dev));

	return h3lis331dl_common_resume(stat);
}

static const struct dev_pm_ops h3lis331dl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(h3lis331dl_suspend, h3lis331dl_resume)
};

#define H3LIS331DL_PM_OPS	(&h3lis331dl_pm_ops)
#else /* CONFIG_PM */
#define H3LIS331DL_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id h3lis331dl_ids[] = {
	{ H3LIS331DL_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, h3lis331dl_ids);

#ifdef CONFIG_OF
static const struct of_device_id h3lis331dl_id_table[] = {
	{ .compatible = "st,h3lis331dl", },
	{ },
};
MODULE_DEVICE_TABLE(of, h3lis331dl_id_table);
#endif

static struct i2c_driver h3lis331dl_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = H3LIS331DL_ACC_DEV_NAME,
#ifdef CONFIG_PM
		.pm = H3LIS331DL_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = h3lis331dl_id_table,
#endif
	},
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,3,0)
	.remove = __devexit_p(h3lis331dl_i2c_remove),
#else
	.remove = h3lis331dl_i2c_remove,
#endif /* LINUX_VERSION_CODE <= KERNEL_VERSION(3,3,0) */
	.probe    = h3lis331dl_i2c_probe,
	.id_table = h3lis331dl_ids,
};

module_i2c_driver(h3lis331dl_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics h3lis331dl i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
