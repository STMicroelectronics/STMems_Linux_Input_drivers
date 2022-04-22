/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis2hh12_i2c.c
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
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/delay.h>

#include "lis2hh12.h"

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

static int lis2hh12_i2c_read(struct lis2hh12_status *stat, u8 reg_addr,
			     int len, u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	int tries = 0;
	struct i2c_client *client = to_i2c_client(stat->dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
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

static int lis2hh12_i2c_write(struct lis2hh12_status *stat, u8 reg_addr,
			      int len, u8 *data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	int tries = 0;
	struct i2c_client *client = to_i2c_client(stat->dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
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

static struct lis2hh12_transfer_function lis2hh12_tf_i2c = {
	.write = lis2hh12_i2c_write,
	.read = lis2hh12_i2c_read,
};

static int lis2hh12_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int err;
	struct lis2hh12_status *stat;

	stat = kmalloc(sizeof(struct lis2hh12_status), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bustype = BUS_I2C;
	stat->tf = &lis2hh12_tf_i2c;
	i2c_set_clientdata(client, stat);

	err = lis2hh12_common_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

static int lis2hh12_i2c_remove(struct i2c_client *client)
{
	struct lis2hh12_status *stat = i2c_get_clientdata(client);

	lis2hh12_common_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lis2hh12_suspend(struct device *dev)
{
	struct lis2hh12_status *stat = i2c_get_clientdata(to_i2c_client(dev));

	return lis2hh12_common_suspend(stat);
}

static int lis2hh12_resume(struct device *dev)
{
	struct lis2hh12_status *stat = i2c_get_clientdata(to_i2c_client(dev));

	return lis2hh12_common_resume(stat);
}

static SIMPLE_DEV_PM_OPS(lis2hh12_pm_ops, lis2hh12_suspend, lis2hh12_resume);

#define LIS2HH12_PM_OPS	(&lis2hh12_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS2HH12_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id lis2hh12_ids[] = {
	{ LIS2HH12_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lis2hh12_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis2hh12_id_table[] = {
	{ .compatible = "st,lis2hh12", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2hh12_id_table);
#endif

static struct i2c_driver lis2hh12_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2HH12_ACC_DEV_NAME,
		.pm = LIS2HH12_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis2hh12_id_table,
#endif
	},
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,3,0)
	.remove = __devexit_p(lis2hh12_i2c_remove),
#else
	.remove = lis2hh12_i2c_remove,
#endif /* LINUX_VERSION_CODE <= KERNEL_VERSION(3,3,0) */
	.probe    = lis2hh12_i2c_probe,
	.id_table = lis2hh12_ids,
};

module_i2c_driver(lis2hh12_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2hh12 i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
