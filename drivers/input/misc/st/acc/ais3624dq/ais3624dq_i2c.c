/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: ais3624dq_i2c.c
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		: Lorenzo Bianconi <lorenzo.bianconi@st.com>
*		: Author is willing to be considered the contact and update
* Version	: V.1.0.0
* Date		: 2016/May/27
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

#include "ais3624dq.h"

#define I2C_AUTO_INCREMENT	0x80
#define I2C_RETRIES		5
#define I2C_RETRY_DELAY		5

static int ais3624dq_i2c_read(struct device *dev, u8 reg_addr, int len,
			      u8 *data)
{
	int err = 0, tries = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = (client->flags &I2C_M_TEN) | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	do {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while (err != 2 && ++tries < I2C_RETRIES);

	return (err != 2) ? -EIO : err;
}

static int ais3624dq_i2c_write(struct device *dev, u8 reg_addr, int len,
			       u8 *data)
{
	int err = 0, tries = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = len;
	msg.buf = send;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while (err != 1 && ++tries < I2C_RETRIES);

	return (err != 1) ? -EIO : err;
}

static struct ais3624dq_transfer_function ais3624dq_tf_i2c = {
	.write = ais3624dq_i2c_write,
	.read = ais3624dq_i2c_read,
};

static int ais3624dq_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int err;
	struct ais3624dq_acc_data *acc;

	acc = kmalloc(sizeof(struct ais3624dq_acc_data), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;

	acc->dev = &client->dev;
	acc->name = client->name;
	acc->bus_type = BUS_I2C;
	acc->tf = &ais3624dq_tf_i2c;
	i2c_set_clientdata(client, acc);

	err = ais3624dq_acc_probe(acc);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(acc);
	return err;
}

static int ais3624dq_i2c_remove(struct i2c_client *client)
{
	struct ais3624dq_acc_data *acc = i2c_get_clientdata(client);

	ais3624dq_acc_remove(acc);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int ais3624dq_suspend(struct device *dev)
{
	struct ais3624dq_acc_data *acc = i2c_get_clientdata(to_i2c_client(dev));

	return ais3624dq_acc_disable(acc);
}

static int ais3624dq_resume(struct device *dev)
{
	struct ais3624dq_acc_data *acc = i2c_get_clientdata(to_i2c_client(dev));

	return ais3624dq_acc_enable(acc);
}

static const struct dev_pm_ops ais3624dq_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ais3624dq_suspend, ais3624dq_resume)
};
#endif /* CONFIG_PM */

static const struct i2c_device_id ais3624dq_ids[] = {
	{ AIS3624DQ_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ais3624dq_ids);

#ifdef CONFIG_OF
static const struct of_device_id ais3624dq_id_table[] = {
	{ .compatible = "st,ais3624dq", },
	{ },
};
MODULE_DEVICE_TABLE(of, ais3624dq_id_table);
#endif

static struct i2c_driver ais3624dq_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AIS3624DQ_ACC_DEV_NAME,
#ifdef CONFIG_PM
		.pm = &ais3624dq_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = ais3624dq_id_table,
#endif
	},
	.remove = ais3624dq_i2c_remove,
	.probe = ais3624dq_i2c_probe,
	.id_table = ais3624dq_ids,
};

module_i2c_driver(ais3624dq_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics ais3624dq i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_LICENSE("GPL v2");
