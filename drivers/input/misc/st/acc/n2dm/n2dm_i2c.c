/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: n2dm_i2c.c
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

#include "n2dm.h"

#define	I2C_AUTO_INCREMENT	0x80

static int n2dm_acc_i2c_read(struct n2dm_acc_status *stat, u8 reg_addr,
			       int len, u8 *data)
{
	struct i2c_msg msg[2];
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

	return i2c_transfer(client->adapter, msg, 2);
}

static int n2dm_acc_i2c_write(struct n2dm_acc_status *stat, u8 reg_addr,
				int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(stat->dev);

	reg_addr |= ((len > 1) ? I2C_AUTO_INCREMENT : 0);
	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static struct n2dm_acc_transfer_function n2dm_acc_tf_i2c = {
	.write = n2dm_acc_i2c_write,
	.read = n2dm_acc_i2c_read,
};

static int n2dm_acc_i2c_probe(struct i2c_client *client,
			        const struct i2c_device_id *id)
{
	int err;
	struct n2dm_acc_status *stat;

	stat = kmalloc(sizeof(struct n2dm_acc_status), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bustype = BUS_I2C;
	stat->tf = &n2dm_acc_tf_i2c;
	i2c_set_clientdata(client, stat);

	err = n2dm_acc_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);
	return err;
}

static int n2dm_acc_i2c_remove(struct i2c_client *client)
{
	struct n2dm_acc_status *stat = i2c_get_clientdata(client);

	n2dm_acc_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int n2dm_acc_suspend(struct device *dev)
{
	struct n2dm_acc_status *stat = i2c_get_clientdata(to_i2c_client(dev));

	return n2dm_acc_common_suspend(stat);
}

static int n2dm_acc_resume(struct device *dev)
{
	struct n2dm_acc_status *stat = i2c_get_clientdata(to_i2c_client(dev));

	return n2dm_acc_common_resume(stat);
}

static const struct dev_pm_ops n2dm_acc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(n2dm_acc_suspend, n2dm_acc_resume)
};

#define N2DM_PM_OPS		(&n2dm_acc_pm_ops)
#else /* CONFIG_PM */
#define N2DM_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id n2dm_acc_ids[] = {
	{ N2DM_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, n2dm_acc_ids);

#ifdef CONFIG_OF
static const struct of_device_id n2dm_acc_id_table[] = {
	{ .compatible = "st,n2dm", },
	{ },
};
MODULE_DEVICE_TABLE(of, n2dm_acc_id_table);
#endif

static struct i2c_driver n2dm_acc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = N2DM_ACC_DEV_NAME,
#ifdef CONFIG_PM
		.pm = N2DM_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = n2dm_acc_id_table,
#endif
	},
	.remove = n2dm_acc_i2c_remove,
	.probe = n2dm_acc_i2c_probe,
	.id_table = n2dm_acc_ids,
};

module_i2c_driver(n2dm_acc_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics n2dm i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
