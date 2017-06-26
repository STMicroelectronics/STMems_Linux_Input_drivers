/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lps25h_i2c.c
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

#include "lps25h.h"

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5

#ifdef HAS_IF_AUTO_INCREMENT
#define	I2C_AUTO_INCREMENT	0x80
#endif

static int lps25h_i2c_read(struct lps25h_prs_data *stat, u8 reg_addr,
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

static int lps25h_i2c_write(struct lps25h_prs_data *stat, u8 reg_addr,
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

static struct lps25h_prs_transfer_function lps25h_tf_i2c = {
	.write = lps25h_i2c_write,
	.read = lps25h_i2c_read,
};

static int lps25h_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int err;
	struct lps25h_prs_data *stat;

	stat = kzalloc(sizeof(struct lps25h_prs_data), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bustype = BUS_I2C;
	stat->tf = &lps25h_tf_i2c;
	i2c_set_clientdata(client, stat);

	err = lps25h_common_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

static int lps25h_i2c_remove(struct i2c_client *client)
{
	struct lps25h_prs_data *stat = i2c_get_clientdata(client);

	lps25h_common_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int lps25h_suspend(struct device *dev)
{
	struct lps25h_prs_data *stat = i2c_get_clientdata(to_i2c_client(dev));

	return lps25h_common_suspend(stat);
}

static int lps25h_resume(struct device *dev)
{
	struct lps25h_prs_data *stat = i2c_get_clientdata(to_i2c_client(dev));

	return lps25h_common_resume(stat);
}

static const struct dev_pm_ops lps25h_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lps25h_suspend, lps25h_resume)
};

#define LPS25H_PM_OPS	(&lps25h_pm_ops)
#else /* CONFIG_PM */
#define LPS25H_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lps25h_ids[] = {
	{ LPS25H_PRS_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lps25h_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps25h_id_table[] = {
	{ .compatible = "st,lps25h", },
	{ .compatible = "st,lps25hb", },
	{ },
};
MODULE_DEVICE_TABLE(of, lps25h_id_table);
#endif

static struct i2c_driver lps25h_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LPS25H_PRS_DEV_NAME,
#ifdef CONFIG_PM
		.pm = LPS25H_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = lps25h_id_table,
#endif
	},
	.remove = lps25h_i2c_remove,
	.probe    = lps25h_i2c_probe,
	.id_table = lps25h_ids,
};

module_i2c_driver(lps25h_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lps25h i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
