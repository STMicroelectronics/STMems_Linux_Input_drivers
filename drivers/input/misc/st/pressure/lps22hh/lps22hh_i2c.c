/*
 * STMicroelectronics lps22hh i2c driver
 *
 * Copyright 2020 STMicroelectronics Inc.
 *
 * Authors: AMG MSD DIVISION
 *        : Mario Tesi (mario.tesi@st.com)
 *
 * Version: 1.0.0
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include "lps22hh.h"

static int lps22hh_i2c_read(struct lps22hh_data *cdata, u8 reg_addr, int len,
			    u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);

	return (err == 2) ? 0 : 1;
}

static int lps22hh_i2c_write(struct lps22hh_data *cdata, u8 reg_addr, int len,
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

	return (err == 1) ? 0 : 1;
}

static const struct lps22hh_transfer_function lps22hh_tf_i2c = {
	.write = lps22hh_i2c_write,
	.read = lps22hh_i2c_read,
};

static int lps22hh_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct lps22hh_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->irq = client->irq;
	cdata->tf = &lps22hh_tf_i2c;
	cdata->bustype = BUS_I2C;
	cdata->name = client->name;
	i2c_set_clientdata(client, cdata);

	err = lps22hh_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int lps22hh_i2c_remove(struct i2c_client *client)
{
	struct lps22hh_data *cdata = i2c_get_clientdata(client);

	lps22hh_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", LPS22HH_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lps22hh_suspend(struct device *dev)
{
	struct lps22hh_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lps22hh_common_suspend(cdata);
}

static int lps22hh_resume(struct device *dev)
{
	struct lps22hh_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lps22hh_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lps22hh_pm_ops, lps22hh_suspend, lps22hh_resume);

#define LPS22HH_PM_OPS		(&lps22hh_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LPS22HH_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id lps22hh_ids[] = {
	{ LPS22HH_DEV_NAME },
	{ LPS22CH_DEV_NAME },
	{ LPS27HHW_DEV_NAME },
	{ LPS27HHTW_DEV_NAME },
	{}
};
MODULE_DEVICE_TABLE(i2c, lps22hh_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps22hh_id_table[] = {
	{
		.compatible = "st,lps22hh",
		.data = LPS22HH_DEV_NAME,
	},
	{
		.compatible = "st,lps22ch",
		.data = LPS22CH_DEV_NAME,
	},
	{
		.compatible = "st,lps27hhw",
		.data = LPS27HHW_DEV_NAME,
	},
	{
		.compatible = "st,lps27hhtw",
		.data = LPS27HHTW_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, lps22hh_id_table);
#endif

static struct i2c_driver lps22hh_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st_lps22hh_i2c_input_drv",
		.pm = LPS22HH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lps22hh_id_table),
#endif
	},
	.probe    = lps22hh_i2c_probe,
	.remove   = lps22hh_i2c_remove,
	.id_table = lps22hh_ids,
};

module_i2c_driver(lps22hh_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22hh i2c driver");
MODULE_AUTHOR("AMG MSD DIVISION, STMicroelectronics");
MODULE_VERSION(LPS22HH_MODULE_VERSION);
MODULE_LICENSE("GPL v2");
