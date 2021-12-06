// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Matteo Dameno <matteo.dameno@st.com>
 *
 * Licensed under the GPL-2.
 */

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#include <linux/input.h>

#include "lps22df.h"

static int lps22df_i2c_read(struct lps22df_prs_data *cdata, u8 reg_addr,
							int len, u8 *data)
{
	int err;
	enum { msg_size = 2 };

	struct i2c_msg msg[msg_size];
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (len > LPS22DF_RX_MAX_LENGTH)
		return -ENOMEM;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, msg_size);

	return (err == msg_size) ? 0 : ((err < 0) ? err : -EIO);
}

static int lps22df_i2c_write(struct lps22df_prs_data *cdata, u8 reg_addr,
							int len, u8 *data)
{
	int err;
	enum { msg_size = 1 };

	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (len > LPS22DF_TX_MAX_LENGTH)
		return -ENOMEM;

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len + 1;
	msg.buf = send;

	err = i2c_transfer(client->adapter, &msg, msg_size);

	return (err == msg_size) ? 0 : ((err < 0) ? err : -EIO);
}

static const struct lps22df_prs_transfer_function lps22df_tf_i2c = {
	.write = lps22df_i2c_write,
	.read = lps22df_i2c_read,
};

static int lps22df_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct lps22df_prs_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->irq = client->irq;
	cdata->tf = &lps22df_tf_i2c;
	cdata->bustype = BUS_I2C;
	cdata->name = client->name;
	i2c_set_clientdata(client, cdata);

	err = lps22df_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int lps22df_i2c_remove(struct i2c_client *client)
{
	struct lps22df_prs_data *cdata = i2c_get_clientdata(client);

	lps22df_common_remove(cdata);
	pr_info("%s: removed\n", LPS22DF_PRS_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lps22df_suspend(struct device *dev)
{
	struct lps22df_prs_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lps22df_common_suspend(cdata);
}

static int lps22df_resume(struct device *dev)
{
	struct lps22df_prs_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lps22df_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lps22df_pm_ops, lps22df_suspend, lps22df_resume);

#define LPS22DF_PM_OPS		(&lps22df_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LPS22DF_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id lps22df_ids[] = {
	{ LPS22DF_PRS_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lps22df_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps22df_id_table[] = {
	{
		.compatible = "st,lps22df",
		.data = LPS22DF_PRS_DEV_NAME,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lps22df_id_table);
#endif

static struct i2c_driver lps22df_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LPS22DF_PRS_DEV_NAME,
		.pm = LPS22DF_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lps22df_id_table),
#endif
	},
	.probe    = lps22df_i2c_probe,
	.remove   = lps22df_i2c_remove,
	.id_table = lps22df_ids,
};

module_i2c_driver(lps22df_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22df i2c driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_LICENSE("GPL v2");
