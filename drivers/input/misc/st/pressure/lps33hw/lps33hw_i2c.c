/*
 * STMicroelectronics lps33hw i2c driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 * v 1.0.0
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

#include "lps33hw.h"

static int lps33hw_i2c_read(struct lps33_prs_data *cdata, u8 reg_addr, int len,
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

static int lps33hw_i2c_write(struct lps33_prs_data *cdata, u8 reg_addr, int len,
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

static const struct lps33_prs_transfer_function lps33hw_tf_i2c = {
	.write = lps33hw_i2c_write,
	.read = lps33hw_i2c_read,
};

static int lps33hw_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct lps33_prs_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->irq = client->irq;
	cdata->tf = &lps33hw_tf_i2c;
	cdata->bustype = BUS_I2C;
	cdata->name = client->name;
	i2c_set_clientdata(client, cdata);

	err = lps33hw_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int lps33hw_i2c_remove(struct i2c_client *client)
{
	struct lps33_prs_data *cdata = i2c_get_clientdata(client);

	lps33hw_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", LPS33_PRS_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lps33hw_suspend(struct device *dev)
{
	struct lps33_prs_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lps33hw_common_suspend(cdata);
}

static int lps33hw_resume(struct device *dev)
{
	struct lps33_prs_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lps33hw_common_resume(cdata);
}

static const struct dev_pm_ops lps33hw_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lps33hw_suspend, lps33hw_resume)
};

#define LPS33HW_PM_OPS		(&lps33hw_pm_ops)
#else /* CONFIG_PM */
#define LPS33HW_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lps33hw_ids[] = {
	{ LPS33_PRS_DEV_NAME },
	{ LPS35_PRS_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lps33hw_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps33hw_id_table[] = {
	{
		.compatible = "st,lps33hw",
		.data = LPS33_PRS_DEV_NAME,
	},
	{
		.compatible = "st,lps35hw",
		.data = LPS35_PRS_DEV_NAME,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lps33hw_id_table);
#endif

static struct i2c_driver lps33hw_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LPS33_PRS_DEV_NAME,
		.pm = LPS33HW_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lps33hw_id_table),
#endif
	},
	.probe    = lps33hw_i2c_probe,
	.remove   = lps33hw_i2c_remove,
	.id_table = lps33hw_ids,
};

module_i2c_driver(lps33hw_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lps33hw i2c driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
