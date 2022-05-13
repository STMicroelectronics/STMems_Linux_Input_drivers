/*
 * STMicroelectronics lsm6dsox i2c driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "lsm6dsox_core.h"

static int lsm6dsox_i2c_read(struct lsm6dsox_data *cdata, u8 reg_addr,
			     int len, u8 *data, bool b_lock)
{
	struct i2c_client *client = to_i2c_client(cdata->dev);
	struct i2c_msg msg[2];
	int err = 0;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	if (b_lock) {
		mutex_lock(&cdata->reg_lock);
		err = i2c_transfer(client->adapter, msg, 2);
		mutex_unlock(&cdata->reg_lock);
	} else {
		err = i2c_transfer(client->adapter, msg, 2);
	}

	return err;
}

static int lsm6dsox_i2c_write(struct lsm6dsox_data *cdata, u8 reg_addr,
			      int len, u8 *data, bool b_lock)
{
	struct i2c_client *client = to_i2c_client(cdata->dev);
	struct i2c_msg msg;
	u8 send[len + 1];
	int err = 0;

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	if (b_lock) {
		mutex_lock(&cdata->reg_lock);
		err = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&cdata->reg_lock);
	} else {
		err = i2c_transfer(client->adapter, &msg, 1);
	}

	return err;
}


static const struct lsm6dsox_transfer_function lsm6dsox_tf_i2c = {
	.write = lsm6dsox_i2c_write,
	.read = lsm6dsox_i2c_read,
};

static int lsm6dsox_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct lsm6dsox_data *cdata;
	int hw_id = id->driver_data;
	int err;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &lsm6dsox_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = lsm6dsox_common_probe(cdata, client->irq, hw_id, BUS_I2C);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lsm6dsox_i2c_remove(struct i2c_client *client)
{
	struct lsm6dsox_data *cdata = i2c_get_clientdata(client);

	lsm6dsox_common_remove(cdata, client->irq);
	dev_info(cdata->dev, "%s: removed\n", LSM6DSOX_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lsm6dsox_suspend(struct device *dev)
{
	struct lsm6dsox_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6dsox_common_suspend(cdata);
}

static int lsm6dsox_resume(struct device *dev)
{
	struct lsm6dsox_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6dsox_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lsm6dsox_pm_ops, lsm6dsox_suspend, lsm6dsox_resume);

#define LSM6DSOX_PM_OPS		(&lsm6dsox_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM6DSOX_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id lsm6dsox_ids[] = {
	{ LSM6DSO_DEV_NAME, LSM6DSO_ID },
	{ LSM6DSOX_DEV_NAME, LSM6DSOX_ID },
	{ LSM6DSO32_DEV_NAME, LSM6DSO32_ID },
	{ LSM6DSO32X_DEV_NAME, LSM6DSO32X_ID },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm6dsox_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6dsox_id_table[] = {
	{
		.compatible = "st,lsm6dso",
		.data = (void *)LSM6DSO_ID,
	},
	{
		.compatible = "st,lsm6dsox",
		.data = (void *)LSM6DSOX_ID,
	},
	{
		.compatible = "st,lsm6dso32",
		.data = (void *)LSM6DSO32_ID,
	},
	{
		.compatible = "st,lsm6dso32x",
		.data = (void *)LSM6DSO32X_ID,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6dsox_id_table);
#endif

static struct i2c_driver lsm6dsox_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DSOX_DEV_NAME,
		.pm = LSM6DSOX_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6dsox_id_table,
#endif
	},
	.probe    = lsm6dsox_i2c_probe,
	.remove   = lsm6dsox_i2c_remove,
	.id_table = lsm6dsox_ids,
};

module_i2c_driver(lsm6dsox_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6dsox i2c driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
