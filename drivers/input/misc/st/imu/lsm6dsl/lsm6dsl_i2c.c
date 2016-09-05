/*
 * STMicroelectronics lsm6dsl i2c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Mario Tesi <mario.tesi@st.com>
 * v 1.2.2
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "lsm6dsl_core.h"

static int lsm6dsl_i2c_read(struct lsm6dsl_data *cdata, u8 reg_addr, int len,
			    u8 *data, bool b_lock)
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

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, msg, 2);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		err = i2c_transfer(client->adapter, msg, 2);

	return err;
}

static int lsm6dsl_i2c_write(struct lsm6dsl_data *cdata, u8 reg_addr, int len,
			     u8 *data, bool b_lock)
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

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		err = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		err = i2c_transfer(client->adapter, &msg, 1);

	return err;
}


static const struct lsm6dsl_transfer_function lsm6dsl_tf_i2c = {
	.write = lsm6dsl_i2c_write,
	.read = lsm6dsl_i2c_read,
};

static int lsm6dsl_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct lsm6dsl_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &lsm6dsl_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = lsm6dsl_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int lsm6dsl_i2c_remove(struct i2c_client *client)
{
	struct lsm6dsl_data *cdata = i2c_get_clientdata(client);

	lsm6dsl_common_remove(cdata, client->irq);
	dev_info(cdata->dev, "%s: removed\n", LSM6DSL_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm6dsl_suspend(struct device *dev)
{
	struct lsm6dsl_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6dsl_common_suspend(cdata);
}

static int lsm6dsl_resume(struct device *dev)
{
	struct lsm6dsl_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lsm6dsl_common_resume(cdata);
}

static const struct dev_pm_ops lsm6dsl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6dsl_suspend, lsm6dsl_resume)
};

#define LSM6DSL_PM_OPS		(&lsm6dsl_pm_ops)
#else /* CONFIG_PM */
#define LSM6DSL_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm6dsl_ids[] = {
	{ LSM6DSL_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm6dsl_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6dsl_id_table[] = {
	{ .compatible = "st,lsm6dsl", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6dsl_id_table);
#endif

static struct i2c_driver lsm6dsl_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DSL_DEV_NAME,
		.pm = LSM6DSL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6dsl_id_table,
#endif
	},
	.probe    = lsm6dsl_i2c_probe,
	.remove   = lsm6dsl_i2c_remove,
	.id_table = lsm6dsl_ids,
};

module_i2c_driver(lsm6dsl_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6dsl i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
