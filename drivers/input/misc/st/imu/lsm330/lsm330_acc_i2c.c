/*
 * STMicroelectronics lsm330_acc_i2c.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/err.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "lsm330.h"
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

/* XXX: caller must hold acc->lock */
static int lsm330_acc_i2c_read(struct device *dev, u8 addr,
			       int len, u8 *data)
{
	int i = 0, err;
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(dev);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	do {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while (err != 2 && ++i < I2C_RETRIES);

	return (err != 2) ? -EIO : 0;
}

/* XXX: caller must hold acc->lock */
static int lsm330_acc_i2c_write(struct device *dev, u8 addr,
				int len, u8 *data)
{
	int i = 0, err;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	send[0] = addr;
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
	} while (err != 1 && ++i < I2C_RETRIES);

	return (err != 1) ? -EIO : 0;
}

static struct lsm330_transfer_function lsm330_acc_i2c_tf = {
	.write = lsm330_acc_i2c_write,
	.read = lsm330_acc_i2c_read,
};

#ifdef CONFIG_PM
static int lsm330_acc_i2c_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	return lsm330_acc_enable(acc);
}

static int lsm330_acc_i2c_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	return lsm330_acc_disable(acc);
}

static const struct dev_pm_ops lsm330_acc_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm330_acc_i2c_suspend,
				lsm330_acc_i2c_resume)
};
#endif /* CONFIG_PM */

static int lsm330_acc_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	int err;
	struct lsm330_acc_data *acc;

#ifdef LSM303D_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
	if (!acc) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	acc->dev = &client->dev;
	acc->name = client->name;
	acc->bus_type = BUS_I2C;
	acc->tf = &lsm330_acc_i2c_tf;

	i2c_set_clientdata(client, acc);

	mutex_init(&acc->lock);

	err = lsm330_acc_probe(acc);
	if (err < 0) {
		kfree(acc);

		return err;
	}

	return 0;
}

int lsm330_acc_i2c_remove(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

#ifdef LSM303D_DEBUG
	dev_info(acc->dev, "driver removing\n");
#endif

	lsm330_acc_remove(acc);
	kfree(acc);

	return 0;
}

static const struct i2c_device_id lsm330_acc_i2c_id[] = {
	{ "lsm330_acc", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lsm330_acc_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id lsm330_acc_i2c_id_table[] = {
	{ .compatible = "st,lsm330_acc", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm330_acc_i2c_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver lsm330_acc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm330_acc_i2c",
#ifdef CONFIG_PM
		.pm = &lsm330_acc_i2c_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = lsm330_acc_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm330_acc_i2c_probe,
	.remove = lsm330_acc_i2c_remove,
	.id_table = lsm330_acc_i2c_id,
};

module_i2c_driver(lsm330_acc_i2c_driver);

MODULE_DESCRIPTION("lsm330 acc i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

