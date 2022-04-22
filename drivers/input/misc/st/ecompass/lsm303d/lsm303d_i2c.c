/*
 * STMicroelectronics lsm303d_i2c.c driver
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
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "lsm303d.h"

#define I2C_AUTO_INCREMENT	0x80

/* XXX: caller must hold dev->lock */
static int lsm303d_i2c_read(struct device *dev, u8 addr, int len, u8 *data)
{
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(dev);

	if (len > 1)
		addr |= I2C_AUTO_INCREMENT;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	return i2c_transfer(client->adapter, msg, 2);
}

/* XXX: caller must hold dev->lock */
static int lsm303d_i2c_write(struct device *dev, u8 addr, int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	if (len > 1)
		addr |= I2C_AUTO_INCREMENT;

	send[0] = addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static struct lsm303d_transfer_function lsm303d_i2c_tf = {
	.write = lsm303d_i2c_write,
	.read = lsm303d_i2c_read,
};

#ifdef CONFIG_PM_SLEEP
static int lsm303d_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm303d_dev *dev = i2c_get_clientdata(client);

	return lsm303d_enable(dev);
}

static int lsm303d_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm303d_dev *dev = i2c_get_clientdata(client);

	return lsm303d_disable(dev);
}

static SIMPLE_DEV_PM_OPS(lsm303d_pm_ops, lsm303d_suspend, lsm303d_resume);

#define LSM303D_PM_OPS	(&lsm303d_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM303D_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id lsm303d_i2c_id_table[] = {
	{ .compatible = "st,lsm303d", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303d_i2c_id_table);
#endif /* CONFIG_OF */

static int lsm303d_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct lsm303d_dev *dev;

#ifdef LSM303D_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct lsm303d_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->dev = &client->dev;
	dev->name = client->name;
	dev->bus_type = BUS_I2C;
	dev->st.dev = dev;
	dev->tf = &lsm303d_i2c_tf;

	i2c_set_clientdata(client, dev);

#ifdef CONFIG_OF
	dev->dev_id = lsm303d_i2c_id_table;
#endif

	err = lsm303d_probe(dev);
	if (err < 0) {
		kfree(dev);

		return err;
	}

	return 0;
}

int lsm303d_i2c_remove(struct i2c_client *client)
{
	struct lsm303d_dev *dev = i2c_get_clientdata(client);

#ifdef LSM303D_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm303d_remove(dev);
	kfree(dev);

	return 0;
}

static const struct i2c_device_id lsm303d_i2c_id[] = {
	{ "lsm303d", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lsm303d_i2c_id);

static struct i2c_driver lsm303d_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm303d_i2c",
		.pm = LSM303D_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm303d_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm303d_i2c_probe,
	.remove = lsm303d_i2c_remove,
	.id_table = lsm303d_i2c_id,
};

module_i2c_driver(lsm303d_i2c_driver);

MODULE_DESCRIPTION("lsm303d i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

