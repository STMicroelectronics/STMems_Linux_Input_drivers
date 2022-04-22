/*
 * STMicroelectronics l3gd20h_gyr_i2c.c driver
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

#include "l3gd20h.h"

#define I2C_AUTO_INCREMENT	0x80

/* XXX: caller must hold dev->lock */
static int l3gd20h_gyr_i2c_read(struct device *dev, u8 addr, int len, u8 *data)
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
static int l3gd20h_gyr_i2c_write(struct device *dev, u8 addr, int len,
				 u8 *data)
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

static struct l3gd20h_gyr_transfer_function l3gd20h_gyr_i2c_tf = {
	.write = l3gd20h_gyr_i2c_write,
	.read = l3gd20h_gyr_i2c_read,
};

#ifdef CONFIG_PM_SLEEP
static int l3gd20h_gyr_if_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct l3gd20h_gyr_status *stat = i2c_get_clientdata(client);

	return l3gd20h_gyr_resume(stat);
}

static int l3gd20h_gyr_if_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct l3gd20h_gyr_status *stat = i2c_get_clientdata(client);

	return l3gd20h_gyr_suspend(stat);
}

static SIMPLE_DEV_PM_OPS(l3gd20h_gyr_pm_ops,
				l3gd20h_gyr_if_suspend,
				l3gd20h_gyr_if_resume);

#define L3GD20H_PM_OPS	(&l3gd20h_gyr_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define L3GD20H_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static int l3gd20h_gyr_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int err;
	struct l3gd20h_gyr_status *stat;

#ifdef L3GD20H_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	stat = kzalloc(sizeof(struct l3gd20h_gyr_status), GFP_KERNEL);
	if (!stat) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bus_type = BUS_I2C;
	stat->tf = &l3gd20h_gyr_i2c_tf;

	i2c_set_clientdata(client, stat);

	mutex_init(&stat->lock);

	err = l3gd20h_gyr_probe(stat);
	if (err < 0) {
		kfree(stat);

		return err;
	}

	return 0;
}

int l3gd20h_gyr_i2c_remove(struct i2c_client *client)
{
	struct l3gd20h_gyr_status *stat = i2c_get_clientdata(client);

#ifdef L3GD20H_DEBUG
	dev_info(stat->dev, "driver removing\n");
#endif

	l3gd20h_gyr_remove(stat);
	kfree(stat);

	return 0;
}

static const struct i2c_device_id l3gd20h_gyr_i2c_id[] = {
	{ "l3gd20h_gyr", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, l3gd20h_gyr_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id l3gd20h_gyr_i2c_id_table[] = {
	{ .compatible = "st,l3gd20h_gyr", },
	{ },
};
MODULE_DEVICE_TABLE(of, l3gd20h_gyr_i2c_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver l3gd20h_gyr_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "l3gd20h_gyr_i2c",
		.pm = L3GD20H_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = l3gd20h_gyr_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = l3gd20h_gyr_i2c_probe,
	.remove = l3gd20h_gyr_i2c_remove,
	.id_table = l3gd20h_gyr_i2c_id,
};

module_i2c_driver(l3gd20h_gyr_i2c_driver);

MODULE_DESCRIPTION("l3gd20h gyro i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

