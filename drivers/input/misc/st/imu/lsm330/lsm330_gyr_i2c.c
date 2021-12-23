/*
 * STMicroelectronics lsm330_gyr_i2c.c driver
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

#include "lsm330.h"

#define I2C_AUTO_INCREMENT	0x80

/* XXX: caller must hold stat->lock */
static int lsm330_gyr_i2c_read(struct device *dev, u8 addr, int len,
			       u8 *data)
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

/* XXX: caller must hold stat->lock */
static int lsm330_gyr_i2c_write(struct device *dev, u8 addr, int len,
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

static struct lsm330_transfer_function lsm330_gyr_i2c_tf = {
	.write = lsm330_gyr_i2c_write,
	.read = lsm330_gyr_i2c_read,
};

#ifdef CONFIG_PM
static int lsm330_gyr_i2c_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

	return lsm330_gyr_enable(stat);
}

static int lsm330_gyr_i2c_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

	return lsm330_gyr_disable(stat);
}

static const struct dev_pm_ops lsm330_gyr_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm330_gyr_i2c_suspend,
				lsm330_gyr_i2c_resume)
};
#endif /* CONFIG_PM */

static int lsm330_gyr_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct lsm330_gyr_status *stat;

#ifdef LSM303D_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	stat = kzalloc(sizeof(struct lsm330_gyr_status), GFP_KERNEL);
	if (!stat) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bus_type = BUS_I2C;
	stat->tf = &lsm330_gyr_i2c_tf;

	i2c_set_clientdata(client, stat);

	mutex_init(&stat->lock);

	err = lsm330_gyr_probe(stat);
	if (err < 0) {
		kfree(stat);

		return err;
	}

	return 0;
}

int lsm330_gyr_i2c_remove(struct i2c_client *client)
{
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

#ifdef LSM303D_DEBUG
	dev_info(stat->dev, "driver removing\n");
#endif

	lsm330_gyr_remove(stat);
	kfree(stat);

	return 0;
}

static const struct i2c_device_id lsm330_gyr_i2c_id[] = {
	{ "lsm330_gyr", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lsm330_gyr_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id lsm330_gyr_i2c_id_table[] = {
	{ .compatible = "st,lsm330_gyr", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm330_gyr_i2c_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver lsm330_gyr_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm330_gyr_i2c",
#ifdef CONFIG_PM
		.pm = &lsm330_gyr_i2c_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = lsm330_gyr_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm330_gyr_i2c_probe,
	.remove = lsm330_gyr_i2c_remove,
	.id_table = lsm330_gyr_i2c_id,
};

module_i2c_driver(lsm330_gyr_i2c_driver);

MODULE_DESCRIPTION("lsm330 gyro i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

