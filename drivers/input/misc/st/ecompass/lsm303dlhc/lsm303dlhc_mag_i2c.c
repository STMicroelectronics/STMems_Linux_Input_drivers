/*
 * STMicroelectronics lsm303dlhc_mag_i2c.c driver
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

#include "lsm303dlhc.h"

#define I2C_AUTO_INCREMENT	0x80

/* XXX: caller must hold stat->lock */
static int lsm303dlhc_mag_i2c_read(struct device *dev, u8 addr, int len,
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
static int lsm303dlhc_mag_i2c_write(struct device *dev, u8 addr, int len,
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

static struct lsm303dlhc_transfer_function lsm303dlhc_mag_i2c_tf = {
	.write = lsm303dlhc_mag_i2c_write,
	.read = lsm303dlhc_mag_i2c_read,
};

#ifdef CONFIG_PM_SLEEP
static int lsm303dlhc_mag_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm303dlhc_mag_status *stat = i2c_get_clientdata(client);

	return lsm303dlhc_mag_enable(stat);
}

static int lsm303dlhc_mag_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lsm303dlhc_mag_status *stat = i2c_get_clientdata(client);

	return lsm303dlhc_mag_disable(stat);
}

static SIMPLE_DEV_PM_OPS(lsm303dlhc_mag_pm_ops,
			lsm303dlhc_mag_suspend,
			lsm303dlhc_mag_resume);


#define LSM303DLHC_MAG_PM_OPS	(&lsm303dlhc_mag_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM303DLHC_MAG_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id lsm303dlhc_mag_i2c_id_table[] = {
	{ .compatible = "st,lsm303dlhc_mag", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm303dlhc_mag_i2c_id_table);
#endif /* CONFIG_OF */

static int lsm303dlhc_mag_i2c_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int err;
	struct lsm303dlhc_mag_status *stat;

#ifdef LSM303DLHC_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	stat = kzalloc(sizeof(struct lsm303dlhc_mag_status), GFP_KERNEL);
	if (!stat) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bus_type = BUS_I2C;
	stat->tf = &lsm303dlhc_mag_i2c_tf;

	i2c_set_clientdata(client, stat);

	mutex_init(&stat->lock);

	err = lsm303dlhc_mag_probe(stat);
	if (err < 0) {
		kfree(stat);

		return err;
	}

	return 0;
}

int lsm303dlhc_mag_i2c_remove(struct i2c_client *client)
{
	struct lsm303dlhc_mag_status *stat = i2c_get_clientdata(client);

#ifdef LSM303DLHC_DEBUG
	dev_info(dev->dev, "driver removing\n");
#endif

	lsm303dlhc_mag_remove(stat);
	kfree(stat);

	return 0;
}

static const struct i2c_device_id lsm303dlhc_mag_i2c_id[] = {
	{ "lsm303dlhc_mag", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lsm303dlhc_mag_i2c_id);

static struct i2c_driver lsm303dlhc_mag_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lsm303dlhc_mag_i2c",
		.pm = LSM303DLHC_MAG_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm303dlhc_mag_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lsm303dlhc_mag_i2c_probe,
	.remove = lsm303dlhc_mag_i2c_remove,
	.id_table = lsm303dlhc_mag_i2c_id,
};

module_i2c_driver(lsm303dlhc_mag_i2c_driver);

MODULE_DESCRIPTION("lsm303dlhc mag i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

