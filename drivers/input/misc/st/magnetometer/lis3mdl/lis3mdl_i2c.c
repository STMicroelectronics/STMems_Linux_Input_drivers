/*
 * STMicroelectronics lis3mdl_i2c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/input.h>

#include "lis3mdl.h"

#define I2C_AUTO_INCREMENT	0x80

/* XXX: caller must hold dev->lock */
static int lis3mdl_i2c_read(struct device *dev, u8 addr, int len, u8 *data)
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
static int lis3mdl_i2c_write(struct device *dev, u8 addr, int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(dev);

	if (len > 1)
		addr |= I2C_AUTO_INCREMENT;

	send[0] = addr;
	memcpy(&send[1], data, len * sizeof(u8));

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len + 1;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static const struct lis3mdl_transfer_function lis3mdl_i2c_tf = {
	.write = lis3mdl_i2c_write,
	.read = lis3mdl_i2c_read,
};

#ifdef CONFIG_PM_SLEEP
static int lis3mdl_i2c_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lis3mdl_dev *dev = i2c_get_clientdata(client);

	return lis3mdl_mag_enable(dev);
}

static int lis3mdl_i2c_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct lis3mdl_dev *dev = i2c_get_clientdata(client);

	return lis3mdl_mag_disable(dev);
}

static SIMPLE_DEV_PM_OPS(lis3mdl_i2c_pm_ops,
				lis3mdl_i2c_suspend,
				lis3mdl_i2c_resume);

#define LIS3MDL_PM_OPS		(&lis3mdl_i2c_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS3MDL_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static int lis3mdl_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int err;
	struct lis3mdl_dev *dev;

#ifdef LIS3MDL_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct lis3mdl_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->name = client->name;
	dev->bus_type = BUS_I2C;
	dev->tf = &lis3mdl_i2c_tf;
	dev->dev = &client->dev;

	i2c_set_clientdata(client, dev);

	mutex_init(&dev->lock);

	err = lis3mdl_mag_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

int lis3mdl_i2c_remove(struct i2c_client *client)
{
	struct lis3mdl_dev *dev = i2c_get_clientdata(client);

#ifdef LIS3MDL_DEBUG
	dev_info(&client->dev, "driver removing\n");
#endif

	lis3mdl_mag_remove(dev);
	kfree(dev);

	return 0;
}

static const struct i2c_device_id lis3mdl_i2c_id[] = {
	{ "lis3mdl", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lis3mdl_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id lis3mdl_i2c_id_table[] = {
	{ .compatible = "st,lis3mdl" },
	{ },
};
MODULE_DEVICE_TABLE(of, lis3mdl_i2c_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver lis3mdl_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lis3mdl_i2c",
		.pm = LIS3MDL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis3mdl_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lis3mdl_i2c_probe,
	.remove = lis3mdl_i2c_remove,
	.id_table = lis3mdl_i2c_id,
};

module_i2c_driver(lis3mdl_i2c_driver);

MODULE_DESCRIPTION("lis3mdl i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

