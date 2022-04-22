/*
 * STMicroelectronics lis2mdl_i2c.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Armando Visconti <armando.visconti@st.com>
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "lis2mdl.h"

#define LIS2MDL_WHO_AM_I_ADDR		(0x4F)
#define LIS2MDL_WHO_AM_I_VAL		(0x40)

/* XXX: caller must hold cdata->lock */
static int lis2mdl_i2c_read(struct st_common_data *cdata,
				 u8 addr, int len, u8 *data)
{
	struct i2c_msg msg[2];
	struct i2c_client *client = to_i2c_client(cdata->dev);

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

/* XXX: caller must hold cdata->lock */
static int lis2mdl_i2c_write(struct st_common_data *cdata,
				  u8 addr, int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	send[0] = addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static struct lis2mdl_transfer_function lis2mdl_i2c_tf = {
	.write = lis2mdl_i2c_write,
	.read = lis2mdl_i2c_read,
};

#ifdef CONFIG_PM_SLEEP
static int lis2mdl_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st_common_data *cdata = i2c_get_clientdata(client);

	return lis2mdl_enable(cdata);
}

static int lis2mdl_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st_common_data *cdata = i2c_get_clientdata(client);

	return lis2mdl_disable(cdata);
}

static SIMPLE_DEV_PM_OPS(lis2mdl_pm_ops, lis2mdl_suspend, lis2mdl_resume);

#define LIS2MDL_PM_OPS		(&lis2mdl_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS2MDL_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id lis2mdl_i2c_id_table[] = {
	{ .compatible = "st,lis2mdl", },
	{ .compatible = "st,iis2mdc", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis2mdl_i2c_id_table);
#endif /* CONFIG_OF */

static int lis2mdl_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct st_common_data *cdata;

#ifdef LIS2MDL_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	cdata = kzalloc(sizeof(struct st_common_data), GFP_KERNEL);
	if (!cdata) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	cdata->sensors = (struct lis2mdl_data *) kmalloc(
						sizeof(struct lis2mdl_data),
						GFP_KERNEL);
	if (!cdata->sensors)
		return -ENOMEM;

	cdata->priv_data = (priv_data_t *) kmalloc(sizeof(priv_data_t),
						   GFP_KERNEL);
	if (!cdata->priv_data)
		return -ENOMEM;

	cdata->irq = client->irq;
	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->bus_type = BUS_I2C;
	cdata->tf = &lis2mdl_i2c_tf;
	cdata->wai_addr = LIS2MDL_WHO_AM_I_ADDR;
	cdata->wai_val = LIS2MDL_WHO_AM_I_VAL;

	mutex_init(&cdata->lock);

	i2c_set_clientdata(client, cdata);

	err = lis2mdl_probe(cdata);
	if (err < 0) {
		kfree(cdata);

		return err;
	}

	return 0;
}

int lis2mdl_i2c_remove(struct i2c_client *client)
{
	struct st_common_data *cdata = i2c_get_clientdata(client);

#ifdef LIS2MDL_DEBUG
	dev_info(cdata->dev, "driver removing\n");
#endif

	lis2mdl_remove(cdata);
	kfree(cdata);

	return 0;
}

static const struct i2c_device_id lis2mdl_i2c_id[] = {
	{ LIS2MDL_DEV_NAME, 0 },
	{ IIS2MDC_DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lis2mdl_i2c_id);

static struct i2c_driver lis2mdl_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS2MDL_DEV_NAME,
		.pm = LIS2MDL_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis2mdl_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = lis2mdl_i2c_probe,
	.remove = lis2mdl_i2c_remove,
	.id_table = lis2mdl_i2c_id,
};

module_i2c_driver(lis2mdl_i2c_driver);

MODULE_DESCRIPTION("lis2mdl i2c driver");
MODULE_AUTHOR("Armando Visconti");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

