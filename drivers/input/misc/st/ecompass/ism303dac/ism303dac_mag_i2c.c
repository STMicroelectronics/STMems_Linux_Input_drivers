/*
 * STMicroelectronics ism303dac_mag_i2c.c driver
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include "ism303dac_core.h"

#define ISM303DAC_MAG_WHO_AM_I_ADDR		(0x4F)
#define ISM303DAC_MAG_WHO_AM_I_VAL		(0x40)

/* XXX: caller must hold cdata->lock */
static int ism303dac_mag_i2c_read(struct st_common_data *cdata,
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
static int ism303dac_mag_i2c_write(struct st_common_data *cdata,
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

static struct st_sensor_transfer_function ism303dac_mag_i2c_tf = {
	.write = ism303dac_mag_i2c_write,
	.read = ism303dac_mag_i2c_read,
};

#ifdef CONFIG_PM
static int ism303dac_mag_i2c_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st_common_data *cdata = i2c_get_clientdata(client);

	return ism303dac_mag_enable(cdata);
}

static int ism303dac_mag_i2c_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st_common_data *cdata = i2c_get_clientdata(client);

	return ism303dac_mag_disable(cdata);
}

static const struct dev_pm_ops ism303dac_mag_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ism303dac_mag_i2c_suspend, ism303dac_mag_i2c_resume)
};
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id ism303dac_mag_i2c_id_table[] = {
	{ .compatible = "st,ism303dac_mag", },
	{ },
};
MODULE_DEVICE_TABLE(of, ism303dac_mag_i2c_id_table);
#endif /* CONFIG_OF */

static int ism303dac_mag_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct st_common_data *cdata;

#ifdef ISM303DAC_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	cdata = kzalloc(sizeof(struct st_common_data), GFP_KERNEL);
	if (!cdata) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	cdata->sensors = (struct st_sensor_data *) kmalloc(
						sizeof(struct st_sensor_data),
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
	cdata->tf = &ism303dac_mag_i2c_tf;
	cdata->wai_addr = ISM303DAC_MAG_WHO_AM_I_ADDR;
	cdata->wai_val = ISM303DAC_MAG_WHO_AM_I_VAL;

	mutex_init(&cdata->lock);

	i2c_set_clientdata(client, cdata);

	err = ism303dac_mag_probe(cdata);
	if (err < 0) {
		kfree(cdata);

		return err;
	}

	return 0;
}

int ism303dac_mag_i2c_remove(struct i2c_client *client)
{
	struct st_common_data *cdata = i2c_get_clientdata(client);

#ifdef ISM303DAC_DEBUG
	dev_info(cdata->dev, "driver removing\n");
#endif

	ism303dac_mag_remove(cdata);
	kfree(cdata);

	return 0;
}

static const struct i2c_device_id ism303dac_mag_i2c_id[] = {
	{ "ism303dac_mag", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ism303dac_mag_i2c_id);

static struct i2c_driver ism303dac_mag_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ism303dac_mag_i2c",
#ifdef CONFIG_PM
		.pm = &ism303dac_mag_i2c_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = ism303dac_mag_i2c_id_table,
#endif /* CONFIG_OF */
	},
	.probe = ism303dac_mag_i2c_probe,
	.remove = ism303dac_mag_i2c_remove,
	.id_table = ism303dac_mag_i2c_id,
};

module_i2c_driver(ism303dac_mag_i2c_driver);

MODULE_DESCRIPTION("ism303dac_mag i2c driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

