/*
 * STMicroelectronics ism330dlc i2c driver
 *
 * Copyright 2018 STMicroelectronics Inc.
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

#include "ism330dlc_core.h"

static int ism330dlc_i2c_read(struct ism330dlc_data *cdata, u8 reg_addr, int len,
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

static int ism330dlc_i2c_write(struct ism330dlc_data *cdata, u8 reg_addr, int len,
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


static const struct ism330dlc_transfer_function ism330dlc_tf_i2c = {
	.write = ism330dlc_i2c_write,
	.read = ism330dlc_i2c_read,
};

static int ism330dlc_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct ism330dlc_data *cdata;

	cdata = kzalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &ism330dlc_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = ism330dlc_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int ism330dlc_i2c_remove(struct i2c_client *client)
{
	struct ism330dlc_data *cdata = i2c_get_clientdata(client);

	ism330dlc_common_remove(cdata, client->irq);
	dev_info(cdata->dev, "%s: removed\n", ISM330DLC_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int ism330dlc_suspend(struct device *dev)
{
	struct ism330dlc_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return ism330dlc_common_suspend(cdata);
}

static int ism330dlc_resume(struct device *dev)
{
	struct ism330dlc_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return ism330dlc_common_resume(cdata);
}

static const struct dev_pm_ops ism330dlc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ism330dlc_suspend, ism330dlc_resume)
};

#define ISM330DLC_PM_OPS		(&ism330dlc_pm_ops)
#else /* CONFIG_PM */
#define ISM330DLC_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id ism330dlc_ids[] = {
	{ ISM330DLC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ism330dlc_ids);

#ifdef CONFIG_OF
static const struct of_device_id ism330dlc_id_table[] = {
	{ .compatible = "st,ism330dlc", },
	{ },
};
MODULE_DEVICE_TABLE(of, ism330dlc_id_table);
#endif

static struct i2c_driver ism330dlc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ISM330DLC_DEV_NAME,
		.pm = ISM330DLC_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = ism330dlc_id_table,
#endif
	},
	.probe    = ism330dlc_i2c_probe,
	.remove   = ism330dlc_i2c_remove,
	.id_table = ism330dlc_ids,
};

module_i2c_driver(ism330dlc_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics ism330dlc i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
