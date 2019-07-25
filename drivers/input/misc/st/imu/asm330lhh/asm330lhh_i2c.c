/*
 * STMicroelectronics asm330lhh i2c driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 * v 1.0
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "asm330lhh_core.h"

static int asm330lhh_i2c_read(struct asm330lhh_data *cdata, u8 reg_addr, int len,
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

static int asm330lhh_i2c_write(struct asm330lhh_data *cdata, u8 reg_addr, int len,
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


static const struct asm330lhh_transfer_function asm330lhh_tf_i2c = {
	.write = asm330lhh_i2c_write,
	.read = asm330lhh_i2c_read,
};

static int asm330lhh_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	struct asm330lhh_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->tf = &asm330lhh_tf_i2c;
	i2c_set_clientdata(client, cdata);

	err = asm330lhh_common_probe(cdata, client->irq, BUS_I2C);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int asm330lhh_i2c_remove(struct i2c_client *client)
{
	struct asm330lhh_data *cdata = i2c_get_clientdata(client);

	asm330lhh_common_remove(cdata, client->irq);
	dev_info(cdata->dev, "%s: removed\n", ASM330LHH_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int asm330lhh_suspend(struct device *dev)
{
	struct asm330lhh_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return asm330lhh_common_suspend(cdata);
}

static int asm330lhh_resume(struct device *dev)
{
	struct asm330lhh_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return asm330lhh_common_resume(cdata);
}

static const struct dev_pm_ops asm330lhh_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(asm330lhh_suspend, asm330lhh_resume)
};

#define ASM330LHH_PM_OPS		(&asm330lhh_pm_ops)
#else /* CONFIG_PM */
#define ASM330LHH_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id asm330lhh_ids[] = {
	{ ASM330LHH_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asm330lhh_ids);

#ifdef CONFIG_OF
static const struct of_device_id asm330lhh_id_table[] = {
	{ .compatible = "st,asm330lhh", },
	{ },
};
MODULE_DEVICE_TABLE(of, asm330lhh_id_table);
#endif

static struct i2c_driver asm330lhh_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ASM330LHH_DEV_NAME,
		.pm = ASM330LHH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = asm330lhh_id_table,
#endif
	},
	.probe    = asm330lhh_i2c_probe,
	.remove   = asm330lhh_i2c_remove,
	.id_table = asm330lhh_ids,
};

module_i2c_driver(asm330lhh_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics asm330lhh i2c driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
