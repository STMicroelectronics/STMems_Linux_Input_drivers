/*
 * STMicroelectronics asm330lxh i2c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
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

#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include "asm330lxh.h"

static int asm330lxh_i2c_read(struct asm330lxh_status *cdata, u8 reg_addr, int len,
			      u8 *data)
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

	err = i2c_transfer(client->adapter, msg, 2);

	return (err == 2) ? 0 : 1;
}

static int asm330lxh_i2c_write(struct asm330lxh_status *cdata, u8 reg_addr, int len,
			       u8 *data)
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

	err = i2c_transfer(client->adapter, &msg, 1);

	return (err == 1) ? 0 : 1;
}

static const struct asm330lxh_transfer_function asm330lxh_tf_i2c = {
	.write = asm330lxh_i2c_write,
	.read = asm330lxh_i2c_read,
};

static int asm330lxh_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int err;
	struct asm330lxh_status *cdata;

	cdata = kzalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->irq = client->irq;
	cdata->tf = &asm330lxh_tf_i2c;
	cdata->bustype = BUS_I2C;
	i2c_set_clientdata(client, cdata);

	err = asm330lxh_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int asm330lxh_i2c_remove(struct i2c_client *client)
{
	struct asm330lxh_status *cdata = i2c_get_clientdata(client);

	asm330lxh_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", ASM330LXH_ACC_GYR_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int asm330lxh_suspend(struct device *dev)
{
	struct asm330lxh_status *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return asm330lxh_common_suspend(cdata);
}

static int asm330lxh_resume(struct device *dev)
{
	struct asm330lxh_status *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return asm330lxh_common_resume(cdata);
}

static const struct dev_pm_ops asm330lxh_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(asm330lxh_suspend, asm330lxh_resume)
};

#define ASM330LXH_PM_OPS	(&asm330lxh_pm_ops)
#else /* CONFIG_PM */
#define ASM330LXH_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id asm330lxh_ids[] = {
	{ ASM330LXH_ACC_GYR_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asm330lxh_ids);

#ifdef CONFIG_OF
static const struct of_device_id asm330lxh_id_table[] = {
	{ .compatible = "st,asm330lxh", },
	{ },
};
MODULE_DEVICE_TABLE(of, asm330lxh_id_table);
#endif

static struct i2c_driver asm330lxh_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ASM330LXH_ACC_GYR_DEV_NAME,
		.pm = ASM330LXH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(asm330lxh_id_table),
#endif
	},
	.probe    = asm330lxh_i2c_probe,
	.remove   = asm330lxh_i2c_remove,
	.id_table = asm330lxh_ids,
};

module_i2c_driver(asm330lxh_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics asm330lxh i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
