/*
 * STMicroelectronics stts751_core.c driver
 *
 * Copyright 2020 STMicroelectronics Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/i2c.h>

#include "stts751_core.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#define	kstrtoul(x, y, z) strict_strtoul(x, y, z)
#endif

static const struct {
	u32 cutoff_period_ms;
	u8 odr;
} stts751_odr_table[] = {
	/* msec, u8 */
	{ 32, 9 },
	{ 63, 8 },
	{ 125, 7 },
	{ 250, 6 },
	{ 500, 5 },
	{ 1000, 4 },
	{ 2000, 3 },
	{ 4000, 2 },
	{ 8000, 1 },
	{ 16000, 0 },

};

static inline s64 stts751_get_time_ns(void)
{
	struct timespec ts;

	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

static int stts751_hw_init(struct stts751_dev *dev)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev->dev);

	data = i2c_smbus_read_byte_data(client, REG_PROD_ID_ADDR);
	if (data < 0)
		return data;

	if ((data != PROD_ID0_VALUE) && (data != PROD_ID1_VALUE)) {
		dev_err(dev->dev, "product id unknown [0x%02x,0x%02x] != %d\n",
			PROD_ID0_VALUE, PROD_ID1_VALUE, data);
		return -ENODEV;
	}

	return 0;
}

static int stts751_update_odr(struct stts751_dev *dev, u32 poll_interval)
{
	int i, err;
	u8 data = DISABLE_SENSOR;
	struct i2c_client *client = to_i2c_client(dev->dev);

	err = i2c_smbus_write_byte_data(client, REG_CONV_RATE_ADDR, data);
	if (err < 0)
		return err;

	for (i = ARRAY_SIZE(stts751_odr_table) - 1; i > 0; i--) {
		if (stts751_odr_table[i].cutoff_period_ms <= poll_interval)
			break;
	}

	dev->odr = stts751_odr_table[i].odr;
	data = ENABLE_SENSOR | dev->odr;
	err = i2c_smbus_write_byte_data(client, REG_CONV_RATE_ADDR, data);
	if (err < 0)
		return err;

	return 0;
}

static int stts751_device_power_on(struct stts751_dev *dev)
{
	u8 data;
	int err;
	struct i2c_client *client = to_i2c_client(dev->dev);

	data = dev->res;
	err = i2c_smbus_write_byte_data(client, REG_CONV_RATE_ADDR, data);
	if (err < 0)
		return err;

	data = DISABLE_SENSOR;
	err = i2c_smbus_write_byte_data(client, REG_CONV_RATE_ADDR, data);
	if (err < 0)
		return err;
	msleep(50);

	data = ENABLE_SENSOR | dev->odr;
	err = i2c_smbus_write_byte_data(client, REG_CONV_RATE_ADDR, data);
	if (err < 0)
		return err;

	dev->enabled = true;

	return 0;
}

static int stts751_device_power_off(struct stts751_dev *dev)
{
	int err;
	u8 data, val = DISABLE_SENSOR;
	struct i2c_client *client = to_i2c_client(dev->dev);

	data = val;
	err = i2c_smbus_write_byte_data(client, REG_CONV_RATE_ADDR, data);
	if (err < 0)
		return err;

	dev->enabled = false;

	return 0;
}

static int stts751_input_init(struct stts751_dev *dev)
{
	int err;

	dev->input_dev = input_allocate_device();
	if (!dev->input_dev)
		return -ENOMEM;

	dev->input_dev->name = "stts751";
	dev->input_dev->id.bustype = BUS_I2C;
	dev->input_dev->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev->evbit);
	set_bit(INPUT_EVENT_TIME_MSB, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_X, dev->input_dev->mscbit);

	err = input_register_device(dev->input_dev);
	if (err) {
		dev_err(dev->dev, "unable to register input device %s\n",
			dev->input_dev->name);
		input_free_device(dev->input_dev);

		return err;
	}

	return 0;
}

static int stts751_enable(struct stts751_dev *dev)
{
	int err = 0;

	mutex_lock(&dev->lock);
	if (!dev->enabled) {
		err = stts751_device_power_on(dev);
		if (dev->enabled) {
			schedule_delayed_work(&dev->input_work,
			      msecs_to_jiffies(dev->poll_interval));
		} else {
			dev_err(dev->dev, "device %s : enable failed\n",
			dev->input_dev->name);
		}

	}
	mutex_unlock(&dev->lock);

	return err;
}

static int stts751_disable(struct stts751_dev *dev)
{
	int err = 0;

	cancel_delayed_work_sync(&dev->input_work);

	mutex_lock(&dev->lock);
	if (dev->enabled)
		err = stts751_device_power_off(dev);
	mutex_unlock(&dev->lock);

	return err;
}

static void stts751_input_cleanup(struct stts751_dev *dev)
{
	input_unregister_device(dev->input_dev);
}

static ssize_t attr_get_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	u32 val;
	struct stts751_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	u32 val;
	struct stts751_dev *dev = dev_get_drvdata(device);

	if (kstrtou32(buf, 10, &val) || !val)
		return -EINVAL;

	mutex_lock(&dev->lock);
	dev->poll_interval = val;
	stts751_update_odr(dev, dev->poll_interval);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_res(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	u8 val;
	struct stts751_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->res;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%u\n", val);
}

static void stts751_get_and_report_data(struct stts751_dev *dev);

static ssize_t attr_set_oneshot(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val;
	int err = 0;
	bool enabled = true;
	u8 random_data = 0xaf;
	struct stts751_dev *dev = dev_get_drvdata(device);
	struct i2c_client *client = to_i2c_client(dev->dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (!val)
		return -EINVAL;

	mutex_lock(&dev->lock);
	if (!dev->enabled) {
		enabled = false;
		err = i2c_smbus_write_byte_data(client, REG_ONE_SHOT_ADDR, random_data);
		stts751_get_and_report_data(dev);
	}
	mutex_unlock(&dev->lock);

	if (enabled)
		return -EBUSY;
	if (err < 0)
		return err;
	else
		return size;
}

static ssize_t attr_get_enable(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	bool enabled;
	struct stts751_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	enabled = dev->enabled;
	mutex_unlock(&dev->lock);

	return  sprintf(buf, "%d\n", enabled);
}

static ssize_t attr_set_enable(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	unsigned long val;
	struct stts751_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		stts751_enable(dev);
	else
		stts751_disable(dev);

	return size;
}

static struct device_attribute stts751_sysfs_attributes[] = {
	__ATTR(poll_ms, 0644, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(t_res, 0444, attr_get_res, NULL),
	__ATTR(oneshot, 0200, NULL, attr_set_oneshot),
	__ATTR(enable_device, 0644, attr_get_enable, attr_set_enable),
};

static int stts751_sysfs_init(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stts751_sysfs_attributes); i++) {
		if (device_create_file(dev, stts751_sysfs_attributes + i))
			goto err;
	}

	return 0;

err:
	for (; i >= 0; i--)
		device_remove_file(dev, stts751_sysfs_attributes + i);

	return -1;
}

static void stts751_sysfs_remove(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stts751_sysfs_attributes); i++)
		device_remove_file(dev, stts751_sysfs_attributes + i);
}

static int stts751_get_data(struct stts751_dev *dev, int *data_t)
{
	int val;
	u8 data[2];
	struct i2c_client *client = to_i2c_client(dev->dev);

	val = i2c_smbus_read_byte_data(client, REG_VAL_HIGH_ADDR);
	if (val < 0)
		return val;

	data[1] = val;

	val = i2c_smbus_read_byte_data(client, REG_VAL_LOW_ADDR);
	if (val < 0)
		return val;

	data[0] = val;
	*data_t = (s16)((data[1] << 8) | data[0]);

#ifdef STTS751_DEBUG
	dev_info(dev->dev, "temperature %d\n", *data_t);
#endif

	return 0;
}

static void stts751_report_data(struct stts751_dev *dev, int data,
				s64 timestamp)
{
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(dev->input_dev);
}

static void stts751_get_and_report_data(struct stts751_dev *dev)
{
	int err, data;

	mutex_lock(&dev->lock);

	err = stts751_get_data(dev, &data);
	if (err < 0)
		dev_err(dev->dev, "get data failed\n");
	else
		stts751_report_data(dev, data, stts751_get_time_ns());

	mutex_unlock(&dev->lock);
}

static void stts751_input_work_fn(struct work_struct *work)
{
	struct stts751_dev *dev;

	dev = container_of((struct delayed_work *)work,
			   struct stts751_dev, input_work);

	stts751_get_and_report_data(dev);

	if (dev->enabled) {
		schedule_delayed_work(&dev->input_work,
			msecs_to_jiffies(dev->poll_interval));
	}
}

#ifdef CONFIG_PM
static int stts751_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct stts751_dev *dev = i2c_get_clientdata(client);

	return stts751_enable(dev);
}

static int stts751_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct stts751_dev *dev = i2c_get_clientdata(client);

	return stts751_disable(dev);
}

static const struct dev_pm_ops stts751_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stts751_suspend, stts751_resume)
};
#endif /* CONFIG_PM */

static int stts751_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err;
	struct stts751_dev *dev;

	dev = kzalloc(sizeof(struct stts751_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->name = client->name;
	dev->dev = &client->dev;

	i2c_set_clientdata(client, dev);

	mutex_init(&dev->lock);
	mutex_lock(&dev->lock);

	dev->poll_interval = STTS751_DEFAULT_POLL_PERIOD_MS;

	err =  stts751_hw_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", err);
		goto unlock;
	}

	err = stts751_update_odr(dev, dev->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "set odr failed: %d\n", err);
		goto unlock;
	}

	err = stts751_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto unlock;
	}

	err = stts751_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "input init failed: %d\n", err);
		goto power_off;
	}

	err = stts751_sysfs_init(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "sysfs register failed\n");
		goto input_cleanup;
	}

	err = stts751_device_power_off(dev);
	if (err < 0) {
		dev_err(dev->dev, "power off failed: %d\n", err);
		goto sysfs_remove;
	}

	INIT_DELAYED_WORK(&dev->input_work, stts751_input_work_fn);

	mutex_unlock(&dev->lock);

	return 0;

sysfs_remove:
	stts751_sysfs_remove(dev->dev);

input_cleanup:
	stts751_input_cleanup(dev);

power_off:
	stts751_device_power_off(dev);

unlock:
	mutex_unlock(&dev->lock);

	kfree(dev);

	return err;
}

static int stts751_remove(struct i2c_client *client)
{
	struct stts751_dev *dev = i2c_get_clientdata(client);

	dev_info(dev->dev, "removing device %s\n", dev->input_dev->name);

	cancel_delayed_work_sync(&dev->input_work);
	stts751_device_power_off(dev);
	stts751_input_cleanup(dev);
	stts751_sysfs_remove(dev->dev);
	kfree(dev);

	return 0;
}

static const struct i2c_device_id stts751_id[] = {
	{ "stts751", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, stts751_id);

#ifdef CONFIG_OF
static const struct of_device_id stts751_id_table[] = {
	{ .compatible = "st,stts751" },
	{ },
};
MODULE_DEVICE_TABLE(of, stts751_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver stts751_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "stts751",
#ifdef CONFIG_PM
		.pm = &stts751_pm_ops,
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
		.of_match_table = stts751_id_table,
#endif /* CONFIG_OF */
	},
	.probe = stts751_probe,
	.remove = stts751_remove,
	.id_table = stts751_id,
};

module_i2c_driver(stts751_driver);

MODULE_DESCRIPTION("stts751 driver");
MODULE_AUTHOR("Mario Tesi <mario.tesi@st.com>");
MODULE_AUTHOR("Matteo Dameno <matteo.dameno@st.com>");
MODULE_LICENSE("GPL v2");
