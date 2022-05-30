/*
 * STMicroelectronics stts22h_core.c driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Authors: AMG MSD DIVISION
 *        : Mario Tesi (mario.tesi@st.com)
 *
 * Licensed under the GPL-2.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "stts22h_core.h"

static const struct {
	u16 poll_ms;
	u8 reg_val;
	bool low_odr;
} stts22h_odr_table[] = {
	/* poll rate ms, reg, low odr */
	{   ODR_TO_MS(1), 0,  true },
	{  ODR_TO_MS(25), 0, false },
	{  ODR_TO_MS(50), 1, false },
	{ ODR_TO_MS(100), 2, false },
	{ ODR_TO_MS(200), 3, false },
};

static inline s64 stts22h_get_time_ns(void)
{
	struct timespec ts;

	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

static int stts22h_check_whoami(struct stts22h_sensor *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int data;

	data = i2c_smbus_read_byte_data(client, STTS22H_WHOAMI_ADDR);
	if (data < 0)
		return data;

	if (data != STTS22H_WHOAMI_VAL) {
		dev_err(sensor->dev,
			"unsupported whoami [%02x]\n",
			data);

		return -EINVAL;
	}

	return 0;
}

static int stts22h_write_data_with_mask(struct stts22h_sensor *sensor,
					u8 reg, u8 data, u8 mask)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int tmp_data;

	tmp_data = i2c_smbus_read_byte_data(client, reg);
	if (tmp_data < 0)
		return tmp_data;

	tmp_data = (tmp_data & ~mask) | ((data << __ffs(mask)) & mask);

	return i2c_smbus_write_byte_data(client, reg, tmp_data);
}

static int stts22h_get_data(struct stts22h_sensor *sensor, int *data)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	__le16 temp;
	int ret;

	ret = i2c_smbus_read_i2c_block_data_or_emulated(client,
				STTS22H_TEMP_L_OUT_ADDR, sizeof(temp),
				(u8 *)&temp);
	if (ret < 0) {
		dev_err(&client->dev,
			"error reading temperature registers\n");
		return ret;
	}

	*data = (s16)le16_to_cpu(temp);

	return 0;
}

static void stts22h_report_data(struct stts22h_sensor *sensor, int data,
				s64 timestamp)
{
	input_event(sensor->input_dev,
		    INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(sensor->input_dev,
		    INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(sensor->input_dev,
		    INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(sensor->input_dev);
}

static void stts22h_get_and_report_data(struct stts22h_sensor *sensor)
{
	int err, data = 0;

	mutex_lock(&sensor->lock);

	err = stts22h_get_data(sensor, &data);
	if (err < 0)
		dev_err(sensor->dev, "get data failed\n");
	else
		stts22h_report_data(sensor, data,
				    stts22h_get_time_ns());

	mutex_unlock(&sensor->lock);
}

static int stts22h_update_odr(struct stts22h_sensor *sensor,
			      u16 poll_interval)
{
	int i;

	if (sensor->enabled)
		return -EBUSY;

	for (i = 0; i < ARRAY_SIZE(stts22h_odr_table); i++) {
		if (poll_interval >= stts22h_odr_table[i].poll_ms)
			break;
	}

	/* set to max odr when user requires odr > 200Hz */
	if (i == ARRAY_SIZE(stts22h_odr_table))
		i--;

	sensor->poll_index = i;
	sensor->poll_ms = stts22h_odr_table[i].poll_ms;
	sensor->low_odr = stts22h_odr_table[i].low_odr;

	return 0;
}

static int stts22h_device_power_on(struct stts22h_sensor *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	u8 lodr = 0, ctrl_mask = STTS22H_FREERUN_MASK;
	int err;

	if (sensor->low_odr) {
		lodr = STTS22H_LOW_ODR_ENABLE_MASK;
		ctrl_mask = STTS22H_LOW_ODR_START_MASK;
	}

	err = i2c_smbus_write_byte_data(client,
					STTS22H_SOFTWARE_RESET_ADDR,
					lodr | STTS22H_SW_RESET_MASK);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client,
					STTS22H_SOFTWARE_RESET_ADDR,
					lodr);
	if (err < 0)
		return err;

	err = stts22h_write_data_with_mask(sensor, STTS22H_CTRL_ADDR,
					   1, ctrl_mask);
	if (err < 0)
		return err;

	err = stts22h_write_data_with_mask(sensor, STTS22H_CTRL_ADDR,
					   1, STTS22H_IF_ADD_INC_MASK);
	if (err < 0)
		return err;

	err = stts22h_write_data_with_mask(sensor, STTS22H_CTRL_ADDR,
			stts22h_odr_table[sensor->poll_index].reg_val,
			STTS22H_AVG_MASK);
	if (err < 0)
		return err;

	sensor->enabled = true;

	return 0;
}

static int stts22h_device_power_off(struct stts22h_sensor *sensor)
{
	u8 ctrl_mask = STTS22H_FREERUN_MASK;
	int err;

	if (sensor->low_odr)
		ctrl_mask = STTS22H_LOW_ODR_START_MASK;

	err = stts22h_write_data_with_mask(sensor, STTS22H_CTRL_ADDR,
					   0, ctrl_mask);
	if (err < 0)
		return err;

	err = stts22h_write_data_with_mask(sensor, STTS22H_CTRL_ADDR,
					   0, STTS22H_AVG_MASK);
	if (err < 0)
		return err;

	sensor->enabled = false;

	return 0;
}

static int stts22h_input_init(struct stts22h_sensor *sensor)
{
	int err;

	sensor->input_dev = input_allocate_device();
	if (!sensor->input_dev)
		return -ENOMEM;

	sensor->input_dev->name = STTS22H_DEVICE_NAME;
	sensor->input_dev->id.bustype = BUS_I2C;
	sensor->input_dev->dev.parent = sensor->dev;

	input_set_drvdata(sensor->input_dev, sensor);

	set_bit(INPUT_EVENT_TYPE, sensor->input_dev->evbit);
	set_bit(INPUT_EVENT_TIME_MSB, sensor->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, sensor->input_dev->mscbit);
	set_bit(INPUT_EVENT_X, sensor->input_dev->mscbit);

	err = input_register_device(sensor->input_dev);
	if (err) {
		dev_err(sensor->dev,
			"unable to register input device %s\n",
			sensor->input_dev->name);
		input_free_device(sensor->input_dev);

		return err;
	}

	return 0;
}

static int stts22h_enable(struct stts22h_sensor *sensor)
{
	int err = 0;

	mutex_lock(&sensor->lock);
	if (!sensor->enabled) {
		err = stts22h_device_power_on(sensor);
		if (err) {
			dev_err(sensor->dev,
				"device %s : enable failed\n",
				sensor->input_dev->name);
		} else {
			schedule_delayed_work(&sensor->input_work,
			      msecs_to_jiffies(sensor->poll_ms));
		}
	}
	mutex_unlock(&sensor->lock);

	return err;
}

static int stts22h_disable(struct stts22h_sensor *sensor)
{
	int err = 0;

	cancel_delayed_work_sync(&sensor->input_work);

	mutex_lock(&sensor->lock);
	if (sensor->enabled)
		err = stts22h_device_power_off(sensor);
	mutex_unlock(&sensor->lock);

	return err;
}

static void stts22h_input_cleanup(struct stts22h_sensor *sensor)
{
	input_unregister_device(sensor->input_dev);
}

static ssize_t attr_get_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct stts22h_sensor *sensor = dev_get_drvdata(device);
	u32 val;

	mutex_lock(&sensor->lock);
	val = sensor->poll_ms;
	mutex_unlock(&sensor->lock);

	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct stts22h_sensor *sensor = dev_get_drvdata(device);
	u32 val;

	if (kstrtou32(buf, 10, &val) || !val)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	stts22h_update_odr(sensor, val);
	mutex_unlock(&sensor->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	struct stts22h_sensor *sensor = dev_get_drvdata(device);
	bool enabled;

	mutex_lock(&sensor->lock);
	enabled = sensor->enabled;
	mutex_unlock(&sensor->lock);

	return  sprintf(buf, "%d\n", enabled);
}

static ssize_t attr_set_enable(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct stts22h_sensor *sensor = dev_get_drvdata(device);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		stts22h_enable(sensor);
	else
		stts22h_disable(sensor);

	return size;
}

static struct device_attribute stts22h_sysfs_attributes[] = {
	__ATTR(poll_ms, 0644, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(enable_device, 0644, attr_get_enable, attr_set_enable),
};

static int stts22h_sysfs_init(struct device *dev)
{
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(stts22h_sysfs_attributes); i++) {
		err = device_create_file(dev,
					 stts22h_sysfs_attributes + i);
		if (err)
			goto remove_attr;
	}

	return 0;

remove_attr:
	for (; i >= 0; i--)
		device_remove_file(dev, stts22h_sysfs_attributes + i);

	return err;
}

static void stts22h_sysfs_remove(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stts22h_sysfs_attributes); i++)
		device_remove_file(dev, stts22h_sysfs_attributes + i);
}

static void stts22h_input_work_fn(struct work_struct *work)
{
	struct stts22h_sensor *sensor;

	sensor = container_of((struct delayed_work *)work,
			      struct stts22h_sensor, input_work);

	stts22h_get_and_report_data(sensor);

	mutex_lock(&sensor->lock);
	if (sensor->enabled)
		schedule_delayed_work(&sensor->input_work,
				msecs_to_jiffies(sensor->poll_ms));
	mutex_unlock(&sensor->lock);
}

#ifdef CONFIG_PM_SLEEP
static int stts22h_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct stts22h_sensor *sensor = i2c_get_clientdata(client);

	return stts22h_enable(sensor);
}

static int stts22h_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct stts22h_sensor *sensor = i2c_get_clientdata(client);

	return stts22h_disable(sensor);
}

static SIMPLE_DEV_PM_OPS(stts22h_pm_ops, stts22h_suspend,
			 stts22h_resume);

#define STTS22H_PM_OPS	(&stts22h_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define STTS22H_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static int stts22h_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct stts22h_sensor *sensor;
	int err;

	sensor = devm_kzalloc(dev, sizeof(struct stts22h_sensor),
			      GFP_KERNEL);
	if (!sensor) {
		dev_err(dev, "failed to allocate sensor data\n");

		return -ENOMEM;
	}

	mutex_init(&sensor->lock);
	sensor->name = client->name;
	sensor->dev = dev;

	i2c_set_clientdata(client, sensor);

	err =  stts22h_check_whoami(sensor);
	if (err < 0)
		return err;

	err = stts22h_input_init(sensor);
	if (err < 0) {
		dev_err(sensor->dev, "input init failed: %d\n", err);

		return err;
	}

	err = stts22h_sysfs_init(sensor->dev);
	if (err < 0) {
		dev_err(sensor->dev, "sysfs register failed\n");

		goto input_cleanup;
	}

	err = stts22h_update_odr(sensor, ODR_TO_MS(25));
	if (err < 0) {
		dev_err(sensor->dev, "set odr failed: %d\n", err);

		goto sysfs_remove;
	}

	INIT_DELAYED_WORK(&sensor->input_work, stts22h_input_work_fn);

	return 0;

sysfs_remove:
	stts22h_sysfs_remove(sensor->dev);

input_cleanup:
	stts22h_input_cleanup(sensor);

	return err;
}

static int stts22h_remove(struct i2c_client *client)
{
	struct stts22h_sensor *sensor = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&sensor->input_work);
	stts22h_device_power_off(sensor);
	stts22h_input_cleanup(sensor);
	stts22h_sysfs_remove(sensor->dev);

	dev_info(sensor->dev,
		 "removed device %s\n",
		 sensor->input_dev->name);

	return 0;
}

static const struct i2c_device_id stts22h_id[] = {
	{ STTS22H_DEVICE_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, stts22h_id);

#ifdef CONFIG_OF
static const struct of_device_id stts22h_id_table[] = {
	{ .compatible = "st," STTS22H_DEVICE_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, stts22h_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver stts22h_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = STTS22H_DEVICE_NAME,
		.pm = STTS22H_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = stts22h_id_table,
#endif /* CONFIG_OF */
	},
	.probe = stts22h_probe,
	.remove = stts22h_remove,
	.id_table = stts22h_id,
};

module_i2c_driver(stts22h_driver);

MODULE_DESCRIPTION(STTS22H_DEVICE_NAME " driver");
MODULE_AUTHOR("Mario Tesi <mario.tesi@st.com>");
MODULE_LICENSE("GPL v2");
