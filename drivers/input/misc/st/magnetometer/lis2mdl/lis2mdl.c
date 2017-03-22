/*
 * STMicroelectronics lis2mdl.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Armando Visconti <armando.visconti@st.com>
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/version.h>
#include "lis2mdl.h"

static enum hrtimer_restart lis2mdl_poll_function(struct hrtimer *timer);
static int lis2mdl_input_init(struct lis2mdl_data *sdata, const char* description,
								bool is_3_axis);
static int lis2mdl_write_data(struct st_common_data *cdata, u8 reg_addr,
							u8 mask, u8 data);
static void lis2mdl_input_cleanup(struct lis2mdl_data *sdata);
static void lis2mdl_report_3axes_event(struct lis2mdl_data *sdata, s32 *xyz,
								s64 timestamp);

static ssize_t lis2mdl_get_enable(struct device *dev,
				    struct device_attribute *attr,
				    char *buf);
static ssize_t lis2mdl_set_enable(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);
static ssize_t lis2mdl_get_polling_rate(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t lis2mdl_set_polling_rate(struct device *dev,
				   struct device_attribute *attr, const char *buf,
				   size_t count);

/* DEVICE REGISTERS */
#define LIS2MDL_WHO_AM_I_ADDR		(0x4F)
#define LIS2MDL_WHO_AM_I_VAL		(0x40)
#define LIS2MDL_CFG_REG_A			(0x60)
#define LIS2MDL_CFG_REG_B			(0x61)
#define LIS2MDL_CFG_REG_B_OFF_CANC_MASK	(0x02)
#define LIS2MDL_CFG_REG_C			(0x62)
#define LIS2MDL_CFG_REG_C_BDU_MASK		(0x10)
#define LIS2MDL_OUT_L_ADDR			(0x68)

/* Device operating modes */
#define LIS2MDL_MD_CONTINUOS_MODE		0x00
#define LIS2MDL_MD_SINGLE_MODE		0x01
#define LIS2MDL_MD_IDLE_MODE		0x03
#define LIS2MDL_MODE_MASK			0x03

/* Device ODRs */
#define LIS2MDL_ODR_10HZ			0x00
#define LIS2MDL_ODR_20HZ			0x01
#define LIS2MDL_ODR_50HZ			0x02
#define LIS2MDL_ODR_100HZ			0x03
#define LIS2MDL_ODR_MASK			0x0C
#define LIS2MDL_BYTE_FOR_SAMPLE		6
#define LIS2MDL_ODR_LIST_NUM		4
#define LIS2MDL_DEV_DESCRIPTION		"ST LIS2MDL Magnetometer Sensor"
#define LIS2MDL_SENSOR_NAME			"magn"
#define LIS2MDL_SCALE			1500

struct lis2mdl_odr_reg {
	u32 hz;
	u8 value;
} lis2mdl_odr_table_t[] = {
	[0] = { .hz = 10, 	.value = LIS2MDL_ODR_10HZ },
	[1] = { .hz = 20, 	.value = LIS2MDL_ODR_20HZ },
	[2] = { .hz = 50, 	.value = LIS2MDL_ODR_50HZ },
	[3] = { .hz = 100, 	.value = LIS2MDL_ODR_100HZ },
};

static void lis2mdl_poll_wk(struct work_struct *input_work)
{
	struct lis2mdl_data *sdata;
	int xyz[3] = { 0 };
	u8 data[6];
	int err;

	sdata = container_of((struct work_struct *)input_work,
			     struct lis2mdl_data, input_work);

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

	err = sdata->cdata->tf->read(sdata->cdata, LIS2MDL_OUT_L_ADDR,
						LIS2MDL_BYTE_FOR_SAMPLE,
						data);
	if (err < 0)
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
			sdata->name, err);
	else {
		xyz[0] = (s32)((s16)(data[0] | data[1] << 8));
		xyz[1] = (s32)((s16)(data[2] | data[3] << 8));
		xyz[2] = (s32)((s16)(data[4] | data[5] << 8));

		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;
		lis2mdl_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
}

int lis2mdl_write_odr(struct lis2mdl_data *sdata) {
	int i;

	for (i = 0; i < LIS2MDL_ODR_LIST_NUM; i++) {
		if (lis2mdl_odr_table_t[i].hz >= sdata->c_odr)
				break;
	}
	if (i == LIS2MDL_ODR_LIST_NUM)
		return -EINVAL;

	return lis2mdl_write_data(sdata->cdata,
				LIS2MDL_CFG_REG_A,
				LIS2MDL_ODR_MASK,
				lis2mdl_odr_table_t[i].value);
}

int lis2mdl_enable_sensors(struct lis2mdl_data *sdata)
{
	int err;

	if (sdata->enabled)
		return 0;

	err = lis2mdl_write_data(sdata->cdata, LIS2MDL_CFG_REG_A,
						LIS2MDL_MODE_MASK,
						LIS2MDL_MD_CONTINUOS_MODE);
	if (err < 0)
		return err;

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
	sdata->enabled = true;

	return 0;
}

int lis2mdl_disable_sensor(struct lis2mdl_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	err = lis2mdl_write_data(sdata->cdata, LIS2MDL_CFG_REG_A,
						LIS2MDL_MODE_MASK,
						LIS2MDL_MD_IDLE_MODE);
	if (err < 0)
		return err;

	cancel_work_sync(&sdata->input_work);
	hrtimer_cancel(&sdata->hr_timer);
	sdata->enabled = false;

	return 0;
}

ssize_t lis2mdl_get_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "50\n");
}

int lis2mdl_init_sensor(struct st_common_data *cdata)
{
	int err;
	struct lis2mdl_data *sdata = cdata->sensors;

	INIT_WORK(&cdata->sensors->input_work, lis2mdl_poll_wk);
	hrtimer_init(&cdata->sensors->hr_timer, CLOCK_MONOTONIC,
					HRTIMER_MODE_REL);

	err = lis2mdl_disable_sensor(sdata);
	if (err < 0)
		return err;

	cdata->sensors->hr_timer.function = &lis2mdl_poll_function;

	/*
	* Enable offset cancellation
	*/
	err = lis2mdl_write_data(cdata, LIS2MDL_CFG_REG_B,
					LIS2MDL_CFG_REG_B_OFF_CANC_MASK,
					LIS2MDL_EN_BIT);
	if (err < 0)
		return err;


	/*
	 * Enable block data update feature.
	 */
	return lis2mdl_write_data(cdata, LIS2MDL_CFG_REG_C,
					LIS2MDL_CFG_REG_C_BDU_MASK,
					LIS2MDL_EN_BIT);
}

ADD_DEVICE_ENABLE_ATTR;
ADD_DEVICE_POLLING_ATTR;

static DEVICE_ATTR(scale_avail, S_IRUGO,
				lis2mdl_get_scale_avail,
				NULL);

static struct attribute *lis2mdl_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_scale_avail.attr,
	NULL,
};

static const struct attribute_group lis2mdl_attribute_groups = {
	.attrs = lis2mdl_attribute,
	.name = LIS2MDL_SENSOR_NAME,
};

int lis2mdl_data_init(struct st_common_data *cdata)
{
	struct lis2mdl_data *sdata = cdata->sensors;

	sdata->enabled = false;
	sdata->cdata = cdata;
	sdata->sindex = 0;
	sdata->c_gain = LIS2MDL_SCALE;
	sdata->name = LIS2MDL_SENSOR_NAME;
	sdata->c_odr = lis2mdl_odr_table_t[0].hz;
	sdata->ktime = ktime_set(0, HZ_TO_NSEC(sdata->c_odr));

	lis2mdl_input_init(sdata, LIS2MDL_DEV_DESCRIPTION, true);

	if (sysfs_create_group(&sdata->input_dev->dev.kobj,
					&lis2mdl_attribute_groups)) {
		dev_err(cdata->dev, "failed to create sysfs group for sensor %s",
					sdata->name);
		return -EINVAL;
	}

	sdata->write_odr = lis2mdl_write_odr;
	sdata->enable = lis2mdl_enable_sensors;
	sdata->disable = lis2mdl_disable_sensor;

	return 0;
}

static int lis2mdl_write_data(struct st_common_data *cdata, u8 reg_addr,
			 u8 mask, u8 data)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data);
}

static int lis2mdl_input_init(struct lis2mdl_data *sdata, const char* description,
							bool is_3_axis)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = description;
	sdata->input_dev->id.bustype = sdata->cdata->bus_type;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit );
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if (is_3_axis) {
		__set_bit(INPUT_EVENT_Y, sdata->input_dev->mscbit);
		__set_bit(INPUT_EVENT_Z, sdata->input_dev->mscbit);
	}

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev, "unable to register sensor %s\n",
								sdata->name);
		input_free_device(sdata->input_dev);
	}

	return err;
}

static void lis2mdl_input_cleanup(struct lis2mdl_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void lis2mdl_report_3axes_event(struct lis2mdl_data *sdata, s32 *xyz,
								s64 timestamp)
{
	struct input_dev *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
							timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
							timestamp & 0xffffffff);
	input_sync(input);
}

static enum hrtimer_restart lis2mdl_poll_function(struct hrtimer *timer)
{
	struct lis2mdl_data *sdata;

	sdata = container_of((struct hrtimer *)timer, struct lis2mdl_data,
							hr_timer);

	sdata->timestamp = lis2mdl_get_time_ns();
	queue_work(sdata->cdata->workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}

static ssize_t lis2mdl_get_enable(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct lis2mdl_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t lis2mdl_set_enable(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err;
	struct lis2mdl_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = sdata->enable(sdata);
	else
		err = sdata->disable(sdata);

	return (err < 0) ? err : count;
}

static ssize_t lis2mdl_get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lis2mdl_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", 1000 / sdata->c_odr);
}

static ssize_t lis2mdl_set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	u32 c_odr_tmp;
	ktime_t ktime_tmp;
	unsigned int polling_rate;
	struct lis2mdl_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	c_odr_tmp = sdata->c_odr;
	ktime_tmp = sdata->ktime;
	sdata->c_odr = 1000 / polling_rate;
	sdata->ktime = ktime_set(0, MS_TO_NS(polling_rate));
	mutex_unlock(&sdata->input_dev->mutex);

	err = sdata->write_odr(sdata);
	if (err < 0) {
		sdata->c_odr = c_odr_tmp;
		sdata->ktime = ktime_tmp;
		return err;
	}

	return count;
}

int lis2mdl_probe(struct st_common_data *cdata)
{
	int err;
	u8 wai = 0;

	err = cdata->tf->read(cdata, cdata->wai_addr, 1, &wai);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register 0x%x.\n",
								cdata->wai_addr);
		return err;
	}
	if (wai != cdata->wai_val) {
		dev_err(cdata->dev, "Who-Am-I value not valid. (expected 0x%x, \
			readed 0x%x))\n", cdata->wai_val, wai);
		return -ENODEV;
	}

	cdata->workqueue = create_workqueue(cdata->name);

	err = lis2mdl_data_init(cdata);
	if (err < 0)
		return err;

	err = lis2mdl_init_sensor(cdata);
	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(lis2mdl_probe);

int lis2mdl_remove(struct st_common_data *cdata)
{
	lis2mdl_disable_sensor(cdata->sensors);
	lis2mdl_input_cleanup(cdata->sensors);

	return 0;
}
EXPORT_SYMBOL(lis2mdl_remove);

int lis2mdl_enable(struct st_common_data *cdata)
{
	return lis2mdl_enable_sensors(cdata->sensors);
}
EXPORT_SYMBOL(lis2mdl_enable);

int lis2mdl_disable(struct st_common_data *cdata)
{
	return lis2mdl_disable_sensor(cdata->sensors);
}
EXPORT_SYMBOL(lis2mdl_disable);

MODULE_DESCRIPTION("STMicroelectronics lis2mdl i2c driver");
MODULE_AUTHOR("Armando Visconti");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_LICENSE("GPL v2");

