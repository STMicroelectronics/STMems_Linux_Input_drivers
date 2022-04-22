/*
 * STMicroelectronics lsm303ah_core.c driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "lsm303ah_core.h"

#include <linux/workqueue.h>
#include <linux/hrtimer.h>

int st_sensor_write_data(struct st_common_data *cdata, u8 reg_addr,
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
EXPORT_SYMBOL(st_sensor_write_data);

int st_sensor_input_init(struct st_sensor_data *sdata, const char* description,
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
EXPORT_SYMBOL(st_sensor_input_init);

void st_sensor_input_cleanup(struct st_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}
EXPORT_SYMBOL(st_sensor_input_cleanup);

void st_sensor_report_3axes_event(struct st_sensor_data *sdata, s32 *xyz,
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
EXPORT_SYMBOL(st_sensor_report_3axes_event);

void st_sensor_report_single_event(struct st_sensor_data *sdata, s32 data)
{
	struct input_dev  *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
						sdata->timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
						sdata->timestamp & 0xffffffff);
	input_sync(input);
}
EXPORT_SYMBOL(st_sensor_report_single_event);

enum hrtimer_restart st_sensor_poll_function(struct hrtimer *timer)
{
	struct st_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer, struct st_sensor_data,
							hr_timer);

	sdata->timestamp = st_sensor_get_time_ns();
	queue_work(sdata->cdata->workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}
EXPORT_SYMBOL(st_sensor_poll_function);

ssize_t st_sensor_get_enable(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct st_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}
EXPORT_SYMBOL(st_sensor_get_enable);

ssize_t st_sensor_set_enable(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = sdata->enable(sdata);
	else
		err = sdata->disable(sdata);

	return (err < 0) ? err : count;
}
EXPORT_SYMBOL(st_sensor_set_enable);

ssize_t st_sensor_get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct st_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", 1000 / sdata->c_odr);
}
EXPORT_SYMBOL(st_sensor_get_polling_rate);

ssize_t st_sensor_set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int polling_rate;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	sdata->c_odr = 1000 / polling_rate;
	sdata->ktime = ktime_set(0, MS_TO_NS(polling_rate));
	mutex_unlock(&sdata->input_dev->mutex);

	err = sdata->write_odr(sdata);
	if (err < 0)
		return err;

	return count;
}
EXPORT_SYMBOL(st_sensor_set_polling_rate);

#if defined(CONFIG_OF)
static u32 st_sensor_parse_dt(struct st_common_data *cdata)
{
	u32 val;
	struct device_node *np;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "st,drdy-int-pin", &val) &&
							(val <= 2) && (val > 0))
		cdata->drdy_int_pin = (u8)val;
	else
		cdata->drdy_int_pin = 1;

	return 0;
}
#endif

int st_sensor_common_probe(struct st_common_data *cdata, int irq)
{
	/* TODO: add errors management */
	int32_t err;
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

	if (irq > 0) {
#if defined(CONFIG_OF)
		err = st_sensor_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lsm303ah_platform_data *)
					cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) || (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else
			cdata->drdy_int_pin = 1;
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
							cdata->drdy_int_pin);
	}

	cdata->workqueue = create_workqueue(cdata->name);

	if (irq > 0)
		cdata->irq = irq;

	return 0;
}
EXPORT_SYMBOL(st_sensor_common_probe);

void st_sensor_common_remove(struct st_common_data *cdata)
{
}
EXPORT_SYMBOL(st_sensor_common_remove);

#if defined(CONFIG_PM_SLEEP)
int st_sensor_common_suspend(struct st_common_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(st_sensor_common_suspend);

int st_sensor_common_resume(struct st_common_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(st_sensor_common_resume);
#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("lsm303ah_core driver");
MODULE_AUTHOR("Giuseppe Barba <giuseppe.barba@st.com>");
MODULE_LICENSE("GPL v2");
