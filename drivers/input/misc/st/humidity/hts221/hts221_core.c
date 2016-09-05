/*
 * STMicroelectronics hts221_core.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/version.h>

#include "hts221_core.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#define	kstrtoul(x, y, z) strict_strtoul(x, y, z)
#endif

#define MS_TO_NS(x)		((x) *1000000L)

#define REG_WHOAMI_ADDR		0x0f  /* Who am i address register */
#define REG_H_RES_ADDR		0x10  /* Humidity res conf register */
#define REG_T_RES_ADDR		0x10  /* Temperature res conf register */
#define REG_CNTRL1_ADDR		0x20  /* CNTRL1 address register */
#define REG_CNTRL2_ADDR		0x21  /* CNTRL2 address register */
#define REG_H_OUT_L		0x28  /* OUT humidity address register */
#define REG_T_OUT_L		0x2A  /* OUT temperature address register */
#define REG_0RH_CAL_X_H		0x36  /* Calibration H 0 address register */
#define REG_1RH_CAL_X_H		0x3a  /* Calibration H 1 address register */
#define REG_0RH_CAL_Y_H		0x30  /* Calibration H 0 RH address register */
#define REG_1RH_CAL_Y_H		0x31  /* Calibration H 1 RH address register */
#define REG_0T_CAL_X_L		0x3c  /* Calibration T 0 address register */
#define REG_1T_CAL_X_L		0x3e  /* Calibration T 1 address register */
#define REG_0T_CAL_Y_H		0x32  /* Calibration T 0 C address register */
#define REG_1T_CAL_Y_H		0x33  /* Calibration T 1 C address register */
#define REG_STATUS		0x27  /* Status address register */

#define REG_T1_T0_CAL_Y_H	0x35  /* Calibration T0 and T! Address register */

#define MASK_ENABLE		0x80
#define ENABLE_SENSOR		0x80
#define DISABLE_SENSOR		0x00

/* Sensor Resolution */
#define HTS221_H_RESOLUTION_MASK	0xf8
#define HTS221_T_RESOLUTION_MASK	0xC7

/* Default values loaded in probe function */
#define WHOIAM_VALUE		0xbc /* Who Am I default value */
#define REG_DEF_H_RES_ADDR	0x00 /* Humidity res register */
#define REG_DEF_T_RES_ADDR	0x00 /* Temperature res register */
#define REG_DEF_CNTRL1		0x00 /* CNTRL1 default value */
#define REG_DEF_CNTRL2		0x00 /* CNTRL2 default value */

#define REG_DEF_ALL_ZEROS	0x00
#define REG_DEF_BDU		0x04

#define HTS221_BDU		1

#define HTS221_DEV_NAME_H	"hts221_humidity"
#define HTS221_DEV_NAME_T	"hts221_temperature"

#define ENABLE_MASK	0x80 /* Power Normal Mode*/
#define PM_NORMAL	0x80 /* Power Normal Mode*/
#define PM_OFF		0x00 /* Power Down */
#define ODR_1		0x01

#define RES_MAX	(RES_AVGTEMP_064 | RES_AVGPRES_512) /* Max Resol */
#define ODR_MASK	0x03	/* CHANGE*/

/* Humidity and Termometer output data rate ODR */
#define ODR_ONESH	0x00	/* one shot */
#define ODR_1_1		0x01	/* 1  Hz */
#define ODR_7_7		0x02	/* 7  Hz */
#define ODR_12_12	0x03	/* 12.5Hz */

/* Humidity Sensor Resolution */
#define HTS221_H_RESOLUTION_4		0x00  /* Resolution set to 0.4 %RH */
#define HTS221_H_RESOLUTION_8		0x01  /* Resolution set to 0.3 %RH */
#define HTS221_H_RESOLUTION_16		0x02  /* Resolution set to 0.2 %RH */
#define HTS221_H_RESOLUTION_32		0x03  /* Resolution set to 0.15 %RH */
#define HTS221_H_RESOLUTION_64		0x04  /* Resolution set to 0.1 %RH */
#define HTS221_H_RESOLUTION_128		0x05  /* Resolution set to 0.07 %RH */
#define HTS221_H_RESOLUTION_256		0x06  /* Resolution set to 0.05 %RH */
#define HTS221_H_RESOLUTION_512		0x07  /* Resolution set to 0.03 %RH */

/* Temperature Sensor Resolution */
#define HTS221_T_RESOLUTION_2		0x00  /* Resolution set to 0.08 DegC */
#define HTS221_T_RESOLUTION_4		0x08  /* Resolution set to 0.05 DegC */
#define HTS221_T_RESOLUTION_8		0x10  /* Resolution set to 0.04 DegC */
#define HTS221_T_RESOLUTION_16		0x18  /* Resolution set to 0.03 DegC */
#define HTS221_T_RESOLUTION_32		0x20  /* Resolution set to 0.02 DegC */
#define HTS221_T_RESOLUTION_64		0x28  /* Resolution set to 0.015 DegC */
#define HTS221_T_RESOLUTION_128		0x30  /* Resolution set to 0.01 DegC */
#define HTS221_T_RESOLUTION_256		0x38  /* Resolution set to 0.007 DegC */

#define HTS221_DEFAULT_POLL_PERIOD_MS	100

struct hts221_odr {
	u32 cutoff;
	u32 mask;
};

static const struct hts221_odr hts221_odr_table[] = {
	{ 80, ODR_12_12 },
	{ 143, ODR_7_7 },
	{ 1000, ODR_1_1 },
};

struct hts221_resolution {
	u32 res;
	u8 value;
};

static struct hts221_resolution hts221_resolution_h[] = {
	{ 4, HTS221_H_RESOLUTION_4 },
	{ 8, HTS221_H_RESOLUTION_8 },
	{ 16, HTS221_H_RESOLUTION_16 },
	{ 32, HTS221_H_RESOLUTION_32 },
	{ 64, HTS221_H_RESOLUTION_64 },
	{ 128, HTS221_H_RESOLUTION_128 },
	{ 256, HTS221_H_RESOLUTION_256 },
	{ 512, HTS221_H_RESOLUTION_512 },
};

static struct hts221_resolution hts221_resolution_t[] = {
	{ 2, HTS221_T_RESOLUTION_2 },
	{ 4, HTS221_T_RESOLUTION_4 },
	{ 8, HTS221_T_RESOLUTION_8 },
	{ 16, HTS221_T_RESOLUTION_16 },
	{ 32, HTS221_T_RESOLUTION_32 },
	{ 64, HTS221_T_RESOLUTION_64 },
	{ 128, HTS221_T_RESOLUTION_128 },
	{ 256, HTS221_T_RESOLUTION_256 },
};

static int hts221_hw_init(struct hts221_dev *dev)
{
	u8 wai;
	int err;

	err = dev->tf->read(dev->dev, REG_WHOAMI_ADDR, 1, &wai);
	if (err < 0) {
		dev_err(dev->dev, "error reading WHOAMI register\n");
		return err;
	}

	if (WHOIAM_VALUE != wai) {
		dev_err(dev->dev, "device unknown {0x%02x-0x%02x}\n",
			WHOIAM_VALUE, wai);
		return -ENODEV;
	}

	return 0;
}

static int hts221_update_odr(struct hts221_dev *dev, u32 poll_interval)
{
	int i, err;
	u8 data = DISABLE_SENSOR;

	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0)
		return err;

	msleep(50);
	/**
	 * Following, looks for the longest possible odr interval scrolling
	 * the odr_table vector from the end (longest period) backward
	 * (shortest period), to support the poll_interval requested by the
	 * system. It must be the longest period shorter then the set poll
	 * period
	 */
	for (i = ARRAY_SIZE(hts221_odr_table) - 1; i > 0; i--) {
		if (hts221_odr_table[i].cutoff <= poll_interval)
			break;
	}

	dev->odr = hts221_odr_table[i].mask;
	msleep(20);

	data = ENABLE_SENSOR | dev->odr;
#ifdef HTS221_BDU
	data |= REG_DEF_BDU;
#endif
	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0)
		return err;

	return 0;
}

static int hts221_device_power_on(struct hts221_dev *dev)
{
	u8 data;
	int err;

	data = dev->sensors[HTS221_SENSOR_H].res;
	err = dev->tf->write(dev->dev, REG_H_RES_ADDR, 1, &data);
	if (err < 0)
		return err;

	data = dev->sensors[HTS221_SENSOR_T].res;
	err = dev->tf->write(dev->dev, REG_T_RES_ADDR, 1, &data);
	if (err < 0)
		return err;

	/* power off */
	data = DISABLE_SENSOR;
	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0)
		return err;
	msleep(50);

	data = ENABLE_SENSOR | dev->odr;
#ifdef HTS221_BDU
	data |= REG_DEF_BDU;
#endif
	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0)
		return err;
	msleep(50);

	dev->enabled = true;

	return 0;
}

static int hts221_device_power_off(struct hts221_dev *dev)
{
	int err;
	u8 data[4], val = DISABLE_SENSOR;

	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &val);
	if (err < 0)
		return err;

	err = dev->tf->read(dev->dev, REG_H_OUT_L, 4 * sizeof(u8), data);
	if (err < 0)
		return err;

	err = dev->tf->write(dev->dev, REG_CNTRL2_ADDR, 1, &val);
	if (err < 0)
		return err;

	dev->enabled = false;

	return 0;
}

static int hts221_update_res(struct hts221_dev *dev,
			     enum hts221_sensor_type type,
			     u16 val)
{
	int err;
	u8 addr, data, res;

	switch (type) {
	case HTS221_SENSOR_T:
		addr = REG_T_RES_ADDR;
		err = dev->tf->read(dev->dev, REG_T_RES_ADDR, 1 , &data);
		if (err < 0)
			return err;
		res = (data & HTS221_T_RESOLUTION_MASK) | val;
		break;
	case HTS221_SENSOR_H:
		addr = REG_H_RES_ADDR;
		err = dev->tf->read(dev->dev, REG_H_RES_ADDR, 1 , &data);
		if (err < 0)
			return err;
		res = (data & HTS221_H_RESOLUTION_MASK) | val;
		break;
	default:
		return -ENODEV;
	}

	err = dev->tf->write(dev->dev, addr, 1 , &res);
	if (!err)
		dev->sensors[type].res = res;
	return err;
}

static int hts221_input_init(struct hts221_dev *dev, const char *description)
{
	int err;

	dev->input_dev = input_allocate_device();
	if (!dev->input_dev)
		return -ENOMEM;

	dev->input_dev->name = description;
	dev->input_dev->id.bustype = dev->bus_type;
	dev->input_dev->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev, dev);

	/* Set the input event characteristics of the probed sensor driver */
	set_bit(INPUT_EVENT_TYPE, dev->input_dev->evbit);
	set_bit(INPUT_EVENT_TIME_MSB, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_X, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev->mscbit);

	err = input_register_device(dev->input_dev);
	if (err) {
		dev_err(dev->dev, "unable to register input device %s\n",
			dev->input_dev->name);
		input_free_device(dev->input_dev);
		return err;
	}

	return 0;
}

int hts221_enable(struct hts221_dev *dev)
{
	int err = 0;

	mutex_lock(&dev->lock);
	if (!dev->enabled) {
		err = hts221_device_power_on(dev);
		schedule_delayed_work(&dev->input_work,
				      msecs_to_jiffies(dev->poll_interval));
	}
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(hts221_enable);

int hts221_disable(struct hts221_dev *dev)
{
	int err = 0;

	cancel_delayed_work_sync(&dev->input_work);

	mutex_lock(&dev->lock);
	if (dev->enabled)
		err = hts221_device_power_off(dev);
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(hts221_disable);

static void hts221_input_cleanup(struct hts221_dev *dev)
{
	input_unregister_device(dev->input_dev);
	input_free_device(dev->input_dev);
}

static ssize_t attr_get_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	u32 val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->poll_interval;
	mutex_unlock(&dev->lock);

        return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val) || !val)
		return -EINVAL;

	mutex_lock(&dev->lock);
	dev->poll_interval = (u32)val;
	hts221_update_odr(dev, dev->poll_interval);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_res_h(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	u8 val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->sensors[HTS221_SENSOR_H].res;
	mutex_unlock(&dev->lock);

        return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_res_h(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	int i, err;
	unsigned long val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(hts221_resolution_h); i++) {
		if (val == hts221_resolution_h[i].res)
			break;
	}

	if (i == ARRAY_SIZE(hts221_resolution_h))
		return size;

	mutex_lock(&dev->lock);
	err = hts221_update_res(dev, HTS221_SENSOR_H,
				hts221_resolution_h[i].value);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_res_t(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	u8 val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->sensors[HTS221_SENSOR_T].res;
	mutex_unlock(&dev->lock);

        return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_res_t(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	int i, err;
	unsigned long val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(hts221_resolution_t); i++) {
		if (val == hts221_resolution_t[i].res)
			break;
	}

	if (i == ARRAY_SIZE(hts221_resolution_t))
		return size;

	mutex_lock(&dev->lock);
	err = hts221_update_res(dev, HTS221_SENSOR_T,
				hts221_resolution_t[i].value);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_set_oneshot(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val != 0) {
		int err;
		u8 data = DISABLE_SENSOR;
		mutex_lock(&dev->lock);

		err = dev->tf->write(device, REG_CNTRL1_ADDR, 1, &data);
		if (err < 0) {
			mutex_unlock(&dev->lock);
			return err;
		}
		msleep(20);

		data = ENABLE_SENSOR;
#ifdef HTS221_BDU
		data |= REG_DEF_BDU;
#endif

		err = dev->tf->write(device, REG_CNTRL1_ADDR, 1, &data);
		if (err < 0) {
			mutex_unlock(&dev->lock);
			return err;
		}
		dev->odr = ODR_ONESH;

		mutex_unlock(&dev->lock);
	}

	return size;
}

static ssize_t attr_get_heater(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	u8 val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->heater;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_heater(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	u8 data;
	int err;
	unsigned long val;
	struct hts221_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	data = (!val) ? DISABLE_SENSOR : 0x02;

	mutex_lock(&dev->lock);
	err = dev->tf->read(device, REG_CNTRL2_ADDR, 1, &data);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->heater = true;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	bool enabled;
	struct hts221_dev *dev = dev_get_drvdata(device);

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
	struct hts221_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		hts221_enable(dev);
	else
		hts221_disable(dev);

	return size;
}

static struct device_attribute hts221_sysfs_attributes[] = {
	__ATTR(poll_ms, 0644, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(h_res, 0644, attr_get_res_h, attr_set_res_h),
	__ATTR(t_res, 0644, attr_get_res_t, attr_set_res_t),
	__ATTR(oneshot, 0200, NULL, attr_set_oneshot),
	__ATTR(heater, 0644, attr_get_heater, attr_set_heater),
	__ATTR(enable_device, 0644, attr_get_enable, attr_set_enable),
};

static int hts221_sysfs_init(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hts221_sysfs_attributes); i++) {
		if (device_create_file(dev, hts221_sysfs_attributes + i))
			goto err;
	}

	return 0;

err:
	for (; i >= 0; i--)
		device_remove_file(dev, hts221_sysfs_attributes + i);

	return -1;
}

static void hts221_sysfs_remove(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hts221_sysfs_attributes); i++)
		device_remove_file(dev, hts221_sysfs_attributes + i);
}

static int hts221_get_cal_data(struct hts221_dev *dev,
			       enum hts221_sensor_type type)
{
	int err, *slope, *b_gen, cal_x0, cal_x1, cal_y0, cal_y1;
	u8 addr_x0, addr_x1, data[2];

	switch (type) {
	case HTS221_SENSOR_T: {
		u16 cal0, cal1;

		addr_x0 = REG_0T_CAL_X_L;
		addr_x1 = REG_1T_CAL_X_L;

		err = dev->tf->read(dev->dev, REG_0T_CAL_Y_H, 1, &data[0]);
		if (err < 0)
			return err;
		cal0 = data[0];

		err = dev->tf->read(dev->dev, REG_T1_T0_CAL_Y_H, 1, &data[0]);
		if (err < 0)
			return err;
		cal1 = data[0] & 0x3;
		cal_y0 = (s32)((cal1 << 8) | cal0);

		err = dev->tf->read(dev->dev, REG_1T_CAL_Y_H, 1, &data[0]);
		if (err < 0)
			return err;
		cal0 = data[0];

		err = dev->tf->read(dev->dev, REG_T1_T0_CAL_Y_H, 1, &data[0]);
		if (err < 0)
			return err;
		cal1 = data[0] & 0xc;
		cal1 >>= 2;
		cal_y1 = (s32)((cal1 << 8) | cal0);
		break;
	}
	case HTS221_SENSOR_H:
		addr_x0 = REG_0RH_CAL_X_H;
		addr_x1 = REG_1RH_CAL_X_H;

		err = dev->tf->read(dev->dev, REG_0RH_CAL_Y_H, 1, &data[0]);
		if (err < 0)
			return err;
		cal_y0 = data[0];

		err = dev->tf->read(dev->dev, REG_1RH_CAL_Y_H, 1, &data[0]);
		if (err < 0)
			return err;
		cal_y1 = data[0];
		break;
	default:
		return -ENODEV;
	}

	err = dev->tf->read(dev->dev, addr_x0, 2, data);
	if (err < 0)
		return err;
	cal_x0 = (s16)((data[1] << 8) | data[0]);

	err = dev->tf->read(dev->dev, addr_x1, 2, data);
	if (err < 0)
		return err;
	cal_x1 = (s16)((data[1] << 8) | data[0]);

	slope = &dev->sensors[type].slope;
	b_gen = &dev->sensors[type].b_gen;

	*slope = ((cal_y1 - cal_y0) * 8000) / (cal_x1 - cal_x0);
	*b_gen = ((cal_x1 * cal_y0 - cal_x0 * cal_y1) * 1000) /
		 (cal_x1 - cal_x0);
	*b_gen *= 8;

#ifdef HTS221_DEBUG
	dev_info(dev->dev, "slope=%d\tb_gen=%d\n", *slope, *b_gen);
	dev_info(dev->dev, "cal_x0=%d\tcal_x1=%d\n", cal_x0, cal_x1);
	dev_info(dev->dev, "cal_y0=%d\tcal_y1=%d\n", cal_y0, cal_y1);
#endif

	return 0;
}

/**
 * hts221_convert - linear interpolation
 * (x0,y0) (x1,y1) y = mx + b
 *
 * m = (y1 - y0) / (x1 - x0)
 * b = (x0 * y1 - x1 * y0) / (x1 - x0)
 *
 * Humidity
 * {x1,y1} = {H1_T0_OUT,H1_RH}
 * {x0,y0} = {H0_T0_OUT,H0_RH}
 * x = H_OUT
 *
 * Temperature
 * {x1,y1} = {T1_OUT,T1_DegC}
 * {x0,y0} = {T0_OUT,T0_DegC}
 * x = T_OUT
 */
static int hts221_convert(int slope, int b_gen, int x,
			  enum hts221_sensor_type type)
{
	int res = 0;

	res = (slope * x) + b_gen;
	switch (type) {
	case HTS221_SENSOR_T:
		res >>= 6;
		break;
	case HTS221_SENSOR_H:
		res >>= 4;
		break;
	default:
		break;
	}

	return res;
}

static int hts221_get_data(struct hts221_dev *dev, int *data_t, int *data_h)
{
	int err;
	u8 data[2];

	if (dev->odr == ODR_ONESH) {
		u8 tmp_data = 1;
        	static u8 cntx = 0;

		err = dev->tf->write(dev->dev, REG_CNTRL2_ADDR, 1, &tmp_data);
		if (err < 0)
			return err;

		do {
			 err = dev->tf->read(dev->dev, REG_STATUS, 1,
					     &tmp_data);
			 if (err < 0)
				 return err;
			 msleep(45);
		} while (((tmp_data & 3) != 3) && (3 >= cntx++));
		cntx = 0;
	}

	err = dev->tf->read(dev->dev, REG_H_OUT_L, 2, data);
	if (err < 0)
		return err;
	*data_h = (s16)((data[1] << 8) | data[0]);
	*data_h = hts221_convert(dev->sensors[HTS221_SENSOR_H].slope,
				 dev->sensors[HTS221_SENSOR_H].b_gen,
				 *data_h, HTS221_SENSOR_H);

	err = dev->tf->read(dev->dev, REG_T_OUT_L, 2, data);
	if (err < 0)
		return err;
	*data_t = (s16)((data[1] << 8) | data[0]);
	*data_t = hts221_convert(dev->sensors[HTS221_SENSOR_T].slope,
				 dev->sensors[HTS221_SENSOR_T].b_gen,
				 *data_t, HTS221_SENSOR_T);

#ifdef HTS221_DEBUG
	dev_info(dev->dev, "data_h=%d\tdata_t=%d\n", *data_h, *data_t);
#endif

	return 0;
}

static void hts221_report_data(struct hts221_dev *dev, int data_t, int data_h,
			       s64 timestamp)
{
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, data_t);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, data_h);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(dev->input_dev);
}

static void hts221_input_work_fn(struct work_struct *work)
{
	int err, data_t, data_h;
	struct hts221_dev *dev;

	dev = container_of((struct delayed_work *)work,
			   struct hts221_dev, input_work);

	mutex_lock(&dev->lock);

	err = hts221_get_data(dev, &data_t, &data_h);
	if (err < 0)
		dev_err(dev->dev, "get data failed\n");
	else
		hts221_report_data(dev, data_t, data_h, hts221_get_time_ns());

	mutex_unlock(&dev->lock);

	schedule_delayed_work(&dev->input_work,
			      msecs_to_jiffies(dev->poll_interval));
}

int hts221_probe(struct hts221_dev *dev)
{
	int err;

	BUILD_BUG_ON(!ARRAY_SIZE(hts221_odr_table));

	mutex_lock(&dev->lock);

	dev->poll_interval = 100;

	err =  hts221_hw_init(dev);
	if (err < 0) {
                dev_err(dev->dev, "hw init failed: %d\n", err);
		goto unlock;
	}

	err = hts221_update_odr(dev, HTS221_DEFAULT_POLL_PERIOD_MS);
	if (err < 0) {
                dev_err(dev->dev, "set odr failed: %d\n", err);
		goto unlock;
	}

	err = hts221_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto unlock;
	}

	err = hts221_update_res(dev, HTS221_SENSOR_T, REG_DEF_T_RES_ADDR);
	if (err < 0) {
		dev_err(dev->dev, "update T resolution failed: %d\n", err);
		goto power_off;
	}

	err = hts221_update_res(dev, HTS221_SENSOR_H, REG_DEF_H_RES_ADDR);
	if (err < 0) {
		dev_err(dev->dev, "update H resolution failed: %d\n", err);
		goto power_off;
	}

	err = hts221_input_init(dev, "hts221");
	if (err < 0) {
		dev_err(dev->dev, "input init failed: %d\n", err);
		goto power_off;
	}

	err = hts221_sysfs_init(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "sysfs register failed\n");
		goto input_cleanup;
	}

	err = hts221_device_power_off(dev);
	if (err < 0) {
		dev_err(dev->dev, "power off failed: %d\n", err);
		goto input_cleanup;
	}

	/* get calibration data */
	if ((hts221_get_cal_data(dev, HTS221_SENSOR_T) < 0) ||
	    (hts221_get_cal_data(dev, HTS221_SENSOR_H) < 0)) {
		dev_err(dev->dev, "get calibration data failed: %d\n", err);
		goto input_cleanup;
	}

	INIT_DELAYED_WORK(&dev->input_work, hts221_input_work_fn);

	mutex_unlock(&dev->lock);

	return 0;

input_cleanup:
	hts221_input_cleanup(dev);
power_off:
	hts221_device_power_off(dev);
unlock:
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(hts221_probe);

void hts221_remove(struct hts221_dev *dev)
{
	cancel_delayed_work_sync(&dev->input_work);
	hts221_device_power_off(dev);
	hts221_input_cleanup(dev);
	hts221_sysfs_remove(dev->dev);
}
EXPORT_SYMBOL(hts221_remove);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");
