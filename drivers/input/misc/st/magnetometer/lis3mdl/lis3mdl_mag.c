/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lis3mdl_mag.c
* Authors            : MSH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
*		     : Lorenzo Bianconi (lorenzo.bianconi@st.com)
*		     : Authors are willing to be considered the contact
*		     : and update points for the driver.
* Version            : V.1.0.2
* Date               : 2016/May/2
* Description        : LIS3MDL magnetometer driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "lis3mdl.h"

#define MS_TO_NS(x)		((x) * 1000000L)

#define LIS3MDL_DEV_NAME	"lis3mdl_mag"

#define LIS3MDL_MAG_MIN_POLL_PERIOD_MS	10

/* Address registers */
#define REG_WHOAMI_ADDR		0x0F /* Who am i address register */
#define REG_CNTRL1_ADDR		0x20 /* CNTRL1 address register */
#define REG_CNTRL2_ADDR		0x21 /* CNTRL2 address register */
#define REG_CNTRL3_ADDR		0x22 /* CNTRL3 address register */
#define REG_CNTRL4_ADDR		0x23 /* CNTRL4 address register */
#define REG_CNTRL5_ADDR		0x24 /* CNTRL5 address register */

#define REG_MAG_DATA_ADDR	0x28 /* Mag. data low address register */

/* Sensitivity [uGauss/LSB] */
#define SENSITIVITY_MAG_4G	146
#define SENSITIVITY_MAG_8G	292
#define SENSITIVITY_MAG_12G	438
#define SENSITIVITY_MAG_16G	584

/* ODR */
#define ODR_MAG_MASK		0x1C /* Mask for odr change on mag */
#define LIS3MDL_MAG_ODR0_625	0x00 /* 0.625Hz output data rate */
#define LIS3MDL_MAG_ODR1_25	0x04 /* 1.25Hz output data rate */
#define LIS3MDL_MAG_ODR2_5	0x08 /* 2.5Hz output data rate */
#define LIS3MDL_MAG_ODR5	0x0C /* 5Hz output data rate */
#define LIS3MDL_MAG_ODR10	0x10 /* 10Hz output data rate */
#define LIS3MDL_MAG_ODR20	0x14 /* 20Hz output data rate */
#define LIS3MDL_MAG_ODR40	0x18 /* 40Hz output data rate */
#define LIS3MDL_MAG_ODR80	0x1C /* 80Hz output data rate */

/* Magnetic sensor mode */
#define MSMS_MASK		0x03 /* Mask magnetic sensor mode */
#define POWEROFF_MAG		0x02 /* Power Down */
#define CONTINUOS_CONVERSION	0x00 /* Continuos Conversion */

/* X and Y axis operative mode selection */
#define X_Y_PERFORMANCE_MASK		0x60
#define X_Y_LOW_PERFORMANCE		0x00
#define X_Y_MEDIUM_PERFORMANCE		0x20
#define X_Y_HIGH_PERFORMANCE		0x40
#define X_Y_ULTRA_HIGH_PERFORMANCE	0x60

/* Z axis operative mode selection */
#define Z_PERFORMANCE_MASK		0x0c
#define Z_LOW_PERFORMANCE		0x00
#define Z_MEDIUM_PERFORMANCE		0x04
#define Z_HIGH_PERFORMANCE		0x08
#define Z_ULTRA_HIGH_PERFORMANCE	0x0c

/* Default values loaded in probe function */
#define WHOIAM_VALUE		0x3d /* Who Am I default value */
#define REG_DEF_CNTRL1		0x60 /* CNTRL1 default value */
#define REG_DEF_CNTRL2		0x00 /* CNTRL2 default value */
#define REG_DEF_CNTRL3		0x03 /* CNTRL3 default value */
#define REG_DEF_CNTRL4		0x0c /* CNTRL4 default value */
#define REG_DEF_CNTRL5		0x40 /* CNTRL5 default value */

#define REG_DEF_ALL_ZEROS	0x00

/* Magnetometer Sensor Full Scale */
#define LIS3MDL_MAG_FS_MASK	0x60
#define LIS3MDL_MAG_FS_4G	0x00 /* Full scale 4 Gauss */
#define LIS3MDL_MAG_FS_8G	0x20 /* Full scale 8 Gauss */
#define LIS3MDL_MAG_FS_12G	0x40 /* Full scale 12 Gauss */
#define LIS3MDL_MAG_FS_16G	0x60 /* Full scale 16 Gauss */


#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

static struct workqueue_struct *lis3mdl_workqueue;

struct {
	unsigned int cutoff_us;
	u8 value;
} lis3mdl_mag_odr_table[] = {
	{ 12, LIS3MDL_MAG_ODR80 },
	{ 25, LIS3MDL_MAG_ODR40 },
	{ 50, LIS3MDL_MAG_ODR20 },
	{ 100, LIS3MDL_MAG_ODR10 },
	{ 200, LIS3MDL_MAG_ODR5 },
	{ 400, LIS3MDL_MAG_ODR2_5 },
	{ 800, LIS3MDL_MAG_ODR1_25 },
	{ 1600, LIS3MDL_MAG_ODR0_625 },
};

static inline int64_t lis3mdl_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static int lis3mdl_write_data_with_mask(struct lis3mdl_dev *dev,
					u8 addr, u8 mask, u8 data)
{
	int err;
	u8 old_data, new_data;

	err = dev->tf->read(dev->dev, addr, 1, &old_data);
	if (err < 0)
		return err;

	new_data = ((old_data & ~mask) | (data & mask));
	if (old_data == new_data)
		return 1;

	return dev->tf->write(dev->dev, addr, 1, &new_data);
}

static int lis3mdl_hw_init(struct lis3mdl_dev *dev)
{
	int err;
	u8 data;

	err = dev->tf->read(dev->dev, REG_WHOAMI_ADDR, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "Error reading WHO_AM_I\n");
		return err;
	}

	if (data != WHOIAM_VALUE) {
		dev_err(dev->dev, "device unknown {0x%02x-0x%02x}\n",
			WHOIAM_VALUE, data);
		return -ENODEV;
	}

	data = REG_DEF_CNTRL1;
	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0)
		return err;

	data = REG_DEF_CNTRL2;
	err = dev->tf->write(dev->dev, REG_CNTRL2_ADDR, 1, &data);
	if (err < 0)
		return err;

	data = REG_DEF_CNTRL3;
	err = dev->tf->write(dev->dev, REG_CNTRL3_ADDR, 1, &data);
	if (err < 0)
		return err;

	data = REG_DEF_CNTRL4;
	err = dev->tf->write(dev->dev, REG_CNTRL4_ADDR, 1, &data);
	if (err < 0)
		return err;

	data = REG_DEF_CNTRL5;
	err = dev->tf->write(dev->dev, REG_CNTRL5_ADDR, 1, &data);
	if (err < 0)
		return err;

	dev->xy_mode = X_Y_ULTRA_HIGH_PERFORMANCE;
	dev->z_mode = Z_ULTRA_HIGH_PERFORMANCE;

	return 0;
}

static int lis3mdl_mag_device_power_off(struct lis3mdl_dev *dev)
{
	int err;

	err = lis3mdl_write_data_with_mask(dev, REG_CNTRL3_ADDR, MSMS_MASK,
					   POWEROFF_MAG);
	if (err < 0)
		dev_err(dev->dev, "power off failed: %d\n", err);

	atomic_set(&dev->enabled_mag, 0);

	return 0;
}

static int lis3mdl_mag_device_power_on(struct lis3mdl_dev *dev)
{
	int err;
	u8 data = REG_DEF_CNTRL1;

	err = dev->tf->write(dev->dev, REG_CNTRL1_ADDR, 1, &data);
	if (err < 0)
		goto err_resume_state;

	err = lis3mdl_write_data_with_mask(dev, REG_CNTRL3_ADDR, MSMS_MASK,
					   CONTINUOS_CONVERSION);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&dev->enabled_mag, 1);

	return 0;

err_resume_state:
	dev_err(dev->dev, "hw power on error: %d\n", err);

	return err;
}

static int lis3mdl_mag_update_fs_range(struct lis3mdl_dev *dev, u8 new_range)
{
	int err;
	u32 sensitivity;
	u8 data = LIS3MDL_MAG_FS_MASK & new_range;

	switch (new_range) {
	case LIS3MDL_MAG_FS_4G:
		sensitivity = SENSITIVITY_MAG_4G;
		break;
	case LIS3MDL_MAG_FS_8G:
		sensitivity = SENSITIVITY_MAG_8G;
		break;
	case LIS3MDL_MAG_FS_12G:
		sensitivity = SENSITIVITY_MAG_12G;
		break;
	case LIS3MDL_MAG_FS_16G:
		sensitivity = SENSITIVITY_MAG_16G;
		break;
	default:
		dev_err(dev->dev, "invalid fs range: %u\n", new_range);
		return -EINVAL;
	}

	err = dev->tf->write(dev->dev, REG_CNTRL2_ADDR, 1, &data);
	if (err < 0) {
		dev_err(dev->dev, "update fs range failed: %d\n", err);
		return err;
	}
	dev->sensitivity_mag = sensitivity;

	return 0;
}

static int lis3mdl_mag_update_odr(struct lis3mdl_dev *dev, u32 poll_interval)
{
	int i, err;

	if (atomic_read(&dev->enabled_mag) == 0)
		return -1;

	for (i = ARRAY_SIZE(lis3mdl_mag_odr_table) - 1; i >= 0; i--) {
		if ((lis3mdl_mag_odr_table[i].cutoff_us <= poll_interval) ||
		    (i == 0))
			break;
	}

	err = lis3mdl_write_data_with_mask(dev, REG_CNTRL1_ADDR, ODR_MAG_MASK,
					   lis3mdl_mag_odr_table[i].value);
	if (err < 0) {
		dev_err(dev->dev, "update odr failed %d\n", err);
		return err;
	}

	dev->ktime = ktime_set(0, MS_TO_NS(poll_interval));

	return err;
}

static int lis3mdl_mag_update_operative_mode(struct lis3mdl_dev *dev, int axis,
					     u8 value)
{
	int err;
	u8 mask, addr, *mode;

	if (axis == 0) {
		addr = REG_CNTRL1_ADDR;
		mask = X_Y_PERFORMANCE_MASK;
		mode = &dev->xy_mode;
	} else {
		addr = REG_CNTRL4_ADDR;
		mask = Z_PERFORMANCE_MASK;
		mode = &dev->z_mode;
	}

	
	mutex_lock(&dev->lock);
	err = lis3mdl_write_data_with_mask(dev, addr, mask, value);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		dev_err(dev->dev, "update operative mode failed: %d\n", err);
		return err;
	}
	mutex_unlock(&dev->lock);
	*mode = value;

	return 0;
}

int lis3mdl_mag_enable(struct lis3mdl_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled_mag, 0, 1)) {
		int err;

		mutex_lock(&dev->lock);
		err = lis3mdl_mag_device_power_on(dev);
		if (err < 0) {
			mutex_unlock(&dev->lock);
			return err;
		}
		lis3mdl_mag_update_odr(dev, dev->poll_interval);
		mutex_unlock(&dev->lock);

		hrtimer_start(&dev->hr_timer_mag, dev->ktime, HRTIMER_MODE_REL);
	}
	return 0;
}
EXPORT_SYMBOL(lis3mdl_mag_enable);

int lis3mdl_mag_disable(struct lis3mdl_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_mag, 1, 0)) {
		cancel_work_sync(&dev->input_work_mag);
		hrtimer_cancel(&dev->hr_timer_mag);

		mutex_lock(&dev->lock);
		lis3mdl_mag_device_power_off(dev);
		mutex_unlock(&dev->lock);
	}
	return 0;
}
EXPORT_SYMBOL(lis3mdl_mag_disable);

static void lis3mdl_mag_input_cleanup(struct lis3mdl_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static ssize_t attr_get_polling_rate_mag(struct device *device,
					 struct device_attribute *attr,
					 char *buf)
{
	u32 val;
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate_mag(struct device *device,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long interval_ms;
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;

	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
			    LIS3MDL_MAG_MIN_POLL_PERIOD_MS);
	mutex_lock(&dev->lock);

	dev->poll_interval = (u32)interval_ms;
	lis3mdl_mag_update_odr(dev, dev->poll_interval);

	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_enable_mag(struct device *device,
				   struct device_attribute *attr, char *buf)
{
	struct lis3mdl_dev *dev = dev_get_drvdata(device);
	int val = atomic_read(&dev->enabled_mag);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_mag(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	unsigned long val;
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis3mdl_mag_enable(dev);
	else
		lis3mdl_mag_disable(dev);

	return size;
}

static ssize_t attr_get_range_mag(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	int range = 2;
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	switch (dev->fs_range) {
	case LIS3MDL_MAG_FS_4G:
		range = 4;
		break;
	case LIS3MDL_MAG_FS_8G:
		range = 8;
		break;
	case LIS3MDL_MAG_FS_12G:
		range = 12;
		break;
	case LIS3MDL_MAG_FS_16G:
		range = 16;
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_mag(struct device *device,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int err;
	u8 range;
	unsigned long val;
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 4:
		range = LIS3MDL_MAG_FS_4G;
		break;
	case 8:
		range = LIS3MDL_MAG_FS_8G;
		break;
	case 12:
		range = LIS3MDL_MAG_FS_12G;
		break;
	case 16:
		range = LIS3MDL_MAG_FS_16G;
		break;
	default:
		dev_err(dev->dev, "invalid range request: %lu\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lis3mdl_mag_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->fs_range = range;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_xy_mode(struct device *device,
				struct device_attribute *attr, char *buf)
{
	char mode[13];
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	switch (dev->xy_mode) {
	case X_Y_HIGH_PERFORMANCE:
		strcpy(mode, "high");
		break;
	case X_Y_LOW_PERFORMANCE:
		strcpy(mode, "low");
		break;
	case X_Y_MEDIUM_PERFORMANCE:
		strcpy(mode, "medium");
		break;
	case X_Y_ULTRA_HIGH_PERFORMANCE:
		strcpy(mode, "ultra_high");
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_xy_mode(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	u8 mode;
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	if (!strncmp(buf, "high", 4)) {
		mode = X_Y_HIGH_PERFORMANCE;
	} else if (!strncmp(buf, "low", 3)) {
		mode = X_Y_LOW_PERFORMANCE;
	} else if (!strncmp(buf, "medium", 6)) {
		mode = X_Y_MEDIUM_PERFORMANCE;
	} else if (!strncmp(buf, "ultra_high", 10)) {
		mode = X_Y_ULTRA_HIGH_PERFORMANCE;
	} else {
		dev_err(dev->dev, "invalid value request: %s\n", buf);
		return -EINVAL;
	}

	err = lis3mdl_mag_update_operative_mode(dev, 0, mode);
	if (err < 0)
		return -EINVAL;

	return size;
}

static ssize_t attr_get_z_mode(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	char mode[13];
	struct lis3mdl_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	switch (dev->z_mode) {
	case Z_HIGH_PERFORMANCE:
		strcpy(mode, "high");
		break;
	case Z_LOW_PERFORMANCE:
		strcpy(mode, "low");
		break;
	case Z_MEDIUM_PERFORMANCE:
		strcpy(mode, "medium");
		break;
	case Z_ULTRA_HIGH_PERFORMANCE:
		strcpy(mode, "ultra_high");
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_z_mode(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3mdl_dev *dev = dev_get_drvdata(device);
	u8 mode;
	int err;

	if (!strncmp(buf, "high", 4)) {
		mode = Z_HIGH_PERFORMANCE;
	} else if (!strncmp(buf, "low", 3)) {
		mode = Z_LOW_PERFORMANCE;
	} else if (!strncmp(buf, "medium", 6)) {
		mode = Z_MEDIUM_PERFORMANCE;
	} else if (!strncmp(buf, "ultra_high", 10)) {
		mode = Z_ULTRA_HIGH_PERFORMANCE;
	} else {
		dev_err(dev->dev, "invalid value request: %s\n", buf);
		return -EINVAL;
	}

	err = lis3mdl_mag_update_operative_mode(dev, 1, mode);
	if (err < 0)
		return err;

	return size;
}

#ifdef LIS3MDL_DEBUG
static int write_bit_on_register(struct lis3mdl_dev *dev,
				 u8 addr, u8 *resume_value,
				 u8 mask, int value)
{
	int err;
	u8 updated_val, data;
	u8 buf[2];
	u8 val = 0x00;

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, addr, 1, &data);
	if (err < 0)
		goto unlock;

	if (resume_value)
		*resume_value = data;

	if (!mask) {
		updated_val = value;
	} else {
		if (value > 0)
			val = 0xff;
		updated_val = (mask & val) | (~mask & data);
	}
	err = dev->tf->write(dev->dev, addr, 1, &updated_val);
	if (err < 0)
		goto unlock;

	if (resume_value != NULL)
		*resume_value = buf[0];

	if (resume_value)
		*resume_value = updated_val;

unlock:
	mutex_unlock(&dev->lock);

	return err;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_mag,
	       attr_set_polling_rate_mag),
	__ATTR(full_scale, 0666, attr_get_range_mag, attr_set_range_mag),
	__ATTR(enable_device, 0666, attr_get_enable_mag, attr_set_enable_mag),
	__ATTR(x_y_opearative_mode, 0666, attr_get_xy_mode, attr_set_xy_mode),
	__ATTR(z_opearative_mode, 0666, attr_get_z_mode, attr_set_z_mode),

};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: unable to create interface\n", __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int lis3mdl_mag_get_data(struct lis3mdl_dev *dev, int *xyz)
{
	int err;
	u8 mag_data[6];

	err = dev->tf->read(dev->dev, REG_MAG_DATA_ADDR, 6, mag_data);
	if (err < 0)
		return err;

	xyz[0] = ((s32)((s16)((mag_data[1] << 8) | (mag_data[0]))));
	xyz[1] = ((s32)((s16)((mag_data[3] << 8) | (mag_data[2]))));
	xyz[2] = ((s32)((s16)((mag_data[5] << 8) | (mag_data[4]))));

	xyz[0] *= dev->sensitivity_mag;
	xyz[1] *= dev->sensitivity_mag;
	xyz[2] *= dev->sensitivity_mag;

#ifdef LIS3MDL_DEBUG
	dev_info(dev->dev, "x=%d\ty=%d\tz=%d\n",  xyz[0],  xyz[1],  xyz[2]);
#endif

	return err;
}

static void lis3mdl_mag_report_values(struct lis3mdl_dev *dev, int *xyz)
{
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    dev->timestamp >> 32);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    dev->timestamp & 0xffffffff);
	input_sync(dev->input_dev_mag);
}

int lis3mdl_mag_input_init(struct lis3mdl_dev *dev)
{
	int err;

	dev->input_dev_mag = input_allocate_device();
	if (!dev->input_dev_mag) {
		dev_err(dev->dev, "input device allocation failed\n");

		return -ENOMEM;
	}

	dev->input_dev_mag->name = LIS3MDL_DEV_NAME;
	dev->input_dev_mag->id.bustype = dev->bus_type;
	dev->input_dev_mag->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_mag, dev);

	__set_bit(INPUT_EVENT_TYPE, dev->input_dev_mag->evbit);
	__set_bit(INPUT_EVENT_TIME_MSB, dev->input_dev_mag->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, dev->input_dev_mag->mscbit);
	__set_bit(INPUT_EVENT_X, dev->input_dev_mag->mscbit);
	__set_bit(INPUT_EVENT_Y, dev->input_dev_mag->mscbit);
	__set_bit(INPUT_EVENT_Z, dev->input_dev_mag->mscbit);

	err = input_register_device(dev->input_dev_mag);
	if (err) {
		dev_err(dev->dev,
			"unable to register magnetometer input device %s\n",
			dev->input_dev_mag->name);
		input_free_device(dev->input_dev_mag);
		return err;
	}

	return 0;
}

static void lis3mdl_input_cleanup(struct lis3mdl_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static void poll_function_work_mag(struct work_struct *input_work_mag)
{
	struct lis3mdl_dev *dev;

	dev = container_of((struct work_struct *)input_work_mag,
			    struct lis3mdl_dev, input_work_mag);

	hrtimer_start(&dev->hr_timer_mag, dev->ktime, HRTIMER_MODE_REL);

	mutex_lock(&dev->lock);

	if (atomic_read(&dev->enabled_mag)) {
		int xyz[3] = {};

		if (lis3mdl_mag_get_data(dev, xyz) < 0)
			dev_err(dev->dev, "get_magnetometer_data failed\n");
		else
			lis3mdl_mag_report_values(dev, xyz);
	}

	mutex_unlock(&dev->lock);
}

static enum hrtimer_restart poll_function_read_mag(struct hrtimer *timer)
{
	struct lis3mdl_dev *dev;

	dev = container_of((struct hrtimer *)timer,
			    struct lis3mdl_dev, hr_timer_mag);

	dev->timestamp = lis3mdl_get_time_ns();
	queue_work(lis3mdl_workqueue, &dev->input_work_mag);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_OF
static u32 lis3mdl_parse_dt(struct lis3mdl_dev *dev, struct device *device)
{
	u32 val;
	struct device_node *np;

	np = device->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "poll-interval", &val))
		dev->poll_interval = val;
	else
		dev->poll_interval = 10;

	if (!of_property_read_u32(np, "fs-range", &val))
		dev->fs_range = val;
	else
		dev->fs_range = LIS3MDL_MAG_FS_4G;

	return 0;
}
#endif

int lis3mdl_mag_probe(struct lis3mdl_dev *dev)
{
	int err;

	if (lis3mdl_workqueue == 0)
		lis3mdl_workqueue = create_workqueue("lis3mdl_workqueue");

	hrtimer_init(&dev->hr_timer_mag, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer_mag.function = &poll_function_read_mag;

	mutex_lock(&dev->lock);

#ifdef CONFIG_OF
	lis3mdl_parse_dt(dev, dev->dev);
#endif

	err = lis3mdl_hw_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", err);
		goto err_unlock;
	}

	err = lis3mdl_mag_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto err_unlock;
	}

	err = lis3mdl_mag_update_fs_range(dev, dev->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lis3mdl_mag_update_odr(dev, dev->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis3mdl_mag_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "sysfs register failed\n");
		goto err_input_cleanup;
	}

	lis3mdl_mag_device_power_off(dev);

	INIT_WORK(&dev->input_work_mag, poll_function_work_mag);

	mutex_unlock(&dev->lock);

	return 0;

err_input_cleanup:
	lis3mdl_input_cleanup(dev);
err_power_off:
	lis3mdl_mag_device_power_off(dev);
err_unlock:
	mutex_unlock(&dev->lock);
	if (lis3mdl_workqueue) {
		flush_workqueue(lis3mdl_workqueue);
		destroy_workqueue(lis3mdl_workqueue);
	}

	return err;
}
EXPORT_SYMBOL(lis3mdl_mag_probe);

int lis3mdl_mag_remove(struct lis3mdl_dev *dev)
{
	lis3mdl_mag_disable(dev);
	lis3mdl_mag_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	if (lis3mdl_workqueue) {
		flush_workqueue(lis3mdl_workqueue);
		destroy_workqueue(lis3mdl_workqueue);
		lis3mdl_workqueue = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(lis3mdl_mag_remove);

MODULE_DESCRIPTION("lis3mdl magnetometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");
