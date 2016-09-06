/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lsm9ds1_mag.c
* Authors            : MSH - C&I BU - Application Team
*		     : Giuseppe Barba (giuseppe.barba@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
*		     : Lorenzo Bianconi (lorenzo.bianconi@st.com)
*		     : Authors are willing to be considered the contact
*		     : and update points for the driver.
* Version            : V.1.0.0
* Date               : 2016/May/13
* Description        : LSM9DS1 magnetometer driver
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
********************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
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

#include "lsm9ds1.h"

#define MS_TO_NS(x)			((x) * 1000000L)

/* Address registers */
#define REG_WHOAMI_ADDR			0x0F
#define CTRL_REG1_M			0x20
#define CTRL_REG2_M			0x21
#define CTRL_REG3_M			0x22
#define CTRL_REG4_M			0x23
#define CTRL_REG5_M			0x24
#define INT_CFG_M			0x30
#define INT_THS_L			0x32
#define INT_THS_H			0x33

#define REG_MAG_DATA_ADDR		0x28 /** Mag. data low address register */

/* Sensitivity [ugauss/LSB] */
#define SENSITIVITY_MAG_4G		146
#define SENSITIVITY_MAG_8G		292
#define SENSITIVITY_MAG_12G		430
#define SENSITIVITY_MAG_16G		584

/* Magnetic sensor mode */
#define CTRL_REG3_M_MD_MASK		0x03
#define CTRL_REG3_M_MD_OFF		0x02
#define CTRL_REG3_M_MD_CONTINUOUS	0x00
#define CTRL_REG3_M_MD_SINGLE		0x01

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
#define DEF_ZERO			0x00

#define WHOIAM_VALUE			0x3D
#define CTRL_REG1_M_DEF			0x60
#define CTRL_REG2_M_DEF			DEF_ZERO
#define CTRL_REG3_M_DEF			CTRL_REG3_M_MD_CONTINUOUS
#define CTRL_REG4_M_DEF			DEF_ZERO
#define CTRL_REG5_M_DEF			0x40
#define INT_CFG_M_DEF			DEF_ZERO
#define INT_THS_H_DEF			DEF_ZERO
#define INT_THS_L_DEF			DEF_ZERO

struct {
	unsigned int cutoff_us;
	u8 value;
} lsm9ds1_mag_odr_table[] = {
	{   12, LSM9DS1_MAG_ODR80 },
	{   25, LSM9DS1_MAG_ODR40 },
	{   50, LSM9DS1_MAG_ODR20 },
	{  100, LSM9DS1_MAG_ODR10 },
	{  200, LSM9DS1_MAG_ODR5 },
	{  400, LSM9DS1_MAG_ODR2_5 },
	{  800, LSM9DS1_MAG_ODR1_25 },
	{ 1600, LSM9DS1_MAG_ODR0_625 },
};

struct interrupt_enable {
	atomic_t enable;
	u8 address;
	u8 mask;
};

struct interrupt_value {
	int value;
	u8 address;
};

static const struct lsm9ds1_mag_platform_data default_lsm9ds1_mag_pdata = {
	.poll_interval = 100,
	.min_interval = LSM9DS1_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = LSM9DS1_MAG_FS_4G,
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
};

struct reg_rw {
	u8 address;
	u8 default_value;
	u8 resume_value;
};

struct reg_r {
	u8 address;
	u8 value;
};

static struct status_registers {
	struct reg_r who_am_i;
	struct reg_rw ctrl_reg1_m;
	struct reg_rw ctrl_reg2_m;
	struct reg_rw ctrl_reg3_m;
	struct reg_rw ctrl_reg4_m;
	struct reg_rw ctrl_reg5_m;
	struct reg_rw int_cfg_m;
	struct reg_rw int_ths_l;
	struct reg_rw int_ths_h;
} status_registers = {
	.who_am_i = {.address = REG_WHOAMI_ADDR, .value = WHOIAM_VALUE,},
	.ctrl_reg1_m = {.address = CTRL_REG1_M, .default_value = CTRL_REG1_M_DEF,},
	.ctrl_reg2_m = {.address = CTRL_REG2_M, .default_value = CTRL_REG2_M_DEF,},
	.ctrl_reg3_m = {.address = CTRL_REG3_M, .default_value = CTRL_REG3_M_DEF,},
	.ctrl_reg4_m = {.address = CTRL_REG4_M, .default_value = CTRL_REG4_M_DEF,},
	.ctrl_reg5_m = {.address = CTRL_REG5_M, .default_value = CTRL_REG5_M_DEF,},
	.int_cfg_m = {.address = INT_CFG_M, .default_value = INT_CFG_M_DEF,},
	.int_ths_h = {.address = INT_THS_H, .default_value = INT_THS_H_DEF,},
	.int_ths_l = {.address = INT_THS_L, .default_value = INT_THS_L_DEF,},
};

static int lsm9ds1_hw_init(struct lsm9ds1_mag_dev *dev)
{
	int err = -1;
	u8 buf[1];

#ifdef LSM9DS1_DEBUG
	pr_info("%s: hw init start\n", LSM9DS1_MAG_DEV_NAME);
#endif

	err = dev->tf->read(dev->dev, status_registers.who_am_i.address, 1,
			    buf);
	if (err < 0) {
		dev_warn(dev->dev, "Error reading WHO_AM_I\n");
		goto err_firstread;
	} else {
		dev->hw_working = 1;
	}

	if (buf[0] != status_registers.who_am_i.value) {
		dev_err(dev->dev,
			"device unknown 0x%02x-0x%02x\n",
			status_registers.who_am_i.value, buf[0]);
			err = -1;
		goto err_unknown_device;
	}

	status_registers.ctrl_reg1_m.resume_value =
				status_registers.ctrl_reg1_m.default_value;
	status_registers.ctrl_reg2_m.resume_value =
				status_registers.ctrl_reg2_m.default_value;
	status_registers.ctrl_reg3_m.resume_value =
				status_registers.ctrl_reg3_m.default_value;
	status_registers.ctrl_reg4_m.resume_value =
				status_registers.ctrl_reg4_m.default_value;
	status_registers.ctrl_reg5_m.resume_value =
				status_registers.ctrl_reg5_m.default_value;
	status_registers.int_cfg_m.resume_value =
				status_registers.int_cfg_m.default_value;
	status_registers.int_ths_h.resume_value =
				status_registers.int_ths_h.default_value;
	status_registers.int_ths_l.resume_value =
				status_registers.int_ths_l.default_value;

	dev->xy_mode = X_Y_ULTRA_HIGH_PERFORMANCE;
	dev->z_mode = Z_ULTRA_HIGH_PERFORMANCE;
	dev->hw_initialized = 1;

#ifdef LSM9DS1_DEBUG
	pr_info("%s: hw init done\n", LSM9DS1_MAG_DEV_NAME);
#endif

	return 0;

err_unknown_device:
err_firstread:
	dev->hw_working = 0;
	dev->hw_initialized = 0;
	return err;
}

static int lsm9ds1_mag_device_power_off(struct lsm9ds1_mag_dev *dev)
{
	int err;
	u8 buf[1];

	buf[0] = ((CTRL_REG3_M_MD_MASK & CTRL_REG3_M_MD_OFF) |
		  (~CTRL_REG3_M_MD_MASK &
		   status_registers.ctrl_reg3_m.resume_value));

	err = dev->tf->write(dev->dev, status_registers.ctrl_reg3_m.address, 1,
			     buf);
	if (err < 0)
		dev_err(dev->dev, "magnetometer soft power off failed: %d\n",
			err);

	if (dev->pdata_mag->power_off)
		dev->pdata_mag->power_off();

	atomic_set(&dev->enabled_mag, 0);

	return 0;
}

static int lsm9ds1_mag_device_power_on(struct lsm9ds1_mag_dev *dev)
{
	int err = -1;
	u8 buf[5];

	if (dev->pdata_mag->power_on) {
		err = dev->pdata_mag->power_on();
		if (err < 0) {
			dev_err(dev->dev, "magnetometer power_on failed: %d\n",
				err);
			return err;
		}
	}
	
	
	buf[0] = status_registers.ctrl_reg1_m.resume_value;
	buf[1] = status_registers.ctrl_reg2_m.resume_value;
	buf[2] = status_registers.ctrl_reg3_m.resume_value;
	buf[3] = status_registers.ctrl_reg4_m.resume_value;
	buf[4] = status_registers.ctrl_reg5_m.resume_value;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg1_m.address, 5,
			     buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_cfg_m.resume_value;
	buf[1] = status_registers.int_ths_h.resume_value;
	buf[2] = status_registers.int_ths_l.resume_value;
	err = dev->tf->write(dev->dev, status_registers.int_cfg_m.address, 5,
			     buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&dev->enabled_mag, 1);

	return 0;

err_resume_state:
	atomic_set(&dev->enabled_mag, 0);
	dev_err(dev->dev, "magnetometer hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int lsm9ds1_mag_update_fs_range(struct lsm9ds1_mag_dev *dev,
				       u8 new_fs_range)
{
	int err;
	u32 sensitivity, updated_val;
	u8 buf[1];

	switch (new_fs_range) {
	case LSM9DS1_MAG_FS_4G:
		sensitivity = SENSITIVITY_MAG_4G;
		break;
	case LSM9DS1_MAG_FS_8G:
		sensitivity = SENSITIVITY_MAG_8G;
		break;
	case LSM9DS1_MAG_FS_12G:
		sensitivity = SENSITIVITY_MAG_12G;
		break;
	case LSM9DS1_MAG_FS_16G:
		sensitivity = SENSITIVITY_MAG_16G;
		break;
	default:
		dev_err(dev->dev, "invalid magn fs range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	err = dev->tf->read(dev->dev, status_registers.ctrl_reg2_m.address, 1,
			    buf);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg2_m.resume_value = buf[0];
	updated_val = (LSM9DS1_MAG_FS_MASK & new_fs_range);
	buf[0] = updated_val;

	err = dev->tf->write(dev->dev, status_registers.ctrl_reg2_m.address, 1,
			     buf);
	if (err < 0)
		goto error;
	status_registers.ctrl_reg2_m.resume_value = updated_val;
	dev->sensitivity_mag = sensitivity;

	return err;

error:
	dev_err(dev->dev, "update magn fs range failed 0x%02x: %d\n",
		buf[0], err);
	return err;
}

static int lsm9ds1_mag_update_odr(struct lsm9ds1_mag_dev *dev,
				  unsigned int poll_ms)
{
	int i, err = -1;
	u8 data;

	for (i = ARRAY_SIZE(lsm9ds1_mag_odr_table) - 1; i >= 0; i--) {
		if ((lsm9ds1_mag_odr_table[i].cutoff_us <= poll_ms) ||
		    (i == 0))
			break;
	}

	data = ((ODR_MAG_MASK & lsm9ds1_mag_odr_table[i].value) |
	        (~ODR_MAG_MASK & status_registers.ctrl_reg1_m.resume_value));

	if (atomic_read(&dev->enabled_mag)) {
		err = dev->tf->write(dev->dev,
				     status_registers.ctrl_reg1_m.address,
				     1, &data);
		if (err < 0)
			goto error;
	}
	status_registers.ctrl_reg1_m.resume_value = data;
	dev->ktime_mag = ktime_set(0, MS_TO_NS(poll_ms));

	return err;

error:
	dev_err(dev->dev, "update magnetometer odr failed 0x%02x: %d\n",
		data, err);

	return err;
}

static int lsm9ds1_mag_update_operative_mode(struct lsm9ds1_mag_dev *dev,
					     int axis, u8 value)
{
	int err;
	u8 raddr, waddr, data, mask;

	if (axis == 0) {
		raddr = CTRL_REG1_M;
		mask = X_Y_PERFORMANCE_MASK;
		waddr = CTRL_REG1_M;
	} else {
		raddr = CTRL_REG4_M;
		mask = Z_PERFORMANCE_MASK;
		waddr = CTRL_REG4_M;
	}

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, raddr, 1, &data);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}

	data = ((mask & value) | (~mask & data));

	err = dev->tf->write(dev->dev, waddr, 1, &data);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}

	if (axis == 0)
		dev->xy_mode = value;
	else
		dev->z_mode = value;

	mutex_unlock(&dev->lock);

	return 0;
}

static int lsm9ds1_mag_validate_pdata(struct lsm9ds1_mag_dev *dev)
{
	dev->pdata_mag->min_interval = 
			    max((unsigned int)LSM9DS1_MAG_MIN_POLL_PERIOD_MS,
				dev->pdata_mag->min_interval);
	dev->pdata_mag->poll_interval = max(dev->pdata_mag->poll_interval,
					    dev->pdata_mag->min_interval);

	return 0;
}

int lsm9ds1_mag_enable(struct lsm9ds1_mag_dev *dev)
{
	int err;

	if (!atomic_cmpxchg(&dev->enabled_mag, 0, 1)) {
		mutex_lock(&dev->lock);
		err = lsm9ds1_mag_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled_mag, 0);
			mutex_unlock(&dev->lock);
			return err;
		}
		hrtimer_start(&dev->hr_timer_mag, dev->ktime_mag,
			      HRTIMER_MODE_REL);
		mutex_unlock(&dev->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm9ds1_mag_enable);

int lsm9ds1_mag_disable(struct lsm9ds1_mag_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_mag, 1, 0)) {
		cancel_work_sync(&dev->input_work_mag);
		hrtimer_cancel(&dev->hr_timer_mag);

		mutex_lock(&dev->lock);
		lsm9ds1_mag_device_power_off(dev);
		mutex_unlock(&dev->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm9ds1_mag_disable);

static void lsm9ds1_mag_input_cleanup(struct lsm9ds1_mag_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static ssize_t attr_get_polling_rate_mag(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	unsigned int val;
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);
	mutex_lock(&dev->lock);
	val = dev->pdata_mag->poll_interval;
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate_mag(struct device *device,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;

	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
			    dev->pdata_mag->min_interval);
	mutex_lock(&dev->lock);
	dev->pdata_mag->poll_interval = (unsigned int)interval_ms;
	lsm9ds1_mag_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_enable_mag(struct device *device,
				   struct device_attribute *attr,
				   char *buf)
{
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", (int)atomic_read(&dev->enabled_mag));
}

static ssize_t attr_set_enable_mag(struct device *device,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm9ds1_mag_enable(dev);
	else
		lsm9ds1_mag_disable(dev);

	return size;
}

static ssize_t attr_get_range_mag(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	u8 val;
	int range = 2;
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata_mag->fs_range;
	switch (val) {
	case LSM9DS1_MAG_FS_4G:
		range = 4;
		break;
	case LSM9DS1_MAG_FS_8G:
		range = 8;
		break;
	case LSM9DS1_MAG_FS_12G:
		range = 10;
		break;
	case LSM9DS1_MAG_FS_16G:
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
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 4:
		range = LSM9DS1_MAG_FS_4G;
		break;
	case 8:
		range = LSM9DS1_MAG_FS_8G;
		break;
	case 10:
		range = LSM9DS1_MAG_FS_12G;
		break;
	case 16:
		range = LSM9DS1_MAG_FS_16G;
		break;
	default:
		dev_err(dev->dev, "magnetometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&dev->lock);
	err = lsm9ds1_mag_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata_mag->fs_range = range;
	mutex_unlock(&dev->lock);
	dev_info(dev->dev, "magnetometer range set to:"
							" %lu g\n", val);

	return size;
}

static ssize_t attr_get_xy_mode(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	u8 val;
	char mode[13];
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->xy_mode;
	switch (val) {
	case X_Y_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "high");
		break;
	case X_Y_LOW_PERFORMANCE:
		strcpy(&(mode[0]), "low");
		break;
	case X_Y_MEDIUM_PERFORMANCE:
		strcpy(&(mode[0]), "medium");
		break;
	case X_Y_ULTRA_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "ultra_high");
		break;
	}
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_xy_mode(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err==0) {
		mode = X_Y_HIGH_PERFORMANCE;
		goto valid;
	}

	err = strncmp(buf, "low", 3);
	if (err == 0) {
		mode = X_Y_LOW_PERFORMANCE;
		goto valid;
	}

	err = strncmp(buf, "medium", 6);
	if (err == 0) {
		mode = X_Y_MEDIUM_PERFORMANCE;
		goto valid;
	}

	err = strncmp(buf, "ultra_high", 10);
	if (err == 0) {
		mode = X_Y_ULTRA_HIGH_PERFORMANCE;
		goto valid;
	}
	goto error;

valid:
	err = lsm9ds1_mag_update_operative_mode(dev,0,mode);
	if(err<0)
		goto error;

	dev_info(dev->dev, "magnetometer x_y op. mode set to: %s", buf);
	return size;

error:
	dev_err(dev->dev, "magnetometer invalid value request: %s\n", buf);
	return -EINVAL;
}

static ssize_t attr_get_z_mode(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	u8 val;
	char mode[13];
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->z_mode;
	switch (val) {
	case Z_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "high");
		break;
	case Z_LOW_PERFORMANCE:
		strcpy(&(mode[0]), "low");
		break;
	case Z_MEDIUM_PERFORMANCE:
		strcpy(&(mode[0]), "medium");
		break;
	case Z_ULTRA_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "ultra_high");
		break;
	}
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_z_mode(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm9ds1_mag_dev *dev = dev_get_drvdata(device);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err == 0) {
		mode = Z_HIGH_PERFORMANCE;
		goto valid;
	}

	err = strncmp(buf, "low", 3);
	if (err == 0) {
		mode = Z_LOW_PERFORMANCE;
		goto valid;
	}

	err = strncmp(buf, "medium", 6);
	if (err == 0) {
		mode = Z_MEDIUM_PERFORMANCE;
		goto valid;
	}

	err = strncmp(buf, "ultra_high", 10);
	if (err == 0) {
		mode = Z_ULTRA_HIGH_PERFORMANCE;
		goto valid;
	}
	goto error;

valid:
	err = lsm9ds1_mag_update_operative_mode(dev,1,mode);
	if (err < 0)
		goto error;
	dev_info(dev->dev, "magnetometer z op. mode set to: %s", buf);
	return size;

error:
	dev_err(dev->dev, "magnetometer invalid value request: %s\n", buf);
	return -EINVAL;
}

#ifdef LSM9DS1_DEBUG
static int write_bit_on_register(struct lsm9ds1_mag_dev *dev, u8 address,
				 u8 *resume_value, u8 mask, int value)
{
	int err;
	u8 updated_val, buf[1], val = 0;

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, address, 1, buf);
	if (err < 0) {
		err = -1;
		goto out;
	}

	if (resume_value != NULL)
		*resume_value = buf[0];

	if (mask == 0) {
		updated_val = (u8)value;
	} else {
		if (value > 0)
			val = 0xFF;
		updated_val = (mask & val) | (~mask & buf[0]);
	}

	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, address, 1, buf);
	if (err < 0) {
		err = -1;
		goto out;
	}

	if (resume_value != NULL)
		*resume_value = updated_val;

out:
	mutex_unlock(&dev->lock);
	return err;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_mag,
						attr_set_polling_rate_mag),
	__ATTR(range, 0666, attr_get_range_mag, attr_set_range_mag),
	__ATTR(enable_device, 0666, attr_get_enable_mag, attr_set_enable_mag),
	__ATTR(x_y_opearative_mode, 0666, attr_get_xy_mode, attr_set_xy_mode),
	__ATTR(z_opearative_mode, 0666, attr_get_z_mode, attr_set_z_mode),

};

static int create_sysfs_interfaces(struct device *device)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(device, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(device, attributes + i);
	dev_err(device, "%s:Unable to create interface\n", __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *device)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(device, attributes + i);
}

static int lsm9ds1_mag_get_data(struct lsm9ds1_mag_dev *dev, int *xyz)
{
	int i, err;
	u8 mag_data[6];
	s32 hw_d[3];

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, REG_MAG_DATA_ADDR, 6, mag_data);
	mutex_unlock(&dev->lock);

	if (err < 0)
		return err;

	hw_d[0] = ((s32)((s16)((mag_data[1] << 8) | (mag_data[0]))));
	hw_d[1] = ((s32)((s16)((mag_data[3] << 8) | (mag_data[2]))));
	hw_d[2] = ((s32)((s16)((mag_data[5] << 8) | (mag_data[4]))));

#ifdef LSM9DS1_DEBUG
	pr_debug("%s read x=0x%02x 0x%02x (regH regL), x=%d (dec) [LSB]\n",
		 LSM9DS1_MAG_DEV_NAME, mag_data[1], mag_data[0], hw_d[0]);
	pr_debug("%s read y=0x%02x 0x%02x (regH regL), y=%d (dec) [LSB]\n",
		 LSM9DS1_MAG_DEV_NAME, mag_data[3], mag_data[2], hw_d[1]);
	pr_debug("%s read z=0x%02x 0x%02x (regH regL), z=%d (dec) [LSB]\n",
		 LSM9DS1_MAG_DEV_NAME, mag_data[5], mag_data[4], hw_d[2]);
#endif

	hw_d[0] = hw_d[0] * dev->sensitivity_mag;
	hw_d[1] = hw_d[1] * dev->sensitivity_mag;
	hw_d[2] = hw_d[2] * dev->sensitivity_mag;

	for (i = 0; i < 3; i++) {
		xyz[i] = dev->pdata_mag->rot_matrix[0][i] * hw_d[0] +
			 dev->pdata_mag->rot_matrix[1][i] * hw_d[1] +
			 dev->pdata_mag->rot_matrix[2][i] * hw_d[2];
	}

	return err;
}

static void lsm9ds1_mag_report_values(struct lsm9ds1_mag_dev *dev, int *xyz,
				      s64 timestamp)
{
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(dev->input_dev_mag);
}

static int lsm9ds1_mag_input_init(struct lsm9ds1_mag_dev *dev)
{
	int err;

	dev->input_dev_mag = input_allocate_device();
	if (!dev->input_dev_mag) {
		dev_err(dev->dev, "acc input dev allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev_mag->name = LSM9DS1_MAG_DEV_NAME;
	dev->input_dev_mag->id.bustype = dev->bus_type;
	dev->input_dev_mag->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_mag, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev_mag->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_TIME_MSB, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, dev->input_dev_mag->mscbit);

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

static void lsm9ds1_input_cleanup(struct lsm9ds1_mag_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static void poll_function_work_mag(struct work_struct *input_work_mag)
{
	struct lsm9ds1_mag_dev *dev;
	int xyz[3] = { 0 };
	int err;

	dev = container_of((struct work_struct *)input_work_mag,
			   struct lsm9ds1_mag_dev, input_work_mag);

	if (atomic_read(&dev->enabled_mag)) {
		err = lsm9ds1_mag_get_data(dev, xyz);
		if (err < 0)
			dev_err(dev->dev, "get_magnetometer_data failed\n");
		else
			lsm9ds1_mag_report_values(dev, xyz,
						  lsm9ds1_get_time_ns());
	}

	hrtimer_start(&dev->hr_timer_mag, dev->ktime_mag, HRTIMER_MODE_REL);
}

static enum hrtimer_restart poll_function_read_mag(struct hrtimer *timer)
{
	struct lsm9ds1_mag_dev *dev;

	dev = container_of((struct hrtimer *)timer,
			   struct lsm9ds1_mag_dev, hr_timer_mag);
	queue_work(dev->mag_workqueue, &dev->input_work_mag);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_OF
static int lsm9ds1_mag_parse_dt(struct lsm9ds1_mag_dev *dev,
			        struct device* device)
{
	u8 i, j;
	u32 val, vect[9];
	struct device_node *dn;

	if (of_match_device(dev->mag_dt_id, device)) {
		dn = device->of_node;
		dev->pdata_mag->of_node = dn;
		
		dev->pdata_mag->gpio_int_m = of_get_gpio(dn, 0);
		if (!gpio_is_valid(dev->pdata_mag->gpio_int_m)) {
			dev_err(dev->dev, "failed to get gpio_int_m\n");
			dev->pdata_mag->gpio_int_m = LSM9DS1_INT_M_GPIO_DEF;
		}

		if (of_property_read_u32_array(dn, "rot-matrix", vect,
					       ARRAY_SIZE(vect)) >= 0) {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					dev->pdata_mag->rot_matrix[i][j] =
						(short)vect[3 * j + i];
				}
			}
		} else {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					dev->pdata_mag->rot_matrix[i][j] =
				default_lsm9ds1_mag_pdata.rot_matrix[i][j];
				}
			}
		}

		if (!of_property_read_u32(dn, "poll-interval", &val)) {
			dev->pdata_mag->poll_interval = val;
		} else {
			dev->pdata_mag->poll_interval =
				LSM9DS1_M_POLL_INTERVAL_DEF;
		}

		if (!of_property_read_u32(dn, "min-interval", &val)) {
			dev->pdata_mag->min_interval = val;
		} else {
			dev->pdata_mag->min_interval =
				LSM9DS1_MAG_MIN_POLL_PERIOD_MS;
		}

		if (!of_property_read_u32(dn, "fs-range", &val)) {
			dev->pdata_mag->fs_range = val;
		} else {
			dev->pdata_mag->fs_range = LSM9DS1_MAG_FS_4G;
		}
		return 0;
	}
	return -1;
}
#endif

int lsm9ds1_mag_probe(struct lsm9ds1_mag_dev *dev)
{
	int err = -1;

	mutex_lock(&dev->lock);

	dev->pdata_mag = kzalloc(sizeof(*dev->pdata_mag), GFP_KERNEL);
	if(dev->pdata_mag == NULL) {
		err = -ENOMEM;
		dev_err(dev->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

#ifdef CONFIG_OF
	lsm9ds1_mag_parse_dt(dev, dev->dev);
#else
	if (dev->dev->platform_data == NULL) {
		memcpy(dev->pdata_mag, &default_lsm9ds1_mag_pdata,
		       sizeof(*dev->pdata_mag));
	}
	else {
		memcpy(dev->pdata_mag, dev->dev->platform_data,
		       sizeof(*dev->pdata_mag));
	}
#endif

	err = lsm9ds1_mag_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate pdata for magn\n");
		goto exit_kfree_pdata;
	}

	if (dev->pdata_mag->init) {
		err = dev->pdata_mag->init();
		if (err < 0) {
			dev_err(dev->dev, "magnetometer init failed: %d\n",
				err);
			goto err_pdata_mag_init;
		}
	}

	err = lsm9ds1_hw_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm9ds1_mag_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "magnetometer power on failed: %d\n",
			err);
		goto err_pdata_init;
	}

	err = lsm9ds1_mag_update_fs_range(dev, dev->pdata_mag->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range on magnetometer failed\n");
		goto  err_power_off_mag;
	}

	err = lsm9ds1_mag_update_odr(dev, dev->pdata_mag->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr on magnetometer failed\n");
		goto  err_power_off;
	}

	err = lsm9ds1_mag_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "magnetometer input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "device %s sysfs register failed\n",
			LSM9DS1_MAG_DEV_NAME);
		goto err_input_cleanup;
	}

	lsm9ds1_mag_device_power_off(dev);

	dev->mag_workqueue = create_workqueue("lsm9ds1_workqueue");

	if (!dev->mag_workqueue)
		goto err_input_cleanup;

	hrtimer_init(&dev->hr_timer_mag, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer_mag.function = &poll_function_read_mag;
	INIT_WORK(&dev->input_work_mag, poll_function_work_mag);

	mutex_unlock(&dev->lock);

	return 0;

err_input_cleanup:
	lsm9ds1_input_cleanup(dev);
err_power_off:
err_power_off_mag:
	lsm9ds1_mag_device_power_off(dev);
err_hw_init:
err_pdata_init:
err_pdata_mag_init:
	if (dev->pdata_mag->exit)
		dev->pdata_mag->exit();
exit_kfree_pdata:
	kfree(dev->pdata_mag);
err_mutexunlock:
	mutex_unlock(&dev->lock);
	if (dev->mag_workqueue)
		destroy_workqueue(dev->mag_workqueue);

	return err;
}
EXPORT_SYMBOL(lsm9ds1_mag_probe);

int lsm9ds1_mag_remove(struct lsm9ds1_mag_dev *dev)
{
	lsm9ds1_mag_disable(dev);
	lsm9ds1_mag_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	if (dev->pdata_mag->exit)
		dev->pdata_mag->exit();

	if (dev->mag_workqueue)
		destroy_workqueue(dev->mag_workqueue);

	kfree(dev->pdata_mag);
	return 0;
}
EXPORT_SYMBOL(lsm9ds1_mag_remove);

MODULE_DESCRIPTION("lsm9ds1 magnetometer driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");
