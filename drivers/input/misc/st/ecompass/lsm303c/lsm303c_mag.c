/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lsm303c_mag.c
* Authors            : MSH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
*		     : Lorenzo Bianconi (lorenzo.bianconi@st.com)
*		     : Authors are willing to be considered the contact
*		     : and update points for the driver.
* Version            : V.1.0.3
* Date               : 2016/May/18
* Description        : LSM303C magnetometer driver
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
********************************************************************************
*******************************************************************************/

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

#include "lsm303c.h"


#define MS_TO_NS(x)		(x*1000000L)

/* Address registers */
#define REG_WHOAMI_ADDR		(0x0F)	/** Who am i address register */
#define REG_CNTRL1_ADDR		(0x20)	/** CNTRL1 address register */
#define REG_CNTRL2_ADDR		(0x21)	/** CNTRL2 address register */
#define REG_CNTRL3_ADDR		(0x22)	/** CNTRL3 address register */
#define REG_CNTRL4_ADDR		(0x23)	/** CNTRL4 address register */
#define REG_CNTRL5_ADDR		(0x24)	/** CNTRL5 address register */

#define REG_MAG_DATA_ADDR	(0x28)	/** Mag. data low address register */

/* Sensitivity */
#define SENSITIVITY_MAG_4G	146156	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_8G	292312	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_10G	365364	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_16G	584454	/**	ngauss/LSB	*/

/* ODR */
#define ODR_MAG_MASK		(0X1C)	/* Mask for odr change on mag */
#define LSM303C_MAG_ODR0_625	(0x00)	/* 0.625Hz output data rate */
#define LSM303C_MAG_ODR1_25	(0x04)	/* 1.25Hz output data rate */
#define LSM303C_MAG_ODR2_5	(0x08)	/* 2.5Hz output data rate */
#define LSM303C_MAG_ODR5	(0x0C)	/* 5Hz output data rate */
#define LSM303C_MAG_ODR10	(0x10)	/* 10Hz output data rate */
#define LSM303C_MAG_ODR20	(0x14)	/* 20Hz output data rate */
#define LSM303C_MAG_ODR40	(0x18)	/* 40Hz output data rate */
#define LSM303C_MAG_ODR80	(0x1C)	/* 80Hz output data rate */

/* Magnetic sensor mode */
#define MSMS_MASK		(0x03)	/* Mask magnetic sensor mode */
#define POWEROFF_MAG		(0x02)	/* Power Down */
#define CONTINUOS_CONVERSION	(0x00)	/* Continuos Conversion */

/* X and Y axis operative mode selection */
#define X_Y_PERFORMANCE_MASK		(0x60)
#define X_Y_LOW_PERFORMANCE		(0x00)
#define X_Y_MEDIUM_PERFORMANCE		(0x20)
#define X_Y_HIGH_PERFORMANCE		(0x40)
#define X_Y_ULTRA_HIGH_PERFORMANCE	(0x60)

/* Z axis operative mode selection */
#define Z_PERFORMANCE_MASK		(0x0c)
#define Z_LOW_PERFORMANCE		(0x00)
#define Z_MEDIUM_PERFORMANCE		(0x04)
#define Z_HIGH_PERFORMANCE		(0x08)
#define Z_ULTRA_HIGH_PERFORMANCE	(0x0c)

/* Default values loaded in probe function */
#define WHOIAM_VALUE		(0x3d)	/** Who Am I default value */
#define REG_DEF_CNTRL1		(0x60)	/** CNTRL1 default value */
#define REG_DEF_CNTRL2		(0x00)	/** CNTRL2 default value */
#define REG_DEF_CNTRL3		(0x03)	/** CNTRL3 default value */
#define REG_DEF_CNTRL4		(0x00)	/** CNTRL4 default value */
#define REG_DEF_CNTRL5		(0x40)	/** CNTRL5 default value */

#define REG_DEF_ALL_ZEROS	(0x00)

struct {
	unsigned int cutoff_us;
	u8 value;
} lsm303c_mag_odr_table[] = {
	{  12, LSM303C_MAG_ODR80  },
	{  25, LSM303C_MAG_ODR40   },
	{  50, LSM303C_MAG_ODR20   },
	{  100, LSM303C_MAG_ODR10 },
	{ 200, LSM303C_MAG_ODR5 },
	{ 400, LSM303C_MAG_ODR2_5},
	{ 800, LSM303C_MAG_ODR1_25},
	{ 1600, LSM303C_MAG_ODR0_625},
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

static const struct lsm303c_mag_platform_data default_lsm303c_mag_pdata = {
	.poll_interval = 100,
	.min_interval = LSM303C_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = LSM303C_MAG_FS_4G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
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
	struct reg_rw cntrl1;
	struct reg_rw cntrl2;
	struct reg_rw cntrl3;
	struct reg_rw cntrl4;
	struct reg_rw cntrl5;
} status_registers = {
	.who_am_i.address = REG_WHOAMI_ADDR,
	.who_am_i.value = WHOIAM_VALUE,
	.cntrl1.address = REG_CNTRL1_ADDR,
	.cntrl1.default_value = REG_DEF_CNTRL1,
	.cntrl2.address = REG_CNTRL2_ADDR,
	.cntrl2.default_value = REG_DEF_CNTRL2,
	.cntrl3.address = REG_CNTRL3_ADDR,
	.cntrl3.default_value = REG_DEF_CNTRL3,
	.cntrl4.address = REG_CNTRL4_ADDR,
	.cntrl4.default_value = REG_DEF_CNTRL4,
	.cntrl5.address = REG_CNTRL5_ADDR,
	.cntrl5.default_value = REG_DEF_CNTRL5,
};

static int lsm303c_hw_init(struct lsm303c_mag_dev *dev)
{
	int err = -1;
	u8 buf[5];

#ifdef LSM303C_DEBUG
	pr_info("%s: hw init start\n", LSM303C_MAG_DEV_NAME);
#endif
	err = dev->tf->read(dev->dev, status_registers.who_am_i.address, 1,
			    buf);
	if (err < 0) {
		dev_warn(dev->dev,
		"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		dev->hw_working = 1;

	if (buf[0] != status_registers.who_am_i.value) {
		dev_err(dev->dev,
		"device unknown. Expected: 0x%02x, Replies: 0x%02x\n",
				status_registers.who_am_i.value, buf[0]);
		err = -1;
		goto err_unknown_device;
	}

	status_registers.cntrl1.resume_value =
					status_registers.cntrl1.default_value;
	status_registers.cntrl2.resume_value =
					status_registers.cntrl2.default_value;
	status_registers.cntrl3.resume_value =
					status_registers.cntrl3.default_value;
	status_registers.cntrl4.resume_value =
					status_registers.cntrl4.default_value;
	status_registers.cntrl5.resume_value =
					status_registers.cntrl5.default_value;

	buf[0] = status_registers.cntrl1.default_value;
	buf[1] = status_registers.cntrl2.default_value;
	buf[2] = status_registers.cntrl3.default_value;
	buf[3] = status_registers.cntrl4.default_value;
	buf[4] = status_registers.cntrl5.default_value;
	err = dev->tf->write(dev->dev, status_registers.cntrl1.address, 5,
			     buf);
	if (err < 0) {
		dev_warn(dev->dev,
		"Error initializing CLTR_REG registers\n");
		goto err_reginit;
	}

	dev->xy_mode = X_Y_ULTRA_HIGH_PERFORMANCE;
	dev->z_mode = Z_ULTRA_HIGH_PERFORMANCE;
	dev->hw_initialized = 1;

#ifdef LSM303C_DEBUG
	pr_info("%s: hw init done\n", LSM303C_MAG_DEV_NAME);
#endif

	return 0;

err_reginit:
err_unknown_device:
err_firstread:
	dev->hw_working = 0;
	dev->hw_initialized = 0;
	return err;
}

static int lsm303c_mag_device_power_off(struct lsm303c_mag_dev *dev)
{
	int err;
	u8 buf[1];

	buf[0] = ((MSMS_MASK & POWEROFF_MAG) |
		  (~MSMS_MASK & status_registers.cntrl3.resume_value));

	err = dev->tf->write(dev->dev, status_registers.cntrl3.address, 1,
			     buf);
	if (err < 0)
		dev_err(dev->dev,
			"magnetometer soft power off failed: %d\n", err);

	if (dev->pdata_mag->power_off)
		dev->pdata_mag->power_off();

	atomic_set(&dev->enabled_mag, 0);

	return 0;
}

static int lsm303c_mag_device_power_on(struct lsm303c_mag_dev *dev)
{
	int err = -1;
	u8 buf[1];

	if (dev->pdata_mag->power_on) {
		err = dev->pdata_mag->power_on();
		if (err < 0) {
			dev_err(dev->dev,
				"magnetometer power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.cntrl1.resume_value;
	err = dev->tf->write(dev->dev, status_registers.cntrl1.address, 1,
			     buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = ((MSMS_MASK & CONTINUOS_CONVERSION) |
		  (~MSMS_MASK & status_registers.cntrl3.resume_value));
	err = dev->tf->write(dev->dev, status_registers.cntrl3.address, 1,
			     buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&dev->enabled_mag, 1);

	return 0;

err_resume_state:
	atomic_set(&dev->enabled_mag, 0);
	dev_err(dev->dev, "magnetometer hw power on error 0x%02x: %d\n",
		buf[0], err);
	return err;
}

static int lsm303c_mag_update_fs_range(struct lsm303c_mag_dev *dev,
				       u8 new_fs_range)
{
	int err = -1;
	u32 sensitivity;
	u8 updated_val;
	u8 buf[1];

	switch (new_fs_range) {
	case LSM303C_MAG_FS_4G:
		sensitivity = SENSITIVITY_MAG_4G;
		break;
	case LSM303C_MAG_FS_8G:
		sensitivity = SENSITIVITY_MAG_8G;
		break;
	case LSM303C_MAG_FS_10G:
		sensitivity = SENSITIVITY_MAG_10G;
		break;
	case LSM303C_MAG_FS_16G:
		sensitivity = SENSITIVITY_MAG_16G;
		break;
	default:
		dev_err(dev->dev,
			"invalid magnetometer fs range requested: %u\n",
								new_fs_range);
		return -EINVAL;
	}

	err = dev->tf->read(dev->dev, status_registers.cntrl2.address, 1, buf);
	if (err < 0)
		goto error;

	status_registers.cntrl2.resume_value = buf[0];
	updated_val = (LSM303C_MAG_FS_MASK & new_fs_range);

	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.cntrl2.address, 1,
			     buf);
	if (err < 0)
		goto error;
	status_registers.cntrl2.resume_value = updated_val;
	dev->sensitivity_mag = sensitivity;

	return err;

error:
	dev_err(dev->dev,
		"update magnetometer fs range failed 0x%02x: %d\n",
		buf[0], err);
	return err;
}

static int lsm303c_mag_update_odr(struct lsm303c_mag_dev *dev,
				  unsigned int poll_interval_ms)
{
	int err = 0;
	u8 config[1];
	int i;

	for (i = ARRAY_SIZE(lsm303c_mag_odr_table) - 1; i >= 0; i--) {
		if ((lsm303c_mag_odr_table[i].cutoff_us <= poll_interval_ms) ||
		    (i == 0))
			break;
	}

	config[0] = ((ODR_MAG_MASK & lsm303c_mag_odr_table[i].value) |
		     (~ODR_MAG_MASK & status_registers.cntrl1.resume_value));

	if (atomic_read(&dev->enabled_mag)) {
		err = dev->tf->write(dev->dev, status_registers.cntrl1.address, 1,
				     config);
		if (err < 0)
			goto error;
	}
	status_registers.cntrl1.resume_value = config[0];
	dev->ktime_mag = ktime_set(0, MS_TO_NS(poll_interval_ms));

	return err;

error:
	dev_err(dev->dev, "update magnetometer odr failed 0x%02x: %d\n",
		config[0], err);

	return err;
}

static int lsm303c_mag_update_operative_mode(struct lsm303c_mag_dev *dev,
					     int axis, u8 value)
{
	int err = -1;
	u8 raddr, mask, waddr, data;

	if (axis == 0) {
		raddr = REG_CNTRL1_ADDR;
		mask = X_Y_PERFORMANCE_MASK;
		waddr = REG_CNTRL1_ADDR;
	} else {
		raddr = REG_CNTRL4_ADDR;
		mask = Z_PERFORMANCE_MASK;
		waddr = REG_CNTRL4_ADDR;
	}

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, raddr, 1, &data);
	if (err < 0)
		goto error;

	data = ((mask & value) | (~mask & data));
	err = dev->tf->write(dev->dev, waddr, 1, &data);
	if (err < 0)
		goto error;
	if (axis == 0)
		dev->xy_mode = value;
	else
		dev->z_mode = value;

	mutex_unlock(&dev->lock);

	return err;

error:
	mutex_unlock(&dev->lock);

	dev_err(dev->dev, "update operative mode failed 0x%02x: %d\n",
		data, err);

	return err;
}

static int lsm303c_validate_polling(unsigned int *min_interval,
					unsigned int *poll_interval,
					unsigned int min, u8 *axis_map_x,
					u8 *axis_map_y, u8 *axis_map_z)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);

	if (*axis_map_x > 2 || *axis_map_y > 2 || *axis_map_z > 2) {
		return -EINVAL;
	}

	return 0;
}

static int lsm303c_validate_negate(u8 *negate_x, u8 *negate_y, u8 *negate_z)
{
	if (*negate_x > 1 || *negate_y > 1 || *negate_z > 1) {
		return -EINVAL;
	}
	return 0;
}

static int lsm303c_mag_validate_pdata(struct lsm303c_mag_dev *dev)
{
	int res = -1;

	res = lsm303c_validate_polling(&dev->pdata_mag->min_interval,
				&dev->pdata_mag->poll_interval,
				(unsigned int)LSM303C_MAG_MIN_POLL_PERIOD_MS,
				&dev->pdata_mag->axis_map_x,
				&dev->pdata_mag->axis_map_y,
				&dev->pdata_mag->axis_map_z);
	if (res < 0)
		return -EINVAL;

	res = lsm303c_validate_negate(&dev->pdata_mag->negate_x,
				&dev->pdata_mag->negate_y,
				&dev->pdata_mag->negate_z);
	if (res < 0)
		return -EINVAL;

	return 0;
}

int lsm303c_mag_enable(struct lsm303c_mag_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled_mag, 0, 1)) {
		int err;

		mutex_lock(&dev->lock);

		err = lsm303c_mag_device_power_on(dev);
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
EXPORT_SYMBOL(lsm303c_mag_enable);

int lsm303c_mag_disable(struct lsm303c_mag_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_mag, 1, 0)) {
		cancel_work_sync(&dev->input_work_mag);
		hrtimer_cancel(&dev->hr_timer_mag);

		mutex_lock(&dev->lock);
		lsm303c_mag_device_power_off(dev);
		mutex_unlock(&dev->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm303c_mag_disable);

static void lsm303c_mag_input_cleanup(struct lsm303c_mag_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static ssize_t attr_get_polling_rate_mag(struct device *device,
						struct device_attribute *attr,
		char *buf)
{
	unsigned int val;
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata_mag->poll_interval;
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate_mag(struct device *device,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
			    dev->pdata_mag->min_interval);

	mutex_lock(&dev->lock);
	dev->pdata_mag->poll_interval = (unsigned int)interval_ms;
	err = lsm303c_mag_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return (!err) ? size : err;
}

static ssize_t attr_get_enable_mag(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);
	int val = (int)atomic_read(&dev->enabled_mag);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_mag(struct device *device,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303c_mag_enable(dev);
	else
		lsm303c_mag_disable(dev);

	return size;
}

static ssize_t attr_get_range_mag(struct device *device,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	int range = 2;
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata_mag->fs_range;
	switch (val) {
	case LSM303C_MAG_FS_4G:
		range = 4;
		break;
	case LSM303C_MAG_FS_8G:
		range = 8;
		break;
	case LSM303C_MAG_FS_10G:
		range = 10;
		break;
	case LSM303C_MAG_FS_16G:
		range = 16;
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_mag(struct device *device,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 4:
		range = LSM303C_MAG_FS_4G;
		break;
	case 8:
		range = LSM303C_MAG_FS_8G;
		break;
	case 10:
		range = LSM303C_MAG_FS_10G;
		break;
	case 16:
		range = LSM303C_MAG_FS_16G;
		break;
	default:
		dev_err(dev->dev,
			"magnetometer invalid range request: %lu, discarded\n",
									val);
		return -EINVAL;
	}
	mutex_lock(&dev->lock);
	err = lsm303c_mag_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata_mag->fs_range = range;
	mutex_unlock(&dev->lock);
	dev_info(dev->dev,
				"magnetometer range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_xy_mode(struct device *device,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	char mode[13];
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);

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
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err == 0) {
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
	err = lsm303c_mag_update_operative_mode(dev, 0, mode);
	if (err < 0)
		goto error;

	dev_info(dev->dev, "magnetometer x_y op. mode set to: %s", buf);
	return size;

error:
	dev_err(dev->dev,
		"magnetometer invalid value request: %s, discarded\n", buf);

	return -EINVAL;
}

static ssize_t attr_get_z_mode(struct device *device,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	char mode[13];
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);

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
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_mag_dev *dev = dev_get_drvdata(device);
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
	err = lsm303c_mag_update_operative_mode(dev, 1, mode);
	if (err < 0)
		goto error;

	dev_info(dev->dev,
			"magnetometer z op. mode set to: %s", buf);
	return size;

error:
	dev_err(dev->dev,
		"magnetometer invalid value request: %s, discarded\n", buf);

	return -EINVAL;
}

#ifdef DEBUG
static int write_bit_on_register(struct lsm303c_mag_dev *dev, u8 address,
				 u8 *resume_value, u8 mask, int value)
{
	int err;
	u8 updated_val;
	u8 buf[1];
	u8 val = 0x00;

	mutex_lock(&dev->lock);

	err = dev->tf->read(dev->dev, address, 1, buf);
	if (err < 0) {
		mutex_lock(&dev->lock);
		return -1;
	}

	if (resume_value != NULL)
		*resume_value = buf[0];

	if (mask == 0)
		updated_val = (u8)value;
	else {
		if (value > 0)
			val = 0xFF;

		updated_val = (mask & val) | ((~mask) & buf[0]);
	}

	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, address, 1, buf);
	if (err < 0) {
		mutex_lock(&dev->lock);
		return -1;
	}

	if (resume_value != NULL)
		*resume_value = updated_val;

	return err;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0664, attr_get_polling_rate_mag,
						attr_set_polling_rate_mag),
	__ATTR(full_scale, 0664, attr_get_range_mag, attr_set_range_mag),
	__ATTR(enable_device, 0664, attr_get_enable_mag, attr_set_enable_mag),
	__ATTR(x_y_opearative_mode, 0664, attr_get_xy_mode, attr_set_xy_mode),
	__ATTR(z_opearative_mode, 0664, attr_get_z_mode, attr_set_z_mode),
#ifdef DEBUG
	//__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	//__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
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
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int lsm303c_mag_get_data(struct lsm303c_mag_dev *dev, int *xyz)
{
	int err = -1;
	u8 mag_data[6];
	s32 hw_d[3] = { 0 };

	err = dev->tf->read(dev->dev, REG_MAG_DATA_ADDR, 6, mag_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s32)((s16)((mag_data[1] << 8) | (mag_data[0]))));
	hw_d[1] = ((s32)((s16)((mag_data[3] << 8) | (mag_data[2]))));
	hw_d[2] = ((s32)((s16)((mag_data[5] << 8) | (mag_data[4]))));

#ifdef DEBUG
	pr_debug("%s read x=0x%02x 0x%02x (regH regL), x=%d (dec) [LSB]\n",
		LSM303C_MAG_DEV_NAME, mag_data[1], mag_data[0], hw_d[0]);
	pr_debug("%s read y=0x%02x 0x%02x (regH regL), y=%d (dec) [LSB]\n",
		LSM303C_MAG_DEV_NAME, mag_data[3], mag_data[2], hw_d[1]);
	pr_debug("%s read z=0x%02x 0x%02x (regH regL), z=%d (dec) [LSB]\n",
		LSM303C_MAG_DEV_NAME, mag_data[5], mag_data[4], hw_d[2]);
#endif
/*
	hw_d[0] = hw_d[0] * dev->sensitivity_mag;
	hw_d[1] = hw_d[1] * dev->sensitivity_mag;
	hw_d[2] = hw_d[2] * dev->sensitivity_mag;
*/
	xyz[0] = ((dev->pdata_mag->negate_x) ?
				(-hw_d[dev->pdata_mag->axis_map_x])
					: (hw_d[dev->pdata_mag->axis_map_x]));
	xyz[1] = ((dev->pdata_mag->negate_y) ?
				(-hw_d[dev->pdata_mag->axis_map_y])
					: (hw_d[dev->pdata_mag->axis_map_y]));
	xyz[2] = ((dev->pdata_mag->negate_z) ?
				(-hw_d[dev->pdata_mag->axis_map_z])
					: (hw_d[dev->pdata_mag->axis_map_z]));

	return err;
}

static void lsm303c_mag_report_values(struct lsm303c_mag_dev *dev, int *xyz)
{
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_sync(dev->input_dev_mag);
}

static int lsm303c_mag_input_init(struct lsm303c_mag_dev *dev)
{
	int err;

	dev->input_dev_mag = input_allocate_device();
	if (!dev->input_dev_mag) {
		err = -ENOMEM;
		dev_err(dev->dev,
			"magnetometer input device allocation failed\n");
		goto err0;
	}

	dev->input_dev_mag->name = LSM303C_MAG_DEV_NAME;
	dev->input_dev_mag->id.bustype = dev->bus_type;
	dev->input_dev_mag->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_mag, dev);

	/* Set the input event characteristics of the probed sensor driver */
	set_bit(INPUT_EVENT_TYPE, dev->input_dev_mag->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev_mag->mscbit);

	err = input_register_device(dev->input_dev_mag);
	if (err) {
		dev_err(dev->dev,
			"unable to register magnetometer input device %s\n",
				dev->input_dev_mag->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(dev->input_dev_mag);
err0:
	return err;
}

static void lsm303c_input_cleanup(struct lsm303c_mag_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static void poll_function_work_mag(struct work_struct *input_work_mag)
{
	struct lsm303c_mag_dev *dev;
	int xyz[3] = { 0 };
	int err;

	dev = container_of((struct work_struct *)input_work_mag,
			   struct lsm303c_mag_dev, input_work_mag);

	mutex_lock(&dev->lock);

	if (atomic_read(&dev->enabled_mag)) {
		err = lsm303c_mag_get_data(dev, xyz);
		if (err < 0)
			dev_err(dev->dev,
					"get_magnetometer_data failed\n");
		else
			lsm303c_mag_report_values(dev, xyz);
	}

	mutex_unlock(&dev->lock);
	hrtimer_start(&dev->hr_timer_mag, dev->ktime_mag, HRTIMER_MODE_REL);
}

static enum hrtimer_restart poll_function_read_mag(struct hrtimer *timer)
{
	struct lsm303c_mag_dev *dev;


	dev = container_of((struct hrtimer *)timer,
			   struct lsm303c_mag_dev, hr_timer_mag);

	queue_work(dev->mag_workqueue, &dev->input_work_mag);
	return HRTIMER_NORESTART;
}

int lsm303c_mag_probe(struct lsm303c_mag_dev *dev)
{
	int err = -1;

	mutex_lock(&dev->lock);

	dev->pdata_mag = kmalloc(sizeof(*dev->pdata_mag), GFP_KERNEL);
	if (dev->pdata_mag == NULL) {
		err = -ENOMEM;
		dev_err(dev->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (dev->dev->platform_data == NULL) {
		memcpy(dev->pdata_mag, &default_lsm303c_mag_pdata,
						sizeof(*dev->pdata_mag));
		dev_info(dev->dev,
			"using default plaform_data for magnetometer\n");
	} else {
		memcpy(dev->pdata_mag, dev->dev->platform_data,
						sizeof(*dev->pdata_mag));
	}

	err = lsm303c_mag_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev,
			"failed to validate platform data for magnetometer\n");
		goto exit_kfree_pdata;
	}

	if (dev->pdata_mag->init) {
		err = dev->pdata_mag->init();
		if (err < 0) {
			dev_err(dev->dev,
				"magnetometer init failed: %d\n", err);
			goto err_pdata_mag_init;
		}
	}

	err = lsm303c_hw_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm303c_mag_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev,
			"magnetometer power on failed: %d\n", err);
		goto err_pdata_init;
	}

	err = lsm303c_mag_update_fs_range(dev, dev->pdata_mag->fs_range);
	if (err < 0) {
		dev_err(dev->dev,
			"update_fs_range on magnetometer failed\n");
		goto  err_power_off_mag;
	}

	err = lsm303c_mag_update_odr(dev, dev->pdata_mag->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr on magnetometer failed\n");
		goto  err_power_off;
	}

	err = lsm303c_mag_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "magnetometer input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev,
			"device LSM303C_MAG_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm303c_mag_device_power_off(dev);

	dev->mag_workqueue = create_workqueue("lsm303c_workqueue");
	if (!dev->mag_workqueue)
		goto err_input_cleanup;

	hrtimer_init(&dev->hr_timer_mag, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer_mag.function = &poll_function_read_mag;
	INIT_WORK(&dev->input_work_mag, poll_function_work_mag);

	mutex_unlock(&dev->lock);

	return 0;

err_input_cleanup:
	lsm303c_input_cleanup(dev);
err_power_off:
err_power_off_mag:
	lsm303c_mag_device_power_off(dev);
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
EXPORT_SYMBOL(lsm303c_mag_probe);

int lsm303c_mag_remove(struct lsm303c_mag_dev *dev)
{
	lsm303c_mag_disable(dev);
	lsm303c_mag_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	if (dev->pdata_mag->exit)
		dev->pdata_mag->exit();

	if (dev->mag_workqueue)
		destroy_workqueue(dev->mag_workqueue);

	kfree(dev->pdata_mag);

	return 0;
}
EXPORT_SYMBOL(lsm303c_mag_remove);

MODULE_DESCRIPTION("lsm303c magnetometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");
