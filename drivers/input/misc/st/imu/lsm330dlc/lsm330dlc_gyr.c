/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: lsm330dlc_gyr.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.5.5 sysfs
* Date			: 2016/May/10
* Description		: LSM330DLC digital output gyroscope sensor API
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
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS	  | DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone | First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno	  | Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno	  | SMB Bus Mng,
*		|		|		  | forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno	  | Introduces FIFO Feat.
* 1.1.5.2	| 2011/Nov/11	| Matteo Dameno	  | enable gpio_int to be
*		|		|		  | passed as parameter at
*		|		|		  | module loading time;
*		|		|		  | corrects polling
*		|		|		  | bug at end of probing;
* 1.1.5.3	| 2011/Dec/20	| Matteo Dameno	  | corrects error in
*		|		|		  | I2C SADROOT; Modifies
*		|		|		  | resume suspend func.
* 1.1.5.4	| 2012/Jan/09	| Matteo Dameno	  | moved under input/misc;
* 1.1.5.5	| 2012/Mar/30	| Matteo Dameno	  | moved watermark, use_smbus,
*		|		|		  | fifomode @ struct foo_status
*		|		|		  | sysfs range input format
*		|		|		  | changed to decimal
*******************************************************************************/

#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include "lsm330dlc.h"

#define WHO_AM_I	0x0F

#define CTRL_REG1	0x20 /* CTRL REG1 */
#define CTRL_REG2	0x21 /* CTRL REG2 */
#define CTRL_REG3	0x22 /* CTRL_REG3 */
#define CTRL_REG4	0x23 /* CTRL_REG4 */
#define CTRL_REG5	0x24 /* CTRL_REG5 */
#define REFERENCE	0x25 /* REFERENCE REG */
#define FIFO_CTRL_REG	0x2E /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	0x2F /* FIFO SOURCE REGISTER */
#define OUT_X_L		0x28 /* 1st AXIS OUT REG of 6 */

#define AXISDATA_REG	OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES		0x00
#define PM_OFF			0x00
#define PM_NORMAL		0x08
#define ENABLE_ALL_AXES		0x07
#define ENABLE_NO_AXES		0x00
#define BW00			0x00
#define BW01			0x10
#define BW10			0x20
#define BW11			0x30
#define ODR095			0x00 /* ODR =  95Hz */
#define ODR190			0x40 /* ODR = 190Hz */
#define ODR380			0x80 /* ODR = 380Hz */
#define ODR760			0xC0 /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define I2_DRDY		0x08
#define I2_WTM		0x04
#define I2_OVRUN	0x02
#define I2_EMPTY	0x01
#define I2_NONE		0x00
#define I2_MASK		0x0F

/* CTRL_REG4 bits */
#define FS_MASK		0x30
#define BDU_ENABLE	0x80

/* CTRL_REG5 bits */
#define FIFO_ENABLE	0x40
#define HPF_ENALBE	0x11

/* FIFO_CTRL_REG bits */
#define FIFO_MODE_MASK		0xE0
#define FIFO_MODE_BYPASS	0x00
#define FIFO_MODE_FIFO		0x20
#define FIFO_MODE_STREAM	0x40
#define FIFO_MODE_STR2FIFO	0x60
#define FIFO_MODE_BYPASS2STR	0x80
#define FIFO_WATERMARK_MASK	0x1F

#define FIFO_STORED_DATA_MASK	0x1F

#define FUZZ			0
#define FLAT			0

/* RESUME STATE INDICES */
#define RES_CTRL_REG1		0
#define RES_CTRL_REG2		1
#define RES_CTRL_REG3		2
#define RES_CTRL_REG4		3
#define RES_CTRL_REG5		4
#define RES_FIFO_CTRL_REG	5

#define LSM330DLC_GYR_MIN_POLL_PERIOD_MS	2

/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones have to be set to -EINVAL
 */
#define LSM330DLC_GYR_DEFAULT_INT1_GPIO		-EINVAL
#define LSM330DLC_GYR_DEFAULT_INT2_GPIO		-EINVAL

/* Gyroscope Sensor Full Scale */
#define LSM330DLC_GYR_FS_250DPS		0x00
#define LSM330DLC_GYR_FS_500DPS		0x10
#define LSM330DLC_GYR_FS_2000DPS	0x30

/** Registers Contents */
#define WHOAMI_LSM330DLC_GYR		0x00D4

static int int1_gpio = LSM330DLC_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330DLC_GYR_DEFAULT_INT2_GPIO;
/* module_param(int1_gpio, int, S_IRUGO); */
module_param(int2_gpio, int, S_IRUGO);

/*
 * LSM330DLC gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */
struct lsm330dlc_gyr_triple {
	short x;
	short y;
	short z;
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {
	{ 2,	ODR760 | BW10 },
	{ 3,	ODR380 | BW01 },
	{ 6,	ODR190 | BW00 },
	{ 11,	ODR095 | BW00 },
};

static struct lsm330dlc_gyr_platform_data default_lsm330dlc_gyr_pdata = {
	.fs_range = LSM330DLC_GYR_FS_250DPS,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,

	.poll_interval = 100,
	.min_interval = LSM330DLC_GYR_MIN_POLL_PERIOD_MS, /* 2ms */

	.gpio_int1 = LSM330DLC_GYR_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330DLC_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */
};

static int lsm330dlc_gyr_register_update(struct lsm330dlc_gyr_dev *dev,
					 u8 reg_address, u8 mask,
					 u8 new_bit_values)
{
	int err;
	u8 data, updated_val;

	err = dev->tf->read(dev->dev, reg_address, 1, &data);
	if (err < 0)
		return err;

	updated_val = ((mask & new_bit_values) | (~mask & data));
	err = dev->tf->write(dev->dev, reg_address, 1, &updated_val);

	return err;
}

static int lsm330dlc_gyr_update_watermark(struct lsm330dlc_gyr_dev *dev,
					  u8 watermark)
{
	int res;
	u8 new_value;

	mutex_lock(&dev->lock);
	new_value = (watermark % 0x20);
	res = lsm330dlc_gyr_register_update(dev, FIFO_CTRL_REG,
					    FIFO_WATERMARK_MASK,
					    new_value);
	if (res < 0) {
		dev_err(dev->dev, "failed to update watermark\n");
		mutex_unlock(&dev->lock);
		return res;
	}

#ifdef LSM303D_DEBUG
	dev_dbg(dev->dev, "%s new_value: 0x%02x, watermark: 0x%02x\n",
		__func__, new_value, watermark);
#endif

	dev->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_WATERMARK_MASK & new_value) |
		 (~FIFO_WATERMARK_MASK & dev->resume_state[RES_FIFO_CTRL_REG]));
	dev->watermark = new_value;
	mutex_unlock(&dev->lock);

	return res;
}

static int lsm330dlc_gyr_update_fifomode(struct lsm330dlc_gyr_dev *dev,
					 u8 fifomode)
{
	int res;

	res = lsm330dlc_gyr_register_update(dev, FIFO_CTRL_REG, FIFO_MODE_MASK,
					    fifomode);
	if (res < 0) {
		dev_err(dev->dev, "failed to update fifoMode\n");
		return res;
	}

	dev->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_MODE_MASK & fifomode) |
		 (~FIFO_MODE_MASK & dev->resume_state[RES_FIFO_CTRL_REG]));
	dev->fifomode = fifomode;

	return res;
}

static int lsm330dlc_gyr_fifo_reset(struct lsm330dlc_gyr_dev *dev)
{
	int res;
	u8 oldmode = dev->fifomode;

	res = lsm330dlc_gyr_update_fifomode(dev, FIFO_MODE_BYPASS);
	if (res < 0)
		return res;

	res = lsm330dlc_gyr_update_fifomode(dev, oldmode);
	if (res >= 0)
		dev_dbg(dev->dev, "%s fifo reset to: 0x%02x\n",
			__func__, oldmode);

	return res;
}

static int lsm330dlc_gyr_fifo_hwenable(struct lsm330dlc_gyr_dev *dev,
				       u8 enable)
{
	int res;
	u8 set = (enable) ? FIFO_ENABLE : 0;

	res = lsm330dlc_gyr_register_update(dev, CTRL_REG5, FIFO_ENABLE, set);
	if (res < 0) {
		dev_err(dev->dev, "fifo_hw switch to:0x%02x failed\n", set);
		return res;
	}

	dev->resume_state[RES_CTRL_REG5] =
		((FIFO_ENABLE & set) |
		 (~FIFO_ENABLE & dev->resume_state[RES_CTRL_REG5]));

#ifdef LSM330DLC_DEBUG
	dev_dbg(dev->dev, "%s set to:0x%02x\n", __func__, set);
#endif

	return res;
}

static int lsm330dlc_gyr_manage_int2settings(struct lsm330dlc_gyr_dev *dev,
					     u8 fifomode)
{
	int res;
	bool enable_fifo_hw, recognized_mode;
	u8 int2bits = I2_NONE;

	switch (fifomode) {
	case FIFO_MODE_FIFO:
	case FIFO_MODE_BYPASS:
		recognized_mode = true;

		int2bits = I2_NONE;

		res = lsm330dlc_gyr_register_update(dev, CTRL_REG3, I2_MASK,
						    int2bits);
		if (res < 0) {
			dev_err(dev->dev,
				"%s : failed to update to CTRL_REG3: 0x%02x\n",
				__func__, fifomode);
			goto err_mutex_unlock;
		}
		dev->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			 (~I2_MASK & dev->resume_state[RES_CTRL_REG3]));
		enable_fifo_hw = false;
		break;

	default:
		recognized_mode = false;
		res = lsm330dlc_gyr_register_update(dev, CTRL_REG3, I2_MASK,
						    I2_NONE);
		if (res < 0) {
			dev_err(dev->dev,
				"%s : failed to update CTRL_REG3:0x%02x\n",
				__func__, fifomode);
			goto err_mutex_unlock;
		}
		enable_fifo_hw = false;
		dev->resume_state[RES_CTRL_REG3] =
			((I2_MASK & 0x00) |
			 (~I2_MASK & dev->resume_state[RES_CTRL_REG3]));
		break;

	}

	if (recognized_mode) {
		res = lsm330dlc_gyr_update_fifomode(dev, fifomode);
		if (res < 0) {
			dev_err(dev->dev,
				"%s : failed to set fifoMode\n", __func__);
			goto err_mutex_unlock;
		}
	}
	res = lsm330dlc_gyr_fifo_hwenable(dev, enable_fifo_hw);

err_mutex_unlock:

	return res;
}

static int lsm330dlc_gyr_update_fs_range(struct lsm330dlc_gyr_dev *dev,
					 u8 new_fs)
{
	int res ;

	res = lsm330dlc_gyr_register_update(dev, CTRL_REG4, FS_MASK, new_fs);
	if (res < 0) {
		dev_err(dev->dev, "%s : failed to update fs:0x%02x\n",
			__func__, new_fs);
		return res;
	}

	dev->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		 (~FS_MASK & dev->resume_state[RES_CTRL_REG4]));

	return res;
}

static int lsm330dlc_gyr_update_odr(struct lsm330dlc_gyr_dev *dev,
				    u32 poll_interval_ms)
{
	if (atomic_read(&dev->enabled)) {
		int err, i;
		u8 data;

		for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
			if ((odr_table[i].poll_rate_ms <= poll_interval_ms) ||
			    (i == 0))
				break;
		}

		data = odr_table[i].mask | (ENABLE_ALL_AXES + PM_NORMAL);

		err = dev->tf->write(dev->dev, CTRL_REG1, 1, &data);
		if (err < 0)
			return err;

		dev->resume_state[RES_CTRL_REG1] = data;
	}


	return 0;
}

/* gyroscope data readout */
static int lsm330dlc_gyr_get_data(struct lsm330dlc_gyr_dev *dev,
				  struct lsm330dlc_gyr_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s16 hw_d[3];

	err = dev->tf->read(dev->dev, AXISDATA_REG, 6, gyro_out);
	if (err < 0)
		return err;

	hw_d[0] = (s16)(((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s16)(((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s16)(((gyro_out[5]) << 8) | gyro_out[4]);

	data->x = ((dev->pdata->negate_x) ? (-hw_d[dev->pdata->axis_map_x])
		   : (hw_d[dev->pdata->axis_map_x]));
	data->y = ((dev->pdata->negate_y) ? (-hw_d[dev->pdata->axis_map_y])
		   : (hw_d[dev->pdata->axis_map_y]));
	data->z = ((dev->pdata->negate_z) ? (-hw_d[dev->pdata->axis_map_z])
		   : (hw_d[dev->pdata->axis_map_z]));

	return err;
}

static void lsm330dlc_gyr_report_values(struct lsm330dlc_gyr_dev *dev,
					struct lsm330dlc_gyr_triple *data)
{
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, data->x);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, data->y);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, data->z);
	input_sync(dev->input_dev);
}

static int lsm330dlc_gyr_hw_init(struct lsm330dlc_gyr_dev *dev)
{
	int err;
	u8 buf[5];

	buf[0] = dev->resume_state[RES_CTRL_REG1];
	buf[1] = dev->resume_state[RES_CTRL_REG2];
	buf[2] = dev->resume_state[RES_CTRL_REG3];
	buf[3] = dev->resume_state[RES_CTRL_REG4];
	buf[4] = dev->resume_state[RES_CTRL_REG5];

	err = dev->tf->write(dev->dev, CTRL_REG1, 5, buf);
	if (err < 0)
		return err;

	buf[0] = dev->resume_state[RES_FIFO_CTRL_REG];
	err = dev->tf->write(dev->dev, FIFO_CTRL_REG, 1, buf);
	if (err < 0)
			return err;

	dev->hw_initialized = 1;

	return err;
}

static void lsm330dlc_gyr_device_power_off(struct lsm330dlc_gyr_dev *dev)
{
	int err;
	u8 data = PM_OFF;

	err = dev->tf->write(dev->dev, CTRL_REG1, 1, &data);
	if (err < 0)
		dev_err(dev->dev, "soft power off failed\n");

	if (dev->pdata->power_off) {
		/* disable_irq_nosync(acc->irq1); */
		disable_irq_nosync(dev->irq);
		dev->pdata->power_off();
		dev->hw_initialized = 0;
	}

	if (dev->hw_initialized) {
		if (dev->pdata->gpio_int2 >= 0) {
			disable_irq_nosync(dev->irq);
			dev_info(dev->dev, "power off: irq disabled\n");
		}
		dev->hw_initialized = 0;
	}
}

static int lsm330dlc_gyr_device_power_on(struct lsm330dlc_gyr_dev *dev)
{
	int err;

	if (dev->pdata->power_on) {
		err = dev->pdata->power_on();
		if (err < 0)
			return err;
		if (dev->pdata->gpio_int2 >= 0)
			enable_irq(dev->irq);
	}

	if (!dev->hw_initialized) {
		err = lsm330dlc_gyr_hw_init(dev);
		if (err < 0) {
			lsm330dlc_gyr_device_power_off(dev);
			return err;
		}
	}

	if (dev->hw_initialized) {
		dev_dbg(dev->dev, "dev->pdata->gpio_int2 = %d\n",
			dev->pdata->gpio_int2);
		if (dev->pdata->gpio_int2 >= 0) {
			enable_irq(dev->irq);
			dev_info(dev->dev, "power on: irq enabled\n");
		}
	}

	return 0;
}

static int lsm330dlc_gyr_enable(struct lsm330dlc_gyr_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled, 0, 1)) {
		int err;

		err = lsm330dlc_gyr_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled, 0);
			return err;
		}
		schedule_delayed_work(&dev->input_work,
			msecs_to_jiffies(dev->pdata->poll_interval));
	}

	return 0;
}

static int lsm330dlc_gyr_disable(struct lsm330dlc_gyr_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled, 1, 0)) {
		cancel_delayed_work_sync(&dev->input_work);
		lsm330dlc_gyr_device_power_off(dev);
	}

	return 0;
}

static ssize_t attr_polling_rate_show(struct device *device,
				      struct device_attribute *attr,
				      char *buf)
{
	int val;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *device,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	unsigned long interval_ms;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, dev->pdata->min_interval);

	mutex_lock(&dev->lock);
	dev->pdata->poll_interval = interval_ms;
	lsm330dlc_gyr_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_range_show(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);
	int range = 0;

	mutex_lock(&dev->lock);
	switch (dev->pdata->fs_range) {
	case LSM330DLC_GYR_FS_250DPS:
		range = 250;
		break;
	case LSM330DLC_GYR_FS_500DPS:
		range = 500;
		break;
	case LSM330DLC_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	u8 range;
	unsigned long val;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 250:
		range = LSM330DLC_GYR_FS_250DPS;
		break;
	case 500:
		range = LSM330DLC_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM330DLC_GYR_FS_2000DPS;
		break;
	default:
		dev_err(dev->dev, "invalid range request: %lu\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lsm330dlc_gyr_update_fs_range(dev, range);
	if (err >= 0)
		dev->pdata->fs_range = range;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_enable_show(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);
	int val = atomic_read(&dev->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	unsigned long val;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330dlc_gyr_enable(dev);
	else
		lsm330dlc_gyr_disable(dev);

	return size;
}

static ssize_t attr_watermark_store(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int res;
	unsigned long watermark;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 16, &watermark))
		return -EINVAL;

	res = lsm330dlc_gyr_update_watermark(dev, watermark);
	if (res < 0)
		return res;

	return size;
}

static ssize_t attr_watermark_show(struct device *device,
				   struct device_attribute *attr,
				   char *buf)
{
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);
	int val = dev->watermark;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_fifomode_store(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long fifomode;
	int res;

	if (kstrtoul(buf, 16, &fifomode))
		return -EINVAL;

	mutex_lock(&dev->lock);
	res = lsm330dlc_gyr_manage_int2settings(dev, (u8)fifomode);
	mutex_unlock(&dev->lock);

	return (res < 0) ? res : size;
}

static ssize_t attr_fifomode_show(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);
	u8 val = dev->fifomode;

	return sprintf(buf, "0x%02x\n", val);
}

#ifdef LSM330DLC_DEBUG
static ssize_t attr_reg_set(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int err;
	unsigned long val;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	err = dev->tf->write(dev->dev, dev->reg_addr, 1, val);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_reg_get(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);
	int err;
	u8 data;

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, dev->reg_addr, 1, &data);
	mutex_unlock(&dev->lock);

	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *device, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	unsigned long val;
	struct lsm330dlc_gyr_dev *dev = dev_get_drvdata(device);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	dev->reg_addr = val;
	mutex_unlock(&dev->lock);

	return size;
}
#endif /* LSM330DLC_DEBUG */

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0644, attr_polling_rate_show,
	       attr_polling_rate_store),
	__ATTR(range, 0644, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0644, attr_enable_show, attr_enable_store),
	__ATTR(fifo_samples, 0644, attr_watermark_show, attr_watermark_store),
	__ATTR(fifo_mode, 0644, attr_fifomode_show, attr_fifomode_store),
#ifdef LSM330DLC_DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
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

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);

	return 0;
}

static void lsm330dlc_gyr_report_triple(struct lsm330dlc_gyr_dev *dev)
{
	int err;
	struct lsm330dlc_gyr_triple data_out;

	err = lsm330dlc_gyr_get_data(dev, &data_out);
	if (err < 0)
		dev_err(dev->dev, "get_gyroscope_data failed\n");
	else
		lsm330dlc_gyr_report_values(dev, &data_out);
}

static void lsm330dlc_gyr_input_work_func(struct work_struct *work)
{
	struct lsm330dlc_gyr_dev *dev;

	dev = container_of((struct delayed_work *)work,
			    struct lsm330dlc_gyr_dev, input_work);

	mutex_lock(&dev->lock);
	lsm330dlc_gyr_report_triple(dev);
	schedule_delayed_work(&dev->input_work,
			      msecs_to_jiffies(dev->pdata->poll_interval));
	mutex_unlock(&dev->lock);
}

static void lsm330dlc_gyr_irq_fifo(struct lsm330dlc_gyr_dev *dev)
{
	mutex_lock(&dev->lock);

	switch (dev->fifomode) {
	case FIFO_MODE_BYPASS: {
		dev_dbg(dev->dev, "%s : fifomode:0x%02x\n", __func__,
			dev->fifomode);
		lsm330dlc_gyr_report_triple(dev);
		break;
	}
	case FIFO_MODE_FIFO: {
		int err;
		u8 stored_samples, int_source, samples = (dev->watermark) + 1;

		dev_dbg(dev->dev, "%s : FIFO_SRC_REG init samples:%d\n",
			__func__, samples);

		err = dev->tf->read(dev->dev, FIFO_SRC_REG, 1, &int_source);
		if (err < 0) {
			dev_err(dev->dev, "error reading fifo source reg\n");
		}

		dev_dbg(dev->dev, "%s :FIFO_SRC_REG content:0x%02x\n",
			__func__, int_source);

		stored_samples = int_source & FIFO_STORED_DATA_MASK;
		dev_dbg(dev->dev, "%s : fifomode:0x%02x\n", __func__,
			dev->fifomode);

		dev_dbg(dev->dev, "%s : samples:%d stored:%d\n",
			__func__, samples, stored_samples);

		for (; samples > 0; samples--) {
			dev_dbg(dev->dev, "%s : current sample:%d\n",
				__func__, samples);
			lsm330dlc_gyr_report_triple(dev);
		}

		lsm330dlc_gyr_fifo_reset(dev);
		break;
	}
	}

	mutex_unlock(&dev->lock);
}

static irqreturn_t lsm330dlc_gyr_isr(int irq, void *data)
{
	struct lsm330dlc_gyr_dev *dev = data;

	disable_irq_nosync(irq);
	queue_work(dev->irq_work_queue, &dev->irq_work);

	return IRQ_HANDLED;
}

static void lsm330dlc_gyr_irq_work_func(struct work_struct *work)
{

	struct lsm330dlc_gyr_dev *dev;

	dev = container_of(work, struct lsm330dlc_gyr_dev, irq_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330dlc_gyr_irq_XXX(stat); */
	lsm330dlc_gyr_irq_fifo(dev);
	enable_irq(dev->irq);
}

static int lsm330dlc_gyr_validate_pdata(struct lsm330dlc_gyr_dev *dev)
{
	/* checks for correctness of minimal polling period */
	dev->pdata->min_interval =
		max((unsigned int)LSM330DLC_GYR_MIN_POLL_PERIOD_MS,
		    dev->pdata->min_interval);

	dev->pdata->poll_interval = max(dev->pdata->poll_interval,
					dev->pdata->min_interval);

	if (dev->pdata->axis_map_x > 2 || dev->pdata->axis_map_y > 2 ||
	    dev->pdata->axis_map_z > 2) {
		dev_err(dev->dev, "invalid axis_map value x:%u y:%u z%u\n",
			dev->pdata->axis_map_x, dev->pdata->axis_map_y,
			dev->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (dev->pdata->negate_x > 1 || dev->pdata->negate_y > 1 ||
	    dev->pdata->negate_z > 1) {
		dev_err(dev->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			dev->pdata->negate_x,
			dev->pdata->negate_y,
			dev->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (dev->pdata->poll_interval < dev->pdata->min_interval) {
		dev_err(dev->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int lsm330dlc_gyr_input_init(struct lsm330dlc_gyr_dev *dev)
{
	int err;

	INIT_DELAYED_WORK(&dev->input_work, lsm330dlc_gyr_input_work_func);

	dev->input_dev = input_allocate_device();
	if (!dev->input_dev) {
		dev_err(dev->dev, "input device allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev->id.bustype = dev->bus_type;
	dev->input_dev->dev.parent = dev->dev;
	dev->input_dev->name = LSM330DLC_GYR_DEV_NAME;

	input_set_drvdata(dev->input_dev, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev->mscbit);

	err = input_register_device(dev->input_dev);
	if (err) {
		dev_err(dev->dev,
			"unable to register input polled device %s\n",
			dev->input_dev->name);
		input_free_device(dev->input_dev);
		return err;
	}

	return 0;
}

static void lsm330dlc_gyr_input_cleanup(struct lsm330dlc_gyr_dev *dev)
{
	input_unregister_device(dev->input_dev);
	input_free_device(dev->input_dev);
}

int lsm330dlc_gyr_probe(struct lsm330dlc_gyr_dev *dev)
{
	int err = -1;

	mutex_lock(&dev->lock);

	dev->pdata = kmalloc(sizeof(*dev->pdata), GFP_KERNEL);
	if (dev->pdata == NULL) {
		dev_err(dev->dev,
			"failed to allocate memory for pdata: %d\n", err);
		mutex_unlock(&dev->lock);
		return err;
	}

	if (dev->dev->platform_data == NULL) {
		default_lsm330dlc_gyr_pdata.gpio_int1 = int1_gpio;
		default_lsm330dlc_gyr_pdata.gpio_int2 = int2_gpio;
		memcpy(dev->pdata, &default_lsm330dlc_gyr_pdata,
		       sizeof(*dev->pdata));
		dev_info(dev->dev, "using default plaform_data\n");
	} else {
		memcpy(dev->pdata, dev->dev->platform_data,
		       sizeof(*dev->pdata));
	}

	err = lsm330dlc_gyr_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	if (dev->pdata->init) {
		err = dev->pdata->init();
		if (err < 0) {
			dev_err(dev->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}

	dev->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES |
					   PM_NORMAL;
	dev->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	dev->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	dev->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	dev->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	dev->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

	err = lsm330dlc_gyr_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&dev->enabled, 1);

	err = lsm330dlc_gyr_update_fs_range(dev, dev->pdata->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range failed\n");
		goto err2;
	}

	err = lsm330dlc_gyr_update_odr(dev, dev->pdata->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr failed\n");
		goto err2;
	}

	err = lsm330dlc_gyr_input_init(dev);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "%s device register failed\n",
			LSM330DLC_GYR_DEV_NAME);
		goto err4;
	}

	lsm330dlc_gyr_device_power_off(dev);

	/* As default, do not report information */
	atomic_set(&dev->enabled, 0);

	if (dev->pdata->gpio_int2 >= 0) {
		dev->irq = gpio_to_irq(dev->pdata->gpio_int2);
		dev_info(dev->dev, "%s: %s has set irq: %d mapped on gpio:%d\n",
			 LSM330DLC_GYR_DEV_NAME, __func__, dev->irq,
			 dev->pdata->gpio_int2);

		INIT_WORK(&dev->irq_work, lsm330dlc_gyr_irq_work_func);
		dev->irq_work_queue =
			create_singlethread_workqueue("lsm330dlc_gyr_irq_wq");
		if (!dev->irq_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev, "cannot create work queue2: %d\n",
				err);
			goto err5;
		}

		err = request_irq(dev->irq, lsm330dlc_gyr_isr,
				  IRQF_TRIGGER_HIGH, "lsm330dlc_gyr_irq", dev);

		if (err < 0) {
			dev_err(dev->dev, "request irq failed: %d\n", err);
			goto err6;
		}
		disable_irq_nosync(dev->irq);
	}

	mutex_unlock(&dev->lock);

	return 0;

err6:
	destroy_workqueue(dev->irq_work_queue);
err5:
	lsm330dlc_gyr_device_power_off(dev);
	remove_sysfs_interfaces(dev->dev);
err4:
	lsm330dlc_gyr_input_cleanup(dev);
err3:
	lsm330dlc_gyr_device_power_off(dev);
err2:
	if (dev->pdata->exit)
		dev->pdata->exit();
err1_1:
	kfree(dev->pdata);
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(lsm330dlc_gyr_probe);

int lsm330dlc_gyr_remove(struct lsm330dlc_gyr_dev *dev)
{
	/* if (dev->pdata->gpio_int1 >= 0)
	{
		free_irq(dev->irq1, dev);
		gpio_free(dev->pdata->gpio_int1);
		destroy_workqueue(dev->irq1_work_queue);
	}
	*/
	if (dev->pdata->gpio_int2 >= 0) {
		free_irq(dev->irq, dev);
		gpio_free(dev->pdata->gpio_int2);
		destroy_workqueue(dev->irq_work_queue);
	}

	lsm330dlc_gyr_disable(dev);
	lsm330dlc_gyr_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);
	kfree(dev->pdata);

	return 0;
}
EXPORT_SYMBOL(lsm330dlc_gyr_remove);

int lsm330dlc_gyr_resume(struct lsm330dlc_gyr_dev *dev)
{
	if (atomic_read(&dev->enabled)) {
		int err;
		mutex_lock(&dev->lock);
		err = lsm330dlc_gyr_register_update(dev, CTRL_REG1, 0x0F,
					(ENABLE_ALL_AXES | PM_NORMAL));
		if (err < 0) {
			mutex_unlock(&dev->lock);
			return err;
		}
		schedule_delayed_work(&dev->input_work,
				msecs_to_jiffies(dev->pdata->poll_interval));
		mutex_unlock(&dev->lock);
	}
	return 0;
}
EXPORT_SYMBOL(lsm330dlc_gyr_resume);

int lsm330dlc_gyr_suspend(struct lsm330dlc_gyr_dev *dev)
{
	if (atomic_read(&dev->enabled)) {
		int err;

		cancel_delayed_work_sync(&dev->input_work);
		mutex_lock(&dev->lock);
		err = lsm330dlc_gyr_register_update(dev, CTRL_REG1, 0x0F,
						    PM_OFF);
		if (err >= 0)
			err = 0;
		mutex_unlock(&dev->lock);
	}
	return 0;
}
EXPORT_SYMBOL(lsm330dlc_gyr_suspend);

MODULE_DESCRIPTION("lsm330dlc digital gyroscope section driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL v2");

