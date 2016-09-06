/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: lsm9ds0_gyr_sysfs.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Lorenzo Bianconi (lorenzo.bianconi@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.2 sysfs
* Date			: 2012/Jul/10
* Description		: LSM9DS0 digital output gyroscope sensor API
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
* 1.2		| 2012/Jul/10	| Denis Ciocca    | input_poll_dev removal
* 1.2.1		| 2012/Jul/10	| Denis Ciocca	  | added high resolution timers
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>

#include "lsm9ds0.h"

/* Maximum polled-device-reported rot speed value value in dps */
#define FS_MAX		32768
#define MS_TO_NS(x)	((x) * 1000000L)
#define SEC_PORTION_FROM_MS(x)	(s64)((x) / 1000)
#define NSEC_PORTION_FROM_MS(x)	MS_TO_NS((x) % 1000)

/* lsm9ds0 gyroscope registers */
#define WHO_AM_I	0x0F

/* udps/LSB */
#define SENSITIVITY_250		8750
#define SENSITIVITY_500		17500
#define SENSITIVITY_2000	70000

#define CTRL_REG1	0x20
#define CTRL_REG2	0x21
#define CTRL_REG3	0x22
#define CTRL_REG4	0x23
#define CTRL_REG5	0x24
#define REFERENCE	0x25
#define FIFO_CTRL_REG	0x2E
#define FIFO_SRC_REG	0x2F
#define OUT_X_L		0x28

#define AXISDATA_REG	OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES	0x00
#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07
#define ENABLE_NO_AXES	0x00
#define BW00		0x00
#define BW01		0x10
#define BW10		0x20
#define BW11		0x30
#define ODR095		0x00  /* ODR =  95Hz */
#define ODR190		0x40  /* ODR = 190Hz */
#define ODR380		0x80  /* ODR = 380Hz */
#define ODR760		0xC0  /* ODR = 760Hz */

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

/* RESUME STATE INDICES */
#define RES_CTRL_REG1			0
#define RES_CTRL_REG2			1
#define RES_CTRL_REG3			2
#define RES_CTRL_REG4			3
#define RES_CTRL_REG5			4
#define RES_FIFO_CTRL_REG		5

/** Registers Contents */
#define WHOAMI_LSM9DS0_GYR	0xD4

static int int1_gpio = LSM9DS0_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM9DS0_GYR_DEFAULT_INT2_GPIO;
/* module_param(int1_gpio, int, S_IRUGO); */
module_param(int2_gpio, int, S_IRUGO);

/*
 * LSM9DS0 gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * s32
 */
struct lsm9ds0_gyr_triple {
	s32 x;
	s32 y;
	s32 z;
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {
	{ 2, ODR760 | BW10 },
	{ 3, ODR380 | BW01 },
	{ 6, ODR190 | BW00 },
	{ 11, ODR095 | BW00 },
};

static struct lsm9ds0_gyr_platform_data default_lsm9ds0_gyr_pdata = {
	.fs_range = LSM9DS0_GYR_FS_250DPS,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,

	.poll_interval = 100,
	.min_interval = LSM9DS0_GYR_MIN_POLL_PERIOD_MS, /* 2ms */

	.gpio_int1 = LSM9DS0_GYR_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM9DS0_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */
};

static int lsm9ds0_gyr_register_update(struct lsm9ds0_gyr_dev *dev,
				       u8 reg_address, u8 mask,
				       u8 new_bit_values)
{
	int err;
	u8 val;
	err = dev->tf->read(dev->dev, reg_address, 1, &val);
	if (err < 0)
		return err;

	val = (mask & new_bit_values) | (~mask & val);
	err = dev->tf->write(dev->dev, reg_address, 1, &val);
	if (err < 0)
		return err;

	return 0;
}

static int lsm9ds0_gyr_update_watermark(struct lsm9ds0_gyr_dev *dev,
					u8 watermark)
{
	int res = 0;
	u8 new_value;

	mutex_lock(&dev->lock);
	new_value = (watermark % 0x20);
	res = lsm9ds0_gyr_register_update(dev, FIFO_CTRL_REG,
					  FIFO_WATERMARK_MASK, new_value);
	if (res < 0) {
		dev_err(dev->dev, "failed to update watermark\n");
		mutex_unlock(&dev->lock);
		return res;
	}
	dev_dbg(dev->dev, "%s new_value:0x%02x,watermark:0x%02x\n",
		__func__, new_value, watermark);

	dev->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_WATERMARK_MASK & new_value) |
		 (~FIFO_WATERMARK_MASK &
		  dev->resume_state[RES_FIFO_CTRL_REG]));
	dev->watermark = new_value;
	mutex_unlock(&dev->lock);

	return res;
}

static int lsm9ds0_gyr_update_fifomode(struct lsm9ds0_gyr_dev *dev,
				       u8 fifomode)
{
	int res;

	res = lsm9ds0_gyr_register_update(dev, FIFO_CTRL_REG,
					  FIFO_MODE_MASK, fifomode);
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

static int lsm9ds0_gyr_fifo_reset(struct lsm9ds0_gyr_dev *dev)
{
	u8 oldmode;
	int res;

	oldmode = dev->fifomode;
	res = lsm9ds0_gyr_update_fifomode(dev, FIFO_MODE_BYPASS);
	if (res < 0)
		return res;
	res = lsm9ds0_gyr_update_fifomode(dev, oldmode);
	if (res >= 0)
		dev_dbg(dev->dev, "%s fifo reset to: 0x%02x\n",
			__func__, oldmode);

	return res;
}

static int lsm9ds0_gyr_fifo_hwenable(struct lsm9ds0_gyr_dev *dev, u8 enable)
{
	int res;
	u8 set = (enable) ? FIFO_ENABLE : 0x00;

	res = lsm9ds0_gyr_register_update(dev, CTRL_REG5, FIFO_ENABLE, set);
	if (res < 0) {
		dev_err(dev->dev, "fifo_hw switch to:0x%02x failed\n", set);
		return res;
	}
	dev->resume_state[RES_CTRL_REG5] =
		((FIFO_ENABLE & set) |
		 (~FIFO_ENABLE & dev->resume_state[RES_CTRL_REG5]));

	return res;
}

static int lsm9ds0_gyr_manage_int2settings(struct lsm9ds0_gyr_dev *dev,
					   u8 fifomode)
{
	int res;
	bool enable_fifo_hw, recognized_mode = false;
	u8 int2bits = I2_NONE;

	switch (fifomode) {
	case FIFO_MODE_FIFO:
		recognized_mode = true;

		if (dev->polling_enabled) {
			int2bits = I2_NONE;
			enable_fifo_hw = false;
		} else {
			int2bits = (I2_WTM | I2_OVRUN);
			enable_fifo_hw = true;
		}
		res = lsm9ds0_gyr_register_update(dev, CTRL_REG3,
						  I2_MASK, int2bits);
		if (res < 0) {
			dev_err(dev->dev,
				"%s : failed to update CTRL_REG3:0x%02x\n",
				__func__, fifomode);
			return res;
		}
		dev->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			 (~I2_MASK & dev->resume_state[RES_CTRL_REG3]));
		/* enable_fifo_hw = true; */
		break;
	case FIFO_MODE_BYPASS:
		recognized_mode = true;

		if (dev->polling_enabled)
			int2bits = I2_NONE;
		else
			int2bits = I2_DRDY;

		res = lsm9ds0_gyr_register_update(dev, CTRL_REG3, I2_MASK,
						  int2bits);
		if (res < 0) {
			dev_err(dev->dev,
				"%s : failed to update to CTRL_REG3:0x%02x\n",
				__func__, fifomode);
			return res;
		}
		dev->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			 (~I2_MASK & dev->resume_state[RES_CTRL_REG3]));
		enable_fifo_hw = false;
		break;
	default:
		recognized_mode = false;
		res = lsm9ds0_gyr_register_update(dev, CTRL_REG3, I2_MASK,
						  I2_NONE);
		if (res < 0) {
			dev_err(dev->dev,
				"%s: failed to update CTRL_REG3:0x%02x\n",
				__func__, fifomode);
			return res;
		}
		enable_fifo_hw = false;
		dev->resume_state[RES_CTRL_REG3] =
			(~I2_MASK & dev->resume_state[RES_CTRL_REG3]);
		break;
	}

	if (recognized_mode) {
		res = lsm9ds0_gyr_update_fifomode(dev, fifomode);
		if (res < 0) {
			dev_err(dev->dev, "%s : failed to set fifoMode\n",
				__func__);
			return res;
		}
	}

	return lsm9ds0_gyr_fifo_hwenable(dev, enable_fifo_hw);
}


static int lsm9ds0_gyr_update_fs_range(struct lsm9ds0_gyr_dev *dev, u8 new_fs)
{
	int res ;
	u32 sensitivity;

	switch(new_fs) {
	case LSM9DS0_GYR_FS_250DPS:
		sensitivity = SENSITIVITY_250;
		break;
	case LSM9DS0_GYR_FS_500DPS:
		sensitivity = SENSITIVITY_500;
		break;
	case LSM9DS0_GYR_FS_2000DPS:
		sensitivity = SENSITIVITY_2000;
		break;
	default:
		dev_err(dev->dev, "invalid g range requested: %u\n", new_fs);
		return -EINVAL;
	}

	res = lsm9ds0_gyr_register_update(dev, CTRL_REG4, FS_MASK, new_fs);

	if (res < 0) {
		dev_err(dev->dev, "%s : failed to update fs:0x%02x\n",
			__func__, new_fs);
		return res;
	}
	dev->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		 (~FS_MASK & dev->resume_state[RES_CTRL_REG4]));

	dev->sensitivity = sensitivity;

	return res;
}


static int lsm9ds0_gyr_update_odr(struct lsm9ds0_gyr_dev *dev,
				  unsigned int poll_ms)
{
	int i, err = -1;
	u8 data = ENABLE_ALL_AXES + PM_NORMAL;

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_ms) || (i == 0))
			break;
	}
	data |= odr_table[i].mask;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&dev->enabled)) {
		err = dev->tf->write(dev->dev, CTRL_REG1, 1, &data);
		if (err < 0)
			return err;
	}

	dev->resume_state[RES_CTRL_REG1] = data;
	dev->ktime = ktime_set(SEC_PORTION_FROM_MS(poll_ms),
				NSEC_PORTION_FROM_MS(poll_ms));

	return err;
}

/* gyroscope data readout */
static int lsm9ds0_gyr_get_data(struct lsm9ds0_gyr_dev *dev,
				struct lsm9ds0_gyr_triple *data)
{
	int err;
	unsigned char gyro_out[6] = {};
	/* y,p,r hardware data */
	s16 hw_d[3];

	err = dev->tf->read(dev->dev, AXISDATA_REG, 6, gyro_out);
	if (err < 0)
		return err;

	hw_d[0] = le16_to_cpu(*(s16 *)&gyro_out[0]);
	hw_d[1] = le16_to_cpu(*(s16 *)&gyro_out[2]);
	hw_d[2] = le16_to_cpu(*(s16 *)&gyro_out[4]);

	data->x = hw_d[0] * dev->sensitivity;
	data->y = hw_d[1] * dev->sensitivity;
	data->z = hw_d[2] * dev->sensitivity;

/*
	data->x = ((dev->pdata->negate_x) ? (-hw_d[dev->pdata->axis_map_x])
		   : (hw_d[dev->pdata->axis_map_x]));
	data->y = ((dev->pdata->negate_y) ? (-hw_d[dev->pdata->axis_map_y])
		   : (hw_d[dev->pdata->axis_map_y]));
	data->z = ((dev->pdata->negate_z) ? (-hw_d[dev->pdata->axis_map_z])
		   : (hw_d[dev->pdata->axis_map_z]));
		   */

#ifdef LSM9DS0_DEBUG
	dev_info(dev->dev, "gyro_out: x = %d, y = %d, z = %d\n",
		 data->x, data->y, data->z);
#endif

	return err;
}

static void lsm9ds0_gyr_report_values(struct lsm9ds0_gyr_dev *dev,
					struct lsm9ds0_gyr_triple *data)
{
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, data->x);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, data->y);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, data->z);
	input_sync(dev->input_dev);
}

static int lsm9ds0_gyr_hw_init(struct lsm9ds0_gyr_dev *dev)
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

static void lsm9ds0_gyr_device_power_off(struct lsm9ds0_gyr_dev *dev)
{
	int err;
	u8 data = PM_OFF;

	err = dev->tf->write(dev->dev, CTRL_REG1, 1, &data);
	if (err < 0)
		dev_err(dev->dev, "soft power off failed\n");

	if (dev->pdata->power_off) {
		/* disable_irq_nosync(acc->irq1); */
		disable_irq_nosync(dev->irq2);
		dev->pdata->power_off();
		dev->hw_initialized = 0;
	}

	if (dev->hw_initialized) {
		/*if (dev->pdata->gpio_int1 >= 0)*/
		/*	disable_irq_nosync(dev->irq1);*/
		if (dev->pdata->gpio_int2 >= 0) {
			disable_irq_nosync(dev->irq2);
			dev_info(dev->dev, "power off: irq2 disabled\n");
		}
		dev->hw_initialized = 0;
	}
}

static int lsm9ds0_gyr_check_whoami(struct lsm9ds0_gyr_dev *dev)
{
	int err;
	u8 data;

	err = dev->tf->read(dev->dev, WHO_AM_I, 1, &data);
	if (err < 0)
		return err;

	return (data == WHOAMI_LSM9DS0_GYR) ? 0 : -ENODEV;
}

static int lsm9ds0_gyr_device_power_on(struct lsm9ds0_gyr_dev *dev)
{
	int err;

	if (dev->pdata->power_on) {
		err = dev->pdata->power_on();
		if (err < 0)
			return err;
		if (dev->pdata->gpio_int2 >= 0)
			enable_irq(dev->irq2);
	}

	if (!dev->hw_initialized) {
		err = lsm9ds0_gyr_hw_init(dev);
		if (err < 0) {
			lsm9ds0_gyr_device_power_off(dev);
			return err;
		}
	}

	if (dev->hw_initialized) {
		/* if (dev->pdata->gpio_int1) {
			enable_irq(dev->irq1);
			dev_info(dev->dev, "power on: irq1 enabled\n");
		} */
		dev_dbg(dev->dev, "dev->pdata->gpio_int2 = %d\n",
			dev->pdata->gpio_int2);
		if (dev->pdata->gpio_int2 >= 0) {
			enable_irq(dev->irq2);
			dev_info(dev->dev, "power on: irq2 enabled\n");
		}
	}

	return 0;
}

static int lsm9ds0_gyr_enable(struct lsm9ds0_gyr_dev *dev)
{
	int err;

	if (!atomic_cmpxchg(&dev->enabled, 0, 1)) {
		mutex_lock(&dev->lock);
		err = lsm9ds0_gyr_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled, 0);
			mutex_unlock(&dev->lock);
			return err;
		}

		if (dev->polling_enabled) {
			hrtimer_start(&(dev->hr_timer), dev->ktime,
				      HRTIMER_MODE_REL);
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

static int lsm9ds0_gyr_disable(struct lsm9ds0_gyr_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled, 1, 0)) {
		hrtimer_cancel(&dev->hr_timer);

		mutex_lock(&dev->lock);
		lsm9ds0_gyr_device_power_off(dev);
		mutex_unlock(&dev->lock);
	}
	return 0;
}

static ssize_t attr_polling_rate_show(struct device *device,
				      struct device_attribute *attr,
				      char *buf)
{
	int val;
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *device,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	int err;
	unsigned long interval_ms;
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &interval_ms) | !interval_ms)
		return -EINVAL;

	mutex_lock(&dev->lock);
	err = lsm9ds0_gyr_update_odr(dev, interval_ms);
	if(err >= 0)
		dev->pdata->poll_interval = interval_ms;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_range_show(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	int range = 0;
	u8 val;
	mutex_lock(&dev->lock);
	val = dev->pdata->fs_range;

	switch (val) {
	case LSM9DS0_GYR_FS_250DPS:
		range = 250;
		break;
	case LSM9DS0_GYR_FS_500DPS:
		range = 500;
		break;
	case LSM9DS0_GYR_FS_2000DPS:
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
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 250:
		range = LSM9DS0_GYR_FS_250DPS;
		break;
	case 500:
		range = LSM9DS0_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM9DS0_GYR_FS_2000DPS;
		break;
	default:
		dev_err(dev->dev, "invalid range request: %lu\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lsm9ds0_gyr_update_fs_range(dev, range);
	if (err >= 0)
		dev->pdata->fs_range = range;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_enable_show(struct device *device,
			        struct device_attribute *attr, char *buf)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	int val = atomic_read(&dev->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm9ds0_gyr_enable(dev);
	else
		lsm9ds0_gyr_disable(dev);

	return size;
}

static ssize_t attr_polling_mode_show(struct device *device,
				      struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	if (dev->polling_enabled)
		val = 1;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_mode_store(struct device *device,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	unsigned long val;
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	if (val) {
		dev->polling_enabled = true;
		lsm9ds0_gyr_manage_int2settings(dev, dev->fifomode);
		if (atomic_read(&dev->enabled)) {
			hrtimer_start(&(dev->hr_timer), dev->ktime,
				      HRTIMER_MODE_REL);
		}
	} else {
		if (dev->polling_enabled) {
			hrtimer_cancel(&dev->hr_timer);
		}
		dev->polling_enabled = false;
		lsm9ds0_gyr_manage_int2settings(dev, dev->fifomode);
	}
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_watermark_store(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long watermark;
	int res;

	if (strict_strtoul(buf, 16, &watermark))
		return -EINVAL;

	res = lsm9ds0_gyr_update_watermark(dev, watermark);
	if (res < 0)
		return res;

	return size;
}

static ssize_t attr_watermark_show(struct device *device,
				   struct device_attribute *attr, char *buf)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "0x%02x\n", dev->watermark);
}

static ssize_t attr_fifomode_store(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long fifomode;
	int res;

	if (strict_strtoul(buf, 16, &fifomode))
		return -EINVAL;
	/* if (!fifomode)
		return -EINVAL; */

	mutex_lock(&dev->lock);
	res = lsm9ds0_gyr_manage_int2settings(dev, (u8) fifomode);
	mutex_unlock(&dev->lock);

	return (res < 0) ? res : size;
}

static ssize_t attr_fifomode_show(struct device *device,
				  struct device_attribute *attr,
				  char *buf)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "0x%02x\n", dev->fifomode);
}

#ifdef LSM9DS0_DEBUG
static ssize_t attr_reg_set(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int err;
	u8 data;
	unsigned long val;
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	data = (u8)val;
	mutex_lock(&dev->lock);
	err = dev->tf->write(dev->dev, dev->reg_addr, 1, &data);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_reg_get(struct device *device,
			    struct device_attribute *attr,
			    char *buf)
{
	int err;
	u8 data;
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, dev->reg_addr, 1, &data);
	mutex_unlock(&dev->lock);

	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *device,
			     struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct lsm9ds0_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	dev->reg_addr = val;
	mutex_unlock(&dev->lock);

	return size;
}
#endif /* LSM9DS0_DEBUG */

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_polling_rate_show,
	       attr_polling_rate_store),
	__ATTR(range, 0666, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0666, attr_enable_show, attr_enable_store),
	__ATTR(enable_polling, 0666, attr_polling_mode_show,
	       attr_polling_mode_store),
	__ATTR(fifo_samples, 0666, attr_watermark_show, attr_watermark_store),
	__ATTR(fifo_mode, 0666, attr_fifomode_show, attr_fifomode_store),
#ifdef LSM9DS0_DEBUG
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

static void lsm9ds0_gyr_report_triple(struct lsm9ds0_gyr_dev *dev)
{
	int err;
	struct lsm9ds0_gyr_triple data_out;

	err = lsm9ds0_gyr_get_data(dev, &data_out);
	if (err < 0)
		dev_err(dev->dev, "get_gyroscope_data failed\n");
	else
		lsm9ds0_gyr_report_values(dev, &data_out);
}

static void lsm9ds0_gyr_irq2_fifo(struct lsm9ds0_gyr_dev *dev)
{
	int err;
	u8 data, int_source, samples;
	u8 workingmode, stored_samples;

	mutex_lock(&dev->lock);

	workingmode = dev->fifomode;

	dev_dbg(dev->dev, "%s : fifomode:0x%02x\n", __func__, workingmode);

	switch (workingmode) {
	case FIFO_MODE_BYPASS:
		dev_dbg(dev->dev, "%s : fifomode:0x%02x\n",
			__func__, dev->fifomode);
		lsm9ds0_gyr_report_triple(dev);
		break;
	case FIFO_MODE_FIFO:
		samples = (dev->watermark)+1;
		dev_dbg(dev->dev, "%s : FIFO_SRC_REG init samples:%d\n",
			__func__, samples);
		err = dev->tf->read(dev->dev, FIFO_SRC_REG, 1, &data);
		if (err < 0)
			dev_err(dev->dev, "error reading fifo source reg\n");

		int_source = data;
		dev_dbg(dev->dev, "%s :FIFO_SRC_REG content:0x%02x\n",
			__func__, int_source);

		stored_samples = int_source & FIFO_STORED_DATA_MASK;
		dev_dbg(dev->dev, "%s : fifomode:0x%02x\n", __func__,
			dev->fifomode);

		dev_dbg(dev->dev, "%s : samples:%d stored:%d\n",
			__func__, samples, stored_samples);

		for (; samples > 0; samples--)
			lsm9ds0_gyr_report_triple(dev);
		lsm9ds0_gyr_fifo_reset(dev);
		break;
	}

	mutex_unlock(&dev->lock);
}

static irqreturn_t lsm9ds0_gyr_isr2(int irq, void *data)
{
	struct lsm9ds0_gyr_dev *dev = (struct lsm9ds0_gyr_dev *)data;

	disable_irq_nosync(irq);
	queue_work(dev->irq2_work_queue, &dev->irq2_work);
	pr_debug("%s %s: isr2 queued\n", LSM9DS0_GYR_DEV_NAME, __func__);

	return IRQ_HANDLED;
}

static void lsm9ds0_gyr_irq2_work_func(struct work_struct *work)
{

	struct lsm9ds0_gyr_dev *dev =
		container_of(work, struct lsm9ds0_gyr_dev, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm9ds0_gyr_irq2_XXX(dev); */
	lsm9ds0_gyr_irq2_fifo(dev);
	/*  */
	pr_debug("%s %s: IRQ2 served\n", LSM9DS0_GYR_DEV_NAME, __func__);

	enable_irq(dev->irq2);
}

static int lsm9ds0_gyr_validate_pdata(struct lsm9ds0_gyr_dev *dev)
{
	/* checks for correctness of minimal polling period */
	dev->pdata->min_interval = max((u32)LSM9DS0_GYR_MIN_POLL_PERIOD_MS,
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
		dev_err(dev->dev, "invalid negate value x:%u y:%u z:%u\n",
			dev->pdata->negate_x, dev->pdata->negate_y,
			dev->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (dev->pdata->poll_interval < dev->pdata->min_interval) {
		dev_err(dev->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int lsm9ds0_gyr_input_init(struct lsm9ds0_gyr_dev *dev)
{
	int err = -1;

	dev->input_dev = input_allocate_device();
	if (!dev->input_dev) {
		dev_err(dev->dev, "input device allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev->name = LSM9DS0_GYR_DEV_NAME;
	dev->input_dev->id.bustype = dev->bus_type;
	dev->input_dev->dev.parent = dev->dev;

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

static void lsm9ds0_gyr_input_cleanup(struct lsm9ds0_gyr_dev *dev)
{
	input_unregister_device(dev->input_dev);
	input_free_device(dev->input_dev);
}

static void poll_function_work(struct work_struct *polling_task)
{
	struct lsm9ds0_gyr_dev *dev;
	struct lsm9ds0_gyr_triple data_out;
	int err;

	dev = container_of((struct work_struct *)polling_task,
			   struct lsm9ds0_gyr_dev, polling_task);
	mutex_lock(&dev->lock);
	err = lsm9ds0_gyr_get_data(dev, &data_out);
	mutex_unlock(&dev->lock);

	if (err < 0)
		dev_err(dev->dev, "get_rotation_data failed.\n");
	else
		lsm9ds0_gyr_report_values(dev, &data_out);

	hrtimer_start(&dev->hr_timer, dev->ktime, HRTIMER_MODE_REL);
}

static enum hrtimer_restart poll_function_read(struct hrtimer *timer)
{
	struct lsm9ds0_gyr_dev *dev;

	dev = container_of((struct hrtimer *)timer,
			   struct lsm9ds0_gyr_dev, hr_timer);

	queue_work(dev->gyr_workqueue, &dev->polling_task);

	return HRTIMER_NORESTART;
}

int lsm9ds0_gyr_probe(struct lsm9ds0_gyr_dev *dev)
{
	int err;

	mutex_lock(&dev->lock);

	err = lsm9ds0_gyr_check_whoami(dev);
	if (err < 0)
		goto err1;

	dev->pdata = kzalloc(sizeof(*dev->pdata), GFP_KERNEL);
	if (dev->pdata == NULL) {
		dev_err(dev->dev, "failed to allocate memory for pdata: %d\n",
			err);
		goto err1;
	}

	if (dev->dev->platform_data == NULL) {
		default_lsm9ds0_gyr_pdata.gpio_int1 = int1_gpio;
		default_lsm9ds0_gyr_pdata.gpio_int2 = int2_gpio;
		memcpy(dev->pdata, &default_lsm9ds0_gyr_pdata,
		       sizeof(*dev->pdata));
		dev_info(dev->dev, "using default plaform_data\n");
	} else {
		memcpy(dev->pdata, dev->dev->platform_data,
		       sizeof(*dev->pdata));
	}

	err = lsm9ds0_gyr_validate_pdata(dev);
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

	dev->polling_enabled = true;
	dev_info(dev->dev, "polling mode enabled\n");

	err = lsm9ds0_gyr_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&dev->enabled, 1);

	err = lsm9ds0_gyr_update_fs_range(dev, dev->pdata->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range failed\n");
		goto err2;
	}

	err = lsm9ds0_gyr_update_odr(dev, dev->pdata->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr failed\n");
		goto err2;
	}

	err = lsm9ds0_gyr_input_init(dev);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "%s device register failed\n",
			LSM9DS0_GYR_DEV_NAME);
		goto err4;
	}

	lsm9ds0_gyr_device_power_off(dev);

	/* As default, do not report information */
	atomic_set(&dev->enabled, 0);

	if (dev->pdata->gpio_int2 >= 0) {
		dev->irq2 = gpio_to_irq(dev->pdata->gpio_int2);
		dev_info(dev->dev,
			"%s: %s has set irq2 to irq %d mapped on gpio:%d\n",
			LSM9DS0_GYR_DEV_NAME, __func__, dev->irq2,
			dev->pdata->gpio_int2);

		INIT_WORK(&dev->irq2_work, lsm9ds0_gyr_irq2_work_func);
		dev->irq2_work_queue =
			create_singlethread_workqueue("lsm9ds0_gyr_irq2_wq");
		if (!dev->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev, "cannot create work queue2: %d\n",
				err);
			goto err5;
		}

		err = request_irq(dev->irq2, lsm9ds0_gyr_isr2,
				  IRQF_TRIGGER_HIGH, "lsm9ds0_gyr_irq2", dev);

		if (err < 0) {
			dev_err(dev->dev, "request irq2 failed: %d\n", err);
			goto err6;
		}
		disable_irq_nosync(dev->irq2);
	}

	INIT_WORK(&dev->polling_task, poll_function_work);
	dev->gyr_workqueue = create_workqueue("lsm9ds0_gyr_workqueue");
	if (!dev->gyr_workqueue)
		goto err6;

	hrtimer_init(&dev->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer.function = &poll_function_read;

	mutex_unlock(&dev->lock);


	return 0;

/*err7:
	free_irq(dev->irq2, stat);
*/
err6:
	destroy_workqueue(dev->irq2_work_queue);
err5:
	remove_sysfs_interfaces(dev->dev);
err4:
	lsm9ds0_gyr_input_cleanup(dev);
err3:
	lsm9ds0_gyr_device_power_off(dev);
err2:
	if (dev->pdata->exit)
		dev->pdata->exit();
err1_1:
	kfree(dev->pdata);
err1:
	mutex_unlock(&dev->lock);
	if (dev->gyr_workqueue)
		destroy_workqueue(dev->gyr_workqueue);
	
	return err;
}
EXPORT_SYMBOL(lsm9ds0_gyr_probe);

int lsm9ds0_gyr_remove(struct lsm9ds0_gyr_dev *dev)
{
	cancel_work_sync(&dev->polling_task);
	if (dev->gyr_workqueue)
		destroy_workqueue(dev->gyr_workqueue);

	/*
	if (dev->pdata->gpio_int1 >= 0)
	{
		free_irq(dev->irq1, stat);
		gpio_free(dev->pdata->gpio_int1);
		destroy_workqueue(dev->irq1_work_queue);
	}
	*/
	if (dev->pdata->gpio_int2 >= 0) {
		free_irq(dev->irq2, dev);
		gpio_free(dev->pdata->gpio_int2);
		destroy_workqueue(dev->irq2_work_queue);
	}

	lsm9ds0_gyr_disable(dev);
	lsm9ds0_gyr_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	kfree(dev->pdata);

	return 0;
}
EXPORT_SYMBOL(lsm9ds0_gyr_remove);

int lsm9ds0_gyr_suspend(struct lsm9ds0_gyr_dev *dev)
{
	int err = 0;

#define SLEEP
#ifdef CONFIG_PM
	if (atomic_read(&dev->enabled)) {
		mutex_lock(&dev->lock);

		if (dev->polling_enabled)
			hrtimer_cancel(&dev->hr_timer);
#ifdef SLEEP
		err = lsm9ds0_gyr_register_update(dev, CTRL_REG1,
					  0x0F, (ENABLE_NO_AXES | PM_NORMAL));
#else
		err = lsm9ds0_gyr_register_update(dev, CTRL_REG1, 0x08,
						  PM_OFF);
#endif /*SLEEP*/
		mutex_unlock(&dev->lock);
	}
#endif /*CONFIG_PM*/

	return err;
}
EXPORT_SYMBOL(lsm9ds0_gyr_suspend);

int lsm9ds0_gyr_resume(struct lsm9ds0_gyr_dev *dev)
{
	int err = 0;

#ifdef CONFIG_PM
	if (atomic_read(&dev->enabled)) {
		mutex_lock(&dev->lock);
		if (dev->polling_enabled)
			hrtimer_start(&(dev->hr_timer), dev->ktime,
				      HRTIMER_MODE_REL);
#ifdef SLEEP
		err = lsm9ds0_gyr_register_update(dev, CTRL_REG1, 0x0F,
					  (ENABLE_ALL_AXES | PM_NORMAL));
#else
		err = lsm9ds0_gyr_register_update(dev, CTRL_REG1, 0x08,
						  PM_NORMAL);
#endif
		mutex_unlock(&dev->lock);

	}
#endif /*CONFIG_PM*/

	return err;
}
EXPORT_SYMBOL(lsm9ds0_gyr_resume);

MODULE_DESCRIPTION("lsm9ds0 gyroscope driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");
