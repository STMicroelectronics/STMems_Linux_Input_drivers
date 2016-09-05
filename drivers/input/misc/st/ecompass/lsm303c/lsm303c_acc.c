/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name          : lsm303c_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Lorenzo Bianconi (lorenzo.bianconi@st.com)
 *		      : Authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.3
 * Date               : 2016/May/18
 * Description        : LSM303C accelerometer driver
 *
 *******************************************************************************
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
 ******************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "lsm303c.h"

#define MS_TO_NS(x)		(x * 1000000L)

#define SENSITIVITY_2G		 61	/**	ug/LSB	*/
#define SENSITIVITY_4G		122	/**	ug/LSB	*/
#define SENSITIVITY_8G		244	/**	ug/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LSM303C_ACC_ENABLE	(0x01)
#define LSM303C_ACC_DISABLE	(0x00)

#define AXISDATA_REG		(0x28)
#define WHOAMI_LSM303C_ACC	(0x41)	/*	Expctd content for WAI	*/
#define ALL_ZEROES		(0x00)
#define LSM303C_ACC_PM_OFF	(0x00)
#define ACC_ENABLE_ALL_AXES	(0x07)

/*	CONTROL REGISTERS	*/
#define TEMP_L			(0x0B)
#define TEMP_H			(0x0C)
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define ACT_THS			(0x1E)	/*	Activity Threshold	*/
#define ACT_DUR			(0x1F)	/*	Activity Duration	*/
/* ctrl 1: HR ODR2 ODR1 ODR0 BDU Zenable Yenable Xenable */
#define CTRL1			(0x20)	/*	control reg 1		*/
#define CTRL2			(0x21)	/*	control reg 2		*/
#define CTRL3			(0x22)	/*	control reg 3		*/
#define CTRL4			(0x23)	/*	control reg 4		*/
#define CTRL5			(0x24)	/*	control reg 5		*/
#define CTRL6			(0x25)	/*	control reg 6		*/
#define CTRL7			(0x26)	/*	control reg 7		*/

#define FIFO_CTRL		(0x2E)	/*	fifo control reg	*/

#define INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define INT_THSX1		(0x32)	/*	interrupt 1 threshold x	*/
#define INT_THSY1		(0x33)	/*	interrupt 1 threshold y	*/
#define INT_THSZ1		(0x34)	/*	interrupt 1 threshold z	*/
#define INT_DUR1		(0x35)	/*	interrupt 1 duration	*/

#define INT_CFG2		(0x36)	/*	interrupt 2 config	*/
#define INT_SRC2		(0x37)	/*	interrupt 2 source	*/
#define INT_THS2		(0x38)	/*	interrupt 2 threshold	*/
#define INT_DUR2		(0x39)	/*	interrupt 2 duration	*/

#define REF_XL			(0x3A)	/*	reference_l_x		*/
#define REF_XH			(0x3B)	/*	reference_h_x		*/
#define REF_YL			(0x3C)	/*	reference_l_y		*/
#define REF_YH			(0x3D)	/*	reference_h_y		*/
#define REF_ZL			(0x3E)	/*	reference_l_z		*/
#define REF_ZH			(0x3F)	/*	reference_h_z		*/
/*	end CONTROL REGISTRES	*/

#define ACC_ODR10		(0x10)	/*   10Hz output data rate */
#define ACC_ODR50		(0x20)	/*   50Hz output data rate */
#define ACC_ODR100		(0x30)	/*  100Hz output data rate */
#define ACC_ODR200		(0x40)	/*  200Hz output data rate */
#define ACC_ODR400		(0x50)	/*  400Hz output data rate */
#define ACC_ODR800		(0x60)	/*  800Hz output data rate */
#define ACC_ODR_MASK		(0X70)

/* Registers configuration Mask and settings */
/* CTRL1 */
#define CTRL1_HR_DISABLE	(0x00)
#define CTRL1_HR_ENABLE		(0x80)
#define CTRL1_HR_MASK		(0x80)
#define CTRL1_BDU_ENABLE	(0x08)
#define CTRL1_BDU_MASK		(0x08)

/* CTRL2 */
#define CTRL2_IG1_INT1		(0x08)

/* CTRL3 */
#define CTRL3_IG1_INT1		(0x08)
#define CTRL3_DRDY_INT1

/* CTRL4 */
#define CTRL4_IF_ADD_INC_EN	(0x04)
#define CTRL4_BW_SCALE_ODR_AUT	(0x00)
#define CTRL4_BW_SCALE_ODR_SEL	(0x08)
#define CTRL4_ANTALIAS_BW_400	(0x00)
#define CTRL4_ANTALIAS_BW_200	(0x40)
#define CTRL4_ANTALIAS_BW_100	(0x80)
#define CTRL4_ANTALIAS_BW_50	(0xC0)
#define CTRL4_ANTALIAS_BW_MASK	(0xC0)

/* CTRL5 */
#define CTRL5_HLACTIVE_L	(0x02)
#define CTRL5_HLACTIVE_H	(0x00)

/* CTRL6 */
#define CTRL6_IG2_INT2		(0x10)
#define CTRL6_DRDY_INT2		(0x01)

/* CTRL7 */
#define CTRL7_LIR2		(0x08)
#define CTRL7_LIR1		(0x04)
/* */

#define NO_MASK			(0xFF)

#define INT1_DURATION_MASK	(0x7F)
#define INT1_THRESHOLD_MASK	(0x7F)

/* RESUME STATE INDICES */
#define RES_CTRL1		0
#define RES_CTRL2		1
#define RES_CTRL3		2
#define RES_CTRL4		3
#define RES_CTRL5		4
#define RES_CTRL6		5
#define RES_CTRL7		6

#define RES_INT_CFG1		7
#define RES_INT_THSX1		8
#define RES_INT_THSY1		9
#define RES_INT_THSZ1		10
#define RES_INT_DUR1		11

#define RES_INT_CFG2		12
#define RES_INT_THS2		13
#define RES_INT_DUR2		14

#define RES_TEMP_CFG_REG	15
#define RES_REFERENCE_REG	16
#define RES_FIFO_CTRL	17

/* end RESUME STATE INDICES */

#define OUTPUT_ALWAYS_ANTI_ALIASED 1

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm303c_acc_odr_table[] = {
	{    2, ACC_ODR800 },
	{    3, ACC_ODR400  },
	{    5, ACC_ODR200  },
	{   10, ACC_ODR100  },
#if (!OUTPUT_ALWAYS_ANTI_ALIASED)
	{   20, ACC_ODR50   },
	{  100, ACC_ODR10   },
#endif
};

static int int1_gpio = LSM303C_ACC_DEFAULT_INT1_GPIO;
module_param(int1_gpio, int, S_IRUGO);

static struct lsm303c_acc_platform_data default_lsm303c_acc_pdata = {
	.fs_range = LSM303C_ACC_FS_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM303C_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM303C_ACC_DEFAULT_INT1_GPIO,
};

/* sets default init values to be written in registers at probe stage */
static void lsm303c_acc_set_init_register_values(
						struct lsm303c_acc_dev *dev)
{
	memset(dev->resume_state, 0, ARRAY_SIZE(dev->resume_state));

	dev->resume_state[RES_CTRL1] = (ALL_ZEROES |
					CTRL1_HR_DISABLE |
					CTRL1_BDU_ENABLE |
					ACC_ENABLE_ALL_AXES);

	if (dev->pdata->gpio_int1 >= 0)
		dev->resume_state[RES_CTRL3] =
			(dev->resume_state[RES_CTRL3] | CTRL3_IG1_INT1);

	dev->resume_state[RES_CTRL4] = (ALL_ZEROES | CTRL4_IF_ADD_INC_EN);

	dev->resume_state[RES_CTRL5] = (ALL_ZEROES | CTRL5_HLACTIVE_H);

	dev->resume_state[RES_CTRL7] = (ALL_ZEROES | CTRL7_LIR2 | CTRL7_LIR1);

}

static int lsm303c_acc_hw_init(struct lsm303c_acc_dev *dev)
{
	int err = -1;
	u8 buf[7];

#ifdef LSM303C_DEBUG
	pr_info("%s: hw init start\n", LSM303C_ACC_DEV_NAME);
#endif
	err = dev->tf->read(dev->dev, WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(dev->dev,
		"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		dev->hw_working = 1;

	if (buf[0] != WHOAMI_LSM303C_ACC) {
		dev_err(dev->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_LSM303C_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = dev->resume_state[RES_CTRL4];
	err = dev->tf->write(dev->dev, CTRL4, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_FIFO_CTRL];
	err = dev->tf->write(dev->dev, FIFO_CTRL, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_INT_THSX1];
	buf[1] = dev->resume_state[RES_INT_THSY1];
	buf[2] = dev->resume_state[RES_INT_THSZ1];
	buf[3] = dev->resume_state[RES_INT_DUR1];
	err = dev->tf->write(dev->dev, INT_THSX1, 4, buf);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = dev->resume_state[RES_INT_CFG1];
	err = dev->tf->write(dev->dev, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;


	buf[0] = dev->resume_state[RES_CTRL2];
	buf[1] = dev->resume_state[RES_CTRL3];
	err = dev->tf->write(dev->dev, CTRL2, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_CTRL5];
	buf[1] = dev->resume_state[RES_CTRL6];
	buf[2] = dev->resume_state[RES_CTRL7];
	err = dev->tf->write(dev->dev, CTRL5, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_CTRL1];
	err = dev->tf->write(dev->dev, CTRL1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	dev->hw_initialized = 1;
#ifdef LSM303C_DEBUG
	pr_info("%s: hw init done\n", LSM303C_ACC_DEV_NAME);
#endif
	return 0;

err_firstread:
	dev->hw_working = 0;
err_unknown_device:
err_resume_state:
	dev->hw_initialized = 0;
	dev_err(dev->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm303c_acc_device_power_off(struct lsm303c_acc_dev *dev)
{
	int err;
	u8 data = LSM303C_ACC_PM_OFF;

	err = dev->tf->write(dev->dev, CTRL1, 1, &data);
	if (err < 0)
		dev_err(dev->dev, "soft power off failed: %d\n", err);

	if (dev->pdata->power_off) {
		if (dev->pdata->gpio_int1 >= 0)
			disable_irq_nosync(dev->irq1);

		dev->pdata->power_off();
		dev->hw_initialized = 0;
	}
	if (dev->hw_initialized) {
		if (dev->pdata->gpio_int1 >= 0)
			disable_irq_nosync(dev->irq1);

		dev->hw_initialized = 0;
	}

}

static int lsm303c_acc_device_power_on(struct lsm303c_acc_dev *dev)
{
	int err = -1;

	if (dev->pdata->power_on) {
		err = dev->pdata->power_on();
		if (err < 0) {
			dev_err(dev->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if (dev->pdata->gpio_int1 >= 0)
			enable_irq(dev->irq1);
	}

	if (!dev->hw_initialized) {
		err = lsm303c_acc_hw_init(dev);
		if (dev->hw_working == 1 && err < 0) {
			lsm303c_acc_device_power_off(dev);
			return err;
		}
	}

	if (dev->hw_initialized) {
		if (dev->pdata->gpio_int1 >= 0)
			enable_irq(dev->irq1);
	}
	return 0;
}


static int lsm303c_acc_update_fs_range(struct lsm303c_acc_dev *dev,
				       u8 new_fs_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LSM303C_ACC_FS_MASK;

	switch (new_fs_range) {
	case LSM303C_ACC_FS_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LSM303C_ACC_FS_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LSM303C_ACC_FS_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(dev->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	err = dev->tf->read(dev->dev, CTRL4, 1, buf);
	if (err < 0)
		goto error;
	init_val = buf[0];
	dev->resume_state[RES_CTRL4] = init_val;
	new_val = new_fs_range;
	updated_val = ((mask & new_val) | ((~mask) & init_val));

	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, CTRL4, 1, buf);
	if (err < 0)
		goto error;
	dev->resume_state[RES_CTRL4] = updated_val;
	dev->sensitivity = sensitivity;

	return err;
error:
	dev_err(dev->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lsm303c_acc_update_odr(struct lsm303c_acc_dev *dev,
				  int poll_interval_ms)
{
	int err;
	int i;
	u8 config[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = ACC_ODR_MASK;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm303c_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm303c_acc_odr_table[i].cutoff_ms <= poll_interval_ms) ||
		    (i == 0))
			break;
	}
	new_val = lsm303c_acc_odr_table[i].mask;

	/* Updates configuration register 1,
	* which contains odr range setting if enabled,
	* otherwise updates RES_CTRL1 for when it will */
	if (atomic_read(&dev->enabled)) {
		err = dev->tf->read(dev->dev, CTRL1, 1, config);
		if (err < 0)
			goto error;
		init_val = config[0];
		dev->resume_state[RES_CTRL1] = init_val;
		updated_val = ((mask & new_val) | ((~mask) & init_val));

		config[0] = updated_val;
		err = dev->tf->write(dev->dev, CTRL1, 1, config);
		if (err < 0)
			goto error;
		dev->resume_state[RES_CTRL1] = updated_val;
		return err;
	} else {
		init_val = dev->resume_state[RES_CTRL1];
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		dev->resume_state[RES_CTRL1] = updated_val;
		return 0;
	}

error:
	dev_err(dev->dev,
			"update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lsm303c_acc_register_write(struct lsm303c_acc_dev *dev,
				      u8 *buf, u8 reg_address, u8 new_value)
{
	int err;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */

	mutex_lock(&dev->lock);
	err = dev->tf->write(dev->dev, reg_address, 1, &new_value);
	mutex_unlock(&dev->lock);

	return err;
}


static int lsm303c_acc_get_data(struct lsm303c_acc_dev *dev, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3];

	err = dev->tf->read(dev->dev, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * dev->sensitivity;
	hw_d[1] = hw_d[1] * dev->sensitivity;
	hw_d[2] = hw_d[2] * dev->sensitivity;


	xyz[0] = ((dev->pdata->negate_x) ? (-hw_d[dev->pdata->axis_map_x])
		   : (hw_d[dev->pdata->axis_map_x]));
	xyz[1] = ((dev->pdata->negate_y) ? (-hw_d[dev->pdata->axis_map_y])
		   : (hw_d[dev->pdata->axis_map_y]));
	xyz[2] = ((dev->pdata->negate_z) ? (-hw_d[dev->pdata->axis_map_z])
		   : (hw_d[dev->pdata->axis_map_z]));

#ifdef LSM303C_DEBUG

	dev_dbg(dev->dev, "%s read x=%d, y=%d, z=%d\n",
			LSM303C_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

#endif
	return err;
}

static void lsm303c_acc_report_values(struct lsm303c_acc_dev *dev,
					int *xyz)
{
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_sync(dev->input_dev);
}

static void lsm303c_acc_report_triple(struct lsm303c_acc_dev *dev)
{
	int err;
	int xyz[3];

	err = lsm303c_acc_get_data(dev, xyz);
	if (err < 0)
		dev_err(dev->dev, "get_data failed\n");
	else
		lsm303c_acc_report_values(dev, xyz);
}

static irqreturn_t lsm303c_acc_isr1(int irq, void *data)
{
	struct lsm303c_acc_dev *dev = data;

	disable_irq_nosync(irq);
	queue_work(dev->irq1_work_queue, &dev->irq1_work);
	pr_debug("%s: isr1 queued\n", LSM303C_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm303c_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm303c_acc_dev *dev =
	container_of(work, struct lsm303c_acc_dev, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm303c_acc_get_int1_source(dev); */
	/* ; */
	pr_debug("%s: IRQ1 served\n", LSM303C_ACC_DEV_NAME);
/* exit: */
	enable_irq(dev->irq1);
}

int lsm303c_acc_enable(struct lsm303c_acc_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled, 0, 1)) {
		int err;

		mutex_lock(&dev->lock);
		err = lsm303c_acc_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled, 0);
			mutex_unlock(&dev->lock);
			return err;
		}
		dev->polling_ktime = ktime_set(
				dev->pdata->poll_interval / 1000,
				MS_TO_NS(dev->pdata->poll_interval % 1000));
		hrtimer_start(&dev->hr_timer_poll,
			      dev->polling_ktime, HRTIMER_MODE_REL);
		mutex_unlock(&dev->lock);
	}
	return 0;
}
EXPORT_SYMBOL(lsm303c_acc_enable);

int lsm303c_acc_disable(struct lsm303c_acc_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled, 1, 0)) {
		cancel_work_sync(&dev->input_poll_work);

		mutex_lock(&dev->lock);
		lsm303c_acc_device_power_off(dev);
		mutex_unlock(&dev->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm303c_acc_disable);

static ssize_t read_single_reg(struct device *device, char *buf, u8 reg)
{
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	u8 data;
	int err;

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, reg, 1, &data);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}

	mutex_unlock(&dev->lock);

	return sprintf(buf, "0x%02x\n", data);
}

static int write_reg(struct device *device, const char *buf, u8 reg,
						u8 mask, int resume_index)
{
	int err = -1;
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lsm303c_acc_register_write(dev, x, reg, new_val);
	if (err < 0)
		return err;
	dev->resume_state[resume_index] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	mutex_lock(&dev->lock);
	val = dev->pdata->poll_interval;
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
						dev->pdata->min_interval);
	mutex_lock(&dev->lock);
	dev->pdata->poll_interval = interval_ms;
	err = lsm303c_acc_update_odr(dev, interval_ms);
	if (err >= 0) {
		dev->pdata->poll_interval = interval_ms;
		dev->polling_ktime = ktime_set(
				dev->pdata->poll_interval / 1000,
				MS_TO_NS(dev->pdata->poll_interval % 1000));
	}
	mutex_unlock(&dev->lock);
	return size;
}

static ssize_t attr_get_range(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	char range = 2;
	mutex_lock(&dev->lock);
	val = dev->pdata->fs_range;
	switch (val) {
	case LSM303C_ACC_FS_2G:
		range = 2;
		break;
	case LSM303C_ACC_FS_4G:
		range = 4;
		break;
	case LSM303C_ACC_FS_8G:
		range = 8;
		break;
	}
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LSM303C_ACC_FS_2G;
		break;
	case 4:
		range = LSM303C_ACC_FS_4G;
		break;
	case 8:
		range = LSM303C_ACC_FS_8G;
		break;
	default:
		dev_err(dev->dev,
				"invalid range request: %lu, discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&dev->lock);
	err = lsm303c_acc_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata->fs_range = range;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *device,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	int val = atomic_read(&dev->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303c_acc_enable(dev);
	else
		lsm303c_acc_disable(dev);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *device,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(device, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *device,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(device, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *device,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(device, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *device,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(device, buf, INT_DUR1);
}

static ssize_t attr_set_threshx1(struct device *device,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(device, buf, INT_THSX1,
					INT1_THRESHOLD_MASK, RES_INT_THSX1);
}

static ssize_t attr_get_threshx1(struct device *device,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(device, buf, INT_THSX1);
}

static ssize_t attr_set_threshy1(struct device *device,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(device, buf, INT_THSY1,
					INT1_THRESHOLD_MASK, RES_INT_THSY1);
}

static ssize_t attr_get_threshy1(struct device *device,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(device, buf, INT_THSY1);
}

static ssize_t attr_set_threshz1(struct device *device,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(device, buf, INT_THSZ1,
					INT1_THRESHOLD_MASK, RES_INT_THSZ1);
}

static ssize_t attr_get_threshz1(struct device *device,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(device, buf, INT_THSZ1);
}

static ssize_t attr_get_source1(struct device *device,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(device, buf, INT_SRC1);
}


#ifdef LSM303C_DEBUG
/* PAY ATTENTION: These LSM303C_DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *device, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	rc = dev->tf->write(dev->dev, dev->reg_addr, 1, &(u8)val);
	mutex_unlock(&dev->lock);

	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *device, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	int rc;
	u8 data;

	mutex_lock(&dev->lock);
	rc = dev->tf->read(dev->dev, dev->reg_addr, 1, &data);
	mutex_unlock(&dev->lock);

	/*TODO: error need to be managed */
	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *device, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303c_acc_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	dev->reg_addr = val;
	mutex_unlock(&dev->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0666, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_thresholdx, 0664, attr_get_threshx1, attr_set_threshx1),
	__ATTR(int1_thresholdy, 0664, attr_get_threshy1, attr_set_threshy1),
	__ATTR(int1_thresholdz, 0664, attr_get_threshz1, attr_set_threshz1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
#ifdef LSM303C_DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
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

static int remove_sysfs_interfaces(struct device *device)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(device, attributes + i);
	return 0;
}

static void lsm303c_acc_input_poll_work_func(struct work_struct *work)
{
	struct lsm303c_acc_dev *dev;

	dev = container_of((struct work_struct *) work,
			   struct lsm303c_acc_dev, input_poll_work);

	mutex_lock(&dev->lock);
	lsm303c_acc_report_triple(dev);
	mutex_unlock(&dev->lock);

	if (atomic_read(&dev->enabled))
		hrtimer_start(&dev->hr_timer_poll,
					dev->polling_ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart lsm303c_acc_hr_timer_poll_function(struct hrtimer *timer)
{
	struct lsm303c_acc_dev *dev;

	dev = container_of((struct hrtimer *)timer,
			   struct lsm303c_acc_dev, hr_timer_poll);

	queue_work(dev->hr_timer_poll_work_queue, &dev->input_poll_work);
	return HRTIMER_NORESTART;
}

static int lsm303c_acc_validate_pdata(struct lsm303c_acc_dev *dev)
{
	/* checks for correctness of minimal polling period */
	dev->pdata->min_interval =
		max((unsigned int)LSM303C_ACC_MIN_POLL_PERIOD_MS,
						dev->pdata->min_interval);

	dev->pdata->poll_interval = max(dev->pdata->poll_interval,
			dev->pdata->min_interval);

	if (dev->pdata->axis_map_x > 2 ||
		dev->pdata->axis_map_y > 2 ||
		 dev->pdata->axis_map_z > 2) {
		dev_err(dev->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
						dev->pdata->axis_map_x,
						dev->pdata->axis_map_y,
						dev->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (dev->pdata->negate_x > 1 || dev->pdata->negate_y > 1
			|| dev->pdata->negate_z > 1) {
		dev_err(dev->dev,
			"invalid negate value x:%u y:%u z:%u\n",
						dev->pdata->negate_x,
						dev->pdata->negate_y,
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

static int lsm303c_acc_input_init(struct lsm303c_acc_dev *dev)
{
	int err;

	INIT_WORK(&dev->input_poll_work, lsm303c_acc_input_poll_work_func);
	dev->input_dev = input_allocate_device();
	if (!dev->input_dev) {
		err = -ENOMEM;
		dev_err(dev->dev, "input device allocation failed\n");
		goto err0;
	}

	dev->input_dev->name = LSM303C_ACC_DEV_NAME;
	dev->input_dev->id.bustype = dev->bus_type;
	dev->input_dev->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev, dev);

	/* Set the input event characteristics of the probed sensor driver */
	set_bit(INPUT_EVENT_TYPE, dev->input_dev->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev->mscbit);

	err = input_register_device(dev->input_dev);
	if (err) {
		dev_err(dev->dev,
				"unable to register input device %s\n",
				dev->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(dev->input_dev);
err0:
	return err;
}

static void lsm303c_acc_input_cleanup(struct lsm303c_acc_dev *dev)
{
	input_unregister_device(dev->input_dev);
	input_free_device(dev->input_dev);
}

int lsm303c_acc_probe(struct lsm303c_acc_dev *dev)
{
	int err = -1;

	mutex_lock(&dev->lock);

	dev->pdata = kmalloc(sizeof(*dev->pdata), GFP_KERNEL);
	if (dev->pdata == NULL) {
		err = -ENOMEM;
		dev_err(dev->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (dev->dev->platform_data == NULL) {
		default_lsm303c_acc_pdata.gpio_int1 = int1_gpio;

		memcpy(dev->pdata, &default_lsm303c_acc_pdata,
		       sizeof(*dev->pdata));
		dev_info(dev->dev, "using default plaform_data\n");
	} else {
		memcpy(dev->pdata, dev->dev->platform_data,
		       sizeof(*dev->pdata));
	}
	dev->hr_timer_poll_work_queue = 0;

	err = lsm303c_acc_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (dev->pdata->init) {
		err = dev->pdata->init();
		if (err < 0) {
			dev_err(dev->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if (dev->pdata->gpio_int1 >= 0) {
		dev->irq1 = gpio_to_irq(dev->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, mapped on gpio:%d\n",
			LSM303C_ACC_DEV_NAME, __func__, dev->irq1,
							dev->pdata->gpio_int1);
	}

	lsm303c_acc_set_init_register_values(dev);

	err = lsm303c_acc_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&dev->enabled, 1);

	err = lsm303c_acc_update_fs_range(dev, dev->pdata->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm303c_acc_update_odr(dev, dev->pdata->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	dev->hr_timer_poll_work_queue =
			create_workqueue("lsm303c_acc_hr_timer_poll_wq");
	hrtimer_init(&dev->hr_timer_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer_poll.function = &lsm303c_acc_hr_timer_poll_function;

	err = lsm303c_acc_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "input init failed\n");
		goto err_remove_hr_work_queue;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev,
		   "device LSM303C_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm303c_acc_device_power_off(dev);

	/* As default, do not report information */
	atomic_set(&dev->enabled, 0);

	if (dev->pdata->gpio_int1 >= 0) {
		INIT_WORK(&dev->irq1_work, lsm303c_acc_irq1_work_func);
		dev->irq1_work_queue =
			create_singlethread_workqueue("lsm303c_acc_wq1");
		if (!dev->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(dev->irq1, lsm303c_acc_isr1,
			IRQF_TRIGGER_RISING, "lsm303c_acc_irq1", dev);
		if (err < 0) {
			dev_err(dev->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(dev->irq1);
	}

	mutex_unlock(&dev->lock);

	return 0;

err_destoyworkqueue1:
	if (dev->pdata->gpio_int1 >= 0)
		destroy_workqueue(dev->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(dev->dev);
err_input_cleanup:
	lsm303c_acc_input_cleanup(dev);
err_remove_hr_work_queue:
	if (dev->hr_timer_poll_work_queue)
		destroy_workqueue(dev->hr_timer_poll_work_queue);
err_power_off:
	lsm303c_acc_device_power_off(dev);
err_pdata_init:
	if (dev->pdata->exit)
		dev->pdata->exit();
exit_kfree_pdata:
	kfree(dev->pdata);
err_mutexunlock:
	mutex_unlock(&dev->lock);
/* err_freedata: */

	return err;
}
EXPORT_SYMBOL(lsm303c_acc_probe);

int lsm303c_acc_remove(struct lsm303c_acc_dev *dev)
{
	if (dev->pdata->gpio_int1 >= 0) {
		free_irq(dev->irq1, dev);
		gpio_free(dev->pdata->gpio_int1);
		destroy_workqueue(dev->irq1_work_queue);
	}

	lsm303c_acc_disable(dev);
	lsm303c_acc_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	if (dev->hr_timer_poll_work_queue)
		destroy_workqueue(dev->hr_timer_poll_work_queue);

	if (dev->pdata->exit)
		dev->pdata->exit();
	kfree(dev->pdata);

	return 0;
}
EXPORT_SYMBOL(lsm303c_acc_remove);

MODULE_DESCRIPTION("lsm303c accelerometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");

