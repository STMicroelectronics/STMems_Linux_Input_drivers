/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name          : lsm330dlc_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Lorenzo Bianconi (lorenzo.bianconi@st.com)
 *		      : Authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.12
 * Date               : 2016/May/6
 * Description        : LSM330DLC accelerometer sensor API
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
 ******************************************************************************
 Revision 1.0.6 15/11/2010
  first revision
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9: 2011/May/23
  update_odr func correction;
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
 ******************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
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

#include "lsm330dlc.h"

/* mg/LSB */
#define SENSITIVITY_2G		1
#define SENSITIVITY_4G		2
#define SENSITIVITY_8G		4
#define SENSITIVITY_16G		12

/* Accelerometer Sensor Operating Mode */
#define LSM330DLC_ACC_ENABLE	0x01
#define LSM330DLC_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LSM330DLC_ACC	0x33

/* CONTROL REGISTERS */
#define WHO_AM_I		0x0F
#define TEMP_CFG_REG		0x1F
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define CTRL_REG6		0x25

#define FIFO_CTRL_REG		0x2E

#define INT_CFG1		0x30
#define INT_SRC1		0x31
#define INT_THS1		0x32
#define INT_DUR1		0x33

#define TT_CFG			0x38
#define TT_SRC			0x39
#define TT_THS			0x3A
#define TT_LIM			0x3B
#define TT_TLAT			0x3C
#define TT_TW			0x3D

#define ENABLE_HIGH_RESOLUTION	1

#define LSM330DLC_ACC_PM_OFF		0x00
#define LSM330DLC_ACC_ENABLE_ALL_AXES	0x07

#define PMODE_MASK		0x08
#define ODR_MASK		0XF0

#define LSM330DLC_ACC_ODR1	0x10
#define LSM330DLC_ACC_ODR10	0x20
#define LSM330DLC_ACC_ODR25	0x30
#define LSM330DLC_ACC_ODR50	0x40
#define LSM330DLC_ACC_ODR100	0x50
#define LSM330DLC_ACC_ODR200	0x60
#define LSM330DLC_ACC_ODR400	0x70
#define LSM330DLC_ACC_ODR1250	0x90

#define IA			0x40
#define ZH			0x20
#define ZL			0x10
#define YH			0x08
#define YL			0x04
#define XH			0x02
#define XL			0x01

/* CTRL REG BITS*/
#define CTRL_REG3_I1_AOI1	0x40
#define CTRL_REG4_BDU_ENABLE	0x80
#define CTRL_REG4_BDU_MASK	0x80
#define CTRL_REG6_I2_TAPEN	0x80
#define CTRL_REG6_HLACTIVE	0x02

#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define TAP_THS_MASK		0x7F
#define TAP_TLIM_MASK		0x7F
#define TAP_TLAT_MASK		NO_MASK
#define TAP_TW_MASK		NO_MASK

/* TAP_SOURCE_REG BIT */
#define DTAP			0x20
#define STAP			0x10
#define SIGNTAP			0x08
#define ZTAP			0x04
#define YTAP			0x02
#define XTAZ			0x01

#define FUZZ			0
#define FLAT			0

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define LSM330DLC_ACC_MIN_POLL_PERIOD_MS	1

/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones havew to be set to -EINVAL
 */
#define LSM330DLC_ACC_DEFAULT_INT1_GPIO		-EINVAL
#define LSM330DLC_ACC_DEFAULT_INT2_GPIO		-EINVAL

/* Accelerometer Sensor Full Scale */
#define LSM330DLC_ACC_FS_MASK		0x30
#define LSM330DLC_ACC_G_2G		0x00
#define LSM330DLC_ACC_G_4G		0x10
#define LSM330DLC_ACC_G_8G		0x20
#define LSM330DLC_ACC_G_16G		0x30

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330dlc_acc_odr_table[] = {
	{    1, LSM330DLC_ACC_ODR1250 },
	{    3, LSM330DLC_ACC_ODR400  },
	{    5, LSM330DLC_ACC_ODR200  },
	{   10, LSM330DLC_ACC_ODR100  },
	{   20, LSM330DLC_ACC_ODR50   },
	{   40, LSM330DLC_ACC_ODR25   },
	{  100, LSM330DLC_ACC_ODR10   },
	{ 1000, LSM330DLC_ACC_ODR1    },
};

static int int1_gpio = LSM330DLC_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330DLC_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

static struct lsm330dlc_acc_platform_data default_lsm330dlc_acc_pdata = {
	.fs_range = LSM330DLC_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM330DLC_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM330DLC_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330DLC_ACC_DEFAULT_INT2_GPIO,
};

static int lsm330dlc_acc_hw_init(struct lsm330dlc_acc_dev *dev)
{
	int err;
	u8 buf[7];

	err = dev->tf->read(dev->dev, WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(dev->dev, "Error reading WHO_AM_I\n");
		goto err_firstread;
	} else {
		dev->hw_working = 1;
	}

	if (buf[0] != WHOAMI_LSM330DLC_ACC) {
		dev_err(dev->dev, "device unknown. [0x%02x 0x%02x]\n",
			WHOAMI_LSM330DLC_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = dev->resume_state[RES_CTRL_REG1];
	err = dev->tf->write(dev->dev, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_TEMP_CFG_REG];
	err = dev->tf->write(dev->dev, TEMP_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_FIFO_CTRL_REG];
	err = dev->tf->write(dev->dev, FIFO_CTRL_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_TT_THS];
	buf[1] = dev->resume_state[RES_TT_LIM];
	buf[2] = dev->resume_state[RES_TT_TLAT];
	buf[3] = dev->resume_state[RES_TT_TW];
	err = dev->tf->write(dev->dev, TT_THS, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_TT_CFG];
	err = dev->tf->write(dev->dev, TT_CFG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_INT_THS1];
	buf[1] = dev->resume_state[RES_INT_DUR1];
	err = dev->tf->write(dev->dev, INT_THS1, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_INT_CFG1];
	err = dev->tf->write(dev->dev, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = dev->resume_state[RES_CTRL_REG2];
	buf[1] = dev->resume_state[RES_CTRL_REG3];
	buf[2] = dev->resume_state[RES_CTRL_REG4];
	buf[3] = dev->resume_state[RES_CTRL_REG5];
	buf[4] = dev->resume_state[RES_CTRL_REG6];
	err = dev->tf->write(dev->dev, CTRL_REG2, 5, buf);
	if (err < 0)
		goto err_resume_state;

	dev->hw_initialized = 1;

	return 0;

err_firstread:
	dev->hw_working = 0;
err_unknown_device:
err_resume_state:
	dev->hw_initialized = 0;

	return err;
}

static void lsm330dlc_acc_device_power_off(struct lsm330dlc_acc_dev *dev)
{
	int err;
	u8 data = LSM330DLC_ACC_PM_OFF;

	err = dev->tf->write(dev->dev, CTRL_REG1, 1, &data);
	if (err < 0)
		dev_err(dev->dev, "soft power off failed: %d\n", err);

	if (dev->pdata->power_off) {
		if (dev->pdata->gpio_int1 >= 0)
			disable_irq_nosync(dev->irq1);
		if (dev->pdata->gpio_int2 >= 0)
			disable_irq_nosync(dev->irq2);
		dev->pdata->power_off();
		dev->hw_initialized = 0;
	}

	if (dev->hw_initialized) {
		if (dev->pdata->gpio_int1 >= 0)
			disable_irq_nosync(dev->irq1);
		if (dev->pdata->gpio_int2 >= 0)
			disable_irq_nosync(dev->irq2);
		dev->hw_initialized = 0;
	}

}

static int lsm330dlc_acc_device_power_on(struct lsm330dlc_acc_dev *dev)
{
	if (dev->pdata->power_on) {
		int err = dev->pdata->power_on();
		if (err < 0) {
			dev_err(dev->dev, "power_on failed: %d\n", err);
			return err;
		}

		if (dev->pdata->gpio_int1 >= 0)
			enable_irq(dev->irq1);
		if (dev->pdata->gpio_int2 >= 0)
			enable_irq(dev->irq2);
	}

	if (!dev->hw_initialized) {
		int err = lsm330dlc_acc_hw_init(dev);
		if (dev->hw_working == 1 && err < 0) {
			lsm330dlc_acc_device_power_off(dev);
			return err;
		}
	}

	if (dev->hw_initialized) {
		if (dev->pdata->gpio_int1 >= 0)
			enable_irq(dev->irq1);
		if (dev->pdata->gpio_int2 >= 0)
			enable_irq(dev->irq2);
	}

	return 0;
}

static irqreturn_t lsm330dlc_acc_isr1(int irq, void *data)
{
	struct lsm330dlc_acc_dev *dev = data;

	disable_irq_nosync(irq);
	queue_work(dev->irq1_work_queue, &dev->irq1_work);

	return IRQ_HANDLED;
}

static irqreturn_t lsm330dlc_acc_isr2(int irq, void *data)
{
	struct lsm330dlc_acc_dev *dev = data;

	disable_irq_nosync(irq);
	queue_work(dev->irq2_work_queue, &dev->irq2_work);

	return IRQ_HANDLED;
}

static void lsm330dlc_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm330dlc_acc_dev *dev;
	
	dev = container_of(work, struct lsm330dlc_acc_dev, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330dlc_acc_get_int1_source(dev); */

	enable_irq(dev->irq1);
}

static void lsm330dlc_acc_irq2_work_func(struct work_struct *work)
{

	struct lsm330dlc_acc_dev *dev;
	
	dev = container_of(work, struct lsm330dlc_acc_dev, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330dlc_acc_get_tap_source(dev); */

	enable_irq(dev->irq2);
}

static int lsm330dlc_acc_update_fs_range(struct lsm330dlc_acc_dev *dev,
					 u8 new_fs_range)
{
	int err;
	u8 data, sensitivity, updated_val;
	u8 mask = LSM330DLC_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_fs_range) {
	case LSM330DLC_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LSM330DLC_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LSM330DLC_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LSM330DLC_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(dev->dev, "invalid fs range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	/* Updates configuration register 4,
	 * which contains fs range setting */
	err = dev->tf->read(dev->dev, CTRL_REG4, 1, &data);
	if (err < 0)
		return err;

	dev->resume_state[RES_CTRL_REG4] = data;
	updated_val = ((mask & (new_fs_range | HIGH_RESOLUTION)) |
		       (~mask & data));
	err = dev->tf->write(dev->dev, CTRL_REG4, 1, &updated_val);
	if (err < 0)
		return err;

	dev->resume_state[RES_CTRL_REG4] = updated_val;
	dev->sensitivity = sensitivity;

	return 0;
}

static int lsm330dlc_acc_update_odr(struct lsm330dlc_acc_dev *dev,
				    int poll_ms)
{
	if (atomic_read(&dev->enabled)) {
		int i, err;
		u8 data = LSM330DLC_ACC_ENABLE_ALL_AXES;

		/* Following, looks for the longest possible odr interval
		 * scrolling the odr_table vector from the end
		 * (shortest interval) backward (longest interval), to support
		 * the poll_interval requested by the system.
		 * It must be the longest interval lower then the poll interval
		 */
		for (i = ARRAY_SIZE(lsm330dlc_acc_odr_table) - 1; i >= 0; i--) {
			if ((lsm330dlc_acc_odr_table[i].cutoff_ms <= poll_ms) ||
			    (i == 0))
				break;
		}
		data |= lsm330dlc_acc_odr_table[i].mask;

		/* If device is currently enabled, we need to write new
		 * configuration out to it */
		err = dev->tf->write(dev->dev, CTRL_REG1, 1, &data);
		if (err < 0)
			return err;
		dev->resume_state[RES_CTRL_REG1] = data;
	}

	return 0;
}

static int lsm330dlc_acc_get_acceleration_data(struct lsm330dlc_acc_dev *dev,
					       int *xyz)
{
	int err;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3];

	err = dev->tf->read(dev->dev, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] *= dev->sensitivity;
	hw_d[1] *= dev->sensitivity;
	hw_d[2] *= dev->sensitivity;

	xyz[0] = ((dev->pdata->negate_x) ? (-hw_d[dev->pdata->axis_map_x])
		   : (hw_d[dev->pdata->axis_map_x]));
	xyz[1] = ((dev->pdata->negate_y) ? (-hw_d[dev->pdata->axis_map_y])
		   : (hw_d[dev->pdata->axis_map_y]));
	xyz[2] = ((dev->pdata->negate_z) ? (-hw_d[dev->pdata->axis_map_z])
		   : (hw_d[dev->pdata->axis_map_z]));

	return err;
}

static void lsm330dlc_acc_report_values(struct lsm330dlc_acc_dev *dev,
					int *xyz)
{
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(dev->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(dev->input_dev);
}

int lsm330dlc_acc_enable(struct lsm330dlc_acc_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled, 0, 1)) {
		int err;

		mutex_lock(&dev->lock);
		err = lsm330dlc_acc_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled, 0);
			mutex_unlock(&dev->lock);
			return err;
		}
		schedule_delayed_work(&dev->input_work,
			msecs_to_jiffies(dev->pdata->poll_interval));
		mutex_unlock(&dev->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm330dlc_acc_enable);

int lsm330dlc_acc_disable(struct lsm330dlc_acc_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled, 1, 0)) {
		cancel_delayed_work_sync(&dev->input_work);

		mutex_lock(&dev->lock);
		lsm330dlc_acc_device_power_off(dev);
		mutex_unlock(&dev->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm330dlc_acc_disable);

static ssize_t read_single_reg(struct device *device, char *buf, u8 reg)
{
	u8 data;
	ssize_t ret;
	int err;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	err = dev->tf->read(device, reg, 1, &data);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	mutex_unlock(&dev->lock);

	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static int write_reg(struct device *device, const char *buf, u8 reg, u8 mask,
		     int resumeIndex)
{
	int err;
	u8 data;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_unlock(&dev->lock);
	data = ((u8)val & mask);
	err = dev->tf->write(device, reg, 1, &data);
	mutex_unlock(&dev->lock);

	if (err < 0)
		return err;

	dev->resume_state[resumeIndex] = data;

	return err;
}

static ssize_t attr_get_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long interval_ms;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, dev->pdata->min_interval);

	mutex_lock(&dev->lock);
	dev->pdata->poll_interval = interval_ms;
	lsm330dlc_acc_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_range(struct device *device,
			      struct device_attribute *attr, char *buf)
{
	char range = 2;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	switch (dev->pdata->fs_range) {
	case LSM330DLC_ACC_G_2G:
		range = 2;
		break;
	case LSM330DLC_ACC_G_4G:
		range = 4;
		break;
	case LSM330DLC_ACC_G_8G:
		range = 8;
		break;
	case LSM330DLC_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	unsigned long val;
	int err;
	u8 range;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM330DLC_ACC_G_2G;
		break;
	case 4:
		range = LSM330DLC_ACC_G_4G;
		break;
	case 8:
		range = LSM330DLC_ACC_G_8G;
		break;
	case 16:
		range = LSM330DLC_ACC_G_16G;
		break;
	default:
		dev_err(device, "invalid range request: %lu\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lsm330dlc_acc_update_fs_range(dev, range);
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
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", atomic_read(&dev->enabled));
}

static ssize_t attr_set_enable(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	unsigned long val;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330dlc_acc_enable(dev);
	else
		lsm330dlc_acc_disable(dev);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

#ifdef LSM330DLC_DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int err;
	unsigned long val;
	u8 data;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	/*TODO: error need to be managed */
	mutex_lock(&dev->lock);
	data = val;
	err = dev->tf->write(device, dev->reg_addr, 1, &data);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_reg_get(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	int err;
	u8 data;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	/*TODO: error need to be managed */
	mutex_lock(&dev->lock);
	err = dev->tf->read(device, dev->reg_addr, 1, &data);
	mutex_unlock(&dev->lock);

	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *device,
			     struct device_attribute *attr,
			     const char *buf, size_t size)
{
	unsigned long val;
	struct lsm330dlc_acc_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&dev->lock);
	dev->reg_addr = val;
	mutex_unlock(&dev->lock);

	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate,
	       attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim, attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
	       attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),

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
	for ( ; i >= 0; i--)
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

static void lsm330dlc_acc_input_work_func(struct work_struct *work)
{
	struct lsm330dlc_acc_dev *dev;
	int err, xyz[3] = {};

	dev = container_of((struct delayed_work *)work,
			   struct lsm330dlc_acc_dev, input_work);

	mutex_lock(&dev->lock);
	err = lsm330dlc_acc_get_acceleration_data(dev, xyz);
	if (err < 0)
		dev_err(dev->dev, "get_acceleration_data failed\n");
	else
		lsm330dlc_acc_report_values(dev, xyz);

	schedule_delayed_work(&dev->input_work,
			      msecs_to_jiffies(dev->pdata->poll_interval));

	mutex_unlock(&dev->lock);
}

static int lsm330dlc_acc_validate_pdata(struct lsm330dlc_acc_dev *dev)
{
	/* checks for correctness of minimal polling period */
	dev->pdata->min_interval = max((u32)LSM330DLC_ACC_MIN_POLL_PERIOD_MS,
				       dev->pdata->min_interval);

	dev->pdata->poll_interval = max(dev->pdata->poll_interval,
					dev->pdata->min_interval);

	if (dev->pdata->axis_map_x > 2 || dev->pdata->axis_map_y > 2 ||
	    dev->pdata->axis_map_z > 2) {
		dev_err(dev->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", dev->pdata->axis_map_x,
			dev->pdata->axis_map_y, dev->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (dev->pdata->negate_x > 1 || dev->pdata->negate_y > 1 ||
	    dev->pdata->negate_z > 1) {
		dev_err(dev->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", dev->pdata->negate_x,
			dev->pdata->negate_y, dev->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (dev->pdata->poll_interval < dev->pdata->min_interval) {
		dev_err(dev->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330dlc_acc_input_init(struct lsm330dlc_acc_dev *dev)
{
	int err;

	INIT_DELAYED_WORK(&dev->input_work, lsm330dlc_acc_input_work_func);

	dev->input_dev = input_allocate_device();
	if (!dev->input_dev) {
		dev_err(dev->dev, "input device allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev->name = LSM330DLC_ACC_DEV_NAME;
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
		dev_err(dev->dev, "unable to register input device %s\n",
			dev->input_dev->name);
		input_free_device(dev->input_dev);
		return err;
	}

	return 0;
}

static void lsm330dlc_acc_input_cleanup(struct lsm330dlc_acc_dev *dev)
{
	input_unregister_device(dev->input_dev);
	input_free_device(dev->input_dev);
}

int lsm330dlc_acc_probe(struct lsm330dlc_acc_dev *dev)
{
	int err;

	mutex_lock(&dev->lock);

	dev->pdata = kmalloc(sizeof(*dev->pdata), GFP_KERNEL);
	if (dev->pdata == NULL) {
		err = -ENOMEM;
		dev_err(dev->dev, "failed to allocate memory for pdata: %d\n",
			err);
		goto err_mutexunlock;
	}

	if (dev->dev->platform_data == NULL) {
		default_lsm330dlc_acc_pdata.gpio_int1 = int1_gpio;
		default_lsm330dlc_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(dev->pdata, &default_lsm330dlc_acc_pdata,
		       sizeof(*dev->pdata));
		dev_info(dev->dev, "using default plaform_data\n");
	} else {
		memcpy(dev->pdata, dev->dev->platform_data,
		       sizeof(*dev->pdata));
	}

	err = lsm330dlc_acc_validate_pdata(dev);
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
			LSM330DLC_ACC_DEV_NAME, __func__, dev->irq1,
			dev->pdata->gpio_int1);
	}

	if (dev->pdata->gpio_int2 >= 0) {
		dev->irq2 = gpio_to_irq(dev->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, mapped on gpio:%d\n",
			LSM330DLC_ACC_DEV_NAME, __func__, dev->irq2,
			dev->pdata->gpio_int2);
	}

	dev->resume_state[RES_CTRL_REG1] = LSM330DLC_ACC_ENABLE_ALL_AXES;
	dev->resume_state[RES_CTRL_REG4] = CTRL_REG4_BDU_ENABLE;

	err = lsm330dlc_acc_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&dev->enabled, 1);

	err = lsm330dlc_acc_update_fs_range(dev, dev->pdata->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330dlc_acc_update_odr(dev, dev->pdata->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm330dlc_acc_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "device sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm330dlc_acc_device_power_off(dev);

	/* As default, do not report information */
	atomic_set(&dev->enabled, 0);

	if (dev->pdata->gpio_int1 >= 0) {
		INIT_WORK(&dev->irq1_work, lsm330dlc_acc_irq1_work_func);
		dev->irq1_work_queue =
			create_singlethread_workqueue("lsm330dlc_acc_wq1");
		if (!dev->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev, "cannot create work queue1: %d\n",
				err);
			goto err_remove_sysfs_int;
		}

		err = request_irq(dev->irq1, lsm330dlc_acc_isr1,
				  IRQF_TRIGGER_RISING, "lsm330dlc_acc_irq1",
				  dev);
		if (err < 0) {
			dev_err(dev->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(dev->irq1);
	}

	if (dev->pdata->gpio_int2 >= 0) {
		INIT_WORK(&dev->irq2_work, lsm330dlc_acc_irq2_work_func);
		dev->irq2_work_queue =
			create_singlethread_workqueue("lsm330dlc_acc_wq2");
		if (!dev->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev, "cannot create work queue2: %d\n",
				err);
			goto err_free_irq1;
		}
		err = request_irq(dev->irq2, lsm330dlc_acc_isr2,
				  IRQF_TRIGGER_RISING, "lsm330dlc_acc_irq2",
				  dev);
		if (err < 0) {
			dev_err(dev->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(dev->irq2);
	}

	mutex_unlock(&dev->lock);

	return 0;

err_destoyworkqueue2:
	if (dev->pdata->gpio_int2 >= 0)
		destroy_workqueue(dev->irq2_work_queue);
err_free_irq1:
	free_irq(dev->irq1, dev);
err_destoyworkqueue1:
	if (dev->pdata->gpio_int1 >= 0)
		destroy_workqueue(dev->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(dev->dev);
err_input_cleanup:
	lsm330dlc_acc_input_cleanup(dev);
err_power_off:
	lsm330dlc_acc_device_power_off(dev);
err_pdata_init:
	if (dev->pdata->exit)
		dev->pdata->exit();
exit_kfree_pdata:
	kfree(dev->pdata);
err_mutexunlock:
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(lsm330dlc_acc_probe);

int lsm330dlc_acc_remove(struct lsm330dlc_acc_dev *dev)
{
	if (dev->pdata->gpio_int1 >= 0) {
		free_irq(dev->irq1, dev);
		gpio_free(dev->pdata->gpio_int1);
		destroy_workqueue(dev->irq1_work_queue);
	}

	if (dev->pdata->gpio_int2 >= 0) {
		free_irq(dev->irq2, dev);
		gpio_free(dev->pdata->gpio_int2);
		destroy_workqueue(dev->irq2_work_queue);
	}

	lsm330dlc_acc_input_cleanup(dev);
	lsm330dlc_acc_device_power_off(dev);
	remove_sysfs_interfaces(dev->dev);

	if (dev->pdata->exit)
		dev->pdata->exit();
	kfree(dev->pdata);

	return 0;
}
EXPORT_SYMBOL(lsm330dlc_acc_remove);

MODULE_DESCRIPTION("lsm330dlc accelerometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Carmine Iascone, STMicroelectronics");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");

