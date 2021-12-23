/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name	: lsm303dlhc_acc.c
 * Authors	: MSH - Motion Mems BU - Application Team
 *		: Matteo Dameno (matteo.dameno@st.com)
 *		: Denis Ciocca (denis.ciocca@st.com)
 *		: Lorenzo Bianconi (lorenzo.bianconi@st.com)
 *		: Authors are willing to be considered the contact
 *		: and update points for the driver.
 * Version	: V.1.0.13
 * Date		: 2016/May/19
 * Description	: LSM303DLHC accelerometer sensor API
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
 Revision 1.0.12.1: 2012/May/29
  modified some defines;
 Revision 1.0.13: 2012/Jun/30
  mag: corrects saturation code management;
  mag: changes sysfs range input commands to decimal mGauss;
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

#include "lsm303dlhc.h"


#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LSM303DLHC_ACC_ENABLE	(0x01)
#define LSM303DLHC_ACC_DISABLE	(0x00)

#define	HIGH_RESOLUTION		(0x08)

#define	AXISDATA_REG		(0x28)
#define WHOAMI_LSM303DLHC_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/

#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/


#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/

#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		(0x00)

#define LSM303DLHC_ACC_PM_OFF		(0x00)
#define LSM303DLHC_ACC_ENABLE_ALL_AXES	(0x07)

#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define LSM303DLHC_ACC_ODR1	(0x10)  /* 1Hz output data rate */
#define LSM303DLHC_ACC_ODR10	(0x20)  /* 10Hz output data rate */
#define LSM303DLHC_ACC_ODR25	(0x30)  /* 25Hz output data rate */
#define LSM303DLHC_ACC_ODR50	(0x40)  /* 50Hz output data rate */
#define LSM303DLHC_ACC_ODR100	(0x50)  /* 100Hz output data rate */
#define LSM303DLHC_ACC_ODR200	(0x60)  /* 200Hz output data rate */
#define LSM303DLHC_ACC_ODR400	(0x70)  /* 400Hz output data rate */
#define LSM303DLHC_ACC_ODR1250	(0x90)  /* 1250Hz output data rate */


#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)

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

/* end RESUME STATE INDICES */

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm303dlhc_acc_odr_table[] = {
	{    1, LSM303DLHC_ACC_ODR1250 },
	{    3, LSM303DLHC_ACC_ODR400  },
	{    5, LSM303DLHC_ACC_ODR200  },
	{   10, LSM303DLHC_ACC_ODR100  },
	{   20, LSM303DLHC_ACC_ODR50   },
	{   40, LSM303DLHC_ACC_ODR25   },
	{  100, LSM303DLHC_ACC_ODR10   },
	{ 1000, LSM303DLHC_ACC_ODR1    },
};

static int int1_gpio = LSM303DLHC_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM303DLHC_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

static struct lsm303dlhc_acc_platform_data
						default_lsm303dlhc_acc_pdata = {
	.fs_range = LSM303DLHC_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM303DLHC_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM303DLHC_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM303DLHC_ACC_DEFAULT_INT2_GPIO,
};

static int lsm303dlhc_acc_hw_init(struct lsm303dlhc_acc_status *stat)
{
	int err = -1;
	u8 buf[5];

	pr_info("%s: hw init start\n", LSM303DLHC_ACC_DEV_NAME);

	err = stat->tf->read(stat->dev, WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(stat->dev, "Error reading WHO_AM_I: is "
				"device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LSM303DLHC_ACC) {
		dev_err(stat->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n", WHOAMI_LSM303DLHC_ACC, buf[0]);
			err = -1; /* choose the right coded error */
			goto err_unknown_device;
	}


	buf[0] = stat->resume_state[RES_CTRL_REG1];
	err = stat->tf->write(stat->dev, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TEMP_CFG_REG];
	err = stat->tf->write(stat->dev, TEMP_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = stat->tf->write(stat->dev, FIFO_CTRL_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TT_THS];
	buf[1] = stat->resume_state[RES_TT_LIM];
	buf[2] = stat->resume_state[RES_TT_TLAT];
	buf[3] = stat->resume_state[RES_TT_TW];
	err = stat->tf->write(stat->dev, TT_THS, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TT_CFG];
	err = stat->tf->write(stat->dev, TT_CFG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_THS1];
	buf[1] = stat->resume_state[RES_INT_DUR1];
	err = stat->tf->write(stat->dev, INT_THS1, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_CFG1];
	err = stat->tf->write(stat->dev, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;


	buf[0] = stat->resume_state[RES_CTRL_REG2];
	buf[1] = stat->resume_state[RES_CTRL_REG3];
	buf[2] = stat->resume_state[RES_CTRL_REG4];
	buf[3] = stat->resume_state[RES_CTRL_REG5];
	buf[4] = stat->resume_state[RES_CTRL_REG6];
	err = stat->tf->write(stat->dev, CTRL_REG2, 5, buf);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM303DLHC_ACC_DEV_NAME);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(stat->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm303dlhc_acc_device_power_off(struct lsm303dlhc_acc_status *stat)
{
	int err;
	u8 buf[1] = { LSM303DLHC_ACC_PM_OFF };

	err = stat->tf->write(stat->dev, CTRL_REG1, 1, buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		if(stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if(stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}
	if (stat->hw_initialized) {
		if(stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if(stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->hw_initialized = 0;
	}

}

static int lsm303dlhc_acc_device_power_on(struct lsm303dlhc_acc_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(stat->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if(stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if(stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}

	if (!stat->hw_initialized) {
		err = lsm303dlhc_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lsm303dlhc_acc_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		if(stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if(stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}
	return 0;
}

static irqreturn_t lsm303dlhc_acc_isr1(int irq, void *dev)
{
	struct lsm303dlhc_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	pr_debug("%s: isr1 queued\n", LSM303DLHC_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lsm303dlhc_acc_isr2(int irq, void *dev)
{
	struct lsm303dlhc_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_debug("%s: isr2 queued\n", LSM303DLHC_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm303dlhc_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm303dlhc_acc_status *stat =
	container_of(work, struct lsm303dlhc_acc_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm303dlhc_acc_get_int1_source(stat); */
	/* ; */
	pr_debug("%s: IRQ1 triggered\n", LSM303DLHC_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq1);
}

static void lsm303dlhc_acc_irq2_work_func(struct work_struct *work)
{

	struct lsm303dlhc_acc_status *stat =
	container_of(work, struct lsm303dlhc_acc_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm303dlhc_acc_get_tap_source(stat); */
	/* ; */
	pr_debug("%s: IRQ2 triggered\n", LSM303DLHC_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq2);
}

static int lsm303dlhc_acc_update_fs_range(struct lsm303dlhc_acc_status *stat,
							u8 new_fs_range)
{
	int err=-1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LSM303DLHC_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_fs_range) {
	case LSM303DLHC_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LSM303DLHC_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LSM303DLHC_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case LSM303DLHC_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(stat->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	err = stat->tf->read(stat->dev, CTRL_REG4, 1, buf);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));

	buf[0] = updated_val;
	err = stat->tf->write(stat->dev, CTRL_REG4, 1, buf);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL_REG4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;
error:
	dev_err(stat->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lsm303dlhc_acc_update_odr(struct lsm303dlhc_acc_status *stat,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[1];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm303dlhc_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm303dlhc_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}

	config[0] = lsm303dlhc_acc_odr_table[i].mask;
	config[0] |= LSM303DLHC_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		err = stat->tf->write(stat->dev, CTRL_REG1, 1, config);
		if (err < 0)
			goto error;
	}
	stat->resume_state[RES_CTRL_REG1] = config[0];

	return err;

error:
	dev_err(stat->dev, "update odr failed 0x%02x: %d\n",
		config[0], err);

	return err;
}



static int lsm303dlhc_acc_register_write(struct lsm303dlhc_acc_status *stat,
					u8 *buf, u8 reg_address, u8 new_value)
{
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	int err;

	mutex_lock(&stat->lock);
	err = stat->tf->write(stat->dev, reg_address, 1, &new_value);
	mutex_unlock(&stat->lock);

	return err;
}

/*
static int lsm303dlhc_acc_register_read(struct lsm303dlhc_acc_status *stat,
							u8 *buf, u8 reg_address)
{
	return stat->tf->read(stat->dev, reg_address, 1, buf);
}
*/

/*
static int lsm303dlhc_acc_register_update(struct lsm303dlhc_acc_status *stat,
		u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm303dlhc_acc_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm303dlhc_acc_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}
*/

static int lsm303dlhc_acc_get_acceleration_data(struct lsm303dlhc_acc_status *stat,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	err = stat->tf->read(stat->dev, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);



	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;


	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

#ifdef LSM303DLHC_DEBUG
/*
	pr_debug("%s read x=%d, y=%d, z=%d\n",
			LSM303DLHC_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
*/
#endif
	return err;
}

static void lsm303dlhc_acc_report_values(struct lsm303dlhc_acc_status *stat,
					int *xyz)
{
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_sync(stat->input_dev);
}

int lsm303dlhc_acc_enable(struct lsm303dlhc_acc_status *stat)
{
	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		int err;

		mutex_lock(&stat->lock);
		err = lsm303dlhc_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			mutex_unlock(&stat->lock);
			return err;
		}
		schedule_delayed_work(&stat->input_work,
			msecs_to_jiffies(stat->pdata->poll_interval));
		mutex_unlock(&stat->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm303dlhc_acc_enable);

int lsm303dlhc_acc_disable(struct lsm303dlhc_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_delayed_work_sync(&stat->input_work);

		mutex_lock(&stat->lock);
		lsm303dlhc_acc_device_power_off(stat);
		mutex_unlock(&stat->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm303dlhc_acc_disable);


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	int err;
	u8 data;

	mutex_lock(&stat->lock);
	err = stat->tf->read(stat->dev, reg, 1, &data);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	ret = sprintf(buf, "0x%02x\n", data);
	mutex_unlock(&stat->lock);

	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val=((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lsm303dlhc_acc_register_write(stat, x,reg,new_val);
	if (err < 0)
		return err;
	stat->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms,stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lsm303dlhc_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
	case LSM303DLHC_ACC_G_2G:
		range = 2;
		break;
	case LSM303DLHC_ACC_G_4G:
		range = 4;
		break;
	case LSM303DLHC_ACC_G_8G:
		range = 8;
		break;
	case LSM303DLHC_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LSM303DLHC_ACC_G_2G;
		break;
	case 4:
		range = LSM303DLHC_ACC_G_4G;
		break;
	case 8:
		range = LSM303DLHC_ACC_G_8G;
		break;
	case 16:
		range = LSM303DLHC_ACC_G_16G;
		break;
	default:
		dev_err(stat->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm303dlhc_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(stat->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303dlhc_acc_enable(stat);
	else
		lsm303dlhc_acc_disable(stat);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf, INT_THS1, INT1_THRESHOLD_MASK,RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev,buf,INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}


#ifdef LSM303DLHC_DEBUG
/* PAY ATTENTION: These LSM303DLHC_DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	/*TODO: error need to be managed */
	rc = stat->tf->write(stat->dev, stat->reg_addr, 1, &(u8)val);
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	rc = stat->tf->read(stat->dev, stat->reg_addr, 1, &data);
	mutex_unlock(&stat->lock);

	/*TODO: error need to be managed */
	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303dlhc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
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

#ifdef LSM303DLHC_DEBUG
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

static void lsm303dlhc_acc_input_work_func(struct work_struct *work)
{
	struct lsm303dlhc_acc_status *stat;

	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct delayed_work *)work,
			struct lsm303dlhc_acc_status, input_work);

	mutex_lock(&stat->lock);
	err = lsm303dlhc_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get_acceleration_data failed\n");
	else
		lsm303dlhc_acc_report_values(stat, xyz);

	schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
			stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);
}

static int lsm303dlhc_acc_validate_pdata(struct lsm303dlhc_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LSM303DLHC_ACC_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
		stat->pdata->axis_map_y > 2 ||
		 stat->pdata->axis_map_z > 2) {
		dev_err(stat->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
					stat->pdata->axis_map_y,
						stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
			|| stat->pdata->negate_z > 1) {
		dev_err(stat->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", stat->pdata->negate_x,
				stat->pdata->negate_y, stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(stat->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303dlhc_acc_input_init(struct lsm303dlhc_acc_status *stat)
{
	int err;

	INIT_DELAYED_WORK(&stat->input_work, lsm303dlhc_acc_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(stat->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->name = LSM303DLHC_ACC_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = stat->bus_type;
	stat->input_dev->dev.parent = stat->dev;

	input_set_drvdata(stat->input_dev, stat);

	/* Set the input event characteristics of the probed sensor driver */
	set_bit(INPUT_EVENT_TYPE, stat->input_dev->evbit);
	set_bit(INPUT_EVENT_X, stat->input_dev->mscbit);
	set_bit(INPUT_EVENT_Y, stat->input_dev->mscbit);
	set_bit(INPUT_EVENT_Z, stat->input_dev->mscbit);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(stat->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm303dlhc_acc_input_cleanup(struct lsm303dlhc_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

int lsm303dlhc_acc_probe(struct lsm303dlhc_acc_status *stat)
{

	int err = -1;

#ifdef LSM303DLHC_DEBUG
	dev_info(stat->dev, "probe start.\n");
#endif

	mutex_lock(&stat->lock);

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(stat->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (stat->dev->platform_data == NULL) {
		default_lsm303dlhc_acc_pdata.gpio_int1 = int1_gpio;
		default_lsm303dlhc_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_lsm303dlhc_acc_pdata,
							sizeof(*stat->pdata));
		dev_info(stat->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, stat->dev->platform_data,
							sizeof(*stat->pdata));
	}

	err = lsm303dlhc_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(stat->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if(stat->pdata->gpio_int1 >= 0){
		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, "
							"mapped on gpio:%d\n",
			LSM303DLHC_ACC_DEV_NAME, __func__, stat->irq1,
							stat->pdata->gpio_int1);
	}

	if(stat->pdata->gpio_int2 >= 0){
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, "
							"mapped on gpio:%d\n",
			LSM303DLHC_ACC_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					LSM303DLHC_ACC_ENABLE_ALL_AXES);
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);

/*
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG6] = ALL_ZEROES;

	stat->resume_state[RES_TEMP_CFG_REG] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;
	stat->resume_state[RES_INT_CFG1] = ALL_ZEROES;
	stat->resume_state[RES_INT_THS1] = ALL_ZEROES;
	stat->resume_state[RES_INT_DUR1] = ALL_ZEROES;

	stat->resume_state[RES_TT_CFG] = ALL_ZEROES;
	stat->resume_state[RES_TT_THS] = ALL_ZEROES;
	stat->resume_state[RES_TT_LIM] = ALL_ZEROES;
	stat->resume_state[RES_TT_TLAT] = ALL_ZEROES;
	stat->resume_state[RES_TT_TW] = ALL_ZEROES;
*/

	err = lsm303dlhc_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm303dlhc_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(stat->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm303dlhc_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm303dlhc_acc_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev,
		   "device LSM303DLHC_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}


	lsm303dlhc_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	if(stat->pdata->gpio_int1 >= 0){
		INIT_WORK(&stat->irq1_work, lsm303dlhc_acc_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("lsm303dlhc_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(stat->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(stat->irq1, lsm303dlhc_acc_isr1,
			IRQF_TRIGGER_RISING, "lsm303dlhc_acc_irq1", stat);
		if (err < 0) {
			dev_err(stat->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}

	if(stat->pdata->gpio_int2 >= 0){
		INIT_WORK(&stat->irq2_work, lsm303dlhc_acc_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("lsm303dlhc_acc_wq2");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(stat->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(stat->irq2, lsm303dlhc_acc_isr2,
			IRQF_TRIGGER_RISING, "lsm303dlhc_acc_irq2", stat);
		if (err < 0) {
			dev_err(stat->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(stat->irq2);
	}

	mutex_unlock(&stat->lock);

#ifdef LSM303DLHC_DEBUG
	dev_info(stat->dev, "%s: probed\n", LSM303DLHC_ACC_DEV_NAME);
#endif

	return 0;

err_destoyworkqueue2:
	if(stat->pdata->gpio_int2 >= 0)
		destroy_workqueue(stat->irq2_work_queue);
err_free_irq1:
	free_irq(stat->irq1, stat);
err_destoyworkqueue1:
	if(stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(stat->dev);
err_input_cleanup:
	lsm303dlhc_acc_input_cleanup(stat);
err_power_off:
	lsm303dlhc_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);

	return err;
}
EXPORT_SYMBOL(lsm303dlhc_acc_probe);

int lsm303dlhc_acc_remove(struct lsm303dlhc_acc_status *stat)
{
	if(stat->pdata->gpio_int1 >= 0){
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}

	if(stat->pdata->gpio_int2 >= 0){
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lsm303dlhc_acc_input_cleanup(stat);
	lsm303dlhc_acc_device_power_off(stat);
	remove_sysfs_interfaces(stat->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);

	return 0;
}
EXPORT_SYMBOL(lsm303dlhc_acc_remove);

MODULE_DESCRIPTION("lsm303dlhc accelerometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");

