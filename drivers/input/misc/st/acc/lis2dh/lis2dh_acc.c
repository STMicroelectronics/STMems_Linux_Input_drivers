/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name          : lis2dh_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Mario Tesi <mario.tesi@st.com>
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.14
 * Date               : 2016/Apr/26
 * Description        : LIS2DH accelerometer sensor API
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
 *******************************************************************************/
/*******************************************************************************
Version History.
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
 Revision 1.0.13: 2013/Feb/25
  modified acc_remove function;
 Revision 1.0.14: 2016/Apr/26
  added new i2c and spi interface
******************************************************************************/

#include <linux/version.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "lis2dh.h"

#define MS_TO_NS(x)		(x*1000000L)
#define	G_MAX			16000

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LIS2DH_ACC_ENABLE	0x01
#define LIS2DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS2DH_ACC	0x33	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/


#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		0x00

#define LIS2DH_ACC_PM_OFF	0x00
#define LIS2DH_ACC_ENABLE_ALL_AXES	0x07

#define PMODE_MASK		0x08
#define ODR_MASK		0XF0

#define LIS2DH_ACC_ODR1		0x10  /* 1Hz output data rate */
#define LIS2DH_ACC_ODR10	0x20  /* 10Hz output data rate */
#define LIS2DH_ACC_ODR25	0x30  /* 25Hz output data rate */
#define LIS2DH_ACC_ODR50	0x40  /* 50Hz output data rate */
#define LIS2DH_ACC_ODR100	0x50  /* 100Hz output data rate */
#define LIS2DH_ACC_ODR200	0x60  /* 200Hz output data rate */
#define LIS2DH_ACC_ODR400	0x70  /* 400Hz output data rate */
#define LIS2DH_ACC_ODR1250	0x90  /* 1250Hz output data rate */

#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG4_BDU_ENABLE	0x80
#define	CTRL_REG4_BDU_MASK	0x80
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */
#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01

#define	FUZZ			0
#define	FLAT			0

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
} lis2dh_acc_odr_table[] = {
		{    1, LIS2DH_ACC_ODR1250 },
		{    3, LIS2DH_ACC_ODR400  },
		{    5, LIS2DH_ACC_ODR200  },
		{   10, LIS2DH_ACC_ODR100  },
		{   20, LIS2DH_ACC_ODR50   },
		{   40, LIS2DH_ACC_ODR25   },
		{  100, LIS2DH_ACC_ODR10   },
		{ 1000, LIS2DH_ACC_ODR1    },
};

static struct lis2dh_acc_platform_data default_lis2dh_acc_pdata = {
	.fs_range = LIS2DH_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LIS2DH_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LIS2DH_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LIS2DH_ACC_DEFAULT_INT2_GPIO,
};

static int lis2dh_acc_hw_init(struct lis2dh_acc_status *stat)
{
	int err = -1;
	u8 buf[6];

	pr_info("%s: hw init start\n", LIS2DH_ACC_DEV_NAME);

	buf[0] = stat->resume_state[RES_CTRL_REG1];
	err = stat->tf->write(stat, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TEMP_CFG_REG];
	err = stat->tf->write(stat, TEMP_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = stat->tf->write(stat, FIFO_CTRL_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TT_THS];
	buf[1] = stat->resume_state[RES_TT_LIM];
	buf[2] = stat->resume_state[RES_TT_TLAT];
	buf[3] = stat->resume_state[RES_TT_TW];
	err = stat->tf->write(stat, TT_THS, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TT_CFG];
	err = stat->tf->write(stat, TT_CFG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_THS1];
	buf[1] = stat->resume_state[RES_INT_DUR1];
	err = stat->tf->write(stat, INT_THS1, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_CFG1];
	err = stat->tf->write(stat, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL_REG2];
	buf[1] = stat->resume_state[RES_CTRL_REG3];
	buf[2] = stat->resume_state[RES_CTRL_REG4];
	buf[3] = stat->resume_state[RES_CTRL_REG5];
	buf[4] = stat->resume_state[RES_CTRL_REG6];
	err = stat->tf->write(stat, CTRL_REG2, 5, buf);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LIS2DH_ACC_DEV_NAME);

	return 0;

	stat->hw_working = 0;
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(stat->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
		buf[1], err);

	return err;
}

static void lis2dh_acc_device_power_off(struct lis2dh_acc_status *stat)
{
	int err;
	u8 buf = LIS2DH_ACC_PM_OFF;

	err = stat->tf->write(stat, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	stat->hw_initialized = 0;
}

static int lis2dh_acc_device_power_on(struct lis2dh_acc_status *stat)
{
	int err = -1;

	if (!stat->hw_initialized) {
		err = lis2dh_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lis2dh_acc_device_power_off(stat);

			return err;
		}
	}

	return 0;
}

static int lis2dh_acc_update_fs_range(struct lis2dh_acc_status *stat,
				      u8 new_fs_range)
{
	int err = -1;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS2DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_fs_range) {
	case LIS2DH_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LIS2DH_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LIS2DH_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LIS2DH_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(stat->dev, "invalid fs range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	/* Updates configuration register 4, which contains fs range setting */
	err = stat->tf->read(stat, CTRL_REG4, 1, buf);
	if (err < 0)
		goto error;

	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	err = stat->tf->write(stat, CTRL_REG4, 1, &updated_val);
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

static int lis2dh_acc_update_odr(struct lis2dh_acc_status *stat,
				 int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis2dh_acc_odr_table) - 1; i >= 0; i--) {
		if ((lis2dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms) ||
		    (i == 0))
			break;
	}
	config = lis2dh_acc_odr_table[i].mask | LIS2DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		err = stat->tf->write(stat, CTRL_REG1, 1, &config);
		if (err >= 0)
			stat->resume_state[RES_CTRL_REG1] = config;
	}

	stat->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));
	
	return err;
}

static int lis2dh_acc_register_write(struct lis2dh_acc_status *stat,
				     u8 *buf, u8 reg_address, u8 new_value)
{
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = new_value;

	return stat->tf->write(stat, reg_address, 1, buf);
}

static inline int64_t lis2dh_acc_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static enum hrtimer_restart lis2dh_acc_poll_function_read(struct hrtimer *timer)
{
	struct lis2dh_acc_status *stat;

	stat = container_of((struct hrtimer *)timer,
			    struct lis2dh_acc_status, hr_timer);
	stat->timestamp = lis2dh_acc_get_time_ns();
	queue_work(stat->acc_workqueue, &stat->polling_task);

	return HRTIMER_NORESTART;
}

static int lis2dh_acc_get_acceleration_data(struct lis2dh_acc_status *stat,
					    int *xyz)
{
	int err = -1;
	u8 acc_data[6];
	s16 hw_d[3] = { 0 };

	err = stat->tf->read(stat, AXISDATA_REG, 6, acc_data);
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

	return err;
}

/* Input events chenged to EV_MSC */
static void lis2dh_acc_report_values(struct lis2dh_acc_status *stat,
				     int *xyz)
{
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    stat->timestamp >> 32);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    stat->timestamp & 0xffffffff);
	input_sync(stat->input_dev);
}

static int lis2dh_acc_enable(struct lis2dh_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lis2dh_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}

		lis2dh_acc_update_odr(stat, stat->pdata->poll_interval);
		hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
	}

	return 0;
}

static int lis2dh_acc_disable(struct lis2dh_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		hrtimer_cancel(&stat->hr_timer);
		lis2dh_acc_device_power_off(stat);
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	int err;
	u8 data;

	err = stat->tf->read(stat, reg, 1, &data);
	if (err < 0)
		return err;

	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		     u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis2dh_acc_register_write(stat, x, reg, new_val);
	if (err >= 0)
		stat->resume_state[resumeIndex] = new_val;

	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lis2dh_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	char val;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
	case LIS2DH_ACC_G_2G:
		range = 2;
		break;
	case LIS2DH_ACC_G_4G:
		range = 4;
		break;
	case LIS2DH_ACC_G_8G:
		range = 8;
		break;
	case LIS2DH_ACC_G_16G:
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
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LIS2DH_ACC_G_2G;
		break;
	case 4:
		range = LIS2DH_ACC_G_4G;
		break;
	case 8:
		range = LIS2DH_ACC_G_8G;
		break;
	case 16:
		range = LIS2DH_ACC_G_16G;
		break;
	default:
		dev_err(stat->dev, "invalid range request: %lu,"
			" discarded\n", val);

		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lis2dh_acc_update_fs_range(stat, range);
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
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis2dh_acc_enable(stat);
	else
		lis2dh_acc_disable(stat);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
				   struct device_attribute *attr, char *buf)
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
				  struct device_attribute *attr, char *buf)
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
				struct device_attribute *attr, char *buf)
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
				  struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
				     struct device_attribute *attr, char *buf)
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
				  struct device_attribute *attr, char *buf)
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
				   struct device_attribute *attr, char *buf)
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
				   struct device_attribute *attr, char *buf)
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
				 struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

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

static void lis2dh_acc_input_work_func(struct work_struct *work)
{
	struct lis2dh_acc_status *stat;
	int xyz[3] = { 0 };
	int err;
	ktime_t tmpkt;

	stat = container_of((struct work_struct *)work,
			    struct lis2dh_acc_status, polling_task);

	/* Adjust new timeout */
	tmpkt = ktime_sub(stat->ktime,
			  ktime_set(0, 
			  (lis2dh_acc_get_time_ns() - stat->timestamp)));

	/* Avoid negative value. */
	if (tmpkt.tv64 < 0)
		tmpkt = stat->ktime;

	/* Reschedule timer */
	hrtimer_start(&stat->hr_timer, tmpkt, HRTIMER_MODE_REL);
	mutex_lock(&stat->lock);
	err = lis2dh_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get_acceleration_data failed\n");
	else
		lis2dh_acc_report_values(stat, xyz);
	mutex_unlock(&stat->lock);
}

#ifdef LIS2DH_EN_OPEN_CLOSE
static int lis2dh_acc_input_open(struct input_dev *input)
{
	struct lis2dh_acc_status *stat = input_get_drvdata(input);

	return lis2dh_acc_enable(stat);
}

static void lis2dh_acc_input_close(struct input_dev *dev)
{
	struct lis2dh_acc_status *stat = input_get_drvdata(dev);

	lis2dh_acc_disable(stat);
}
#endif

static int lis2dh_acc_validate_pdata(struct lis2dh_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LIS2DH_ACC_MIN_POLL_PERIOD_MS,
		    stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
					 stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 || stat->pdata->axis_map_y > 2 ||
	    stat->pdata->axis_map_z > 2) {
		dev_err(stat->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
			stat->pdata->axis_map_y, stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1 ||
	    stat->pdata->negate_z > 1) {
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

static int lis2dh_acc_input_init(struct lis2dh_acc_status *stat)
{
	int err;

	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		dev_err(stat->dev, "input device allocation failed\n");
		return -ENOMEM;
	}

	stat->input_dev->name = LIS2DH_ACC_DEV_NAME;
#ifdef LIS2DH_EN_OPEN_CLOSE
	stat->input_dev->open = lis2dh_acc_input_open;
	stat->input_dev->close = lis2dh_acc_input_close;
#endif
	stat->input_dev->id.bustype = stat->bustype;
	stat->input_dev->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev, stat);

	__set_bit(INPUT_EVENT_TYPE, stat->input_dev->evbit );
	__set_bit(INPUT_EVENT_X, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Z, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_MSB, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, stat->input_dev->mscbit);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(stat->dev, "unable to register input device %s\n",
			stat->input_dev->name);
		input_free_device(stat->input_dev);
	}

	return err;
}

static void lis2dh_acc_input_cleanup(struct lis2dh_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

/*
 * struct lis2dh_acc_status *stat is allocated/freed in tf probing
 * so let it manage this stuff
 */
int lis2dh_acc_probe(struct lis2dh_acc_status *stat)
{
	int err = -1;
	u8 wai = 0;

	dev_info(stat->dev, "probe start.\n");

	mutex_init(&stat->lock);
	mutex_init(&stat->tb.buf_lock);

	/* Check device ID and bus connection */
	err = stat->tf->read(stat, WHO_AM_I, 1, &wai);
	if (err < 0) {
		dev_warn(stat->dev, "Error reading WHO_AM_I:"
			 " is device available/working?\n");

		return err;
	}

	if (wai != WHOAMI_LIS2DH_ACC) {
		dev_err(stat->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n", WHOAMI_LIS2DH_ACC, wai);

		return -ENODEV;
	}

	mutex_lock(&stat->lock);
	
	stat->hw_working = 1;
	stat->pdata = kmalloc(sizeof(struct lis2dh_acc_platform_data), GFP_KERNEL);
	if (!stat->pdata) {
		err = -ENOMEM;
		dev_err(stat->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (stat->dev->platform_data == NULL) {
		memcpy(stat->pdata, &default_lis2dh_acc_pdata,
		       sizeof(*stat->pdata));
		dev_info(stat->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, stat->dev->platform_data,
		       sizeof(*stat->pdata));
	}

	err = lis2dh_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));
	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					     LIS2DH_ACC_ENABLE_ALL_AXES);
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);

	err = lis2dh_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto exit_kfree_pdata;
	}

	atomic_set(&stat->enabled, 1);

	err = lis2dh_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(stat->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lis2dh_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis2dh_acc_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev,
			"device LIS2DH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lis2dh_acc_device_power_off(stat);

	mutex_unlock(&stat->lock);

	stat->acc_workqueue = create_workqueue("lis2dh_acc_workqueue");
	if (!stat->acc_workqueue)
		goto err_remove_sysfs_int;

	hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer.function = &lis2dh_acc_poll_function_read;

	INIT_WORK(&stat->polling_task, lis2dh_acc_input_work_func);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	dev_info(stat->dev, "%s: probed\n", LIS2DH_ACC_DEV_NAME);

	return 0;

err_remove_sysfs_int:
	remove_sysfs_interfaces(stat->dev);
err_input_cleanup:
	lis2dh_acc_input_cleanup(stat);
err_power_off:
	lis2dh_acc_device_power_off(stat);
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
	pr_err("%s: Driver Init failed\n", LIS2DH_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lis2dh_acc_probe);

int lis2dh_acc_remove(struct lis2dh_acc_status *stat)
{
	dev_info(stat->dev, "driver removing\n");

	cancel_work_sync(&stat->polling_task);
	if (stat->acc_workqueue) {
		destroy_workqueue(stat->acc_workqueue);
		stat->acc_workqueue = NULL;
	}

	lis2dh_acc_disable(stat);
	lis2dh_acc_input_cleanup(stat);
	remove_sysfs_interfaces(stat->dev);
	kfree(stat->pdata);

	return 0;
}
EXPORT_SYMBOL(lis2dh_acc_remove);

#ifdef CONFIG_PM
int lis2dh_acc_common_resume(struct lis2dh_acc_status *stat)
{
	int err = 0;

	if (stat->on_before_suspend) {
		err = lis2dh_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);

			return err;
		}

		lis2dh_acc_update_odr(stat, stat->pdata->poll_interval);
		hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
	}

	return err;
}
EXPORT_SYMBOL(lis2dh_acc_common_resume);

int lis2dh_acc_common_suspend(struct lis2dh_acc_status *stat)
{
	stat->on_before_suspend = atomic_read(&stat->enabled);

	if (stat->on_before_suspend) {
		mutex_lock(&stat->lock);
		hrtimer_cancel(&stat->hr_timer);
		lis2dh_acc_device_power_off(stat);
		mutex_unlock(&stat->lock);
	}
	return 0;
}
EXPORT_SYMBOL(lis2dh_acc_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("lis2dh accelerometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Mario tesi");
MODULE_LICENSE("GPL v2");
