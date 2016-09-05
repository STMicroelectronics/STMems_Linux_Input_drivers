/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name          : lis2hh12_core.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Mario Tesi (mario.tesi@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.1.1
 * Date               : 2016/May/5
 * Description        : LIS2HH12 accelerometer sensor API
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
 Revision 1.0.0 25/Feb/2013
  first revision
  supports sysfs;
 Revision 1.1.0 28/Mar/2013
  introduces hr_timers for polling;
 Revision 1.1.1 05/May/2016
  introduces spi device;
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>
#include	<linux/version.h>

#include	"lis2hh12.h"

//#define DEBUG		1

#define G_MAX			7995148 /* (SENSITIVITY_8G*(2^15-1)) */
#define G_MIN			- 7995392 /* (-SENSITIVITY_8G*(2^15)   */
#define FUZZ			0
#define FLAT			0

#define MS_TO_NS(x)		(x*1000000L)

#define SENSITIVITY_2G		 61	/**	ug/LSB	*/
#define SENSITIVITY_4G		122	/**	ug/LSB	*/
#define SENSITIVITY_8G		244	/**	ug/LSB	*/


/* Accelerometer Sensor Operating Mode */
#define LIS2HH12_ACC_ENABLE	(0x01)
#define LIS2HH12_ACC_DISABLE	(0x00)

#define AXISDATA_REG		(0x28)
#define WHOAMI_LIS2HH12_ACC	(0x41)	/*	Expctd content for WAI	*/
#define ALL_ZEROES		(0x00)
#define LIS2HH12_ACC_PM_OFF	(0x00)
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
#define RES_FIFO_CTRL		17

/* end RESUME STATE INDICES */

#define OUTPUT_ALWAYS_ANTI_ALIASED 1

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis2hh12_acc_odr_table[] = {
		{   2, ACC_ODR800 },
		{   3, ACC_ODR400 },
		{   5, ACC_ODR200 },
		{  10, ACC_ODR100 },
#if(!OUTPUT_ALWAYS_ANTI_ALIASED)
		{  20, ACC_ODR50  },
		{ 100, ACC_ODR10  },
#endif
};

static int int1_gpio = LIS2HH12_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LIS2HH12_ACC_DEFAULT_INT2_GPIO;

static struct lis2hh12_platform_data default_lis2hh12_acc_pdata = {
	.fs_range = LIS2HH12_ACC_FS_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LIS2HH12_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LIS2HH12_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LIS2HH12_ACC_DEFAULT_INT2_GPIO,
};

/* sets default init values to be written in registers at probe stage */
static void lis2hh12_acc_set_init_register_values(struct lis2hh12_status *stat)
{
	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL1] = (ALL_ZEROES | \
					 CTRL1_HR_DISABLE | \
					 CTRL1_BDU_ENABLE | \
					 ACC_ENABLE_ALL_AXES);

	if(stat->pdata->gpio_int1 >= 0)
		stat->resume_state[RES_CTRL3] =
			(stat->resume_state[RES_CTRL3] | \
			 CTRL3_IG1_INT1);

	stat->resume_state[RES_CTRL4] = (ALL_ZEROES | \
					 CTRL4_IF_ADD_INC_EN);

	stat->resume_state[RES_CTRL5] = (ALL_ZEROES | \
					 CTRL5_HLACTIVE_H);

	if(stat->pdata->gpio_int2 >= 0)
		stat->resume_state[RES_CTRL6] =
			(stat->resume_state[RES_CTRL6] | \
			 CTRL6_IG2_INT2);

	stat->resume_state[RES_CTRL7] = (ALL_ZEROES | \
					 CTRL7_LIR2 | CTRL7_LIR1);
}

static int lis2hh12_acc_hw_init(struct lis2hh12_status *stat)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", LIS2HH12_ACC_DEV_NAME);

	err = stat->tf->read(stat, WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(stat->dev, "Error reading WHO_AM_I:"
			 " is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LIS2HH12_ACC) {
		dev_err(stat->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_LIS2HH12_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = stat->resume_state[RES_FIFO_CTRL];
	err = stat->tf->write(stat, FIFO_CTRL, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_THSX1];
	buf[1] = stat->resume_state[RES_INT_THSY1];
	buf[2] = stat->resume_state[RES_INT_THSZ1];
	buf[3] = stat->resume_state[RES_INT_DUR1];
	err = stat->tf->write(stat, INT_THSX1, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_CFG1];
	err = stat->tf->write(stat, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL2];
	buf[1] = stat->resume_state[RES_CTRL3];
	buf[2] = stat->resume_state[RES_CTRL4];
	buf[3] = stat->resume_state[RES_CTRL5];
	buf[4] = stat->resume_state[RES_CTRL6];
	buf[5] = stat->resume_state[RES_CTRL7];
	err = stat->tf->write(stat, CTRL2, 6, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL1];
	err = stat->tf->write(stat, CTRL1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LIS2HH12_ACC_DEV_NAME);

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

static void lis2hh12_acc_device_power_off(struct lis2hh12_status *stat)
{
	int err;
	u8 buf = LIS2HH12_ACC_PM_OFF;

	err = stat->tf->write(stat, CTRL1, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}
	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->hw_initialized = 0;
	}
}

static int lis2hh12_acc_device_power_on(struct lis2hh12_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(stat->dev,
				"power_on failed: %d\n", err);
			return err;
		}
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}

	if (!stat->hw_initialized) {
		err = lis2hh12_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lis2hh12_acc_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}

	return 0;
}

static int lis2hh12_acc_update_fs_range(struct lis2hh12_status *stat,
					u8 new_fs_range)
{
	int err = -1;
	u8 sensitivity;
	u8 updated_val;
	u8 init_val;
	u8 new_val;

	switch (new_fs_range) {
	case LIS2HH12_ACC_FS_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LIS2HH12_ACC_FS_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LIS2HH12_ACC_FS_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(stat->dev, "invalid fs range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	/* Updates configuration register 4,
	* which contains fs range setting */
	err = stat->tf->read(stat, CTRL4, 1, &init_val);
	if (err < 0)
		goto error;

	stat->resume_state[RES_CTRL4] = init_val;
	new_val = new_fs_range;
	updated_val = ((LIS2HH12_ACC_FS_MASK & new_val) |
		       ((~LIS2HH12_ACC_FS_MASK) & init_val));
	err = stat->tf->write(stat, CTRL4, 1, &updated_val);
	if (err < 0)
		goto error;

	stat->resume_state[RES_CTRL4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;

error:
	dev_err(stat->dev,
		"update fs range failed 0x%02x,0x%02x: %d\n",
		CTRL4, updated_val, err);

	return err;
}

static int lis2hh12_acc_update_odr(struct lis2hh12_status *stat,
				   int poll_interval_ms)
{
	int err;
	int i;
	u8 updated_val;
	u8 init_val;
	u8 new_val;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis2hh12_acc_odr_table) - 1; i >= 0; i--) {
		if ((lis2hh12_acc_odr_table[i].cutoff_ms <= poll_interval_ms) ||
		    (i == 0))
			break;
	}
	new_val = lis2hh12_acc_odr_table[i].mask;

	/* Updates configuration register 1,
	* which contains odr range setting if enabled,
	* otherwise updates RES_CTRL1 for when it will */
	if (atomic_read(&stat->enabled)) {
		err = stat->tf->read(stat, CTRL1, 1, &init_val);
		if (err < 0)
			goto error;

		stat->resume_state[RES_CTRL1] = init_val;
		updated_val = ((ACC_ODR_MASK & new_val) |
			       ((~ACC_ODR_MASK) & init_val));
		err = stat->tf->write(stat, CTRL1, 1, &updated_val);
		if (err < 0)
			goto error;

		stat->resume_state[RES_CTRL1] = updated_val;

		return err;
	} else {
		init_val = stat->resume_state[RES_CTRL1];
		updated_val = ((ACC_ODR_MASK & new_val) |
			       ((~ACC_ODR_MASK) & init_val));
		stat->resume_state[RES_CTRL1] = updated_val;

		return 0;
	}

error:
	dev_err(stat->dev,
		"update odr failed 0x%02x,0x%02x: %d\n",
		CTRL1, updated_val, err);

	return err;
}

static int lis2hh12_acc_register_write(struct lis2hh12_status *stat,
				       u8 *buf, u8 reg_address, u8 new_value)
{
	u8 val = new_value;

	return stat->tf->write(stat, reg_address, 1, &val);
}


static int lis2hh12_acc_get_data(struct lis2hh12_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	err = stat->tf->read(stat, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;

	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

//#ifdef DEBUG

	pr_info("%s read x=%d, y=%d, z=%d\n",
		LIS2HH12_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

//#endif
	return err;
}

static void lis2hh12_acc_report_values(struct lis2hh12_status *stat,
				       int *xyz)
{
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(stat->input_dev);
}

static void lis2hh12_acc_report_triple(struct lis2hh12_status *stat)
{
	int err;
	int xyz[3];

	err = lis2hh12_acc_get_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get_data failed\n");
	else
		lis2hh12_acc_report_values(stat, xyz);
}

static irqreturn_t lis2hh12_acc_isr1(int irq, void *dev)
{
	struct lis2hh12_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	pr_debug("%s: isr1 queued\n", LIS2HH12_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lis2hh12_acc_isr2(int irq, void *dev)
{
	struct lis2hh12_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_debug("%s: isr2 queued\n", LIS2HH12_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lis2hh12_acc_irq1_work_func(struct work_struct *work)
{
	struct lis2hh12_status *stat =

	container_of(work, struct lis2hh12_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lis2hh12_acc_get_int1_source(stat); */
	pr_debug("%s: IRQ1 served\n", LIS2HH12_ACC_DEV_NAME);
	enable_irq(stat->irq1);
}

static void lis2hh12_acc_irq2_work_func(struct work_struct *work)
{
	struct lis2hh12_status *stat =

	container_of(work, struct lis2hh12_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lis2hh12_acc_get_tap_source(stat); */
	pr_debug("%s: IRQ2 served\n", LIS2HH12_ACC_DEV_NAME);
	enable_irq(stat->irq2);
}

static int lis2hh12_acc_enable(struct lis2hh12_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lis2hh12_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		stat->polling_ktime = ktime_set(stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
		hrtimer_start(&stat->hr_timer_poll,
			      stat->polling_ktime, HRTIMER_MODE_REL);
	}

	return 0;
}

static int lis2hh12_acc_disable(struct lis2hh12_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_work_sync(&stat->input_poll_work);
		lis2hh12_acc_device_power_off(stat);
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
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
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis2hh12_acc_register_write(stat, x, reg, new_val);
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
	struct lis2hh12_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	err = lis2hh12_acc_update_odr(stat, interval_ms);
	if (err >= 0) {
		stat->pdata->poll_interval = interval_ms;
		stat->polling_ktime = ktime_set(stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
	}
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	char val;
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
	case LIS2HH12_ACC_FS_2G:
		range = 2;
		break;
	case LIS2HH12_ACC_FS_4G:
		range = 4;
		break;
	case LIS2HH12_ACC_FS_8G:
		range = 8;
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LIS2HH12_ACC_FS_2G;
		break;
	case 4:
		range = LIS2HH12_ACC_FS_4G;
		break;
	case 8:
		range = LIS2HH12_ACC_FS_8G;
		break;
	default:
		dev_err(stat->dev, "invalid range request: %lu,"
			" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lis2hh12_acc_update_fs_range(stat, range);
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
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis2hh12_acc_enable(stat);
	else
		lis2hh12_acc_disable(stat);

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

static ssize_t attr_set_threshx1(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSX1, INT1_THRESHOLD_MASK, RES_INT_THSX1);
}

static ssize_t attr_get_threshx1(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_THSX1);
}

static ssize_t attr_set_threshy1(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSY1, INT1_THRESHOLD_MASK, RES_INT_THSY1);
}

static ssize_t attr_get_threshy1(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_THSY1);
}

static ssize_t attr_set_threshz1(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSZ1, INT1_THRESHOLD_MASK, RES_INT_THSZ1);
}

static ssize_t attr_get_threshz1(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_THSZ1);
}

static ssize_t attr_get_source1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int rc;
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	u8 x;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	x = val;
	rc = stat->tf->write(stat, stat->reg_addr, 1, &x);

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	rc = stat->tf->read(stat, stat->reg_addr, 1, &data);
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct lis2hh12_status *stat = dev_get_drvdata(dev);
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
	__ATTR(int1_thresholdx, 0664, attr_get_threshx1, attr_set_threshx1),
	__ATTR(int1_thresholdy, 0664, attr_get_threshy1, attr_set_threshy1),
	__ATTR(int1_thresholdz, 0664, attr_get_threshz1, attr_set_threshz1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),

#ifdef DEBUG
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

static void lis2hh12_acc_input_poll_work_func(struct work_struct *work)
{
	struct lis2hh12_status *stat;

	stat = container_of((struct work_struct *) work,
			    struct lis2hh12_status, input_poll_work);

	mutex_lock(&stat->lock);
	lis2hh12_acc_report_triple(stat);
	mutex_unlock(&stat->lock);

	if (atomic_read(&stat->enabled))
		hrtimer_start(&stat->hr_timer_poll, stat->polling_ktime,
			      HRTIMER_MODE_REL);
}

enum hrtimer_restart lis2hh12_acc_hr_timer_poll_function(struct hrtimer *timer)
{
	struct lis2hh12_status *stat;

	stat = container_of((struct hrtimer *)timer,
			    struct lis2hh12_status, hr_timer_poll);

	queue_work(stat->hr_timer_poll_work_queue, &stat->input_poll_work);

	return HRTIMER_NORESTART;
}

#ifdef LIS2HH_EN_OPEN
int lis2hh12_acc_input_open(struct input_dev *input)
{
	struct lis2hh12_status *stat = input_get_drvdata(input);

	dev_dbg(stat->dev, "%s\n", __func__);

	return lis2hh12_acc_enable(stat);
}

void lis2hh12_acc_input_close(struct input_dev *dev)
{
	struct lis2hh12_status *stat = input_get_drvdata(dev);

	dev_dbg(stat->dev, "%s\n", __func__);
	lis2hh12_acc_disable(stat);
}
#endif /* LIS2HH_EN_OPEN */

static int lis2hh12_acc_validate_pdata(struct lis2hh12_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LIS2HH12_ACC_MIN_POLL_PERIOD_MS,
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

static int lis2hh12_acc_input_init(struct lis2hh12_status *stat)
{
	int err;

	INIT_WORK(&stat->input_poll_work, lis2hh12_acc_input_poll_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(stat->dev, "input device allocation failed\n");
		return err;
	}

#ifdef LIS2HH_EN_OPEN
	stat->input_dev->open = lis2hh12_acc_input_open;
	stat->input_dev->close = lis2hh12_acc_input_close;
#else /* LIS2HH_EN_OPEN */
	stat->input_dev->open = NULL;
	stat->input_dev->close = NULL;
#endif /* LIS2HH_EN_OPEN */
	stat->input_dev->name = stat->name;
	stat->input_dev->id.bustype = stat->bustype;
	stat->input_dev->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev, stat);

	/* Set Misc event type */
	__set_bit(INPUT_EVENT_TYPE, stat->input_dev->evbit);
	__set_bit(INPUT_EVENT_X, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Z, stat->input_dev->mscbit);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(stat->dev, "unable to register input device %s\n",
			stat->input_dev->name);
		input_free_device(stat->input_dev);
	}

	return 0;
}

static void lis2hh12_acc_input_cleanup(struct lis2hh12_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

int lis2hh12_common_probe(struct lis2hh12_status *stat)
{
	int err = -1;

	dev_info(stat->dev, "probe start.\n");

	mutex_init(&stat->lock);
	mutex_init(&stat->tb.buf_lock);
	mutex_lock(&stat->lock);

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(stat->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (stat->dev->platform_data == NULL) {
		default_lis2hh12_acc_pdata.gpio_int1 = int1_gpio;
		default_lis2hh12_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_lis2hh12_acc_pdata,
		       sizeof(*stat->pdata));
		dev_info(stat->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, stat->dev->platform_data,
		       sizeof(*stat->pdata));
	}
	stat->hr_timer_poll_work_queue = NULL;

	err = lis2hh12_acc_validate_pdata(stat);
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

	if (stat->pdata->gpio_int1 >= 0) {
		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, "
			"mapped on gpio:%d\n",
			LIS2HH12_ACC_DEV_NAME, __func__, stat->irq1,
			stat->pdata->gpio_int1);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, "
			"mapped on gpio:%d\n",
			LIS2HH12_ACC_DEV_NAME, __func__, stat->irq2,
			stat->pdata->gpio_int2);
	}

	lis2hh12_acc_set_init_register_values(stat);

	err = lis2hh12_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	err = lis2hh12_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(stat->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lis2hh12_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	stat->hr_timer_poll_work_queue = 
			create_workqueue("lis2hh12_acc_hr_timer_poll_wq");
	hrtimer_init(&stat->hr_timer_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_poll.function = &lis2hh12_acc_hr_timer_poll_function;

	err = lis2hh12_acc_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "input init failed\n");
		goto err_remove_hr_work_queue;
	}

	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev,
		   	"device %s sysfs register failed\n",
			LIS2HH12_ACC_DEV_NAME);
		goto err_input_cleanup;
	}

	lis2hh12_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	if (stat->pdata->gpio_int1 >= 0) {
		INIT_WORK(&stat->irq1_work, lis2hh12_acc_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("lis2hh12_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(stat->dev,
				"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(stat->irq1, lis2hh12_acc_isr1,
			IRQF_TRIGGER_RISING, "lis2hh12_acc_irq1", stat);
		if (err < 0) {
			dev_err(stat->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		INIT_WORK(&stat->irq2_work, lis2hh12_acc_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("lis2hh12_acc_wq2");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(stat->dev,
				"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(stat->irq2, lis2hh12_acc_isr2,
			IRQF_TRIGGER_RISING, "lis2hh12_acc_irq2", stat);
		if (err < 0) {
			dev_err(stat->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(stat->irq2);
	}

	mutex_unlock(&stat->lock);

	dev_info(stat->dev, "%s: probed\n", LIS2HH12_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if (stat->pdata->gpio_int2 >= 0)
		destroy_workqueue(stat->irq2_work_queue);
err_free_irq1:
	free_irq(stat->irq1, stat);
err_destoyworkqueue1:
	if (stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(stat->dev);
err_input_cleanup:
	lis2hh12_acc_input_cleanup(stat);
err_remove_hr_work_queue:
	if (!stat->hr_timer_poll_work_queue) {
		flush_workqueue(stat->hr_timer_poll_work_queue);
		destroy_workqueue(stat->hr_timer_poll_work_queue);
		stat->hr_timer_poll_work_queue = NULL;
	}
err_power_off:
	lis2hh12_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
	pr_err("%s: Driver Init failed\n", LIS2HH12_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lis2hh12_common_probe);

int lis2hh12_common_remove(struct lis2hh12_status *stat)
{
	dev_info(stat->dev, "driver removing\n");

	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lis2hh12_acc_disable(stat);
	lis2hh12_acc_input_cleanup(stat);

	remove_sysfs_interfaces(stat->dev);

	if (!stat->hr_timer_poll_work_queue) {
		flush_workqueue(stat->hr_timer_poll_work_queue);
		destroy_workqueue(stat->hr_timer_poll_work_queue);
		stat->hr_timer_poll_work_queue = NULL;
	}

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);

	return 0;
}
EXPORT_SYMBOL(lis2hh12_common_remove);

#ifdef CONFIG_PM
int lis2hh12_common_resume(struct lis2hh12_status *stat)
{
	if (stat->on_before_suspend)
		return lis2hh12_acc_enable(stat);

	return 0;
}
EXPORT_SYMBOL(lis2hh12_common_resume);

int lis2hh12_common_suspend(struct lis2hh12_status *stat)
{
	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lis2hh12_acc_disable(stat);
}
EXPORT_SYMBOL(lis2hh12_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("lis2hh12 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
