/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: lis331hh_core.c
* Authors		: MSH - Motion Mems BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Mario Tesi (mario.tesi@st.com)
*			: Authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.7.2
* Date			: 2012/10/07
* Description		: LIS331HH 3D accelerometer sensor API
*
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

 Revision 1.5.0 2010/09/05:
	lis331hh_acc_device_power_off now calling CTRL_REG1 to set power off
	manages 2 interrupts;
	correction to update_g_range;
	modified_get_acceleration_data function
	modified update_odr function and lis331hh_acc_odr_table;
	don't support ioclt;
	supports sysfs;
 Revision 1.6.0 2011/02/28
	checks for availability of interrupts pins
 Revision 1.7.0 2011/03/02
	adds self test enable/disable
 Revision 1.7.1 2012/10/07
	corrects default permissions on sys fs files
 Revision 1.7.2 2016/06/07
	added spi support and default platform data
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
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "lis331hh.h"

#define G_MAX	10000

#define SENSITIVITY_6G		3	/**	mg/LSB	*/
#define SENSITIVITY_12G		6	/**	mg/LSB	*/
#define SENSITIVITY_24G		12	/**	mg/LSB	*/

#define AXISDATA_REG		0x28
#define WHOAMI_LIS331HH_ACC	0x32	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register */
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define	CTRL_REG5		0x24

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold */
#define	INT_DUR1		0x33	/*	interrupt 1 duration */

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold */
#define	INT_DUR2		0x37	/*	interrupt 2 duration */
/*	end CONTROL REGISTRES	*/

#define LIS331HH_ACC_ENABLE_ALL_AXES	0x07
#define LIS331HH_SELFTEST_EN		0x02
#define LIS331HH_SELFTEST_DIS		0x00
#define LIS331HH_SELFTEST_POS		0x00
#define LIS331HH_SELFTEST_NEG		0x08

/* Accelerometer output data rate  */
#define LIS331HH_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define LIS331HH_ACC_ODR1		0x60	/* 1Hz output data rate */
#define LIS331HH_ACC_ODR2		0x80	/* 2Hz output data rate */
#define LIS331HH_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define LIS331HH_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define LIS331HH_ACC_ODR50		0x00	/* 50Hz output data rate */
#define LIS331HH_ACC_ODR100		0x08	/* 100Hz output data rate */
#define LIS331HH_ACC_ODR400		0x10	/* 400Hz output data rate */
#define LIS331HH_ACC_ODR1000		0x18	/* 1000Hz output data rate */

#define FUZZ			0
#define FLAT			0

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_REFERENCE		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8
#define	RES_INT_CFG2		9
#define	RES_INT_THS2		10
#define	RES_INT_DUR2		11
/* end RESUME STATE INDICES */

static struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis331hh_odr_table[] = {
	{ 1, LIS331HH_ACC_PM_NORMAL | LIS331HH_ACC_ODR1000 },
	{ 3, LIS331HH_ACC_PM_NORMAL | LIS331HH_ACC_ODR400 },
	{ 10, LIS331HH_ACC_PM_NORMAL | LIS331HH_ACC_ODR100 },
	{ 20, LIS331HH_ACC_PM_NORMAL | LIS331HH_ACC_ODR50},
	/* low power settings, max low pass filter cut-off freq */
	{ 100, LIS331HH_ACC_ODR10 | LIS331HH_ACC_ODR1000 },
	{ 200, LIS331HH_ACC_ODR5 | LIS331HH_ACC_ODR1000 },
	{ 5000, LIS331HH_ACC_ODR2 | LIS331HH_ACC_ODR1000 },
	{ 1000, LIS331HH_ACC_ODR1 | LIS331HH_ACC_ODR1000 },
	{ 2000, LIS331HH_ACC_ODRHALF | LIS331HH_ACC_ODR1000 },
};

static struct lis331hh_platform_data default_lis331hh_pdata = {
	.g_range = LIS331HH_ACC_G_6G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LIS331HH_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LIS331HH_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LIS331HH_ACC_DEFAULT_INT2_GPIO,
};

static int lis331hh_hw_init(struct lis331hh_data *acc)
{
	int err = -1;
	u8 buf[4];

	printk(KERN_INFO "%s: hw init start\n", LIS331HH_ACC_DEV_NAME);

	err = acc->tf->read(acc, WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(acc->dev, "Error reading WHO_AM_I: is device "
			"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_LIS331HH_ACC) {
		dev_err(acc->dev,
			"device unknown. Expected: 0x%x, Replies: 0x%x\n",
			WHOAMI_LIS331HH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = acc->resume_state[RES_CTRL_REG1];
	err = acc->tf->write(acc, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS1];
	buf[1] = acc->resume_state[RES_INT_DUR1];
	err = acc->tf->write(acc, INT_THS1, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_CFG1];
	err = acc->tf->write(acc, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS2];
	buf[1] = acc->resume_state[RES_INT_DUR2];
	err = acc->tf->write(acc, INT_THS2, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_CFG2];
	err = acc->tf->write(acc, INT_CFG2, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_CTRL_REG2];
	buf[1] = acc->resume_state[RES_CTRL_REG3];
	buf[2] = acc->resume_state[RES_CTRL_REG4];
	buf[3] = acc->resume_state[RES_CTRL_REG5];
	err = acc->tf->write(acc, CTRL_REG2, 4, buf);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", LIS331HH_ACC_DEV_NAME);

	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(acc->dev, "hw init error: %d\n", err);

	return err;
}

static void lis331hh_device_power_off(struct lis331hh_data *acc)
{
	int err;
	u8 buf = LIS331HH_ACC_PM_OFF;

	err = acc->tf->write(acc, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(acc->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if (acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if (acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lis331hh_device_power_on(struct lis331hh_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(acc->dev,
				"power_on failed: %d\n", err);
			return err;
		}
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = lis331hh_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lis331hh_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t lis331hh_isr1(int irq, void *dev)
{
	struct lis331hh_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	printk(KERN_INFO "%s: isr1 queued\n", LIS331HH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lis331hh_isr2(int irq, void *dev)
{
	struct lis331hh_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk(KERN_INFO "%s: isr2 queued\n", LIS331HH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lis331hh_irq1_work_func(struct work_struct *work)
{
	struct lis331hh_data *acc =

	container_of(work, struct lis331hh_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lis331hh_get_int1_source(acc); */
	printk(KERN_INFO "%s: IRQ1 triggered\n", LIS331HH_ACC_DEV_NAME);
	enable_irq(acc->irq1);
}

static void lis331hh_irq2_work_func(struct work_struct *work)
{
	struct lis331hh_data *acc =

	container_of(work, struct lis331hh_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lis331hh_get_tap_source(acc); */
	printk(KERN_INFO "%s: IRQ2 triggered\n", LIS331HH_ACC_DEV_NAME);
	enable_irq(acc->irq2);
}

static int lis331hh_update_g_range(struct lis331hh_data *acc, u8 new_g_range)
{
	int err = -1;
	u8 sensitivity;
	u8 updated_val;
	u8 init_val;
	u8 new_val;

	switch (new_g_range) {
	case LIS331HH_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		break;
	case LIS331HH_ACC_G_12G:
		sensitivity = SENSITIVITY_12G;
		break;
	case LIS331HH_ACC_G_24G:
		sensitivity = SENSITIVITY_24G;
		break;
	default:
		dev_err(acc->dev, "invalid g range requested: %u\n",
			new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		err = acc->tf->read(acc, CTRL_REG4, 1, &init_val);
		if (err < 0)
			goto error;

		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range;
		updated_val = ((LIS331HH_ACC_FS_MASK & new_val) |
			       ((~LIS331HH_ACC_FS_MASK) & init_val));
		err = acc->tf->write(acc, CTRL_REG4, 1, &updated_val);
		if (err < 0)
			goto error;

		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}

	return err;
error:
	dev_err(acc->dev, "update g range failed 0x%x,0x%x: %d\n",
		CTRL_REG4, updated_val, err);

	return err;
}

static int lis331hh_update_odr(struct lis331hh_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis331hh_odr_table) - 1; i >= 0; i--) {
		if (lis331hh_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config = lis331hh_odr_table[i].mask | LIS331HH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		err = acc->tf->write(acc, CTRL_REG1, 1, &config);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config;
	}

	return err;

error:
	dev_err(acc->dev, "update odr failed 0x%x,0x%x: %d\n",
		CTRL_REG1, config, err);

	return err;
}

static int lis331hh_register_write(struct lis331hh_data *acc, u8 *buf,
				   u8 reg_address, u8 new_value)
{
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = new_value;

	return acc->tf->write(acc, reg_address, 1, buf);
}

static int lis331hh_register_read(struct lis331hh_data *acc, u8 *buf,
				  u8 reg_address)
{
	return acc->tf->read(acc, reg_address, 1, buf);
}

/* This function must be called with acc->lock taken */
static int lis331hh_register_update(struct lis331hh_data *acc, u8 *buf,
				    u8 reg_address, u8 mask, u8 new_bit_values)
{
	u8 init_val;
	u8 updated_val;
	int err = lis331hh_register_read(acc, buf, reg_address);

	if (err >= 0) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lis331hh_register_write(acc, buf, reg_address, updated_val);
	}

	return err;
}

static int lis331hh_selftest(struct lis331hh_data *acc, u8 enable)
{
	int err = -1;
	u8 buf[2] = { 0x00, 0x00 };
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = 0x0A;
	if (enable > 0)
		bit_values = LIS331HH_SELFTEST_EN | LIS331HH_SELFTEST_POS;
	else
		bit_values = LIS331HH_SELFTEST_DIS | LIS331HH_SELFTEST_POS;

	if (atomic_read(&acc->enabled)) {
		mutex_lock(&acc->lock);
		err = lis331hh_register_update(acc, buf, reg_address, mask,
					       bit_values);
		acc->selftest_enabled = enable;
		mutex_unlock(&acc->lock);
		if (err < 0)
			return err;

		acc->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_CTRL_REG4]));
	}

	return err;
}

static int lis331hh_get_acceleration_data(struct lis331hh_data *acc, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	err = acc->tf->read(acc, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		  : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		  : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		  : (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG
		printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
		       LIS331HH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif

	return err;
}

static void lis331hh_report_values(struct lis331hh_data *acc, int *xyz)
{
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int lis331hh_enable(struct lis331hh_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lis331hh_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		/* Android:
		 * Udpate polling rate in case odr was changed while sensor disabled */
		lis331hh_update_odr(acc, acc->pdata->poll_interval);
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lis331hh_disable(struct lis331hh_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work(&acc->input_work);
		lis331hh_device_power_off(acc);
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	int rc = 0;
	u8 data;

	rc = acc->tf->read(acc, reg, 1, &data);
	if (rc < 0)
		return rc;

	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static int write_reg(struct device *dev, const char *buf, u8 reg)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	u8 x;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	x = val;

	return acc->tf->write(acc, reg, 1, &x);
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis331hh_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	lis331hh_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	char val;
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->g_range ;
	switch (val) {
	case LIS331HH_ACC_G_6G:
		range = 6;
		break;
	case LIS331HH_ACC_G_12G:
		range = 12;
		break;
	case LIS331HH_ACC_G_24G:
		range = 24;
		break;
	}
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	lis331hh_update_g_range(acc, val);
	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	
	if (val)
		lis331hh_enable(acc);
	else
		lis331hh_disable(acc);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int val;
	struct lis331hh_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->selftest_enabled;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	lis331hh_selftest(acc, val);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1);
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
	return write_reg(dev, buf, INT_DUR1);
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
	return write_reg(dev, buf, INT_THS1);
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

static ssize_t attr_set_intconfig2(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG2);
}

static ssize_t attr_get_intconfig2(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_CFG2);
}

static ssize_t attr_set_duration2(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR2);
}

static ssize_t attr_get_duration2(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_DUR2);
}

static ssize_t attr_set_thresh2(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	return write_reg(dev, buf, INT_THS2);
}

static ssize_t attr_get_thresh2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_THS2);
}
static ssize_t attr_get_source2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC2);
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int rc;
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	u8 reg;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	reg = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = acc->tf->write(acc, x, 1, (u8 *)&val);

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct lis3de_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 reg;

	mutex_lock(&stat->lock);
	reg = stat->reg_addr;
	mutex_unlock(&stat->lock);
	ret = stat->tf->read(stat, reg, 1, buf);

	return ret;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	int ret;
	u8 reg;

	mutex_lock(&acc->lock);
	reg = acc->reg_addr;
	mutex_unlock(&acc->lock);
	ret = acc->tf->read(acc, &reg, 1, buf);

	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct lis331hh_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);

	return size;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(enable_selftest, 0664, attr_get_selftest, attr_set_selftest),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(int2_config, 0664, attr_get_intconfig2, attr_set_intconfig2),
	__ATTR(int2_duration, 0664, attr_get_duration2, attr_set_duration2),
	__ATTR(int2_threshold, 0664, attr_get_thresh2, attr_set_thresh2),
	__ATTR(int2_source, 0444, attr_get_source2, NULL),
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

static void lis331hh_input_work_func(struct work_struct *work)
{
	struct lis331hh_data *acc;
	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			    struct lis331hh_data, input_work);

	mutex_lock(&acc->lock);
	err = lis331hh_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(acc->dev, "get_acceleration_data failed\n");
	else
		lis331hh_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work,
			      msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef LIS331HH_EN_OPEN
static int lis331hh_input_open(struct input_dev *input)
{
	struct lis331hh_data *acc = input_get_drvdata(input);

	return lis331hh_enable(acc);
}

static void lis331hh_input_close(struct input_dev *dev)
{
	struct lis331hh_data *acc = input_get_drvdata(dev);

	lis331hh_disable(acc);
}
#endif /* LIS331HH_EN_OPEN */

static int lis331hh_validate_pdata(struct lis331hh_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
					acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 || acc->pdata->axis_map_y > 2 ||
	    acc->pdata->axis_map_z > 2) {
		dev_err(acc->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x, acc->pdata->axis_map_y,
			acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 ||
	    acc->pdata->negate_z > 1) {
		dev_err(acc->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x, acc->pdata->negate_y,
			acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(acc->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis331hh_input_init(struct lis331hh_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lis331hh_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(acc->dev, "input device allocation failed\n");
		return err;
	}

#ifdef LIS331HH_EN_OPEN
	acc->input_dev->open = lis331hh_input_open;
	acc->input_dev->close = lis331hh_input_close;
#else /* LIS331HH_EN_OPEN */
	acc->input_dev->open = NULL;
	acc->input_dev->close = NULL;
#endif /* LIS331HH_EN_OPEN */
	acc->input_dev->name = acc->name;
	acc->input_dev->id.bustype = acc->bustype;
	acc->input_dev->dev.parent = acc->dev;
	input_set_drvdata(acc->input_dev, acc);

	/* Set Misc event type */
	__set_bit(INPUT_EVENT_TYPE, acc->input_dev->evbit);
	__set_bit(INPUT_EVENT_X, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Z, acc->input_dev->mscbit);

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(acc->dev,
			"unable to register input device %s\n",
			acc->input_dev->name);
		input_free_device(acc->input_dev);
		return err;
	}

	return 0;
}

static void lis331hh_input_cleanup(struct lis331hh_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

#ifdef CONFIG_OF
static u32 lis331hh_parse_dt(struct lis331hh_data *acc)
{
	u32 val;
	struct device_node *np;

	np = acc->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "gpio_int1", &val))
		acc->pdata->gpio_int1 = (u8)val;
	if (!of_property_read_u32(np, "gpio_int2", &val))
		acc->pdata->gpio_int2 = (u8)val;

	return 0;
}
#endif

int lis331hh_common_probe(struct lis331hh_data *acc)
{
	int err = -1;

	pr_info("%s: probe start.\n", LIS331HH_ACC_DEV_NAME);

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->pdata = kzalloc(sizeof(struct lis331hh_platform_data), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(acc->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (acc->dev->platform_data == NULL) {
		memcpy(acc->pdata, &default_lis331hh_pdata,
		       sizeof(struct lis331hh_platform_data));
		dev_info(acc->dev, "using default plaform_data\n");
	} else {
		memcpy(acc->pdata, acc->dev->platform_data,
		       sizeof(struct lis331hh_platform_data));
	}

#ifdef CONFIG_OF
	/* override default gpio_irq[1,2] if required */
	lis331hh_parse_dt(acc);
#endif

	err = lis331hh_validate_pdata(acc);
	if (err < 0) {
		dev_err(acc->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(acc->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if (acc->pdata->gpio_int1 >= 0) {
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		printk(KERN_INFO "%s: %s has set irq1 to irq: %d "
		       "mapped on gpio:%d\n", LIS331HH_ACC_DEV_NAME, __func__,
		        acc->irq1, acc->pdata->gpio_int1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		printk(KERN_INFO "%s: %s has set irq2 to irq: %d "
		       "mapped on gpio:%d\n", LIS331HH_ACC_DEV_NAME, __func__,
		       acc->irq2, acc->pdata->gpio_int2);
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = LIS331HH_ACC_ENABLE_ALL_AXES;

	err = lis331hh_device_power_on(acc);
	if (err < 0) {
		dev_err(acc->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lis331hh_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(acc->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	err = lis331hh_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(acc->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = lis331hh_input_init(acc);
	if (err < 0) {
		dev_err(acc->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(acc->dev);
	if (err < 0) {
		dev_err(acc->dev,
			"device %s register failed\n",  LIS331HH_ACC_DEV_NAME);
		goto err_input_cleanup;
	}

	lis331hh_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if (acc->pdata->gpio_int1 >= 0) {
		INIT_WORK(&acc->irq1_work, lis331hh_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lis331hh_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev, "cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lis331hh_isr1,
				  IRQF_TRIGGER_RISING, "lis331hh_irq1", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		INIT_WORK(&acc->irq2_work, lis331hh_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lis331hh_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev, "cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lis331hh_isr2,
				  IRQF_TRIGGER_RISING, "lis331hh_irq2", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq2 failed: %d\n", err);
				goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	mutex_unlock(&acc->lock);

	dev_info(acc->dev, "%s: probed\n", LIS331HH_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(acc->dev);
err_input_cleanup:
	lis331hh_input_cleanup(acc);
err_power_off:
	lis331hh_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	printk(KERN_ERR "%s: Driver Init failed\n", LIS331HH_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lis331hh_common_probe);

int lis331hh_common_remove(struct lis331hh_data *acc)
{
	if (acc->pdata->gpio_int1 >= 0) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	lis331hh_disable(acc);
	lis331hh_input_cleanup(acc);
	remove_sysfs_interfaces(acc->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);

	return 0;
}
EXPORT_SYMBOL(lis331hh_common_remove);

#ifdef CONFIG_PM
int lis331hh_common_resume(struct lis331hh_data *acc)
{
	if (acc->on_before_suspend)
		return lis331hh_enable(acc);

	return 0;
}
EXPORT_SYMBOL(lis331hh_common_resume);

int lis331hh_common_suspend(struct lis331hh_data *acc)
{
	acc->on_before_suspend = atomic_read(&acc->enabled);

	return lis331hh_disable(acc);
}
EXPORT_SYMBOL(lis331hh_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("lis331hh accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
