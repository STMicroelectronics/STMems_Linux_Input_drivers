/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: h3lis331dl_core.c
* Authors		: MSH - Motion Mems BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Mario Tesi (mario.tesi@st.com)
*			: Authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.0.1
* Date			: 2016/05/30
* Description		: H3LIS331DL 3D accelerometer sensor API starting from
* 			: lis331dlh driver
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
 Revision 1.0.1 2016/05/30:
  First porting from lis331dlh driver impl.
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

#include "h3lis331dl.h"

#define G_MAX	8000

#define SENSITIVITY_100G	50	/** mg/LSB */
#define SENSITIVITY_200G	100	/** mg/LSB */
#define SENSITIVITY_400G	200	/** mg/LSB */

#define AXISDATA_REG		0x28
#define WHOAMI_H3LIS331DL_ACC	0x32	/* Expctd content for WAI */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/* WhoAmI register */
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define	CTRL_REG5		0x24

#define	INT_CFG1		0x30	/* interrupt 1 config */
#define	INT_SRC1		0x31	/* interrupt 1 source */
#define	INT_THS1		0x32	/* interrupt 1 threshold */
#define	INT_DUR1		0x33	/* interrupt 1 duration */

#define	INT_CFG2		0x34	/* interrupt 2 config */
#define	INT_SRC2		0x35	/* interrupt 2 source */
#define	INT_THS2		0x36	/* interrupt 2 threshold */
#define	INT_DUR2		0x37	/* interrupt 2 duration */
/*	end CONTROL REGISTRES	*/

#define H3LIS331DL_ACC_ENABLE_ALL_AXES	0x07

/* Accelerometer output data rate  */
#define H3LIS331DL_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define H3LIS331DL_ACC_ODR1		0x60	/* 1Hz output data rate */
#define H3LIS331DL_ACC_ODR2		0x80	/* 2Hz output data rate */
#define H3LIS331DL_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define H3LIS331DL_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define H3LIS331DL_ACC_ODR50		0x00	/* 50Hz output data rate */
#define H3LIS331DL_ACC_ODR100		0x08	/* 100Hz output data rate */
#define H3LIS331DL_ACC_ODR400		0x10	/* 400Hz output data rate */
#define H3LIS331DL_ACC_ODR1000		0x18	/* 1000Hz output data rate */

#define FUZZ		0
#define FLAT		0

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1	0
#define	RES_CTRL_REG2	1
#define	RES_CTRL_REG3	2
#define	RES_CTRL_REG4	3
#define	RES_CTRL_REG5	4
#define	RES_REFERENCE	5

#define	RES_INT_CFG1	6
#define	RES_INT_THS1	7
#define	RES_INT_DUR1	8
#define	RES_INT_CFG2	9
#define	RES_INT_THS2	10
#define	RES_INT_DUR2	11
/* end RESUME STATE INDICES */

static struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} h3lis331dl_odr_table[] = {
	{ 1, H3LIS331DL_ACC_PM_NORMAL | H3LIS331DL_ACC_ODR1000 },
	{ 3, H3LIS331DL_ACC_PM_NORMAL | H3LIS331DL_ACC_ODR400 },
	{ 10, H3LIS331DL_ACC_PM_NORMAL | H3LIS331DL_ACC_ODR100 },
	{ 20, H3LIS331DL_ACC_PM_NORMAL | H3LIS331DL_ACC_ODR50},
	/* low power settings, max low pass filter cut-off freq */
	{ 100, H3LIS331DL_ACC_ODR10 | H3LIS331DL_ACC_ODR1000 },
	{ 200, H3LIS331DL_ACC_ODR5 | H3LIS331DL_ACC_ODR1000 },
	{ 5000, H3LIS331DL_ACC_ODR2 | H3LIS331DL_ACC_ODR1000 },
	{ 1000, H3LIS331DL_ACC_ODR1 | H3LIS331DL_ACC_ODR1000 },
	{ 2000, H3LIS331DL_ACC_ODRHALF | H3LIS331DL_ACC_ODR1000 },
};

static struct h3lis331dl_platform_data default_h3lis331dl_pdata = {
	.g_range = H3LIS331DL_ACC_G_100G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = H3LIS331DL_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = H3LIS331DL_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = H3LIS331DL_ACC_DEFAULT_INT2_GPIO,
};

static int h3lis331dl_hw_init(struct h3lis331dl_data *acc)
{
	int err = -1;
	u8 buf[5];

	printk(KERN_INFO "%s: hw init start\n", H3LIS331DL_ACC_DEV_NAME);

	err = acc->tf->read(acc, WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(acc->dev, "Error reading WHO_AM_I: is device "
			 "available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_H3LIS331DL_ACC) {
		dev_err(acc->dev,
			"device unknown. Expected: 0x%x,"
			" Replies: 0x%x\n", WHOAMI_H3LIS331DL_ACC, buf[0]);
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
	printk(KERN_INFO "%s: hw init done\n", H3LIS331DL_ACC_DEV_NAME);

	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(acc->dev, "hw init error: %d\n", err);

	return err;
}

static void h3lis331dl_device_power_off(struct h3lis331dl_data *acc)
{
	int err;
	u8 buf = H3LIS331DL_ACC_PM_OFF;

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

static int h3lis331dl_device_power_on(struct h3lis331dl_data *acc)
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
		err = h3lis331dl_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			h3lis331dl_device_power_off(acc);
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

static irqreturn_t h3lis331dl_isr1(int irq, void *dev)
{
	struct h3lis331dl_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	printk(KERN_INFO "%s: isr1 queued\n", H3LIS331DL_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t h3lis331dl_isr2(int irq, void *dev)
{
	struct h3lis331dl_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk(KERN_INFO "%s: isr2 queued\n", H3LIS331DL_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void h3lis331dl_irq1_work_func(struct work_struct *work)
{
	struct h3lis331dl_data *acc =

	container_of(work, struct h3lis331dl_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:h3lis331dl_get_int1_source(acc); */
	printk(KERN_INFO "%s: IRQ1 triggered\n", H3LIS331DL_ACC_DEV_NAME);
	enable_irq(acc->irq1);
}

static void h3lis331dl_irq2_work_func(struct work_struct *work)
{
	struct h3lis331dl_data *acc =

	container_of(work, struct h3lis331dl_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:h3lis331dl_get_tap_source(acc); */
	printk(KERN_INFO "%s: IRQ2 triggered\n", H3LIS331DL_ACC_DEV_NAME);
	enable_irq(acc->irq2);
}

int h3lis331dl_update_g_range(struct h3lis331dl_data *acc, u8 new_g_range)
{
	int err = -1;
	u8 sensitivity;
	u8 updated_val;
	u8 init_val;
	u8 new_val;

	switch (new_g_range) {
	case H3LIS331DL_ACC_G_100G:
		sensitivity = SENSITIVITY_100G;
		break;
	case H3LIS331DL_ACC_G_200G:
		sensitivity = SENSITIVITY_200G;
		break;
	case H3LIS331DL_ACC_G_400G:
		sensitivity = SENSITIVITY_400G;
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
		updated_val = ((H3LIS331DL_ACC_FS_MASK & new_val) |
			       ((~H3LIS331DL_ACC_FS_MASK) & init_val));
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

int h3lis331dl_update_odr(struct h3lis331dl_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(h3lis331dl_odr_table) - 1; i >= 0; i--) {
		if (h3lis331dl_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config = h3lis331dl_odr_table[i].mask | H3LIS331DL_ACC_ENABLE_ALL_AXES;

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

static int h3lis331dl_get_acceleration_data(struct h3lis331dl_data *acc, int *xyz)
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

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x]) :
		  (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y]) :
		  (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z]) :
		  (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG
		printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
		       H3LIS331DL_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif

	return err;
}

static void h3lis331dl_report_values(struct h3lis331dl_data *acc, int *xyz)
{
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int h3lis331dl_enable(struct h3lis331dl_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = h3lis331dl_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		/* Android:
		 * Udpate polling rate in case odr was changed while sensor disabled */
		h3lis331dl_update_odr(acc, acc->pdata->poll_interval);
		schedule_delayed_work(&acc->input_work,
				msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int h3lis331dl_disable(struct h3lis331dl_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work(&acc->input_work);
		h3lis331dl_device_power_off(acc);
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
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
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	u8 x;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	x = val;

	return acc->tf->write(acc, reg, 1, &x);
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	h3lis331dl_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	char val;
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->g_range ;
	switch (val) {
	case H3LIS331DL_ACC_G_100G:
		range = 2;
		break;
	case H3LIS331DL_ACC_G_200G:
		range = 4;
		break;
	case H3LIS331DL_ACC_G_400G:
		range = 8;
		break;
	}
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	h3lis331dl_update_g_range(acc, val);
	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		h3lis331dl_enable(acc);
	else
		h3lis331dl_disable(acc);

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
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
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
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
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
	struct h3lis331dl_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
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

static void h3lis331dl_input_work_func(struct work_struct *work)
{
	struct h3lis331dl_data *acc;
	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			    struct h3lis331dl_data, input_work);

	mutex_lock(&acc->lock);
	err = h3lis331dl_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(acc->dev, "get_acceleration_data failed\n");
	else
		h3lis331dl_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work,
			      msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef H3LIS331DL_EN_OPEN
int h3lis331dl_input_open(struct input_dev *input)
{
	struct h3lis331dl_data *acc = input_get_drvdata(input);

	return h3lis331dl_enable(acc);
}

void h3lis331dl_input_close(struct input_dev *dev)
{
	struct h3lis331dl_data *acc = input_get_drvdata(dev);

	h3lis331dl_disable(acc);
}
#endif /* H3LIS331DL_EN_OPEN */

static int h3lis331dl_validate_pdata(struct h3lis331dl_data *acc)
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

static int h3lis331dl_input_init(struct h3lis331dl_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, h3lis331dl_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(acc->dev, "input device allocation failed\n");
		return err;
	}

#ifdef H3LIS331DL_EN_OPEN
	acc->input_dev->open = h3lis331dl_input_open;
	acc->input_dev->close = h3lis331dl_input_close;
#else /* H3LIS331DL_EN_OPEN */
	acc->input_dev->open = NULL;
	acc->input_dev->close = NULL;
#endif /* H3LIS331DL_EN_OPEN */
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

static void h3lis331dl_input_cleanup(struct h3lis331dl_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

#ifdef CONFIG_OF
static u32 h3lis331dl_parse_dt(struct h3lis331dl_data *acc)
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

int h3lis331dl_common_probe(struct h3lis331dl_data *acc)
{
	int err = -1;

	pr_info("%s: probe start.\n", H3LIS331DL_ACC_DEV_NAME);

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->pdata = kzalloc(sizeof(struct h3lis331dl_platform_data), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(acc->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (acc->dev->platform_data == NULL) {
		memcpy(acc->pdata, &default_h3lis331dl_pdata,
		       sizeof(struct h3lis331dl_platform_data));
		dev_info(acc->dev, "using default plaform_data\n");
	} else {
		memcpy(acc->pdata, acc->dev->platform_data,
		       sizeof(struct h3lis331dl_platform_data));
	}

#ifdef CONFIG_OF
	/* override default gpio_irq[1,2] if required */
	h3lis331dl_parse_dt(acc);
#endif

	err = h3lis331dl_validate_pdata(acc);
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
		       "mapped on gpio:%d\n", H3LIS331DL_ACC_DEV_NAME, __func__,
		        acc->irq1, acc->pdata->gpio_int1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		printk(KERN_INFO "%s: %s has set irq2 to irq: %d "
		       "mapped on gpio:%d\n", H3LIS331DL_ACC_DEV_NAME, __func__,
		       acc->irq2, acc->pdata->gpio_int2);
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = H3LIS331DL_ACC_ENABLE_ALL_AXES;

	err = h3lis331dl_device_power_on(acc);
	if (err < 0) {
		dev_err(acc->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = h3lis331dl_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(acc->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	err = h3lis331dl_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(acc->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = h3lis331dl_input_init(acc);
	if (err < 0) {
		dev_err(acc->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(acc->dev);
	if (err < 0) {
		dev_err(acc->dev,
			"device %s register failed\n",  H3LIS331DL_ACC_DEV_NAME);
		goto err_input_cleanup;
	}

	h3lis331dl_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if (acc->pdata->gpio_int1 >= 0) {
		INIT_WORK(&acc->irq1_work, h3lis331dl_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("h3lis331dl_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev, "cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, h3lis331dl_isr1,
				  IRQF_TRIGGER_RISING, "h3lis331dl_irq1", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		INIT_WORK(&acc->irq2_work, h3lis331dl_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("h3lis331dl_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev, "cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, h3lis331dl_isr2,
				  IRQF_TRIGGER_RISING, "h3lis331dl_irq2", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq2 failed: %d\n", err);
				goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	mutex_unlock(&acc->lock);

	dev_info(acc->dev, "%s: probed\n", H3LIS331DL_ACC_DEV_NAME);

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
	h3lis331dl_input_cleanup(acc);
err_power_off:
	h3lis331dl_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	printk(KERN_ERR "%s: Driver Init failed\n", H3LIS331DL_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(h3lis331dl_common_probe);

int h3lis331dl_common_remove(struct h3lis331dl_data *acc)
{
	if(acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if(acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	h3lis331dl_input_cleanup(acc);
	h3lis331dl_device_power_off(acc);
	remove_sysfs_interfaces(acc->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);

	return 0;
}
EXPORT_SYMBOL(h3lis331dl_common_remove);

#ifdef CONFIG_PM_SLEEP
int h3lis331dl_common_resume(struct h3lis331dl_data *acc)
{
	if (acc->on_before_suspend)
		return h3lis331dl_enable(acc);

	return 0;
}
EXPORT_SYMBOL(h3lis331dl_common_resume);

int h3lis331dl_common_suspend(struct h3lis331dl_data *acc)
{
	acc->on_before_suspend = atomic_read(&acc->enabled);

	return h3lis331dl_disable(acc);
}
EXPORT_SYMBOL(h3lis331dl_common_suspend);
#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("h3lis331dl accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
