/* Copyright (C) 2016 STMicroelectronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * File Name		: ais328dq_acc.c
 * Authors		: VMA - Volume Mems & Analog Division
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Lorenzo Bianconi (lorenzo.bianconi@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V 1.0.1
 * Date			: 2016/05/27
 * Description		: AIS328DQ 3D accelerometer sensor LDD
 *
 ******************************************************************************
 * Revision 1.0.0 2011/03/02
 *	first release
 * Revision 1.0.1 2012/10/07
 *	corrects default permissions on sysfs files
 *****************************************************************************/

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

#include "ais328dq.h"

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/

#define AXISDATA_REG		0x28
#define WHOAMI_AIS328DQ_ACC	0x32	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS ADDRESSES	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define CTRL_REG1		0x20	/*				*/
#define CTRL_REG2		0x21	/*				*/
#define CTRL_REG3		0x22	/*				*/
#define CTRL_REG4		0x23	/*				*/
#define	CTRL_REG5		0x24	/*				*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold	*/
#define	INT_DUR2		0x37	/*	interrupt 2 duration	*/
/*	end CONTROL REGISTRES ADDRESSES	*/

#define AIS328DQ_ACC_ENABLE_ALL_AXES	0x07
#define AIS328DQ_SELFTEST_EN		0x02
#define AIS328DQ_SELFTEST_DIS		0x00
#define AIS328DQ_SELFTEST_POS		0x00
#define AIS328DQ_SELFTEST_NEG		0x08

#define AIS328DQ_ACC_BDU_EN		0x80

/* Accelerometer output data rate  */
#define AIS328DQ_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define AIS328DQ_ACC_ODR1		0x60	/* 1Hz output data rate */
#define AIS328DQ_ACC_ODR2		0x80	/* 2Hz output data rate */
#define AIS328DQ_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define AIS328DQ_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define AIS328DQ_ACC_ODR50		0x00	/* 50Hz output data rate */
#define AIS328DQ_ACC_ODR100		0x08	/* 100Hz output data rate */
#define AIS328DQ_ACC_ODR400		0x10	/* 400Hz output data rate */
#define AIS328DQ_ACC_ODR1000		0x18	/* 1000Hz output data rate */

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

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define MS_TO_NS(x)	((x)*1000000L)

static struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} ais328dq_acc_odr_table[] = {
	{1, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR1000},
	{3, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR400},
	{10, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR100},
	{20, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR50},
	/* low power settings, max low pass filter cut-off freq */
	{100, AIS328DQ_ACC_ODR10 | AIS328DQ_ACC_ODR1000},
	{200, AIS328DQ_ACC_ODR5 | AIS328DQ_ACC_ODR1000},
	{5000, AIS328DQ_ACC_ODR2 | AIS328DQ_ACC_ODR1000 },
	{1000, AIS328DQ_ACC_ODR1 | AIS328DQ_ACC_ODR1000 },
	{2000, AIS328DQ_ACC_ODRHALF | AIS328DQ_ACC_ODR1000 },
};

static struct ais328dq_acc_platform_data def_pdata = {
	.poll_interval = 100,
	.min_interval = 1,
	.g_range = 2,

	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.gpio_int1 = -1,
	.gpio_int2 = -1,
};

static inline s64 ais328dq_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

static int ais328dq_acc_hw_init(struct ais328dq_acc_data *acc)
{
	int err = -1;
	u8 buf[4];

	printk(KERN_INFO "%s: hw init start\n", AIS328DQ_ACC_DEV_NAME);

	err = acc->tf->read(acc->dev, WHO_AM_I, 1, buf);
	if (err < 0){
		dev_warn(acc->dev, "Error reading WHO_AM_I: is device "
			"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_AIS328DQ_ACC) {
		dev_err(acc->dev,
			"device unknown. Expected: 0x%x,"
			" Replies: 0x%x\n", WHOAMI_AIS328DQ_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = acc->resume_state[RES_CTRL_REG1];
	err = acc->tf->write(acc->dev, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS1];
	buf[1] = acc->resume_state[RES_INT_DUR1];
	err = acc->tf->write(acc->dev, INT_THS1, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_CFG1];
	err = acc->tf->write(acc->dev, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS2];
	buf[1] = acc->resume_state[RES_INT_DUR2];
	err = acc->tf->write(acc->dev, INT_THS2, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_CFG2];
	err = acc->tf->write(acc->dev, INT_CFG2, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_CTRL_REG2];
	buf[1] = acc->resume_state[RES_CTRL_REG3];
	buf[2] = acc->resume_state[RES_CTRL_REG4];
	buf[3] = acc->resume_state[RES_CTRL_REG5];
	err = acc->tf->write(acc->dev, CTRL_REG2, 4, buf);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", AIS328DQ_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(acc->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void ais328dq_acc_device_power_off(struct ais328dq_acc_data *acc)
{
	int err;
	u8 buf[1] = {AIS328DQ_ACC_PM_OFF};

	err = acc->tf->write(acc->dev, CTRL_REG1, 1, buf);
	if (err < 0)
		dev_err(acc->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}

}

static int ais328dq_acc_device_power_on(struct ais328dq_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(acc->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = ais328dq_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			ais328dq_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t ais328dq_acc_isr1(int irq, void *dev)
{
	struct ais328dq_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	printk(KERN_INFO "%s: isr1 queued\n", AIS328DQ_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t ais328dq_acc_isr2(int irq, void *dev)
{
	struct ais328dq_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk(KERN_INFO "%s: isr2 queued\n", AIS328DQ_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void ais328dq_acc_irq1_work_func(struct work_struct *work)
{

	struct ais328dq_acc_data *acc =
	container_of(work, struct ais328dq_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:ais328dq_acc_get_int1_source(acc); */
	;
	/*  */
	enable_irq(acc->irq1);
}

static void ais328dq_acc_irq2_work_func(struct work_struct *work)
{

	struct ais328dq_acc_data *acc =
	container_of(work, struct ais328dq_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:ais328dq_acc_get_tap_source(acc); */
	;
	/*  */

	enable_irq(acc->irq2);
}

int ais328dq_acc_update_g_range(struct ais328dq_acc_data *acc, u8 range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[1];
	u8 updated_val;
	u8 init_val;
	u8 new_val, new_g_range;
	u8 mask = AIS328DQ_ACC_FS_MASK;

	switch (range) {
	case 2:
		new_g_range = AIS328DQ_ACC_G_2G;
		sensitivity = SENSITIVITY_2G;
		break;
	case 4:
		new_g_range = AIS328DQ_ACC_G_4G;
		sensitivity = SENSITIVITY_4G;
		break;
	case 8:
		new_g_range = AIS328DQ_ACC_G_8G;
		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(acc->dev, "invalid g range requested: %u\n", range);
		return -EINVAL;
	}

	/* Set configuration register 4, which contains g range setting
	 *  NOTE: this is a straight overwrite because this driver does
	 *  not use any of the other configuration bits in this
	 *  register.  Should this become untrue, we will have to read
	 *  out the value and only change the relevant bits --XX----
	 *  (marked by X) */
	err = acc->tf->read(acc->dev, CTRL_REG4, 1, buf);
	if (err < 0)
		goto error;

	init_val = buf[0];
	new_val = new_g_range;
	updated_val = ((mask & new_val) | ((~mask) & init_val));

	if (atomic_read(&acc->enabled)) {
		buf[0] = updated_val;
		err = acc->tf->write(acc->dev, CTRL_REG4, 1, buf);
		if (err < 0)
			goto error;
	}

	acc->resume_state[RES_CTRL_REG4] = updated_val;
	acc->sensitivity = sensitivity;

	return 0;

error:
	dev_err(acc->dev, "update g range failed 0x%x: %d\n",
		buf[0], err);

	return err;
}

int ais328dq_acc_update_odr(struct ais328dq_acc_data *acc,
			    int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[1];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(ais328dq_acc_odr_table) - 1; i >= 0; i--) {
		if (ais328dq_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}

	config[0] = ais328dq_acc_odr_table[i].mask;
	config[0] |= AIS328DQ_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		err = acc->tf->write(acc->dev, CTRL_REG1, 1, config);
		if (err < 0)
			goto error;
	}
	acc->resume_state[RES_CTRL_REG1] = config[0];
	acc->delta_ts = MS_TO_NS(poll_interval_ms);

	return err;

error:
	dev_err(acc->dev, "update odr failed 0x%x: %d\n",
		config[0], err);

	return err;
}

static int ais328dq_acc_register_update(struct ais328dq_acc_data *acc,
			u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = acc->tf->read(acc->dev, reg_address, 1, buf);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | (~mask & init_val));
		err = acc->tf->write(acc->dev, reg_address, 1, &updated_val);
	}

	return err;
}



static int ais328dq_acc_selftest(struct ais328dq_acc_data *acc, u8 enable)
{
	int err = -1;
	u8 buf[2]={0x00,0x00};
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = 0x0A;
	if (enable > 0)
		bit_values = AIS328DQ_SELFTEST_EN |
						AIS328DQ_SELFTEST_POS;
	else
		bit_values = AIS328DQ_SELFTEST_DIS |
						AIS328DQ_SELFTEST_POS;
	if (atomic_read(&acc->enabled)) {
		mutex_lock(&acc->lock);
		err = ais328dq_acc_register_update(acc, buf, reg_address,
				mask, bit_values);
		acc->selftest_enabled = enable;
		mutex_unlock(&acc->lock);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				( ~mask & acc->resume_state[RES_CTRL_REG4]));
	}
	return err;
}

static int ais328dq_acc_get_acceleration_data(struct ais328dq_acc_data *acc,
					       int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	err = acc->tf->read(acc->dev, AXISDATA_REG, 6, acc_data);
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

	return err;
}

static void ais328dq_acc_report_values(struct ais328dq_acc_data *acc,
					int *xyz)
{
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    acc->timestamp >> 32);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    acc->timestamp & 0xffffffff);
	input_sync(acc->input_dev);
}

int ais328dq_acc_enable(struct ais328dq_acc_data *acc)
{
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		int err;

		mutex_lock(&acc->lock);
		err = ais328dq_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			mutex_unlock(&acc->lock);
			return err;
		}

		hrtimer_start(&acc->hr_timer, ktime_set(0, acc->delta_ts),
			      HRTIMER_MODE_REL);

		mutex_unlock(&acc->lock);
	}

	return 0;
}
EXPORT_SYMBOL(ais328dq_acc_enable);

int ais328dq_acc_disable(struct ais328dq_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		mutex_lock(&acc->lock);
		ais328dq_acc_device_power_off(acc);
		mutex_unlock(&acc->lock);

		cancel_work_sync(&acc->poll_work);
		hrtimer_cancel(&acc->hr_timer);
	}

	return 0;
}
EXPORT_SYMBOL(ais328dq_acc_disable);

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	u8 data;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	int rc = 0;

	mutex_lock(&acc->lock);
	rc = acc->tf->read(acc->dev, reg, 1, &data);
	mutex_unlock(&acc->lock);

	/*TODO: error need to be managed */
	return sprintf(buf, "0x%02x\n", data);

}

static int write_reg(struct device *dev, const char *buf, u8 reg)
{
	int rc = 0;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	u8 x[1];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	x[0] = val;
	rc = acc->tf->write(acc->dev, reg, 1, x);
	mutex_unlock(&acc->lock);

	/*TODO: error need to be managed */
	return rc;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	ais328dq_acc_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&acc->lock);
	range = acc->pdata->g_range ;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	int err;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	err = ais328dq_acc_update_g_range(acc, val);
	if (err == 0)
		acc->pdata->g_range = val;
	mutex_unlock(&acc->lock);

	return !err ? size : err;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		ais328dq_acc_enable(acc);
	else
		ais328dq_acc_disable(acc);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->selftest_enabled;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	ais328dq_acc_selftest(acc, val);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_intconfig2(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG2);
}

static ssize_t attr_get_intconfig2(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG2);
}

static ssize_t attr_set_duration2(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR2);
}

static ssize_t attr_get_duration2(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR2);
}

static ssize_t attr_set_thresh2(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS2);
}

static ssize_t attr_get_thresh2(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS2);
}
static ssize_t attr_get_source2(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC2);
}




#ifdef AIS328DQ_DEBUG
/* PAY ATTENTION: These AIS328DQ_DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = val;
	rc = acc->tf->write(acc->dev, acc->reg_addr, 1, x);
	mutex_unlock(&acc->lock);

	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	err = acc->tf->read(acc->dev, acc->reg_addr, 1, &data);
	mutex_unlock(&acc->lock);

	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
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
	__ATTR(enable_selftest, 0664, attr_get_selftest, attr_set_selftest),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(int2_config, 0664, attr_get_intconfig2, attr_set_intconfig2),
	__ATTR(int2_duration, 0664, attr_get_duration2, attr_set_duration2),
	__ATTR(int2_threshold, 0664, attr_get_thresh2, attr_set_thresh2),
	__ATTR(int2_source, 0444, attr_get_source2, NULL),
#ifdef AIS328DQ_DEBUG
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

static enum hrtimer_restart ais328dq_timer_poll(struct hrtimer *timer)
{
	struct ais328dq_acc_data *acc;

	acc = container_of((struct hrtimer *)timer, struct ais328dq_acc_data,
			    hr_timer);

	acc->timestamp = ais328dq_get_time_ns();

	queue_work(acc->work_queue, &acc->poll_work);

	return HRTIMER_NORESTART;
}

static void ais328dq_poll_work_func(struct work_struct *work)
{
	struct ais328dq_acc_data *acc;
	int xyz[3] = { 0 };
	int err;
	s64 ts, delta;

	acc = container_of((struct work_struct *)work,
			    struct ais328dq_acc_data, poll_work);

	ts = ais328dq_get_time_ns();
	delta = acc->delta_ts - (ts - acc->timestamp);
	hrtimer_start(&acc->hr_timer, ktime_set(0, delta),
		      HRTIMER_MODE_REL);

	mutex_lock(&acc->lock);
	err = ais328dq_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(acc->dev, "get_acceleration_data failed\n");
	else
		ais328dq_acc_report_values(acc, xyz);

	mutex_unlock(&acc->lock);
}

static int ais328dq_acc_validate_pdata(struct ais328dq_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
					acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
	    acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2) {
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

static int ais328dq_acc_input_init(struct ais328dq_acc_data *acc)
{
	int err;

	acc->work_queue = create_workqueue("ais328dq_wq");
	if (!acc->work_queue)
		return -ENOMEM;

	hrtimer_init(&acc->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acc->hr_timer.function = &ais328dq_timer_poll;

	INIT_WORK(&acc->poll_work, ais328dq_poll_work_func);

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(acc->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->name = AIS328DQ_ACC_DEV_NAME;
	acc->input_dev->id.bustype = acc->bus_type;
	acc->input_dev->dev.parent = acc->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(INPUT_EVENT_TYPE, acc->input_dev->evbit);
	set_bit(INPUT_EVENT_X, acc->input_dev->mscbit);
	set_bit(INPUT_EVENT_Y, acc->input_dev->mscbit);
	set_bit(INPUT_EVENT_Z, acc->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_MSB, acc->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, acc->input_dev->mscbit);


	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(acc->dev,
			"unable to register input device %s\n",
			acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void ais328dq_acc_input_cleanup(struct ais328dq_acc_data *acc)
{
	if (acc->work_queue) {
		flush_workqueue(acc->work_queue);
		destroy_workqueue(acc->work_queue);
		acc->work_queue = NULL;
	}
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

int ais328dq_acc_probe(struct ais328dq_acc_data *acc)
{
	int err = -1;

	if (acc->dev->platform_data == NULL) {
		dev_info(acc->dev,
			 "platform data is NULL. using default one.\n");
		acc->dev->platform_data = &def_pdata;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(acc->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	memcpy(acc->pdata, acc->dev->platform_data, sizeof(*acc->pdata));

	err = ais328dq_acc_validate_pdata(acc);
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

	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		printk(KERN_INFO "%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
		AIS328DQ_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		printk(KERN_INFO "%s: %s has set irq2 to irq: %d "
						"mapped on gpio:%d\n",
			AIS328DQ_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = AIS328DQ_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG4] = AIS328DQ_ACC_BDU_EN;

	err = ais328dq_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(acc->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = ais328dq_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(acc->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	err = ais328dq_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(acc->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = ais328dq_acc_input_init(acc);
	if (err < 0) {
		dev_err(acc->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(acc->dev);
	if (err < 0) {
		dev_err(acc->dev,
			"device AIS328DQ_ACC_DEV_NAME "
						"sysfs register failed\n");
		goto err_input_cleanup;
	}

	ais328dq_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, ais328dq_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("ais328dq_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, ais328dq_acc_isr1,
			IRQF_TRIGGER_RISING, "ais328dq_acc_irq1", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, ais328dq_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("ais328dq_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev,
				"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, ais328dq_acc_isr2,
			IRQF_TRIGGER_RISING, "ais328dq_acc_irq2", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq2 failed: %d\n", err);
				goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	mutex_unlock(&acc->lock);

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
	ais328dq_acc_input_cleanup(acc);
err_power_off:
	ais328dq_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);

	return err;
}
EXPORT_SYMBOL(ais328dq_acc_probe);

int ais328dq_acc_remove(struct ais328dq_acc_data *acc)
{
	if (acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if (acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	ais328dq_acc_input_cleanup(acc);
	ais328dq_acc_device_power_off(acc);
	remove_sysfs_interfaces(acc->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);

	return 0;
}
EXPORT_SYMBOL(ais328dq_acc_remove);

MODULE_DESCRIPTION("ais328dq accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, STMicroelectronics");
MODULE_AUTHOR("Lorenzo Bianconi, STMicroelectronics");
MODULE_LICENSE("GPL v2");

