/*
 * STMicroelectronics h3lis100dl Accelerometer driver
 * Based on lis331dlh input driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Armando Visconti <armando.visconti@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
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
#include <linux/version.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "h3lis100dl.h"

#define MS_TO_NS(msec)          ((msec) * 1000 * 1000)

//#define	DEBUG	1

#define SENSITIVITY_100G	780	/**	mg/LSB	*/

#define AXISDATA_REG		0x28
#define WHOAMI_H3LIS100DL	0x32	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
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
/*	end CONTROL REGISTRES	*/

#define H3LIS100DL_ENABLE_ALL_AXES	0x07
#define H3LIS100DL_SELFTEST_EN	0x02
#define H3LIS100DL_SELFTEST_DIS	0x00
#define H3LIS100DL_SELFTEST_POS	0x00
#define H3LIS100DL_SELFTEST_NEG	0x08

/* Accelerometer output data rate  */
#define H3LIS100DL_ODRHALF	0x40	/* 0.5Hz output data rate */
#define H3LIS100DL_ODR1		0x60	/* 1Hz output data rate */
#define H3LIS100DL_ODR2		0x80	/* 2Hz output data rate */
#define H3LIS100DL_ODR5		0xA0	/* 5Hz output data rate */
#define H3LIS100DL_ODR10	0xC0	/* 10Hz output data rate */
#define H3LIS100DL_ODR50	0x00	/* 50Hz output data rate */
#define H3LIS100DL_ODR100	0x08	/* 100Hz output data rate */
#define H3LIS100DL_ODR400	0x10	/* 400Hz output data rate */
#define H3LIS100DL_ODR1000	0x18	/* 1000Hz output data rate */

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
} h3lis100dl_odr_table[] = {
	{1, H3LIS100DL_PM_NORMAL | H3LIS100DL_ODR1000},
	{3, H3LIS100DL_PM_NORMAL | H3LIS100DL_ODR400},
	{10, H3LIS100DL_PM_NORMAL | H3LIS100DL_ODR100},
	{20, H3LIS100DL_PM_NORMAL | H3LIS100DL_ODR50},
	/* low power settings, max low pass filter cut-off freq */
	{100, H3LIS100DL_ODR10 | H3LIS100DL_ODR1000},
	{200, H3LIS100DL_ODR5 | H3LIS100DL_ODR1000},
	{5000, H3LIS100DL_ODR2 | H3LIS100DL_ODR1000 },
	{1000, H3LIS100DL_ODR1 | H3LIS100DL_ODR1000 },
	{2000, H3LIS100DL_ODRHALF | H3LIS100DL_ODR1000 },
};

static inline int64_t h3lis100dl_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static struct workqueue_struct *h3lis100dl_workqueue;

static int h3lis100dl_hw_init(struct h3lis100dl_data *acc)
{
	int err = -1;
	u8 buf[6];

#ifdef DEBUG
	printk(KERN_INFO "%s: hw init start\n", H3LIS100DL_DEV_NAME);
#endif

	err = acc->tf->read(acc, WHO_AM_I, 1, buf);
	if (err < 0){
		dev_warn(acc->dev, "Error reading WHO_AM_I: is device "
			 "available/working?\n");
		goto err_firstread;
	} else {
		acc->hw_working = 1;
	}
	if (buf[0] != WHOAMI_H3LIS100DL) {
		dev_err(acc->dev,
			"device unknown. Expected: 0x%x,"
			" Replies: 0x%x\n", WHOAMI_H3LIS100DL, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = acc->resume_state[RES_CTRL_REG1];
	err = acc->tf->write(acc, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS1];
	buf[1] = acc->resume_state[RES_INT_DUR1];
	err = acc->tf->write(acc, (AUTO_INCREMENT | INT_THS1), 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_CFG1];
	err = acc->tf->write(acc, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS2];
	buf[1] = acc->resume_state[RES_INT_DUR2];
	err = acc->tf->write(acc, (AUTO_INCREMENT | INT_THS2), 2, buf);
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
	err = acc->tf->write(acc, (AUTO_INCREMENT | CTRL_REG2), 4, buf);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;

#ifdef DEBUG
	printk(KERN_INFO "%s: hw init done\n", H3LIS100DL_DEV_NAME);
#endif
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

static void h3lis100dl_device_power_off(struct h3lis100dl_data *acc)
{
	int err;
	u8 buf = H3LIS100DL_PM_OFF;

	err = acc->tf->write(acc, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(acc->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized)
		acc->hw_initialized = 0;
}

static int h3lis100dl_device_power_on(struct h3lis100dl_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(acc->dev, "power_on failed: %d\n", err);
			return err;
		}
	}

	if (!acc->hw_initialized) {
		err = h3lis100dl_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			h3lis100dl_device_power_off(acc);
			return err;
		}
	}

	return 0;
}

/*
 * hrtimer callback - it is called every polling_rate msec.
 */
enum hrtimer_restart h3lis100dl_timer_func_queue_work(struct hrtimer *timer)
{
	struct h3lis100dl_data *acc;


	acc = container_of((struct hrtimer *)timer,
			   struct h3lis100dl_data, hr_timer);

	acc->timestamp = h3lis100dl_get_time_ns();
	queue_work(h3lis100dl_workqueue, &acc->input_work);

	return HRTIMER_NORESTART;
}

int h3lis100dl_update_g_range(struct h3lis100dl_data *acc, u8 new_g_range)
{
	int err = -1;

	u16 sensitivity;
	u8 buf;
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = H3LIS100DL_FS_MASK;

	sensitivity = SENSITIVITY_100G;

	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		err = acc->tf->read(acc, CTRL_REG4, 1, &buf);
		if (err < 0)
			goto error;

		init_val = buf;
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf = updated_val;
		err = acc->tf->write(acc, CTRL_REG4, 1, &buf);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}

	return err;
error:
	dev_err(acc->dev, "update g range failed 0x%x,0x%x: %d\n",
		CTRL_REG4, buf, err);

	return err;
}

int h3lis100dl_update_odr(struct h3lis100dl_data *acc,
			  int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(h3lis100dl_odr_table) - 1; i >= 0; i--) {
		if (h3lis100dl_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config = h3lis100dl_odr_table[i].mask;

	config |= H3LIS100DL_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		flush_workqueue(h3lis100dl_workqueue);

		err = acc->tf->write(acc, CTRL_REG1, 1, &config);
		if (err < 0)
			goto error;
	}
	acc->resume_state[RES_CTRL_REG1] = config;
	acc->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));

	return err;

error:
	dev_err(acc->dev, "update odr failed 0x%x,0x%x: %d\n",
		CTRL_REG1, config, err);

	return err;
}

static int h3lis100dl_register_write(struct h3lis100dl_data *acc, u8 *buf,
				     u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = new_value;
	err = acc->tf->write(acc, reg_address, 1, buf);
	if (err < 0)
		return err;

	return err;
}

static int h3lis100dl_register_read(struct h3lis100dl_data *acc, u8 *buf,
				    u8 reg_address)
{

	int err = -1;

	err = acc->tf->read(acc, reg_address, 1, buf);

	return err;
}

static int h3lis100dl_register_update(struct h3lis100dl_data *acc,
				      u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = h3lis100dl_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = h3lis100dl_register_write(acc, buf, reg_address,
						updated_val);
	}
	return err;
}

static int h3lis100dl_selftest(struct h3lis100dl_data *acc, u8 enable)
{
	int err = -1;
	u8 buf[2] = { 0x00, 0x00 };
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = 0x0A;
	if (enable > 0)
		bit_values = H3LIS100DL_SELFTEST_EN |
			     H3LIS100DL_SELFTEST_POS;
	else
		bit_values = H3LIS100DL_SELFTEST_DIS |
			     H3LIS100DL_SELFTEST_POS;
	if (atomic_read(&acc->enabled)) {
		mutex_lock(&acc->lock);
		err = h3lis100dl_register_update(acc, buf, reg_address,
						 mask, bit_values);
		acc->selftest_enabled = enable;
		mutex_unlock(&acc->lock);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_CTRL_REG4]));
	}
	return err;
}

static int h3lis100dl_get_acceleration_data(struct h3lis100dl_data *acc,
					    int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	err = acc->tf->read(acc, (AUTO_INCREMENT | AXISDATA_REG), 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = (s8)acc_data[1];
	hw_d[1] = (s8)acc_data[3];
	hw_d[2] = (s8)acc_data[5];

	xyz[0] = hw_d[0] * acc->sensitivity;
	xyz[1] = hw_d[1] * acc->sensitivity;
	xyz[2] = hw_d[2] * acc->sensitivity;

#ifdef DEBUG
	printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
	       H3LIS100DL_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	return err;
}

static void h3lis100dl_report_values(struct h3lis100dl_data *acc,
				     int *xyz)
{
	struct input_dev  *input = acc->input_dev;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    acc->timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    acc->timestamp & 0xffffffff);
	input_sync(input);
}

static int h3lis100dl_enable(struct h3lis100dl_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = h3lis100dl_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}

		hrtimer_start(&acc->hr_timer, acc->ktime, HRTIMER_MODE_REL);
	}

	return 0;
}

static int h3lis100dl_disable(struct h3lis100dl_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_work_sync(&acc->input_work);
		hrtimer_cancel(&acc->hr_timer);
		h3lis100dl_device_power_off(acc);
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	int rc = 0;
	u8 data;

	rc = acc->tf->read(acc, reg, 1, &data);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static int write_reg(struct device *dev, const char *buf, u8 reg)
{
	int rc = 0;
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 x[2];

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	x[0] = val;
	rc = acc->tf->write(acc, reg, 1, x);
	/*TODO: error need to be managed */
	return rc;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	h3lis100dl_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		h3lis100dl_enable(acc);
	else
		h3lis100dl_disable(acc);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int val;
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->selftest_enabled;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	h3lis100dl_selftest(acc, val);

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
				struct device_attribute *attr,
				const char *buf, size_t size)
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
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	u8 reg;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	reg = acc->reg_addr;
	mutex_unlock(&acc->lock);

	rc = acc->tf->write(acc, reg, 1, &val);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 reg, val;

	mutex_lock(&acc->lock);
	reg = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = acc->tf->read(acc, reg, 1, val);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", val);

	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct h3lis100dl_data *acc = dev_get_drvdata(dev);
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

/*
 * Workqueue callback - it is called as soon as a job in the queue
 * is scheduled.
 */
static void h3lis100dl_input_work_func(struct work_struct *work)
{
	struct h3lis100dl_data *acc;
	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct work_struct *)work,
			   struct h3lis100dl_data, input_work);

	hrtimer_start(&acc->hr_timer, acc->ktime, HRTIMER_MODE_REL);

	err = h3lis100dl_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(acc->dev, "get_acceleration_data failed\n");
	else
		h3lis100dl_report_values(acc, xyz);
}

static int h3lis100dl_validate_pdata(struct h3lis100dl_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
					acc->pdata->min_interval);

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(acc->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int h3lis100dl_input_init(struct h3lis100dl_data *acc)
{
	int err;

	if (!h3lis100dl_workqueue)
		h3lis100dl_workqueue = create_workqueue(acc->name);

	if (!h3lis100dl_workqueue)
		return -EINVAL;

	INIT_WORK(&acc->input_work, h3lis100dl_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(acc->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->name = H3LIS100DL_DEV_NAME;
	acc->input_dev->id.bustype = acc->bustype;
	acc->input_dev->dev.parent = acc->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(INPUT_EVENT_TYPE, acc->input_dev->evbit);
	__set_bit(INPUT_EVENT_X, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Z, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_MSB, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, acc->input_dev->mscbit);

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

static void h3lis100dl_input_cleanup(struct h3lis100dl_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

#ifdef CONFIG_OF
static u32 h3lis100dl_parse_dt(struct h3lis100dl_data *cdata, struct device* dev)
{
	u32 val;
	struct device_node *np;

	np = dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "poll-interval", &val))
		cdata->pdata->poll_interval = val;
	else
		cdata->pdata->poll_interval = 10;

	cdata->pdata->init = NULL;
	cdata->pdata->exit = NULL;
	cdata->pdata->power_on = NULL;
	cdata->pdata->power_off = NULL;

	return 0;
}
#endif

int h3lis100dl_common_probe(struct h3lis100dl_data *acc)
{
	int err = -1;

	pr_info("%s: probe start.\n", H3LIS100DL_DEV_NAME);

#ifndef CONFIG_OF
	if (acc->dev->platform_data == NULL) {
		dev_err(acc->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;

	}
#else
	acc->dev->platform_data = NULL;
#endif

	mutex_init(&acc->lock);
	mutex_init(&acc->tb.buf_lock);
	mutex_lock(&acc->lock);

	acc->pdata = kzalloc(sizeof(struct h3lis100dl_platform_data), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(acc->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

#ifdef CONFIG_OF
	h3lis100dl_parse_dt(acc, acc->dev);
#else
	memcpy(acc->pdata, acc->dev->platform_data, sizeof(*acc->pdata));
#endif

	err = h3lis100dl_validate_pdata(acc);
	if (err < 0) {
		dev_err(acc->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	acc->pdata->g_range = H3LIS100DL_G_100G;

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(acc->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = H3LIS100DL_ENABLE_ALL_AXES;

	err = h3lis100dl_input_init(acc);
	if (err < 0) {
		dev_err(acc->dev, "input init failed\n");
		goto err_power_off;
	}

	err = h3lis100dl_device_power_on(acc);
	if (err < 0) {
		dev_err(acc->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = h3lis100dl_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(acc->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	err = h3lis100dl_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(acc->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(acc->dev);
	if (err < 0) {
		dev_err(acc->dev,
			"device H3LIS100DL_DEV_NAME "
			"sysfs register failed\n");
		goto err_input_cleanup;
	}

	h3lis100dl_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	hrtimer_init(&acc->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acc->hr_timer.function = &h3lis100dl_timer_func_queue_work;

	mutex_unlock(&acc->lock);

	dev_info(acc->dev, "%s: probed\n", H3LIS100DL_DEV_NAME);

	return 0;

err_input_cleanup:
	h3lis100dl_input_cleanup(acc);
err_power_off:
	h3lis100dl_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
#ifndef CONFIG_OF
exit_check_functionality_failed:
#endif
	printk(KERN_ERR "%s: Driver Init failed\n", H3LIS100DL_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(h3lis100dl_common_probe);

int h3lis100dl_common_remove(struct h3lis100dl_data *acc)
{
	h3lis100dl_input_cleanup(acc);
	h3lis100dl_device_power_off(acc);
	remove_sysfs_interfaces(acc->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);

	if(h3lis100dl_workqueue) {
		flush_workqueue(h3lis100dl_workqueue);
		destroy_workqueue(h3lis100dl_workqueue);
		h3lis100dl_workqueue = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(h3lis100dl_common_remove);

#ifdef CONFIG_PM
int h3lis100dl_common_resume(struct h3lis100dl_data *acc)
{
	if (acc->on_before_suspend)
		return h3lis100dl_enable(acc);
	return 0;
}
EXPORT_SYMBOL(h3lis100dl_common_resume);

int h3lis100dl_common_suspend(struct h3lis100dl_data *acc)
{
	acc->on_before_suspend = atomic_read(&acc->enabled);
	return h3lis100dl_disable(acc);
}
EXPORT_SYMBOL(h3lis100dl_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("h3lis100dl accelerometer driver");
MODULE_AUTHOR("Armando Visconti, STMicroelectronics");
MODULE_LICENSE("GPL v2");
