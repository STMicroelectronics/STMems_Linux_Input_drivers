/*
 * STMicroelectronics lps22hh driver
 *
 * Copyright 2020 STMicroelectronics Inc.
 *
 * Authors: AMG MSD DIVISION
 *        : Mario Tesi (mario.tesi@st.com)
 *
 * Version: 1.0.0
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
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/pm.h>

#include "lps22hh.h"

/* 24 bit 2'compl */
#define	PR_ABS_MAX		8388607
#define	PR_ABS_MIN		-8388608

#define	WHOAMI_LPS22HH_PRS	0xB3

#define	WHO_AM_I		0x0F
#define	CTRL_REG1		0x10
#define	CTRL_REG2		0x11
#define	CTRL_REG3		0x12

#define	PRESS_OUT_XL		0x28

#define	ODR_MASK		0x70
#define	BDU_MASK		0x02
#define SW_RESET_MASK		0x04
#define IF_ADD_INC_MASK		0x10

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lps22hh_odr_table[] = {
	{    5, 0x70 },
	{   10, 0x60 },
	{   13, 0x50 },
	{   20, 0x40 },
	{   40, 0x30 },
	{  100, 0x20 },
	{ 1000, 0x10 },
};

static const struct lps22hh_platform_data default_lps22hh_pdata = {
	.poll_interval = 1000,
	.min_interval = LPS22HH_MIN_POLL_PERIOD_MS,
};

static int lps22hh_hw_init(struct lps22hh_data *prs)
{
	int err;
	u8 buf[2];

 	buf[0] = prs->resume_state[RES_CTRL_REG2] | IF_ADD_INC_MASK;
	buf[1] = prs->resume_state[RES_CTRL_REG3];

	err = prs->tf->write(prs, CTRL_REG2, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = prs->resume_state[RES_CTRL_REG1];
	err = prs->tf->write(prs, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	prs->hw_initialized = 1;

	return 0;

err_resume_state:
	prs->hw_initialized = 0;
	dev_err(prs->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
		buf[1], err);
	return err;
}

static void lps22hh_device_power_off(struct lps22hh_data *prs)
{
	int err;
	u8 buf;

	buf = 0x00;
	err = prs->tf->write(prs, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(prs->dev, "soft power off failed: %d\n", err);

	buf = SW_RESET_MASK;
	err = prs->tf->write(prs, CTRL_REG2, 1, &buf);
	if (err < 0)
		dev_err(prs->dev, "soft power off failed: %d\n", err);

	udelay(2);

	if (prs->pdata->power_off)
		prs->pdata->power_off();

	prs->hw_initialized = 0;
}

int lps22hh_update_odr(struct lps22hh_data *prs, int poll_period_ms)
{
	u8 init_val, updated_val, new_val;
	int err = -1;
	int i;

	for (i = ARRAY_SIZE(lps22hh_odr_table) - 1; i >= 0; i--) {
		if ((lps22hh_odr_table[i].cutoff_ms <= poll_period_ms) ||
		    (i == 0))
			break;
	}

	new_val = lps22hh_odr_table[i].mask;
	init_val = prs->resume_state[RES_CTRL_REG1];
	updated_val = ((ODR_MASK & new_val) | ((~ODR_MASK) & init_val) | BDU_MASK);
	err = prs->tf->write(prs, CTRL_REG1, 1, &updated_val);
	if (err < 0)
		goto error;

	prs->resume_state[RES_CTRL_REG1] = updated_val;

	prs->delta_ts = ktime_set(0, 1000000 * lps22hh_odr_table[i].cutoff_ms);

	return err;

error:
	dev_err(prs->dev, "update odr failed: %d\n", err);

	return err;
}

static int lps22hh_device_power_on(struct lps22hh_data *prs)
{
	int err;

	if (prs->pdata->power_on) {
		err = prs->pdata->power_on();
		if (err < 0) {
			dev_err(prs->dev, "power_on failed: %d\n", err);

			return err;
		}
	}

	if (!prs->hw_initialized) {
		err = lps22hh_hw_init(prs);
		lps22hh_update_odr(prs, prs->pdata->poll_interval);
		if (prs->hw_working == 1 && err < 0) {
			lps22hh_device_power_off(prs);

			return err;
		}
	}

	return 0;
}

static int lps22hh_get_presstemp_data(struct lps22hh_data *prs)
{
	int err;
	u8 prs_data[5];

	err = prs->tf->read(prs, PRESS_OUT_XL, sizeof(prs_data), prs_data);
	if (err < 0)
		return err;

	prs->press = (s32)((((s8)prs_data[2]) << 16) | (prs_data[1] << 8) |
			 prs_data[0]);
	prs->temperature = (s16)((((s8)prs_data[4]) << 8) | prs_data[3]);

	return err;
}

static void lps22hh_report_values(struct lps22hh_data *prs)
{
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE,
		    INPUT_EVENT_X, prs->press);
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE,
		    INPUT_EVENT_Y, prs->temperature);
#ifdef LPS22HH_TIMESTAMP
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    prs->timestamp >> 32);
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    prs->timestamp & 0xffffffff);
#endif /* LPS22HH_TIMESTAMP */
	input_sync(prs->input_dev_pres);
}

static int lps22hh_enable(struct lps22hh_data *prs)
{
	int err;

	if (!atomic_cmpxchg(&prs->enabled, 0, 1)) {
		err = lps22hh_device_power_on(prs);
		if (err < 0) {
			atomic_set(&prs->enabled, 0);

			return err;
		}

		hrtimer_start(&prs->hr_timer, prs->delta_ts, HRTIMER_MODE_REL);
	}

	return 0;
}

static int lps22hh_disable(struct lps22hh_data *prs)
{
	if (atomic_cmpxchg(&prs->enabled, 1, 0)) {
		hrtimer_cancel(&prs->hr_timer);
		lps22hh_device_power_off(prs);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lps22hh_data *prs = dev_get_drvdata(dev);

	mutex_lock(&prs->lock);
	val = prs->pdata->poll_interval;
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lps22hh_data *prs = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, prs->pdata->min_interval);

	mutex_lock(&prs->lock);
	prs->pdata->poll_interval = interval_ms;
	lps22hh_update_odr(prs, interval_ms);
	mutex_unlock(&prs->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lps22hh_data *prs = dev_get_drvdata(dev);
	int val = atomic_read(&prs->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lps22hh_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lps22hh_enable(prs);
	else
		lps22hh_disable(prs);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(poll_period_ms, 0664, attr_get_polling_rate,attr_set_polling_rate),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(dev, attributes + i);
		if (ret < 0)
			goto error;
	}
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);

	dev_err(dev, "%s:Unable to create interface\n", __func__);

	return ret;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static inline int64_t lps22hh_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static void lps22hh_input_work_func(struct work_struct *work)
{
	struct lps22hh_data *prs = container_of((struct work_struct *)work,
						  struct lps22hh_data,
						  input_work);
	ktime_t tmpkt;
	int err;

	tmpkt = ktime_sub(prs->delta_ts,
			  ktime_set(0,
				   (lps22hh_get_time_ns() - prs->timestamp)));

	if (tmpkt < 0LL)
		tmpkt = ktime_set(0,prs->delta_ts);

	hrtimer_start(&prs->hr_timer, tmpkt, HRTIMER_MODE_REL);

	mutex_lock(&prs->lock);
	err = lps22hh_get_presstemp_data(prs);
	if (err < 0)
		dev_err(prs->dev, "get_pressure_data failed\n");
	else
		lps22hh_report_values(prs);

	mutex_unlock(&prs->lock);
}

static enum hrtimer_restart lps22hh_poll_function_read(struct hrtimer *timer)
{
	struct lps22hh_data *prs;

	prs = container_of((struct hrtimer *)timer, struct lps22hh_data,
			   hr_timer);

	prs->timestamp = lps22hh_get_time_ns();
	queue_work(prs->workqueue, &prs->input_work);

	return HRTIMER_NORESTART;
}

#ifdef LPS22HH_EN_ON_OPEN
int lps22hh_input_open(struct input_dev *input)
{
	struct lps22hh_data *prs = input_get_drvdata(input);

	return lps22hh_enable(prs);
}

void lps22hh_input_close(struct input_dev *dev)
{
	lps22hh_disable(input_get_drvdata(dev));
}
#endif

static int lps22hh_validate_pdata(struct lps22hh_data *prs)
{
	prs->pdata->min_interval = (unsigned int)LPS22HH_MIN_POLL_PERIOD_MS;
	prs->pdata->poll_interval = max(prs->pdata->poll_interval,
					prs->pdata->min_interval);

	if (prs->pdata->poll_interval < prs->pdata->min_interval) {
		dev_err(prs->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lps22hh_input_init(struct lps22hh_data *prs)
{
	int err;

	prs->workqueue = create_workqueue(prs->name);
	if (!prs->workqueue)
		return -ENOMEM;

	INIT_WORK(&prs->input_work, lps22hh_input_work_func);
	hrtimer_init(&prs->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	prs->hr_timer.function = &lps22hh_poll_function_read;

	prs->input_dev_pres = input_allocate_device();
	if (!prs->input_dev_pres) {
		dev_err(prs->dev, "input device allocate failed\n");

		return -ENOMEM;
	}

#ifdef LPS22HH_EN_ON_OPEN
	prs->input_dev_pres->open = lps22hh_input_open;
	prs->input_dev_pres->close = lps22hh_input_close;
#endif
	prs->input_dev_pres->name = prs->name;
	prs->input_dev_pres->id.bustype = prs->bustype;
	prs->input_dev_pres->dev.parent = prs->dev;
	input_set_drvdata(prs->input_dev_pres, prs);

	__set_bit(INPUT_EVENT_TYPE, prs->input_dev_pres->evbit);
	__set_bit(INPUT_EVENT_X, prs->input_dev_pres->mscbit);
	__set_bit(INPUT_EVENT_Y, prs->input_dev_pres->mscbit);
#ifdef LPS22HH_TIMESTAMP
	__set_bit(INPUT_EVENT_TIME_MSB, prs->input_dev_pres->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, prs->input_dev_pres->mscbit);
#endif /* LPS22HH_TIMESTAMP */
	err = input_register_device(prs->input_dev_pres);
	if (err) {
		dev_err(prs->dev,
			"unable to register input polled device %s\n",
			prs->input_dev_pres->name);
		input_free_device(prs->input_dev_pres);

		return err;
	}

	return 0;
}

static void lps22hh_input_cleanup(struct lps22hh_data *prs)
{
	input_unregister_device(prs->input_dev_pres);
	input_free_device(prs->input_dev_pres);
}

int lps22hh_common_probe(struct lps22hh_data *prs)
{
	int err;
	u8 wai;

	mutex_init(&prs->lock);
	mutex_init(&prs->tb.buf_lock);
	mutex_lock(&prs->lock);

	prs->pdata = kzalloc(sizeof(*prs->pdata), GFP_KERNEL);
	if(prs->pdata == NULL) {
		err = -ENOMEM;
		dev_err(prs->dev,
			"failed to allocate memory for pdata: %d\n", err);

		goto err_mutexunlockfreedata;
	}

	if (prs->dev->platform_data == NULL) {
		memcpy(prs->pdata, &default_lps22hh_pdata,
		       sizeof(struct lps22hh_platform_data));
		dev_info(prs->dev, "using default plaform_data for lps22\n");
	} else {
		memcpy(prs->pdata, prs->dev->platform_data,
		       sizeof(struct lps22hh_platform_data));
		dev_info(prs->dev, "using user plaform_data for lps22\n");
        }

	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			dev_err(prs->dev, "lps pdata init failed: %d\n", err);

			goto err_pdata_init;
		}
	}

	err = prs->tf->read(prs, WHO_AM_I, 1, &wai);
	if (err < 0) {
		dev_warn(prs->dev, "Error reading WHO_AM_I register\n");

		goto err_mutexunlockfreedata;
	} else {
		prs->hw_working = 1;
	}

	if (wai != WHOAMI_LPS22HH_PRS) {
		dev_err(prs->dev, "device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n", WHOAMI_LPS22HH_PRS, wai);
		err = -1;

		goto err_mutexunlockfreedata;
	}

	err = lps22hh_validate_pdata(prs);
	if (err < 0) {
		dev_err(prs->dev, "failed to validate platform data\n");

		goto err_exit_kfree_pdata;
	}

	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			dev_err(prs->dev, "init failed: %d\n", err);

			goto err_exit_pointer;
		}
	}

	memset(prs->resume_state, 0, ARRAY_SIZE(prs->resume_state));
	prs->resume_state[RES_CTRL_REG1] = (ODR_MASK & 0x10) | (BDU_MASK);

	err = lps22hh_device_power_on(prs);
	if (err < 0) {
		dev_err(prs->dev, "power on failed: %d\n", err);

		goto err_exit_pointer;
	}

	atomic_set(&prs->enabled, 1);

	err = lps22hh_update_odr(prs, prs->pdata->poll_interval);
	if (err < 0) {
		dev_err(prs->dev, "update_odr failed\n");

		goto err_power_off;
	}

	err = lps22hh_input_init(prs);
	if (err < 0) {
		dev_err(prs->dev, "input init failed\n");

		goto err_power_off;
	}

	err = create_sysfs_interfaces(prs->dev);
	if (err < 0) {
		dev_err(prs->dev,
			"device sysfs register failed\n");

		goto err_input_cleanup;
	}

	lps22hh_device_power_off(prs);

	atomic_set(&prs->enabled, 0);
	mutex_unlock(&prs->lock);

	prs->resume_state[RES_CTRL_REG2] = IF_ADD_INC_MASK;
	err = prs->tf->write(prs, CTRL_REG2, 1,
			     &prs->resume_state[RES_CTRL_REG2]);
	if (err < 0)
		return err;

	dev_info(prs->dev, "%s: probed\n", LPS22HH_DEV_NAME);

	return 0;

err_input_cleanup:
	lps22hh_input_cleanup(prs);

err_power_off:
	lps22hh_device_power_off(prs);

err_exit_pointer:
	if (prs->pdata->exit)
		prs->pdata->exit();

err_pdata_init:
	if (prs->pdata->exit)
		prs->pdata->exit();

err_exit_kfree_pdata:
	kfree(prs->pdata);

err_mutexunlockfreedata:
	mutex_unlock(&prs->lock);
	pr_err("%s: Driver Init failed\n", LPS22HH_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lps22hh_common_probe);

int lps22hh_common_remove(struct lps22hh_data *prs)
{
	lps22hh_disable(prs);
	lps22hh_input_cleanup(prs);
	lps22hh_device_power_off(prs);
	remove_sysfs_interfaces(prs->dev);

	if (prs->workqueue) {
		flush_workqueue(prs->workqueue);
		destroy_workqueue(prs->workqueue);
		prs->workqueue = NULL;
	}

	if (prs->pdata->exit)
		prs->pdata->exit();
	kfree(prs->pdata);

	return 0;
}
EXPORT_SYMBOL(lps22hh_common_remove);

#ifdef CONFIG_PM
int lps22hh_common_resume(struct lps22hh_data *prs)
{
	if (prs->on_before_suspend)
		return lps22hh_enable(prs);

	return 0;
}
EXPORT_SYMBOL(lps22hh_common_resume);

int lps22hh_common_suspend(struct lps22hh_data *prs)
{
	prs->on_before_suspend = atomic_read(&prs->enabled);

	return lps22hh_disable(prs);
}
EXPORT_SYMBOL(lps22hh_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("STMicrolelectronics lps22hh pressure sensor driver");
MODULE_AUTHOR("AMG MSD DIVISION, STMicroelectronics");
MODULE_VERSION(LPS22HH_MODULE_VERSION);
MODULE_LICENSE("GPL v2");
