// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Matteo Dameno <matteo.dameno@st.com>
 *
 * Licensed under the GPL-2.
 */

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

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
#include <linux/unaligned/access_ok.h>

#include "lps22df.h"

/*	REGISTERS */
#define LPS22DF_WHO_AM_I_ADDR		0x0f
#define LPS22DF_WHO_AM_I_VAL		0xb4

#define LPS22DF_CTRL_REG1_ADDR		0x10
#define LPS22DF_AVG_MASK		GENMASK(2, 0)
#define LPS22DF_ODR_MASK		GENMASK(6, 3)
#define LPS22DF_CTR_REG1_SAFE_MASK	(LPS22DF_AVG_MASK | LPS22DF_ODR_MASK)

#define LPS22DF_CTRL_REG2_ADDR		0x11
#define LPS22DF_BDU_MASK		BIT(3)

#define LPS22DF_CTRL3_ADDR		0x12
#define LPS22DF_IF_ADD_INC_MASK		BIT(0)
#define LPS22DF_PP_OD_MASK		BIT(1)
#define LPS22DF_INT_H_L_MASK		BIT(3)

#define LPS22DF_CTRL4_ADDR		0x13

#define LPS22DF_PRESS_OUT_XL_ADDR	0x28

#define LPS22DF_TEMP_OUT_L_ADDR		0x2b

/*	REGISTERS ALIASES	*/
#define P_OUTDATA_REG		LPS22DF_PRESS_OUT_XL_ADDR
#define T_OUTDATA_REG		LPS22DF_TEMP_OUT_L_ADDR
#define OUTDATA_REG		LPS22DF_PRESS_OUT_XL_ADDR

/* Bitmasks */

/* Barometer and Termometer output data rate ODR */
#define ODR_ONESH	(0x00 << 3) /* pwr-dwn/one shot, both press, temp*/
#define ODR_1_1		(0x01 << 3) /*   1  Hz baro,   1  Hz term ODR	*/
#define ODR_4_4		(0x02 << 3) /*   4  Hz baro,   4  Hz term ODR	*/
#define ODR_10_10	(0x03 << 3) /*  10  Hz baro,  10  Hz term ODR	*/
#define ODR_25_25	(0x04 << 3) /*  25  Hz baro,  25  Hz term ODR	*/
#define ODR_50_50	(0x05 << 3) /*  50  Hz baro,  50  Hz term ODR	*/
#define ODR_75_75	(0x06 << 3) /*  75  Hz baro,  75  Hz term ODR	*/
#define ODR_100_100	(0x07 << 3) /* 100  Hz baro, 100  Hz term ODR	*/
#define ODR_200_200	(0x08 << 3) /* 200  Hz baro, 200  Hz term ODR	*/
#define ODR_DEFAULT	ODR_1_1

#define AVG4	(0x00) /* avg4 */
#define AVG8	(0x01) /* avg8 */
#define AVG16	(0x02) /* avg16 */
#define AVG32	(0x03) /* avg32 */
#define AVG64	(0x04) /* avg64 */
#define AVG128	(0x05) /* avg128 */
#define AVG256	(0x06) /* avg256 */
#define AVG512	(0x07) /* avg512 */
#define RESOLUTION_DEFAULT	AVG4

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lps22df_prs_odr_table[] = {
	{ 5, ODR_200_200 },
	{ 10, ODR_100_100 },
	{ 13, ODR_75_75 },
	{ 20, ODR_50_50 },
	{ 40, ODR_25_25 },
	{ 100, ODR_10_10 },
	{ 250, ODR_4_4 },
	{ 1000, ODR_1_1 },
};

struct outputdata {
	s32 press;
	s16 temperature;
};

static const struct lps22df_prs_platform_data default_lps22df_pdata = {
	.poll_interval = 1000,
	.min_interval = LPS22DF_PRS_MIN_POLL_PERIOD_MS,
};

static int lps22df_prs_hw_init(struct lps22df_prs_data *prs)
{
	int err;
	u8 buf[RESUME_ENTRIES];

	pr_info("hw init start\n");

	buf[0] = (prs->resume_state[RES_CTRL_REG2]);
	buf[1] = (prs->resume_state[RES_CTRL_REG3]);
	buf[2] = (prs->resume_state[RES_CTRL_REG4]);

	err = prs->tf->write(prs, LPS22DF_CTRL_REG2_ADDR, 3, buf);
	if (err < 0)
		goto err_resume_state;

	pr_debug("hw_init: CTRL_REG2, CTRL_REG3 CTRL_REG4 pass\n");

	buf[0] = prs->resume_state[RES_CTRL_REG1];
	err = prs->tf->write(prs, LPS22DF_CTRL_REG1_ADDR, 1, buf);
	if (err < 0)
		goto err_resume_state;

	pr_debug("hw_init: CTRL_REG1 pass\n");


	prs->hw_initialized = 1;

	pr_info("hw init done\n");

	return 0;

err_resume_state:
	prs->hw_initialized = 0;
	pr_err("hw init error 0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static void lps22df_prs_device_power_off(struct lps22df_prs_data *prs)
{
	int err;
	u8 buf;

	// Power Down
	buf = 0x00;
	err = prs->tf->write(prs, LPS22DF_CTRL_REG1_ADDR, 1, &buf);
	if (err < 0)
		pr_err("soft power off failed: %d\n", err);

	if (prs->pdata->power_off)
		prs->pdata->power_off();

	prs->hw_initialized = 0;
}

int lps22df_prs_update_odr(struct lps22df_prs_data *prs,
						unsigned int poll_period_ms)
{
	int err = -1;
	int i;
	u8 buf[2];
	u8 init_val, updated_val;
	u8 curr_val, new_val;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (longest period) backward (shortest
	 * period), to support the poll_interval requested by the system.
	 * It must be the longest period shorter then the set poll period.
	 */
	for (i = ARRAY_SIZE(lps22df_prs_odr_table) - 1; i >= 0; i--) {
		if ((lps22df_prs_odr_table[i].cutoff_ms <= poll_period_ms) ||
		    (i == 0))
			break;
	}

	new_val = lps22df_prs_odr_table[i].mask;

	err = prs->tf->read(prs, LPS22DF_CTRL_REG1_ADDR, 1, buf);
	if (err < 0)
		return err;

	init_val = buf[0];
	prs->resume_state[RES_CTRL_REG1] = init_val;

	/* set new ODR*/
	curr_val = init_val;
	buf[0] = LPS22DF_CTRL_REG1_ADDR;
	updated_val = (LPS22DF_CTR_REG1_SAFE_MASK) &
				((LPS22DF_ODR_MASK & new_val) |
					((~LPS22DF_ODR_MASK) & curr_val));

	err = prs->tf->write(prs, LPS22DF_CTRL_REG1_ADDR, 1, &updated_val);
	if (err < 0)
		return err;

	prs->resume_state[RES_CTRL_REG1] = updated_val;

	prs->delta_ts = ktime_set(0,
				1000000 * lps22df_prs_odr_table[i].cutoff_ms);

	return err;
}

static int lps22df_prs_device_power_on(struct lps22df_prs_data *prs)
{
	int err;

	if (prs->pdata->power_on) {
		err = prs->pdata->power_on();
		if (err < 0) {
			pr_err("custom power_on failed: %d\n", err);
			return err;
		}
	}

	if (!prs->hw_initialized) {
		err = lps22df_prs_hw_init(prs);
		if (prs->hw_working == 1 && err < 0) {
			lps22df_prs_device_power_off(prs);

			return err;
		}
		err = lps22df_prs_update_odr(prs, prs->pdata->poll_interval);
		if (err < 0)
			return err;
	}

	return 0;
}

static int lps22df_prs_get_presstemp_data(struct lps22df_prs_data *prs,
					struct outputdata *out)
{
	int err;
	/** Data bytes from hardware PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H,
	 *  TEMP_OUT_L, TEMP_OUT_H
	 */
	u8 prs_data[5];

	err = prs->tf->read(prs, OUTDATA_REG, sizeof(prs_data), prs_data);
	if (err < 0)
		return err;

	out->temperature = (s16) get_unaligned_le16(prs_data+3);
	prs_data[3] = 0;
	out->press = (s32) get_unaligned_le32(prs_data);

	return err;
}

static void lps22df_prs_report_values(struct lps22df_prs_data *prs,
				    struct outputdata *out)
{
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    out->press);
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    out->temperature);
#ifdef LPS22DF_TIMESTAMP
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    prs->timestamp >> 32);
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    prs->timestamp & 0xffffffff);
#endif /* LPS22DF_TIMESTAMP */
	input_sync(prs->input_dev_pres);
}

static int lps22df_prs_enable(struct lps22df_prs_data *prs)
{
	int err;

	if (!atomic_cmpxchg(&prs->enabled, 0, 1)) {
		err = lps22df_prs_device_power_on(prs);
		if (err < 0) {
			atomic_set(&prs->enabled, 0);

			return err;
		}
		mutex_lock(&prs->lock);
		hrtimer_start(&prs->hr_timer, prs->delta_ts, HRTIMER_MODE_REL);
		mutex_unlock(&prs->lock);
	}

	return 0;
}

static int lps22df_prs_disable(struct lps22df_prs_data *prs)
{
	if (atomic_cmpxchg(&prs->enabled, 1, 0)) {
		mutex_lock(&prs->lock);
		hrtimer_cancel(&prs->hr_timer);
		mutex_unlock(&prs->lock);
		lps22df_prs_device_power_off(prs);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lps22df_prs_data *prs = dev_get_drvdata(dev);

	mutex_lock(&prs->lock);
	val = prs->pdata->poll_interval;
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct lps22df_prs_data *prs = dev_get_drvdata(dev);
	unsigned int interval_ms;

	if (kstrtouint(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		return -EINVAL;

	interval_ms = max(interval_ms, prs->pdata->min_interval);

	mutex_lock(&prs->lock);
	prs->pdata->poll_interval = interval_ms;
	err = lps22df_prs_update_odr(prs, interval_ms);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lps22df_prs_data *prs = dev_get_drvdata(dev);
	int val = atomic_read(&prs->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lps22df_prs_data *prs = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	if (val)
		lps22df_prs_enable(prs);
	else
		lps22df_prs_disable(prs);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(poll_period_ms, 0664, attr_get_polling_rate,
						attr_set_polling_rate),
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

	pr_err("Unable to create interface\n");

	return ret;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static inline int64_t lps22df_prs_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static void lps22df_prs_input_work_func(struct work_struct *work)
{
	struct lps22df_prs_data *prs = container_of((struct work_struct *)work,
						  struct lps22df_prs_data,
						  input_work);
	ktime_t tmpkt;
	struct outputdata output;
	int err;

	err = lps22df_prs_get_presstemp_data(prs, &output);
	if (err < 0)
		pr_err("get_pressure_data failed. err: %d\n", err);
	else
		lps22df_prs_report_values(prs, &output);

	mutex_lock(&prs->lock);
	tmpkt = ktime_sub(prs->delta_ts,
			  ktime_set(0,
				(lps22df_prs_get_time_ns() - prs->timestamp)));

	if (tmpkt < 0LL)
		tmpkt = ktime_set(0, prs->delta_ts);

	if (atomic_read(&prs->enabled)) {
		hrtimer_start(&prs->hr_timer, tmpkt, HRTIMER_MODE_REL);
	}

	mutex_unlock(&prs->lock);
}

static enum hrtimer_restart lps22df_prs_poll_function_read(
							struct hrtimer *timer)
{
	struct lps22df_prs_data *prs;

	prs = container_of((struct hrtimer *)timer, struct lps22df_prs_data,
			   hr_timer);

	prs->timestamp = lps22df_prs_get_time_ns();
	queue_work(prs->workqueue, &prs->input_work);

	return HRTIMER_NORESTART;
}

#ifdef LPS22DF_EN_ON_OPEN
int lps22df_prs_input_open(struct input_dev *input)
{
	struct lps22df_prs_data *prs = input_get_drvdata(input);

	return lps22df_prs_enable(prs);
}

void lps22df_prs_input_close(struct input_dev *dev)
{
	lps22df_prs_disable(input_get_drvdata(dev));
}
#endif /* LPS22DF_EN_ON_OPEN */

static int lps22df_prs_validate_pdata(struct lps22df_prs_data *prs)
{
	/* checks for correctness of minimal polling period */
	prs->pdata->min_interval = (unsigned int)LPS22DF_PRS_MIN_POLL_PERIOD_MS;
	prs->pdata->poll_interval = max(prs->pdata->poll_interval,
					prs->pdata->min_interval);

	/* Checks polling interval relative to minimum polling interval */
	if (prs->pdata->poll_interval < prs->pdata->min_interval) {
		pr_err("minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lps22df_prs_prepare_hrtimer(struct lps22df_prs_data *prs)
{

	prs->workqueue = create_workqueue(prs->name);
	if (!prs->workqueue) {
		pr_err("workqueue creation failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&prs->input_work, lps22df_prs_input_work_func);
	mutex_lock(&prs->lock);
	hrtimer_init(&prs->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	prs->hr_timer.function = &lps22df_prs_poll_function_read;
	mutex_unlock(&prs->lock);
	return 0;
}

static int lps22df_prs_input_init(struct lps22df_prs_data *prs)
{
	int err;

	prs->input_dev_pres = input_allocate_device();
	if (!prs->input_dev_pres) {
		pr_err("input device allocate failed\n");

		return -ENOMEM;
	}

#ifdef LPS22DF_EN_ON_OPEN
	prs->input_dev_pres->open = lps22df_prs_input_open;
	prs->input_dev_pres->close = lps22df_prs_input_close;
#endif /* LPS22DF_EN_ON_OPEN */
	prs->input_dev_pres->name = prs->name;
	prs->input_dev_pres->id.bustype = prs->bustype;
	prs->input_dev_pres->dev.parent = prs->dev;
	input_set_drvdata(prs->input_dev_pres, prs);

	__set_bit(INPUT_EVENT_TYPE, prs->input_dev_pres->evbit);
	__set_bit(INPUT_EVENT_X, prs->input_dev_pres->mscbit);
	__set_bit(INPUT_EVENT_Y, prs->input_dev_pres->mscbit);
#ifdef LPS22DF_TIMESTAMP
	__set_bit(INPUT_EVENT_TIME_MSB, prs->input_dev_pres->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, prs->input_dev_pres->mscbit);
#endif /* LPS22DF_TIMESTAMP */
	err = input_register_device(prs->input_dev_pres);
	if (err) {
		pr_err("unable to register input polled device %s\n",
			prs->input_dev_pres->name);
		input_free_device(prs->input_dev_pres);

		return err;
	}

	return 0;
}

static void lps22df_prs_cancel_destroy_hrtimer_wk(struct lps22df_prs_data *prs)
{
	cancel_work_sync(&prs->input_work);
	if (prs->workqueue) {
		flush_workqueue(prs->workqueue);
		destroy_workqueue(prs->workqueue);
		prs->workqueue = NULL;
	}
}

static void lps22df_prs_input_cleanup(struct lps22df_prs_data *prs)
{
	input_unregister_device(prs->input_dev_pres);
}

int lps22df_common_probe(struct lps22df_prs_data *prs)
{
	int err = -1;
	u8 buf[5];

	pr_info("probe start.\n");

	mutex_init(&prs->lock);
	mutex_init(&prs->tb.buf_lock);

	prs->pdata = kzalloc(sizeof(*prs->pdata), GFP_KERNEL);
	if (prs->pdata == NULL) {
		err = -ENOMEM;
		pr_err("failed to allocate memory for pdata: %d\n", err);

		goto err_mutexunlockfreedata;
	}

	if (prs->dev->platform_data == NULL) {
		memcpy(prs->pdata, &default_lps22df_pdata,
			sizeof(struct lps22df_prs_platform_data));
		pr_info("using default plaform_data for lps22df\n");
	} else {
		memcpy(prs->pdata, prs->dev->platform_data,
			sizeof(struct lps22df_prs_platform_data));
		pr_info("using user plaform_data for lps22df\n");
	}

	/* read chip id */
	err = prs->tf->read(prs, LPS22DF_WHO_AM_I_ADDR, 1, buf);
	if (err < 0) {
		pr_warn("Error reading WHO_AM_I register\n");

		goto err_mutexunlockfreedata;
	} else {
		prs->hw_working = 1;
	}

	if (buf[0] != LPS22DF_WHO_AM_I_VAL) {
		pr_err("dev unknown. Expected: 0x%02x,", LPS22DF_WHO_AM_I_VAL);
		pr_cont(" Replies: 0x%02x\n", buf[0]);
		err = -ENODEV;

		goto err_mutexunlockfreedata;
	}

	pr_info("ID Chip OK\n");

	err = lps22df_prs_validate_pdata(prs);
	if (err < 0) {
		pr_err("failed to validate platform data\n");

		goto err_exit_kfree_pdata;
	}

	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			pr_err("custom init failed: %d\n", err);

			goto err_exit_pointer;
		}
	}

	memset(prs->resume_state, 0, ARRAY_SIZE(prs->resume_state));

	/* init registers which need values different from zero */
	prs->resume_state[RES_CTRL_REG1] = (LPS22DF_CTR_REG1_SAFE_MASK) &
				((LPS22DF_ODR_MASK & ODR_DEFAULT) |
				(LPS22DF_AVG_MASK & RESOLUTION_DEFAULT));
	prs->resume_state[RES_CTRL_REG2] = (LPS22DF_BDU_MASK);
	prs->resume_state[RES_CTRL_REG3] = (LPS22DF_IF_ADD_INC_MASK);
	prs->resume_state[RES_CTRL_REG4] = (0x00);

	err = lps22df_prs_device_power_on(prs);
	if (err < 0) {
		pr_err("power on failed: %d\n", err);

		goto err_exit_pointer;
	}
	atomic_set(&prs->enabled, 1);

	err = lps22df_prs_update_odr(prs, prs->pdata->poll_interval);
	if (err < 0) {
		pr_err("update_odr failed\n");

		goto err_power_off;
	}

	err = lps22df_prs_prepare_hrtimer(prs);
	if (err < 0) {
		pr_err("hrtimer init failed\n");

		goto err_power_off;
	}

	err = lps22df_prs_input_init(prs);
	if (err < 0) {
		pr_err("input init failed\n");

		goto err_destroy_wk;
	}

	err = create_sysfs_interfaces(prs->dev);
	if (err < 0) {
		pr_err("device sysfs register failed\n");

		goto err_input_cleanup;
	}

	lps22df_prs_device_power_off(prs);

	/* As default, do not report information */
	atomic_set(&prs->enabled, 0);

	pr_info("probed\n");

	return 0;



err_input_cleanup:
	lps22df_prs_input_cleanup(prs);
err_destroy_wk:
	lps22df_prs_cancel_destroy_hrtimer_wk(prs);
err_power_off:
	lps22df_prs_device_power_off(prs);

err_exit_pointer:
	if (prs->pdata->exit)
		prs->pdata->exit();

err_exit_kfree_pdata:
	kfree(prs->pdata);

err_mutexunlockfreedata:
	pr_err("Driver Init failed\n");

	return err;
}
EXPORT_SYMBOL(lps22df_common_probe);

int lps22df_common_remove(struct lps22df_prs_data *prs)
{

	lps22df_prs_disable(prs);

	lps22df_prs_cancel_destroy_hrtimer_wk(prs);

	lps22df_prs_input_cleanup(prs);

	remove_sysfs_interfaces(prs->dev);

	if (prs->pdata->exit)
		prs->pdata->exit();
	kfree(prs->pdata);

	return 0;
}
EXPORT_SYMBOL(lps22df_common_remove);

#ifdef CONFIG_PM_SLEEP
int lps22df_common_resume(struct lps22df_prs_data *prs)
{
	if (prs->on_before_suspend)
		return lps22df_prs_enable(prs);

	return 0;
}
EXPORT_SYMBOL(lps22df_common_resume);

int lps22df_common_suspend(struct lps22df_prs_data *prs)
{
	prs->on_before_suspend = atomic_read(&prs->enabled);

	return lps22df_prs_disable(prs);
}
EXPORT_SYMBOL(lps22df_common_suspend);
#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("STMicroelectronics lps22df pressure sensor driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_LICENSE("GPL v2");
