/*
 * STMicroelectronics lis3dhh core driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Version 1.0.0
 *
 * Licensed under the GPL-2.
 */

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

#include	"lis3dhh.h"

/* Acc. full scale fixed */
#define LIS3DHH_ACC_FULL_SCALE  "2.5"

/* Acc. sensitivity in ug/LSB */
#define LIS3DHH_SENSITIVITY	76

/* Acc. number of samples to discard for stable data */
#define LIS3DHH_DISC_SAMPLE 10

/* Acc. operating mode */
#define LIS3DHH_ENABLE	0x80
#define LIS3DHH_DISABLE	0x00

/* Acc. ID */
#define LIS3DHH_WHOAMI	0x11

/* Device registers */
#define WHO_AM_I		0x0F
#define CTRL1			0x20
#define STATUS			0x27
#define AXISDATA_REG	0x28

/* Register bitmasks */
#define CTRL1_EN_MODE   	0x80
#define CTRL1_EN_MASK   	0x80

#define CTRL1_BDU_ENABLE	0x08
#define CTRL1_BDU_MASK		0x08

#define CTRL1_IF_ADD_INC	0x40
#define CTRL1_IF_ADD_INC_MASK	0x40

static inline int64_t lis3dhh_acc_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static int lis3dhh_acc_hw_init(struct lis3dhh_acc_status *stat)
{
	int err;

	stat->resume_state |= CTRL1_EN_MODE;

	err = stat->tf->write(stat, CTRL1, 1, &stat->resume_state);
	if (err < 0)
		return err;

	stat->hw_initialized = 1;

	return 0;
}

static void lis3dhh_acc_device_power_off(struct lis3dhh_acc_status *stat)
{
	int err;
	u8 buf = (CTRL1_BDU_ENABLE | CTRL1_IF_ADD_INC);

	err = stat->tf->write(stat, CTRL1, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized) {
		stat->hw_initialized = 0;
	}
}

static int lis3dhh_acc_device_power_on(struct lis3dhh_acc_status *stat)
{
	int err;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(stat->dev,
				"power_on failed: %d\n", err);
			return err;
		}
	}

	if (!stat->hw_initialized) {
		err = lis3dhh_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lis3dhh_acc_device_power_off(stat);
			return err;
		}
	}

	return 0;
}

static void lis3dhh_acc_report_values(struct lis3dhh_acc_status *stat,
				       int *xyz, int64_t timestamp)
{
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(stat->input_dev);
}

static void lis3dhh_acc_report_triple(struct lis3dhh_acc_status *stat)
{
	int err;
	u8 acc_data[6];
	int xyz[3] = { 0 };

	err = stat->tf->read(stat, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return;

	xyz[0] = (s32)((s16)(acc_data[0] | (acc_data[1] << 8)));
	xyz[1] = (s32)((s16)(acc_data[2] | (acc_data[3] << 8)));
	xyz[2] = (s32)((s16)(acc_data[4] | (acc_data[5] << 8)));
	xyz[0] *= LIS3DHH_SENSITIVITY;
	xyz[1] *= LIS3DHH_SENSITIVITY;
	xyz[2] *= LIS3DHH_SENSITIVITY;

	if (stat->sample_to_discard)
		stat->sample_to_discard--;
	else
		lis3dhh_acc_report_values(stat, xyz, stat->timestamp);
}

static int lis3dhh_acc_enable(struct lis3dhh_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lis3dhh_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}

		/* Dischard first # samples. */
		stat->sample_to_discard = LIS3DHH_DISC_SAMPLE;
		stat->polling_ktime = ktime_set(0,
                                        (LIS3DHH_MIN_POLL_PERIOD_NS % 1000000));
		hrtimer_start(&stat->hr_timer_poll,
			      stat->polling_ktime, HRTIMER_MODE_REL);
	}

	return 0;
}

static int lis3dhh_acc_disable(struct lis3dhh_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_work_sync(&stat->input_poll_work);
		lis3dhh_acc_device_power_off(stat);
	}

	return 0;
}

/*
 * Device doesn't support change in ODR HW so we left it run at 1.1 kHz
 * and change only the timer period
 */
static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	return size;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return sprintf(buf, "%d.%d\n", LIS3DHH_MIN_POLL_PERIOD_NS / 1000000,
            (LIS3DHH_MIN_POLL_PERIOD_NS % 1000000) / 1000);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis3dhh_acc_status *stat = dev_get_drvdata(dev);

	dev_info(stat->dev, "range set fixed to %s\n", LIS3DHH_ACC_FULL_SCALE);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", LIS3DHH_ACC_FULL_SCALE);
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dhh_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3dhh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis3dhh_acc_enable(stat);
	else
		lis3dhh_acc_disable(stat);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
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

static void lis3dhh_acc_input_poll_work_func(struct work_struct *work)
{
	struct lis3dhh_acc_status *stat;
	ktime_t tmpkt, ktdelta;

	stat = container_of((struct work_struct *) work,
			    struct lis3dhh_acc_status, input_poll_work);

	if (atomic_read(&stat->enabled)) {
		/* Adjust new timeout. */
		ktdelta = ktime_set(0, lis3dhh_acc_get_time_ns() - stat->timestamp);
		/* Avoid negative value in case of High ODR. */
		if (stat->polling_ktime > ktdelta)
			tmpkt = ktime_sub(stat->polling_ktime, ktdelta);
		else
			tmpkt = stat->polling_ktime;

		hrtimer_start(&stat->hr_timer_poll, tmpkt, HRTIMER_MODE_REL);
	}

	lis3dhh_acc_report_triple(stat);
}

enum hrtimer_restart lis3dhh_acc_hr_timer_poll_function(struct hrtimer *timer)
{
	struct lis3dhh_acc_status *stat;

	stat = container_of((struct hrtimer *)timer,
			    struct lis3dhh_acc_status, hr_timer_poll);

	stat->timestamp = lis3dhh_acc_get_time_ns();

	queue_work(stat->hr_timer_poll_work_queue, &stat->input_poll_work);

	return HRTIMER_NORESTART;
}

int lis3dhh_acc_input_open(struct input_dev *input)
{
	struct lis3dhh_acc_status *stat = input_get_drvdata(input);

	return lis3dhh_acc_enable(stat);
}

void lis3dhh_acc_input_close(struct input_dev *dev)
{
	struct lis3dhh_acc_status *stat = input_get_drvdata(dev);

	lis3dhh_acc_disable(stat);
}

static int lis3dhh_acc_input_init(struct lis3dhh_acc_status *stat)
{
	int err;

	INIT_WORK(&stat->input_poll_work, lis3dhh_acc_input_poll_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(stat->dev, "input device allocation failed\n");
		return err;
	}

	stat->input_dev->open = lis3dhh_acc_input_open;
	stat->input_dev->close = lis3dhh_acc_input_close;
	stat->input_dev->name = stat->name;
	stat->input_dev->id.bustype = stat->bustype;
	stat->input_dev->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev, stat);

	/* Set Misc event type */
	__set_bit(INPUT_EVENT_TYPE, stat->input_dev->evbit);
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
        return err;
	}

	return 0;
}

static void lis3dhh_acc_input_cleanup(struct lis3dhh_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int lis3dhh_discover(struct lis3dhh_acc_status *stat)
{
	int err;
	u8 wai = 0x00;

	err = stat->tf->read(stat, WHO_AM_I, 1, &wai);
	if (err < 0) {
		dev_warn(stat->dev, "failed to read Who-Am-I register");
		return err;
	}

	if (wai != LIS3DHH_WHOAMI) {
		dev_err(stat->dev, "device unknown. Expected: 0x%02x,"
				" Replies: 0x%02x\n", LIS3DHH_WHOAMI, wai);
		return -ENODEV;
	}

	stat->hw_working = 1;

	return 0;
}

int lis3dhh_common_probe(struct lis3dhh_acc_status *stat)
{
	int err;

	mutex_init(&stat->tb.buf_lock);

	err = lis3dhh_discover(stat);
	if (err < 0)
		return err;

	stat->pdata = kzalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		dev_err(stat->dev, "failed to allocate memory for pdata: %d\n", err);
		return -ENOMEM;
	}

	/* Attach if exists custom platform data/functions */
	if (stat->dev->platform_data != NULL)
		memcpy(stat->pdata, stat->dev->platform_data, sizeof(*stat->pdata));

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(stat->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	/* Set registers initial value */
	stat->resume_state = (CTRL1_BDU_ENABLE | CTRL1_IF_ADD_INC);

	err = lis3dhh_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	/* Init polling workqueue */
	stat->hr_timer_poll_work_queue =
			create_workqueue("lis3dhh_acc_hr_timer_poll_wq");
	if (!stat->hr_timer_poll_work_queue) {
		dev_err(stat->dev, "create workqueue failed\n");
		err = -ENOMEM;
		goto err_wk_null;
	}

	hrtimer_init(&stat->hr_timer_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_poll.function = &lis3dhh_acc_hr_timer_poll_function;

	err = lis3dhh_acc_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "input init failed\n");
		goto err_remove_hr_work_queue;
	}

	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev, "device %s sysfs register failed\n",
			LIS3DHH_ACC_DEV_NAME);
		goto err_input_cleanup;
	}

	lis3dhh_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	dev_info(stat->dev, "%s: probed\n", LIS3DHH_ACC_DEV_NAME);

	return 0;

err_input_cleanup:
	lis3dhh_acc_input_cleanup(stat);

err_remove_hr_work_queue:
	if (stat->hr_timer_poll_work_queue) {
		flush_workqueue(stat->hr_timer_poll_work_queue);
		destroy_workqueue(stat->hr_timer_poll_work_queue);
		stat->hr_timer_poll_work_queue = NULL;
	}

err_wk_null:
	lis3dhh_acc_device_power_off(stat);

err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	pr_err("%s: Driver Init failed\n", LIS3DHH_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lis3dhh_common_probe);

int lis3dhh_common_remove(struct lis3dhh_acc_status *stat)
{
	dev_info(stat->dev, "driver removing\n");

	lis3dhh_acc_disable(stat);
	lis3dhh_acc_input_cleanup(stat);
	remove_sysfs_interfaces(stat->dev);

	if (stat->hr_timer_poll_work_queue) {
		flush_workqueue(stat->hr_timer_poll_work_queue);
		destroy_workqueue(stat->hr_timer_poll_work_queue);
		stat->hr_timer_poll_work_queue = NULL;
	}

	if (stat->pdata->exit)
		stat->pdata->exit();

	if (stat->pdata) {
		kfree(stat->pdata);
		stat->pdata = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(lis3dhh_common_remove);

#ifdef CONFIG_PM_SLEEP
int lis3dhh_common_resume(struct lis3dhh_acc_status *stat)
{
	if (stat->on_before_suspend)
		return lis3dhh_acc_enable(stat);

	return 0;
}
EXPORT_SYMBOL(lis3dhh_common_resume);

int lis3dhh_common_suspend(struct lis3dhh_acc_status *stat)
{
	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lis3dhh_acc_disable(stat);
}
EXPORT_SYMBOL(lis3dhh_common_suspend);
#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("lis3dhh acc driver");
MODULE_AUTHOR("Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
