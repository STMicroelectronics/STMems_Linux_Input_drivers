/*
* Copyright (C) 2016 STMicroelectronics
*
* Authors: Mario Tesi (mario.tesi@st.com)
*
* Authors is willing to be considered the contact and update points for
* the driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 1.0.0 2016/May/20:
	first release
******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "uvis25.h"

/* Expected device */
#define	WHOAMI_UVIS25	0xCA

/* REGISTERS */
#define	WHO_AM_I_REG	0x0F	/* Who Am I register */
#define	CTRL_REG1	0x20	/* ODR control reg */
#define	CTRL_REG2	0x21	/* boot reg */
#define	CTRL_REG3	0x22	/* interrupt control reg */
#define	INT_CFG_REG	0x23	/* interrupt config reg */
#define	INT_SRC_REG	0x24	/* interrupt source reg */
#define THS_UV 		0x25	/* Threshold register */
#define	STATUS_REG	0x27	/* status reg */
#define	UV_OUT_REG 	0x28	/* 8 bit output */

/* Masks */
#define	ODR_MASK	0x01	/* ctrl_reg1 */
#define	BDU_MASK	0x02	/* ctrl_reg1 */
#define	ONESH_MASK	0x01	/* ctrl_reg2 */

/* Usefull data */
#define	BDU_ON		BDU_MASK /* En BDU Block Data Upd */

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1	0
#define	RES_CTRL_REG2	1
#define	RES_CTRL_REG3	2
#define	RES_INT_CFG_REG	3
#define	RES_THS_UV	4

/* Output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot */
#define	ODR_1_1		0x01	/* 1 Hz ODR */

#define PM_OFF (ODR_MASK & ODR_ONESH)

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} uvis25_odr_table[] = {
	{ 1000, ODR_1_1 },
};

/* Default platform data value */
static struct uvis25_platform_data default_uvis25_pdata = {
	.poll_interval = 1000,
	.min_interval = UVIS25_MIN_POLL_PERIOD_MS,
	.ev_type = INPUT_EVENT_TYPE,
};

static int uvis25_hw_init(struct uvis25_data *stat)
{
	int err;
	u8 buf[2];

	dev_dbg(stat->dev, "%s: hw init start\n", UVIS25_DEV_NAME);

	buf[0] = stat->resume_state[RES_THS_UV];
	err = stat->tf->write(stat, THS_UV, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL_REG2];
	buf[1] = stat->resume_state[RES_CTRL_REG3];
	err = stat->tf->write(stat, CTRL_REG2, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_CFG_REG];
	err = stat->tf->write(stat, INT_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL_REG1];
	err = stat->tf->write(stat, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	dev_dbg(stat->dev, "%s: hw init done\n", UVIS25_DEV_NAME);

	return 0;

err_resume_state:
	stat->hw_initialized = 0;
	dev_err(stat->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);

	return err;
}

static void uvis25_device_power_off(struct uvis25_data *stat)
{
	int err;
	u8 buf;

	/* Power off: set ODR to 0, one shoot must be cleared but it shoul be yet
	 * in any case set to zero and never touch more time */
	if (stat->resume_state[RES_CTRL_REG2] & ONESH_MASK) {
		buf = stat->resume_state[RES_CTRL_REG2] & (~ONESH_MASK);
		err = stat->tf->write(stat, CTRL_REG2, 1, &buf);
		if (err < 0) {
			dev_err(stat->dev, "soft power off failed: %d\n", err);
			
			return;
		}
		stat->resume_state[RES_CTRL_REG2] = buf;
	}
	
	buf = PM_OFF;
	err = stat->tf->write(stat, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	stat->hw_initialized = 0;
}

static int uvis25_device_power_on(struct uvis25_data *stat)
{
	int err = -1;

	if (!stat->hw_initialized) {
		err = uvis25_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			uvis25_device_power_off(stat);

			return err;
		}
	}

	return 0;
}

static int uvis25_update_odr(struct uvis25_data *stat, int poll_period_ms)
{
	int err = -1;
	int i;
	u8 init_val, updated_val;
	u8 new_val;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (longest period) backward (shortest
	 * period), to support the poll_interval requested by the system.
	 * It must be the longest period shorter then the set poll period.*/
	for (i = ARRAY_SIZE(uvis25_odr_table) - 1; i >= 0; i--) {
		if ((uvis25_odr_table[i].cutoff_ms <= poll_period_ms) ||
		    (i == 0))
			break;
	}

	new_val = uvis25_odr_table[i].mask;

	/* read actual value */
	err = stat->tf->read(stat, CTRL_REG1, 1, &init_val);
	if (err < 0)
		goto error;

	stat->resume_state[RES_CTRL_REG1] = init_val;

	/* set new ODR */
	updated_val = (init_val & ~ODR_MASK) | new_val;
	err = stat->tf->write(stat, CTRL_REG1, 1, &updated_val);
	if (err < 0)
		goto error;

	/* Save new status */
	stat->resume_state[RES_CTRL_REG1] = updated_val;
	stat->pdata->poll_interval = uvis25_odr_table[i].cutoff_ms;

	return err;

error:
	dev_err(stat->dev, "update odr failed: %d\n", err);

	return err;
}

static int uvis25_get_uv_data(struct uvis25_data *stat, unsigned char *out)
{
	int err;

	err = stat->tf->read(stat, UV_OUT_REG, 1, out);
	if (err < 0)
		return err;

	dev_dbg(stat->dev, "UV %d = \n", *out);

	return err;
}

static void uvis25_report_values(struct uvis25_data *stat, unsigned char out)
{
	input_event(stat->input_dev, stat->pdata->ev_type, INPUT_EVENT_X, out);
	input_sync(stat->input_dev);
}

static int uvis25_enable(struct uvis25_data *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = uvis25_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}

		schedule_delayed_work(&stat->input_work,
			msecs_to_jiffies(stat->pdata->poll_interval));
	}

	return 0;
}

static int uvis25_disable(struct uvis25_data *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_delayed_work_sync(&stat->input_work);
		uvis25_device_power_off(stat);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int val;
	struct uvis25_data *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct uvis25_data *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms,
			  stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	uvis25_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct uvis25_data *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct uvis25_data *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	if (val)
		uvis25_enable(stat);
	else
		uvis25_disable(stat);
	mutex_unlock(&stat->lock);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(poll_period_ms, 0664, attr_get_polling_rate,
	       attr_set_polling_rate),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
};

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int create_sysfs_interfaces(struct device *dev)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		ret = device_create_file(dev, attributes + i);
		if (ret < 0)
			goto error;

	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: Unable to create interface\n", __func__);

	return ret;
}

static void uvis25_input_work_func(struct work_struct *work)
{
	struct uvis25_data *stat = container_of((struct delayed_work *)work,
						struct uvis25_data,
						input_work);

	unsigned char output;
	int err;

	err = uvis25_get_uv_data(stat, &output);
	if (err < 0)
		dev_err(stat->dev, "get data failed\n");
	else
		uvis25_report_values(stat, output);

	schedule_delayed_work(&stat->input_work,
			      msecs_to_jiffies(stat->pdata->poll_interval));
}

static int uvis25_input_init(struct uvis25_data *stat)
{
	int err;

	INIT_DELAYED_WORK(&stat->input_work, uvis25_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		dev_err(stat->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	stat->input_dev->name = stat->name;
	stat->input_dev->id.bustype = stat->bustype;
	stat->input_dev->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev, stat);

	__set_bit(stat->pdata->ev_type, stat->input_dev->evbit);
	if (stat->pdata->ev_type == EV_MSC) {
		dev_info(stat->dev, "%s: Init event type EV_REL\n", UVIS25_DEV_NAME);
		__set_bit(INPUT_EVENT_X, stat->input_dev->mscbit);
	} else {
		dev_info(stat->dev, "%s: Init event type EV_ABS\n", UVIS25_DEV_NAME);
		__set_bit(ABS_X, stat->input_dev->absbit);
		input_set_abs_params(stat->input_dev, ABS_X, UVIS25_MIN_UV, UVIS25_MAX_UV, 
						     0, 0);
	}
		
	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(stat->dev,
			"unable to register input polled device %s\n",
			stat->input_dev->name);
		input_free_device(stat->input_dev);

		return err;
	}

	return 0;
}

static void uvis25_input_cleanup(struct uvis25_data *stat)
{
	input_unregister_device(stat->input_dev);
}

int uvis25_common_probe(struct uvis25_data *stat)
{
	int err = -1;
	u8 wai;

	dev_info(stat->dev, "probe start.\n");

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	/* read chip WHO_AM_I_REG */
	err = stat->tf->read(stat, WHO_AM_I_REG, 1, &wai);
	if (wai != WHOAMI_UVIS25) {
		err = -ENODEV;
		dev_err(stat->dev, "Not replying or not recognized\n");
		goto err_mutexunlock;
	}

	stat->pdata = kzalloc(sizeof(struct uvis25_platform_data), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(stat->dev, "failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	/* select default or custom platform_data */
	if (stat->dev->platform_data == NULL) {
		memcpy(stat->pdata, &default_uvis25_pdata, sizeof(*stat->pdata));
		dev_info(stat->dev, "using default platform_data\n");
	} else
		memcpy(stat->pdata, stat->dev->platform_data, sizeof(*stat->pdata));

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	/* init registers which need values different from zero */
	stat->resume_state[RES_CTRL_REG1] = ((ODR_MASK & ODR_1_1) |
					     (BDU_MASK & BDU_ON));

	err = uvis25_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto err_exit_pointer;
	}

	atomic_set(&stat->enabled, 1);

	err = uvis25_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = uvis25_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev,
			"device UVIS25_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	uvis25_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	mutex_unlock(&stat->lock);

	dev_info(stat->dev, "%s: probed\n", UVIS25_DEV_NAME);

	return 0;

err_input_cleanup:
	uvis25_input_cleanup(stat);
err_power_off:
	uvis25_device_power_off(stat);
err_exit_pointer:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);

	pr_err("%s: Driver Init failed\n", UVIS25_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(uvis25_common_probe);

int uvis25_common_remove(struct uvis25_data *stat)
{
	cancel_delayed_work_sync(&stat->input_work);
	uvis25_input_cleanup(stat);
	uvis25_device_power_off(stat);
	remove_sysfs_interfaces(stat->dev);
	kfree(stat->pdata);

	return 0;
}
EXPORT_SYMBOL(uvis25_common_remove);

#ifdef CONFIG_PM
int uvis25_common_resume(struct uvis25_data *stat)
{
	if (stat->on_before_suspend)
		return uvis25_enable(stat);
		
	pr_err("%s: Resuming Driver\n", UVIS25_DEV_NAME);

	return 0;
}
EXPORT_SYMBOL(uvis25_common_resume);

int uvis25_common_suspend(struct uvis25_data *stat)
{
	stat->on_before_suspend = atomic_read(&stat->enabled);

	pr_err("%s: Sunspendig Driver\n", UVIS25_DEV_NAME);

	return uvis25_disable(stat);
}
EXPORT_SYMBOL(uvis25_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("STMicrolelectronics uvis25 sensor driver");
MODULE_AUTHOR("Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
