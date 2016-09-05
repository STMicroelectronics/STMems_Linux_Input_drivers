/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: l3g4200d_gyr_sysfs.c
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
			: Mario Tesi (mario.tesi@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.5
* Date			: 2016/Apr/28
* Description		: L3G4200D digital output gyroscope sensor API
*
********************************************************************************
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
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0		| 2010/11/19	| Carmine Iascone	| First Release
* 1.1.0		| 2011/02/28	| Matteo Dameno		| Self Test Added
* 1.1.1		| 2011/05/25	| Matteo Dameno		| Corrects Polling Bug
* 1.1.2		| 2011/05/30	| Matteo Dameno		| Corrects ODR Bug
* 1.1.3		| 2011/06/24	| Matteo Dameno		| Corrects ODR Bug
* 1.1.4		| 2011/09/24	| Matteo Dameno		| forces BDU setting
* 1.1.5		| 2016/04/28	| Mario Tesi		| add spi support
*******************************************************************************/

#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "l3g4200d.h"

/** Maximum polled-device-reported rot speed value value in dps*/
#define FS_MAX		32768

/* l3g4200d gyroscope registers */
#define WHO_AM_I        0x0F
#define WHOAMI_L3G4200D	0xD3	/* Expected content for WAI register*/


#define CTRL_REG1       0x20    /* CTRL REG1 */
#define CTRL_REG2       0x21    /* CTRL REG2 */
#define CTRL_REG3       0x22    /* CTRL_REG3 */
#define CTRL_REG4       0x23    /* CTRL_REG4 */
#define CTRL_REG5       0x24    /* CTRL_REG5 */

/* CTRL_REG1 */
#define ALL_ZEROES	0x00
#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07
#define BW00		0x00
#define BW01		0x10
#define BW10		0x20
#define BW11		0x30
#define ODR100		0x00  /* ODR = 100Hz */
#define ODR200		0x40  /* ODR = 200Hz */
#define ODR400		0x80  /* ODR = 400Hz */
#define ODR800		0xC0  /* ODR = 800Hz */

/* CTRL_REG4 bits */
#define	FS_MASK				0x30
#define	BDU_ENABLE			0x80

#define	SELFTEST_MASK			0x06
#define L3G4200D_SELFTEST_DIS		0x00
#define L3G4200D_SELFTEST_EN_POS	0x02
#define L3G4200D_SELFTEST_EN_NEG	0x04

#define AXISDATA_REG			0x28

#define FUZZ		0
#define FLAT		0
#define AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1	0
#define	RES_CTRL_REG2	1
#define	RES_CTRL_REG3	2
#define	RES_CTRL_REG4	3
#define	RES_CTRL_REG5	4

//#define DEBUG 1

/*
 * L3G4200D gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */

static const struct output_rate odr_table[] = {
	{	2,	ODR800|BW10},
	{	3,	ODR400|BW01},
	{	5,	ODR200|BW00},
	{	10,	ODR100|BW00},
};

static int l3g4200d_register_write(struct l3g4200d_data *gyro, u8 *buf,
				   u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = new_value;
	err = gyro->tf->write(gyro, reg_address, 1, buf);
	if (err < 0)
		return err;

	return err;
}

static int l3g4200d_register_read(struct l3g4200d_data *gyro, u8 *buf,
				  u8 reg_address)
{
	return gyro->tf->read(gyro, reg_address, 1, buf);
}

static int l3g4200d_register_update(struct l3g4200d_data *gyro, u8 *buf,
				    u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;

	err = l3g4200d_register_read(gyro, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = l3g4200d_register_write(gyro, buf, reg_address,
					      updated_val);
	}

	return err;
}

static int l3g4200d_update_fs_range(struct l3g4200d_data *gyro,
				    u8 new_fs)
{
	int res ;
	u8 buf[2];

	buf[0] = CTRL_REG4;

	res = l3g4200d_register_update(gyro, buf, CTRL_REG4,
				       FS_MASK, new_fs);

	if (res < 0) {
		pr_err("%s : failed to update fs:0x%02x\n",
			__func__, new_fs);
		return res;
	}
	gyro->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs ) |
		( ~FS_MASK & gyro->resume_state[RES_CTRL_REG4]));

	return res;
}

static int l3g4200d_selftest(struct l3g4200d_data *gyro, u8 enable)
{
	int err = -1;
	u8 buf[2] = {0x00,0x00};
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = SELFTEST_MASK;
	if (enable > 0)
		bit_values = L3G4200D_SELFTEST_EN_POS;
	else
		bit_values = L3G4200D_SELFTEST_DIS;

	if (atomic_read(&gyro->enabled)) {
		mutex_lock(&gyro->lock);
		err = l3g4200d_register_update(gyro, buf, reg_address,
					       mask, bit_values);
		gyro->selftest_enabled = enable;
		mutex_unlock(&gyro->lock);
		if (err < 0)
			return err;
		gyro->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				( ~mask & gyro->resume_state[RES_CTRL_REG4]));
	}
	return err;
}

static int l3g4200d_update_odr(struct l3g4200d_data *gyro,
				int poll_interval)
{
	int err = -1;
	int i;
	u8 config;

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval) || (i == 0))
			break;
	}

	config = odr_table[i].mask;
	config |= (ENABLE_ALL_AXES + PM_NORMAL);

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&gyro->enabled)) {
		err = gyro->tf->write(gyro, CTRL_REG1, 1, &config);
		if (err < 0)
			return err;
		gyro->resume_state[RES_CTRL_REG1] = config;
	}

	return err;
}

/* data readout */
static int l3g4200d_get_data(struct l3g4200d_data *gyro,
			     struct l3g4200d_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s16 hw_d[3] = { 0 };

	err = gyro->tf->read(gyro, AXISDATA_REG, 6, gyro_out);
	if (err < 0)
		return err;

	hw_d[0] = (s16)(((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s16)(((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s16)(((gyro_out[5]) << 8) | gyro_out[4]);

	data->x = ((gyro->pdata->negate_x) ? (-hw_d[gyro->pdata->axis_map_x])
		   : (hw_d[gyro->pdata->axis_map_x]));
	data->y = ((gyro->pdata->negate_y) ? (-hw_d[gyro->pdata->axis_map_y])
		   : (hw_d[gyro->pdata->axis_map_y]));
	data->z = ((gyro->pdata->negate_z) ? (-hw_d[gyro->pdata->axis_map_z])
		   : (hw_d[gyro->pdata->axis_map_z]));

	return err;
}

static void l3g4200d_report_values(struct l3g4200d_data *l3g,
				   struct l3g4200d_triple *data)
{
	struct input_dev *input = l3g->input_poll_dev->input;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data->x);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, data->y);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, data->z);
	input_sync(input);
}

static int l3g4200d_hw_init(struct l3g4200d_data *gyro)
{
	int err = -1;
	u8 buf[5];

	pr_info("%s hw init\n", L3G4200D_DEV_NAME);

	buf[0] = gyro->resume_state[RES_CTRL_REG1];
	buf[1] = gyro->resume_state[RES_CTRL_REG2];
	buf[2] = gyro->resume_state[RES_CTRL_REG3];
	buf[3] = gyro->resume_state[RES_CTRL_REG4];
	buf[4] = gyro->resume_state[RES_CTRL_REG5];

	err = gyro->tf->write(gyro, CTRL_REG1, 5, buf);
	if (err < 0)
		return err;

	gyro->hw_initialized = 1;

	return err;
}

static void l3g4200d_device_power_off(struct l3g4200d_data *dev_data)
{
	int err;
	u8 buf;

	pr_info("%s power off\n", L3G4200D_DEV_NAME);

	buf = PM_OFF;
	err = dev_data->tf->write(dev_data, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(dev_data->dev, "soft power off failed\n");

	if (dev_data->pdata->power_off) {
		dev_data->pdata->power_off();
		dev_data->hw_initialized = 0;
	}

	if (dev_data->hw_initialized)
		dev_data->hw_initialized = 0;
}

static int l3g4200d_device_power_on(struct l3g4200d_data *dev_data)
{
	int err;

	if (dev_data->pdata->power_on) {
		err = dev_data->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!dev_data->hw_initialized) {
		err = l3g4200d_hw_init(dev_data);
		if (err < 0) {
			l3g4200d_device_power_off(dev_data);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_enable(struct l3g4200d_data *dev_data)
{
	int err;

	if (!atomic_cmpxchg(&dev_data->enabled, 0, 1)) {
		err = l3g4200d_device_power_on(dev_data);
		if (err < 0) {
			atomic_set(&dev_data->enabled, 0);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_disable(struct l3g4200d_data *dev_data)
{
	if (atomic_cmpxchg(&dev_data->enabled, 1, 0))
		l3g4200d_device_power_off(dev_data);

	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	mutex_lock(&gyro->lock);
	val = gyro->input_poll_dev->poll_interval;
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms,gyro->pdata->min_interval);
	mutex_lock(&gyro->lock);
	gyro->input_poll_dev->poll_interval = interval_ms;
	gyro->pdata->poll_interval = interval_ms;
	l3g4200d_update_odr(gyro, interval_ms);
	mutex_unlock(&gyro->lock);

	return size;
}

static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int range = 0;
	char val;

	mutex_lock(&gyro->lock);
	val = gyro->pdata->fs_range;
	switch (val) {
	case L3G4200D_GYR_FS_250DPS:
		range = 250;
		break;
	case L3G4200D_GYR_FS_500DPS:
		range = 500;
		break;
	case L3G4200D_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);
	gyro->pdata->fs_range = val;
	l3g4200d_update_fs_range(gyro, val);
	mutex_unlock(&gyro->lock);

	return size;
}

static ssize_t attr_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int val = atomic_read(&gyro->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		l3g4200d_enable(gyro);
	else
		l3g4200d_disable(gyro);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	mutex_lock(&gyro->lock);
	val = gyro->selftest_enabled;
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	l3g4200d_selftest(gyro, val);

	return size;
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	u8 x;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	x = val;
	gyro->tf->write(gyro, gyro->reg_addr, 1, &x);

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&gyro->lock);
	data = gyro->reg_addr;
	mutex_unlock(&gyro->lock);
	rc = gyro->tf->read(gyro, gyro->reg_addr, 1, &data);
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);

	gyro->reg_addr = val;

	mutex_unlock(&gyro->lock);

	return size;
}
#endif /* DEBUG */

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_polling_rate_show,
	       attr_polling_rate_store),
	__ATTR(range, 0666, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0666, attr_enable_show, attr_enable_store),
	__ATTR(enable_selftest, 0666, attr_get_selftest, attr_set_selftest),
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

static void l3g4200d_input_poll_func(struct input_polled_dev *dev)
{
	struct l3g4200d_data *gyro = dev->private;
	struct l3g4200d_triple data_out;
	int err;

	err = l3g4200d_get_data(gyro, &data_out);
	if (err < 0)
		dev_err(gyro->dev, "get_gyroscope_data failed\n");
	else
		l3g4200d_report_values(gyro, &data_out);
}

int l3g4200d_input_open(struct input_dev *input)
{
	struct l3g4200d_data *gyro = input_get_drvdata(input);

	return l3g4200d_enable(gyro);
}

void l3g4200d_input_close(struct input_dev *dev)
{
	struct l3g4200d_data *gyro = input_get_drvdata(dev);

	l3g4200d_disable(gyro);
}

static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	/* checks for correctness of minimal polling period */
	gyro->pdata->min_interval =
		max((unsigned int)L3G4200D_MIN_POLL_PERIOD_MS,
		    gyro->pdata->min_interval);

	gyro->pdata->poll_interval = max(gyro->pdata->poll_interval,
					 gyro->pdata->min_interval);

	if (gyro->pdata->axis_map_x > 2 ||
	    gyro->pdata->axis_map_y > 2 ||
	    gyro->pdata->axis_map_z > 2) {
		dev_err(gyro->dev,
			"invalid axis_map value x:%u y:%u z:%u\n",
			gyro->pdata->axis_map_x,
			gyro->pdata->axis_map_y,
			gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
	    gyro->pdata->negate_y > 1 ||
	    gyro->pdata->negate_z > 1) {
		dev_err(gyro->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			gyro->pdata->negate_x,
			gyro->pdata->negate_y,
			gyro->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (gyro->pdata->poll_interval < gyro->pdata->min_interval) {
		dev_err(gyro->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int l3g4200d_input_init(struct l3g4200d_data *gyro)
{
	int err = -1;
	struct input_dev *input;


	gyro->input_poll_dev = input_allocate_polled_device();
	if (!gyro->input_poll_dev) {
		err = -ENOMEM;
		dev_err(gyro->dev, "input device allocate failed\n");
		goto err0;
	}

	gyro->input_poll_dev->private = gyro;
	gyro->input_poll_dev->poll = l3g4200d_input_poll_func;
	gyro->input_poll_dev->poll_interval = gyro->pdata->poll_interval;

	input = gyro->input_poll_dev->input;
	input->name = L3G4200D_DEV_NAME;
	input->open = l3g4200d_input_open;
	input->close = l3g4200d_input_close;
	input->id.bustype = gyro->bustype;
	input->dev.parent = gyro->dev;
	input_set_drvdata(gyro->input_poll_dev->input, gyro);

	__set_bit(INPUT_EVENT_TYPE, input->evbit );
	__set_bit(INPUT_EVENT_X, input->mscbit);
	__set_bit(INPUT_EVENT_Y, input->mscbit);
	__set_bit(INPUT_EVENT_Z, input->mscbit);

	err = input_register_polled_device(gyro->input_poll_dev);
	if (err) {
		dev_err(gyro->dev,
			"unable to register input polled device %s\n",
			gyro->input_poll_dev->input->name);
		goto err1;
	}

	return 0;

err1:
	input_free_polled_device(gyro->input_poll_dev);
err0:
	return err;
}

static void l3g4200d_input_cleanup(struct l3g4200d_data *gyro)
{
	input_unregister_polled_device(gyro->input_poll_dev);
	input_free_polled_device(gyro->input_poll_dev);
}

#ifdef CONFIG_OF
static u32 l3g4200d_parse_dt(struct l3g4200d_data *gyro, struct device* dev)
{
	/* TODO: Add parsing of default configuration */
	gyro->pdata->axis_map_x = 0;
	gyro->pdata->axis_map_y = 1;
	gyro->pdata->axis_map_z = 2;
	gyro->pdata->negate_x = 0;
	gyro->pdata->negate_y = 0;
	gyro->pdata->negate_z = 0;
	gyro->pdata->fs_range = L3G4200D_GYR_FS_250DPS;

	return 0;
}
#endif

int l3g4200d_common_probe(struct l3g4200d_data *gyro)
{
	int err = -1;
	u8 whoami;

	pr_info("%s: probe start.\n", L3G4200D_DEV_NAME);
	
	mutex_init(&gyro->lock);
	mutex_init(&gyro->tb.buf_lock);

	err = gyro->tf->read(gyro, WHO_AM_I, 1, &whoami);
	if (err < 0) {
		dev_warn(gyro->dev, "Error reading WHO_AM_I: is device "
			 "available/working?\n");
		return -ENODEV;
	}

	if (whoami != WHOAMI_L3G4200D) {
		dev_err(gyro->dev,
			"device unknown. Expected: 0x%x,"
			" Replies: 0x%x\n", WHOAMI_L3G4200D, whoami);
		return -ENODEV;
	}

#ifndef CONFIG_OF
	if (gyro->dev->platform_data == NULL) {
		dev_err(gyro->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}
#else
	gyro->dev->platform_data = NULL;
#endif

	mutex_lock(&gyro->lock);

	gyro->pdata = kzalloc(sizeof(struct l3g4200d_platform_data), GFP_KERNEL);
	if (gyro->pdata == NULL) {
		dev_err(gyro->dev, "failed to allocate memory for pdata: %d\n",
			err);
		goto err1;
	}
#ifdef CONFIG_OF
	l3g4200d_parse_dt(gyro, gyro->dev);
#else
	memcpy(gyro->pdata, gyro->dev->platform_data,
	       sizeof(*gyro->pdata));
#endif

	err = l3g4200d_validate_pdata(gyro);
	if (err < 0) {
		dev_err(gyro->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	if (gyro->pdata->init) {
		err = gyro->pdata->init();
		if (err < 0) {
			dev_err(gyro->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}

	memset(gyro->resume_state, 0, ARRAY_SIZE(gyro->resume_state));

	gyro->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES;
	gyro->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	gyro->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	gyro->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	gyro->resume_state[RES_CTRL_REG5] = ALL_ZEROES;

	err = l3g4200d_device_power_on(gyro);
	if (err < 0) {
		dev_err(gyro->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&gyro->enabled, 1);

	err = l3g4200d_update_fs_range(gyro, gyro->pdata->fs_range);
	if (err < 0) {
		dev_err(gyro->dev, "update_fs_range failed\n");
		goto err2;
	}

	err = l3g4200d_update_odr(gyro, gyro->pdata->poll_interval);
	if (err < 0) {
		dev_err(gyro->dev, "update_odr failed\n");
		goto err2;
	}

	err = l3g4200d_input_init(gyro);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(gyro->dev);
	if (err < 0) {
		dev_err(gyro->dev, "%s device register failed\n",
			L3G4200D_DEV_NAME);
		goto err4;
	}

	l3g4200d_device_power_off(gyro);

	/* As default, do not report information */
	atomic_set(&gyro->enabled, 0);

	mutex_unlock(&gyro->lock);

	pr_info("%s: probed\n", L3G4200D_DEV_NAME);

	return 0;

err4:
	l3g4200d_input_cleanup(gyro);
err3:
	l3g4200d_device_power_off(gyro);
err2:
	if (gyro->pdata->exit)
		gyro->pdata->exit();
err1_1:
	mutex_unlock(&gyro->lock);
	kfree(gyro->pdata);
err1:
	pr_err("%s: Driver Initialization failed\n", L3G4200D_DEV_NAME);
	return err;
}
EXPORT_SYMBOL(l3g4200d_common_probe);

int l3g4200d_common_remove(struct l3g4200d_data *gyro)
{
#ifdef DEBUG
	pr_info(KERN_INFO "L3G4200D driver removing\n");
#endif
	l3g4200d_input_cleanup(gyro);
	l3g4200d_device_power_off(gyro);
	remove_sysfs_interfaces(gyro->dev);
	kfree(gyro->pdata);

	return 0;
}
EXPORT_SYMBOL(l3g4200d_common_remove);

#ifdef CONFIG_PM
int l3g4200d_common_suspend(struct l3g4200d_data *gyro)
{
#ifdef DEBUG
	pr_info(KERN_INFO "l3g4200d_suspend\n");
#endif /* DEBUG */
	return 0;
}
EXPORT_SYMBOL(l3g4200d_common_suspend);

int l3g4200d_common_resume(struct l3g4200d_data *gyro)
{
#ifdef DEBUG
	pr_info(KERN_INFO "l3g4200d_resume\n");
#endif /*DEBUG */
	return 0;
}
EXPORT_SYMBOL(l3g4200d_common_resume);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("l3g4200d digital gyroscope sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
