/*
 * STMicroelectronics ism303dac_mag.c driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/version.h>
#include "ism303dac_core.h"


/* DEVICE REGISTERS */
#define ISM303DAC_MAG_WHO_AM_I_ADDR		(0x4F)
#define ISM303DAC_MAG_WHO_AM_I_VAL		(0x40)
#define ISM303DAC_MAG_CFG_REG_A			(0x60)
#define ISM303DAC_MAG_CFG_REG_B			(0x61)
#define ISM303DAC_MAG_CFG_REG_B_OFF_CANC_MASK	(0x02)
#define ISM303DAC_MAG_CFG_REG_C			(0x62)
#define ISM303DAC_MAG_CFG_REG_C_BDU_MASK		(0x10)
#define ISM303DAC_MAG_OUT_L_ADDR			(0x68)

/* Device operating modes */
#define ISM303DAC_MAG_MD_CONTINUOS_MODE		0x00
#define ISM303DAC_MAG_MD_SINGLE_MODE		0x01
#define ISM303DAC_MAG_MD_IDLE_MODE		0x03
#define ISM303DAC_MAG_MODE_MASK			0x03

/* Device ODRs */
#define ISM303DAC_MAG_ODR_10HZ			0x00
#define ISM303DAC_MAG_ODR_20HZ			0x01
#define ISM303DAC_MAG_ODR_50HZ			0x02
#define ISM303DAC_MAG_ODR_100HZ			0x03
#define ISM303DAC_MAG_ODR_MASK			0x0C
#define ISM303DAC_MAG_BYTE_FOR_SAMPLE		6
#define ISM303DAC_MAG_ODR_LIST_NUM		4
#define ISM303DAC_MAG_DEV_DESCRIPTION		"ST ISM303DAC Magnetometer Sensor"
#define ISM303DAC_MAG_DEV_NAME			"magn"
#define ISM303DAC_MAGN_NAME			"ism303dac_magn"
#define ISM303DAC_MAG_SCALE			1500

struct ism303dac_odr_reg {
	u32 hz;
	u8 value;
} ism303dac_mag_odr_table_t[] = {
	[0] = { .hz = 10, 	.value = ISM303DAC_MAG_ODR_10HZ },
	[1] = { .hz = 20, 	.value = ISM303DAC_MAG_ODR_20HZ },
	[2] = { .hz = 50, 	.value = ISM303DAC_MAG_ODR_50HZ },
	[3] = { .hz = 100, 	.value = ISM303DAC_MAG_ODR_100HZ },
};

static void ism303dac_mag_poll_wk(struct work_struct *input_work)
{
	struct st_sensor_data *sdata;
	int xyz[3] = { 0 };
	u8 data[6];
	int err;

	sdata = container_of((struct work_struct *)input_work,
			     struct st_sensor_data, input_work);

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

	err = sdata->cdata->tf->read(sdata->cdata, ISM303DAC_MAG_OUT_L_ADDR,
						ISM303DAC_MAG_BYTE_FOR_SAMPLE,
						data);
	if (err < 0)
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
			sdata->name, err);
	else {
		xyz[0] = (s32)((s16)(data[0] | data[1] << 8));
		xyz[1] = (s32)((s16)(data[2] | data[3] << 8));
		xyz[2] = (s32)((s16)(data[4] | data[5] << 8));

		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;
		st_ism303dac_sensor_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
}

int ism303dac_mag_write_odr(struct st_sensor_data *sdata) {
	int i;

	for (i = 0; i < ISM303DAC_MAG_ODR_LIST_NUM; i++) {
		if (ism303dac_mag_odr_table_t[i].hz >= sdata->c_odr)
				break;
	}
	if (i == ISM303DAC_MAG_ODR_LIST_NUM)
		return -EINVAL;

	return st_ism303dac_sensor_write_data(sdata->cdata,
				ISM303DAC_MAG_CFG_REG_A,
				ISM303DAC_MAG_ODR_MASK,
				ism303dac_mag_odr_table_t[i].value);
}

int ism303dac_mag_enable_sensors(struct st_sensor_data *sdata)
{
	int err;

	if (sdata->enabled)
		return 0;

	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_MAG_CFG_REG_A,
						ISM303DAC_MAG_MODE_MASK,
						ISM303DAC_MAG_MD_CONTINUOS_MODE);
	if (err < 0)
		return err;

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
	sdata->enabled = true;

	return 0;
}

int ism303dac_mag_disable_sensor(struct st_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_MAG_CFG_REG_A,
						ISM303DAC_MAG_MODE_MASK,
						ISM303DAC_MAG_MD_IDLE_MODE);
	if (err < 0)
		return err;

	cancel_work_sync(&sdata->input_work);
	hrtimer_cancel(&sdata->hr_timer);
	sdata->enabled = false;

	return 0;
}

ssize_t ism303dac_mag_get_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "50\n");
}

int ism303dac_mag_init_sensor(struct st_common_data *cdata)
{
	int err;
	struct st_sensor_data *sdata = cdata->sensors;

	INIT_WORK(&cdata->sensors->input_work, ism303dac_mag_poll_wk);
	hrtimer_init(&cdata->sensors->hr_timer, CLOCK_MONOTONIC,
					HRTIMER_MODE_REL);

	err = ism303dac_mag_disable_sensor(sdata);
	if (err < 0)
		return err;

	cdata->sensors->hr_timer.function = &st_ism303dac_sensor_poll_function;

	/*
	* Enable offset cancellation
	*/
	err = st_ism303dac_sensor_write_data(cdata, ISM303DAC_MAG_CFG_REG_B,
					ISM303DAC_MAG_CFG_REG_B_OFF_CANC_MASK,
					ISM303DAC_EN_BIT);
	if (err < 0)
		return err;


	/*
	 * Enable block data update feature.
	 */
	return st_ism303dac_sensor_write_data(cdata, ISM303DAC_MAG_CFG_REG_C,
					ISM303DAC_MAG_CFG_REG_C_BDU_MASK,
					ISM303DAC_EN_BIT);
}

ADD_DEVICE_ENABLE_ATTR;
ADD_DEVICE_POLLING_ATTR;

static DEVICE_ATTR(scale_avail, S_IRUGO,
				ism303dac_mag_get_scale_avail,
				NULL);

static struct attribute *ism303dac_mag_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_scale_avail.attr,
	NULL,
};

static const struct attribute_group ism303dac_mag_attribute_groups = {
	.attrs = ism303dac_mag_attribute,
	.name = ISM303DAC_MAG_DEV_NAME,
};

int ism303dac_mag_data_init(struct st_common_data *cdata)
{
	struct st_sensor_data *sdata = cdata->sensors;

	sdata->enabled = false;
	sdata->cdata = cdata;
	sdata->sindex = 0;
	sdata->c_gain = ISM303DAC_MAG_SCALE;
	sdata->name = ISM303DAC_MAG_DEV_NAME;
	sdata->c_odr = ism303dac_mag_odr_table_t[0].hz;
	sdata->ktime = ktime_set(0, HZ_TO_NSEC(sdata->c_odr));

	st_ism303dac_sensor_input_init(sdata, ISM303DAC_MAG_DEV_DESCRIPTION, true);

	if (sysfs_create_group(&sdata->input_dev->dev.kobj,
					&ism303dac_mag_attribute_groups)) {
		dev_err(cdata->dev, "failed to create sysfs group for sensor %s",
					sdata->name);
		return -EINVAL;
	}

	sdata->write_odr = ism303dac_mag_write_odr;
	sdata->enable = ism303dac_mag_enable_sensors;
	sdata->disable = ism303dac_mag_disable_sensor;

	return 0;
}

int ism303dac_mag_probe(struct st_common_data *cdata)
{
	int err;

	err = st_ism303dac_sensor_common_probe(cdata, 0);
	if (err < 0)
		return err;

	err = ism303dac_mag_data_init(cdata);
	if (err < 0)
		return err;

	err = ism303dac_mag_init_sensor(cdata);
	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(ism303dac_mag_probe);

int ism303dac_mag_remove(struct st_common_data *cdata)
{
	ism303dac_mag_disable_sensor(cdata->sensors);
	st_ism303dac_sensor_input_cleanup(cdata->sensors);

	st_ism303dac_sensor_common_remove(cdata);

	return 0;
}
EXPORT_SYMBOL(ism303dac_mag_remove);

int ism303dac_mag_enable(struct st_common_data *cdata)
{
	return ism303dac_mag_enable_sensors(cdata->sensors);
}
EXPORT_SYMBOL(ism303dac_mag_enable);

int ism303dac_mag_disable(struct st_common_data *cdata)
{
	return ism303dac_mag_disable_sensor(cdata->sensors);
}
EXPORT_SYMBOL(ism303dac_mag_disable);

MODULE_DESCRIPTION("STMicroelectronics ism303dac_mag i2c driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_LICENSE("GPL v2");

