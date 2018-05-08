/*
 * STMicroelectronics ism303dac_acc.c driver
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
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include "ism303dac_core.h"

#define ISM303DAC_CTRL1_ADDR			0x20
#define ISM303DAC_CTRL2_ADDR			0x21
#define ISM303DAC_CTRL3_ADDR			0x22
#define ISM303DAC_CTRL4_INT1_PAD_ADDR		0x23
#define ISM303DAC_CTRL5_INT2_PAD_ADDR		0x24
#define ISM303DAC_STATUS_ADDR			0x27
#define ISM303DAC_OUTX_L_ADDR			0x28
#define ISM303DAC_TAP_THS_6D_ADDR		0x31
#define ISM303DAC_INT_DUR_ADDR			0x32
#define ISM303DAC_WAKE_UP_THS_ADDR		0x33
#define ISM303DAC_WAKE_UP_DUR_ADDR		0x34
#define ISM303DAC_FREE_FALL_ADDR			0x35
#define ISM303DAC_STATUS_DUP_ADDR		0x36
#define ISM303DAC_WAKE_UP_SRC_ADDR		0x37
#define ISM303DAC_TAP_SRC_ADDR			0x38
#define ISM303DAC_6D_SRC_ADDR			0x39

#define ISM303DAC_INT_STATUS_ADDR		ISM303DAC_STATUS_ADDR
#define ISM303DAC_WAKE_UP_IA_MASK		0x40
#define ISM303DAC_DOUBLE_TAP_MASK		0x10
#define ISM303DAC_SINGLE_TAP_MASK		0x08
#define ISM303DAC_6D_IA_MASK			0x04
#define ISM303DAC_FF_IA_MASK			0x02
#define ISM303DAC_DRDY_MASK			0x01
#define ISM303DAC_EVENT_MASK			(ISM303DAC_WAKE_UP_IA_MASK | \
						ISM303DAC_DOUBLE_TAP_MASK | \
						ISM303DAC_SINGLE_TAP_MASK | \
						ISM303DAC_6D_IA_MASK | \
						ISM303DAC_FF_IA_MASK)

#define ISM303DAC_ODR_ADDR			ISM303DAC_CTRL1_ADDR
#define ISM303DAC_ODR_MASK			0xf0
#define ISM303DAC_ODR_POWER_OFF_VAL		0x00
#define ISM303DAC_ODR_1HZ_LP_VAL			0x08
#define ISM303DAC_ODR_12HZ_LP_VAL		0x09
#define ISM303DAC_ODR_25HZ_LP_VAL		0x0a
#define ISM303DAC_ODR_50HZ_LP_VAL		0x0b
#define ISM303DAC_ODR_100HZ_LP_VAL		0x0c
#define ISM303DAC_ODR_200HZ_LP_VAL		0x0d
#define ISM303DAC_ODR_400HZ_LP_VAL		0x0e
#define ISM303DAC_ODR_800HZ_LP_VAL		0x0f
#define ISM303DAC_ODR_LP_LIST_NUM		9

#define ISM303DAC_ODR_12_5HZ_HR_VAL		0x01
#define ISM303DAC_ODR_25HZ_HR_VAL		0x02
#define ISM303DAC_ODR_50HZ_HR_VAL		0x03
#define ISM303DAC_ODR_100HZ_HR_VAL		0x04
#define ISM303DAC_ODR_200HZ_HR_VAL		0x05
#define ISM303DAC_ODR_400HZ_HR_VAL		0x06
#define ISM303DAC_ODR_800HZ_HR_VAL		0x07
#define ISM303DAC_ODR_HR_LIST_NUM		8

#define ISM303DAC_FS_ADDR			ISM303DAC_CTRL1_ADDR
#define ISM303DAC_FS_MASK			0x0c
#define ISM303DAC_FS_2G_VAL			0x00
#define ISM303DAC_FS_4G_VAL			0x02
#define ISM303DAC_FS_8G_VAL			0x03
#define ISM303DAC_FS_16G_VAL			0x01

/*
 * Sensitivity sets in LP mode [ug]
 */
#define ISM303DAC_FS_2G_GAIN_LP			3906
#define ISM303DAC_FS_4G_GAIN_LP			7813
#define ISM303DAC_FS_8G_GAIN_LP			15625
#define ISM303DAC_FS_16G_GAIN_LP			31250

/*
 * Sensitivity sets in HR mode [ug]
 */
#define ISM303DAC_FS_2G_GAIN_HR			244
#define ISM303DAC_FS_4G_GAIN_HR			488
#define ISM303DAC_FS_8G_GAIN_HR			976
#define ISM303DAC_FS_16G_GAIN_HR			1952

#define ISM303DAC_FS_LIST_NUM			4
enum {
	ISM303DAC_LP_MODE = 0,
	ISM303DAC_HR_MODE,
	ISM303DAC_MODE_COUNT,
};
#define ISM303DAC_PMODE_DEFAULT			ISM303DAC_LP_MODE

#define ISM303DAC_INT1_S_TAP_MASK		0x40
#define ISM303DAC_INT1_WAKEUP_MASK		0x20
#define ISM303DAC_INT1_FREE_FALL_MASK		0x10
#define ISM303DAC_INT1_TAP_MASK			0x08
#define ISM303DAC_INT1_6D_MASK			0x04
#define ISM303DAC_INT1_FTH_MASK			0x02
#define ISM303DAC_INT1_DRDY_MASK			0x01
#define ISM303DAC_INT1_EVENTS_MASK		0x7f

#define ISM303DAC_INT_DUR_SHOCK_MASK		0x03
#define ISM303DAC_INT_DUR_QUIET_MASK		0x0c
#define ISM303DAC_INT_DUR_LAT_MASK		0xf0
#define ISM303DAC_INT_DUR_MASK			(ISM303DAC_INT_DUR_SHOCK_MASK | \
						ISM303DAC_INT_DUR_QUIET_MASK | \
						ISM303DAC_INT_DUR_LAT_MASK)
#define ISM303DAC_INT_DUR_STAP_DEFAULT		0x06
#define ISM303DAC_INT_DUR_DTAP_DEFAULT		0x7f
#define ISM303DAC_WAKE_UP_THS_S_D_TAP_MASK	0x80
#define ISM303DAC_WAKE_UP_THS_SLEEP_MASK		0x40
#define ISM303DAC_WAKE_UP_THS_WU_MASK		0x3f
#define ISM303DAC_WAKE_UP_THS_WU_DEFAULT		0x02
#define ISM303DAC_FREE_FALL_THS_MASK		0x07
#define ISM303DAC_FREE_FALL_DUR_MASK		0xF8
#define ISM303DAC_FREE_FALL_THS_DEFAULT		0x01
#define ISM303DAC_FREE_FALL_DUR_DEFAULT		0x01
#define ISM303DAC_BDU_ADDR			ISM303DAC_CTRL1_ADDR
#define ISM303DAC_BDU_MASK			0x01
#define ISM303DAC_SOFT_RESET_ADDR		ISM303DAC_CTRL2_ADDR
#define ISM303DAC_SOFT_RESET_MASK		0x40
#define ISM303DAC_LIR_ADDR			ISM303DAC_CTRL3_ADDR
#define ISM303DAC_LIR_MASK			0x04
#define ISM303DAC_TAP_AXIS_ADDR			ISM303DAC_CTRL3_ADDR
#define ISM303DAC_TAP_AXIS_MASK			0x38
#define ISM303DAC_TAP_AXIS_ANABLE_ALL		0x07

#define ISM303DAC_TAP_THS_ADDR			ISM303DAC_TAP_THS_6D_ADDR
#define ISM303DAC_TAP_THS_MASK			0x1f
#define ISM303DAC_TAP_THS_DEFAULT		0x09

#define ISM303DAC_INT2_ON_INT1_ADDR		ISM303DAC_CTRL5_INT2_PAD_ADDR
#define ISM303DAC_INT2_ON_INT1_MASK		0x20

#define ISM303DAC_ACCEL_ODR			1
#define ISM303DAC_ACCEL_FS			2
#define ISM303DAC_FF_ODR				25
#define ISM303DAC_TAP_ODR			400
#define ISM303DAC_WAKEUP_ODR			25
#define ISM303DAC_ACTIVITY_ODR			12
#define ISM303DAC_MIN_EVENT_ODR			25

#define ISM303DAC_BYTE_FOR_SAMPLE		6
#define SET_BIT(a, b)				{a |= (1 << b);}
#define RESET_BIT(a, b)				{a &= ~(1 << b);}
#define CHECK_BIT(a, b)				(a & (1 << b))

#define ISM303DAC_NAME				"ism303dac_acc"

#define GET_PDATA(cdata)	((priv_data_t *)(cdata->priv_data))

static const struct ism303dac_sensors_table {
	const char *name;
	const char *description;
	const u32 min_odr_hz;
} ism303dac_sensors_table[ISM303DAC_SENSORS_NUMB] = {
	[ISM303DAC_ACCEL] = {
		.name = "accel",
		.description = "ST ISM303DAC Accelerometer Sensor",
		.min_odr_hz = ISM303DAC_ACCEL_ODR,
	},
	[ISM303DAC_FF] = {
		.name = "free_fall",
		.description = "ST ISM303DAC Free Fall Sensor",
		.min_odr_hz = ISM303DAC_FF_ODR,
	},
	[ISM303DAC_TAP] = {
		.name = "tap",
		.description = "ST ISM303DAC Tap Sensor",
		.min_odr_hz = ISM303DAC_TAP_ODR,
	},
	[ISM303DAC_DOUBLE_TAP] = {
		.name = "double_tap",
		.description = "ST ISM303DAC Double Tap Sensor",
		.min_odr_hz = ISM303DAC_TAP_ODR,
	},
	[ISM303DAC_WAKEUP] = {
		.name = "wake_up",
		.description = "ST ISM303DAC Free Fall Sensor",
		.min_odr_hz = ISM303DAC_WAKEUP_ODR,
	},
	[ISM303DAC_ACTIVITY] = {
		.name = "act",
		.description = "ST ISM303DAC Activity Sensor",
		.min_odr_hz = ISM303DAC_ACTIVITY_ODR,
	},
};

static const struct ism303dac_odr_reg {
	u32 hz;
	u8 value;
} ism303dac_odr_table[ISM303DAC_MODE_COUNT][ISM303DAC_ODR_LP_LIST_NUM] = {
	[ISM303DAC_LP_MODE] = {
		{ .hz = 0,	.value = ISM303DAC_ODR_POWER_OFF_VAL, },
		{ .hz = 1,	.value = ISM303DAC_ODR_1HZ_LP_VAL, },
		{ .hz = 12,	.value = ISM303DAC_ODR_12HZ_LP_VAL, },
		{ .hz = 25,	.value = ISM303DAC_ODR_25HZ_LP_VAL, },
		{ .hz = 50,	.value = ISM303DAC_ODR_50HZ_LP_VAL, },
		{ .hz = 100,	.value = ISM303DAC_ODR_100HZ_LP_VAL, },
		{ .hz = 200,	.value = ISM303DAC_ODR_200HZ_LP_VAL, },
		{ .hz = 400,	.value = ISM303DAC_ODR_400HZ_LP_VAL, },
		{ .hz = 800,	.value = ISM303DAC_ODR_800HZ_LP_VAL, },
	},
	[ISM303DAC_HR_MODE] = {
		{ .hz = 0,	.value = ISM303DAC_ODR_POWER_OFF_VAL, },
		{ .hz = 12,	.value = ISM303DAC_ODR_12_5HZ_HR_VAL },
		{ .hz = 25,	.value = ISM303DAC_ODR_25HZ_HR_VAL },
		{ .hz = 50,	.value = ISM303DAC_ODR_50HZ_HR_VAL },
		{ .hz = 100,	.value = ISM303DAC_ODR_100HZ_HR_VAL },
		{ .hz = 200,	.value = ISM303DAC_ODR_200HZ_HR_VAL },
		{ .hz = 400,	.value = ISM303DAC_ODR_400HZ_HR_VAL },
		{ .hz = 800,	.value = ISM303DAC_ODR_800HZ_HR_VAL },
	}
};

struct ism303dac_fs_reg {
	unsigned int gain[ISM303DAC_MODE_COUNT];
	u8 value;
	int urv;
};

static struct ism303dac_fs_table {
	u8 addr;
	u8 mask;
	struct ism303dac_fs_reg fs_avl[ISM303DAC_FS_LIST_NUM];
} ism303dac_fs_table = {
	.addr = ISM303DAC_FS_ADDR,
	.mask = ISM303DAC_FS_MASK,
	.fs_avl[0] = { .gain = {ISM303DAC_FS_2G_GAIN_LP, ISM303DAC_FS_2G_GAIN_HR,},
			.value = ISM303DAC_FS_2G_VAL,
			.urv = 2, },
	.fs_avl[1] = { .gain = {ISM303DAC_FS_4G_GAIN_LP, ISM303DAC_FS_4G_GAIN_HR,},
			.value = ISM303DAC_FS_4G_VAL,
			.urv = 4, },
	.fs_avl[2] = { .gain = {ISM303DAC_FS_8G_GAIN_LP, ISM303DAC_FS_8G_GAIN_LP,},
			.value = ISM303DAC_FS_8G_VAL,
			.urv = 8, },
	.fs_avl[3] = { .gain = {ISM303DAC_FS_16G_GAIN_LP, ISM303DAC_FS_16G_GAIN_HR,},
			.value = ISM303DAC_FS_16G_VAL,
			.urv = 16, },
};

static struct workqueue_struct *ism303dac_acc_wkq = 0;

static int ism303dac_acc_get_poll_data(struct st_sensor_data *sdata, u8 *data)
{
	int err = 0;
	u8 reg_addr;

	switch(sdata->sindex) {
		case ISM303DAC_ACCEL:
			reg_addr = ISM303DAC_OUTX_L_ADDR;

			break;
		default:
			dev_err(sdata->cdata->dev, "invalid polling mode for "
						"sensor %s\n", sdata->name);
			return -1;
	}

	err = sdata->cdata->tf->read(sdata->cdata, reg_addr,
					ISM303DAC_BYTE_FOR_SAMPLE,
					data);

	return err;
}

static inline s32 ism303dac_acc_data_align(u8 ms, u8 ls, u8 power_mode)
{
	if (power_mode == ISM303DAC_LP_MODE)
		return (s32)(((s16)(ls | ms << 8)) >> 6);
	else
		return (s32)(((s16)(ls | ms << 8)) >> 2);
}

static void ism303dac_acc_poll_wk(struct work_struct *input_work)
{
	struct st_sensor_data *sdata;
	int xyz[3] = { 0 };
	u8 pmode;
	u8 data[6];
	int err;

	sdata = container_of((struct work_struct *)input_work,
			     struct st_sensor_data, input_work);

	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

	pmode = GET_PDATA(sdata->cdata)->power_mode;
	err = ism303dac_acc_get_poll_data(sdata, data);
	if (err < 0)
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
			sdata->name, err);
	else {
		xyz[0] = ism303dac_acc_data_align(data[1], data[0], pmode);
		xyz[1] = ism303dac_acc_data_align(data[3], data[2], pmode);
		xyz[2] = ism303dac_acc_data_align(data[5], data[4], pmode);

		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;
		st_ism303dac_sensor_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
}

u8 ism303dac_event_irq1_value(struct st_common_data *cdata)
{
	u8 value = 0x0;
	priv_data_t *priv = GET_PDATA(cdata);

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_FF))
		value |= ISM303DAC_INT1_FREE_FALL_MASK;

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_DOUBLE_TAP))
		value |= ISM303DAC_INT1_TAP_MASK;

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_TAP))
		value |= ISM303DAC_INT1_S_TAP_MASK | ISM303DAC_INT1_TAP_MASK;

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_WAKEUP))
		value |= ISM303DAC_INT1_WAKEUP_MASK;
	
	return value;
}

int st_ism303dac_update_drdy_irq(struct st_sensor_data *sdata)
{
	u8 reg_addr, reg_val, reg_mask;
	
	switch (sdata->sindex) {
		case ISM303DAC_FF:
		case ISM303DAC_TAP:
		case ISM303DAC_DOUBLE_TAP:
		case ISM303DAC_WAKEUP:
		case ISM303DAC_ACTIVITY:
			reg_val = ism303dac_event_irq1_value(sdata->cdata);
			reg_addr = ISM303DAC_CTRL4_INT1_PAD_ADDR;
			reg_mask = ISM303DAC_INT1_EVENTS_MASK;
			
			break;
			
		case ISM303DAC_ACCEL:
			return 0;
			
			break;
			
		default:
			return -EINVAL;
	}
	
	return st_ism303dac_sensor_write_data(sdata->cdata, reg_addr, reg_mask, reg_val);
}

int ism303dac_set_fs(struct st_sensor_data *sdata, unsigned int fs)
{
	int err, i;
	u8 pmode = GET_PDATA(sdata->cdata)->power_mode;

	for (i = 0; i < ISM303DAC_FS_LIST_NUM; i++) {
		if (ism303dac_fs_table.fs_avl[i].urv == fs)
			break;
	}

	if (i == ISM303DAC_FS_LIST_NUM)
		return -EINVAL;

	err = st_ism303dac_sensor_write_data(sdata->cdata, ism303dac_fs_table.addr,
					ism303dac_fs_table.mask,
					ism303dac_fs_table.fs_avl[i].value);
	if (err < 0)
		return err;

	sdata->c_gain = ism303dac_fs_table.fs_avl[i].gain[pmode];

	return 0;
}

void ism303dac_event_management(struct st_common_data *cdata, u8 int_reg_val)
{
	priv_data_t *priv = GET_PDATA(cdata);

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_TAP) &&
				(int_reg_val & ISM303DAC_SINGLE_TAP_MASK)) {
		cdata->sensors[ISM303DAC_TAP].timestamp = priv->timestamp;
		st_ism303dac_sensor_report_single_event(&cdata->sensors[ISM303DAC_TAP], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_DOUBLE_TAP) &&
				(int_reg_val & ISM303DAC_DOUBLE_TAP_MASK)) {
		cdata->sensors[ISM303DAC_DOUBLE_TAP].timestamp = priv->timestamp;
		st_ism303dac_sensor_report_single_event(&cdata->sensors[ISM303DAC_DOUBLE_TAP], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_FF) &&
				(int_reg_val & ISM303DAC_FF_IA_MASK)) {
		cdata->sensors[ISM303DAC_FF].timestamp = priv->timestamp;
		st_ism303dac_sensor_report_single_event(&cdata->sensors[ISM303DAC_FF], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, ISM303DAC_WAKEUP) &&
				(int_reg_val & ISM303DAC_WAKE_UP_IA_MASK)) {
		cdata->sensors[ISM303DAC_WAKEUP].timestamp = priv->timestamp;
		st_ism303dac_sensor_report_single_event(&cdata->sensors[ISM303DAC_WAKEUP], 1);
	}
}

void ism303dac_irq_management(struct work_struct *irq_work)
{
	priv_data_t *priv;
	u8 status[4];
	struct st_common_data *cdata;

	priv = container_of((struct work_struct *)irq_work, priv_data_t,
								irq_work);

	cdata = priv->cdata;
	cdata->tf->read(cdata, ISM303DAC_STATUS_DUP_ADDR, 4, status);

	if (status[0] & ISM303DAC_EVENT_MASK)
		ism303dac_event_management(cdata, status[0]);

	enable_irq(cdata->irq);

	return;
}

int ism303dac_acc_write_odr(struct st_sensor_data *sdata) {
	int err, i;
	u32 max_odr = 0;
	priv_data_t *priv = GET_PDATA(sdata->cdata);
	u8 pmode = priv->power_mode;

	for (i = 0; i < ISM303DAC_SENSORS_NUMB; i++)
		if (CHECK_BIT(priv->enabled_sensor, i)) {
			if (sdata->cdata->sensors[i].c_odr > max_odr)
				max_odr = sdata->cdata->sensors[i].c_odr;
		}

	if (max_odr != priv->common_odr) {
		for (i = 0; i < ISM303DAC_ODR_LP_LIST_NUM; i++) {
			if (ism303dac_odr_table[pmode][i].hz >= max_odr)
				break;
		}
		if (i == ISM303DAC_ODR_LP_LIST_NUM)
			return -EINVAL;

		err = st_ism303dac_sensor_write_data(sdata->cdata,
					ISM303DAC_ODR_ADDR,
					ISM303DAC_ODR_MASK,
					ism303dac_odr_table[pmode][i].value);
		if (err < 0)
			return err;

		priv->common_odr = max_odr;
	}

	return 0;
}

int ism303dac_configure_tap_event(struct st_sensor_data *sdata, bool single_tap)
{
	u8 err = 0;

	if (single_tap) {
		err = st_ism303dac_sensor_write_data(sdata->cdata,
					ISM303DAC_INT_DUR_ADDR,
					ISM303DAC_INT_DUR_MASK,
					ISM303DAC_INT_DUR_STAP_DEFAULT);
		if (err < 0)
			return err;

		err = st_ism303dac_sensor_write_data(sdata->cdata,
					ISM303DAC_WAKE_UP_THS_ADDR,
					ISM303DAC_WAKE_UP_THS_S_D_TAP_MASK,
					ISM303DAC_DIS_BIT);
		if (err < 0)
			return err;
	} else {
		err = st_ism303dac_sensor_write_data(sdata->cdata,
					ISM303DAC_INT_DUR_ADDR,
					ISM303DAC_INT_DUR_MASK,
					ISM303DAC_INT_DUR_DTAP_DEFAULT);
		if (err < 0)
			return err;

		err = st_ism303dac_sensor_write_data(sdata->cdata,
					ISM303DAC_WAKE_UP_THS_ADDR,
					ISM303DAC_WAKE_UP_THS_S_D_TAP_MASK,
					ISM303DAC_EN_BIT);
		if (err < 0)
			return err;
	}

	return err;
}

int st_ism303dac_set_enable(struct st_sensor_data *sdata, bool state)
{
	int err = 0;
	priv_data_t *priv = GET_PDATA(sdata->cdata);

	if (sdata->enabled == state)
		return 0;

	if (state) {
		SET_BIT(priv->enabled_sensor, sdata->sindex);
	} else {
		RESET_BIT(priv->enabled_sensor, sdata->sindex);
	}

	switch (sdata->sindex) {
	case ISM303DAC_TAP:
		if (state && CHECK_BIT(priv->enabled_sensor,
				ISM303DAC_DOUBLE_TAP)) {
			err = -EINVAL;

			goto enable_sensor_error;
		}

		break;

	case ISM303DAC_DOUBLE_TAP:
		if (state && CHECK_BIT(priv->enabled_sensor,
				ISM303DAC_TAP)) {
			err = -EINVAL;

			goto enable_sensor_error;
		}

		break;

	case ISM303DAC_FF:
	case ISM303DAC_WAKEUP:
	case ISM303DAC_ACTIVITY:
		break;

	case ISM303DAC_ACCEL:
		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

		break;

	default:
		return -EINVAL;
	}

	err = st_ism303dac_update_drdy_irq(sdata);
	if (err < 0)
		goto enable_sensor_error;

	err = ism303dac_acc_write_odr(sdata);
	if (err < 0)
		goto enable_sensor_error;

	sdata->enabled = state;

	return 0;

enable_sensor_error:
	if (state) {
		RESET_BIT(priv->enabled_sensor, sdata->sindex);
	} else {
		SET_BIT(priv->enabled_sensor, sdata->sindex);
	}

	return err;
}

int ism303dac_acc_enable_sensors(struct st_sensor_data *sdata)
{
	return st_ism303dac_set_enable(sdata, true);
}

int ism303dac_acc_disable_sensors(struct st_sensor_data *sdata)
{
	return st_ism303dac_set_enable(sdata, false);
}

ssize_t ism303dac_acc_get_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < ISM303DAC_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			ism303dac_fs_table.fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

ssize_t ism303dac_acc_get_cur_scale(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	u8 pmode = GET_PDATA(sdata->cdata)->power_mode;

	for (i = 0; i < ISM303DAC_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
			ism303dac_fs_table.fs_avl[i].gain[pmode])
			break;

	return sprintf(buf, "%d\n", ism303dac_fs_table.fs_avl[i].urv);
}

ssize_t ism303dac_acc_set_cur_scale(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int urv, err;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	err = ism303dac_set_fs(sdata, urv);
	if (err < 0)
		return err;

	return count;
}

static ssize_t ism303dac_acc_get_resolution_mode(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	u8 pmode = GET_PDATA(sdata->cdata)->power_mode;

	return sprintf(buf, "%s\n", (pmode == ISM303DAC_LP_MODE) ? "low" : "high");
}

static ssize_t ism303dac_acc_set_resolution_mode(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err, i;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	u8 *pmode = &(GET_PDATA(sdata->cdata)->power_mode);

	for (i = 0; i < ISM303DAC_FS_LIST_NUM; i++) {
		if (sdata->c_gain ==
			ism303dac_fs_table.fs_avl[i].gain[*pmode])
			break;
	}

	if (!strncmp(buf, "low", count - 1))
		*pmode = ISM303DAC_LP_MODE;
	else if (!strncmp(buf, "high", count - 1))
		*pmode = ISM303DAC_HR_MODE;
	else
		return -EINVAL;

	err = ism303dac_acc_write_odr(sdata);
	if (err < 0)
		return err;

	sdata->c_gain = ism303dac_fs_table.fs_avl[i].gain[*pmode];

	return count;
}

int ism303dac_init_sensors(struct st_common_data *cdata)
{
	int err, i;
	struct st_sensor_data *sdata;
	
	for (i = 0; i < ISM303DAC_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		
		err = ism303dac_acc_disable_sensors(sdata);
		if (err < 0)
			return err;
		
		if (sdata->sindex == ISM303DAC_ACCEL) {
			err = ism303dac_set_fs(sdata, ISM303DAC_ACCEL_FS);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[ISM303DAC_ACCEL].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	cdata->sensors[ISM303DAC_ACCEL].hr_timer.function =
						&st_ism303dac_sensor_poll_function;

	/*
	 * Soft reset the device on power on.
	 */
	err = st_ism303dac_sensor_write_data(cdata, ISM303DAC_SOFT_RESET_ADDR,
						ISM303DAC_SOFT_RESET_MASK,
						ISM303DAC_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Enable latched interrupt mode.
	 */
	err = st_ism303dac_sensor_write_data(cdata, ISM303DAC_LIR_ADDR,
						ISM303DAC_LIR_MASK,
						ISM303DAC_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Enable block data update feature.
	 */
	err = st_ism303dac_sensor_write_data(cdata, ISM303DAC_BDU_ADDR,
						ISM303DAC_BDU_MASK,
						ISM303DAC_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Route interrupt from INT2 to INT1 pin.
	 */
	err = st_ism303dac_sensor_write_data(cdata, ISM303DAC_INT2_ON_INT1_ADDR,
						ISM303DAC_INT2_ON_INT1_MASK,
						ISM303DAC_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Configure default free fall event threshold.
	 */
	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_FREE_FALL_ADDR,
						ISM303DAC_FREE_FALL_THS_MASK,
						ISM303DAC_FREE_FALL_THS_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure default free fall event duration.
	 */
	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_FREE_FALL_ADDR,
						ISM303DAC_FREE_FALL_DUR_MASK,
						ISM303DAC_FREE_FALL_DUR_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure Tap event recognition on all direction (X, Y and Z axes).
	 */

	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_TAP_AXIS_ADDR,
						ISM303DAC_TAP_AXIS_MASK,
						ISM303DAC_TAP_AXIS_ANABLE_ALL);
	if (err < 0)
		return err;

	/*
	 * Configure default threshold for Tap event recognition.
	 */
	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_TAP_THS_ADDR,
						ISM303DAC_TAP_THS_MASK,
						ISM303DAC_TAP_THS_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure default threshold for Wake Up event recognition.
	 */
	err = st_ism303dac_sensor_write_data(sdata->cdata, ISM303DAC_WAKE_UP_THS_ADDR,
						ISM303DAC_WAKE_UP_THS_WU_MASK,
						ISM303DAC_WAKE_UP_THS_WU_DEFAULT);
	if (err < 0)
		return err;

	return 0;
}

static irqreturn_t ism303dac_save_tstamp(int irq, void *private)
{
	struct st_common_data *cdata = private;

	GET_PDATA(cdata)->timestamp = st_sensor_get_time_ns();
	queue_work(ism303dac_acc_wkq, &(GET_PDATA(cdata)->irq_work));
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

int ism303dac_allocate_workqueue(struct st_common_data *cdata)
{
	int err;

	if (!ism303dac_acc_wkq)
		ism303dac_acc_wkq = create_workqueue(cdata->name);

	if (!ism303dac_acc_wkq)
		return -EINVAL;

	INIT_WORK(&(GET_PDATA(cdata)->irq_work), ism303dac_irq_management);

	err = request_threaded_irq(cdata->irq, ism303dac_save_tstamp, NULL,
				   IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		return err;

	return 0;
}

ADD_DEVICE_ENABLE_ATTR;
ADD_DEVICE_POLLING_ATTR;

static DEVICE_ATTR(resolution, S_IWUSR | S_IRUGO,
				ism303dac_acc_get_resolution_mode,
				ism303dac_acc_set_resolution_mode);

static DEVICE_ATTR(scale_avail, S_IRUGO,
				ism303dac_acc_get_scale_avail,
				NULL);

static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO,
				ism303dac_acc_get_cur_scale,
				ism303dac_acc_set_cur_scale);

static struct attribute *ism303dac_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_resolution.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *ism303dac_ff_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ism303dac_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ism303dac_double_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ism303dac_wakeup_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ism303dac_act_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group ism303dac_attribute_groups[] = {
	[ISM303DAC_ACCEL] = {
		.attrs = ism303dac_accel_attribute,
		.name = "accel",
	},
	[ISM303DAC_FF] = {
		.attrs = ism303dac_ff_attribute,
		.name = "free_fall",
	},
	[ISM303DAC_TAP] = {
		.name = "tap",
		.attrs = ism303dac_tap_attribute,
	},
	[ISM303DAC_DOUBLE_TAP] = {
		.name = "double_tap",
		.attrs = ism303dac_double_tap_attribute,
	},
	[ISM303DAC_WAKEUP] = {
		.name = "wake_up",
		.attrs = ism303dac_wakeup_attribute,
	},
	[ISM303DAC_ACTIVITY] = {
		.name = "act",
		.attrs = ism303dac_act_attribute,
	},
};

int ism303dac_sensors_data_init(struct st_common_data *cdata)
{
	int32_t i;
	bool is_3_axis;
	struct st_sensor_data *sdata;

	GET_PDATA(cdata)->common_odr = 0;
	GET_PDATA(cdata)->enabled_sensor = 0;
	GET_PDATA(cdata)->power_mode = ISM303DAC_PMODE_DEFAULT;
	GET_PDATA(cdata)->cdata = cdata;
	cdata->sensors_count = ISM303DAC_SENSORS_NUMB;

	for (i = 0; i < ISM303DAC_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = ism303dac_sensors_table[i].name;
		sdata->c_odr = ism303dac_sensors_table[i].min_odr_hz;
		sdata->write_odr = ism303dac_acc_write_odr;
		sdata->enable = ism303dac_acc_enable_sensors;
		sdata->disable = ism303dac_acc_disable_sensors;

		switch(i) {
			case ISM303DAC_ACCEL:
				sdata->ktime = ktime_set(0, MS_TO_NS(1000 /
								sdata->c_odr));
				is_3_axis = true;

				break;

			default:
				is_3_axis = false;

				break;
		}

		st_ism303dac_sensor_input_init(sdata, kasprintf(GFP_KERNEL, "%s_%s",
						cdata->name, sdata->name),
						is_3_axis);

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
						&ism303dac_attribute_groups[i]))
			goto error_ism303dac_sensors_data_init;
	}

	return 0;

error_ism303dac_sensors_data_init:
	dev_err(cdata->dev, "failed to create sysfs group for sensor %s",
							sdata->name);
	for (; i >= 0; i--) {
		sysfs_remove_group(&sdata->input_dev->dev.kobj,
						&ism303dac_attribute_groups[i]);
		input_unregister_device(cdata->sensors[i].input_dev);
	}

	return -EINVAL;
}

int ism303dac_acc_probe(struct st_common_data *cdata)
{
	int err;

	err = st_ism303dac_sensor_common_probe(cdata, cdata->irq);
	if (err < 0)
		return err;

	err = ism303dac_sensors_data_init(cdata);
	if (err < 0)
		return err;

	err = ism303dac_init_sensors(cdata);
	if (err < 0)
		return err;

	INIT_WORK(&cdata->sensors[ISM303DAC_ACCEL].input_work, ism303dac_acc_poll_wk);

	if (cdata->irq > 0) {
		err = ism303dac_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}
	return 0;
}
EXPORT_SYMBOL(ism303dac_acc_probe);

int ism303dac_acc_remove(struct st_common_data *cdata)
{
	u8 i;

	for (i = 0; i < cdata->sensors_count; i++) {
		ism303dac_acc_disable_sensors(&cdata->sensors[i]);
		st_ism303dac_sensor_input_cleanup(&cdata->sensors[i]);
	}
	st_ism303dac_sensor_common_remove(cdata);

	return 0;
}
EXPORT_SYMBOL(ism303dac_acc_remove);

int ism303dac_acc_enable(struct st_common_data *cdata)
{
	int i, err;

	for (i = 0; i < cdata->sensors_count; i++) {
		err = ism303dac_acc_enable_sensors(&cdata->sensors[i]);
		if (err < 0)
			return err;
	}
	return 0;
}
EXPORT_SYMBOL(ism303dac_acc_enable);

int ism303dac_acc_disable(struct st_common_data *cdata)
{
	int i, err;

	for (i = 0; i < cdata->sensors_count; i++) {
		err = ism303dac_acc_disable_sensors(&cdata->sensors[i]);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(ism303dac_acc_disable);

MODULE_DESCRIPTION("STMicroelectronics ism303dac_acc driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_LICENSE("GPL v2");
