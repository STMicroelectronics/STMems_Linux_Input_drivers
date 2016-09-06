/*
 * STMicroelectronics lsm303ah_acc.c driver
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
#include "lsm303ah_core.h"

#define LSM303AH_CTRL1_ADDR			0x20
#define LSM303AH_CTRL2_ADDR			0x21
#define LSM303AH_CTRL3_ADDR			0x22
#define LSM303AH_CTRL4_INT1_PAD_ADDR		0x23
#define LSM303AH_CTRL5_INT2_PAD_ADDR		0x24
#define LSM303AH_STATUS_ADDR			0x27
#define LSM303AH_OUTX_L_ADDR			0x28
#define LSM303AH_TAP_THS_6D_ADDR		0x31
#define LSM303AH_INT_DUR_ADDR			0x32
#define LSM303AH_WAKE_UP_THS_ADDR		0x33
#define LSM303AH_WAKE_UP_DUR_ADDR		0x34
#define LSM303AH_FREE_FALL_ADDR			0x35
#define LSM303AH_STATUS_DUP_ADDR		0x36
#define LSM303AH_WAKE_UP_SRC_ADDR		0x37
#define LSM303AH_TAP_SRC_ADDR			0x38
#define LSM303AH_6D_SRC_ADDR			0x39
#define LSM303AH_STEP_C_MINTHS_ADDR		0x3a
#define LSM303AH_STEP_C_MINTHS_RST_NSTEP_MASK	0x80
#define LSM303AH_STEP_C_OUT_L_ADDR		0x3b
#define LSM303AH_STEP_C_OUT_SIZE		2

#define LSM303AH_F_CK_GATE_ADDR			0x3d
#define LSM303AH_F_CK_GATE_TILT_INT_MASK	0x80
#define LSM303AH_F_CK_GATE_SIGN_M_DET_MASK	0x10
#define LSM303AH_F_CK_GATE_RST_SIGN_M_MASK	0x08
#define LSM303AH_F_CK_GATE_RST_PEDO_MASK	0x04
#define LSM303AH_F_CK_GATE_STEP_D_MASK		0x02
#define LSM303AH_F_CK_GATE_MASK			(LSM303AH_F_CK_GATE_TILT_INT_MASK | \
						LSM303AH_F_CK_GATE_SIGN_M_DET_MASK | \
						LSM303AH_F_CK_GATE_STEP_D_MASK)

#define LSM303AH_F_CTRL_ADDR			0x3f
#define LSM303AH_F_CTRL_TILT_MASK		0x10
#define LSM303AH_F_CTRL_SIGN_MOT_MASK		0x02
#define LSM303AH_F_CTRL_STEP_CNT_MASK		0x01
#define LSM303AH_F_CTRL_EV_MASK			(LSM303AH_F_CTRL_TILT_MASK | \
						LSM303AH_F_CTRL_SIGN_MOT_MASK | \
						LSM303AH_F_CTRL_STEP_CNT_MASK)

#define LSM303AH_INT_STATUS_ADDR		LSM303AH_STATUS_ADDR
#define LSM303AH_WAKE_UP_IA_MASK		0x40
#define LSM303AH_DOUBLE_TAP_MASK		0x10
#define LSM303AH_SINGLE_TAP_MASK		0x08
#define LSM303AH_6D_IA_MASK			0x04
#define LSM303AH_FF_IA_MASK			0x02
#define LSM303AH_DRDY_MASK			0x01
#define LSM303AH_EVENT_MASK			(LSM303AH_WAKE_UP_IA_MASK | \
						LSM303AH_DOUBLE_TAP_MASK | \
						LSM303AH_SINGLE_TAP_MASK | \
						LSM303AH_6D_IA_MASK | \
						LSM303AH_FF_IA_MASK)

#define LSM303AH_ODR_ADDR			LSM303AH_CTRL1_ADDR
#define LSM303AH_ODR_MASK			0xf0
#define LSM303AH_ODR_POWER_OFF_VAL		0x00
#define LSM303AH_ODR_1HZ_LP_VAL			0x08
#define LSM303AH_ODR_12HZ_LP_VAL		0x09
#define LSM303AH_ODR_25HZ_LP_VAL		0x0a
#define LSM303AH_ODR_50HZ_LP_VAL		0x0b
#define LSM303AH_ODR_100HZ_LP_VAL		0x0c
#define LSM303AH_ODR_200HZ_LP_VAL		0x0d
#define LSM303AH_ODR_400HZ_LP_VAL		0x0e
#define LSM303AH_ODR_800HZ_LP_VAL		0x0f
#define LSM303AH_ODR_LP_LIST_NUM		9

#define LSM303AH_ODR_12_5HZ_HR_VAL		0x01
#define LSM303AH_ODR_25HZ_HR_VAL		0x02
#define LSM303AH_ODR_50HZ_HR_VAL		0x03
#define LSM303AH_ODR_100HZ_HR_VAL		0x04
#define LSM303AH_ODR_200HZ_HR_VAL		0x05
#define LSM303AH_ODR_400HZ_HR_VAL		0x06
#define LSM303AH_ODR_800HZ_HR_VAL		0x07
#define LSM303AH_ODR_HR_LIST_NUM		8

#define LSM303AH_FS_ADDR			LSM303AH_CTRL1_ADDR
#define LSM303AH_FS_MASK			0x0c
#define LSM303AH_FS_2G_VAL			0x00
#define LSM303AH_FS_4G_VAL			0x02
#define LSM303AH_FS_8G_VAL			0x03
#define LSM303AH_FS_16G_VAL			0x01

/*
 * Sensitivity sets in LP mode [ug]
 */
#define LSM303AH_FS_2G_GAIN_LP			3906
#define LSM303AH_FS_4G_GAIN_LP			7813
#define LSM303AH_FS_8G_GAIN_LP			15625
#define LSM303AH_FS_16G_GAIN_LP			31250

/*
 * Sensitivity sets in HR mode [ug]
 */
#define LSM303AH_FS_2G_GAIN_HR			244
#define LSM303AH_FS_4G_GAIN_HR			488
#define LSM303AH_FS_8G_GAIN_HR			976
#define LSM303AH_FS_16G_GAIN_HR			1952

#define LSM303AH_FS_LIST_NUM			4
enum {
	LSM303AH_LP_MODE = 0,
	LSM303AH_HR_MODE,
	LSM303AH_MODE_COUNT,
};
#define LSM303AH_PMODE_DEFAULT			LSM303AH_LP_MODE

#define LSM303AH_INT1_S_TAP_MASK		0x40
#define LSM303AH_INT1_WAKEUP_MASK		0x20
#define LSM303AH_INT1_FREE_FALL_MASK		0x10
#define LSM303AH_INT1_TAP_MASK			0x08
#define LSM303AH_INT1_6D_MASK			0x04
#define LSM303AH_INT1_FTH_MASK			0x02
#define LSM303AH_INT1_DRDY_MASK			0x01
#define LSM303AH_INT1_EVENTS_MASK		0x7f

#define LSM303AH_INT2_ON_INT1_MASK		0x20
#define LSM303AH_INT2_TILT_MASK			0x04
#define LSM303AH_INT2_SIG_MOT_DET_MASK		0x02
#define LSM303AH_INT2_STEP_DET_MASK		0x01
#define LSM303AH_INT2_EVENTS_MASK		0x1c

#define LSM303AH_INT_DUR_SHOCK_MASK		0x03
#define LSM303AH_INT_DUR_QUIET_MASK		0x0c
#define LSM303AH_INT_DUR_LAT_MASK		0xf0
#define LSM303AH_INT_DUR_MASK			(LSM303AH_INT_DUR_SHOCK_MASK | \
						LSM303AH_INT_DUR_QUIET_MASK | \
						LSM303AH_INT_DUR_LAT_MASK)
#define LSM303AH_INT_DUR_STAP_DEFAULT		0x06
#define LSM303AH_INT_DUR_DTAP_DEFAULT		0x7f
#define LSM303AH_WAKE_UP_THS_S_D_TAP_MASK	0x80
#define LSM303AH_WAKE_UP_THS_SLEEP_MASK		0x40
#define LSM303AH_WAKE_UP_THS_WU_MASK		0x3f
#define LSM303AH_WAKE_UP_THS_WU_DEFAULT		0x02
#define LSM303AH_FREE_FALL_THS_MASK		0x07
#define LSM303AH_FREE_FALL_DUR_MASK		0xF8
#define LSM303AH_FREE_FALL_THS_DEFAULT		0x01
#define LSM303AH_FREE_FALL_DUR_DEFAULT		0x01
#define LSM303AH_BDU_ADDR			LSM303AH_CTRL1_ADDR
#define LSM303AH_BDU_MASK			0x01
#define LSM303AH_SOFT_RESET_ADDR		LSM303AH_CTRL2_ADDR
#define LSM303AH_SOFT_RESET_MASK		0x40
#define LSM303AH_LIR_ADDR			LSM303AH_CTRL3_ADDR
#define LSM303AH_LIR_MASK			0x04
#define LSM303AH_TAP_AXIS_ADDR			LSM303AH_CTRL3_ADDR
#define LSM303AH_TAP_AXIS_MASK			0x38
#define LSM303AH_TAP_AXIS_ANABLE_ALL		0x07

#define LSM303AH_TAP_THS_ADDR			LSM303AH_TAP_THS_6D_ADDR
#define LSM303AH_TAP_THS_MASK			0x1f
#define LSM303AH_TAP_THS_DEFAULT		0x09

#define LSM303AH_INT2_ON_INT1_ADDR		LSM303AH_CTRL5_INT2_PAD_ADDR
#define LSM303AH_INT2_ON_INT1_MASK		0x20

#define LSM303AH_ACCEL_ODR			1
#define LSM303AH_ACCEL_FS			2
#define LSM303AH_FF_ODR				25
#define LSM303AH_STEP_D_ODR			25
#define LSM303AH_TILT_ODR			25
#define LSM303AH_SIGN_M_ODR			25
#define LSM303AH_TAP_ODR			400
#define LSM303AH_WAKEUP_ODR			25
#define LSM303AH_ACTIVITY_ODR			12
#define LSM303AH_MIN_EVENT_ODR			25

#define LSM303AH_BYTE_FOR_SAMPLE		6
#define SET_BIT(a, b)				{a |= (1 << b);}
#define RESET_BIT(a, b)				{a &= ~(1 << b);}
#define CHECK_BIT(a, b)				(a & (1 << b))

#define LSM303AH_NAME				"lsm303ah_acc"

#define GET_PDATA(cdata)	((priv_data_t *)(cdata->priv_data))

static const struct lsm303ah_sensors_table {
	const char *name;
	const char *description;
	const u32 min_odr_hz;
} lsm303ah_sensors_table[LSM303AH_SENSORS_NUMB] = {
	[LSM303AH_ACCEL] = {
		.name = "accel",
		.description = "ST LSM303AH Accelerometer Sensor",
		.min_odr_hz = LSM303AH_ACCEL_ODR,
	},
	[LSM303AH_STEP_C] = {
		.name = "step_c",
		.description = "ST LSM303AH Step Counter Sensor",
		.min_odr_hz = LSM303AH_STEP_D_ODR,
	},
	[LSM303AH_FF] = {
		.name = "free_fall",
		.description = "ST LSM303AH Free Fall Sensor",
		.min_odr_hz = LSM303AH_FF_ODR,
	},
	[LSM303AH_TAP] = {
		.name = "tap",
		.description = "ST LSM303AH Tap Sensor",
		.min_odr_hz = LSM303AH_TAP_ODR,
	},
	[LSM303AH_DOUBLE_TAP] = {
		.name = "double_tap",
		.description = "ST LSM303AH Double Tap Sensor",
		.min_odr_hz = LSM303AH_TAP_ODR,
	},
	[LSM303AH_STEP_D] = {
		.name = "step_d",
		.description = "ST LSM303AH Step Detector Sensor",
		.min_odr_hz = LSM303AH_STEP_D_ODR,
	},
	[LSM303AH_TILT] = {
		.name = "tilt",
		.description = "ST LSM303AH Tilt Sensor",
		.min_odr_hz = LSM303AH_TILT_ODR,
	},
	[LSM303AH_SIGN_M] = {
		.name = "sign_m",
		.description = "ST LSM303AH Significant Motion Sensor",
		.min_odr_hz = LSM303AH_SIGN_M_ODR,
	},
	[LSM303AH_WAKEUP] = {
		.name = "wake_up",
		.description = "ST LSM303AH Free Fall Sensor",
		.min_odr_hz = LSM303AH_WAKEUP_ODR,
	},
	[LSM303AH_ACTIVITY] = {
		.name = "act",
		.description = "ST LSM303AH Activity Sensor",
		.min_odr_hz = LSM303AH_ACTIVITY_ODR,
	},
};

static const struct lsm303ah_odr_reg {
	u32 hz;
	u8 value;
} lsm303ah_odr_table[LSM303AH_MODE_COUNT][LSM303AH_ODR_LP_LIST_NUM] = {
	[LSM303AH_LP_MODE] = {
		{ .hz = 0,	.value = LSM303AH_ODR_POWER_OFF_VAL, },
		{ .hz = 1,	.value = LSM303AH_ODR_1HZ_LP_VAL, },
		{ .hz = 12,	.value = LSM303AH_ODR_12HZ_LP_VAL, },
		{ .hz = 25,	.value = LSM303AH_ODR_25HZ_LP_VAL, },
		{ .hz = 50,	.value = LSM303AH_ODR_50HZ_LP_VAL, },
		{ .hz = 100,	.value = LSM303AH_ODR_100HZ_LP_VAL, },
		{ .hz = 200,	.value = LSM303AH_ODR_200HZ_LP_VAL, },
		{ .hz = 400,	.value = LSM303AH_ODR_400HZ_LP_VAL, },
		{ .hz = 800,	.value = LSM303AH_ODR_800HZ_LP_VAL, },
	},
	[LSM303AH_HR_MODE] = {
		{ .hz = 0,	.value = LSM303AH_ODR_POWER_OFF_VAL, },
		{ .hz = 12,	.value = LSM303AH_ODR_12_5HZ_HR_VAL },
		{ .hz = 25,	.value = LSM303AH_ODR_25HZ_HR_VAL },
		{ .hz = 50,	.value = LSM303AH_ODR_50HZ_HR_VAL },
		{ .hz = 100,	.value = LSM303AH_ODR_100HZ_HR_VAL },
		{ .hz = 200,	.value = LSM303AH_ODR_200HZ_HR_VAL },
		{ .hz = 400,	.value = LSM303AH_ODR_400HZ_HR_VAL },
		{ .hz = 800,	.value = LSM303AH_ODR_800HZ_HR_VAL },
	}
};

struct lsm303ah_fs_reg {
	unsigned int gain[LSM303AH_MODE_COUNT];
	u8 value;
	int urv;
};

static struct lsm303ah_fs_table {
	u8 addr;
	u8 mask;
	struct lsm303ah_fs_reg fs_avl[LSM303AH_FS_LIST_NUM];
} lsm303ah_fs_table = {
	.addr = LSM303AH_FS_ADDR,
	.mask = LSM303AH_FS_MASK,
	.fs_avl[0] = { .gain = {LSM303AH_FS_2G_GAIN_LP, LSM303AH_FS_2G_GAIN_HR,},
			.value = LSM303AH_FS_2G_VAL,
			.urv = 2, },
	.fs_avl[1] = { .gain = {LSM303AH_FS_4G_GAIN_LP, LSM303AH_FS_4G_GAIN_HR,},
			.value = LSM303AH_FS_4G_VAL,
			.urv = 4, },
	.fs_avl[2] = { .gain = {LSM303AH_FS_8G_GAIN_LP, LSM303AH_FS_8G_GAIN_LP,},
			.value = LSM303AH_FS_8G_VAL,
			.urv = 8, },
	.fs_avl[3] = { .gain = {LSM303AH_FS_16G_GAIN_LP, LSM303AH_FS_16G_GAIN_HR,},
			.value = LSM303AH_FS_16G_VAL,
			.urv = 16, },
};

static struct workqueue_struct *lsm303ah_acc_wkq = 0;

static int lsm303ah_acc_get_poll_data(struct st_sensor_data *sdata, u8 *data)
{
	int err = 0;
	u8 reg_addr;

	switch(sdata->sindex) {
		case LSM303AH_ACCEL:
			reg_addr = LSM303AH_OUTX_L_ADDR;

			break;
		default:
			dev_err(sdata->cdata->dev, "invalid polling mode for "
						"sensor %s\n", sdata->name);
			return -1;
	}

	err = sdata->cdata->tf->read(sdata->cdata, reg_addr,
					LSM303AH_BYTE_FOR_SAMPLE,
					data);

	return err;
}

static int lsm303ah_report_step_c_data(struct st_common_data *cdata)
{
	int err;
	s32 steps;
	u8 data[2];

	err = cdata->tf->read(cdata, LSM303AH_STEP_C_OUT_L_ADDR,
						LSM303AH_STEP_C_OUT_SIZE, data);
	if (err < 0)
		return err;

	steps = (s32)((u16)(data[1] << 8) | data[0]);
	st_sensor_report_single_event(&cdata->sensors[LSM303AH_STEP_C], steps);

	return 0;
}

static inline s32 lsm303ah_acc_data_align(u8 ms, u8 ls, u8 power_mode)
{
	if (power_mode == LSM303AH_LP_MODE)
		return (s32)(((s16)(ls | ms << 8)) >> 6);
	else
		return (s32)(((s16)(ls | ms << 8)) >> 2);
}

static void lsm303ah_acc_poll_wk(struct work_struct *input_work)
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
	err = lsm303ah_acc_get_poll_data(sdata, data);
	if (err < 0)
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
			sdata->name, err);
	else {
		xyz[0] = lsm303ah_acc_data_align(data[1], data[0], pmode);
		xyz[1] = lsm303ah_acc_data_align(data[3], data[2], pmode);
		xyz[2] = lsm303ah_acc_data_align(data[5], data[4], pmode);

		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;
		st_sensor_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
}

u8 lsm303ah_event_irq1_value(struct st_common_data *cdata)
{
	u8 value = 0x0;
	priv_data_t *priv = GET_PDATA(cdata);

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_FF))
		value |= LSM303AH_INT1_FREE_FALL_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_DOUBLE_TAP))
		value |= LSM303AH_INT1_TAP_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_TAP))
		value |= LSM303AH_INT1_S_TAP_MASK | LSM303AH_INT1_TAP_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_WAKEUP))
		value |= LSM303AH_INT1_WAKEUP_MASK;
	
	return value;
}

u8 lsm303ah_event_irq2_value(struct st_common_data *cdata)
{
	u8 value = 0x0;
	priv_data_t *priv = GET_PDATA(cdata);

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_TILT))
		value |= LSM303AH_INT2_TILT_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_SIGN_M))
		value |= LSM303AH_INT2_SIG_MOT_DET_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_STEP_C) ||
			CHECK_BIT(priv->enabled_sensor, LSM303AH_STEP_D))
		value |= LSM303AH_INT2_STEP_DET_MASK;
	
	return value;
}

int lsm303ah_acc_set_enable_function(struct st_common_data *cdata, bool state,
				 u8 func_bit_mask)
{
	int err = 0;
	
	err = st_sensor_write_data(cdata, LSM303AH_F_CTRL_ADDR,
						func_bit_mask, state);
	if (err < 0)
		return err;
	
	return 0;
}

int lsm303ah_update_drdy_irq(struct st_sensor_data *sdata)
{
	u8 reg_addr, reg_val, reg_mask;
	
	switch (sdata->sindex) {
		case LSM303AH_FF:
		case LSM303AH_TAP:
		case LSM303AH_DOUBLE_TAP:
		case LSM303AH_WAKEUP:
		case LSM303AH_ACTIVITY:
			reg_val = lsm303ah_event_irq1_value(sdata->cdata);
			reg_addr = LSM303AH_CTRL4_INT1_PAD_ADDR;
			reg_mask = LSM303AH_INT1_EVENTS_MASK;
			
			break;
			
		case LSM303AH_SIGN_M:
		case LSM303AH_TILT:
		case LSM303AH_STEP_D:
		case LSM303AH_STEP_C:
			reg_val = lsm303ah_event_irq2_value(sdata->cdata);
			reg_addr = LSM303AH_CTRL5_INT2_PAD_ADDR;
			reg_mask = LSM303AH_INT2_EVENTS_MASK;
			
			break;
			
		case LSM303AH_ACCEL:
			return 0;
			
			break;
			
		default:
			return -EINVAL;
	}
	
	return st_sensor_write_data(sdata->cdata, reg_addr, reg_mask, reg_val);
}

int lsm303ah_set_fs(struct st_sensor_data *sdata, unsigned int fs)
{
	int err, i;
	u8 pmode = GET_PDATA(sdata->cdata)->power_mode;

	for (i = 0; i < LSM303AH_FS_LIST_NUM; i++) {
		if (lsm303ah_fs_table.fs_avl[i].urv == fs)
			break;
	}

	if (i == LSM303AH_FS_LIST_NUM)
		return -EINVAL;

	err = st_sensor_write_data(sdata->cdata, lsm303ah_fs_table.addr,
					lsm303ah_fs_table.mask,
					lsm303ah_fs_table.fs_avl[i].value);
	if (err < 0)
		return err;

	sdata->c_gain = lsm303ah_fs_table.fs_avl[i].gain[pmode];

	return 0;
}

void lsm303ah_event_management(struct st_common_data *cdata, u8 int_reg_val,
								u8 ck_gate_val)
{
	priv_data_t *priv = GET_PDATA(cdata);

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_TAP) &&
				(int_reg_val & LSM303AH_SINGLE_TAP_MASK)) {
		cdata->sensors[LSM303AH_TAP].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_TAP], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_DOUBLE_TAP) &&
				(int_reg_val & LSM303AH_DOUBLE_TAP_MASK)) {
		cdata->sensors[LSM303AH_DOUBLE_TAP].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_DOUBLE_TAP], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_FF) &&
				(int_reg_val & LSM303AH_FF_IA_MASK)) {
		cdata->sensors[LSM303AH_FF].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_FF], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_WAKEUP) &&
				(int_reg_val & LSM303AH_WAKE_UP_IA_MASK)) {
		cdata->sensors[LSM303AH_WAKEUP].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_WAKEUP], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_STEP_D) &&
				(ck_gate_val & LSM303AH_F_CK_GATE_STEP_D_MASK)) {
		cdata->sensors[LSM303AH_STEP_D].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_STEP_D], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_TILT) &&
				(ck_gate_val & LSM303AH_F_CK_GATE_TILT_INT_MASK)) {
		cdata->sensors[LSM303AH_TILT].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_TILT], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_SIGN_M) &&
			(ck_gate_val & LSM303AH_F_CK_GATE_SIGN_M_DET_MASK)) {
		cdata->sensors[LSM303AH_SIGN_M].timestamp = priv->timestamp;
		st_sensor_report_single_event(&cdata->sensors[LSM303AH_SIGN_M], 1);
	}

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_STEP_C) &&
			(ck_gate_val & LSM303AH_F_CK_GATE_STEP_D_MASK)) {
		cdata->sensors[LSM303AH_STEP_C].timestamp = priv->timestamp;
		lsm303ah_report_step_c_data(cdata);
	}
}

void lsm303ah_irq_management(struct work_struct *irq_work)
{
	priv_data_t *priv;
	u8 status[4], func[2];
	struct st_common_data *cdata;

	priv = container_of((struct work_struct *)irq_work, priv_data_t,
								irq_work);

	cdata = priv->cdata;
	cdata->tf->read(cdata, LSM303AH_STATUS_DUP_ADDR, 4, status);
	cdata->tf->read(cdata, LSM303AH_F_CK_GATE_ADDR, 2, func);

	if ((status[0] & LSM303AH_EVENT_MASK) ||
	    (func[0] & LSM303AH_F_CK_GATE_MASK))
		/*
		 * Detected an event! Decode and report it.
		 */
		lsm303ah_event_management(cdata, status[0], func[0]);

	enable_irq(cdata->irq);

	return;
}

int lsm303ah_acc_write_odr(struct st_sensor_data *sdata) {
	int err, i;
	u32 max_odr = 0;
	priv_data_t *priv = GET_PDATA(sdata->cdata);
	u8 pmode = priv->power_mode;

	for (i = 0; i < LSM303AH_SENSORS_NUMB; i++)
		if (CHECK_BIT(priv->enabled_sensor, i)) {
			if (sdata->cdata->sensors[i].c_odr > max_odr)
				max_odr = sdata->cdata->sensors[i].c_odr;
		}

	if (max_odr != priv->common_odr) {
		for (i = 0; i < LSM303AH_ODR_LP_LIST_NUM; i++) {
			if (lsm303ah_odr_table[pmode][i].hz >= max_odr)
				break;
		}
		if (i == LSM303AH_ODR_LP_LIST_NUM)
			return -EINVAL;

		err = st_sensor_write_data(sdata->cdata,
					LSM303AH_ODR_ADDR,
					LSM303AH_ODR_MASK,
					lsm303ah_odr_table[pmode][i].value);
		if (err < 0)
			return err;

		priv->common_odr = max_odr;
	}

	return 0;
}

int lsm303ah_configure_tap_event(struct st_sensor_data *sdata, bool single_tap)
{
	u8 err = 0;

	if (single_tap) {
		err = st_sensor_write_data(sdata->cdata,
					LSM303AH_INT_DUR_ADDR,
					LSM303AH_INT_DUR_MASK,
					LSM303AH_INT_DUR_STAP_DEFAULT);
		if (err < 0)
			return err;

		err = st_sensor_write_data(sdata->cdata,
					LSM303AH_WAKE_UP_THS_ADDR,
					LSM303AH_WAKE_UP_THS_S_D_TAP_MASK,
					LSM303AH_DIS_BIT);
		if (err < 0)
			return err;
	} else {
		err = st_sensor_write_data(sdata->cdata,
					LSM303AH_INT_DUR_ADDR,
					LSM303AH_INT_DUR_MASK,
					LSM303AH_INT_DUR_DTAP_DEFAULT);
		if (err < 0)
			return err;

		err = st_sensor_write_data(sdata->cdata,
					LSM303AH_WAKE_UP_THS_ADDR,
					LSM303AH_WAKE_UP_THS_S_D_TAP_MASK,
					LSM303AH_EN_BIT);
		if (err < 0)
			return err;
	}

	return err;
}

int lsm303ah_update_event_functions(struct st_common_data *cdata)
{
	u8 reg_val = 0;
	priv_data_t *priv = GET_PDATA(cdata);

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_SIGN_M))
		reg_val |= LSM303AH_F_CTRL_SIGN_MOT_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_TILT))
		reg_val |= LSM303AH_F_CTRL_TILT_MASK;

	if (CHECK_BIT(priv->enabled_sensor, LSM303AH_STEP_D) ||
			CHECK_BIT(priv->enabled_sensor, LSM303AH_STEP_C))
		reg_val |= LSM303AH_F_CTRL_STEP_CNT_MASK;

	return st_sensor_write_data(cdata, LSM303AH_F_CTRL_ADDR,
						LSM303AH_F_CTRL_EV_MASK,
						LSM303AH_F_CTRL_EV_MASK);
}

int lsm303ah_set_enable(struct st_sensor_data *sdata, bool state)
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
	case LSM303AH_TAP:
		if (state && CHECK_BIT(priv->enabled_sensor,
				LSM303AH_DOUBLE_TAP)) {
			err = -EINVAL;

			goto enable_sensor_error;
		}

		break;

	case LSM303AH_DOUBLE_TAP:
		if (state && CHECK_BIT(priv->enabled_sensor,
				LSM303AH_TAP)) {
			err = -EINVAL;

			goto enable_sensor_error;
		}

		break;

	case LSM303AH_FF:
	case LSM303AH_WAKEUP:
	case LSM303AH_ACTIVITY:
		break;

	case LSM303AH_TILT:
	case LSM303AH_SIGN_M:
	case LSM303AH_STEP_D:
	case LSM303AH_STEP_C:
		err = lsm303ah_update_event_functions(sdata->cdata);
		if (err < 0)
			goto enable_sensor_error;

		break;

	case LSM303AH_ACCEL:
		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);

		break;

	default:
		return -EINVAL;
	}

	err = lsm303ah_update_drdy_irq(sdata);
	if (err < 0)
		goto enable_sensor_error;

	err = lsm303ah_acc_write_odr(sdata);
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

int lsm303ah_acc_enable_sensors(struct st_sensor_data *sdata)
{
	return lsm303ah_set_enable(sdata, true);
}

int lsm303ah_acc_disable_sensors(struct st_sensor_data *sdata)
{
	return lsm303ah_set_enable(sdata, false);
}

ssize_t lsm303ah_acc_get_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < LSM303AH_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			lsm303ah_fs_table.fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

ssize_t lsm303ah_acc_get_cur_scale(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	u8 pmode = GET_PDATA(sdata->cdata)->power_mode;

	for (i = 0; i < LSM303AH_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
			lsm303ah_fs_table.fs_avl[i].gain[pmode])
			break;

	return sprintf(buf, "%d\n", lsm303ah_fs_table.fs_avl[i].urv);
}

ssize_t lsm303ah_acc_set_cur_scale(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int urv, err;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	err = lsm303ah_set_fs(sdata, urv);
	if (err < 0)
		return err;

	return count;
}

static ssize_t lsm303ah_acc_get_resolution_mode(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	u8 pmode = GET_PDATA(sdata->cdata)->power_mode;

	return sprintf(buf, "%s\n", (pmode == LSM303AH_LP_MODE) ? "low" : "high");
}

static ssize_t lsm303ah_acc_set_resolution_mode(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err, i;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);
	u8 *pmode = &(GET_PDATA(sdata->cdata)->power_mode);

	for (i = 0; i < LSM303AH_FS_LIST_NUM; i++) {
		if (sdata->c_gain ==
			lsm303ah_fs_table.fs_avl[i].gain[*pmode])
			break;
	}

	if (!strncmp(buf, "low", count - 1))
		*pmode = LSM303AH_LP_MODE;
	else if (!strncmp(buf, "high", count - 1))
		*pmode = LSM303AH_HR_MODE;
	else
		return -EINVAL;

	err = lsm303ah_acc_write_odr(sdata);
	if (err < 0)
		return err;

	sdata->c_gain = lsm303ah_fs_table.fs_avl[i].gain[*pmode];

	return count;
}

ssize_t lsm303ah_reset_step_counter(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err;
	struct st_sensor_data *sdata = dev_get_drvdata(dev);

	err = st_sensor_write_data(sdata->cdata, LSM303AH_STEP_C_MINTHS_ADDR,
					LSM303AH_STEP_C_MINTHS_RST_NSTEP_MASK,
					LSM303AH_EN_BIT);
	if (err < 0)
		return err;

	return count;
}

int lsm303ah_init_sensors(struct st_common_data *cdata)
{
	int err, i;
	struct st_sensor_data *sdata;
	
	for (i = 0; i < LSM303AH_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		
		err = lsm303ah_acc_disable_sensors(sdata);
		if (err < 0)
			return err;
		
		if (sdata->sindex == LSM303AH_ACCEL) {
			err = lsm303ah_set_fs(sdata, LSM303AH_ACCEL_FS);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[LSM303AH_ACCEL].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	cdata->sensors[LSM303AH_ACCEL].hr_timer.function =
						&st_sensor_poll_function;

	/*
	 * Soft reset the device on power on.
	 */
	err = st_sensor_write_data(cdata, LSM303AH_SOFT_RESET_ADDR,
						LSM303AH_SOFT_RESET_MASK,
						LSM303AH_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Enable latched interrupt mode.
	 */
	err = st_sensor_write_data(cdata, LSM303AH_LIR_ADDR,
						LSM303AH_LIR_MASK,
						LSM303AH_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Enable block data update feature.
	 */
	err = st_sensor_write_data(cdata, LSM303AH_BDU_ADDR,
						LSM303AH_BDU_MASK,
						LSM303AH_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Route interrupt from INT2 to INT1 pin.
	 */
	err = st_sensor_write_data(cdata, LSM303AH_INT2_ON_INT1_ADDR,
						LSM303AH_INT2_ON_INT1_MASK,
						LSM303AH_EN_BIT);
	if (err < 0)
		return err;

	/*
	 * Configure default free fall event threshold.
	 */
	err = st_sensor_write_data(sdata->cdata, LSM303AH_FREE_FALL_ADDR,
						LSM303AH_FREE_FALL_THS_MASK,
						LSM303AH_FREE_FALL_THS_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure default free fall event duration.
	 */
	err = st_sensor_write_data(sdata->cdata, LSM303AH_FREE_FALL_ADDR,
						LSM303AH_FREE_FALL_DUR_MASK,
						LSM303AH_FREE_FALL_DUR_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure Tap event recognition on all direction (X, Y and Z axes).
	 */

	err = st_sensor_write_data(sdata->cdata, LSM303AH_TAP_AXIS_ADDR,
						LSM303AH_TAP_AXIS_MASK,
						LSM303AH_TAP_AXIS_ANABLE_ALL);
	if (err < 0)
		return err;

	/*
	 * Configure default threshold for Tap event recognition.
	 */
	err = st_sensor_write_data(sdata->cdata, LSM303AH_TAP_THS_ADDR,
						LSM303AH_TAP_THS_MASK,
						LSM303AH_TAP_THS_DEFAULT);
	if (err < 0)
		return err;

	/*
	 * Configure default threshold for Wake Up event recognition.
	 */
	err = st_sensor_write_data(sdata->cdata, LSM303AH_WAKE_UP_THS_ADDR,
						LSM303AH_WAKE_UP_THS_WU_MASK,
						LSM303AH_WAKE_UP_THS_WU_DEFAULT);
	if (err < 0)
		return err;

	return 0;
}

static irqreturn_t lsm303ah_save_tstamp(int irq, void *private)
{
	struct st_common_data *cdata = private;

	GET_PDATA(cdata)->timestamp = st_sensor_get_time_ns();
	queue_work(lsm303ah_acc_wkq, &(GET_PDATA(cdata)->irq_work));
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

int lsm303ah_allocate_workqueue(struct st_common_data *cdata)
{
	int err;

	if (!lsm303ah_acc_wkq)
		lsm303ah_acc_wkq = create_workqueue(cdata->name);

	if (!lsm303ah_acc_wkq)
		return -EINVAL;

	INIT_WORK(&(GET_PDATA(cdata)->irq_work), lsm303ah_irq_management);

	err = request_threaded_irq(cdata->irq, lsm303ah_save_tstamp, NULL,
				   IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		return err;

	return 0;
}

ADD_DEVICE_ENABLE_ATTR;
ADD_DEVICE_POLLING_ATTR;

static DEVICE_ATTR(resolution, S_IWUSR | S_IRUGO,
				lsm303ah_acc_get_resolution_mode,
				lsm303ah_acc_set_resolution_mode);

static DEVICE_ATTR(scale_avail, S_IRUGO,
				lsm303ah_acc_get_scale_avail,
				NULL);

static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO,
				lsm303ah_acc_get_cur_scale,
				lsm303ah_acc_set_cur_scale);

static DEVICE_ATTR(reset_steps, S_IWUSR,
				NULL,
				lsm303ah_reset_step_counter);

static struct attribute *lsm303ah_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_resolution.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm303ah_step_c_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_reset_steps.attr,
	NULL,
};

static struct attribute *lsm303ah_step_ff_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_double_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_step_d_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_tilt_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_sign_m_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_wakeup_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm303ah_act_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group lsm303ah_attribute_groups[] = {
	[LSM303AH_ACCEL] = {
		.attrs = lsm303ah_accel_attribute,
		.name = "accel",
	},
	[LSM303AH_STEP_C] = {
		.attrs = lsm303ah_step_c_attribute,
		.name = "step_c",
	},
	[LSM303AH_FF] = {
		.attrs = lsm303ah_step_ff_attribute,
		.name = "free_fall",
	},
	[LSM303AH_TAP] = {
		.name = "tap",
		.attrs = lsm303ah_tap_attribute,
	},
	[LSM303AH_DOUBLE_TAP] = {
		.name = "double_tap",
		.attrs = lsm303ah_double_tap_attribute,
	},
	[LSM303AH_STEP_D] = {
		.name = "step_d",
		.attrs = lsm303ah_step_d_attribute,
	},
	[LSM303AH_TILT] = {
		.name = "tilt",
		.attrs = lsm303ah_tilt_attribute,
	},
	[LSM303AH_SIGN_M] = {
		.name = "sign_m",
		.attrs = lsm303ah_sign_m_attribute,
	},
	[LSM303AH_WAKEUP] = {
		.name = "wake_up",
		.attrs = lsm303ah_wakeup_attribute,
	},
	[LSM303AH_ACTIVITY] = {
		.name = "act",
		.attrs = lsm303ah_act_attribute,
	},
};

int lsm303ah_sensors_data_init(struct st_common_data *cdata)
{
	int32_t i;
	bool is_3_axis;
	struct st_sensor_data *sdata;

	GET_PDATA(cdata)->common_odr = 0;
	GET_PDATA(cdata)->enabled_sensor = 0;
	GET_PDATA(cdata)->power_mode = LSM303AH_PMODE_DEFAULT;
	GET_PDATA(cdata)->cdata = cdata;
	cdata->sensors_count = LSM303AH_SENSORS_NUMB;

	for (i = 0; i < LSM303AH_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lsm303ah_sensors_table[i].name;
		sdata->c_odr = lsm303ah_sensors_table[i].min_odr_hz;
		sdata->write_odr = lsm303ah_acc_write_odr;
		sdata->enable = lsm303ah_acc_enable_sensors;
		sdata->disable = lsm303ah_acc_disable_sensors;

		switch(i) {
			case LSM303AH_ACCEL:
				sdata->ktime = ktime_set(0, MS_TO_NS(1000 /
								sdata->c_odr));
				is_3_axis = true;

				break;

			default:
				is_3_axis = false;

				break;
		}

		st_sensor_input_init(sdata, kasprintf(GFP_KERNEL, "%s_%s",
						cdata->name, sdata->name),
						is_3_axis);

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
						&lsm303ah_attribute_groups[i]))
			goto error_lsm303ah_sensors_data_init;
	}

	return 0;

error_lsm303ah_sensors_data_init:
	dev_err(cdata->dev, "failed to create sysfs group for sensor %s",
							sdata->name);
	for (; i >= 0; i--) {
		sysfs_remove_group(&sdata->input_dev->dev.kobj,
						&lsm303ah_attribute_groups[i]);
		input_unregister_device(cdata->sensors[i].input_dev);
	}

	return -EINVAL;
}

int lsm303ah_acc_probe(struct st_common_data *cdata)
{
	int err;

	err = st_sensor_common_probe(cdata, cdata->irq);
	if (err < 0)
		return err;

	err = lsm303ah_sensors_data_init(cdata);
	if (err < 0)
		return err;

	err = lsm303ah_init_sensors(cdata);
	if (err < 0)
		return err;

	INIT_WORK(&cdata->sensors[LSM303AH_ACCEL].input_work, lsm303ah_acc_poll_wk);

	if (cdata->irq > 0) {
		err = lsm303ah_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}
	return 0;
}
EXPORT_SYMBOL(lsm303ah_acc_probe);

int lsm303ah_acc_remove(struct st_common_data *cdata)
{
	u8 i;

	for (i = 0; i < cdata->sensors_count; i++) {
		lsm303ah_acc_disable_sensors(&cdata->sensors[i]);
		st_sensor_input_cleanup(&cdata->sensors[i]);
	}
	st_sensor_common_remove(cdata);

	return 0;
}
EXPORT_SYMBOL(lsm303ah_acc_remove);

int lsm303ah_acc_enable(struct st_common_data *cdata)
{
	int i, err;

	for (i = 0; i < cdata->sensors_count; i++) {
		err = lsm303ah_acc_enable_sensors(&cdata->sensors[i]);
		if (err < 0)
			return err;
	}
	return 0;
}
EXPORT_SYMBOL(lsm303ah_acc_enable);

int lsm303ah_acc_disable(struct st_common_data *cdata)
{
	int i, err;

	for (i = 0; i < cdata->sensors_count; i++) {
		err = lsm303ah_acc_disable_sensors(&cdata->sensors[i]);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(lsm303ah_acc_disable);

MODULE_DESCRIPTION("STMicroelectronics lsm303ah_acc driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_LICENSE("GPL v2");
