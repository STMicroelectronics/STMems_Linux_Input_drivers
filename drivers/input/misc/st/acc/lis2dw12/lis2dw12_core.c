/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lis2dw12_core.c
* Authors            : AMG - Application Team
*		     : Mario Tesi <mario.tesi@st.com>
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.0
* Date               : 2016/Oct/18
* Description        : LIS2DW12 driver
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
*******************************************************************************/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "lis2dw12_core.h"

#define LIS2DW12_WHO_AM_I_ADDR		0x0f
#define LIS2DW12_WHO_AM_I_DEF		0x44

#define LIS2DW12_CTRL1_ADDR		0x20
#define LIS2DW12_CTRL2_ADDR		0x21
#define LIS2DW12_CTRL3_ADDR		0x22
#define LIS2DW12_CTRL4_INT1_PAD_ADDR	0x23
#define LIS2DW12_CTRL5_INT2_PAD_ADDR	0x24
#define LIS2DW12_CTRL6_ADDR		0x25
#define LIS2DW12_OUT_T_ADDR		0x26
#define LIS2DW12_STATUS_ADDR		0x27
#define LIS2DW12_OUTX_L_ADDR		0x28

#define LIS2DW12_TAP_THS_X_ADDR		0x30
#define LIS2DW12_TAP_THS_Y_ADDR		0x31
#define LIS2DW12_TAP_THS_Z_ADDR		0x32
#define LIS2DW12_INT_DUR_ADDR		0x33
#define LIS2DW12_WAKE_UP_THS_ADDR		0x34
#define LIS2DW12_WAKE_UP_DUR_ADDR		0x35
#define LIS2DW12_FREE_FALL_ADDR		0x36
#define LIS2DW12_STATUS_DUP_ADDR		0x37
#define LIS2DW12_WAKE_UP_SRC_ADDR		0x38
#define LIS2DW12_TAP_SRC_ADDR		0x39
#define LIS2DW12_6D_SRC_ADDR		0x3a
#define LIS2DW12_ALL_INT_ADDR		0x3b

#define LIS2DW12_WAKE_UP_IA_MASK		0x40
#define LIS2DW12_DOUBLE_TAP_MASK		0x10
#define LIS2DW12_SINGLE_TAP_MASK		0x08
#define LIS2DW12_6D_IA_MASK		0x04
#define LIS2DW12_FF_IA_MASK		0x02
#define LIS2DW12_DRDY_MASK		0x01
#define LIS2DW12_EVENT_MASK		(LIS2DW12_WAKE_UP_IA_MASK | \
					 LIS2DW12_DOUBLE_TAP_MASK | \
					 LIS2DW12_SINGLE_TAP_MASK | \
					 LIS2DW12_6D_IA_MASK | \
					 LIS2DW12_FF_IA_MASK)

#define LIS2DW12_ODR_MASK			0xf0
#define LIS2DW12_ODR_POWER_OFF_VAL	0x00
#define LIS2DW12_ODR_1HZ_LP_VAL		0x01
#define LIS2DW12_ODR_12HZ_LP_VAL		0x02
#define LIS2DW12_ODR_25HZ_LP_VAL		0x03
#define LIS2DW12_ODR_50HZ_LP_VAL		0x04
#define LIS2DW12_ODR_100HZ_LP_VAL		0x05
#define LIS2DW12_ODR_200HZ_LP_VAL		0x06
#define LIS2DW12_ODR_400HZ_LP_VAL		0x06
#define LIS2DW12_ODR_800HZ_LP_VAL		0x06
#define LIS2DW12_ODR_LP_LIST_NUM		9

#define LIS2DW12_ODR_12_5HZ_HR_VAL	0x02
#define LIS2DW12_ODR_25HZ_HR_VAL		0x03
#define LIS2DW12_ODR_50HZ_HR_VAL		0x04
#define LIS2DW12_ODR_100HZ_HR_VAL		0x05
#define LIS2DW12_ODR_200HZ_HR_VAL		0x06
#define LIS2DW12_ODR_400HZ_HR_VAL		0x07
#define LIS2DW12_ODR_800HZ_HR_VAL		0x08
#define LIS2DW12_ODR_HR_LIST_NUM		8

#define LIS2DW12_LP_MODE_MASK			0x03
#define LIS2DW12_POWER_MODE_MASK		0x0c

#define LIS2DW12_FS_MASK			0x30
#define LIS2DW12_FS_2G_VAL		0x00
#define LIS2DW12_FS_4G_VAL		0x01
#define LIS2DW12_FS_8G_VAL		0x02
#define LIS2DW12_FS_16G_VAL		0x03

/*
 * Sensitivity sets in LP mode [ug]
 */
#define LIS2DW12_FS_2G_GAIN_LP		976
#define LIS2DW12_FS_4G_GAIN_LP		1952
#define LIS2DW12_FS_8G_GAIN_LP		3904
#define LIS2DW12_FS_16G_GAIN_LP		7808

/*
 * Sensitivity sets in HR mode [ug]
 */
#define LIS2DW12_FS_2G_GAIN_HR		244
#define LIS2DW12_FS_4G_GAIN_HR		488
#define LIS2DW12_FS_8G_GAIN_HR		976
#define LIS2DW12_FS_16G_GAIN_HR		1952

#define LIS2DW12_FS_LIST_NUM		4

#define LIS2DW12_INT1_6D_MASK		0x80
#define LIS2DW12_INT1_S_TAP_MASK		0x40
#define LIS2DW12_INT1_WAKEUP_MASK		0x20
#define LIS2DW12_INT1_FREE_FALL_MASK	0x10
#define LIS2DW12_INT1_TAP_MASK		0x08
#define LIS2DW12_INT1_DRDY_MASK		0x01
#define LIS2DW12_INT2_SLEEP_MASK		0x40
#define LIS2DW12_INT1_EVENTS_MASK		(LIS2DW12_INT1_S_TAP_MASK | \
					 LIS2DW12_INT1_WAKEUP_MASK | \
					 LIS2DW12_INT1_FREE_FALL_MASK | \
					 LIS2DW12_INT1_TAP_MASK | \
					 LIS2DW12_INT1_6D_MASK)

#define LIS2DW12_INT_DUR_SHOCK_MASK	0x03
#define LIS2DW12_INT_DUR_QUIET_MASK	0x0c
#define LIS2DW12_INT_DUR_LAT_MASK		0xf0
#define LIS2DW12_INT_DUR_MASK		(LIS2DW12_INT_DUR_SHOCK_MASK | \
					 LIS2DW12_INT_DUR_QUIET_MASK | \
					 LIS2DW12_INT_DUR_LAT_MASK)

#define LIS2DW12_INT_DUR_STAP_DEFAULT	0x06
#define LIS2DW12_INT_DUR_DTAP_DEFAULT	0x7f

#define LIS2DW12_WAKE_UP_THS_S_D_TAP_MASK	0x80
#define LIS2DW12_WAKE_UP_THS_SLEEP_MASK	0x40
#define LIS2DW12_WAKE_UP_THS_WU_MASK	0x3f
#define LIS2DW12_WAKE_UP_THS_WU_DEFAULT	0x02

#define LIS2DW12_FREE_FALL_THS_MASK	0x07
#define LIS2DW12_FREE_FALL_DUR_MASK	0xf8
#define LIS2DW12_FREE_FALL_THS_DEFAULT	0x01
#define LIS2DW12_FREE_FALL_DUR_DEFAULT	0x01

#define LIS2DW12_BDU_MASK			0x08
#define LIS2DW12_SOFT_RESET_MASK		0x40
#define LIS2DW12_LIR_MASK			0x10

#define LIS2DW12_TAP_AXIS_MASK		0xe0
#define LIS2DW12_TAP_AXIS_ANABLE_ALL	0xe0
#define LIS2DW12_TAP_THS_MASK		0x1f
#define LIS2DW12_TAP_THS_DEFAULT		0x09
#define LIS2DW12_INT2_ON_INT1_MASK	0x20

#define LIS2DW12_OUT_XYZ_SIZE		6
#define LIS2DW12_EN_BIT			0x01
#define LIS2DW12_DIS_BIT			0x00
#define LIS2DW12_EN_LP_MODE_02	0x01

#define LIS2DW12_ACCEL_FS			2
#define LIS2DW12_FF_ODR			25
#define LIS2DW12_TAP_ODR			400
#define LIS2DW12_WAKEUP_ODR		25

#define LIS2DW12_MIN_EVENT_ODR		25

enum {
	LIS2DW12_LP_MODE = 0,
	LIS2DW12_HR_MODE,
	LIS2DW12_MODE_COUNT,
};

static struct workqueue_struct *lis2dw12_workqueue;

static const struct lis2dw12_sensor_name {
	const char *name;
	const char *description;
} lis2dw12_sensor_name[LIS2DW12_SENSORS_NUMB] = {
	[LIS2DW12_ACCEL] = {
		.name = "accel",
		.description = "ST LIS2DW12 Accelerometer Sensor",
	},
	[LIS2DW12_FF] = {
		.name = "free_fall",
		.description = "ST LIS2DW12 Free Fall Sensor",
	},
	[LIS2DW12_TAP] = {
		.name = "tap",
		.description = "ST LIS2DW12 Tap Sensor",
	},
	[LIS2DW12_DOUBLE_TAP] = {
		.name = "double_tap",
		.description = "ST LIS2DW12 Double Tap Sensor",
	},
	[LIS2DW12_WAKEUP] = {
		.name = "wake_up",
		.description = "ST LIS2DW12 Wake Up Sensor",
	},
};

struct lis2dw12_odr_reg {
	u32 hz;
	u8 value;
};

static const struct lis2dw12_odr_table_t {
	struct lis2dw12_odr_reg odr_avl[LIS2DW12_MODE_COUNT][LIS2DW12_ODR_LP_LIST_NUM];
} lis2dw12_odr_table = {
	.odr_avl[LIS2DW12_LP_MODE][0] = {
		.hz = 0,
		.value = LIS2DW12_ODR_POWER_OFF_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][1] = {
		.hz = 1,
		.value = LIS2DW12_ODR_1HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][2] = {
		.hz = 12,
		.value = LIS2DW12_ODR_12HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][3] = {
		.hz = 25,
		.value = LIS2DW12_ODR_25HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][4] = {
		.hz = 50,
		.value = LIS2DW12_ODR_50HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][5] = {
		.hz = 100,
		.value = LIS2DW12_ODR_100HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][6] = {
		.hz = 200,
		.value = LIS2DW12_ODR_200HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][7] = {
		.hz = 400,
		.value = LIS2DW12_ODR_400HZ_LP_VAL
	},
	.odr_avl[LIS2DW12_LP_MODE][8] = {
		.hz = 800,
		.value = LIS2DW12_ODR_800HZ_LP_VAL
	},

	.odr_avl[LIS2DW12_HR_MODE][0] = {
		.hz = 0,
		.value = LIS2DW12_ODR_POWER_OFF_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][1] = {
		.hz = 12,
		.value = LIS2DW12_ODR_12_5HZ_HR_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][2] = {
		.hz = 25,
		.value = LIS2DW12_ODR_25HZ_HR_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][3] = {
		.hz = 50,
		.value = LIS2DW12_ODR_50HZ_HR_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][4] = {
		.hz = 100,
		.value = LIS2DW12_ODR_100HZ_HR_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][5] = {
		.hz = 200,
		.value = LIS2DW12_ODR_200HZ_HR_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][6] = {
		.hz = 400,
		.value = LIS2DW12_ODR_400HZ_HR_VAL
	},
	.odr_avl[LIS2DW12_HR_MODE][7] = {
		.hz = 800,
		.value = LIS2DW12_ODR_800HZ_HR_VAL
	},
};

struct lis2dw12_fs_reg {
	unsigned int gain[LIS2DW12_MODE_COUNT];
	u8 value;
	int urv;
};

static struct lis2dw12_fs_table {
	u8 addr;
	u8 mask;
	struct lis2dw12_fs_reg fs_avl[LIS2DW12_FS_LIST_NUM];
} lis2dw12_fs_table = {
	.addr = LIS2DW12_CTRL6_ADDR,
	.mask = LIS2DW12_FS_MASK,
	.fs_avl[0] = {
		.gain = {
			LIS2DW12_FS_2G_GAIN_LP,
			LIS2DW12_FS_2G_GAIN_HR,
		},
		.value = LIS2DW12_FS_2G_VAL,
		.urv = 2, 
	},
	.fs_avl[1] = {
		.gain = {
			LIS2DW12_FS_4G_GAIN_LP,
			LIS2DW12_FS_4G_GAIN_HR,
		},
		.value = LIS2DW12_FS_4G_VAL,
		.urv = 4,
	},
	.fs_avl[2] = {
		.gain = {
			LIS2DW12_FS_8G_GAIN_LP,
			LIS2DW12_FS_8G_GAIN_LP,
		},
		.value = LIS2DW12_FS_8G_VAL,
		.urv = 8,
	},
	.fs_avl[3] = {
		.gain = {
			LIS2DW12_FS_16G_GAIN_LP,
			LIS2DW12_FS_16G_GAIN_HR,
		},
		.value = LIS2DW12_FS_16G_VAL,
		.urv = 16,
	},
};

static int lis2dw12_write_data_with_mask(struct lis2dw12_data *cdata,
				       u8 reg_addr, u8 mask, u8 data,
				       bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}

static int lis2dw12_input_init(struct lis2dw12_sensor_data *sdata, u16 bustype)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = lis2dw12_sensor_name[sdata->sindex].description;
	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit);
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if (sdata->sindex == LIS2DW12_ACCEL) {
		__set_bit(INPUT_EVENT_Y, sdata->input_dev->mscbit);
		__set_bit(INPUT_EVENT_Z, sdata->input_dev->mscbit);
	}

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev, "unable to register sensor %s\n",
			sdata->name);
		input_free_device(sdata->input_dev);
	}

	return err;
}

static void lis2dw12_input_cleanup(struct lis2dw12_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void lis2dw12_report_3axes_event(struct lis2dw12_sensor_data *sdata,
				      s32 *xyz, s64 timestamp)
{
	struct input_dev *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(input);
}

static void lis2dw12_report_single_event(struct lis2dw12_sensor_data *sdata,
				       s32 data)
{
	struct input_dev  *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    sdata->timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    sdata->timestamp & 0xffffffff);
	input_sync(input);
}

static inline s32 lis2dw12_data_align_bit(u8 ms, u8 ls, u8 power_mode)
{
	if (power_mode == LIS2DW12_LP_MODE)
		return (s32)(((s16)(ls | ms << 8)) >> 4);

	return (s32)(((s16)(ls | ms << 8)) >> 2);
}

static u8 lis2dw12_event_irq1_value(struct lis2dw12_data *cdata)
{
	u8 value = 0x0;

	if (cdata->sensors[LIS2DW12_FF].enabled)
		value |= LIS2DW12_INT1_FREE_FALL_MASK;

	if (cdata->sensors[LIS2DW12_DOUBLE_TAP].enabled)
		value |= LIS2DW12_INT1_TAP_MASK;

	if (cdata->sensors[LIS2DW12_TAP].enabled)
		value |= LIS2DW12_INT1_S_TAP_MASK | LIS2DW12_INT1_TAP_MASK;

	if (cdata->sensors[LIS2DW12_WAKEUP].enabled)
		value |= LIS2DW12_INT1_WAKEUP_MASK;

	return value;
}

static int lis2dw12_update_drdy_irq(struct lis2dw12_sensor_data *sdata)
{
	u8 reg_addr = LIS2DW12_CTRL4_INT1_PAD_ADDR, reg_val, reg_mask;

	switch (sdata->sindex) {
	case LIS2DW12_FF:
	case LIS2DW12_TAP:
	case LIS2DW12_DOUBLE_TAP:
	case LIS2DW12_WAKEUP:
		reg_val = lis2dw12_event_irq1_value(sdata->cdata);
		reg_mask = LIS2DW12_INT1_EVENTS_MASK;
		break;
	case LIS2DW12_ACCEL:
		return 0;
	default:
		return -EINVAL;
	}

	return lis2dw12_write_data_with_mask(sdata->cdata, reg_addr, reg_mask,
					   reg_val >> __ffs(reg_mask), true);
}

static int lis2dw12_set_fs(struct lis2dw12_sensor_data *sdata, unsigned int fs)
{
	int err, i;

	for (i = 0; i < LIS2DW12_FS_LIST_NUM; i++) {
		if (lis2dw12_fs_table.fs_avl[i].urv == fs)
			break;
	}

	if (i == LIS2DW12_FS_LIST_NUM)
		return -EINVAL;

	err = lis2dw12_write_data_with_mask(sdata->cdata, lis2dw12_fs_table.addr,
					  lis2dw12_fs_table.mask,
					  lis2dw12_fs_table.fs_avl[i].value, true);
	if (err < 0)
		return err;

	sdata->c_gain = lis2dw12_fs_table.fs_avl[i].gain[sdata->cdata->power_mode];

	return 0;
}

static inline int64_t lis2dw12_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

/* Acc data */
static int lis2dw12_get_acc_data(struct lis2dw12_data *cdata)
{
	u8 data[LIS2DW12_OUT_XYZ_SIZE];
	int err, xyz[3];
	struct lis2dw12_sensor_data *sdata = &cdata->sensors[LIS2DW12_ACCEL];

	err = cdata->tf->read(cdata, LIS2DW12_OUTX_L_ADDR, LIS2DW12_OUT_XYZ_SIZE,
			      data, true);
	if (err < 0) {
		dev_err(cdata->dev, "get acc data failed %d\n", err);
		return err;
	} else {
		xyz[0] = lis2dw12_data_align_bit(data[1], data[0], cdata->power_mode);
		xyz[1] = lis2dw12_data_align_bit(data[3], data[2], cdata->power_mode);
		xyz[2] = lis2dw12_data_align_bit(data[5], data[4], cdata->power_mode);

		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;

		lis2dw12_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
	
	return 0;
}

static void lis2dw12_acc_poll_function_work(struct work_struct *input_work)
{
	struct lis2dw12_sensor_data *sdata;
	sdata = container_of((struct work_struct *)input_work,
			     struct lis2dw12_sensor_data, input_work);

	lis2dw12_get_acc_data(sdata->cdata);
}

static enum hrtimer_restart lis2dw12_hrtimer_acc_callback(struct hrtimer *timer)
{
	struct lis2dw12_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer, struct lis2dw12_sensor_data,
			     hr_timer);

	sdata->timestamp = lis2dw12_get_time_ns();
	queue_work(lis2dw12_workqueue, &sdata->input_work);
	hrtimer_forward(timer, ktime_get(), sdata->oldktime);

	return HRTIMER_RESTART;
}

/* Events data */
static irqreturn_t lis2dw12_thread_fn(int irq, void *private)
{
	u8 status;
	struct lis2dw12_data *cdata = private;

	cdata->tf->read(cdata, LIS2DW12_STATUS_ADDR, 1, &status, true);

	if (status & LIS2DW12_EVENT_MASK) {
		if ((cdata->sensors[LIS2DW12_TAP].enabled) &&
			(status & LIS2DW12_SINGLE_TAP_MASK)) {
			cdata->sensors[LIS2DW12_TAP].timestamp = cdata->timestamp;
			lis2dw12_report_single_event(&cdata->sensors[LIS2DW12_TAP], 1);
		}

		if ((cdata->sensors[LIS2DW12_DOUBLE_TAP].enabled) &&
			(status & LIS2DW12_DOUBLE_TAP_MASK)) {
			cdata->sensors[LIS2DW12_DOUBLE_TAP].timestamp = cdata->timestamp;
			lis2dw12_report_single_event(&cdata->sensors[LIS2DW12_DOUBLE_TAP], 1);
		}

		if ((cdata->sensors[LIS2DW12_FF].enabled) &&
			(status & LIS2DW12_FF_IA_MASK)) {
			cdata->sensors[LIS2DW12_FF].timestamp = cdata->timestamp;
			lis2dw12_report_single_event(&cdata->sensors[LIS2DW12_FF], 1);
		}

		if ((cdata->sensors[LIS2DW12_WAKEUP].enabled) &&
			(status & LIS2DW12_WAKE_UP_IA_MASK)) {
			cdata->sensors[LIS2DW12_WAKEUP].timestamp = cdata->timestamp;
			lis2dw12_report_single_event(&cdata->sensors[LIS2DW12_WAKEUP], 1);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t lis2dw12_save_timestamp(int irq, void *private)
{
	struct lis2dw12_data *cdata = (struct lis2dw12_data *)private;

	cdata->timestamp = lis2dw12_get_time_ns();

	return IRQ_WAKE_THREAD;
}

static int lis2dw12_update_hw_odr(struct lis2dw12_sensor_data *sdata, int odr)
{
	int err, i;
	struct lis2dw12_data *cdata = sdata->cdata;
	u8 num_aval;

	/* odr num. may differs for LP and HR mode */
	num_aval = (cdata->power_mode == LIS2DW12_LP_MODE) ?
			LIS2DW12_ODR_LP_LIST_NUM : LIS2DW12_ODR_HR_LIST_NUM;

	for (i = 0; i < num_aval; i++) {
		if (lis2dw12_odr_table.odr_avl[cdata->power_mode][i].hz >= odr)
			break;
	}

	if (i == num_aval)
		return -EINVAL;

	if (sdata->c_odr == lis2dw12_odr_table.odr_avl[cdata->power_mode][i].hz)
		return 0;

	err = lis2dw12_write_data_with_mask(sdata->cdata,
					  LIS2DW12_CTRL1_ADDR,
					  LIS2DW12_ODR_MASK,
					  lis2dw12_odr_table.odr_avl[cdata->power_mode][i].value,
					  true);
	if (err < 0)
		return err;

	sdata->c_odr = lis2dw12_odr_table.odr_avl[cdata->power_mode][i].hz;

	return 0;
}

static int lis2dw12_configure_tap_event(struct lis2dw12_sensor_data *sdata,
				      bool single_tap)
{
	u8 err = 0;

	if (single_tap) {
		err = lis2dw12_write_data_with_mask(sdata->cdata,
						  LIS2DW12_INT_DUR_ADDR,
						  LIS2DW12_INT_DUR_MASK,
						  LIS2DW12_INT_DUR_STAP_DEFAULT,
						  true);
		if (err < 0)
			return err;

		err = lis2dw12_write_data_with_mask(sdata->cdata,
						  LIS2DW12_WAKE_UP_THS_ADDR,
						  LIS2DW12_WAKE_UP_THS_S_D_TAP_MASK,
						  LIS2DW12_DIS_BIT, true);
		if (err < 0)
			return err;
	} else {
		err = lis2dw12_write_data_with_mask(sdata->cdata,
						  LIS2DW12_INT_DUR_ADDR,
						  LIS2DW12_INT_DUR_MASK,
						  LIS2DW12_INT_DUR_DTAP_DEFAULT,
						  true);
		if (err < 0)
			return err;

		err = lis2dw12_write_data_with_mask(sdata->cdata,
						  LIS2DW12_WAKE_UP_THS_ADDR,
						  LIS2DW12_WAKE_UP_THS_S_D_TAP_MASK,
						  LIS2DW12_EN_BIT, true);
		if (err < 0)
			return err;
	}

	return err;
}

static int _lis2dw12_enable_sensors(struct lis2dw12_sensor_data *sdata)
{
	int err = 0, i;
	struct lis2dw12_data *cdata = sdata->cdata;
	u8 num_aval;
	int64_t newTime;

	err = lis2dw12_update_drdy_irq(sdata);
	if (err < 0)
		return err;

	switch (sdata->sindex) {
	case LIS2DW12_ACCEL:
		num_aval = (cdata->power_mode == LIS2DW12_LP_MODE) ?
			LIS2DW12_ODR_LP_LIST_NUM : LIS2DW12_ODR_HR_LIST_NUM;

		for (i = 0; i < num_aval; i++) {
			if (lis2dw12_odr_table.odr_avl[cdata->power_mode][i].hz >= sdata->c_odr)
				break;
		}

		if (i == num_aval)
			return -EINVAL;

		err = lis2dw12_write_data_with_mask(sdata->cdata,
						LIS2DW12_CTRL1_ADDR,
						LIS2DW12_ODR_MASK,
						lis2dw12_odr_table.odr_avl[cdata->power_mode][i].value,
						true);
		if (err < 0)
			return err;

		err = lis2dw12_write_data_with_mask(sdata->cdata,
						LIS2DW12_CTRL1_ADDR,
						LIS2DW12_POWER_MODE_MASK,
						cdata->power_mode,
						true);
		if (err < 0)
			return err;

		sdata->c_odr = lis2dw12_odr_table.odr_avl[cdata->power_mode][i].hz;
		newTime = MS_TO_NS(sdata->poll_ms);
		sdata->oldktime = ktime_set(0, newTime);
		hrtimer_start(&sdata->hr_timer, sdata->oldktime, HRTIMER_MODE_REL);
		break;
	case LIS2DW12_TAP:
		lis2dw12_configure_tap_event(sdata, 1);
		break;
	case LIS2DW12_DOUBLE_TAP:
		lis2dw12_configure_tap_event(sdata, 0);
		break;
	case LIS2DW12_FF:
	case LIS2DW12_WAKEUP:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int lis2dw12_enable_sensors(struct lis2dw12_sensor_data *sdata)
{
	int err = 0;
	
	if (sdata->enabled)
		return 0;

	sdata->enabled = true;
	err = _lis2dw12_enable_sensors(sdata);
	if (err < 0)
		sdata->enabled = false;

	return err;
}

static int _lis2dw12_disable_sensors(struct lis2dw12_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case LIS2DW12_ACCEL:
		err = lis2dw12_write_data_with_mask(sdata->cdata,
					LIS2DW12_CTRL1_ADDR,
					LIS2DW12_ODR_MASK,
					lis2dw12_odr_table.odr_avl[sdata->cdata->power_mode][0].value,
					true);
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);
		break;
	case LIS2DW12_FF:
	case LIS2DW12_TAP:
	case LIS2DW12_DOUBLE_TAP:
	case LIS2DW12_WAKEUP:
		break;
	default:
		return -EINVAL;
	}

	err = lis2dw12_update_drdy_irq(sdata);
	if (err < 0)
		return err;

	return 0;
}

static int lis2dw12_disable_sensors(struct lis2dw12_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	sdata->enabled = false;
	err = _lis2dw12_disable_sensors(sdata);
	if (err < 0)
		sdata->enabled = true;

	return err;
}

static int lis2dw12_allocate_workqueue(struct lis2dw12_data *cdata)
{
	if (!lis2dw12_workqueue)
		lis2dw12_workqueue = create_workqueue(cdata->name);

	if (!lis2dw12_workqueue)
		return -EINVAL;

	return devm_request_threaded_irq(cdata->dev, cdata->irq,
					 lis2dw12_save_timestamp,
					 lis2dw12_thread_fn,
					 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					 cdata->name, cdata);
}

static int lis2dw12_init_sensors(struct lis2dw12_data *cdata)
{
	int err, i;
	struct lis2dw12_sensor_data *sdata;

	for (i = 0; i < LIS2DW12_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = lis2dw12_disable_sensors(sdata);
		if (err < 0)
			return err;

		if (sdata->sindex == LIS2DW12_ACCEL) {
			err = lis2dw12_set_fs(sdata, LIS2DW12_ACCEL_FS);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[LIS2DW12_ACCEL].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	cdata->sensors[LIS2DW12_ACCEL].hr_timer.function =
					&lis2dw12_hrtimer_acc_callback;
	cdata->selftest_status = 0;
	err = lis2dw12_write_data_with_mask(cdata,
					  LIS2DW12_CTRL2_ADDR,
					  LIS2DW12_SOFT_RESET_MASK,
					  LIS2DW12_EN_BIT, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(cdata,
					  LIS2DW12_CTRL1_ADDR,
					  LIS2DW12_LP_MODE_MASK,
					  LIS2DW12_EN_LP_MODE_02, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(cdata,
					  LIS2DW12_CTRL3_ADDR,
					  LIS2DW12_LIR_MASK,
					  LIS2DW12_EN_BIT, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(cdata,
					  LIS2DW12_CTRL2_ADDR,
					  LIS2DW12_BDU_MASK,
					  LIS2DW12_EN_BIT, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(sdata->cdata,
					  LIS2DW12_FREE_FALL_ADDR,
					  LIS2DW12_FREE_FALL_THS_MASK,
					  LIS2DW12_FREE_FALL_THS_DEFAULT, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(sdata->cdata,
					  LIS2DW12_FREE_FALL_ADDR,
					  LIS2DW12_FREE_FALL_DUR_MASK,
					  LIS2DW12_FREE_FALL_DUR_DEFAULT, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(sdata->cdata,
					  LIS2DW12_TAP_THS_Z_ADDR,
					  LIS2DW12_TAP_AXIS_MASK,
					  LIS2DW12_TAP_AXIS_ANABLE_ALL, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(sdata->cdata,
					  LIS2DW12_TAP_THS_X_ADDR,
					  LIS2DW12_TAP_THS_MASK,
					  LIS2DW12_TAP_THS_DEFAULT, true);
	if (err < 0)
		return err;

	err = lis2dw12_write_data_with_mask(sdata->cdata,
					  LIS2DW12_WAKE_UP_THS_ADDR,
					  LIS2DW12_WAKE_UP_THS_WU_MASK,
					  LIS2DW12_WAKE_UP_THS_WU_DEFAULT, true);
	if (err < 0)
		return err;

	cdata->sensors[LIS2DW12_ACCEL].oldktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LIS2DW12_ACCEL].poll_ms));
	INIT_WORK(&cdata->sensors[LIS2DW12_ACCEL].input_work,
		  lis2dw12_acc_poll_function_work);

	return 0;
}

static ssize_t lis2dw12_get_enable(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t lis2dw12_set_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		lis2dw12_enable_sensors(sdata);
	else
		lis2dw12_disable_sensors(sdata);

	return count;
}

static ssize_t lis2dw12_get_resolution_mode(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", 
		(sdata->cdata->power_mode == LIS2DW12_LP_MODE) ? "low" : "high");
}

static ssize_t lis2dw12_set_resolution_mode(struct device *dev,
					  struct device_attribute *attr, const char *buf,
					  size_t count)
{
	int i;
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < LIS2DW12_FS_LIST_NUM; i++) {
		if (sdata->c_gain ==
			lis2dw12_fs_table.fs_avl[i].gain[sdata->cdata->power_mode])
			break;
	}

	if (!strncmp(buf, "low", count - 1))
		sdata->cdata->power_mode = LIS2DW12_LP_MODE;
	else if (!strncmp(buf, "high", count - 1))
		sdata->cdata->power_mode = LIS2DW12_HR_MODE;
	else
		return -EINVAL;

	sdata->c_gain = lis2dw12_fs_table.fs_avl[i].gain[sdata->cdata->power_mode];

	return count;
}

static ssize_t lis2dw12_get_polling_rate(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", 1000 / sdata->c_odr);
}

static ssize_t lis2dw12_set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned int polling_rate;
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);
	int64_t newTime;

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = lis2dw12_update_hw_odr(sdata, 1000 / polling_rate);
	if (err < 0)
		goto err_poll;

	sdata->poll_ms = polling_rate;
	newTime = MS_TO_NS(1000 / sdata->c_odr);
	sdata->oldktime = ktime_set(0, newTime);
	mutex_unlock(&sdata->input_dev->mutex);

	return count;

err_poll:
	mutex_unlock(&sdata->input_dev->mutex);

	return err;
}

static ssize_t lis2dw12_get_scale_avail(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < LIS2DW12_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			lis2dw12_fs_table.fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t lis2dw12_get_cur_scale(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int i;
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < LIS2DW12_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
			lis2dw12_fs_table.fs_avl[i].gain[sdata->cdata->power_mode])
			break;

	return sprintf(buf, "%d\n", lis2dw12_fs_table.fs_avl[i].urv);
}

static ssize_t lis2dw12_set_cur_scale(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int urv, err;
	struct lis2dw12_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	err = lis2dw12_set_fs(sdata, urv);
	if (err < 0)
		return err;

	return count;
}

static DEVICE_ATTR(enable,
		   S_IWUSR | S_IRUGO,
		   lis2dw12_get_enable,
		   lis2dw12_set_enable);

static DEVICE_ATTR(resolution,
		   S_IWUSR | S_IRUGO,
		   lis2dw12_get_resolution_mode,
		   lis2dw12_set_resolution_mode);

static DEVICE_ATTR(polling_rate,
		   S_IWUSR | S_IRUGO,
		   lis2dw12_get_polling_rate,
		   lis2dw12_set_polling_rate);

static DEVICE_ATTR(scale_avail,
		   S_IRUGO,
		   lis2dw12_get_scale_avail,
		   NULL);

static DEVICE_ATTR(scale,
		   S_IWUSR | S_IRUGO,
		   lis2dw12_get_cur_scale,
		   lis2dw12_set_cur_scale);

static struct attribute *lis2dw12_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_resolution.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lis2dw12_step_ff_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lis2dw12_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lis2dw12_double_tap_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lis2dw12_wakeup_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group lis2dw12_attribute_groups[] = {
	[LIS2DW12_ACCEL] = {
		.attrs = lis2dw12_accel_attribute,
		.name = "accel",
	},
	[LIS2DW12_FF] = {
		.attrs = lis2dw12_step_ff_attribute,
		.name = "free_fall",
	},
	[LIS2DW12_TAP] = {
		.name = "tap",
		.attrs = lis2dw12_tap_attribute,
	},
	[LIS2DW12_DOUBLE_TAP] = {
		.name = "double_tap",
		.attrs = lis2dw12_double_tap_attribute,
	},
	[LIS2DW12_WAKEUP] = {
		.name = "wake_up",
		.attrs = lis2dw12_wakeup_attribute,
	},
};

#ifdef CONFIG_OF
static u32 lis2dw12_parse_dt(struct lis2dw12_data *cdata)
{
	u32 val;
	struct device_node *np;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "st,drdy-int-pin", &val) &&
	    (val <= 2) && (val > 0))
		cdata->drdy_int_pin = (u8)val;
	else
		cdata->drdy_int_pin = 1;

	return 0;
}
#endif /* CONFIG_OF */

int lis2dw12_common_probe(struct lis2dw12_data *cdata, int irq, u16 bustype)
{
	int32_t err, i;
	u8 wai = 0;
	struct lis2dw12_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->tb.buf_lock);

	err = cdata->tf->read(cdata, LIS2DW12_WHO_AM_I_ADDR, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != LIS2DW12_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	if (irq > 0) {
#ifdef CONFIG_OF
		err = lis2dw12_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lis2dw12_platform_data *)
				cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) || (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else {
			cdata->drdy_int_pin = 1;
		}
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
			 cdata->drdy_int_pin);
	}

	cdata->power_mode = LIS2DW12_LP_MODE;

	for (i = 0; i < LIS2DW12_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lis2dw12_sensor_name[i].name;

		if (i == LIS2DW12_ACCEL) {
			sdata->c_odr = lis2dw12_odr_table.odr_avl[cdata->power_mode][1].hz;
			sdata->poll_ms = 1000 / sdata->c_odr;
		}

		lis2dw12_input_init(sdata, bustype);

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
				       &lis2dw12_attribute_groups[i])) {
			dev_err(cdata->dev, "failed to create sysfs group for sensor %s",
					sdata->name);

			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	err = lis2dw12_init_sensors(cdata);
	if (err < 0)
		return err;

	if (irq > 0) {
		cdata->irq = irq;

		err = lis2dw12_allocate_workqueue(cdata);
		if (err)
			return err;
	}

	dev_info(cdata->dev, "%s: probed\n", LIS2DW12_DEV_NAME);

	return 0;
}
EXPORT_SYMBOL(lis2dw12_common_probe);

void lis2dw12_common_remove(struct lis2dw12_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < LIS2DW12_SENSORS_NUMB; i++) {
		lis2dw12_disable_sensors(&cdata->sensors[i]);
		lis2dw12_input_cleanup(&cdata->sensors[i]);
	}

	if(lis2dw12_workqueue) {
		flush_workqueue(lis2dw12_workqueue);
		destroy_workqueue(lis2dw12_workqueue);
		lis2dw12_workqueue = NULL;
	}
}
EXPORT_SYMBOL(lis2dw12_common_remove);

#ifdef CONFIG_PM
static int lis2dw12_resume_sensors(struct lis2dw12_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _lis2dw12_enable_sensors(sdata);
}
EXPORT_SYMBOL(lis2dw12_resume_sensors);

static int lis2dw12_suspend_sensors(struct lis2dw12_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _lis2dw12_disable_sensors(sdata);
}
EXPORT_SYMBOL(lis2dw12_suspend_sensors);

int lis2dw12_common_suspend(struct lis2dw12_data *cdata)
{
	lis2dw12_suspend_sensors(&cdata->sensors[LIS2DW12_ACCEL]);

	return 0;
}
EXPORT_SYMBOL(lis2dw12_common_suspend);

int lis2dw12_common_resume(struct lis2dw12_data *cdata)
{
	lis2dw12_resume_sensors(&cdata->sensors[LIS2DW12_ACCEL]);

	return 0;
}
EXPORT_SYMBOL(lis2dw12_common_resume);

#endif /* CONFIG_PM */

MODULE_DESCRIPTION("STMicroelectronics lis2dw12 driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
