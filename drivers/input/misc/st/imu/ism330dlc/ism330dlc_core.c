/*
 * STMicroelectronics ism330dlc driver
 *
 * Copyright 2018 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Mario Tesi <mario.tesi@st.com>
 * v 1.2.2
 * Licensed under the GPL-2.
 */

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

#include "linux/platform_data/st/ism330dlc.h"
#include "ism330dlc_core.h"

/* COMMON DEFINE FOR ACCEL-GYRO SENSORS */
#define ISM330DLC_EN_BIT			0x01
#define ISM330DLC_DIS_BIT			0x00

#define ISM330DLC_WHO_AM_I			0x0f
#define ISM330DLC_WHO_AM_I_DEF			0x6a

#define ISM330DLC_INT1_CTRL_ADDR		0x0d
#define ISM330DLC_MD1_ADDR			0x5e

#define ISM330DLC_ODR_LIST_NUM			6
#define ISM330DLC_ODR_POWER_OFF_VAL		0x00
#define ISM330DLC_ODR_13HZ_VAL			0x01
#define ISM330DLC_ODR_26HZ_VAL			0x02
#define ISM330DLC_ODR_52HZ_VAL			0x03
#define ISM330DLC_ODR_104HZ_VAL			0x04
#define ISM330DLC_ODR_208HZ_VAL			0x05
#define ISM330DLC_ODR_416HZ_VAL			0x06
#define ISM330DLC_FS_LIST_NUM			4

#define ISM330DLC_BDU_ADDR			0x12
#define ISM330DLC_BDU_MASK			0x40

#define ISM330DLC_FUNC_EN_ADDR			0x19
#define ISM330DLC_FUNC_EN_MASK			0x04
#define ISM330DLC_FUNC_CFG_ACCESS_ADDR		0x01
#define ISM330DLC_FUNC_CFG_REG_MASK		0x80

#define ISM330DLC_LIR_ADDR			0x58
#define ISM330DLC_LIR_MASK			0x01

#define ISM330DLC_TIMER_EN_ADDR			0x19
#define ISM330DLC_TIMER_EN_MASK			0x20

#define ISM330DLC_INT2_ON_INT1_ADDR		0x13
#define ISM330DLC_INT2_ON_INT1_MASK		0x20

#define ISM330DLC_ROUNDING_ADDR			0x16
#define ISM330DLC_ROUNDING_MASK			0x04

#define ISM330DLC_FIFO_PEDO_E_ADDR		0x07
#define ISM330DLC_FIFO_PEDO_E_MASK		0x80

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define ISM330DLC_ACCEL_ODR_ADDR		0x10
#define ISM330DLC_ACCEL_ODR_MASK		0xf0
#define ISM330DLC_ACCEL_FS_ADDR			0x10
#define ISM330DLC_ACCEL_FS_MASK			0x0c
#define ISM330DLC_ACCEL_FS_2G_VAL		0x00
#define ISM330DLC_ACCEL_FS_4G_VAL		0x02
#define ISM330DLC_ACCEL_FS_8G_VAL		0x03
#define ISM330DLC_ACCEL_FS_16G_VAL		0x01
#define ISM330DLC_ACCEL_FS_2G_GAIN		61
#define ISM330DLC_ACCEL_FS_4G_GAIN		122
#define ISM330DLC_ACCEL_FS_8G_GAIN		244
#define ISM330DLC_ACCEL_FS_16G_GAIN		488
#define ISM330DLC_ACCEL_OUT_X_L_ADDR		0x28
#define ISM330DLC_ACCEL_OUT_Y_L_ADDR		0x2a
#define ISM330DLC_ACCEL_OUT_Z_L_ADDR		0x2c
#define ISM330DLC_ACCEL_AXIS_EN_ADDR		0x18
#define ISM330DLC_ACCEL_DRDY_IRQ_MASK		0x01
#define ISM330DLC_ACCEL_STD			1
#define ISM330DLC_ACCEL_STD_FROM_PD		2

/* CUSTOM VALUES FOR GYRO SENSOR */
#define ISM330DLC_GYRO_ODR_ADDR			0x11
#define ISM330DLC_GYRO_ODR_MASK			0xf0
#define ISM330DLC_GYRO_FS_ADDR			0x11
#define ISM330DLC_GYRO_FS_MASK			0x0c
#define ISM330DLC_GYRO_FS_250_VAL		0x00
#define ISM330DLC_GYRO_FS_500_VAL		0x01
#define ISM330DLC_GYRO_FS_1000_VAL		0x02
#define ISM330DLC_GYRO_FS_2000_VAL		0x03
#define ISM330DLC_GYRO_FS_250_GAIN		8750
#define ISM330DLC_GYRO_FS_500_GAIN		17500
#define ISM330DLC_GYRO_FS_1000_GAIN		35000
#define ISM330DLC_GYRO_FS_2000_GAIN		70000
#define ISM330DLC_GYRO_OUT_X_L_ADDR		0x22
#define ISM330DLC_GYRO_OUT_Y_L_ADDR		0x24
#define ISM330DLC_GYRO_OUT_Z_L_ADDR		0x26
#define ISM330DLC_GYRO_AXIS_EN_ADDR		0x19
#define ISM330DLC_GYRO_DRDY_IRQ_MASK		0x02
#define ISM330DLC_GYRO_STD			6
#define ISM330DLC_GYRO_STD_FROM_PD		2

#define ISM330DLC_OUT_XYZ_SIZE			6

/* CUSTOM VALUES FOR TILT SENSOR */
#define ISM330DLC_TILT_EN_ADDR			0x19
#define ISM330DLC_TILT_EN_MASK			0x08
#define ISM330DLC_TILT_DRDY_IRQ_MASK		0x02
#define ISM330DLC_WAKEUP_IRQ_MASK		0x20
#define ISM330DLC_ENABLE_INT_ADDR		0x58
#define ISM330DLC_ENABLE_INT_WK_MASK		0x80
#define ISM330DLC_WAKE_UP_THS_ADDR		0x5b
#define ISM330DLC_WAKE_UP_THS_VAL		0x02
#define ISM330DLC_WAKE_UP_SRC_ADDR		0x1b
#define ISM330DLC_WAKE_UP_DATA_AVL		0x08
#define ISM330DLC_WAKE_UP_DATA_MASK		0x0f

#define ISM330DLC_SRC_FUNC_ADDR			0x53
#define ISM330DLC_SRC2_FUNC_ADDR		0x54
#define ISM330DLC_SRC_TILT_DATA_AVL		0x20

/* Sensor Software Reset Bit */
#define ISM330DLC_RESET_ADDR			0x12
#define ISM330DLC_RESET_MASK			0x01

static const struct ism330dlc_sensor_name {
	const char *name;
	const char *description;
} ism330dlc_sensor_name[ISM330DLC_SENSORS_NUMB] = {
	[ISM330DLC_ACCEL] = {
		.name = "accel",
		.description = "ST ISM330DLC Accelerometer Sensor",
	},
	[ISM330DLC_GYRO] = {
		.name = "gyro",
		.description = "ST ISM330DLC Gyroscope Sensor",
	},
	[ISM330DLC_TILT] = {
		.name = "tilt",
		.description = "ST ISM330DLC Tilt Sensor",
	},
	[ISM330DLC_WAKEUP] = {
		.name = "wakeup",
		.description = "ST ISM330DLC Wakeup Sensor",
	},
};

struct ism330dlc_odr_reg {
	u32 hz;
	u8 value;
};

static const struct ism330dlc_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct ism330dlc_odr_reg odr_avl[6];
} ism330dlc_odr_table = {
	.addr[ISM330DLC_ACCEL] = ISM330DLC_ACC_ODR_ADDR,
	.mask[ISM330DLC_ACCEL] = ISM330DLC_ACC_ODR_MASK,
	.addr[ISM330DLC_GYRO] = ISM330DLC_GYR_ODR_ADDR,
	.mask[ISM330DLC_GYRO] = ISM330DLC_GYR_ODR_MASK,
	.odr_avl[0] = { .hz = 13, .value = ISM330DLC_ODR_13HZ_VAL },
	.odr_avl[1] = { .hz = 26, .value = ISM330DLC_ODR_26HZ_VAL },
	.odr_avl[2] = { .hz = 52, .value = ISM330DLC_ODR_52HZ_VAL },
	.odr_avl[3] = { .hz = 104, .value = ISM330DLC_ODR_104HZ_VAL },
	.odr_avl[4] = { .hz = 208, .value = ISM330DLC_ODR_208HZ_VAL },
	.odr_avl[5] = { .hz = 416, .value = ISM330DLC_ODR_416HZ_VAL },
};

struct ism330dlc_fs_reg {
	unsigned int gain;
	u8 value;
	int urv;
};

static struct ism330dlc_fs_table {
	u8 addr;
	u8 mask;
	struct ism330dlc_fs_reg fs_avl[ISM330DLC_FS_LIST_NUM];
} ism330dlc_fs_table[ISM330DLC_SENSORS_NUMB] = {
	[ISM330DLC_ACCEL] = {
		.addr = ISM330DLC_ACCEL_FS_ADDR,
		.mask = ISM330DLC_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = ISM330DLC_ACCEL_FS_2G_GAIN,
			       .value = ISM330DLC_ACCEL_FS_2G_VAL,
			       .urv = 2, },
		.fs_avl[1] = { .gain = ISM330DLC_ACCEL_FS_4G_GAIN,
			       .value = ISM330DLC_ACCEL_FS_4G_VAL,
			       .urv = 4, },
		.fs_avl[2] = { .gain = ISM330DLC_ACCEL_FS_8G_GAIN,
			       .value = ISM330DLC_ACCEL_FS_8G_VAL,
			       .urv = 8, },
		.fs_avl[3] = { .gain = ISM330DLC_ACCEL_FS_16G_GAIN,
			       .value = ISM330DLC_ACCEL_FS_16G_VAL,
			       .urv = 16, },
	},
	[ISM330DLC_GYRO] = {
		.addr = ISM330DLC_GYRO_FS_ADDR,
		.mask = ISM330DLC_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = ISM330DLC_GYRO_FS_250_GAIN,
			       .value = ISM330DLC_GYRO_FS_250_VAL,
			       .urv = 250, },
		.fs_avl[1] = { .gain = ISM330DLC_GYRO_FS_500_GAIN,
			       .value = ISM330DLC_GYRO_FS_500_VAL,
			       .urv = 500, },
		.fs_avl[2] = { .gain = ISM330DLC_GYRO_FS_1000_GAIN,
			       .value = ISM330DLC_GYRO_FS_1000_VAL,
			        .urv = 1000, },
		.fs_avl[3] = { .gain = ISM330DLC_GYRO_FS_2000_GAIN,
			       .value = ISM330DLC_GYRO_FS_2000_VAL,
			       .urv = 2000, },
	}
};

static struct workqueue_struct *ism330dlc_workqueue;

static inline void ism330dlc_flush_works(void)
{
	flush_workqueue(ism330dlc_workqueue);
}

static inline int64_t ism330dlc_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static int ism330dlc_write_data_with_mask(struct ism330dlc_data *cdata,
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

static int ism330dlc_input_init(struct ism330dlc_sensor_data *sdata,
				u16 bustype, const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = ism330dlc_sensor_name[sdata->sindex].description;

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit );
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if ((sdata->sindex == ISM330DLC_ACCEL) ||
	    (sdata->sindex == ISM330DLC_GYRO)) {
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

static void ism330dlc_input_cleanup(struct ism330dlc_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void ism330dlc_report_3axes_event(struct ism330dlc_sensor_data *sdata,
					 s32 *xyz, int64_t timestamp)
{
	struct input_dev  *input = sdata->input_dev;

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

static void ism330dlc_report_single_event(struct ism330dlc_sensor_data *sdata,
					  s32 data, int64_t timestamp)
{
	struct input_dev *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(input);
}

static enum hrtimer_restart ism330dlc_poll_function_read(struct hrtimer *timer)
{
	struct ism330dlc_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer,
			      struct ism330dlc_sensor_data,
			      hr_timer);

	sdata->timestamp = ism330dlc_get_time_ns();
	queue_work(ism330dlc_workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}

static int ism330dlc_get_poll_data(struct ism330dlc_sensor_data *sdata,
				   u8 *data)
{
	int err = 0;
	u8 reg_addr;

	switch(sdata->sindex) {
	case ISM330DLC_ACCEL:
		reg_addr = ISM330DLC_ACCEL_OUT_X_L_ADDR;

		break;
	case ISM330DLC_GYRO:
		reg_addr = ISM330DLC_GYRO_OUT_X_L_ADDR;

		break;
	default:
		dev_err(sdata->cdata->dev,
			"invalid polling mode for sensor %s\n",
			sdata->name);
		return -1;
	}

	err = sdata->cdata->tf->read(sdata->cdata, reg_addr,
				     ISM330DLC_OUT_XYZ_SIZE,
				     data, true);

	return err;
}

static void poll_function_work(struct work_struct *input_work)
{
	struct ism330dlc_sensor_data *sdata;
	int xyz[3] = { 0 };
	u8 data[6];
	int err;
	ktime_t tmpkt;

	sdata = container_of((struct work_struct *)input_work,
			     struct ism330dlc_sensor_data, input_work);

	/* Adjust to new timeout */
	tmpkt = ktime_sub(sdata->oldktime,
		   ktime_set(0, (ism330dlc_get_time_ns() - sdata->timestamp)));

	if (tmpkt < 0LL)
		tmpkt = ktime_set(0, MS_TO_NS(sdata->poll_interval));

	hrtimer_start(&sdata->hr_timer, tmpkt, HRTIMER_MODE_REL);
	if(sdata->sample_to_discard) {
		sdata->sample_to_discard--;
		return;
	}

	err = ism330dlc_get_poll_data(sdata, data);
	if (err < 0)
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
			sdata->name, err);
	else {
		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;

		ism330dlc_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
}

static int ism330dlc_set_drdy_irq(struct ism330dlc_sensor_data *sdata,
				  bool state)
{
	u8 reg_addr = 0, mask = 0, value;
	int err;

	if (state)
		value = ISM330DLC_EN_BIT;
	else
		value = ISM330DLC_DIS_BIT;

	switch (sdata->sindex) {
	case ISM330DLC_ACCEL:
	case ISM330DLC_GYRO:
		return 0;

	case ISM330DLC_TILT:
		reg_addr = ISM330DLC_MD1_ADDR;
		mask = ISM330DLC_TILT_DRDY_IRQ_MASK;
		break;
	case ISM330DLC_WAKEUP:
		reg_addr = ISM330DLC_MD1_ADDR;
		mask = ISM330DLC_WAKEUP_IRQ_MASK;
		err = ism330dlc_write_data_with_mask(sdata->cdata,
						ISM330DLC_ENABLE_INT_ADDR,
						ISM330DLC_ENABLE_INT_WK_MASK,
						value, true);
		break;
	default:
		return -EINVAL;
	}

	return ism330dlc_write_data_with_mask(sdata->cdata, reg_addr, mask,
					      value, true);
}

static int ism330dlc_set_fs(struct ism330dlc_sensor_data *sdata, u32 gain)
{
	int err, i;

	for (i = 0; i < ISM330DLC_FS_LIST_NUM; i++) {
		if (ism330dlc_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}

	if (i == ISM330DLC_FS_LIST_NUM)
		return -EINVAL;

	err = ism330dlc_write_data_with_mask(sdata->cdata,
			ism330dlc_fs_table[sdata->sindex].addr,
			ism330dlc_fs_table[sdata->sindex].mask,
			ism330dlc_fs_table[sdata->sindex].fs_avl[i].value,
			true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

static irqreturn_t ism330dlc_save_timestamp(int irq, void *private)
{
	struct ism330dlc_data *cdata = (struct ism330dlc_data *)private;

	cdata->timestamp = ism330dlc_get_time_ns();
	queue_work(ism330dlc_workqueue, &cdata->input_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static int ism330dlc_disable_sensors(struct ism330dlc_sensor_data *sdata);

static void ism330dlc_irq_management(struct work_struct *input_work)
{
	struct ism330dlc_data *cdata;
	u8 src_value = 0x00, src2_value = 0x00;
	struct ism330dlc_sensor_data *sdata;

	cdata = container_of((struct work_struct *)input_work,
			     struct ism330dlc_data, input_work);

	cdata->tf->read(cdata, ISM330DLC_SRC_FUNC_ADDR, 1, &src_value, true);
	cdata->tf->read(cdata, ISM330DLC_SRC2_FUNC_ADDR, 1, &src2_value, true);

	if (src_value & ISM330DLC_SRC_TILT_DATA_AVL) {
		sdata = &cdata->sensors[ISM330DLC_TILT];
		sdata->timestamp = cdata->timestamp;
		ism330dlc_report_single_event(sdata, 1, sdata->timestamp);
	}

	cdata->tf->read(cdata, ISM330DLC_WAKE_UP_SRC_ADDR, 1, &src_value,
			true);

	/* Wakeup event indicate wich axis involved */
	if (src_value & ISM330DLC_WAKE_UP_DATA_AVL) {
		sdata = &cdata->sensors[ISM330DLC_WAKEUP];
		sdata->timestamp = cdata->timestamp;
		ism330dlc_report_single_event(sdata,
					src_value & ISM330DLC_WAKE_UP_DATA_MASK,
					sdata->timestamp);
	}

	enable_irq(cdata->irq);
}

static int ism330dlc_allocate_workqueue(struct ism330dlc_data *cdata)
{
	int err;

	if (!ism330dlc_workqueue)
		ism330dlc_workqueue = create_workqueue(cdata->name);

	if (!ism330dlc_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->input_work, ism330dlc_irq_management);

	err = request_threaded_irq(cdata->irq, ism330dlc_save_timestamp, NULL,
				   IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		return err;

	return 0;
}

static int ism330dlc_set_extra_dependency(struct ism330dlc_sensor_data *sdata,
					bool enable)
{
	int err;

	if (!(sdata->cdata->sensors[ISM330DLC_TILT].enabled |
		 sdata->cdata->sensors[ISM330DLC_WAKEUP].enabled)) {
		if (enable) {
			err = ism330dlc_write_data_with_mask(sdata->cdata,
						ISM330DLC_FUNC_EN_ADDR,
						ISM330DLC_FUNC_EN_MASK,
						ISM330DLC_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = ism330dlc_write_data_with_mask(sdata->cdata,
						ISM330DLC_FUNC_EN_ADDR,
						ISM330DLC_FUNC_EN_MASK,
						ISM330DLC_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	if (!sdata->cdata->sensors[ISM330DLC_ACCEL].enabled) {
		if (enable) {
			u8 idx = 1;
			u16 acc_odr = sdata->cdata->sensors[ISM330DLC_ACCEL].c_odr;

			if (acc_odr > 26) {
				for (; idx < ISM330DLC_ODR_LIST_NUM; idx++)
					if (ism330dlc_odr_table.odr_avl[idx].hz == acc_odr)
						break;
			}
			err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[ISM330DLC_ACCEL],
				ism330dlc_odr_table.mask[ISM330DLC_ACCEL],
				ism330dlc_odr_table.odr_avl[idx].value, true);
			if (err < 0)
				return err;
		} else {
			err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[ISM330DLC_ACCEL],
				ism330dlc_odr_table.mask[ISM330DLC_ACCEL],
				ISM330DLC_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int _ism330dlc_enable_sensors(struct ism330dlc_sensor_data *sdata)
{
	int err, i;
	int64_t newTime;

	switch (sdata->sindex) {
	case ISM330DLC_ACCEL:
	case ISM330DLC_GYRO:
		for (i = 0; i < ISM330DLC_ODR_LIST_NUM; i++) {
			if (ism330dlc_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == ISM330DLC_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->sindex == ISM330DLC_ACCEL)
			sdata->sample_to_discard = ISM330DLC_ACCEL_STD +
						ISM330DLC_ACCEL_STD_FROM_PD;

		sdata->cdata->sensors[ISM330DLC_GYRO].sample_to_discard =
						ISM330DLC_GYRO_STD +
						ISM330DLC_GYRO_STD_FROM_PD;

		err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[sdata->sindex],
				ism330dlc_odr_table.mask[sdata->sindex],
				ism330dlc_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		sdata->c_odr = ism330dlc_odr_table.odr_avl[i].hz;
		newTime = 1000000000 / sdata->c_odr;
		sdata->oldktime = ktime_set(0, newTime);
		hrtimer_start(&sdata->hr_timer, sdata->oldktime, HRTIMER_MODE_REL);

		break;
	case ISM330DLC_TILT:
		err = ism330dlc_write_data_with_mask(sdata->cdata,
					ISM330DLC_TILT_EN_ADDR,
					ISM330DLC_TILT_EN_MASK,
					ISM330DLC_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	case ISM330DLC_WAKEUP:
		break;
	default:
		return -EINVAL;
	}

	err = ism330dlc_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;

	err = ism330dlc_set_drdy_irq(sdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int ism330dlc_enable_sensors(struct ism330dlc_sensor_data *sdata)
{
	int err;

	if (sdata->enabled)
		return 0;

	err = _ism330dlc_enable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = true;

	return 0;
}

static int _ism330dlc_disable_sensors(struct ism330dlc_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case ISM330DLC_ACCEL:
		if (sdata->cdata->sensors[ISM330DLC_TILT].enabled |
			sdata->cdata->sensors[ISM330DLC_WAKEUP].enabled) {
			err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[ISM330DLC_ACCEL],
				ism330dlc_odr_table.mask[ISM330DLC_ACCEL],
				ism330dlc_odr_table.odr_avl[0].value, true);
		} else {
			err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[ISM330DLC_ACCEL],
				ism330dlc_odr_table.mask[ISM330DLC_ACCEL],
				ISM330DLC_ODR_POWER_OFF_VAL, true);
		}
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);

		break;
	case ISM330DLC_GYRO:
		err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[ISM330DLC_GYRO],
				ism330dlc_odr_table.mask[ISM330DLC_GYRO],
				ISM330DLC_ODR_POWER_OFF_VAL, true);
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);

		break;
	case ISM330DLC_TILT:
		err = ism330dlc_write_data_with_mask(sdata->cdata,
				ISM330DLC_TILT_EN_ADDR,
				ISM330DLC_TILT_EN_MASK,
				ISM330DLC_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	case ISM330DLC_WAKEUP:
		break;
	default:
		return -EINVAL;
	}

	err = ism330dlc_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	err = ism330dlc_set_drdy_irq(sdata, false);
	if (err < 0)
		return err;

	return 0;
}

static int ism330dlc_disable_sensors(struct ism330dlc_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	err = _ism330dlc_disable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = false;

	return 0;
}

static int ism330dlc_init_sensors(struct ism330dlc_data *cdata)
{
	int err, i;
	struct ism330dlc_sensor_data *sdata;
	u8 wk_val = ISM330DLC_WAKE_UP_THS_VAL;

	for (i = 0; i < ISM330DLC_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = ism330dlc_disable_sensors(sdata);
		if (err < 0)
			return err;

		if ((sdata->sindex == ISM330DLC_ACCEL) ||
		    (sdata->sindex == ISM330DLC_GYRO)) {
			err = ism330dlc_set_fs(sdata, sdata->c_gain);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[ISM330DLC_ACCEL].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	hrtimer_init(&cdata->sensors[ISM330DLC_GYRO].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	cdata->sensors[ISM330DLC_ACCEL].hr_timer.function =
						&ism330dlc_poll_function_read;
	cdata->sensors[ISM330DLC_GYRO].hr_timer.function =
						&ism330dlc_poll_function_read;


	/* Software reset */
	err = ism330dlc_write_data_with_mask(cdata, ISM330DLC_RESET_ADDR,
					   ISM330DLC_RESET_MASK, ISM330DLC_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable Latch Mode Bit */
	err = ism330dlc_write_data_with_mask(cdata, ISM330DLC_LIR_ADDR,
					   ISM330DLC_LIR_MASK, ISM330DLC_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable timestamp count */
	err = ism330dlc_write_data_with_mask(cdata, ISM330DLC_TIMER_EN_ADDR,
					   ISM330DLC_TIMER_EN_MASK,
					   ISM330DLC_EN_BIT, true);
		if (err < 0)
			return err;

	/* Output data not updated until have been read */
	err = ism330dlc_write_data_with_mask(cdata, ISM330DLC_BDU_ADDR,
					   ISM330DLC_BDU_MASK, ISM330DLC_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable Source Rounding function */
	err = ism330dlc_write_data_with_mask(cdata, ISM330DLC_ROUNDING_ADDR,
					   ISM330DLC_ROUNDING_MASK,
					   ISM330DLC_EN_BIT, true);
	if (err < 0)
		return err;

	/* Set all interrupt signals in logic or on INT1 pad */
	err = ism330dlc_write_data_with_mask(cdata,
					   ISM330DLC_INT2_ON_INT1_ADDR,
					   ISM330DLC_INT2_ON_INT1_MASK,
					   ISM330DLC_EN_BIT, true);
	if (err < 0)
		return err;

	err = cdata->tf->write(cdata, ISM330DLC_WAKE_UP_THS_ADDR, 1, &wk_val,
			       true);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);
	err = ism330dlc_write_data_with_mask(sdata->cdata,
					   ISM330DLC_FUNC_CFG_ACCESS_ADDR,
					   ISM330DLC_FUNC_CFG_REG_MASK,
					   ISM330DLC_EN_BIT, false);
	if (err < 0)
		goto ism330dlc_init_sensor_mutex_unlock;

	err = ism330dlc_write_data_with_mask(sdata->cdata,
					   ISM330DLC_FUNC_CFG_ACCESS_ADDR,
					   ISM330DLC_FUNC_CFG_REG_MASK,
					   ISM330DLC_DIS_BIT, false);
	if (err < 0)
		goto ism330dlc_init_sensor_mutex_unlock;

	mutex_unlock(&cdata->bank_registers_lock);
	cdata->sensors[ISM330DLC_ACCEL].oldktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[ISM330DLC_ACCEL].poll_interval));
	cdata->sensors[ISM330DLC_GYRO].oldktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[ISM330DLC_ACCEL].poll_interval));
	INIT_WORK(&cdata->sensors[ISM330DLC_ACCEL].input_work, poll_function_work);
	INIT_WORK(&cdata->sensors[ISM330DLC_GYRO].input_work, poll_function_work);

	return 0;

ism330dlc_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int ism330dlc_set_odr(struct ism330dlc_sensor_data *sdata, u32 odr)
{
	int err = 0, i;

	for (i = 0; i < ISM330DLC_ODR_LIST_NUM; i++) {
		if (ism330dlc_odr_table.odr_avl[i].hz >= odr)
			break;
	}
	if (i == ISM330DLC_ODR_LIST_NUM)
		return -EINVAL;

	if (sdata->c_odr == ism330dlc_odr_table.odr_avl[i].hz)
		return 0;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
		ism330dlc_flush_works();

		if (sdata->sindex == ISM330DLC_ACCEL)
			sdata->cdata->sensors[ISM330DLC_ACCEL].sample_to_discard +=
							ISM330DLC_ACCEL_STD;

		if (sdata->cdata->sensors[ISM330DLC_GYRO].enabled)
			sdata->cdata->sensors[ISM330DLC_GYRO].sample_to_discard +=
							ISM330DLC_GYRO_STD;

		err = ism330dlc_write_data_with_mask(sdata->cdata,
				ism330dlc_odr_table.addr[sdata->sindex],
				ism330dlc_odr_table.mask[sdata->sindex],
				ism330dlc_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = ism330dlc_odr_table.odr_avl[i].hz;
		enable_irq(sdata->cdata->irq);
	} else
		sdata->c_odr = ism330dlc_odr_table.odr_avl[i].hz;

	return err;
}

static ssize_t get_enable(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	int err;
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = ism330dlc_enable_sensors(sdata);
	else
		err = ism330dlc_disable_sensors(sdata);

	return count;
}

static ssize_t get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_interval);
}

static ssize_t set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int polling_rate;
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);
	int64_t newTime;

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	/*
	 * Polling interval is in msec, then we have to convert it in Hz to
	 * configure ODR through ism330dlc_set_odr
	 */
	err = ism330dlc_set_odr(sdata, 1000 / polling_rate);
	if (!(err < 0)) {
		sdata->poll_interval = 1000 / sdata->c_odr;
		newTime = MS_TO_NS(sdata->poll_interval);
		sdata->oldktime = ktime_set(0, newTime);
	}
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t get_sampling_freq(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static ssize_t set_sampling_freq(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int err;
	unsigned int odr;
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = ism330dlc_set_odr(sdata, odr);
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t get_sampling_frequency_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int i, len = 0;

	for (i = 0; i < ISM330DLC_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 ism330dlc_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_scale_avail(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < ISM330DLC_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			ism330dlc_fs_table[sdata->sindex].fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_cur_scale(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int i;
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < ISM330DLC_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
				ism330dlc_fs_table[sdata->sindex].fs_avl[i].gain)
			break;

	return sprintf(buf, "%d\n",
		       ism330dlc_fs_table[sdata->sindex].fs_avl[i].urv);
}

static ssize_t set_cur_scale(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int i, urv, err;
	struct ism330dlc_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	for (i = 0; i < ISM330DLC_FS_LIST_NUM; i++)
		if (urv == ism330dlc_fs_table[sdata->sindex].fs_avl[i].urv)
			break;

	if (i == ISM330DLC_FS_LIST_NUM)
		return -EINVAL;

	err = ism330dlc_set_fs(sdata,
			     ism330dlc_fs_table[sdata->sindex].fs_avl[i].gain);
	if (err < 0)
		return err;

	return count;
}


static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, get_enable, set_enable);
static DEVICE_ATTR(sampling_freq, S_IWUSR | S_IRUGO, get_sampling_freq,
		   set_sampling_freq);
static DEVICE_ATTR(polling_rate, S_IWUSR | S_IRUGO, get_polling_rate,
		   set_polling_rate);
static DEVICE_ATTR(sampling_freq_avail, S_IRUGO, get_sampling_frequency_avail,
		   NULL);
static DEVICE_ATTR(scale_avail, S_IRUGO, get_scale_avail, NULL);
static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO, get_cur_scale, set_cur_scale);

static struct attribute *ism330dlc_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *ism330dlc_gyro_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *ism330dlc_tilt_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *ism330dlc_wakeup_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group ism330dlc_attribute_groups[] = {
	[ISM330DLC_ACCEL] = {
		.attrs = ism330dlc_accel_attribute,
		.name = "accel",
	},
	[ISM330DLC_GYRO] = {
		.attrs = ism330dlc_gyro_attribute,
		.name = "gyro",
	},
	[ISM330DLC_TILT] = {
		.attrs = ism330dlc_tilt_attribute,
		.name = "tilt",
	},
	[ISM330DLC_WAKEUP] = {
		.attrs = ism330dlc_wakeup_attribute,
		.name = "wakeup",
	},
};

#ifdef CONFIG_OF
static u32 ism330dlc_parse_dt(struct ism330dlc_data *cdata)
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
#endif

int ism330dlc_common_probe(struct ism330dlc_data *cdata, int irq, u16 bustype)
{
	int32_t err, i;
	u8 wai = 0x00;
	struct ism330dlc_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->tb.buf_lock);

	/* Read Chip ID register */
	err = cdata->tf->read(cdata, ISM330DLC_WHO_AM_I, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != ISM330DLC_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	if (irq > 0) {
#ifdef CONFIG_OF
		err = ism330dlc_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct ism330dlc_platform_data *)
					cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) ||
			    (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else {
			cdata->drdy_int_pin = 1;
		}
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
			 cdata->drdy_int_pin);
	}

	for (i = 0; i < ISM330DLC_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = ism330dlc_sensor_name[i].name;
		if ((i == ISM330DLC_ACCEL) || (i == ISM330DLC_GYRO)) {
			sdata->c_odr = ism330dlc_odr_table.odr_avl[0].hz;
			sdata->c_gain = ism330dlc_fs_table[i].fs_avl[0].gain;
			sdata->poll_interval = 1000 / sdata->c_odr;
		}

		if (ism330dlc_input_init(sdata, bustype,
				       ism330dlc_sensor_name[i].description)) {
			dev_err(cdata->dev,
				"failed to register input device %s",
				sdata->name);
			sdata->input_dev = NULL;
			continue;
		}

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
				       &ism330dlc_attribute_groups[i])) {
			dev_err(cdata->dev,
				"failed to create sysfs group for sensor %s",
				sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	err = ism330dlc_init_sensors(cdata);
	if (err < 0)
		return err;
	if (irq > 0)
		cdata->irq = irq;

	if (irq > 0) {
		err = ism330dlc_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}

	dev_info(cdata->dev, "%s: probed\n", ISM330DLC_DEV_NAME);

	return 0;
}
EXPORT_SYMBOL(ism330dlc_common_probe);

void ism330dlc_common_remove(struct ism330dlc_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < ISM330DLC_SENSORS_NUMB; i++) {
		ism330dlc_disable_sensors(&cdata->sensors[i]);
		ism330dlc_input_cleanup(&cdata->sensors[i]);
	}

	if(ism330dlc_workqueue) {
		flush_workqueue(ism330dlc_workqueue);
		destroy_workqueue(ism330dlc_workqueue);
		ism330dlc_workqueue = NULL;
	}
}
EXPORT_SYMBOL(ism330dlc_common_remove);

#ifdef CONFIG_PM_SLEEP
static int ism330dlc_resume_sensors(struct ism330dlc_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _ism330dlc_enable_sensors(sdata);
}

static int ism330dlc_suspend_sensors(struct ism330dlc_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _ism330dlc_disable_sensors(sdata);
}

int ism330dlc_common_suspend(struct ism330dlc_data *cdata)
{
	ism330dlc_suspend_sensors(&cdata->sensors[ISM330DLC_ACCEL]);
	ism330dlc_suspend_sensors(&cdata->sensors[ISM330DLC_GYRO]);

	return 0;
}
EXPORT_SYMBOL(ism330dlc_common_suspend);

int ism330dlc_common_resume(struct ism330dlc_data *cdata)
{
	ism330dlc_resume_sensors(&cdata->sensors[ISM330DLC_ACCEL]);
	ism330dlc_resume_sensors(&cdata->sensors[ISM330DLC_GYRO]);

	return 0;
}
EXPORT_SYMBOL(ism330dlc_common_resume);

#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("STMicroelectronics ism330dlc driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
