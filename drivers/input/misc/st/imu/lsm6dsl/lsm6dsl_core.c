/*
 * STMicroelectronics lsm6dsl driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Mario Tesi <mario.tesi@st.com>
 * v 1.0.1
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

#include "linux/platform_data/st/lsm6dsl.h"

#include "lsm6dsl_core.h"

/* COMMON DEFINE FOR ACCEL-GYRO SENSORS */
#define LSM6DSL_EN_BIT			0x01
#define LSM6DSL_DIS_BIT			0x00

#define LSM6DSL_WHO_AM_I		0x0f
#define LSM6DSL_WHO_AM_I_DEF		0x6a

#define LSM6DSL_INT1_CTRL_ADDR		0x0d
#define LSM6DSL_MD1_ADDR		0x5e

#define LSM6DSL_ODR_LIST_NUM		5
#define LSM6DSL_ODR_POWER_OFF_VAL	0x00
#define LSM6DSL_ODR_13HZ_VAL		0x01
#define LSM6DSL_ODR_26HZ_VAL		0x02
#define LSM6DSL_ODR_52HZ_VAL		0x03
#define LSM6DSL_ODR_104HZ_VAL		0x04
#define LSM6DSL_ODR_208HZ_VAL		0x05
#define LSM6DSL_ODR_416HZ_VAL		0x06
#define LSM6DSL_FS_LIST_NUM		4

#define LSM6DSL_BDU_ADDR		0x12
#define LSM6DSL_BDU_MASK		0x40

#define LSM6DSL_FUNC_EN_ADDR		0x19
#define LSM6DSL_FUNC_EN_MASK		0x04
#define LSM6DSL_FUNC_CFG_ACCESS_ADDR	0x01
#define LSM6DSL_FUNC_CFG_REG_MASK	0x80

#define LSM6DSL_LIR_ADDR		0x58
#define LSM6DSL_LIR_MASK		0x01

#define LSM6DSL_TIMER_EN_ADDR		0x19
#define LSM6DSL_TIMER_EN_MASK		0x20

#define LSM6DSL_PEDOMETER_EN_ADDR	0x19
#define LSM6DSL_PEDOMETER_EN_MASK	0x10

#define LSM6DSL_INT2_ON_INT1_ADDR	0x13
#define LSM6DSL_INT2_ON_INT1_MASK	0x20

#define LSM6DSL_MIN_DURATION_MS		1638
#define LSM6DSL_ROUNDING_ADDR		0x16
#define LSM6DSL_ROUNDING_MASK		0x04

#define LSM6DSL_FIFO_PEDO_E_ADDR	0x07
#define LSM6DSL_FIFO_PEDO_E_MASK	0x80

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define LSM6DSL_ACCEL_ODR_ADDR		0x10
#define LSM6DSL_ACCEL_ODR_MASK		0xf0
#define LSM6DSL_ACCEL_FS_ADDR		0x10
#define LSM6DSL_ACCEL_FS_MASK		0x0c
#define LSM6DSL_ACCEL_FS_2G_VAL		0x00
#define LSM6DSL_ACCEL_FS_4G_VAL		0x02
#define LSM6DSL_ACCEL_FS_8G_VAL		0x03
#define LSM6DSL_ACCEL_FS_16G_VAL	0x01
#define LSM6DSL_ACCEL_FS_2G_GAIN	61
#define LSM6DSL_ACCEL_FS_4G_GAIN	122
#define LSM6DSL_ACCEL_FS_8G_GAIN	244
#define LSM6DSL_ACCEL_FS_16G_GAIN	488
#define LSM6DSL_ACCEL_OUT_X_L_ADDR	0x28
#define LSM6DSL_ACCEL_OUT_Y_L_ADDR	0x2a
#define LSM6DSL_ACCEL_OUT_Z_L_ADDR	0x2c
#define LSM6DSL_ACCEL_AXIS_EN_ADDR	0x18
#define LSM6DSL_ACCEL_DRDY_IRQ_MASK	0x01
#define LSM6DSL_ACCEL_STD		1
#define LSM6DSL_ACCEL_STD_FROM_PD	2

/* CUSTOM VALUES FOR GYRO SENSOR */
#define LSM6DSL_GYRO_ODR_ADDR		0x11
#define LSM6DSL_GYRO_ODR_MASK		0xf0
#define LSM6DSL_GYRO_FS_ADDR		0x11
#define LSM6DSL_GYRO_FS_MASK		0x0c
#define LSM6DSL_GYRO_FS_250_VAL		0x00
#define LSM6DSL_GYRO_FS_500_VAL		0x01
#define LSM6DSL_GYRO_FS_1000_VAL	0x02
#define LSM6DSL_GYRO_FS_2000_VAL	0x03
#define LSM6DSL_GYRO_FS_250_GAIN	8750
#define LSM6DSL_GYRO_FS_500_GAIN	17500
#define LSM6DSL_GYRO_FS_1000_GAIN	35000
#define LSM6DSL_GYRO_FS_2000_GAIN	70000
#define LSM6DSL_GYRO_OUT_X_L_ADDR	0x22
#define LSM6DSL_GYRO_OUT_Y_L_ADDR	0x24
#define LSM6DSL_GYRO_OUT_Z_L_ADDR	0x26
#define LSM6DSL_GYRO_AXIS_EN_ADDR	0x19
#define LSM6DSL_GYRO_DRDY_IRQ_MASK	0x02
#define LSM6DSL_GYRO_STD		6
#define LSM6DSL_GYRO_STD_FROM_PD	2

#define LSM6DSL_OUT_XYZ_SIZE		6

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define LSM6DSL_SIGN_MOTION_EN_ADDR	0x19
#define LSM6DSL_SIGN_MOTION_EN_MASK	0x01
#define LSM6DSL_SIGN_MOTION_DRDY_IRQ_MASK	0x40

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define LSM6DSL_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define LSM6DSL_STEP_COUNTER_OUT_L_ADDR		0x4b
#define LSM6DSL_STEP_COUNTER_OUT_SIZE	2
#define LSM6DSL_STEP_COUNTER_RES_ADDR	0x19
#define LSM6DSL_STEP_COUNTER_RES_MASK	0x06
#define LSM6DSL_STEP_COUNTER_RES_ALL_EN		0x03
#define LSM6DSL_STEP_COUNTER_RES_FUNC_EN	0x02
#define LSM6DSL_STEP_COUNTER_DURATION_ADDR	0x15

/* CUSTOM VALUES FOR TILT SENSOR */
#define LSM6DSL_TILT_EN_ADDR		0x19
#define LSM6DSL_TILT_EN_MASK		0x08
#define LSM6DSL_TILT_DRDY_IRQ_MASK	0x02

#define LSM6DSL_SRC_FUNC_ADDR		0x53
#define LSM6DSL_SRC2_FUNC_ADDR		0x54
#define LSM6DSL_SRC_SIGN_MOTION_DATA_AVL	0x40
#define LSM6DSL_SRC_TILT_DATA_AVL	0x20
#define LSM6DSL_SRC_STEP_DETECTOR_DATA_AVL	0x10
#define LSM6DSL_SRC_STEP_COUNTER_DATA_AVL	0x80

/* Sensor Software Reset Bit */
#define LSM6DSL_RESET_ADDR		0x12
#define LSM6DSL_RESET_MASK		0x01

static const struct lsm6dsl_sensor_name {
	const char *name;
	const char *description;
} lsm6dsl_sensor_name[LSM6DSL_SENSORS_NUMB] = {
	[LSM6DSL_ACCEL] = {
		.name = "accel",
		.description = "ST LSM6DSL Accelerometer Sensor",
	},
	[LSM6DSL_GYRO] = {
		.name = "gyro",
		.description = "ST LSM6DSL Gyroscope Sensor",
	},
	[LSM6DSL_SIGN_MOTION] = {
		.name = "sign_m",
		.description = "ST LSM6DSL Significant Motion Sensor",
	},
	[LSM6DSL_STEP_COUNTER] = {
		.name = "step_c",
		.description = "ST LSM6DSL Step Counter Sensor",
	},
	[LSM6DSL_STEP_DETECTOR] = {
		.name = "step_d",
		.description = "ST LSM6DSL Step Detector Sensor",
	},
	[LSM6DSL_TILT] = {
		.name = "tilt",
		.description = "ST LSM6DSL Tilt Sensor",
	},
};

struct lsm6dsl_odr_reg {
	u32 hz;
	u8 value;
};

static const struct lsm6dsl_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct lsm6dsl_odr_reg odr_avl[6];
} lsm6dsl_odr_table = {
	.addr[LSM6DSL_ACCEL] = LSM6DSL_ACC_ODR_ADDR,
	.mask[LSM6DSL_ACCEL] = LSM6DSL_ACC_ODR_MASK,
	.addr[LSM6DSL_GYRO] = LSM6DSL_GYR_ODR_ADDR,
	.mask[LSM6DSL_GYRO] = LSM6DSL_GYR_ODR_MASK,
	.odr_avl[0] = { .hz = 13, .value = LSM6DSL_ODR_13HZ_VAL },
	.odr_avl[1] = { .hz = 26, .value = LSM6DSL_ODR_26HZ_VAL },
	.odr_avl[2] = { .hz = 52, .value = LSM6DSL_ODR_52HZ_VAL },
	.odr_avl[3] = { .hz = 104, .value = LSM6DSL_ODR_104HZ_VAL },
	.odr_avl[4] = { .hz = 208, .value = LSM6DSL_ODR_208HZ_VAL },
	.odr_avl[5] = { .hz = 416, .value = LSM6DSL_ODR_416HZ_VAL },
};

struct lsm6dsl_fs_reg {
	unsigned int gain;
	u8 value;
	int urv;
};

static struct lsm6dsl_fs_table {
	u8 addr;
	u8 mask;
	struct lsm6dsl_fs_reg fs_avl[LSM6DSL_FS_LIST_NUM];
} lsm6dsl_fs_table[LSM6DSL_SENSORS_NUMB] = {
	[LSM6DSL_ACCEL] = {
		.addr = LSM6DSL_ACCEL_FS_ADDR,
		.mask = LSM6DSL_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DSL_ACCEL_FS_2G_GAIN,
			       .value = LSM6DSL_ACCEL_FS_2G_VAL,
			       .urv = 2, },
		.fs_avl[1] = { .gain = LSM6DSL_ACCEL_FS_4G_GAIN,
			       .value = LSM6DSL_ACCEL_FS_4G_VAL,
			       .urv = 4, },
		.fs_avl[2] = { .gain = LSM6DSL_ACCEL_FS_8G_GAIN,
			       .value = LSM6DSL_ACCEL_FS_8G_VAL,
			       .urv = 8, },
		.fs_avl[3] = { .gain = LSM6DSL_ACCEL_FS_16G_GAIN,
			       .value = LSM6DSL_ACCEL_FS_16G_VAL,
			       .urv = 16, },
	},
	[LSM6DSL_GYRO] = {
		.addr = LSM6DSL_GYRO_FS_ADDR,
		.mask = LSM6DSL_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DSL_GYRO_FS_250_GAIN,
			       .value = LSM6DSL_GYRO_FS_250_VAL,
			       .urv = 250, },
		.fs_avl[1] = { .gain = LSM6DSL_GYRO_FS_500_GAIN,
			       .value = LSM6DSL_GYRO_FS_500_VAL,
			       .urv = 500, },
		.fs_avl[2] = { .gain = LSM6DSL_GYRO_FS_1000_GAIN,
			       .value = LSM6DSL_GYRO_FS_1000_VAL,
			        .urv = 1000, },
		.fs_avl[3] = { .gain = LSM6DSL_GYRO_FS_2000_GAIN,
			       .value = LSM6DSL_GYRO_FS_2000_VAL,
			       .urv = 2000, },
	}
};

static struct workqueue_struct *lsm6dsl_workqueue;

static inline void lsm6dsl_flush_works(void)
{
	flush_workqueue(lsm6dsl_workqueue);
}

static inline int64_t lsm6dsl_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

static int lsm6dsl_write_data_with_mask(struct lsm6dsl_data *cdata,
					u8 reg_addr, u8 mask, u8 data, bool b_lock)
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

static int lsm6dsl_input_init(struct lsm6dsl_sensor_data *sdata, u16 bustype,
			      const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = lsm6dsl_sensor_name[sdata->sindex].description;

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit );
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if ((sdata->sindex == LSM6DSL_ACCEL) ||
	    (sdata->sindex == LSM6DSL_GYRO)) {
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

static void lsm6dsl_input_cleanup(struct lsm6dsl_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void lsm6dsl_report_3axes_event(struct lsm6dsl_sensor_data *sdata,
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

static void lsm6dsl_report_single_event(struct lsm6dsl_sensor_data *sdata,
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

static enum hrtimer_restart lsm6dsl_poll_function_read(struct hrtimer *timer)
{
	struct lsm6dsl_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer, struct lsm6dsl_sensor_data,
			     hr_timer);

	sdata->timestamp = lsm6dsl_get_time_ns();
	queue_work(lsm6dsl_workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}

static int lsm6dsl_get_step_c_data(struct lsm6dsl_sensor_data *sdata, u16 *steps)
{
	u8 data[2];
	int err = 0;
	err = sdata->cdata->tf->read(sdata->cdata,
				     LSM6DSL_STEP_COUNTER_OUT_L_ADDR,
				     LSM6DSL_STEP_COUNTER_OUT_SIZE,
				     data, true);
	if (err < 0)
		return err;

	*steps = data[0] | (data[1] << 8);

	return 0;
}

static int lsm6dsl_get_poll_data(struct lsm6dsl_sensor_data *sdata, u8 *data)
{
	int err = 0;
	u8 reg_addr;

	switch(sdata->sindex) {
	case LSM6DSL_ACCEL:
		reg_addr = LSM6DSL_ACCEL_OUT_X_L_ADDR;

		break;
	case LSM6DSL_GYRO:
		reg_addr = LSM6DSL_GYRO_OUT_X_L_ADDR;

		break;
	default:
		dev_err(sdata->cdata->dev, "invalid polling mode for sensor %s\n",
			sdata->name);
		return -1;
	}

	err = sdata->cdata->tf->read(sdata->cdata, reg_addr, LSM6DSL_OUT_XYZ_SIZE,
				     data, true);

	return err;
}

static void poll_function_work(struct work_struct *input_work)
{
	struct lsm6dsl_sensor_data *sdata;
	int xyz[3] = { 0 };
	ktime_t delta;
	u8 data[6];
	int err;

	sdata = container_of((struct work_struct *)input_work,
			     struct lsm6dsl_sensor_data, input_work);

	delta = ktime_set(0, lsm6dsl_get_time_ns() - sdata->timestamp);
	hrtimer_start(&sdata->hr_timer, ktime_sub(sdata->ktime, delta),
		      HRTIMER_MODE_REL);

	if(sdata->sample_to_discard) {
		sdata->sample_to_discard--;
		return;
	}

	err = lsm6dsl_get_poll_data(sdata, data);
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

		lsm6dsl_report_3axes_event(sdata, xyz, sdata->timestamp);
	}
}

static int lsm6dsl_set_drdy_irq(struct lsm6dsl_sensor_data *sdata, bool state)
{
	u8 reg_addr = 0, mask = 0, value;

	if (state)
		value = LSM6DSL_EN_BIT;
	else
		value = LSM6DSL_DIS_BIT;

	switch (sdata->sindex) {
	case LSM6DSL_ACCEL:
	case LSM6DSL_GYRO:
		return 0;

	/* Route Step Detection/sig. Motion Interrupt on INT1 */
	case LSM6DSL_SIGN_MOTION:
	case LSM6DSL_STEP_DETECTOR:
	case LSM6DSL_STEP_COUNTER:
		if (sdata->cdata->sensors[LSM6DSL_STEP_DETECTOR].enabled ||
		    sdata->cdata->sensors[LSM6DSL_SIGN_MOTION].enabled ||
		    sdata->cdata->sensors[LSM6DSL_STEP_COUNTER].enabled)
			return 0;

		reg_addr = LSM6DSL_INT1_CTRL_ADDR;
		mask = LSM6DSL_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case LSM6DSL_TILT:
		reg_addr = LSM6DSL_MD1_ADDR;
		mask = LSM6DSL_TILT_DRDY_IRQ_MASK;
		break;
	default:
		return -EINVAL;
	}

	return lsm6dsl_write_data_with_mask(sdata->cdata, reg_addr, mask, value,
					    true);
}

static int lsm6dsl_set_fs(struct lsm6dsl_sensor_data *sdata, u32 gain)
{
	int err, i;

	for (i = 0; i < LSM6DSL_FS_LIST_NUM; i++) {
		if (lsm6dsl_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}

	if (i == LSM6DSL_FS_LIST_NUM)
		return -EINVAL;

	err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_fs_table[sdata->sindex].addr,
				lsm6dsl_fs_table[sdata->sindex].mask,
				lsm6dsl_fs_table[sdata->sindex].fs_avl[i].value,
				true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

static irqreturn_t lsm6dsl_save_timestamp(int irq, void *private)
{
	struct lsm6dsl_data *cdata = (struct lsm6dsl_data *)private;

	cdata->timestamp = lsm6dsl_get_time_ns();
	queue_work(lsm6dsl_workqueue, &cdata->input_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static int lsm6dsl_disable_sensors(struct lsm6dsl_sensor_data *sdata);

static void lsm6dsl_irq_management(struct work_struct *input_work)
{
	struct lsm6dsl_data *cdata;
	u8 src_value = 0x00, src2_value = 0x00;
	struct lsm6dsl_sensor_data *sdata;
	u16 steps_c;
	int err;

	cdata = container_of((struct work_struct *)input_work,
			     struct lsm6dsl_data, input_work);

	cdata->tf->read(cdata, LSM6DSL_SRC_FUNC_ADDR, 1, &src_value, true);
	cdata->tf->read(cdata, LSM6DSL_SRC2_FUNC_ADDR, 1, &src2_value, true);

	if (src_value & LSM6DSL_SRC_STEP_COUNTER_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DSL_STEP_COUNTER];
		sdata->timestamp = cdata->timestamp;
		err = lsm6dsl_get_step_c_data(sdata, &steps_c);
		if (err < 0) {
			dev_err(cdata->dev,
				"error while reading step counter data\n");
			enable_irq(cdata->irq);

			return;
		}

		lsm6dsl_report_single_event(&cdata->sensors[LSM6DSL_STEP_COUNTER],
					    steps_c,
					    cdata->sensors[LSM6DSL_STEP_COUNTER].timestamp);
		cdata->steps_c = steps_c;
	}

	if (src_value & LSM6DSL_SRC_STEP_DETECTOR_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DSL_STEP_DETECTOR];
		sdata->timestamp = cdata->timestamp;
		lsm6dsl_report_single_event(sdata, 1, sdata->timestamp);

		if (cdata->sign_motion_event_ready) {
			sdata = &cdata->sensors[LSM6DSL_SIGN_MOTION];
			sdata->timestamp = cdata->timestamp;
			lsm6dsl_report_single_event(sdata, 1, sdata->timestamp);
			cdata->sign_motion_event_ready = false;
			lsm6dsl_disable_sensors(sdata);
		}
	}

	if (src_value & LSM6DSL_SRC_TILT_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DSL_TILT];
		sdata->timestamp = cdata->timestamp;
		lsm6dsl_report_single_event(sdata, 1, sdata->timestamp);
	}

	enable_irq(cdata->irq);
}

static int lsm6dsl_allocate_workqueue(struct lsm6dsl_data *cdata)
{
	int err;

	if (!lsm6dsl_workqueue)
		lsm6dsl_workqueue = create_workqueue(cdata->name);

	if (!lsm6dsl_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->input_work, lsm6dsl_irq_management);

	err = request_threaded_irq(cdata->irq, lsm6dsl_save_timestamp, NULL,
				   IRQF_TRIGGER_HIGH, cdata->name, cdata);
	if (err)
		return err;

	return 0;
}

static int lsm6dsl_set_extra_dependency(struct lsm6dsl_sensor_data *sdata,
					bool enable)
{
	int err;

	if (!(sdata->cdata->sensors[LSM6DSL_SIGN_MOTION].enabled |
	    sdata->cdata->sensors[LSM6DSL_STEP_COUNTER].enabled |
	    sdata->cdata->sensors[LSM6DSL_STEP_DETECTOR].enabled |
	    sdata->cdata->sensors[LSM6DSL_TILT].enabled)) {
		/* Enable/Disable Embedded Function only once */
		if (enable) {
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
						LSM6DSL_FUNC_EN_ADDR,
						LSM6DSL_FUNC_EN_MASK,
						LSM6DSL_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
						LSM6DSL_FUNC_EN_ADDR,
						LSM6DSL_FUNC_EN_MASK,
						LSM6DSL_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	if (!sdata->cdata->sensors[LSM6DSL_ACCEL].enabled) {
		if (enable) {
			u8 idx = 1;
			u16 acc_odr = sdata->cdata->sensors[LSM6DSL_ACCEL].c_odr;

			if (acc_odr > 26) {
				for (; idx < LSM6DSL_ODR_LIST_NUM; idx++)
					if (lsm6dsl_odr_table.odr_avl[idx].hz == acc_odr)
						break;
			}
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[LSM6DSL_ACCEL],
				lsm6dsl_odr_table.mask[LSM6DSL_ACCEL],
				lsm6dsl_odr_table.odr_avl[idx].value, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[LSM6DSL_ACCEL],
				lsm6dsl_odr_table.mask[LSM6DSL_ACCEL],
				LSM6DSL_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int lsm6dsl_enable_pedometer(struct lsm6dsl_sensor_data *sdata,
				    bool enable)
{
	int err = 0;
	u8 value = LSM6DSL_DIS_BIT;

	if (sdata->cdata->sensors[LSM6DSL_STEP_COUNTER].enabled &&
	    sdata->cdata->sensors[LSM6DSL_STEP_DETECTOR].enabled)
		return 0;

	if (enable)
		value = LSM6DSL_EN_BIT;

	err = lsm6dsl_write_data_with_mask(sdata->cdata,
					   LSM6DSL_FIFO_PEDO_E_ADDR,
					   LSM6DSL_FIFO_PEDO_E_MASK,
					   value, true);
	if (err < 0)
		return err;

	return lsm6dsl_write_data_with_mask(sdata->cdata,
					    LSM6DSL_PEDOMETER_EN_ADDR,
					    LSM6DSL_PEDOMETER_EN_MASK,
					    value, true);
}

static int _lsm6dsl_enable_sensors(struct lsm6dsl_sensor_data *sdata)
{
	int err, i;

	switch (sdata->sindex) {
	case LSM6DSL_ACCEL:
	case LSM6DSL_GYRO:
		for (i = 0; i < LSM6DSL_ODR_LIST_NUM; i++) {
			if (lsm6dsl_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == LSM6DSL_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->sindex == LSM6DSL_ACCEL)
			sdata->sample_to_discard = LSM6DSL_ACCEL_STD +
						LSM6DSL_ACCEL_STD_FROM_PD;

		sdata->cdata->sensors[LSM6DSL_GYRO].sample_to_discard =
						LSM6DSL_GYRO_STD +
						LSM6DSL_GYRO_STD_FROM_PD;

		err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[sdata->sindex],
				lsm6dsl_odr_table.mask[sdata->sindex],
				lsm6dsl_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
		sdata->c_odr = lsm6dsl_odr_table.odr_avl[i].hz;

		break;
	case LSM6DSL_SIGN_MOTION:
		err = lsm6dsl_write_data_with_mask(sdata->cdata,
						LSM6DSL_SIGN_MOTION_EN_ADDR,
						LSM6DSL_SIGN_MOTION_EN_MASK,
						LSM6DSL_EN_BIT, true);
		if (err < 0)
			return err;

		if ((sdata->cdata->sensors[LSM6DSL_STEP_COUNTER].enabled) ||
		    (sdata->cdata->sensors[LSM6DSL_STEP_DETECTOR].enabled)) {
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
						LSM6DSL_PEDOMETER_EN_ADDR,
						LSM6DSL_PEDOMETER_EN_MASK,
						LSM6DSL_DIS_BIT, true);
			if (err < 0)
				return err;

			err = lsm6dsl_write_data_with_mask(sdata->cdata,
						LSM6DSL_PEDOMETER_EN_ADDR,
						LSM6DSL_PEDOMETER_EN_MASK,
						LSM6DSL_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6dsl_enable_pedometer(sdata, true);
			if (err < 0)
				return err;
		}

		sdata->cdata->sign_motion_event_ready = true;

		break;
	case LSM6DSL_STEP_COUNTER:
	case LSM6DSL_STEP_DETECTOR:
		err = lsm6dsl_enable_pedometer(sdata, true);
		if (err < 0)
			return err;

		break;
	case LSM6DSL_TILT:
		err = lsm6dsl_write_data_with_mask(sdata->cdata,
					LSM6DSL_TILT_EN_ADDR,
					LSM6DSL_TILT_EN_MASK,
					LSM6DSL_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6dsl_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;

	err = lsm6dsl_set_drdy_irq(sdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int lsm6dsl_enable_sensors(struct lsm6dsl_sensor_data *sdata)
{
	int err;

	if (sdata->enabled)
		return 0;

	err = _lsm6dsl_enable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = true;

	return 0;
}

static int _lsm6dsl_disable_sensors(struct lsm6dsl_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case LSM6DSL_ACCEL:
		if (sdata->cdata->sensors[LSM6DSL_SIGN_MOTION].enabled |
		    sdata->cdata->sensors[LSM6DSL_STEP_COUNTER].enabled |
		    sdata->cdata->sensors[LSM6DSL_STEP_DETECTOR].enabled |
		    sdata->cdata->sensors[LSM6DSL_TILT].enabled) {
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[LSM6DSL_ACCEL],
				lsm6dsl_odr_table.mask[LSM6DSL_ACCEL],
				lsm6dsl_odr_table.odr_avl[0].value, true);
		} else {
			err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[LSM6DSL_ACCEL],
				lsm6dsl_odr_table.mask[LSM6DSL_ACCEL],
				LSM6DSL_ODR_POWER_OFF_VAL, true);
		}
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);

		break;
	case LSM6DSL_GYRO:
		err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[LSM6DSL_GYRO],
				lsm6dsl_odr_table.mask[LSM6DSL_GYRO],
				LSM6DSL_ODR_POWER_OFF_VAL, true);
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);

		break;
	case LSM6DSL_SIGN_MOTION:
		err = lsm6dsl_write_data_with_mask(sdata->cdata,
				LSM6DSL_SIGN_MOTION_EN_ADDR,
				LSM6DSL_SIGN_MOTION_EN_MASK,
				LSM6DSL_DIS_BIT, true);
		if (err < 0)
			return err;

		err = lsm6dsl_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		sdata->cdata->sign_motion_event_ready = false;

		break;
	case LSM6DSL_STEP_COUNTER:
	case LSM6DSL_STEP_DETECTOR:
		err = lsm6dsl_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case LSM6DSL_TILT:
		err = lsm6dsl_write_data_with_mask(sdata->cdata,
				LSM6DSL_TILT_EN_ADDR,
				LSM6DSL_TILT_EN_MASK,
				LSM6DSL_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6dsl_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	err = lsm6dsl_set_drdy_irq(sdata, false);
	if (err < 0)
		return err;

	return 0;
}

static int lsm6dsl_disable_sensors(struct lsm6dsl_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	err = _lsm6dsl_disable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = false;

	return 0;
}

static int lsm6dsl_reset_steps(struct lsm6dsl_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata, LSM6DSL_STEP_COUNTER_RES_ADDR, 1,
			      &reg_value, true);
	if (err < 0)
		return err;

	/* Check if embedded functionalities are enabled */
	if (reg_value & LSM6DSL_FUNC_EN_MASK)
		reg_value = LSM6DSL_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = LSM6DSL_DIS_BIT;

	err = lsm6dsl_write_data_with_mask(cdata,
				LSM6DSL_STEP_COUNTER_RES_ADDR,
				LSM6DSL_STEP_COUNTER_RES_MASK,
				LSM6DSL_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = lsm6dsl_write_data_with_mask(cdata,
				LSM6DSL_STEP_COUNTER_RES_ADDR,
				LSM6DSL_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

static int lsm6dsl_init_sensors(struct lsm6dsl_data *cdata)
{
	int err, i;
	u8 default_reg_value = 0;
	struct lsm6dsl_sensor_data *sdata;

	for (i = 0; i < LSM6DSL_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = lsm6dsl_disable_sensors(sdata);
		if (err < 0)
			return err;

		if ((sdata->sindex == LSM6DSL_ACCEL) ||
		    (sdata->sindex == LSM6DSL_GYRO)) {
			err = lsm6dsl_set_fs(sdata, sdata->c_gain);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[LSM6DSL_ACCEL].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	hrtimer_init(&cdata->sensors[LSM6DSL_GYRO].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	cdata->sensors[LSM6DSL_ACCEL].hr_timer.function =
						&lsm6dsl_poll_function_read;
	cdata->sensors[LSM6DSL_GYRO].hr_timer.function =
						&lsm6dsl_poll_function_read;

	cdata->steps_c = 0;
	cdata->reset_steps = false;

	/* Software reset */
	err = lsm6dsl_write_data_with_mask(cdata, LSM6DSL_RESET_ADDR,
					   LSM6DSL_RESET_MASK, LSM6DSL_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable Latch Mode Bit */
	err = lsm6dsl_write_data_with_mask(cdata, LSM6DSL_LIR_ADDR,
					   LSM6DSL_LIR_MASK, LSM6DSL_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable timestamp count */
	err = lsm6dsl_write_data_with_mask(cdata, LSM6DSL_TIMER_EN_ADDR,
					   LSM6DSL_TIMER_EN_MASK,
					   LSM6DSL_EN_BIT, true);
		if (err < 0)
			return err;

	/* Output data not updated until have been read */
	err = lsm6dsl_write_data_with_mask(cdata, LSM6DSL_BDU_ADDR,
					   LSM6DSL_BDU_MASK, LSM6DSL_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable Source Rounding function */
	err = lsm6dsl_write_data_with_mask(cdata, LSM6DSL_ROUNDING_ADDR,
					   LSM6DSL_ROUNDING_MASK,
					   LSM6DSL_EN_BIT, true);
	if (err < 0)
		return err;

	/* Set all interrupt signals in logic or on INT1 pad */
	err = lsm6dsl_write_data_with_mask(cdata,
					   LSM6DSL_INT2_ON_INT1_ADDR,
					   LSM6DSL_INT2_ON_INT1_MASK,
					   LSM6DSL_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6dsl_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);
	err = lsm6dsl_write_data_with_mask(sdata->cdata,
					   LSM6DSL_FUNC_CFG_ACCESS_ADDR,
					   LSM6DSL_FUNC_CFG_REG_MASK,
					   LSM6DSL_EN_BIT, false);
	if (err < 0)
		goto lsm6dsl_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
				      LSM6DSL_STEP_COUNTER_DURATION_ADDR,
				      1, &default_reg_value, false);
	if (err < 0)
		goto lsm6dsl_init_sensor_mutex_unlock;

	err = lsm6dsl_write_data_with_mask(sdata->cdata,
					   LSM6DSL_FUNC_CFG_ACCESS_ADDR,
					   LSM6DSL_FUNC_CFG_REG_MASK,
					   LSM6DSL_DIS_BIT, false);
	if (err < 0)
		goto lsm6dsl_init_sensor_mutex_unlock;

	mutex_unlock(&cdata->bank_registers_lock);
	cdata->sensors[LSM6DSL_ACCEL].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DSL_ACCEL].poll_interval));
	cdata->sensors[LSM6DSL_GYRO].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DSL_GYRO].poll_interval));
	INIT_WORK(&cdata->sensors[LSM6DSL_ACCEL].input_work, poll_function_work);
	INIT_WORK(&cdata->sensors[LSM6DSL_GYRO].input_work, poll_function_work);

	return 0;

lsm6dsl_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int lsm6dsl_set_odr(struct lsm6dsl_sensor_data *sdata, u32 odr)
{
	int err = 0, i;

	for (i = 0; i < LSM6DSL_ODR_LIST_NUM; i++) {
		if (lsm6dsl_odr_table.odr_avl[i].hz >= odr)
			break;
	}
	if (i == LSM6DSL_ODR_LIST_NUM)
		return -EINVAL;

	if (sdata->c_odr == lsm6dsl_odr_table.odr_avl[i].hz)
		return 0;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
		lsm6dsl_flush_works();

		if (sdata->sindex == LSM6DSL_ACCEL)
			sdata->cdata->sensors[LSM6DSL_ACCEL].sample_to_discard +=
							LSM6DSL_ACCEL_STD;

		if (sdata->cdata->sensors[LSM6DSL_GYRO].enabled)
			sdata->cdata->sensors[LSM6DSL_GYRO].sample_to_discard +=
							LSM6DSL_GYRO_STD;

		err = lsm6dsl_write_data_with_mask(sdata->cdata,
				lsm6dsl_odr_table.addr[sdata->sindex],
				lsm6dsl_odr_table.mask[sdata->sindex],
				lsm6dsl_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = lsm6dsl_odr_table.odr_avl[i].hz;
		enable_irq(sdata->cdata->irq);
	} else {
		sdata->c_odr = lsm6dsl_odr_table.odr_avl[i].hz;
	}

	return err;
}

static ssize_t get_enable(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	int err;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = lsm6dsl_enable_sensors(sdata);
	else
		err = lsm6dsl_disable_sensors(sdata);

	return count;
}

static ssize_t get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_interval);
}

static ssize_t set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int polling_rate;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	/*
	 * Polling interval is in msec, then we have to convert it in Hz to
	 * configure ODR through lsm6dsl_set_odr
	 */
	err = lsm6dsl_set_odr(sdata, 1000 / polling_rate);
	if (!(err < 0)) {
		sdata->poll_interval = 1000 / sdata->c_odr;
		sdata->ktime = ktime_set(0, MS_TO_NS(sdata->poll_interval));
	}
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t get_sampling_freq(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static ssize_t set_sampling_freq(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int err;
	unsigned int odr;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = lsm6dsl_set_odr(sdata, odr);
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t reset_steps(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t count)
{
	int err;
	unsigned int reset;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &reset);
	if (err < 0)
		return err;

	lsm6dsl_reset_steps(sdata->cdata);

	return count;
}

static ssize_t set_max_delivery_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	u8 duration;
	int err, err2;
	unsigned int max_delivery_rate;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtouint(buf, 10, &max_delivery_rate);
	if (err < 0)
		return -EINVAL;

	if (max_delivery_rate == sdata->c_odr)
		return size;

	duration = max_delivery_rate / LSM6DSL_MIN_DURATION_MS;

	mutex_lock(&sdata->cdata->bank_registers_lock);

	err = lsm6dsl_write_data_with_mask(sdata->cdata,
					   LSM6DSL_FUNC_CFG_ACCESS_ADDR,
					   LSM6DSL_FUNC_CFG_REG_MASK,
					   LSM6DSL_EN_BIT, false);
	if (err < 0)
		goto lsm6dsl_set_max_delivery_rate_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
				      LSM6DSL_STEP_COUNTER_DURATION_ADDR,
				      1, &duration, false);
	if (err < 0)
		goto lsm6dsl_set_max_delivery_rate_restore_bank;

	err = lsm6dsl_write_data_with_mask(sdata->cdata,
					   LSM6DSL_FUNC_CFG_ACCESS_ADDR,
					   LSM6DSL_FUNC_CFG_REG_MASK,
					   LSM6DSL_DIS_BIT, false);
	if (err < 0)
		goto lsm6dsl_set_max_delivery_rate_restore_bank;

	mutex_unlock(&sdata->cdata->bank_registers_lock);

	sdata->c_odr = max_delivery_rate;

	return size;

lsm6dsl_set_max_delivery_rate_restore_bank:
	do {
		err2 = lsm6dsl_write_data_with_mask(sdata->cdata,
					LSM6DSL_FUNC_CFG_ACCESS_ADDR,
					LSM6DSL_FUNC_CFG_REG_MASK,
					LSM6DSL_DIS_BIT, false);

		msleep(500);
	} while (err2 < 0);

lsm6dsl_set_max_delivery_rate_mutex_unlock:
	mutex_unlock(&sdata->cdata->bank_registers_lock);
	return err;
}

static ssize_t get_max_delivery_rate(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t get_sampling_frequency_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int i, len = 0;

	for (i = 0; i < LSM6DSL_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 lsm6dsl_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_scale_avail(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < LSM6DSL_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			lsm6dsl_fs_table[sdata->sindex].fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_cur_scale(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int i;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < LSM6DSL_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
				lsm6dsl_fs_table[sdata->sindex].fs_avl[i].gain)
			break;

	return sprintf(buf, "%d\n",
		       lsm6dsl_fs_table[sdata->sindex].fs_avl[i].urv);
}

static ssize_t set_cur_scale(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int i, urv, err;
	struct lsm6dsl_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	for (i = 0; i < LSM6DSL_FS_LIST_NUM; i++)
		if (urv == lsm6dsl_fs_table[sdata->sindex].fs_avl[i].urv)
			break;

	if (i == LSM6DSL_FS_LIST_NUM)
		return -EINVAL;

	err = lsm6dsl_set_fs(sdata,
			     lsm6dsl_fs_table[sdata->sindex].fs_avl[i].gain);
	if (err < 0)
		return err;

	return count;
}


static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, get_enable, set_enable);
static DEVICE_ATTR(sampling_freq, S_IWUSR | S_IRUGO, get_sampling_freq,
		   set_sampling_freq);
static DEVICE_ATTR(polling_rate, S_IWUSR | S_IRUGO, get_polling_rate,
		   set_polling_rate);
static DEVICE_ATTR(reset_steps, S_IWUSR, NULL, reset_steps);
static DEVICE_ATTR(max_delivery_rate, S_IWUSR | S_IRUGO, get_max_delivery_rate,
		   set_max_delivery_rate);
static DEVICE_ATTR(sampling_freq_avail, S_IRUGO, get_sampling_frequency_avail,
		   NULL);
static DEVICE_ATTR(scale_avail, S_IRUGO, get_scale_avail, NULL);
static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO, get_cur_scale, set_cur_scale);

static struct attribute *lsm6dsl_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm6dsl_gyro_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm6dsl_sign_m_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm6dsl_step_c_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_reset_steps.attr,
	&dev_attr_max_delivery_rate.attr,
	NULL,
};

static struct attribute *lsm6dsl_step_d_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm6dsl_tilt_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group lsm6dsl_attribute_groups[] = {
	[LSM6DSL_ACCEL] = {
		.attrs = lsm6dsl_accel_attribute,
		.name = "accel",
	},
	[LSM6DSL_GYRO] = {
		.attrs = lsm6dsl_gyro_attribute,
		.name = "gyro",
	},
	[LSM6DSL_SIGN_MOTION] = {
		.attrs = lsm6dsl_sign_m_attribute,
		.name = "sign_m",
	},
	[LSM6DSL_STEP_COUNTER] = {
		.attrs = lsm6dsl_step_c_attribute,
		.name = "step_c",
	},
	[LSM6DSL_STEP_DETECTOR] = {
		.attrs = lsm6dsl_step_d_attribute,
		.name = "step_d",
	},
	[LSM6DSL_TILT] = {
		.attrs = lsm6dsl_tilt_attribute,
		.name = "tilt",
	},
};

#ifdef CONFIG_OF
static u32 lsm6dsl_parse_dt(struct lsm6dsl_data *cdata)
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

int lsm6dsl_common_probe(struct lsm6dsl_data *cdata, int irq, u16 bustype)
{
	int32_t err, i;
	u8 wai = 0x00;
	struct lsm6dsl_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->tb.buf_lock);

	/* Read Chip ID register */
	err = cdata->tf->read(cdata, LSM6DSL_WHO_AM_I, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != LSM6DSL_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	if (irq > 0) {
#ifdef CONFIG_OF
		err = lsm6dsl_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lsm6dsl_platform_data *)
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

	for (i = 0; i < LSM6DSL_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lsm6dsl_sensor_name[i].name;
		if ((i == LSM6DSL_ACCEL) || (i == LSM6DSL_GYRO)) {
			sdata->c_odr = lsm6dsl_odr_table.odr_avl[0].hz;
			sdata->c_gain = lsm6dsl_fs_table[i].fs_avl[0].gain;
			sdata->poll_interval = 1000 / sdata->c_odr;
		}
		if (i == LSM6DSL_STEP_COUNTER) {
			sdata->c_odr = LSM6DSL_MIN_DURATION_MS;
		}

		if (lsm6dsl_input_init(sdata, bustype,
				       lsm6dsl_sensor_name[i].description)) {
			dev_err(cdata->dev,
				"failed to register input device %s",
				sdata->name);
			sdata->input_dev = NULL;
			continue;
		}

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
				       &lsm6dsl_attribute_groups[i])) {
			dev_err(cdata->dev,
				"failed to create sysfs group for sensor %s",
				sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	err = lsm6dsl_init_sensors(cdata);
	if (err < 0)
		return err;
	if (irq > 0)
		cdata->irq = irq;

	if (irq > 0) {
		err = lsm6dsl_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}

	dev_info(cdata->dev, "%s: probed\n", LSM6DSL_DEV_NAME);

	return 0;
}
EXPORT_SYMBOL(lsm6dsl_common_probe);

void lsm6dsl_common_remove(struct lsm6dsl_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < LSM6DSL_SENSORS_NUMB; i++) {
		lsm6dsl_disable_sensors(&cdata->sensors[i]);
		lsm6dsl_input_cleanup(&cdata->sensors[i]);
	}

	if(lsm6dsl_workqueue) {
		flush_workqueue(lsm6dsl_workqueue);
		destroy_workqueue(lsm6dsl_workqueue);
		lsm6dsl_workqueue = NULL;
	}
}
EXPORT_SYMBOL(lsm6dsl_common_remove);

#ifdef CONFIG_PM_SLEEP
static int lsm6dsl_resume_sensors(struct lsm6dsl_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _lsm6dsl_enable_sensors(sdata);
}

static int lsm6dsl_suspend_sensors(struct lsm6dsl_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _lsm6dsl_disable_sensors(sdata);
}

int lsm6dsl_common_suspend(struct lsm6dsl_data *cdata)
{
	lsm6dsl_suspend_sensors(&cdata->sensors[LSM6DSL_ACCEL]);
	lsm6dsl_suspend_sensors(&cdata->sensors[LSM6DSL_GYRO]);

	return 0;
}
EXPORT_SYMBOL(lsm6dsl_common_suspend);

int lsm6dsl_common_resume(struct lsm6dsl_data *cdata)
{
	lsm6dsl_resume_sensors(&cdata->sensors[LSM6DSL_ACCEL]);
	lsm6dsl_resume_sensors(&cdata->sensors[LSM6DSL_GYRO]);

	return 0;
}
EXPORT_SYMBOL(lsm6dsl_common_resume);

#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("STMicroelectronics lsm6dsl driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
