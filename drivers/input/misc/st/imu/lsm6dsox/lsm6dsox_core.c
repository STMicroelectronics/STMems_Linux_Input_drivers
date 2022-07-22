/*
 * STMicroelectronics lsm6dsox driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
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

#include "linux/platform_data/st/lsm6dsox.h"
#include "lsm6dsox_core.h"

static struct workqueue_struct *lsm6dsox_workqueue;

static const struct lsm6dsox_sensor_name {
	const char *name;
	const char *description;
} lsm6dsox_sensor_name[LSM6DSOX_MAX_SENSOR] = {
	[LSM6DSOX_ACCEL] = {
		.name = "accel",
		.description = "ST LSM6DSOX Accelerometer Sensor",
	},
	[LSM6DSOX_GYRO] = {
		.name = "gyro",
		.description = "ST LSM6DSOX Gyroscope Sensor",
	},
	[LSM6DSOX_SIGN_MOTION] = {
		.name = "sign_m",
		.description = "ST LSM6DSOX Significant Motion Sensor",
	},
	[LSM6DSOX_STEP_COUNTER] = {
		.name = "step_c",
		.description = "ST LSM6DSOX Step Counter Sensor",
	},
	[LSM6DSOX_STEP_DETECTOR] = {
		.name = "step_d",
		.description = "ST LSM6DSOX Step Detector Sensor",
	},
	[LSM6DSOX_TILT] = {
		.name = "tilt",
		.description = "ST LSM6DSOX Tilt Sensor",
	},
};

static const struct lsm6dsox_odr_table_t lsm6dsox_odr_table = {
	.addr[LSM6DSOX_ACCEL] = LSM6DSOX_CTRL1_XL_ADDR,
	.mask[LSM6DSOX_ACCEL] = LSM6DSOX_ODR_XL_MASK,
	.addr[LSM6DSOX_GYRO] = LSM6DSOX_CTRL2_G_ADDR,
	.mask[LSM6DSOX_GYRO] = LSM6DSOX_ODR_G_MASK,
	.odr_avl[0] = { .hz = 13, .value = 0x01 },
	.odr_avl[1] = { .hz = 26, .value = 0x02 },
	.odr_avl[2] = { .hz = 52, .value = 0x03 },
	.odr_avl[3] = { .hz = 104, .value = 0x04 },
	.odr_avl[4] = { .hz = 208, .value = 0x05 },
	.odr_avl[5] = { .hz = 416, .value = 0x06 },
};

static const struct lsm6dsox_settings lsm6dsox_sensor_settings[] = {
	{
		.id = {
			{
				.hw_id = LSM6DSO_ID,
				.name = LSM6DSO_DEV_NAME,
			},
			{
				.hw_id = LSM6DSOX_ID,
				.name = LSM6DSOX_DEV_NAME,
			},
		},
		.fs_table = {
			[LSM6DSOX_ACCEL] = {
				.addr = LSM6DSOX_CTRL1_XL_ADDR,
				.mask = LSM6DSOX_FS_XL_MASK,
				.fs_avl[0] = { 61,  0x0,  2 },
				.fs_avl[1] = { 122, 0x2,  4 },
				.fs_avl[2] = { 244, 0x3,  8 },
				.fs_avl[3] = { 488, 0x1, 16 },
			},
			[LSM6DSOX_GYRO] = {
				.addr = LSM6DSOX_CTRL2_G_ADDR,
				.mask = LSM6DSOX_FS_G_MASK,
				.fs_avl[0] = {  8750, 0x0,  8750 },
				.fs_avl[1] = { 17500, 0x1, 17500 },
				.fs_avl[2] = { 35000, 0x2, 35000 },
				.fs_avl[3] = { 70000, 0x3, 70000 },
			},
		},
	},
	{
		.id = {
			{
				.hw_id = LSM6DSO32_ID,
				.name = LSM6DSO32_DEV_NAME,
			},
			{
				.hw_id = LSM6DSO32X_ID,
				.name = LSM6DSO32X_DEV_NAME,
			},
		},
		.fs_table = {
			[LSM6DSOX_ACCEL] = {
				.addr = LSM6DSOX_CTRL1_XL_ADDR,
				.mask = LSM6DSOX_FS_XL_MASK,
				.fs_avl[0] = { 122, 0x0,  4 },
				.fs_avl[1] = { 244, 0x2,  8 },
				.fs_avl[2] = { 488, 0x3, 16 },
				.fs_avl[3] = { 976, 0x1, 32 },
			},
			[LSM6DSOX_GYRO] = {
				.addr = LSM6DSOX_CTRL2_G_ADDR,
				.mask = LSM6DSOX_FS_G_MASK,
				.fs_avl[0] = {  8750, 0x0,  8750 },
				.fs_avl[1] = { 17500, 0x1, 17500 },
				.fs_avl[2] = { 35000, 0x2, 35000 },
				.fs_avl[3] = { 70000, 0x3, 70000 },
			},
		},
	},
};

static int lsm6dsox_disable_sensors(struct lsm6dsox_sensor_data *sdata);

static inline void lsm6dsox_flush_works(void)
{
	flush_workqueue(lsm6dsox_workqueue);
}

static inline int64_t lsm6dsox_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

static int lsm6dsox_write_data_with_mask(struct lsm6dsox_data *cdata,
					 u8 reg_addr, u8 mask, u8 data,
					 bool b_lock)
{
	u8 new_data, old_data;
	int err;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) |
		    ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}

/*
 * lsm6dsox_write_emb_with_mask - write to embedded function register
 *
 * write with mask to embedded function register page while locking
 * reg_lock mutex
 */
static int lsm6dsox_write_emb_with_mask(struct lsm6dsox_data *cdata,
					u8 reg_addr, u8 mask, u8 data)
{
	u8 page, new_data, old_data;
	int err;

	mutex_lock(&cdata->reg_lock);
	page = LSM6DSOX_FUNC_CFG_REG_MASK;
	err = cdata->tf->write(cdata, LSM6DSOX_FUNC_CFG_ACCESS_ADDR,
			       1, &page, false);
	if (err < 0)
		goto unlock;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, false);
	if (err < 0)
		goto unlock;

	new_data = ((old_data & (~mask)) |
		    ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		goto unlock;

	err = cdata->tf->write(cdata, reg_addr, 1, &new_data, false);

unlock:
	page = 0;
	cdata->tf->write(cdata, LSM6DSOX_FUNC_CFG_ACCESS_ADDR,
			 1, &page, false);
	mutex_unlock(&cdata->reg_lock);

	return err;
}

static int lsm6dsox_input_init(struct lsm6dsox_sensor_data *sdata,
			       u16 bustype, const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev,
			"failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name =
			lsm6dsox_sensor_name[sdata->sindex].description;

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit);
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if ((sdata->sindex == LSM6DSOX_ACCEL) ||
	    (sdata->sindex == LSM6DSOX_GYRO)) {
		__set_bit(INPUT_EVENT_Y, sdata->input_dev->mscbit);
		__set_bit(INPUT_EVENT_Z, sdata->input_dev->mscbit);
	}

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev,
			"unable to register sensor %s\n",
			sdata->name);
		input_free_device(sdata->input_dev);
	}

	return err;
}

static void lsm6dsox_input_cleanup(struct lsm6dsox_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void
lsm6dsox_report_3axes_event(struct lsm6dsox_sensor_data *sdata,
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

static void
lsm6dsox_report_single_event(struct lsm6dsox_sensor_data *sdata,
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

static enum hrtimer_restart
lsm6dsox_poll_function_read(struct hrtimer *timer)
{
	struct lsm6dsox_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer,
			     struct lsm6dsox_sensor_data,
			     hr_timer);

	sdata->timestamp = lsm6dsox_get_time_ns();
	queue_work(lsm6dsox_workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}

static int lsm6dsox_get_step_c_data(struct lsm6dsox_sensor_data *sdata,
				    u16 *steps)
{
	struct lsm6dsox_data *cdata = sdata->cdata;
	u8 data[2], page;
	int err;

	mutex_lock(&cdata->reg_lock);
	page = LSM6DSOX_FUNC_CFG_REG_MASK;
	err = cdata->tf->write(cdata, LSM6DSOX_FUNC_CFG_ACCESS_ADDR,
			       1, &page, false);
	if (err < 0)
		goto unlock;

	err = cdata->tf->read(cdata, LSM6DSOX_STEP_COUNTER_L_ADDR,
			      sizeof(data), data, false);
	if (err < 0)
		goto unlock;

	*steps = data[0] | (data[1] << 8);

unlock:
	page = 0;
	cdata->tf->write(cdata, LSM6DSOX_FUNC_CFG_ACCESS_ADDR,
			 1, &page, false);
	mutex_unlock(&cdata->reg_lock);

	return err;
}

static int lsm6dsox_get_poll_data(struct lsm6dsox_sensor_data *sdata,
				  u8 *data)
{
	u8 reg_addr;

	switch (sdata->sindex) {
	case LSM6DSOX_ACCEL:
		reg_addr = LSM6DSOX_OUTX_L_A_ADDR;
		break;
	case LSM6DSOX_GYRO:
		reg_addr = LSM6DSOX_OUTX_L_G_ADDR;
		break;
	default:
		dev_err(sdata->cdata->dev,
			"invalid polling mode for sensor %s\n",
			sdata->name);
		return -1;
	}

	return sdata->cdata->tf->read(sdata->cdata, reg_addr,
				      LSM6DSOX_OUT_XYZ_SIZE,
				      data, true);
}

static void poll_function_work(struct work_struct *input_work)
{
	struct lsm6dsox_sensor_data *sdata;
	int xyz[3] = { 0 };
	ktime_t tmpkt;
	u8 data[6];
	int err;

	sdata = container_of((struct work_struct *)input_work,
			     struct lsm6dsox_sensor_data, input_work);

	tmpkt = ktime_sub(sdata->oldktime,
			  ktime_set(0, (lsm6dsox_get_time_ns() -
					sdata->timestamp)));

	hrtimer_start(&sdata->hr_timer, tmpkt, HRTIMER_MODE_REL);
	if (sdata->sample_to_discard) {
		sdata->sample_to_discard--;

		return;
	}

	err = lsm6dsox_get_poll_data(sdata, data);
	if (err < 0) {
		dev_err(sdata->cdata->dev, "get %s data failed %d\n",
			sdata->name, err);
	} else {
		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;

		lsm6dsox_report_3axes_event(sdata, xyz,
					    sdata->timestamp);
	}
}

static int lsm6dsox_set_drdy_irq(struct lsm6dsox_sensor_data *sdata,
				 bool state)
{
	u8 reg_addr;
	u8 mask;

	switch (sdata->sindex) {
	case LSM6DSOX_ACCEL:
	case LSM6DSOX_GYRO:
		return 0;
	case LSM6DSOX_STEP_COUNTER:
	case LSM6DSOX_STEP_DETECTOR:
		if (sdata->cdata->sensors[LSM6DSOX_STEP_DETECTOR].enabled ||
		    sdata->cdata->sensors[LSM6DSOX_STEP_COUNTER].enabled)
			return 0;

		if (sdata->cdata->drdy_int_pin == 1) {
			reg_addr = LSM6DSOX_EMB_FUNC_INT1;
			mask = LSM6DSOX_INT1_STEP_DETECTOR_MASK;
		} else {
			reg_addr = LSM6DSOX_EMB_FUNC_INT2;
			mask = LSM6DSOX_INT2_STEP_DETECTOR_MASK;
		}
		break;
	case LSM6DSOX_SIGN_MOTION:
		if (sdata->cdata->sensors[LSM6DSOX_SIGN_MOTION].enabled)
			return 0;

		if (sdata->cdata->drdy_int_pin == 1) {
			reg_addr = LSM6DSOX_EMB_FUNC_INT1;
			mask = LSM6DSOX_INT1_SIG_MOT_MASK;
		} else {
			reg_addr = LSM6DSOX_EMB_FUNC_INT2;
			mask = LSM6DSOX_INT2_SIG_MOT_MASK;
		}
		break;
	case LSM6DSOX_TILT:
		if (sdata->cdata->drdy_int_pin == 1) {
			reg_addr = LSM6DSOX_EMB_FUNC_INT1;
			mask = LSM6DSOX_INT1_TILT_MASK;
		} else {
			reg_addr = LSM6DSOX_EMB_FUNC_INT2;
			mask = LSM6DSOX_INT2_TILT_MASK;
		}
		break;
	default:
		return -EINVAL;
	}

	return lsm6dsox_write_emb_with_mask(sdata->cdata, reg_addr,
					    mask, state ? 1 : 0);
}

static int lsm6dsox_set_fs(struct lsm6dsox_sensor_data *sdata, u32 gain)
{
	const struct lsm6dsox_fs_table *fs_table;
	int err, i;

	fs_table = &sdata->cdata->settings->fs_table[sdata->sindex];

	for (i = 0; i < LSM6DSOX_FS_LIST_NUM; i++) {
		if (fs_table->fs_avl[i].gain == gain)
			break;
	}

	if (i == LSM6DSOX_FS_LIST_NUM)
		return -EINVAL;

	err = lsm6dsox_write_data_with_mask(sdata->cdata,
		       fs_table->addr,
		       fs_table->mask,
		       fs_table->fs_avl[i].value,
		       true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

static irqreturn_t lsm6dsox_save_timestamp(int irq, void *private)
{
	struct lsm6dsox_data *cdata = (struct lsm6dsox_data *)private;

	cdata->timestamp = lsm6dsox_get_time_ns();
	queue_work(lsm6dsox_workqueue, &cdata->input_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static void lsm6dsox_irq_management(struct work_struct *input_work)
{
	struct lsm6dsox_sensor_data *sdata;
	struct lsm6dsox_data *cdata;
	u8 src_value = 0x00;
	u16 steps_c = 0;
	int err;

	cdata = container_of((struct work_struct *)input_work,
			     struct lsm6dsox_data, input_work);
	cdata->tf->read(cdata, LSM6DSOX_EMB_FUNC_STATUS_MAINPAGE_ADDR,
			1, &src_value, true);

	if (src_value & LSM6DSOX_IS_STEP_DET_MASK) {
		sdata = &cdata->sensors[LSM6DSOX_STEP_COUNTER];
		sdata->timestamp = cdata->timestamp;
		err = lsm6dsox_get_step_c_data(sdata, &steps_c);
		if (err < 0) {
			dev_err(cdata->dev,
				"error reading step counter data\n");
			enable_irq(cdata->irq);

			return;
		}

		lsm6dsox_report_single_event(&cdata->sensors[LSM6DSOX_STEP_COUNTER],
					     steps_c,
					     cdata->sensors[LSM6DSOX_STEP_COUNTER].timestamp);
		cdata->steps_c = steps_c;
	}

	if (src_value & LSM6DSOX_IS_STEP_DET_MASK) {
		sdata = &cdata->sensors[LSM6DSOX_STEP_DETECTOR];
		sdata->timestamp = cdata->timestamp;
		lsm6dsox_report_single_event(sdata, 1, sdata->timestamp);

		if (cdata->sign_motion_event_ready) {
			sdata = &cdata->sensors[LSM6DSOX_SIGN_MOTION];
			sdata->timestamp = cdata->timestamp;
			lsm6dsox_report_single_event(sdata, 1,
						     sdata->timestamp);
			cdata->sign_motion_event_ready = false;
			lsm6dsox_disable_sensors(sdata);
		}
	}

	if (src_value & LSM6DSOX_IS_TILT_MASK) {
		sdata = &cdata->sensors[LSM6DSOX_TILT];
		sdata->timestamp = cdata->timestamp;
		lsm6dsox_report_single_event(sdata, 1,
					     sdata->timestamp);
	}

	enable_irq(cdata->irq);
}

static int lsm6dsox_allocate_workqueue(struct lsm6dsox_data *cdata)
{
	int err;

	if (!lsm6dsox_workqueue)
		lsm6dsox_workqueue = create_workqueue(cdata->name);

	if (!lsm6dsox_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->input_work, lsm6dsox_irq_management);

	err = request_threaded_irq(cdata->irq, lsm6dsox_save_timestamp,
				   NULL, IRQF_TRIGGER_HIGH, cdata->name,
				   cdata);
	if (err)
		return err;

	return 0;
}

static int
lsm6dsox_set_extra_dependency(struct lsm6dsox_sensor_data *sdata,
			      bool enable)
{
	int err;

	if (!sdata->cdata->sensors[LSM6DSOX_ACCEL].enabled) {
		if (enable) {
			u8 idx = 1;
			u16 acc_odr = sdata->cdata->sensors[LSM6DSOX_ACCEL].c_odr;

			if (acc_odr > 26) {
				for (; idx < LSM6DSOX_ODR_LIST_NUM; idx++)
					if (lsm6dsox_odr_table.odr_avl[idx].hz == acc_odr)
						break;
			}
			err = lsm6dsox_write_data_with_mask(sdata->cdata,
				lsm6dsox_odr_table.addr[LSM6DSOX_ACCEL],
				lsm6dsox_odr_table.mask[LSM6DSOX_ACCEL],
				lsm6dsox_odr_table.odr_avl[idx].value, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6dsox_write_data_with_mask(sdata->cdata,
				lsm6dsox_odr_table.addr[LSM6DSOX_ACCEL],
				lsm6dsox_odr_table.mask[LSM6DSOX_ACCEL],
				0, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int lsm6dsox_enable_pedometer(struct lsm6dsox_sensor_data *sdata,
				     bool enable)
{
	u8 value = enable ? 1 : 0;

	if (sdata->cdata->sensors[LSM6DSOX_STEP_COUNTER].enabled &&
	    sdata->cdata->sensors[LSM6DSOX_STEP_DETECTOR].enabled)
		return 0;

	return lsm6dsox_write_emb_with_mask(sdata->cdata,
					    LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					    LSM6DSOX_PEDO_EN_MASK,
					    value);
}

static int _lsm6dsox_enable_sensors(struct lsm6dsox_sensor_data *sdata)
{
	int64_t newTime;
	int err, i;

	switch (sdata->sindex) {
	case LSM6DSOX_ACCEL:
	case LSM6DSOX_GYRO:
		for (i = 0; i < LSM6DSOX_ODR_LIST_NUM; i++) {
			if (lsm6dsox_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}

		if (i == LSM6DSOX_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->sindex == LSM6DSOX_ACCEL)
			sdata->sample_to_discard = LSM6DSOX_ACCEL_STD +
					     LSM6DSOX_ACCEL_STD_FROM_PD;

		sdata->cdata->sensors[LSM6DSOX_GYRO].sample_to_discard =
					      LSM6DSOX_GYRO_STD +
					      LSM6DSOX_GYRO_STD_FROM_PD;

		err = lsm6dsox_write_data_with_mask(sdata->cdata,
			     lsm6dsox_odr_table.addr[sdata->sindex],
			     lsm6dsox_odr_table.mask[sdata->sindex],
			     lsm6dsox_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		sdata->c_odr = lsm6dsox_odr_table.odr_avl[i].hz;
		newTime = 1000000000 / sdata->c_odr;
		sdata->oldktime = ktime_set(0, newTime);
		hrtimer_start(&sdata->hr_timer, sdata->oldktime,
			      HRTIMER_MODE_REL);

		break;
	case LSM6DSOX_SIGN_MOTION:
		err = lsm6dsox_write_emb_with_mask(sdata->cdata,
					LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					LSM6DSOX_SIGN_MOTION_EN_MASK,
					1);
		if (err < 0)
			return err;

		err = lsm6dsox_write_emb_with_mask(sdata->cdata,
					    LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					    LSM6DSOX_PEDO_EN_MASK, 1);
		if (err < 0)
			return err;

		sdata->cdata->sign_motion_event_ready = true;
		break;
	case LSM6DSOX_STEP_COUNTER:
	case LSM6DSOX_STEP_DETECTOR:
		err = lsm6dsox_enable_pedometer(sdata, true);
		if (err < 0)
			return err;
		break;
	case LSM6DSOX_TILT:
		err = lsm6dsox_write_emb_with_mask(sdata->cdata,
					    LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					    LSM6DSOX_TILT_EN_MASK, 1);
		if (err < 0)
			return err;
		break;
	default:
		return -EINVAL;
	}

	err = lsm6dsox_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;

	err = lsm6dsox_set_drdy_irq(sdata, true);

	return err < 0 ? err : 0;
}

static int lsm6dsox_enable_sensors(struct lsm6dsox_sensor_data *sdata)
{
	int err;

	if (sdata->enabled)
		return 0;

	err = _lsm6dsox_enable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = true;

	return 0;
}

static int _lsm6dsox_disable_sensors(struct lsm6dsox_sensor_data *sdata)
{
	int err;

	switch (sdata->sindex) {
	case LSM6DSOX_ACCEL:
		if (sdata->cdata->sensors[LSM6DSOX_SIGN_MOTION].enabled |
		    sdata->cdata->sensors[LSM6DSOX_STEP_COUNTER].enabled |
		    sdata->cdata->sensors[LSM6DSOX_STEP_DETECTOR].enabled |
		    sdata->cdata->sensors[LSM6DSOX_TILT].enabled) {
			err = lsm6dsox_write_data_with_mask(sdata->cdata,
				lsm6dsox_odr_table.addr[LSM6DSOX_ACCEL],
				lsm6dsox_odr_table.mask[LSM6DSOX_ACCEL],
				lsm6dsox_odr_table.odr_avl[0].value,
				true);
		} else {
			err = lsm6dsox_write_data_with_mask(sdata->cdata,
				lsm6dsox_odr_table.addr[LSM6DSOX_ACCEL],
				lsm6dsox_odr_table.mask[LSM6DSOX_ACCEL],
				0, true);
		}

		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);
		break;
	case LSM6DSOX_GYRO:
		err = lsm6dsox_write_data_with_mask(sdata->cdata,
				 lsm6dsox_odr_table.addr[LSM6DSOX_GYRO],
				 lsm6dsox_odr_table.mask[LSM6DSOX_GYRO],
				 0, true);
		if (err < 0)
			return err;

		cancel_work_sync(&sdata->input_work);
		hrtimer_cancel(&sdata->hr_timer);
		break;
	case LSM6DSOX_SIGN_MOTION:
		err = lsm6dsox_write_emb_with_mask(sdata->cdata,
				       LSM6DSOX_EMB_FUNC_EN_A_ADDR,
				       LSM6DSOX_SIGN_MOTION_EN_MASK, 0);
		if (err < 0)
			return err;

		err = lsm6dsox_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		sdata->cdata->sign_motion_event_ready = false;
		break;
	case LSM6DSOX_STEP_COUNTER:
	case LSM6DSOX_STEP_DETECTOR:
		err = lsm6dsox_enable_pedometer(sdata, false);
		if (err < 0)
			return err;
		break;
	case LSM6DSOX_TILT:
		err = lsm6dsox_write_emb_with_mask(sdata->cdata,
					LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					LSM6DSOX_TILT_EN_MASK, 0);
		if (err < 0)
			return err;
		break;
	default:
		return -EINVAL;
	}

	err = lsm6dsox_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	err = lsm6dsox_set_drdy_irq(sdata, false);

	return err < 0 ? err : 0;
}

static int lsm6dsox_disable_sensors(struct lsm6dsox_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	err = _lsm6dsox_disable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = false;

	return 0;
}

static int lsm6dsox_reset_steps(struct lsm6dsox_data *cdata)
{
	int err;

	err = lsm6dsox_write_emb_with_mask(cdata,
					   LSM6DSOX_EMB_FUNC_SRC_ADDR,
					   LSM6DSOX_PEDO_RST_STEP_MASK,
					   1);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

static int lsm6dsox_init_sensors(struct lsm6dsox_data *cdata)
{
	struct lsm6dsox_sensor_data *sdata;
	int err, i;

	for (i = 0; i < LSM6DSOX_MAX_SENSOR; i++) {
		sdata = &cdata->sensors[i];

		err = lsm6dsox_disable_sensors(sdata);
		if (err < 0)
			return err;

		if ((sdata->sindex == LSM6DSOX_ACCEL) ||
		    (sdata->sindex == LSM6DSOX_GYRO)) {
			err = lsm6dsox_set_fs(sdata, sdata->c_gain);
			if (err < 0)
				return err;
		}
	}

	hrtimer_init(&cdata->sensors[LSM6DSOX_ACCEL].hr_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&cdata->sensors[LSM6DSOX_GYRO].hr_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cdata->sensors[LSM6DSOX_ACCEL].hr_timer.function =
					&lsm6dsox_poll_function_read;
	cdata->sensors[LSM6DSOX_GYRO].hr_timer.function =
					&lsm6dsox_poll_function_read;

	cdata->steps_c = 0;
	cdata->reset_steps = false;

	err = lsm6dsox_write_data_with_mask(cdata,
					    LSM6DSOX_CTRL3_C_ADDR,
					    LSM6DSOX_SW_RESET_MASK,
					    1, true);
	if (err < 0)
		return err;

	err = lsm6dsox_write_data_with_mask(cdata,
					    LSM6DSOX_TAP_CFG0_ADDR,
					    LSM6DSOX_LIR_MASK,
					    1, true);
	if (err < 0)
		return err;

	err = lsm6dsox_write_data_with_mask(cdata,
					    LSM6DSOX_CTRL10_C_ADDR,
					    LSM6DSOX_TIMESTAMP_EN_MASK,
					    1, true);
		if (err < 0)
			return err;

	err = lsm6dsox_write_data_with_mask(cdata,
					    LSM6DSOX_CTRL3_C_ADDR,
					    LSM6DSOX_BDU_MASK,
					    1, true);
	if (err < 0)
		return err;

	err = lsm6dsox_write_data_with_mask(cdata,
					    LSM6DSOX_CTRL5_C_ADDR,
					    LSM6DSOX_ROUNDING_MASK,
					    0x03,
					    true);
	if (err < 0)
		return err;

	err = lsm6dsox_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	/* enable emb. func. latched interrupts */
	err  = lsm6dsox_write_emb_with_mask(cdata,
					    LSM6DSOX_PAGE_RW_ADDR,
					    LSM6DSOX_EMB_FUNC_LIR_MASK,
					    1);
	if (err < 0)
		return err;

	/* enable embedded function interrupts enable */
	if (cdata->drdy_int_pin == 1) {
		err = lsm6dsox_write_data_with_mask(cdata,
						    LSM6DSOX_MD1_CFG_ADDR,
						    LSM6DSOX_INT1_EMB_FUNC_MASK,
						    1, true);
	} else {
		err = lsm6dsox_write_data_with_mask(cdata,
						    LSM6DSOX_MD2_CFG_ADDR,
						    LSM6DSOX_INT2_EMB_FUNC_MASK,
						    1, true);
	}

	if (err < 0)
		return err;

	cdata->sensors[LSM6DSOX_ACCEL].oldktime = ktime_set(0,
		MS_TO_NS(cdata->sensors[LSM6DSOX_ACCEL].poll_interval));
	cdata->sensors[LSM6DSOX_GYRO].oldktime = ktime_set(0,
		MS_TO_NS(cdata->sensors[LSM6DSOX_GYRO].poll_interval));
	INIT_WORK(&cdata->sensors[LSM6DSOX_ACCEL].input_work,
		  poll_function_work);
	INIT_WORK(&cdata->sensors[LSM6DSOX_GYRO].input_work,
		  poll_function_work);

	return 0;
}

static int lsm6dsox_set_odr(struct lsm6dsox_sensor_data *sdata, u32 odr)
{
	int err = 0, i;

	for (i = 0; i < LSM6DSOX_ODR_LIST_NUM; i++) {
		if (lsm6dsox_odr_table.odr_avl[i].hz >= odr)
			break;
	}

	if (i == LSM6DSOX_ODR_LIST_NUM)
		return -EINVAL;

	if (sdata->c_odr == lsm6dsox_odr_table.odr_avl[i].hz)
		return 0;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
		lsm6dsox_flush_works();

		if (sdata->sindex == LSM6DSOX_ACCEL)
			sdata->cdata->sensors[LSM6DSOX_ACCEL].sample_to_discard +=
						     LSM6DSOX_ACCEL_STD;

		if (sdata->cdata->sensors[LSM6DSOX_GYRO].enabled)
			sdata->cdata->sensors[LSM6DSOX_GYRO].sample_to_discard +=
						      LSM6DSOX_GYRO_STD;

		err = lsm6dsox_write_data_with_mask(sdata->cdata,
				lsm6dsox_odr_table.addr[sdata->sindex],
				lsm6dsox_odr_table.mask[sdata->sindex],
				lsm6dsox_odr_table.odr_avl[i].value,
				true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = lsm6dsox_odr_table.odr_avl[i].hz;
		enable_irq(sdata->cdata->irq);
	} else {
		sdata->c_odr = lsm6dsox_odr_table.odr_avl[i].hz;
	}

	return err;
}

static ssize_t get_enable(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t set_enable(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;
	int err;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = lsm6dsox_enable_sensors(sdata);
	else
		err = lsm6dsox_disable_sensors(sdata);

	return count;
}

static ssize_t get_polling_rate(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_interval);
}

static ssize_t set_polling_rate(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned int polling_rate;
	int64_t newTime;
	int err;

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = lsm6dsox_set_odr(sdata, 1000 / polling_rate);
	if (!(err < 0)) {
		sdata->poll_interval = 1000 / sdata->c_odr;
		newTime = MS_TO_NS(sdata->poll_interval);
		sdata->oldktime = ktime_set(0, newTime);
	}
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t get_sampling_freq(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static ssize_t set_sampling_freq(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned int odr;
	int err;

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = lsm6dsox_set_odr(sdata, odr);
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t reset_steps(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned int reset;
	int err;

	err = kstrtoint(buf, 10, &reset);
	if (err < 0)
		return err;

	lsm6dsox_reset_steps(sdata->cdata);

	return count;
}

static ssize_t
sampling_freq_avail_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	int i, len = 0;

	for (i = 0; i < LSM6DSOX_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 lsm6dsox_odr_table.odr_avl[i].hz);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_scale_avail(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	const struct lsm6dsox_fs_table *fs_table;
	int i, len = 0;

	fs_table = &sdata->cdata->settings->fs_table[sdata->sindex];

	for (i = 0; i < LSM6DSOX_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 fs_table->fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t scale_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	const struct lsm6dsox_fs_table *fs_table;
	int i;

	fs_table = &sdata->cdata->settings->fs_table[sdata->sindex];

	for (i = 0; i < LSM6DSOX_FS_LIST_NUM; i++)
		if (sdata->c_gain ==
				 fs_table->fs_avl[i].gain)
			break;

	return sprintf(buf, "%d\n",
		       fs_table->fs_avl[i].urv);
}

static ssize_t
scale_store(struct device *dev, struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct lsm6dsox_sensor_data *sdata = dev_get_drvdata(dev);
	const struct lsm6dsox_fs_table *fs_table;
	int i, urv, err;

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	fs_table = &sdata->cdata->settings->fs_table[sdata->sindex];
	for (i = 0; i < LSM6DSOX_FS_LIST_NUM; i++)
		if (urv == fs_table->fs_avl[i].urv)
			break;

	if (i == LSM6DSOX_FS_LIST_NUM)
		return -EINVAL;

	err = lsm6dsox_set_fs(sdata, fs_table->fs_avl[i].gain);

	return err < 0 ? err : count;
}


static DEVICE_ATTR(enable, 0644, get_enable, set_enable);
static DEVICE_ATTR(sampling_freq, 0644, get_sampling_freq,
		   set_sampling_freq);
static DEVICE_ATTR(polling_rate, 0644, get_polling_rate,
		   set_polling_rate);
static DEVICE_ATTR(reset_steps, 0200, NULL, reset_steps);
static DEVICE_ATTR(sampling_freq_avail, 0444,
		   sampling_freq_avail_show, NULL);
static DEVICE_ATTR(scale_avail, 0444, get_scale_avail, NULL);
static DEVICE_ATTR(scale, 0644, scale_show, scale_store);

static struct attribute *lsm6dsox_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm6dsox_gyro_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm6dsox_sign_m_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm6dsox_step_c_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_reset_steps.attr,
	NULL,
};

static struct attribute *lsm6dsox_step_d_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm6dsox_tilt_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group lsm6dsox_attribute_groups[] = {
	[LSM6DSOX_ACCEL] = {
		.attrs = lsm6dsox_accel_attribute,
		.name = "accel",
	},
	[LSM6DSOX_GYRO] = {
		.attrs = lsm6dsox_gyro_attribute,
		.name = "gyro",
	},
	[LSM6DSOX_SIGN_MOTION] = {
		.attrs = lsm6dsox_sign_m_attribute,
		.name = "sign_m",
	},
	[LSM6DSOX_STEP_COUNTER] = {
		.attrs = lsm6dsox_step_c_attribute,
		.name = "step_c",
	},
	[LSM6DSOX_STEP_DETECTOR] = {
		.attrs = lsm6dsox_step_d_attribute,
		.name = "step_d",
	},
	[LSM6DSOX_TILT] = {
		.attrs = lsm6dsox_tilt_attribute,
		.name = "tilt",
	},
};

#ifdef CONFIG_OF
static u32 lsm6dsox_parse_dt(struct lsm6dsox_data *cdata)
{
	struct device_node *np;
	u32 val;

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

static int lsm6dsox_check_whoami(struct lsm6dsox_data *cdata, int id,
				 const char **name)
{
	int err, i, j;
	u8 wai;

	for (i = 0; i < ARRAY_SIZE(lsm6dsox_sensor_settings); i++) {
		for (j = 0; j < LSM6DSOX_MAX_ID; j++) {
			if (lsm6dsox_sensor_settings[i].id[j].name &&
			    lsm6dsox_sensor_settings[i].id[j].hw_id == id)
				break;
		}

		if (j < LSM6DSOX_MAX_ID)
			break;
	}

	if (i == ARRAY_SIZE(lsm6dsox_sensor_settings)) {
		dev_err(cdata->dev, "unsupported hw id [%02x]\n", id);

		return -ENODEV;
	}

	err = cdata->tf->read(cdata, LSM6DSOX_WHO_AM_I, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev,
			"failed to read Who-Am-I register (err %d)\n",
			err);

		return err;
	}

	if (wai != LSM6DSOX_WHO_AM_I_VAL) {
		dev_err(cdata->dev,
			"Who-Am-I value not valid (%d)\n", wai);

		return -ENODEV;
	}

	*name = lsm6dsox_sensor_settings[i].id[j].name;
	cdata->settings = &lsm6dsox_sensor_settings[i];

	return 0;
}

int lsm6dsox_common_probe(struct lsm6dsox_data *cdata, int irq,
			  int hw_id, u16 bustype)
{
	const struct lsm6dsox_fs_table *fs_table;
	struct lsm6dsox_sensor_data *sdata;
	const char *name = NULL;
	int32_t err, i;

	mutex_init(&cdata->reg_lock);
	mutex_init(&cdata->tb.buf_lock);

	err = lsm6dsox_check_whoami(cdata, hw_id, &name);
	if (err < 0)
		return err;

	scnprintf(cdata->dev_name, sizeof(cdata->dev_name), "%s", name);

	if (irq > 0) {
#ifdef CONFIG_OF
		err = lsm6dsox_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lsm6dsox_platform_data *)
			       cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) ||
			    (cdata->drdy_int_pin < 1)) {
				cdata->drdy_int_pin = 1;
				dev_warn(cdata->dev,
					 "using int pin 1 by default");
			}
		} else {
			cdata->drdy_int_pin = 1;
		}
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
			 cdata->drdy_int_pin);
	}

	for (i = 0; i < LSM6DSOX_MAX_SENSOR; i++) {
		sdata = &cdata->sensors[i];
		sdata->name = lsm6dsox_sensor_name[i].name;
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;

		if ((i == LSM6DSOX_ACCEL) || (i == LSM6DSOX_GYRO)) {
			fs_table = &cdata->settings->fs_table[i];
			sdata->c_odr = lsm6dsox_odr_table.odr_avl[0].hz;
			sdata->c_gain = fs_table->fs_avl[0].gain;
			sdata->poll_interval = 1000 / sdata->c_odr;
		}

		if (i == LSM6DSOX_STEP_COUNTER)
			sdata->c_odr = LSM6DSOX_MIN_DURATION_MS;

		if (lsm6dsox_input_init(sdata, bustype,
					lsm6dsox_sensor_name[i].description)) {
			dev_err(cdata->dev,
				"failed to register input device %s",
				sdata->name);
			sdata->input_dev = NULL;

			continue;
		}

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
				       &lsm6dsox_attribute_groups[i])) {
			dev_err(cdata->dev,
				"failed to create sysfs for sensor %s",
				sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	err = lsm6dsox_init_sensors(cdata);
	if (err < 0)
		return err;

	if (irq > 0) {
		cdata->irq = irq;

		err = lsm6dsox_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(lsm6dsox_common_probe);

void lsm6dsox_common_remove(struct lsm6dsox_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < LSM6DSOX_MAX_SENSOR; i++) {
		lsm6dsox_disable_sensors(&cdata->sensors[i]);
		lsm6dsox_input_cleanup(&cdata->sensors[i]);
	}

	if (lsm6dsox_workqueue) {
		flush_workqueue(lsm6dsox_workqueue);
		destroy_workqueue(lsm6dsox_workqueue);
		lsm6dsox_workqueue = NULL;
	}
}
EXPORT_SYMBOL(lsm6dsox_common_remove);

#ifdef CONFIG_PM_SLEEP
static int lsm6dsox_resume_sensors(struct lsm6dsox_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _lsm6dsox_enable_sensors(sdata);
}

static int lsm6dsox_suspend_sensors(struct lsm6dsox_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _lsm6dsox_disable_sensors(sdata);
}

int lsm6dsox_common_suspend(struct lsm6dsox_data *cdata)
{
	lsm6dsox_suspend_sensors(&cdata->sensors[LSM6DSOX_ACCEL]);
	lsm6dsox_suspend_sensors(&cdata->sensors[LSM6DSOX_GYRO]);

	return 0;
}
EXPORT_SYMBOL(lsm6dsox_common_suspend);

int lsm6dsox_common_resume(struct lsm6dsox_data *cdata)
{
	lsm6dsox_resume_sensors(&cdata->sensors[LSM6DSOX_ACCEL]);
	lsm6dsox_resume_sensors(&cdata->sensors[LSM6DSOX_GYRO]);

	return 0;
}
EXPORT_SYMBOL(lsm6dsox_common_resume);

#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("STMicroelectronics lsm6dsox driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
