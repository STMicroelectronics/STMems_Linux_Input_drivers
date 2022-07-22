/*
 * STMicroelectronics asm330lhh driver
 *
 * Copyright 2019 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 * v 1.0
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

#include "linux/platform_data/st/asm330lhh.h"
#include "asm330lhh_core.h"

/* COMMON DEFINE FOR ACCEL-GYRO SENSORS */
#define ASM330LHH_EN_BIT			0x01
#define ASM330LHH_DIS_BIT			0x00

#define ASM330LHH_WHO_AM_I			0x0f
#define ASM330LHH_WHO_AM_I_DEF			0x6b

#define ASM330LHH_INT1_CTRL_ADDR		0x0d
#define ASM330LHH_INT2_CTRL_ADDR		0x0e
#define ASM330LHH_CTRL1_XL_ADDR			0x10
#define ASM330LHH_CTRL2_G_ADDR			0x11
#define ASM330LHH_CTRL3_C_ADDR			0x12
#define ASM330LHH_CTRL4_C_ADDR			0x13
#define ASM330LHH_CTRL7_G_ADDR			0x16
#define ASM330LHH_CTRL10_C_ADDR			0x19
#define ASM330LHH_STATUS_REG			0x1e
#define ASM330LHH_OUT_TEMP_L_ADDR		0x20
#define ASM330LHH_OUTX_L_G_L_ADDR		0x22
#define ASM330LHH_OUTX_L_A_ADDR			0x28
#define ASM330LHH_INT_CFG1_ADDR			0x58

#define ASM330LHH_FS_LIST_NUM			4
#define ASM330LHH_ODR_LIST_NUM			6

#define ASM330LHH_ODR_POWER_OFF_VAL		0x00
#define ASM330LHH_ODR_13HZ_VAL			0x01
#define ASM330LHH_ODR_26HZ_VAL			0x02
#define ASM330LHH_ODR_52HZ_VAL			0x03
#define ASM330LHH_ODR_104HZ_VAL			0x04
#define ASM330LHH_ODR_208HZ_VAL			0x05
#define ASM330LHH_ODR_416HZ_VAL			0x06

#define ASM330LHH_BDU_MASK			0x40
#define ASM330LHH_LIR_MASK			0x01
#define ASM330LHH_TIMER_EN_MASK			0x20
#define ASM330LHH_INT2_ON_INT1_MASK		0x20
#define ASM330LHH_ROUNDING_MASK			0x04
#define ASM330LHH_RESET_MASK			0x01

#define ASM330LHH_XL_DATA_AVAIL			0x01
#define ASM330LHH_G_DATA_AVAIL			0x02
#define ASM330LHH_T_DATA_AVAIL			0x04

/* VALUES FOR ACCEL SENSOR */
#define ASM330LHH_CTRL1_XL_ODR_MASK		0xf0
#define ASM330LHH_CTRL1_XL_FS_MASK		0x0c

#define ASM330LHH_ACCEL_FS_2G_VAL		0x00
#define ASM330LHH_ACCEL_FS_4G_VAL		0x02
#define ASM330LHH_ACCEL_FS_8G_VAL		0x03
#define ASM330LHH_ACCEL_FS_16G_VAL		0x01

#define ASM330LHH_ACCEL_FS_2G_GAIN		61
#define ASM330LHH_ACCEL_FS_4G_GAIN		122
#define ASM330LHH_ACCEL_FS_8G_GAIN		244
#define ASM330LHH_ACCEL_FS_16G_GAIN		488

#define ASM330LHH_ACCEL_DRDY_IRQ_MASK		0x01
#define ASM330LHH_ACCEL_STD			1
#define ASM330LHH_ACCEL_STD_FROM_PD		2

/* VALUES FOR GYRO SENSOR */
#define ASM330LHH_CTRL2_G_ODR_MASK		0xf0
#define ASM330LHH_CTRL2_G_FS_MASK		0x0c

#define ASM330LHH_GYRO_FS_250_VAL		0x00
#define ASM330LHH_GYRO_FS_500_VAL		0x01
#define ASM330LHH_GYRO_FS_1000_VAL		0x02
#define ASM330LHH_GYRO_FS_2000_VAL		0x03

#define ASM330LHH_GYRO_FS_250_GAIN		8750
#define ASM330LHH_GYRO_FS_500_GAIN		17500
#define ASM330LHH_GYRO_FS_1000_GAIN		35000
#define ASM330LHH_GYRO_FS_2000_GAIN		70000

#define ASM330LHH_GYRO_DRDY_IRQ_MASK		0x02
#define ASM330LHH_GYRO_STD			6
#define ASM330LHH_GYRO_STD_FROM_PD		2

#define ASM330LHH_OUT_XYZ_SIZE			6
#define ASM330LHH_OUT_TEMP_SIZE			2

static int asm330lhh_disable_sensors(struct asm330lhh_sensor_data *sdata);

/* Minimal number of sample to be discarded */
static const u8 st_asm330lhh_std_table[] = { 2, 3, 4, 6, 8, 18 };

static const struct asm330lhh_sensor_name {
	const char *name;
	const char *description;
} asm330lhh_sensor_name[ASM330LHH_MAX_ID] = {
	[ASM330LHH_ACCEL] = {
		.name = "accel",
		.description = "ST ASM330LHH Accelerometer Sensor",
	},
	[ASM330LHH_GYRO] = {
		.name = "gyro",
		.description = "ST ASM330LHH Gyroscope Sensor",
	},
	[ASM330LHH_TEMP] = {
		.name = "temp",
		.description = "ST ASM330LHH Temperature Sensor",
	},
};

struct asm330lhh_odr_reg {
	u32 hz;
	u8 value;
};

static const struct asm330lhh_odr_table {
	u8 addr;
	u8 mask;
	u8 odr_len;
	struct asm330lhh_odr_reg odr_avl[ASM330LHH_ODR_LIST_NUM];
} asm330lhh_odr_table[ASM330LHH_MAX_ID] = {
	[ASM330LHH_ACCEL] = {
		.addr = ASM330LHH_CTRL1_XL_ADDR,
		.mask = ASM330LHH_CTRL1_XL_ODR_MASK,
		.odr_len = 6,
		.odr_avl[0] = {
			.hz = 13,
			.value = ASM330LHH_ODR_13HZ_VAL
		},
		.odr_avl[1] = {
			.hz = 26,
			.value = ASM330LHH_ODR_26HZ_VAL
		},
		.odr_avl[2] = {
			.hz = 52,
			.value = ASM330LHH_ODR_52HZ_VAL
		},
		.odr_avl[3] = {
			.hz = 104,
			.value = ASM330LHH_ODR_104HZ_VAL
		},
		.odr_avl[4] = {
			.hz = 208,
			.value = ASM330LHH_ODR_208HZ_VAL
		},
		.odr_avl[5] = {
			.hz = 416,
			.value = ASM330LHH_ODR_416HZ_VAL
		},
	},
	[ASM330LHH_GYRO] = {
		.addr = ASM330LHH_CTRL2_G_ADDR,
		.mask = ASM330LHH_CTRL2_G_ODR_MASK,
		.odr_len = 6,
		.odr_avl[0] = {
			.hz = 13,
			.value = ASM330LHH_ODR_13HZ_VAL
		},
		.odr_avl[1] = {
			.hz = 26,
			.value = ASM330LHH_ODR_26HZ_VAL
		},
		.odr_avl[2] = {
			.hz = 52,
			.value = ASM330LHH_ODR_52HZ_VAL
		},
		.odr_avl[3] = {
			.hz = 104,
			.value = ASM330LHH_ODR_104HZ_VAL
		},
		.odr_avl[4] = {
			.hz = 208,
			.value = ASM330LHH_ODR_208HZ_VAL
		},
		.odr_avl[5] = {
			.hz = 416,
			.value = ASM330LHH_ODR_416HZ_VAL
		},
	},
	[ASM330LHH_TEMP] = {
		/* enable temperature sensor by enabling acc or gyro */
		.addr = ASM330LHH_CTRL1_XL_ADDR,
		.mask = ASM330LHH_CTRL1_XL_ODR_MASK,
		.odr_len = 1,
		.odr_avl[0] = {
			.hz = 52,
			.value = ASM330LHH_ODR_52HZ_VAL
		},
	},
};

struct asm330lhh_fs_reg {
	unsigned int gain;
	u8 value;
	int urv;
};

static const struct asm330lhh_fs_table {
	u8 addr;
	u8 mask;
	u8 fs_len;
	struct asm330lhh_fs_reg fs_avl[ASM330LHH_FS_LIST_NUM];
} asm330lhh_fs_table[ASM330LHH_MAX_ID] = {
	[ASM330LHH_ACCEL] = {
		.addr = ASM330LHH_CTRL1_XL_ADDR,
		.mask = ASM330LHH_CTRL1_XL_FS_MASK,
		.fs_len = 4,
		.fs_avl[0] = {
			.gain = ASM330LHH_ACCEL_FS_2G_GAIN,
			.value = ASM330LHH_ACCEL_FS_2G_VAL,
			.urv = 2,
		},
		.fs_avl[1] = {
			.gain = ASM330LHH_ACCEL_FS_4G_GAIN,
			.value = ASM330LHH_ACCEL_FS_4G_VAL,
			.urv = 4,
		},
		.fs_avl[2] = {
			.gain = ASM330LHH_ACCEL_FS_8G_GAIN,
			.value = ASM330LHH_ACCEL_FS_8G_VAL,
			.urv = 8,
		},
		.fs_avl[3] = {
			.gain = ASM330LHH_ACCEL_FS_16G_GAIN,
			.value = ASM330LHH_ACCEL_FS_16G_VAL,
			.urv = 16,
		},
	},
	[ASM330LHH_GYRO] = {
		.addr = ASM330LHH_CTRL2_G_ADDR,
		.mask = ASM330LHH_CTRL2_G_FS_MASK,
		.fs_len = 4,
		.fs_avl[0] = {
			.gain = ASM330LHH_GYRO_FS_250_GAIN,
			.value = ASM330LHH_GYRO_FS_250_VAL,
			.urv = 250,
		},
		.fs_avl[1] = {
			.gain = ASM330LHH_GYRO_FS_500_GAIN,
			.value = ASM330LHH_GYRO_FS_500_VAL,
			.urv = 500,
		},
		.fs_avl[2] = {
			.gain = ASM330LHH_GYRO_FS_1000_GAIN,
			.value = ASM330LHH_GYRO_FS_1000_VAL,
			.urv = 1000,
		},
		.fs_avl[3] = {
			.gain = ASM330LHH_GYRO_FS_2000_GAIN,
			.value = ASM330LHH_GYRO_FS_2000_VAL,
			.urv = 2000,
		},
	},
	[ASM330LHH_TEMP] = {
		.fs_len = 1,
		.fs_avl[0] = {
			.gain = 0,
			.urv = 0,
		},
	},
};

#ifndef CONFIG_ASM330LHH_IRQ_THREAD
static struct workqueue_struct *asm330lhh_workqueue;

static inline void asm330lhh_flush_works(void)
{
	flush_workqueue(asm330lhh_workqueue);
}
#endif /* !CONFIG_ASM330LHH_IRQ_THREAD */

static inline int64_t asm330lhh_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

static int asm330lhh_write_data_with_mask(struct asm330lhh_data *cdata,
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

static int asm330lhh_input_init(struct asm330lhh_sensor_data *sdata, u16 bustype,
				const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = asm330lhh_sensor_name[sdata->sindex].description;

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	/* for acc/gyro 6 bytes, for temperature 2 bytes + timestamp */
	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit );
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);

	if ((sdata->sindex == ASM330LHH_ACCEL) ||
	    (sdata->sindex == ASM330LHH_GYRO)) {
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

static void asm330lhh_input_cleanup(struct asm330lhh_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void asm330lhh_report_3axes_event(struct asm330lhh_sensor_data *sdata,
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

static void asm330lhh_report_1axes_event(struct asm330lhh_sensor_data *sdata,
					 s16 temperature, int64_t timestamp)
{
	struct input_dev  *input = sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, temperature);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(input);
}

static int asm330lhh_get_poll_data(struct asm330lhh_sensor_data *sdata, u8 *data)
{
	int err = 0;
	int len;
	u8 reg_addr;

	switch(sdata->sindex) {
	case ASM330LHH_ACCEL:
		reg_addr = ASM330LHH_OUTX_L_A_ADDR;
		len = ASM330LHH_OUT_XYZ_SIZE;
		break;
	case ASM330LHH_GYRO:
		reg_addr = ASM330LHH_OUTX_L_G_L_ADDR;
		len = ASM330LHH_OUT_XYZ_SIZE;
		break;
	case ASM330LHH_TEMP:
		reg_addr = ASM330LHH_OUT_TEMP_L_ADDR;
		len = ASM330LHH_OUT_TEMP_SIZE;
		break;
	default:
		dev_err(sdata->cdata->dev, "invalid polling mode for sensor %s\n",
			sdata->name);
		return -1;
	}

	err = sdata->cdata->tf->read(sdata->cdata, reg_addr, len, data, true);

	return err;
}

static int asm330lhh_set_fs(struct asm330lhh_sensor_data *sdata, u32 gain)
{
	int err, i;

	for (i = 0; i < asm330lhh_fs_table[sdata->sindex].fs_len; i++) {
		if (asm330lhh_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}

	if (i == asm330lhh_fs_table[sdata->sindex].fs_len)
		return -EINVAL;

	err = asm330lhh_write_data_with_mask(sdata->cdata,
				asm330lhh_fs_table[sdata->sindex].addr,
				asm330lhh_fs_table[sdata->sindex].mask,
				asm330lhh_fs_table[sdata->sindex].fs_avl[i].value,
				true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

#ifdef CONFIG_ASM330LHH_IRQ_THREAD
irqreturn_t asm330lhh_save_timestamp(int irq, void *private)
{
	struct asm330lhh_data *cdata = (struct asm330lhh_data *)private;

	cdata->timestamp = asm330lhh_get_time_ns();
	disable_irq_nosync(irq);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t asm330lhh_irq_management(int irq, void *private)
{
	struct asm330lhh_data *cdata;
	u8 src_value = 0x00;
	struct asm330lhh_sensor_data *sdata;
	int err;

	cdata = private;

	cdata->tf->read(cdata, ASM330LHH_STATUS_REG, 1, &src_value, true);

	if (src_value & (ASM330LHH_XL_DATA_AVAIL | ASM330LHH_G_DATA_AVAIL)) {
		int xyz[3] = { 0 };
		u8 data[ASM330LHH_OUT_XYZ_SIZE];

		sdata = (src_value & ASM330LHH_XL_DATA_AVAIL) ?
			&cdata->sensors[ASM330LHH_ACCEL] :
			&cdata->sensors[ASM330LHH_GYRO];

		err = asm330lhh_get_poll_data(sdata, data);
		if (err < 0) {
			dev_err(sdata->cdata->dev, "get %s data failed %d\n",
				sdata->name, err);
		} else {
			if(sdata->sample_to_discard) {
				sdata->sample_to_discard--;
			} else {
				sdata->timestamp = cdata->timestamp;
				xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
				xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
				xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
				xyz[0] *= sdata->c_gain;
				xyz[1] *= sdata->c_gain;
				xyz[2] *= sdata->c_gain;
				asm330lhh_report_3axes_event(sdata, xyz,
							     sdata->timestamp);
			}
		}
	}
	if (src_value & ASM330LHH_T_DATA_AVAIL) {
		s16 temperature;
		__le16 data;

		sdata = &cdata->sensors[ASM330LHH_TEMP];
		err = asm330lhh_get_poll_data(sdata, (u8*)&data);
		if (err < 0) {
			dev_err(sdata->cdata->dev, "get %s data failed %d\n",
				sdata->name, err);
		} else {
			if(sdata->sample_to_discard) {
				sdata->sample_to_discard--;
			} else {
				sdata->timestamp = cdata->timestamp;
				temperature = (s16)le16_to_cpu(data);
				asm330lhh_report_1axes_event(sdata, temperature,
							     sdata->timestamp);
			}
		}
	}

	enable_irq(cdata->irq);

	return IRQ_HANDLED;
}

static int asm330lhh_allocate_threaded_irq(struct asm330lhh_data *cdata)
{
	int err;

	err = request_threaded_irq(cdata->irq, asm330lhh_save_timestamp,
				   asm330lhh_irq_management,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   cdata->name, cdata);
	return err;
}

int asm330lhh_set_drdy_irq(struct asm330lhh_sensor_data *sdata, bool state)
{
	u8 reg_addr = 0, mask = 0, value;

	if (state)
		value = ASM330LHH_EN_BIT;
	else
		value = ASM330LHH_DIS_BIT;

	switch (sdata->sindex) {
	case ASM330LHH_ACCEL:
		mask = ASM330LHH_ACCEL_DRDY_IRQ_MASK;
		switch(sdata->cdata->drdy_int_pin) {
		case 1:
			reg_addr = ASM330LHH_INT1_CTRL_ADDR;
			break;
		case 2:
			reg_addr = ASM330LHH_INT2_CTRL_ADDR;
			break;
		default:
			return -EINVAL;
		}
		break;
	case ASM330LHH_GYRO:
		mask = ASM330LHH_GYRO_DRDY_IRQ_MASK;
		switch(sdata->cdata->drdy_int_pin) {
		case 1:
			reg_addr = ASM330LHH_INT1_CTRL_ADDR;
			break;
		case 2:
			reg_addr = ASM330LHH_INT2_CTRL_ADDR;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return asm330lhh_write_data_with_mask(sdata->cdata, reg_addr, mask, value,
					    true);
}
#else /* CONFIG_ASM330LHH_IRQ_THREAD */
static enum hrtimer_restart asm330lhh_poll_function_read(struct hrtimer *timer)
{
	struct asm330lhh_sensor_data *sdata;

	sdata = container_of((struct hrtimer *)timer, struct asm330lhh_sensor_data,
			     hr_timer);

	sdata->timestamp = asm330lhh_get_time_ns();
	queue_work(asm330lhh_workqueue, &sdata->input_work);

	return HRTIMER_NORESTART;
}

static void poll_function_work(struct work_struct *input_work)
{
	struct asm330lhh_sensor_data *sdata;
	int err;
	ktime_t tmpkt, ktdelta;

	sdata = container_of((struct work_struct *)input_work,
			     struct asm330lhh_sensor_data, input_work);

	/* Adjust new timeout */
	ktdelta = ktime_set(0, (asm330lhh_get_time_ns() - sdata->timestamp));

	/* Avoid negative value in case of High ODR. */
	if (sdata->oldktime.tv64 > ktdelta.tv64)
		tmpkt = ktime_sub(sdata->oldktime, ktdelta);
	else
		tmpkt = sdata->oldktime;

	hrtimer_start(&sdata->hr_timer, tmpkt, HRTIMER_MODE_REL);
	if(sdata->sample_to_discard) {
		sdata->sample_to_discard--;
		return;
	}

	switch (sdata->sindex) {
	case ASM330LHH_ACCEL:
	case ASM330LHH_GYRO: {
		int xyz[3] = { 0 };
		u8 data[ASM330LHH_OUT_XYZ_SIZE];

		err = asm330lhh_get_poll_data(sdata, data);
		if (err < 0) {
			dev_err(sdata->cdata->dev,
				"get %s data failed %d\n",
				sdata->name, err);
			break;
		}

		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;

		asm330lhh_report_3axes_event(sdata,
					     xyz,
					     sdata->timestamp);
		}
		break;
	case ASM330LHH_TEMP: {
		s16 temperature;
		__le16 data;

		err = asm330lhh_get_poll_data(sdata, (u8 *)&data);
		if (err < 0) {
			dev_err(sdata->cdata->dev, "get %s data failed %d\n",
				sdata->name, err);
			break;
		}

		temperature = (s16)le16_to_cpu(data);

		asm330lhh_report_1axes_event(sdata, temperature,
					     sdata->timestamp);
		}
		break;
	default:
		break;
	}
}

static int asm330lhh_allocate_workqueue(struct asm330lhh_data *cdata)
{
	if (!asm330lhh_workqueue)
		asm330lhh_workqueue = create_workqueue(cdata->name);

	if (!asm330lhh_workqueue)
		return -EINVAL;

	return 0;
}
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */

static int _asm330lhh_enable_sensors(struct asm330lhh_sensor_data *sdata)
{
	int err, i, id;
#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	int64_t newTime;
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */

	id = sdata->sindex;

	switch (id) {
	case ASM330LHH_ACCEL:
	case ASM330LHH_GYRO:
		for (i = 0; i < asm330lhh_odr_table[id].odr_len; i++) {
			if (asm330lhh_odr_table[id].odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == asm330lhh_odr_table[id].odr_len)
			i = asm330lhh_odr_table[id].odr_len - 1;

		sdata->sample_to_discard = st_asm330lhh_std_table[i];
		err = asm330lhh_write_data_with_mask(sdata->cdata,
				asm330lhh_odr_table[id].addr,
				asm330lhh_odr_table[id].mask,
				asm330lhh_odr_table[id].odr_avl[i].value,
				true);
		if (err < 0)
			return err;
		break;
	case ASM330LHH_TEMP:
		for (i = 0; i < asm330lhh_odr_table[id].odr_len; i++) {
			if (asm330lhh_odr_table[id].odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == asm330lhh_odr_table[id].odr_len)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	sdata->c_odr = asm330lhh_odr_table[id].odr_avl[i].hz;

#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	newTime = 1000000000 / sdata->c_odr;
	sdata->oldktime = ktime_set(0, newTime);
	hrtimer_start(&sdata->hr_timer, sdata->oldktime, HRTIMER_MODE_REL);
#else /* CONFIG_ASM330LHH_IRQ_THREAD */
	err = asm330lhh_set_drdy_irq(sdata, true);
	if (err < 0)
		return err;
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */

	return 0;
}

static int asm330lhh_enable_sensors(struct asm330lhh_sensor_data *sdata)
{
	int err;

	if (sdata->enabled)
		return 0;

	err = _asm330lhh_enable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = true;

	return 0;
}

static int _asm330lhh_disable_sensors(struct asm330lhh_sensor_data *sdata)
{
	int err, id;
	u8 addr, mask;

	id = sdata->sindex;
	switch (id) {
	case ASM330LHH_ACCEL:
		addr = asm330lhh_odr_table[ASM330LHH_ACCEL].addr;
		mask = asm330lhh_odr_table[ASM330LHH_ACCEL].mask;
		break;
	case ASM330LHH_GYRO:
		addr = asm330lhh_odr_table[ASM330LHH_GYRO].addr;
		mask = asm330lhh_odr_table[ASM330LHH_GYRO].mask;
		break;
	case ASM330LHH_TEMP:
		break;
	default:
		return -EINVAL;
	}

	if (id != ASM330LHH_TEMP) {
		err = asm330lhh_write_data_with_mask(sdata->cdata, addr, mask,
						ASM330LHH_ODR_POWER_OFF_VAL,
						true);
		if (err < 0)
			return err;
	}

#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	cancel_work_sync(&sdata->input_work);
	hrtimer_cancel(&sdata->hr_timer);
#else /* CONFIG_ASM330LHH_IRQ_THREAD */
	err = asm330lhh_set_drdy_irq(sdata, false);
	if (err < 0)
		return err;
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */

	return 0;
}

static int asm330lhh_disable_sensors(struct asm330lhh_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	err = _asm330lhh_disable_sensors(sdata);
	if (err < 0)
		return err;

	sdata->enabled = false;

	return 0;
}

static int asm330lhh_init_sensors(struct asm330lhh_data *cdata)
{
	int err, i;
	struct asm330lhh_sensor_data *sdata;

	for (i = 0; i < ASM330LHH_MAX_ID; i++) {
		sdata = &cdata->sensors[i];

		/* skip temperature sensor */
		if (sdata->sindex != ASM330LHH_TEMP) {
			err = asm330lhh_disable_sensors(sdata);
			if (err < 0)
				return err;

			err = asm330lhh_set_fs(sdata, sdata->c_gain);
			if (err < 0)
				return err;
			}

#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	hrtimer_init(&cdata->sensors[i].hr_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	cdata->sensors[i].hr_timer.function = &asm330lhh_poll_function_read;

	cdata->sensors[i].oldktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[i].poll_interval));
	INIT_WORK(&cdata->sensors[i].input_work, poll_function_work);
#endif /* !CONFIG_ASM330LHH_IRQ_THREAD */
	}

	/* Software reset */
	err = asm330lhh_write_data_with_mask(cdata, ASM330LHH_CTRL3_C_ADDR,
					   ASM330LHH_RESET_MASK, ASM330LHH_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable Latch Mode Bit */
	err = asm330lhh_write_data_with_mask(cdata, ASM330LHH_INT_CFG1_ADDR,
					   ASM330LHH_LIR_MASK, ASM330LHH_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable timestamp count */
	err = asm330lhh_write_data_with_mask(cdata, ASM330LHH_CTRL10_C_ADDR,
					   ASM330LHH_TIMER_EN_MASK,
					   ASM330LHH_EN_BIT, true);
		if (err < 0)
			return err;

	/* Output data not updated until have been read */
	err = asm330lhh_write_data_with_mask(cdata, ASM330LHH_CTRL3_C_ADDR,
					   ASM330LHH_BDU_MASK, ASM330LHH_EN_BIT,
					   true);
	if (err < 0)
		return err;

	/* Enable Source Rounding function */
	err = asm330lhh_write_data_with_mask(cdata, ASM330LHH_CTRL7_G_ADDR,
					   ASM330LHH_ROUNDING_MASK,
					   ASM330LHH_EN_BIT, true);
	if (err < 0)
		return err;

	/* Set all interrupt signals in logic or on INT1 pad */
	err = asm330lhh_write_data_with_mask(cdata,
					   ASM330LHH_CTRL4_C_ADDR,
					   ASM330LHH_INT2_ON_INT1_MASK,
					   ASM330LHH_EN_BIT, true);
	if (err < 0)
		return err;

	return 0;
}

static int asm330lhh_set_odr(struct asm330lhh_sensor_data *sdata, u32 odr)
{
	int err = 0, i, id;

	id = sdata->sindex;

	for (i = 0; i < asm330lhh_odr_table[id].odr_len; i++) {
		if (asm330lhh_odr_table[id].odr_avl[i].hz >= odr)
			break;
	}

	if (i == asm330lhh_odr_table[id].odr_len)
		i = asm330lhh_odr_table[id].odr_len - 1;

	/* Already configured */
	if (sdata->c_odr == asm330lhh_odr_table[id].odr_avl[i].hz)
		return 0;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
#ifndef CONFIG_ASM330LHH_IRQ_THREAD
		asm330lhh_flush_works();
#endif /* !CONFIG_ASM330LHH_IRQ_THREAD */

		/* Reset discard samples counter */
		if (id != ASM330LHH_TEMP)
			sdata->cdata->sensors[id].sample_to_discard =
						st_asm330lhh_std_table[i];

		err = asm330lhh_write_data_with_mask(sdata->cdata,
				asm330lhh_odr_table[id].addr,
				asm330lhh_odr_table[id].mask,
				asm330lhh_odr_table[id].odr_avl[i].value,
				true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = asm330lhh_odr_table[id].odr_avl[i].hz;
		enable_irq(sdata->cdata->irq);
	} else {
		sdata->c_odr = asm330lhh_odr_table[id].odr_avl[i].hz;
	}

	return err;
}

static ssize_t get_enable(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	int err;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = asm330lhh_enable_sensors(sdata);
	else
		err = asm330lhh_disable_sensors(sdata);

	return count;
}

static ssize_t get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_interval);
}

static ssize_t set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int polling_rate;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);
#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	int64_t newTime;
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);

	/*
	 * Polling interval is in msec, then we have to convert it in Hz to
	 * configure ODR through asm330lhh_set_odr
	 */
	err = asm330lhh_set_odr(sdata, 1000 / polling_rate);
	if (!(err < 0)) {
		sdata->poll_interval = 1000 / sdata->c_odr;
#ifndef CONFIG_ASM330LHH_IRQ_THREAD
		newTime = MS_TO_NS(sdata->poll_interval);
		sdata->oldktime = ktime_set(0, newTime);
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */
	}
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t get_sampling_freq(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static ssize_t set_sampling_freq(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int err;
	unsigned int odr;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = asm330lhh_set_odr(sdata, odr);
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

static ssize_t get_sampling_frequency_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int i, len = 0;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < asm330lhh_odr_table[sdata->sindex].odr_len; i++) {
		if (asm330lhh_odr_table[sdata->sindex].odr_avl[i].hz == 0)
			break;

		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 asm330lhh_odr_table[sdata->sindex].odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_scale_avail(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i <  asm330lhh_fs_table[sdata->sindex].fs_len; i++) {
		if (asm330lhh_fs_table[sdata->sindex].fs_avl[i].urv == 0)
			break;

		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			asm330lhh_fs_table[sdata->sindex].fs_avl[i].urv);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_cur_scale(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int i;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i <  asm330lhh_fs_table[sdata->sindex].fs_len; i++)
		if (sdata->c_gain ==
		    asm330lhh_fs_table[sdata->sindex].fs_avl[i].gain)
			break;

	return sprintf(buf, "%d\n",
		       asm330lhh_fs_table[sdata->sindex].fs_avl[i].urv);
}

static ssize_t set_cur_scale(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int i, urv, err;
	struct asm330lhh_sensor_data *sdata = dev_get_drvdata(dev);

	/* Temperature sensor has fixed FS */
	if (sdata->sindex == ASM330LHH_TEMP)
		return count;

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	for (i = 0; i <  asm330lhh_fs_table[sdata->sindex].fs_len; i++)
		if (urv == asm330lhh_fs_table[sdata->sindex].fs_avl[i].urv)
			break;

	if (i ==  asm330lhh_fs_table[sdata->sindex].fs_len)
		return -EINVAL;

	err = asm330lhh_set_fs(sdata,
			asm330lhh_fs_table[sdata->sindex].fs_avl[i].gain);
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

static struct attribute *asm330lhh_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *asm330lhh_gyro_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *asm330lhh_temp_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static const struct attribute_group asm330lhh_attribute_groups[] = {
	[ASM330LHH_ACCEL] = {
		.attrs = asm330lhh_accel_attribute,
		.name = "accel",
	},
	[ASM330LHH_GYRO] = {
		.attrs = asm330lhh_gyro_attribute,
		.name = "gyro",
	},
	[ASM330LHH_TEMP] = {
		.attrs = asm330lhh_temp_attribute,
		.name = "temp",
	},
};

#ifdef CONFIG_OF
static u32 asm330lhh_parse_dt(struct asm330lhh_data *cdata)
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

int asm330lhh_common_probe(struct asm330lhh_data *cdata, int irq, u16 bustype)
{
	int32_t err, i;
	u8 wai =0x00;
	struct asm330lhh_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->tb.buf_lock);

	/* Read Chip ID register */
	err = cdata->tf->read(cdata, ASM330LHH_WHO_AM_I, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != ASM330LHH_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	if (irq > 0) {

#ifdef CONFIG_OF
		err = asm330lhh_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct asm330lhh_platform_data *)
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

	for (i = 0; i < ASM330LHH_MAX_ID; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = asm330lhh_sensor_name[i].name;
		sdata->c_odr = asm330lhh_odr_table[i].odr_avl[0].hz;
		sdata->c_gain = asm330lhh_fs_table[i].fs_avl[0].gain;
		sdata->poll_interval = 1000 / sdata->c_odr;

		if (asm330lhh_input_init(sdata, bustype,
				       asm330lhh_sensor_name[i].description)) {
			dev_err(cdata->dev,
				"failed to register input device %s",
				sdata->name);
			sdata->input_dev = NULL;
			continue;
		}

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
				       &asm330lhh_attribute_groups[i])) {
			dev_err(cdata->dev,
				"failed to create sysfs group for sensor %s",
				sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	err = asm330lhh_init_sensors(cdata);
	if (err < 0)
		return err;

#ifdef CONFIG_ASM330LHH_IRQ_THREAD
	if (irq > 0) {
		cdata->irq = irq;
		err = asm330lhh_allocate_threaded_irq(cdata);
		if (err < 0)
			return err;
	}
#else /* !CONFIG_ASM330LHH_IRQ_THREAD */
	err = asm330lhh_allocate_workqueue(cdata);
	if (err < 0)
		return err;
#endif /* !CONFIG_ASM330LHH_IRQ_THREAD */

	dev_info(cdata->dev, "%s: probed\n", ASM330LHH_DEV_NAME);

	return 0;
}
EXPORT_SYMBOL(asm330lhh_common_probe);

void asm330lhh_common_remove(struct asm330lhh_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < ASM330LHH_MAX_ID; i++) {
		asm330lhh_disable_sensors(&cdata->sensors[i]);
		asm330lhh_input_cleanup(&cdata->sensors[i]);
	}

#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	if(asm330lhh_workqueue) {
		asm330lhh_flush_works();
		destroy_workqueue(asm330lhh_workqueue);
		asm330lhh_workqueue = NULL;
	}
#endif /* !CONFIG_ASM330LHH_IRQ_THREAD */
}
EXPORT_SYMBOL(asm330lhh_common_remove);

#ifdef CONFIG_PM_SLEEP
static int asm330lhh_resume_sensors(struct asm330lhh_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _asm330lhh_enable_sensors(sdata);
}

static int asm330lhh_suspend_sensors(struct asm330lhh_sensor_data *sdata)
{
	if (!sdata->enabled)
		return 0;

	return _asm330lhh_disable_sensors(sdata);
}

int asm330lhh_common_suspend(struct asm330lhh_data *cdata)
{
	uint8_t i;

	for (i = 0; i < ASM330LHH_MAX_ID; i++)
		asm330lhh_suspend_sensors(&cdata->sensors[i]);

	return 0;
}
EXPORT_SYMBOL(asm330lhh_common_suspend);

int asm330lhh_common_resume(struct asm330lhh_data *cdata)
{
	uint8_t i;

	for (i = 0; i < ASM330LHH_MAX_ID; i++)
		asm330lhh_resume_sensors(&cdata->sensors[i]);

	return 0;
}
EXPORT_SYMBOL(asm330lhh_common_resume);

#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("STMicroelectronics asm330lhh driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
