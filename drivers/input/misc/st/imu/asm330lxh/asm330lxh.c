/******************** (C) COPYRIGHT 2014 STMicroelectronics ******************
 *
 * File Name		: asm330lxh.c
 * Author		: MSH - C&I BU - Application Team
 *			: Giuseppe Barba (giuseppe.barba@st.com)
 *			: Alberto MARINONI (alberto.marinoni@st.com)
 *			: Mario Tesi (mario.tesi@st.com)
 *			: Authors is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V.1.0.1
 * Date			: 2016/Jul/07
 * Description	: STMicroelectronics asm330lxh driver
 *
 ******************************************************************************
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
 *****************************************************************************/
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#endif

#include "asm330lxh.h"

#define MS_TO_US(x)		(x * 1000L)
#define US_TO_NS(x)		(x * 1000L)
#define MS_TO_NS(x)		(US_TO_NS(MS_TO_US(x)))
#define NS_TO_US(x)		(x / 1000)
#define US_TO_MS(x)		(x / 1000)
#define NS_TO_MS(x)		(US_TO_MS(NS_TO_US(x)))

#define DEFAULT_POLL_RATE_15HZ_US	66666

/* TODO: check the following values */
/* Sensitivity */
#define SENSITIVITY_ACC_2G	61	/** ug/LSB */
#define SENSITIVITY_ACC_4G	122	/** ug/LSB */
#define SENSITIVITY_ACC_8G	244	/** ug/LSB */
#define SENSITIVITY_ACC_16G	488	/** ug/LSB */

#define SENSITIVITY_GYR_125	4370	/** udps/LSB */
#define SENSITIVITY_GYR_250	8750	/** udps/LSB */
#define SENSITIVITY_GYR_500	17500	/** udps/LSB */
#define SENSITIVITY_GYR_1000	35000	/** udps/LSB */
#define SENSITIVITY_GYR_2000	70000	/** udps/LSB */

#define SENSITIVITY_TEMP	16	/** LSB/C */
#define OFFSET_TEMP		25	/** Offset temperature */

#define ACC_G_MAX_POS		(32767 * SENSITIVITY_ACC_16G)	/** max positive value acc [ug] */
#define ACC_G_MAX_NEG		(-ACC_G_MAX_POS)		/** max negative value acc [ug] */
#define GYR_DPS_MAX_POS		(28571 * SENSITIVITY_GYR_2000)	/** max positive value gyr [udps] */
#define GYR_DPS_MAX_NEG		(-GYR_DPS_MAX_POS)		/** max negative value gyr [udps] */
#define FUZZ			0
#define FLAT			0

/* Device Registers */
#define DEF_ZERO		0x00
#define UNDEF			0x00

#define INT_CFG2_XL		0x01
#define INT_THS2_XL		0x02
#define INT_DUR2_XL		0x03
#define ACT_THS			0x04
#define ACT_DUR			0x05
#define INT_CFG1_XL		0x06
#define INT_THS_X1_XL		0x07
#define INT_THS_Y1_XL		0x08
#define INT_THS_Z1_XL		0x09
#define INT_DUR1_XL		0x0A
#define ORIENT_CFG_G		0x0B
#define REFERENCE_G		0x0C
#define INT1_CTRL		0x0D
#define INT2_CTRL		0x0E
#define WHO_AM_I		0x0F
#define WHO_AM_I_VAL		0x61
#define CTRL1_XL		0x10
#define CTRL2_G			0x11
#define CTRL3_C			0x12
#define CTRL3_IF_ADD_INC	0x04
#define CTRL3_C_BDU		0x40
#define CTRL4_C			0x13
#define CTRL5_C			0x14
#define CTRL6_G			0x15
#define CTRL7_G			0x16
#define CTRL8_XL		0x17
#define CTRL9_XL		0x18
#define CTRL9_XL_DEF		0x38
#define CTRL10_C		0x19
#define CTRL10_C_DEF		0x38
#define CTRL10_C_ALLAXIS_EN	0x38
#define FIFO_CTRL		0x1A
#define INT_SRC_G		0x1B
#define INT_SRC2_XL		0x1C
#define INT_SRC1_XL		0x1D
#define FIFO_SRC		0x1E
#define STATUS_REG		0x1F
#define OUT_TEMP_L		0x20
#define OUT_TEMP_H		0x21
#define	OUT_X_L_G		0x22	/* 1st AXIS OUT REG of 6 */
#define OUT_X_L_XL		0x28	/* 1st AXIS OUT REG of 6 */
#define INT_GEN_CFG_G		0x30
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32
#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34
#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36
#define INT_GEN_DUR_G		0x37

#define to_dev(obj) container_of(obj, struct device, kobj)

static struct kobject *acc_kobj;
static struct kobject *gyr_kobj;

struct workqueue_struct *asm330lxh_workqueue = 0;

struct reg_rw {
	uint8_t address;
	uint8_t default_val;
	uint8_t resume_val;
};

struct reg_r {
	uint8_t address;
	uint8_t default_val;
};

static struct device_registers {
	struct reg_rw ctrl1_xl;
	struct reg_rw ctrl2_g;
	struct reg_rw ctrl3_c;
	struct reg_rw ctrl4_c;
	struct reg_rw ctrl5_c;
	struct reg_rw ctrl6_g;
	struct reg_rw ctrl7_g;
	struct reg_rw ctrl8_xl;
	struct reg_rw ctrl9_xl;
	struct reg_rw ctrl10_c;
} device_registers = {
	.ctrl1_xl = {
		.address = CTRL1_XL,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl2_g = {
		.address = CTRL2_G,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl3_c = {
		/* Set address auto increment */
		.address = CTRL3_C,
		.default_val = (CTRL3_IF_ADD_INC | CTRL3_C_BDU),
		.resume_val = (CTRL3_IF_ADD_INC | CTRL3_C_BDU),
	},
	.ctrl4_c = {
		.address = CTRL4_C,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl5_c = {
		.address = CTRL5_C,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl6_g = {
		.address = CTRL6_G,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl7_g = {
		.address = CTRL7_G,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl8_xl = {
		.address = CTRL8_XL,
		.default_val = DEF_ZERO,
		.resume_val = DEF_ZERO,
	},
	.ctrl9_xl = {
		.address = CTRL9_XL,
		.default_val = CTRL9_XL_DEF,
		.resume_val = CTRL9_XL_DEF,
	},
	.ctrl10_c = {
		.address = CTRL10_C,
		.default_val = CTRL10_C_DEF,
		.resume_val = CTRL10_C_DEF,
	},
};

struct output_rate {
	uint32_t odr_us;
	uint8_t value;
};

static const struct output_rate asm330lxh_gyr_odr_table[] = {
	{ ASM330LXH_ODR_PERIOD_US_800_HZ,
	 (ASM330LXH_GYR_ODR_110 | ASM330LXH_GYR_BW_00) },
	{ ASM330LXH_ODR_PERIOD_US_400_HZ,
	 (ASM330LXH_GYR_ODR_101 | ASM330LXH_GYR_BW_00) },
	{ ASM330LXH_ODR_PERIOD_US_200_HZ,
	 (ASM330LXH_GYR_ODR_100 | ASM330LXH_GYR_BW_00) },
	{ ASM330LXH_ODR_PERIOD_US_100_HZ,
	 (ASM330LXH_GYR_ODR_011 | ASM330LXH_GYR_BW_00) },
	{ ASM330LXH_ODR_PERIOD_US_50_HZ,
	 (ASM330LXH_GYR_ODR_010 | ASM330LXH_GYR_BW_00) },
	{ ASM330LXH_ODR_PERIOD_US_12_5_HZ,
	 (ASM330LXH_GYR_ODR_001 | ASM330LXH_GYR_BW_00) },
};

static const struct output_rate asm330lxh_acc_odr_table[] = {
	{ ASM330LXH_ODR_PERIOD_US_800_HZ, (ASM330LXH_GYR_ODR_110) },
	{ ASM330LXH_ODR_PERIOD_US_400_HZ, (ASM330LXH_GYR_ODR_101) },
	{ ASM330LXH_ODR_PERIOD_US_200_HZ, (ASM330LXH_GYR_ODR_100) },
	{ ASM330LXH_ODR_PERIOD_US_100_HZ, (ASM330LXH_GYR_ODR_011) },
	{ ASM330LXH_ODR_PERIOD_US_50_HZ, (ASM330LXH_GYR_ODR_010) },
	{ ASM330LXH_ODR_PERIOD_US_12_5_HZ, (ASM330LXH_GYR_ODR_001) },
};

static const struct axl_turn_on_time {
	uint32_t odr_us;
	uint32_t samples_to_discard;
} asm330lxh_axl_turn_on_time[] = { 
	{ ASM330LXH_ODR_PERIOD_US_800_HZ, 2 },
	{ ASM330LXH_ODR_PERIOD_US_400_HZ, 2 },
	{ ASM330LXH_ODR_PERIOD_US_200_HZ, 2 },
	{ ASM330LXH_ODR_PERIOD_US_100_HZ, 2 },
	{ ASM330LXH_ODR_PERIOD_US_50_HZ, 0 },
	{ ASM330LXH_ODR_PERIOD_US_12_5_HZ, 0 },
};

static const struct gyr_turn_on_time {
	uint32_t odr_us;
	uint32_t samples_to_discard;
} asm330lxh_gyr_turn_on_time[] = {
		{ ASM330LXH_ODR_PERIOD_US_800_HZ, 9 },
		{ ASM330LXH_ODR_PERIOD_US_400_HZ, 7 },
		{ ASM330LXH_ODR_PERIOD_US_200_HZ, 5 },
		{ ASM330LXH_ODR_PERIOD_US_100_HZ, 4 },
		{ ASM330LXH_ODR_PERIOD_US_50_HZ, 4 },
		{ ASM330LXH_ODR_PERIOD_US_12_5_HZ, 2 },
};

static const struct asm330lxh_acc_platform_data default_asm330lxh_acc_pdata = {
	.fs_range = ASM330LXH_ACC_FS_4G,
	.poll_interval = ASM330LXH_ACC_POLL_INTERVAL_DEF,
	.min_interval = ASM330LXH_ACC_MIN_POLL_PERIOD_US,
};

static const struct asm330lxh_gyr_platform_data default_asm330lxh_gyr_pdata = {
	.fs_range = ASM330LXH_GYR_FS_2000DPS,
	.poll_interval = ASM330LXH_GYR_POLL_INTERVAL_DEF,
	.min_interval = ASM330LXH_GYR_MIN_POLL_PERIOD_US,
};

static inline s64 asm330lxh_get_time_ns(void)
{
	struct timespec ts;

	/*
	 * calls getnstimeofday.
	 * If hrtimers then up to ns accurate, if not microsecond.
	 */
	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

struct asm330lxh_main_platform_data default_asm330lxh_main_platform_data = {
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
	.gpio_int1 = ASM330LXH_INT1_GPIO_DEF,
	.gpio_int2 = ASM330LXH_INT2_GPIO_DEF,
};

struct interrupt_enable {
	atomic_t enable;
	uint8_t address;
	uint8_t mask;
};

struct interrupt_value {
	int32_t value;
	uint8_t address;
};

/* Function Prototype */
static int32_t asm330lxh_gyr_update_odr(struct asm330lxh_status *stat,
					uint32_t poll_interval_us);

static int32_t asm330lxh_acc_device_power_off(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t buf = (ASM330LXH_ACC_ODR_MASK & ASM330LXH_ACC_ODR_OFF) | 
		       ((~ASM330LXH_ACC_ODR_MASK) & 
		       device_registers.ctrl1_xl.resume_val);

	err = stat->tf->write(stat, device_registers.ctrl1_xl.address, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "accelerometer soft power off "
			"failed: %d\n", err);

	if (stat->pdata_acc->power_off) {
		stat->pdata_acc->power_off();
	}

	if (!atomic_read(&stat->on_before_suspend))
		atomic_set(&stat->enabled_acc, 0);
	
	dev_info(stat->dev, "accelerometer switched off.");

	/* Restore Gyro ODR */
	if (atomic_read(&stat->enabled_gyr)) {
		// If gyro ODR != of latest requested ODR
		if (stat->pdata_gyr->poll_interval != stat->gyr_current_ODR_interval_us)
			asm330lxh_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	}

	return 0;
}

static int32_t asm330lxh_gyr_device_power_off(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t buf = (ASM330LXH_GYR_ODR_MASK & ASM330LXH_GYR_ODR_OFF) | 
		       ((~ASM330LXH_GYR_ODR_MASK) & 
		       device_registers.ctrl2_g.resume_val);

	err = stat->tf->write(stat, device_registers.ctrl2_g.address, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "gyroscope soft power off "
			"failed: %d\n", err);

	if (stat->pdata_gyr->power_off)
		stat->pdata_gyr->power_off();

	if (!atomic_read(&stat->on_before_suspend))
		atomic_set(&stat->enabled_gyr, 0);

	dev_info(stat->dev, "gyroscope switched off.");

	return 0;
}

static int32_t _asm330lxh_gyr_disable(struct asm330lxh_status *stat)
{
	cancel_work_sync(&stat->input_work_gyr);
	hrtimer_cancel(&stat->hr_timer_gyr);
	asm330lxh_gyr_device_power_off(stat);

	return 0;
}

static int32_t asm330lxh_gyr_disable(struct asm330lxh_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_gyr, 1, 0))
		return _asm330lxh_gyr_disable(stat);

	return 0;
}

static int32_t _asm330lxh_acc_disable(struct asm330lxh_status *stat)
{
	cancel_work_sync(&stat->input_work_acc);
	hrtimer_cancel(&stat->hr_timer_acc);
	asm330lxh_acc_device_power_off(stat);

	return 0;
}

static int32_t asm330lxh_acc_disable(struct asm330lxh_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_acc, 1, 0))
		return _asm330lxh_acc_disable(stat);

	return 0;
}

static void asm330lxh_acc_input_cleanup(struct asm330lxh_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);
}

static void asm330lxh_gyr_input_cleanup(struct asm330lxh_status *stat)
{
	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

static enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct asm330lxh_status *stat;

	stat = container_of((struct hrtimer *)timer,
			    struct asm330lxh_status, hr_timer_acc);

	stat->acc_ts = asm330lxh_get_time_ns();

	queue_work(asm330lxh_workqueue, &stat->input_work_acc);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart poll_function_read_gyr(struct hrtimer *timer)
{
	struct asm330lxh_status *stat;

	stat = container_of((struct hrtimer *)timer,
			    struct asm330lxh_status, hr_timer_gyr);

	stat->gyr_ts = asm330lxh_get_time_ns();

	queue_work(asm330lxh_workqueue, &stat->input_work_gyr);

	return HRTIMER_NORESTART;
}

static void asm330lxh_validate_polling(uint32_t * min_interval,
				       uint32_t * poll_interval, uint32_t min)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);
}

static int32_t asm330lxh_acc_validate_pdata(struct asm330lxh_status *stat)
{

	asm330lxh_validate_polling(&stat->pdata_acc->min_interval,
				   &stat->pdata_acc->poll_interval,
				   (unsigned int)
				   ASM330LXH_ACC_MIN_POLL_PERIOD_US);

	return 1;
}

static int32_t asm330lxh_gyr_validate_pdata(struct asm330lxh_status *stat)
{
	/* checks for correctness of minimal polling period */
	asm330lxh_validate_polling(&stat->pdata_gyr->min_interval,
				   &stat->pdata_gyr->poll_interval,
				   (unsigned int)
				   ASM330LXH_GYR_MIN_POLL_PERIOD_US);

	/* Enforce minimum polling interval */
	if (stat->pdata_gyr->poll_interval < stat->pdata_gyr->min_interval) {
		dev_err(stat->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int32_t asm330lxh_acc_gyr_hw_init(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t buf;

	dev_info(stat->dev, "%s: hw init start\n", ASM330LXH_ACC_GYR_DEV_NAME);

	err = stat->tf->read(stat, WHO_AM_I, 1, &buf);
	if (err < 0) {
		dev_warn(stat->dev, 
			 "Error reading WHO_AM_I: is device available/working?\n");
		return err;
	}

	if (buf != WHO_AM_I_VAL) {
		dev_err(stat->dev, "device unknown: Expected: 0x%02x Replies: 0x%02x\n",
			WHO_AM_I_VAL, buf);
		return -1;
	}

	stat->acc_discard_samples = 0;
	stat->gyr_discard_samples = 0;

	dev_info(stat->dev, "%s: hw init done\n", ASM330LXH_ACC_GYR_DEV_NAME);

	return 0;
}

static int32_t asm330lxh_acc_device_power_on(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t buf[3] = { 0 };

	if (stat->pdata_acc->power_on) {
		err = stat->pdata_acc->power_on();
		if (err < 0) {
			dev_err(stat->dev, "accelerometer power_on failed: %d\n", err);
			return err;
		}
	}

	err = stat->tf->write(stat, device_registers.ctrl1_xl.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = device_registers.ctrl3_c.resume_val;
	buf[1] = device_registers.ctrl4_c.resume_val;
	buf[2] = device_registers.ctrl5_c.resume_val;
	err = stat->tf->write(stat, device_registers.ctrl3_c.address, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = device_registers.ctrl8_xl.resume_val;
	buf[1] = device_registers.ctrl9_xl.resume_val;
	buf[2] = device_registers.ctrl10_c.resume_val;
	err = stat->tf->write(stat, device_registers.ctrl8_xl.address, 3, buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&stat->enabled_acc, 1);
	
	return 0;

err_resume_state:
	atomic_set(&stat->enabled_acc, 0);
	dev_err(stat->dev, "accelerometer hw power on error : %d\n", err);

	return err;
}

static int32_t asm330lxh_gyr_device_power_on(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t buf[6] = { 0 };

	if (stat->pdata_gyr->power_on) {
		err = stat->pdata_gyr->power_on();
		if (err < 0) {
			dev_err(stat->dev, "gyroscope power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = device_registers.ctrl2_g.resume_val;
	buf[1] = device_registers.ctrl3_c.resume_val;
	buf[2] = device_registers.ctrl4_c.resume_val;
	buf[3] = device_registers.ctrl5_c.resume_val;
	buf[4] = device_registers.ctrl6_g.resume_val;
	buf[5] = device_registers.ctrl7_g.resume_val;
	err = stat->tf->write(stat, device_registers.ctrl2_g.address, 6, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = device_registers.ctrl10_c.resume_val;
	err = stat->tf->write(stat, device_registers.ctrl10_c.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&stat->enabled_gyr, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_gyr, 0);
	dev_err(stat->dev, "gyroscope hw power on error: %d\n", err);

	return err;
}

static int32_t asm330lxh_acc_update_fs_range(struct asm330lxh_status *stat,
					     uint8_t new_fs_range)
{
	int32_t sensitivity, err = -1;
	uint8_t val;

	switch (new_fs_range) {
	case ASM330LXH_ACC_FS_2G:
		sensitivity = SENSITIVITY_ACC_2G;
		break;
	case ASM330LXH_ACC_FS_4G:
		sensitivity = SENSITIVITY_ACC_4G;
		break;
	case ASM330LXH_ACC_FS_8G:
		sensitivity = SENSITIVITY_ACC_8G;
		break;
	case ASM330LXH_ACC_FS_16G:
		sensitivity = SENSITIVITY_ACC_16G;
		break;
	default:
		dev_err(stat->dev, "invalid accelerometer "
			"fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	val = ((ASM330LXH_ACC_FS_MASK & new_fs_range) |
	       ((~ASM330LXH_ACC_FS_MASK) & device_registers.ctrl1_xl.resume_val));

	err = stat->tf->write(stat, device_registers.ctrl1_xl.address, 1, &val);
	if (err < 0)
		goto error;

	device_registers.ctrl1_xl.resume_val = val;

	mutex_lock(&stat->lock);
	stat->sensitivity_acc = sensitivity;
	mutex_unlock(&stat->lock);

	return err;

error:
	dev_err(stat->dev, "update accelerometer fs range failed: %d\n", err);

	return err;
}

static int32_t asm330lxh_gyr_update_fs_range(struct asm330lxh_status *stat,
					     uint8_t new_fs_range)
{
	int32_t err = -1;
	uint8_t updated_val, buf;
	u32 sensitivity;

	switch (new_fs_range) {
	case ASM330LXH_GYR_FS_245DPS:
		sensitivity = SENSITIVITY_GYR_250;
		break;
	case ASM330LXH_GYR_FS_500DPS:
		sensitivity = SENSITIVITY_GYR_500;
		break;
	case ASM330LXH_GYR_FS_1000DPS:
		sensitivity = SENSITIVITY_GYR_1000;
		break;
	case ASM330LXH_GYR_FS_2000DPS:
		sensitivity = SENSITIVITY_GYR_2000;
		break;
	default:
		dev_err(stat->dev, "invalid g range "
			"requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	err = stat->tf->read(stat, device_registers.ctrl2_g.address, 1, &buf);
	if (err < 0)
		goto error;

	updated_val = ((ASM330LXH_GYR_FS_MASK & new_fs_range) |
		       ((~ASM330LXH_GYR_FS_MASK) & buf));

	err = stat->tf->write(stat, device_registers.ctrl2_g.address, 1, &updated_val);
	if (err < 0)
		goto error;

	device_registers.ctrl2_g.resume_val = updated_val;

	mutex_lock(&stat->lock);
	stat->sensitivity_gyr = sensitivity;
	mutex_unlock(&stat->lock);

error:
	return err;
}

static int32_t asm330lxh_acc_update_odr(struct asm330lxh_status *stat,
					uint32_t poll_interval_us)
{
	int32_t err = -1;
	uint8_t buf;
	uint32_t i;

	if (atomic_read(&stat->enabled_acc)) {
		for (i = ARRAY_SIZE(asm330lxh_acc_odr_table) - 1; i >= 0; i--) {
			if (((uint32_t)asm330lxh_acc_odr_table[i].odr_us
			     <= poll_interval_us) || (i == 0))
				break;
		}

		buf = ASM330LXH_ACC_ODR_MASK & asm330lxh_acc_odr_table[i].value;
		buf |= (~ASM330LXH_ACC_ODR_MASK) & device_registers.ctrl1_xl.resume_val;

		err = stat->tf->write(stat, device_registers.ctrl1_xl.address, 1, &buf);
		if (err < 0)
			goto error;

		device_registers.ctrl1_xl.resume_val = buf;

		/* Adjust Gyro ODR value */
		if (atomic_read(&stat->enabled_gyr) &&
		    (poll_interval_us < stat->gyr_current_ODR_interval_us))
			asm330lxh_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);

		mutex_lock(&stat->lock);
		stat->ktime_acc = ktime_set(0, US_TO_NS(poll_interval_us));
		stat->pdata_acc->poll_interval = poll_interval_us;
		mutex_unlock(&stat->lock);
	} else {
		mutex_lock(&stat->lock);
		stat->pdata_acc->poll_interval = poll_interval_us;
		mutex_unlock(&stat->lock);
		err = 0;
	}

	return err;

error:
	dev_err(stat->dev, "update accelerometer odr failed: %d\n", err);

	return err;
}

static int32_t asm330lxh_gyr_update_odr(struct asm330lxh_status *stat,
					uint32_t poll_interval_us)
{
	uint8_t buf;
	uint32_t val, i;
	int32_t err = -1;

	if (atomic_read(&stat->enabled_gyr)) {
		// min poll rate from axl and gyro
		if (atomic_read(&stat->enabled_acc))
			val = min(poll_interval_us, stat->pdata_acc->poll_interval);
		else
			val = poll_interval_us;

		for (i = ARRAY_SIZE(asm330lxh_gyr_odr_table) - 1; i >= 0; i--) {
			if ((asm330lxh_gyr_odr_table[i].odr_us <= val)
			    || (i == 0))
				break;
		}

		/* Set ODR value */
		buf = ASM330LXH_GYR_ODR_MASK & asm330lxh_gyr_odr_table[i].value;
		buf |= (~ASM330LXH_GYR_ODR_MASK) & device_registers.ctrl2_g.resume_val;

		err = stat->tf->write(stat, device_registers.ctrl2_g.address, 1, &buf);
		if (err < 0)
			goto error;

		device_registers.ctrl2_g.resume_val = buf;

		/* Enable all axes */
		buf = CTRL10_C_ALLAXIS_EN | device_registers.ctrl10_c.resume_val;

		err = stat->tf->write(stat, device_registers.ctrl10_c.address, 1, &buf);
		if (err < 0)
			goto error;

		device_registers.ctrl10_c.resume_val = buf;

		mutex_lock(&stat->lock);
		stat->ktime_gyr = ktime_set(0, US_TO_NS(poll_interval_us));
		stat->pdata_gyr->poll_interval = poll_interval_us;
		stat->gyr_current_ODR_interval_us = val;
		mutex_unlock(&stat->lock);
	} else {
		mutex_lock(&stat->lock);
		stat->pdata_gyr->poll_interval = poll_interval_us;
		mutex_unlock(&stat->lock);
		err = 0;
	}

	return err;

error:	
	dev_err(stat->dev, "update accelerometer odr failed: %d\n", err);

	return err;
}

static int32_t _asm330lxh_acc_enable(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t j;

	err = asm330lxh_acc_device_power_on(stat);
	if (err < 0) {
		atomic_set(&stat->enabled_acc, 0);
		dev_err(stat->dev, "enable accelerometer failed");
		return err;
	}

	for (j = ARRAY_SIZE(asm330lxh_axl_turn_on_time) - 1; j >= 0; j--) {
		if ((asm330lxh_axl_turn_on_time[j].odr_us <=
		     stat->pdata_acc->poll_interval) || (j == 0))
			break;
	}

	stat->acc_discard_samples =
	    asm330lxh_axl_turn_on_time[j].samples_to_discard;

	err = asm330lxh_acc_update_odr(stat, stat->pdata_acc->poll_interval);
	if (err < 0)
		return err;

	hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc, HRTIMER_MODE_REL);

	return 0;
}

static int32_t asm330lxh_acc_enable(struct asm330lxh_status *stat)
{
	if (!atomic_cmpxchg(&stat->enabled_acc, 0, 1))
		return _asm330lxh_acc_enable(stat);

	return 0;
}

static int32_t _asm330lxh_gyr_enable(struct asm330lxh_status *stat)
{
	int32_t err = -1;
	uint8_t j;

	err = asm330lxh_gyr_device_power_on(stat);
	if (err < 0) {
		atomic_set(&stat->enabled_gyr, 0);
		return err;
	}

	for (j = ARRAY_SIZE(asm330lxh_gyr_turn_on_time) - 1; j >= 0; j--) {
		if ((asm330lxh_gyr_turn_on_time[j].odr_us <=
		     stat->pdata_gyr->poll_interval) || (j == 0))
			break;
	}

	stat->gyr_discard_samples =
	    asm330lxh_gyr_turn_on_time[j].samples_to_discard;

	err = asm330lxh_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	if (err < 0)
		return err;

	hrtimer_start(&(stat->hr_timer_gyr), stat->ktime_gyr, HRTIMER_MODE_REL);

	return 0;
}

static int32_t asm330lxh_gyr_enable(struct asm330lxh_status *stat)
{
	if (!atomic_cmpxchg(&stat->enabled_gyr, 0, 1))
		return _asm330lxh_gyr_enable(stat);

	return 0;
}

#ifdef ASM330LXH_EN_ON_OPEN
int32_t asm330lxh_acc_input_open(struct input_dev * input)
{
	struct asm330lxh_status *stat = input_get_drvdata(input);

	return asm330lxh_acc_enable(stat);
}

void asm330lxh_acc_input_close(struct input_dev *dev)
{
	struct asm330lxh_status *stat = input_get_drvdata(dev);

	asm330lxh_acc_disable(stat);
}

int32_t asm330lxh_gyr_input_open(struct input_dev *input)
{
	struct asm330lxh_status *stat = input_get_drvdata(input);

	return asm330lxh_gyr_enable(stat);
}

void asm330lxh_gyr_input_close(struct input_dev *dev)
{
	struct asm330lxh_status *stat = input_get_drvdata(dev);

	asm330lxh_gyr_disable(stat);
}
#endif

static int32_t asm330lxh_acc_get_data(struct asm330lxh_status *stat,
				      int32_t * xyz)
{
	int32_t i, err = -1, hw_d[3] = { 0 };
	uint8_t acc_data[6];

	err = stat->tf->read(stat, OUT_X_L_XL, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((int32_t)((int16_t)((acc_data[1] << 8) | (acc_data[0]))));
	hw_d[1] = ((int32_t)((int16_t)((acc_data[3] << 8) | (acc_data[2]))));
	hw_d[2] = ((int32_t)((int16_t)((acc_data[5] << 8) | (acc_data[4]))));

	mutex_lock(&stat->lock);
	hw_d[0] = hw_d[0] * stat->sensitivity_acc;
	hw_d[1] = hw_d[1] * stat->sensitivity_acc;
	hw_d[2] = hw_d[2] * stat->sensitivity_acc;

	for (i = 0; i < 3; i++) {
		xyz[i] = stat->pdata_main->rot_matrix[0][i] * hw_d[0] +
			 stat->pdata_main->rot_matrix[1][i] * hw_d[1] +
			 stat->pdata_main->rot_matrix[2][i] * hw_d[2];
	}
	mutex_unlock(&stat->lock);

	return err;
}

static int32_t asm330lxh_gyr_get_data(struct asm330lxh_status *stat,
				      int32_t * xyz)
{
	int32_t i, err = 1, hw_d[3] = { 0 };
	uint8_t gyro_data[6];

	err = stat->tf->read(stat, OUT_X_L_G, 6, gyro_data);

	if (err < 0)
		return err;

	hw_d[0] = (int32_t)((int16_t)((gyro_data[1]) << 8) | gyro_data[0]);
	hw_d[1] = (int32_t)((int16_t)((gyro_data[3]) << 8) | gyro_data[2]);
	hw_d[2] = (int32_t)((int16_t)((gyro_data[5]) << 8) | gyro_data[4]);

	mutex_lock(&stat->lock);

	hw_d[0] = hw_d[0] * stat->sensitivity_gyr;
	hw_d[1] = hw_d[1] * stat->sensitivity_gyr;
	hw_d[2] = hw_d[2] * stat->sensitivity_gyr;

	for (i = 0; i < 3; i++) {
		xyz[i] = stat->pdata_main->rot_matrix[0][i] * hw_d[0] +
			 stat->pdata_main->rot_matrix[1][i] * hw_d[1] +
			 stat->pdata_main->rot_matrix[2][i] * hw_d[2];
	}
	mutex_unlock(&stat->lock);
	return err;
}

static void asm330lxh_acc_report_values(struct asm330lxh_status *stat,
					int32_t * xyz)
{
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    stat->acc_ts >> 32);
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    stat->acc_ts & 0xffffffff);
	input_sync(stat->input_dev_acc);
}

static void asm330lxh_gyr_report_values(struct asm330lxh_status *stat,
					int32_t * xyz)
{
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    stat->gyr_ts >> 32);
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    stat->gyr_ts & 0xffffffff);
	input_sync(stat->input_dev_gyr);
}

static int32_t asm330lxh_acc_input_init(struct asm330lxh_status *stat)
{
	int32_t err = -1;

	mutex_lock(&stat->lock);
	stat->input_dev_acc = input_allocate_device();
	if (!stat->input_dev_acc) {
		err = -ENOMEM;
		dev_err(stat->dev,
			"accelerometer input device allocation failed\n");
		mutex_unlock(&stat->lock);
		return err;
	}

#ifdef ASM330LXH_EN_ON_OPEN
	stat->input_dev_acc->open = asm330lxh_acc_input_open;
	stat->input_dev_acc->close = asm330lxh_acc_input_close;
#endif
	stat->input_dev_acc->name = ASM330LXH_ACC_DEV_NAME;
	stat->input_dev_acc->id.bustype = stat->bustype;
	stat->input_dev_acc->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev_acc, stat);

	__set_bit(INPUT_EVENT_TYPE, stat->input_dev_acc->evbit);
	__set_bit(INPUT_EVENT_X, stat->input_dev_acc->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev_acc->mscbit);
	__set_bit(INPUT_EVENT_Z, stat->input_dev_acc->mscbit);
	__set_bit(INPUT_EVENT_TIME_MSB, stat->input_dev_acc->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, stat->input_dev_acc->mscbit);

	err = input_register_device(stat->input_dev_acc);
	if (err) {
		dev_err(stat->dev, "unable to register input device %s\n",
			stat->input_dev_acc->name);
		input_free_device(stat->input_dev_acc);
	}
	mutex_unlock(&stat->lock);

	return err;
}

static int32_t asm330lxh_gyr_input_init(struct asm330lxh_status *stat)
{
	int32_t err = -1;

	dev_dbg(stat->dev, "%s\n", __func__);

	mutex_lock(&stat->lock);
	stat->input_dev_gyr = input_allocate_device();
	if (!stat->input_dev_gyr) {
		err = -ENOMEM;
		dev_err(stat->dev, "input device allocation failed\n");
		mutex_unlock(&stat->lock);
		return err;
	}

#ifdef ASM330LXH_EN_ON_OPEN
	stat->input_dev_gyr->open = asm330lxh_gyr_input_open;
	stat->input_dev_gyr->close = asm330lxh_gyr_input_close;
#endif
	stat->input_dev_gyr->name = ASM330LXH_GYR_DEV_NAME;
	stat->input_dev_gyr->id.bustype = stat->bustype;
	stat->input_dev_gyr->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev_gyr, stat);

	__set_bit(INPUT_EVENT_TYPE, stat->input_dev_gyr->evbit);
	__set_bit(INPUT_EVENT_X, stat->input_dev_gyr->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev_gyr->mscbit);
	__set_bit(INPUT_EVENT_Z, stat->input_dev_gyr->mscbit);
	__set_bit(INPUT_EVENT_TIME_MSB, stat->input_dev_gyr->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, stat->input_dev_gyr->mscbit);

	err = input_register_device(stat->input_dev_gyr);
	if (err) {
		dev_err(stat->dev, "unable to register input device %s\n",
			stat->input_dev_gyr->name);
		input_free_device(stat->input_dev_gyr);
	}
	mutex_unlock(&stat->lock);

	return err;
}

static void asm330lxh_input_cleanup(struct asm330lxh_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);

	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

static ssize_t attr_set_polling_rate_acc(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t size)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned long)max((unsigned int)MS_TO_US(interval_ms),
					 stat->pdata_acc->min_interval);

	asm330lxh_acc_update_odr(stat, interval_us);

	return size;
}

static ssize_t attr_get_polling_rate_acc(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	uint32_t val = 0;
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%u\n", US_TO_MS(val));
}

static ssize_t attr_get_enable_acc(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);

	int32_t val = (int)atomic_read(&stat->enabled_acc);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_acc(struct kobject *kobj,
				   struct kobj_attribute *attr, const char *buf,
				   size_t size)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		asm330lxh_acc_enable(stat);
	else
		asm330lxh_acc_disable(stat);

	return size;
}

static ssize_t attr_get_range_acc(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	struct device
	*dev = to_dev(kobj->parent);
	uint8_t val;
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	int32_t range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->fs_range;
	mutex_unlock(&stat->lock);

	switch (val) {
	case ASM330LXH_ACC_FS_2G:
		range = RANGE_2G;
		break;
	case ASM330LXH_ACC_FS_4G:
		range = RANGE_4G;
		break;
	case ASM330LXH_ACC_FS_8G:
		range = RANGE_8G;
		break;
	case ASM330LXH_ACC_FS_16G:
		range = RANGE_16G;
		break;
	}
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_acc(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t size)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case RANGE_2G:
		range = ASM330LXH_ACC_FS_2G;
		break;
	case RANGE_4G:
		range = ASM330LXH_ACC_FS_4G;
		break;
	case RANGE_8G:
		range = ASM330LXH_ACC_FS_8G;
		break;
	case RANGE_16G:
		range = ASM330LXH_ACC_FS_16G;
		break;
	default:
		dev_err(stat->dev, "accelerometer invalid range "
			"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = asm330lxh_acc_update_fs_range(stat, range);
	if (err < 0)
		return err;

	mutex_lock(&stat->lock);
	stat->pdata_acc->fs_range = range;
	mutex_unlock(&stat->lock);

	dev_info(stat->dev, "accelerometer range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_polling_rate_gyr(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	uint32_t val;
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", US_TO_MS(val));
}

static ssize_t attr_set_polling_rate_gyr(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t size)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned int)max((unsigned int)MS_TO_US(interval_ms),
					stat->pdata_gyr->min_interval);

	asm330lxh_gyr_update_odr(stat, interval_us);

	return size;
}

static ssize_t attr_get_enable_gyr(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	int32_t val = atomic_read(&stat->enabled_gyr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_gyr(struct kobject *kobj,
				   struct kobj_attribute *attr, const char *buf,
				   size_t size)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		asm330lxh_gyr_enable(stat);
	else
		asm330lxh_gyr_disable(stat);

	return size;
}

static ssize_t attr_get_range_gyr(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	int32_t range = 0;
	uint8_t val;

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->fs_range;
	switch (val) {
	case ASM330LXH_GYR_FS_245DPS:
		range = RANGE_245DPS;
		break;
	case ASM330LXH_GYR_FS_500DPS:
		range = RANGE_500DPS;
		break;
	case ASM330LXH_GYR_FS_1000DPS:
		range = RANGE_1000DPS;
		break;
	case ASM330LXH_GYR_FS_2000DPS:
		range = RANGE_2000DPS;
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_gyr(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t size)
{
	struct device
	*dev = to_dev(kobj->parent);
	struct asm330lxh_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err = -1;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 245:
		range = ASM330LXH_GYR_FS_245DPS;
		break;
	case 500:
		range = ASM330LXH_GYR_FS_500DPS;
		break;
	case 1000:
		range = ASM330LXH_GYR_FS_1000DPS;
		break;
	case 2000:
		range = ASM330LXH_GYR_FS_2000DPS;
		break;
	default:
		dev_err(stat->dev, "invalid range request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = asm330lxh_gyr_update_fs_range(stat, range);
	if (err >= 0) {
		mutex_lock(&stat->lock);
		stat->pdata_gyr->fs_range = range;
		mutex_unlock(&stat->lock);
	}

	dev_info(stat->dev, "range set to: %lu dps\n", val);

	return size;
}

static struct kobj_attribute poll_attr_acc =
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_acc,
	       attr_set_polling_rate_acc);
static struct kobj_attribute enable_attr_acc =
	__ATTR(enable_device, 0666, attr_get_enable_acc, attr_set_enable_acc);
static struct kobj_attribute fs_attr_acc =
	__ATTR(range, 0666, attr_get_range_acc, attr_set_range_acc);
static struct kobj_attribute poll_attr_gyr =
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_gyr,
	       attr_set_polling_rate_gyr);
static struct kobj_attribute enable_attr_gyr =
	__ATTR(enable_device, 0666, attr_get_enable_gyr, attr_set_enable_gyr);
static struct kobj_attribute range_attr_gyr =
	__ATTR(range, 0666, attr_get_range_gyr, attr_set_range_gyr);

static struct attribute *attributes_acc[] = { &poll_attr_acc.attr,
	&enable_attr_acc.attr, &fs_attr_acc.attr,
	NULL,
};

static struct attribute *attributes_gyr[] = { &poll_attr_gyr.attr,
	&enable_attr_gyr.attr, &range_attr_gyr.attr,
	NULL,
};

static struct attribute_group attr_group_acc = {
	.attrs = attributes_acc,
};

static struct attribute_group attr_group_gyr = {
	.attrs = attributes_gyr,
};

static int32_t create_sysfs_interfaces(struct device *dev)
{
	int32_t err = -1;

	acc_kobj = kobject_create_and_add("accelerometer", &dev->kobj);
	if (!acc_kobj)
		return -ENOMEM;

	gyr_kobj = kobject_create_and_add("gyroscope", &dev->kobj);
	if (!gyr_kobj)
		return -ENOMEM;

	err = sysfs_create_group(acc_kobj, &attr_group_acc);
	if (err)
		kobject_put(acc_kobj);

	err = sysfs_create_group(gyr_kobj, &attr_group_gyr);
	if (err)
		kobject_put(gyr_kobj);

	return 0;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	kobject_put(acc_kobj);
	kobject_put(gyr_kobj);
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
	struct asm330lxh_status *stat;
	int32_t xyz[3] = { 0 }, err = -1;

	stat = container_of((struct work_struct *)input_work_acc,
			    struct asm330lxh_status, input_work_acc);

	err = asm330lxh_acc_get_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get accelerometer data failed\n");
	else {
		mutex_lock(&stat->lock);
		if (stat->acc_discard_samples > 0) {
			stat->acc_discard_samples--;
			mutex_unlock(&stat->lock);
		} else {
			mutex_unlock(&stat->lock);
			asm330lxh_acc_report_values(stat, xyz);
		}
	}

	hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc, HRTIMER_MODE_REL);
}

static void poll_function_work_gyr(struct work_struct *input_work_gyr)
{
	struct asm330lxh_status *stat;
	int32_t xyz[3] = { 0 }, err = -1;

	stat = container_of((struct work_struct *)input_work_gyr,
			    struct asm330lxh_status, input_work_gyr);

	err = asm330lxh_gyr_get_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get gyroscope data failed.\n");
	else
		asm330lxh_gyr_report_values(stat, xyz);

	hrtimer_start(&stat->hr_timer_gyr, stat->ktime_gyr, HRTIMER_MODE_REL);
}

#ifdef CONFIG_OF
static int32_t asm330lxh_acc_gyr_parse_dt(struct asm330lxh_status *stat,
					  struct device *dev)
{
	struct device_node *dn;
	uint8_t i, j;
	uint32_t val;
	uint8_t range;
	short vect[9] = { 0 };

	mutex_lock(&stat->lock);
	dn = dev->of_node;
	stat->pdata_main->of_node = dn;

	stat->pdata_main->gpio_int1 = of_get_gpio(dn, 0);
	if (!gpio_is_valid(stat->pdata_main->gpio_int1)) {
		dev_err(dev, "INT1 gpio not supported\n");
		stat->pdata_main->gpio_int1 = ASM330LXH_INT1_GPIO_DEF;
	}

	stat->pdata_main->gpio_int2 = of_get_gpio(dn, 1);
	if (!gpio_is_valid(stat->pdata_main->gpio_int2)) {
		dev_err(dev, "INT2 gpio not supported\n");
		stat->pdata_main->gpio_int2 = ASM330LXH_INT2_GPIO_DEF;
	}

	if (of_property_read_u16_array(dn, "rot-matrix", vect,
				       ARRAY_SIZE(vect)) >= 0) {
		for (j = 0; j < 3; j++) {
			for (i = 0; i < 3; i++) {
				stat->pdata_main->rot_matrix[i][j] =
				    (short)vect[3 * j + i];
			}
		}
	} else {
		for (j = 0; j < 3; j++) {
			for (i = 0; i < 3; i++) {
				stat->pdata_main->rot_matrix[i][j] =
				    default_asm330lxh_main_platform_data.rot_matrix[i][j];
			}
		}
	}

	if (!of_property_read_u32(dn, "g-poll-interval-ms", &val))
		stat->pdata_gyr->poll_interval = MS_TO_US(val);
	else
		stat->pdata_gyr->poll_interval = ASM330LXH_GYR_POLL_INTERVAL_DEF;

	if (!of_property_read_u32(dn, "g-min-interval-ms", &val))
		stat->pdata_gyr->min_interval = MS_TO_US(val);
	else
		stat->pdata_gyr->min_interval = ASM330LXH_GYR_MIN_POLL_PERIOD_US;

	if (!of_property_read_u32(dn, "g-fs-range-dps", &val)) {
		switch (val) {
		case RANGE_245DPS:
			range = ASM330LXH_GYR_FS_245DPS;
			break;
		case RANGE_500DPS:
			range = ASM330LXH_GYR_FS_500DPS;
			break;
		case RANGE_1000DPS:
			range = ASM330LXH_GYR_FS_1000DPS;
			break;
		case RANGE_2000DPS:
			range = ASM330LXH_GYR_FS_2000DPS;
			break;
		default:
			range = ASM330LXH_GYR_FS_2000DPS;
			break;
		}

		stat->pdata_gyr->fs_range = range;
	} else
		stat->pdata_gyr->fs_range = ASM330LXH_GYR_FS_2000DPS;

	if (!of_property_read_u32(dn, "x-poll-interval-ms", &val))
		stat->pdata_acc->poll_interval =  MS_TO_US(val);
	else
		stat->pdata_acc->poll_interval = ASM330LXH_ACC_POLL_INTERVAL_DEF;

	if (!of_property_read_u32(dn, "x-min-interval-ms", &val))
		stat->pdata_acc->min_interval =  MS_TO_US(val);
	else
		stat->pdata_acc->min_interval = ASM330LXH_ACC_MIN_POLL_PERIOD_US;

	if (!of_property_read_u32(dn, "x-fs-range-g", &val)) {
		switch (val) {
		case RANGE_2G:
			range = ASM330LXH_ACC_FS_2G;
			break;
		case RANGE_4G:
			range = ASM330LXH_ACC_FS_4G;
			break;
		case RANGE_8G:
			range = ASM330LXH_ACC_FS_8G;
			break;
		case RANGE_16G:
			range = ASM330LXH_ACC_FS_16G;
			break;
		default:
			range = ASM330LXH_ACC_FS_4G;
			break;
		}

		stat->pdata_acc->fs_range = range;
	} else
		stat->pdata_acc->fs_range = ASM330LXH_ACC_FS_4G;

	mutex_unlock(&stat->lock);

	return 0;
}
#endif

int32_t asm330lxh_common_probe(struct asm330lxh_status *stat)
{
	int32_t err = -1;

	dev_info(stat->dev, "probe start.\n");

	if (!asm330lxh_workqueue)
		asm330lxh_workqueue = create_workqueue("asm330lxh_workqueue");

	hrtimer_init(&stat->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_acc.function = &poll_function_read_acc;
	hrtimer_init(&stat->hr_timer_gyr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_gyr.function = &poll_function_read_gyr;

	mutex_init(&stat->lock);
	mutex_init(&stat->tb.buf_lock);

	stat->pdata_main = kzalloc(sizeof(*stat->pdata_main), GFP_KERNEL);
	stat->pdata_acc = kzalloc(sizeof(*stat->pdata_acc), GFP_KERNEL);
	stat->pdata_gyr = kzalloc(sizeof(*stat->pdata_gyr), GFP_KERNEL);

	if ((stat->pdata_main == NULL) || (stat->pdata_acc == NULL)
	    || (stat->pdata_gyr == NULL)) {
		err = -ENOMEM;
		dev_err(stat->dev, "failed to allocate memory for pdata: %d\n",
			err);
		goto err_memory_alloc;
	}

	stat->pdata_main->pdata_acc = stat->pdata_acc;
	stat->pdata_main->pdata_gyr = stat->pdata_gyr;

#ifdef CONFIG_OF
	/* Device Tree */
	asm330lxh_acc_gyr_parse_dt(stat, stat->dev);
#else
	/* Board File */
	if (client->dev.platform_data == NULL) {
		memcpy(stat->pdata_main, &default_asm330lxh_main_platform_data,
		       sizeof(*stat->pdata_main));
		memcpy(stat->pdata_acc, &default_asm330lxh_acc_pdata,
		       sizeof(*stat->pdata_acc));
		memcpy(stat->pdata_gyr, &default_asm330lxh_gyr_pdata,
		       sizeof(*stat->pdata_gyr));
		dev_info(stat->dev, "using default plaform_data for "
			 "accelerometer and gyroscope\n");
	} else {
		struct asm330lxh_main_platform_data *platform_data;
		platform_data = client->dev.platform_data;

		if (platform_data == NULL) {
			memcpy(stat->pdata_main,fv
			       &default_asm330lxh_main_platform_data,
			       sizeof(*stat->pdata_main));
			dev_info(stat->dev,
				 "using default plaform_data for "
				 "accelerometer\n");
		} else {
			memcpy(stat->pdata_main, platform_data,
			       sizeof(*stat->pdata_acc));
		}

		if (platform_data->pdata_acc == NULL) {
			memcpy(stat->pdata_acc, &default_asm330lxh_acc_pdata,
			       sizeof(*stat->pdata_acc));
			dev_info(stat->dev, "using default plaform_data for "
				 "accelerometer\n");
		} else {
			memcpy(stat->pdata_acc, platform_data->pdata_acc,
			       sizeof(*stat->pdata_acc));
		}

		if (platform_data->pdata_gyr == NULL) {
			memcpy(stat->pdata_gyr, &default_asm330lxh_gyr_pdata,
			       sizeof(*stat->pdata_gyr));
			dev_info(stat->dev, "using default plaform_data for "
				 "gyroscope\n");
		} else {
			memcpy(stat->pdata_gyr, platform_data->pdata_gyr,
			       sizeof(*stat->pdata_gyr));
		}
	}
#endif // Board File

	err = asm330lxh_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data for "
			"accelerometer \n");
		goto exit_kfree_pdata;
	}

	err = asm330lxh_gyr_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data for "
			"gyroscope\n");
		goto exit_kfree_pdata;
	}

	if (stat->pdata_acc->init) {
		err = stat->pdata_acc->init();
		if (err < 0) {
			dev_err(stat->dev, "accelerometer init failed: "
				"%d\n", err);
			goto err_pdata_acc_init;
		}
	}
	if (stat->pdata_gyr->init) {
		err = stat->pdata_gyr->init();
		if (err < 0) {
			dev_err(stat->dev, "gyroscope init failed: "
				"%d\n", err);
			goto err_pdata_gyr_init;
		}
	}

	err = asm330lxh_acc_gyr_hw_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = asm330lxh_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "accelerometer power on failed: "
			"%d\n", err);
		goto err_pdata_init;
	}

	err = asm330lxh_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "gyroscope power on failed: %d\n", err);
		goto err_pdata_init;
	}

	err = asm330lxh_acc_update_fs_range(stat, stat->pdata_acc->fs_range);
	if (err < 0) {
		dev_err(stat->dev,
			"update accelerometer full scale range failed\n");
		goto err_power_off_acc;
	}

	err = asm330lxh_gyr_update_fs_range(stat, stat->pdata_gyr->fs_range);
	if (err < 0) {
		dev_err(stat->dev,
			"update gyroscope full scale range failed\n");
		goto err_power_off_gyr;
	}

	err = asm330lxh_acc_update_odr(stat, stat->pdata_acc->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update accelerometer ODR failed\n");
		goto err_power_off;
	}

	err = asm330lxh_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update gyroscope ODR failed\n");
		goto err_power_off;
	}

	err = asm330lxh_acc_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "accelerometer input init failed\n");
		goto err_power_off;
	}

	err = asm330lxh_gyr_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "gyroscope input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev, "device %s sysfs register failed\n",
			ASM330LXH_ACC_GYR_DEV_NAME);
		goto err_input_cleanup;
	}

	asm330lxh_acc_device_power_off(stat);
	asm330lxh_gyr_device_power_off(stat);

	INIT_WORK(&stat->input_work_acc, poll_function_work_acc);
	INIT_WORK(&stat->input_work_gyr, poll_function_work_gyr);

	dev_info(stat->dev, "%s: probed\n", ASM330LXH_ACC_GYR_DEV_NAME);

	return 0;

err_input_cleanup:
	asm330lxh_input_cleanup(stat);
err_power_off:
err_power_off_gyr:
	asm330lxh_gyr_device_power_off(stat);
err_power_off_acc:
	asm330lxh_acc_device_power_off(stat);
err_hw_init:
err_pdata_init:
err_pdata_gyr_init:
	if (stat->pdata_gyr->exit)
		stat->pdata_gyr->exit();
err_pdata_acc_init:
	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();
exit_kfree_pdata:
	mutex_lock(&stat->lock);
	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_main);
	mutex_unlock(&stat->lock);
err_memory_alloc:
	if (!asm330lxh_workqueue) {
		flush_workqueue(asm330lxh_workqueue);
		destroy_workqueue(asm330lxh_workqueue);
	}
	dev_err(stat->dev, "%s: Driver Init failed\n", ASM330LXH_ACC_GYR_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(asm330lxh_common_probe);

int32_t asm330lxh_common_remove(struct asm330lxh_status *stat)
{
	if (atomic_read(&stat->enabled_gyr)) {
		asm330lxh_gyr_disable(stat);
		asm330lxh_gyr_input_cleanup(stat);

		if (stat->pdata_gyr->exit)
			stat->pdata_gyr->exit();
	}

	asm330lxh_acc_disable(stat);
	asm330lxh_acc_input_cleanup(stat);

	remove_sysfs_interfaces(stat->dev);

	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();

	if (!asm330lxh_workqueue) {
		flush_workqueue(asm330lxh_workqueue);
		destroy_workqueue(asm330lxh_workqueue);
		asm330lxh_workqueue = NULL;
	}

	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_main);

	return 0;
}
EXPORT_SYMBOL(asm330lxh_common_remove);

#ifdef CONFIG_PM
int32_t asm330lxh_common_suspend(struct asm330lxh_status *stat)
{
	atomic_set(&stat->on_before_suspend, 1);
	if (atomic_read(&stat->enabled_gyr) > 0)
		_asm330lxh_gyr_disable(stat);

	if (atomic_read(&stat->enabled_acc) > 0) {
		_asm330lxh_acc_disable(stat);
	}

	return 0;
}
EXPORT_SYMBOL(asm330lxh_common_suspend);

int32_t asm330lxh_common_resume(struct asm330lxh_status *stat)
{
	if (atomic_read(&stat->enabled_acc) > 0) {
		_asm330lxh_acc_enable(stat);
	}

	if (atomic_read(&stat->enabled_gyr) > 0) {
		_asm330lxh_gyr_enable(stat);
	}
	atomic_set(&stat->on_before_suspend, 0);

	return 0;
}
EXPORT_SYMBOL(asm330lxh_common_resume);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("asm330lxh driver");
MODULE_AUTHOR("Giuseppe Barba, Alberto Marinoni, Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");
