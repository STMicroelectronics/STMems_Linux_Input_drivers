/******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
*
* File Name	: lsm9ds0_acc_mag.c
* Authors	: AMS - Motion Sensors Div - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
*		: Lorenzo Bianconi (lorenzo.bianconi@st.com)
*		: Authors are willing to be considered the contact
*		: and update points for the driver.
* Version       : V.1.0.5
* Date          : 2016/May/13
* Description   : LSM9DS0 accelerometer & magnetometer driver
*
*******************************************************************************
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
******************************************************************************/
/******************************************************************************
Version History.

Revision 1-0-0 2012/05/04
 first revision
Revision 1-0-1 2012/05/07
 New sysfs architecture
 Support antialiasing filter
Revision 1-0-2 2012/10/15
 I2C address bugfix
Revision 1-0-3 2013/01/21
 Move CTLREG7 resume write from acc_power_on to magn_power_on
Revision 1-0-4 2013/05/09
 Added rotation matrix
Revision 1-0-5 2013/10/23
 Corrects Mag Enable bug, Corrects missing BDU enable
******************************************************************************/

#include <linux/module.h>
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

#include "lsm9ds0.h"

#define MS_TO_NS(x)				((x) * 1000000L)
#define SEC_PORTION_FROM_MS(x)			(s64)((x) / 1000)
#define NSEC_PORTION_FROM_MS(x)			MS_TO_NS((x) % 1000)

/* Address registers */
#define REG_WHOAMI_ADDR		0x0F	/** Who am i address register */
#define REG_CNTRL0_ADDR		0x1F	/** CNTRL0 address register */
#define REG_CNTRL1_ADDR		0x20	/** CNTRL1 address register */
#define REG_CNTRL2_ADDR		0x21	/** CNTRL2 address register */
#define REG_CNTRL3_ADDR		0x22	/** CNTRL3 address register */
#define REG_CNTRL4_ADDR		0x23	/** CNTRL4 address register */
#define REG_CNTRL5_ADDR		0x24	/** CNTRL5 address register */
#define REG_CNTRL6_ADDR		0x25	/** CNTRL6 address register */
#define REG_CNTRL7_ADDR		0x26	/** CNTRL7 address register */

#define REG_ACC_DATA_ADDR	0x28	/** Acc. data low address register */
#define REG_MAG_DATA_ADDR	0x08	/** Mag. data low address register */
#define REG_TEMP_DATA_ADDR	0x05	/** Temp. data low address register */

#define REG_GEN_MAG_ADDR	0x12	/** INT_CTRL_REG_M address register */
#define INT_SRC_REG_M_ADDR	0x13	/** INT_SRC_REG_M address register */
#define REG_GEN_MAG_THR_ADDR	0x14	/** INT_THS_L_M address register */
#define MIG_THRESHOLD_ADDR_H	0x15	/** INT_THS_H_M address register */
#define REG_GEN1_AXIS_ADDR	0x30	/** INT_GEN1_REG address register */
#define INT_GEN1_SRC_ADDR	0x31	/** INT_GEN1_SRC address register */
#define REG_GEN1_THR_ADDR	0x32	/** INT_GEN1_THS address register */
#define REG_GEN1_DUR_ADDR	0x33	/** INT_GEN1_DUR address register */
#define REG_GEN2_AXIS_ADDR	0x34	/** INT_GEN2_REG address register */
#define INT_GEN2_SRC_ADDR	0x35	/** INT_GEN2_SRC address register */
#define REG_GEN2_THR_ADDR	0x36	/** INT_GEN2_THS address register */
#define REG_GEN2_DUR_ADDR	0x37	/** INT_GEN2_DUR address register */

/* Sensitivity [ug/LSB] */
#define SENSITIVITY_ACC_2G	60
#define SENSITIVITY_ACC_4G	120
#define SENSITIVITY_ACC_8G	240
#define SENSITIVITY_ACC_16G	730

/* [ugauss/LSB] */
#define SENSITIVITY_MAG_2G	80
#define SENSITIVITY_MAG_4G	160
#define SENSITIVITY_MAG_8G	320
#define SENSITIVITY_MAG_12G	480

/* ODR */
#define ODR_ACC_MASK		0xF0	/* Mask for odr change on acc */
#define LSM9DS0_ACC_ODR_OFF	0x00  /* Power down */
#define LSM9DS0_ACC_ODR3_125	0x10  /* 3.25Hz output data rate */
#define LSM9DS0_ACC_ODR6_25	0x20  /* 6.25Hz output data rate */
#define LSM9DS0_ACC_ODR12_5	0x30  /* 12.5Hz output data rate */
#define LSM9DS0_ACC_ODR25	0x40  /* 25Hz output data rate */
#define LSM9DS0_ACC_ODR50	0x50  /* 50Hz output data rate */
#define LSM9DS0_ACC_ODR100	0x60  /* 100Hz output data rate */
#define LSM9DS0_ACC_ODR200	0x70  /* 200Hz output data rate */
#define LSM9DS0_ACC_ODR400	0x80  /* 400Hz output data rate */
#define LSM9DS0_ACC_ODR800	0x90  /* 800Hz output data rate */
#define LSM9DS0_ACC_ODR1600	0xA0  /* 1600Hz output data rate */

#define ODR_MAG_MASK		0x1C  /* Mask for odr change on mag */
#define LSM9DS0_MAG_ODR3_125	0x00  /* 3.25Hz output data rate */
#define LSM9DS0_MAG_ODR6_25	0x04  /* 6.25Hz output data rate */
#define LSM9DS0_MAG_ODR12_5	0x08  /* 12.5Hz output data rate */
#define LSM9DS0_MAG_ODR25	0x0C  /* 25Hz output data rate */
#define LSM9DS0_MAG_ODR50	0x10  /* 50Hz output data rate */
#define LSM9DS0_MAG_ODR100	0x14  /* 100Hz output data rate */

/* Magnetic sensor mode */
#define MSMS_MASK		0x03	/* Mask magnetic sensor mode */
#define POWEROFF_MAG		0x02	/* Power Down */
#define CONTINUOS_CONVERSION	0x00	/* Continuos Conversion */

/* Default values loaded in probe function */
#define WHOIAM_VALUE		0x49	/** Who Am I default value */
#define REG_DEF_CNTRL0		0x00	/** CNTRL0 default value */
#define REG_DEF_CNTRL1		0x0F	/** CNTRL1 default value */
#define REG_DEF_CNTRL2		0x00	/** CNTRL2 default value */
#define REG_DEF_CNTRL3		0x00	/** CNTRL3 default value */
#define REG_DEF_CNTRL4		0x00	/** CNTRL4 default value */
#define REG_DEF_CNTRL5		0x18	/** CNTRL5 default value */
#define REG_DEF_CNTRL6		0x20	/** CNTRL6 default value */
#define REG_DEF_CNTRL7		0x02	/** CNTRL7 default value */

#define REG_DEF_INT_CNTRL_MAG	0x00	/** INT_CTRL_REG_M default value */
#define REG_DEF_INT_GEN1	0x00	/** INT_GEN1_REG default value */
#define REG_DEF_INT_GEN2	0x00	/** INT_GEN2_REG default value */
#define REG_DEF_IIG1_DURATION	0x00	/** INT_GEN1_DUR default value */
#define REG_DEF_IIG2_DURATION	0x00	/** INT_GEN2_DUR default value */
#define REG_DEF_IIG1_THRESHOLD	0x00	/** INT_GEN1_THS default value */
#define REG_DEF_IIG2_THRESHOLD	0x00	/** INT_GEN2_THS default value */
#define REG_DEF_MIG_THRESHOLD_L	0x00	/** INT_THS_L_M default value */
#define REG_DEF_MIG_THRESHOLD_H	0x00	/** INT_THS_H_M default value */

#define REG_DEF_ALL_ZEROS	0x00

/* Accelerometer Filter */
#define LSM9DS0_ACC_FILTER_MASK	0xC0	/* Mask for filter band change on acc */
#define FILTER_773		773	/* Anti-Aliasing 773 Hz */
#define FILTER_362		362	/* Anti-Aliasing 362 Hz */
#define FILTER_194		194	/* Anti-Aliasing 194 Hz */
#define FILTER_50		50	/* Anti-Aliasing 50 Hz */

/* Temperature */
#define TEMP_MASK		0x80	/* Mask for temperature change */
#define TEMP_ON			0x80	/* Enable temperature */
#define TEMP_OFF		0x00	/* Disable temperature */
#define TEMP_SENSITIVITY	8	/* Sensitivity temperature */
#define OFFSET_TEMP		25	/* Offset temperature */
#define NDTEMP			1000	/* Not Available temperature */

/* Interrupt */
#define GEN1_PIN1_MASK		0x20
#define GEN1_PIN2_MASK		0x40
#define GEN2_PIN1_MASK		0x10
#define GEN2_PIN2_MASK		0x20
#define GEN_MAG_PIN1_MASK	0x08
#define GEN_MAG_PIN2_MASK	0x10
#define GEN_MAG_EN_MASK		0x01
#define MAX_DUR_TH		127
#define MAX_TH_MAG		131071
#define GEN_X_HIGH_MASK		0x02
#define GEN_X_LOW_MASK		0x01
#define GEN_Y_HIGH_MASK		0x08
#define GEN_Y_LOW_MASK		0x04
#define GEN_Z_HIGH_MASK		0x20
#define GEN_Z_LOW_MASK		0x10
#define GEN_X_MAG_MASK		0x80
#define GEN_Y_MAG_MASK		0x40
#define GEN_Z_MAG_MASK		0x20

#define GEN1_AND_OR_MASK	0x80
#define GEN2_AND_OR_MASK	0x83

#define INT_PIN_CONF_MASK	0x10
#define INT_POLARITY_MASK	0x80

#define to_dev(obj) container_of(obj, struct device, kobj)
#define to_dev_attr(_attr) container_of(_attr, struct device_attribute, attr)

static struct kobject *acc_kobj;
static struct kobject *mag_kobj;

struct {
	unsigned int cutoff_us;
	u8 value;
} lsm9ds0_acc_odr_table[] = {
	{   1, LSM9DS0_ACC_ODR800  },
	{   2, LSM9DS0_ACC_ODR400  },
	{   5, LSM9DS0_ACC_ODR200  },
	{  10, LSM9DS0_ACC_ODR100  },
	{  20, LSM9DS0_ACC_ODR50   },
	{  40, LSM9DS0_ACC_ODR25   },
	{  80, LSM9DS0_ACC_ODR12_5 },
	{ 160, LSM9DS0_ACC_ODR6_25 },
	{ 320, LSM9DS0_ACC_ODR3_125},
};

struct {
	unsigned int cutoff_us;
	u8 value;
} lsm9ds0_mag_odr_table[] = {
	{  10, LSM9DS0_MAG_ODR100  },
	{  20, LSM9DS0_MAG_ODR50   },
	{  40, LSM9DS0_MAG_ODR25   },
	{  80, LSM9DS0_MAG_ODR12_5 },
	{ 160, LSM9DS0_MAG_ODR6_25 },
	{ 320, LSM9DS0_MAG_ODR3_125},
};

struct interrupt_enable {
	atomic_t enable;
	u8 address;
	u8 mask;
};

struct interrupt_value {
	int value;
	u8 address;
};

struct lsm9ds0_interrupt {
	struct interrupt_enable gen1_pin1;
	struct interrupt_enable gen1_pin2;
	struct interrupt_enable gen2_pin1;
	struct interrupt_enable gen2_pin2;
	struct interrupt_value gen1_threshold;
	struct interrupt_value gen2_threshold;
	struct interrupt_value gen1_duration;
	struct interrupt_value gen2_duration;
	struct interrupt_enable gen_mag_pin1;
	struct interrupt_enable gen_mag_pin2;
	struct interrupt_enable gen_mag;
	struct interrupt_value gen_mag_threshold;
	struct interrupt_enable gen1_axis[6];
	struct interrupt_enable gen2_axis[6];
	struct interrupt_enable gen_mag_axis[3];
	struct interrupt_enable gen1_and_or;
	struct interrupt_enable gen2_and_or;
	struct interrupt_enable interrupt_pin_conf;
	struct interrupt_enable interrupt_polarity;
};

static const struct lsm9ds0_acc_platform_data default_lsm9ds0_acc_pdata = {
	.fs_range = LSM9DS0_ACC_FS_2G,
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
	.poll_interval = 100,
	.min_interval = LSM9DS0_ACC_MIN_POLL_PERIOD_MS,
	.aa_filter_bandwidth = ANTI_ALIASING_773,
	.gpio_int1 = DEFAULT_INT1_GPIO,
	.gpio_int2 = DEFAULT_INT2_GPIO,
};

static const struct lsm9ds0_mag_platform_data default_lsm9ds0_mag_pdata = {
	.poll_interval = 100,
	.min_interval = LSM9DS0_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = LSM9DS0_MAG_FS_2G,
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
};

struct reg_rw {
	u8 address;
	u8 default_value;
	u8 resume_value;
};

struct reg_r {
	u8 address;
	u8 value;
};

static struct status_registers {
	struct reg_r who_am_i;
	struct reg_rw cntrl0;
	struct reg_rw cntrl1;
	struct reg_rw cntrl2;
	struct reg_rw cntrl3;
	struct reg_rw cntrl4;
	struct reg_rw cntrl5;
	struct reg_rw cntrl6;
	struct reg_rw cntrl7;
	struct reg_rw int_ctrl_reg_m;
	struct reg_rw int_mag_threshold_low;
	struct reg_rw int_mag_threshold_high;
	struct reg_rw int_gen1_reg;
	struct reg_rw int_gen2_reg;
	struct reg_rw int_gen1_duration;
	struct reg_rw int_gen2_duration;
	struct reg_rw int_gen1_threshold;
	struct reg_rw int_gen2_threshold;
	struct reg_r int_src_reg_m;
	struct reg_r int_gen1_src;
	struct reg_r int_gen2_src;
	struct reg_r int_gen_mag_src;
} status_registers = {
	.who_am_i.address=REG_WHOAMI_ADDR, .who_am_i.value=WHOIAM_VALUE,
	.cntrl0.address=REG_CNTRL0_ADDR, .cntrl0.default_value=REG_DEF_CNTRL0,
	.cntrl1.address=REG_CNTRL1_ADDR, .cntrl1.default_value=REG_DEF_CNTRL1,
	.cntrl2.address=REG_CNTRL2_ADDR, .cntrl2.default_value=REG_DEF_CNTRL2,
	.cntrl3.address=REG_CNTRL3_ADDR, .cntrl3.default_value=REG_DEF_CNTRL3,
	.cntrl4.address=REG_CNTRL4_ADDR, .cntrl4.default_value=REG_DEF_CNTRL4,
	.cntrl5.address=REG_CNTRL5_ADDR, .cntrl5.default_value=REG_DEF_CNTRL5,
	.cntrl6.address=REG_CNTRL6_ADDR, .cntrl6.default_value=REG_DEF_CNTRL6,
	.cntrl7.address=REG_CNTRL7_ADDR, .cntrl7.default_value=REG_DEF_CNTRL7,
	.int_ctrl_reg_m.address=REG_GEN_MAG_ADDR,
		.int_ctrl_reg_m.default_value=REG_DEF_INT_CNTRL_MAG,
	.int_mag_threshold_low.address=REG_GEN_MAG_THR_ADDR,
		.int_mag_threshold_low.default_value=REG_DEF_MIG_THRESHOLD_L,
	.int_mag_threshold_low.address=MIG_THRESHOLD_ADDR_H,
		.int_mag_threshold_low.default_value=REG_DEF_MIG_THRESHOLD_H,
	.int_gen1_reg.address=REG_GEN1_AXIS_ADDR,
		.int_gen1_reg.default_value=REG_DEF_INT_GEN1,
	.int_gen2_reg.address=REG_GEN2_AXIS_ADDR,
		.int_gen2_reg.default_value=REG_DEF_INT_GEN2,
	.int_gen1_duration.address=REG_GEN1_DUR_ADDR,
		.int_gen1_duration.default_value=REG_DEF_IIG1_DURATION,
	.int_gen2_duration.address=REG_GEN2_DUR_ADDR,
		.int_gen2_duration.default_value=REG_DEF_IIG2_DURATION,
	.int_gen1_threshold.address=REG_GEN1_THR_ADDR,
		.int_gen1_threshold.default_value=REG_DEF_IIG1_THRESHOLD,
	.int_gen2_threshold.address=REG_GEN2_THR_ADDR,
		.int_gen2_threshold.default_value=REG_DEF_IIG2_THRESHOLD,
	.int_src_reg_m.address = INT_SRC_REG_M_ADDR,
				.int_src_reg_m.value = REG_DEF_ALL_ZEROS,
	.int_gen1_src.address = INT_GEN1_SRC_ADDR,
				.int_gen1_src.value = REG_DEF_ALL_ZEROS,
	.int_gen2_src.address = INT_GEN2_SRC_ADDR,
				.int_gen2_src.value = REG_DEF_ALL_ZEROS,
	.int_gen_mag_src.address = INT_SRC_REG_M_ADDR,
				.int_gen_mag_src.value = REG_DEF_ALL_ZEROS,
};

static int lsm9ds0_check_whoami(struct lsm9ds0_dev *dev)
{
	int err;
	u8 data;

	err = dev->tf->read(dev->dev, status_registers.who_am_i.address, 1,
			    &data);
	if (err < 0) {
		dev_warn(dev->dev, "error reading who_am_i\n");
		return err;
	} else
		dev->hw_working = 1;

	if (data != status_registers.who_am_i.value) {
		dev_err(dev->dev,
			"device unknown. expected: 0x%02x-0x%02x\n",
			status_registers.who_am_i.value, data);
		return -1;
	}

	return 0;
}

static int lsm9ds0_hw_init(struct lsm9ds0_dev *dev)
{
	int i, err = -1;

#ifdef LSM9DS0_DEBUG
	pr_info("%s: hw init start\n", LSM9DS0_DEV_NAME);
#endif

	status_registers.cntrl1.resume_value =
					status_registers.cntrl1.default_value;
	status_registers.cntrl2.resume_value =
					status_registers.cntrl2.default_value;
	status_registers.cntrl3.resume_value =
					status_registers.cntrl3.default_value;
	status_registers.cntrl4.resume_value =
					status_registers.cntrl4.default_value;
	status_registers.cntrl5.resume_value =
					status_registers.cntrl5.default_value;
	status_registers.cntrl6.resume_value =
					status_registers.cntrl6.default_value;
	status_registers.cntrl7.resume_value =
					status_registers.cntrl7.default_value;

	status_registers.int_ctrl_reg_m.resume_value =
			status_registers.int_ctrl_reg_m.default_value;
	status_registers.int_mag_threshold_low.resume_value =
			status_registers.int_mag_threshold_low.default_value;
	status_registers.int_mag_threshold_high.resume_value =
			status_registers.int_mag_threshold_high.default_value;
	status_registers.int_gen1_reg.resume_value =
			status_registers.int_gen1_reg.default_value;
	status_registers.int_gen2_reg.resume_value =
			status_registers.int_gen2_reg.default_value;
	status_registers.int_gen1_duration.resume_value =
			status_registers.int_gen1_duration.default_value;
	status_registers.int_gen2_duration.resume_value =
			status_registers.int_gen2_duration.default_value;
	status_registers.int_gen1_threshold.resume_value =
			status_registers.int_gen1_threshold.default_value;
	status_registers.int_gen2_threshold.resume_value =
			status_registers.int_gen2_threshold.default_value;


	dev->temp_value_dec = NDTEMP;

	if ((dev->pdata_acc->gpio_int1 >= 0) ||
	    (dev->pdata_acc->gpio_int2 >= 0)) {

		dev->interrupt = kmalloc(sizeof(*dev->interrupt), GFP_KERNEL);
		if (dev->interrupt == NULL)
			goto error_interrupt;

		dev->interrupt->gen1_pin1.address = REG_CNTRL3_ADDR;
		dev->interrupt->gen1_pin2.address = REG_CNTRL4_ADDR;
		dev->interrupt->gen2_pin1.address = REG_CNTRL3_ADDR;
		dev->interrupt->gen2_pin2.address = REG_CNTRL4_ADDR;
		dev->interrupt->gen_mag_pin1.address = REG_CNTRL3_ADDR;
		dev->interrupt->gen_mag_pin2.address = REG_CNTRL4_ADDR;
		dev->interrupt->gen_mag.address = REG_GEN_MAG_ADDR;
		dev->interrupt->gen1_duration.address = REG_GEN1_DUR_ADDR;
		dev->interrupt->gen2_duration.address = REG_GEN2_DUR_ADDR;
		dev->interrupt->gen1_threshold.address = REG_GEN1_THR_ADDR;
		dev->interrupt->gen2_threshold.address = REG_GEN2_THR_ADDR;
		dev->interrupt->gen_mag_threshold.address =
							REG_GEN_MAG_THR_ADDR;

		dev->interrupt->gen1_pin1.mask = GEN1_PIN1_MASK;
		dev->interrupt->gen1_pin2.mask = GEN1_PIN2_MASK;
		dev->interrupt->gen2_pin1.mask = GEN2_PIN1_MASK;
		dev->interrupt->gen2_pin2.mask = GEN2_PIN2_MASK;
		dev->interrupt->gen_mag_pin1.mask = GEN_MAG_PIN1_MASK;
		dev->interrupt->gen_mag_pin2.mask = GEN_MAG_PIN2_MASK;
		dev->interrupt->gen_mag.mask = GEN_MAG_EN_MASK;

		atomic_set(&dev->interrupt->gen1_pin1.enable, 0);
		atomic_set(&dev->interrupt->gen1_pin2.enable, 0);
		atomic_set(&dev->interrupt->gen2_pin1.enable, 0);
		atomic_set(&dev->interrupt->gen2_pin2.enable, 0);
		atomic_set(&dev->interrupt->gen_mag_pin1.enable, 0);
		atomic_set(&dev->interrupt->gen_mag_pin2.enable, 0);
		atomic_set(&dev->interrupt->gen_mag.enable, 0);

		dev->interrupt->gen1_threshold.value = 0;
		dev->interrupt->gen2_threshold.value = 0;
		dev->interrupt->gen1_duration.value = 0;
		dev->interrupt->gen2_duration.value = 0;
		dev->interrupt->gen_mag_threshold.value = 0;

		for (i = 0; i < 6; i++) {
			dev->interrupt->gen1_axis[i].address =
						REG_GEN1_AXIS_ADDR;
			dev->interrupt->gen2_axis[i].address =
						REG_GEN2_AXIS_ADDR;

			atomic_set(&dev->interrupt->gen1_axis[i].enable, 0);
			atomic_set(&dev->interrupt->gen2_axis[i].enable, 0);
		}

		for (i = 0; i < 3; i++) {
			dev->interrupt->gen_mag_axis[i].address =
						REG_GEN_MAG_ADDR;
			atomic_set(&dev->interrupt->gen_mag_axis[i].enable, 0);
		}

		dev->interrupt->gen1_axis[0].mask = GEN_X_LOW_MASK;
		dev->interrupt->gen1_axis[1].mask = GEN_Y_LOW_MASK;
		dev->interrupt->gen1_axis[2].mask = GEN_Z_LOW_MASK;
		dev->interrupt->gen1_axis[3].mask = GEN_X_HIGH_MASK;
		dev->interrupt->gen1_axis[4].mask = GEN_Y_HIGH_MASK;
		dev->interrupt->gen1_axis[5].mask = GEN_Z_HIGH_MASK;

		dev->interrupt->gen2_axis[0].mask = GEN_X_LOW_MASK;
		dev->interrupt->gen2_axis[1].mask = GEN_Y_LOW_MASK;
		dev->interrupt->gen2_axis[2].mask = GEN_Z_LOW_MASK;
		dev->interrupt->gen2_axis[3].mask = GEN_X_HIGH_MASK;
		dev->interrupt->gen2_axis[4].mask = GEN_Y_HIGH_MASK;
		dev->interrupt->gen2_axis[5].mask = GEN_Z_HIGH_MASK;

		dev->interrupt->gen_mag_axis[0].mask = GEN_X_MAG_MASK;
		dev->interrupt->gen_mag_axis[1].mask = GEN_Y_MAG_MASK;
		dev->interrupt->gen_mag_axis[2].mask = GEN_Z_MAG_MASK;

		dev->interrupt->gen1_and_or.address = REG_GEN1_AXIS_ADDR;
		dev->interrupt->gen1_and_or.mask = GEN1_AND_OR_MASK;
		atomic_set(&dev->interrupt->gen1_and_or.enable, 0);
		dev->interrupt->gen2_and_or.address = REG_GEN1_DUR_ADDR;
		dev->interrupt->gen2_and_or.mask = GEN2_AND_OR_MASK;
		atomic_set(&dev->interrupt->gen2_and_or.enable, 0);

		dev->interrupt->interrupt_pin_conf.address = REG_GEN_MAG_ADDR;
		dev->interrupt->interrupt_pin_conf.mask = INT_PIN_CONF_MASK;
		atomic_set(&dev->interrupt->interrupt_pin_conf.enable, 0);

		dev->interrupt->interrupt_polarity.address = REG_GEN_MAG_ADDR;
		dev->interrupt->interrupt_polarity.mask = INT_POLARITY_MASK;
		atomic_set(&dev->interrupt->interrupt_polarity.enable, 0);
	}

	dev->hw_initialized = 1;

#ifdef LSM9DS0_DEBUG
	pr_info("%s: hw init done\n", LSM9DS0_DEV_NAME);
#endif

	return 0;

error_interrupt:
	dev->hw_working = 0;
	dev->hw_initialized = 0;
	return err;
}

static irqreturn_t lsm9ds0_isr1(int irq, void *data)
{
	struct lsm9ds0_dev *dev = (struct lsm9ds0_dev *)data;

	disable_irq_nosync(irq);
	queue_work(dev->irq1_work_queue, &dev->irq1_work);
	pr_debug("%s: isr1 queued\n", LSM9DS0_DEV_NAME);
	return IRQ_HANDLED;
}

static irqreturn_t lsm9ds0_isr2(int irq, void *data)
{
	struct lsm9ds0_dev *dev = (struct lsm9ds0_dev *)data;

	disable_irq_nosync(irq);
	queue_work(dev->irq2_work_queue, &dev->irq2_work);
	pr_debug("%s: isr2 queued\n", LSM9DS0_DEV_NAME);
	return IRQ_HANDLED;
}

static void lsm9ds0_interrupt_catch(struct lsm9ds0_dev *dev, int pin) 
{
	u8 buf[1];
	u8 val;

	mutex_lock(&dev->lock);
	if (atomic_read(&dev->interrupt->gen1_pin1.enable) == 1) {
		val = dev->tf->read(dev->dev,
				    status_registers.int_gen1_src.address, 1,
				    buf);
		if (val < 0)
			return;
		status_registers.int_gen1_src.value = buf[0];

		if (((int)status_registers.int_gen1_src.value) > 64)
			pr_info("interrupt send by acc interrupt generator 1\n");
	}

	if (atomic_read(&dev->interrupt->gen_mag_pin1.enable) == 1) {
		val = dev->tf->read(dev->dev,
				    status_registers.int_gen_mag_src.address,
				    1, buf);
		if(val < 0)
			return;
		status_registers.int_gen_mag_src.value = buf[0];

		if (((int)status_registers.int_gen_mag_src.value) > 1)
			pr_info("interrupt send by magn interrupt generator\n");
	}
	mutex_unlock(&dev->lock);
}

static void lsm9ds0_irq1_work_func(struct work_struct *work)
{

	struct lsm9ds0_dev *dev;

	dev = container_of(work, struct lsm9ds0_dev, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm9ds0_get_int1_source(dev); */

	lsm9ds0_interrupt_catch(dev, 1);
	pr_info("%s: IRQ1 triggered\n", LSM9DS0_DEV_NAME);

	enable_irq(dev->irq1);
}

static void lsm9ds0_irq2_work_func(struct work_struct *work)
{

	struct lsm9ds0_dev *dev;

	dev = container_of(work, struct lsm9ds0_dev, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm9ds0_get_int2_source(dev); */

	lsm9ds0_interrupt_catch(dev, 2);
	pr_info("%s: IRQ2 triggered\n", LSM9DS0_DEV_NAME);

	enable_irq(dev->irq2);
}

static int lsm9ds0_acc_device_power_off(struct lsm9ds0_dev *dev)
{
	int err;
	u8 buf[1];

	buf[0] = ((ODR_ACC_MASK & LSM9DS0_ACC_ODR_OFF) | 
		  (~ODR_ACC_MASK & status_registers.cntrl1.resume_value));

	err = dev->tf->write(dev->dev, status_registers.cntrl1.address, 1,
			     buf);
	if (err < 0)
		dev_err(dev->dev, "accelerometer soft power off failed: %d\n",
			err);

	if (dev->pdata_acc->power_off) {
		dev->pdata_acc->power_off();
	}

	atomic_set(&dev->enabled_acc, 0);

	return 0;
}

static int lsm9ds0_mag_device_power_off(struct lsm9ds0_dev *dev)
{
	int err;
	u8 buf[1];

	buf[0] = ((MSMS_MASK & POWEROFF_MAG) |
		  (~MSMS_MASK & status_registers.cntrl7.resume_value));

	err = dev->tf->write(dev->dev, status_registers.cntrl7.address, 1,
			     buf);
	if (err < 0)
		dev_err(dev->dev, "magn soft power off failed: %d\n", err);

	if (dev->pdata_mag->power_off) {
		dev->pdata_mag->power_off();
	}

	atomic_set(&dev->enabled_mag, 0);

	return 0;
}

static int lsm9ds0_acc_device_power_on(struct lsm9ds0_dev *dev)
{
	int err = -1;
	u8 buf[4];

	if (dev->pdata_acc->power_on) {
		err = dev->pdata_acc->power_on();
		if (err < 0) {
			dev_err(dev->dev,
				"accelerometer power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.cntrl0.resume_value;
	err = dev->tf->write(dev->dev, status_registers.cntrl0.address, 1,
			     buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.cntrl1.resume_value;
	buf[1] = status_registers.cntrl2.resume_value;
	buf[2] = status_registers.cntrl3.resume_value;
	buf[3] = status_registers.cntrl4.resume_value;
	err = dev->tf->write(dev->dev, status_registers.cntrl1.address, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen1_reg.resume_value;
	err = dev->tf->write(dev->dev, status_registers.int_gen1_reg.address, 1,
			 buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen1_threshold.resume_value;
	buf[1] = status_registers.int_gen1_duration.resume_value;
	err = dev->tf->write(dev->dev, status_registers.int_gen1_threshold.address,
			 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen2_reg.resume_value;
	err = dev->tf->write(dev->dev, status_registers.int_gen2_reg.address, 1,
			 buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen2_threshold.resume_value;
	buf[1] = status_registers.int_gen2_duration.resume_value;
	err = dev->tf->write(dev->dev, status_registers.int_gen2_threshold.address,
			 2, buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&dev->enabled_acc, 1);

	return 0;

err_resume_state:
	atomic_set(&dev->enabled_acc, 0);
	dev_err(dev->dev, "accelerometer hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int lsm9ds0_mag_device_power_on(struct lsm9ds0_dev *dev)
{
	int err = -1;
	u8 buf[5];

	if (dev->pdata_mag->power_on) {
		err = dev->pdata_mag->power_on();
		if (err < 0) {
			dev_err(dev->dev,
				"magnetometer power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.cntrl0.resume_value;
	err = dev->tf->write(dev->dev, status_registers.cntrl0.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.cntrl3.resume_value;
	buf[1] = status_registers.cntrl4.resume_value;
	buf[2] = status_registers.cntrl5.resume_value;
	buf[3] = status_registers.cntrl6.resume_value;
	err = dev->tf->write(dev->dev, status_registers.cntrl3.address, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_ctrl_reg_m.resume_value;
	err = dev->tf->write(dev->dev, status_registers.int_ctrl_reg_m.address,
			 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_mag_threshold_low.resume_value;
	buf[1] = status_registers.int_mag_threshold_high.resume_value;
	err = dev->tf->write(dev->dev,
			 status_registers.int_mag_threshold_low.address,
			 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = ((MSMS_MASK & CONTINUOS_CONVERSION) |
		  (~MSMS_MASK & status_registers.cntrl7.resume_value));
	err = dev->tf->write(dev->dev, status_registers.cntrl7.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&dev->enabled_mag, 1);

	return 0;

err_resume_state:
	atomic_set(&dev->enabled_mag, 0);
	dev_err(dev->dev, "magnetometer hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int lsm9ds0_acc_update_filter(struct lsm9ds0_dev *dev,
				     u8 new_bandwidth)
{
	int err;
	u8 updated_val, buf[1];

	switch (new_bandwidth) {
	case ANTI_ALIASING_50:
		break;
	case ANTI_ALIASING_194:
		break;
	case ANTI_ALIASING_362:
		break;
	case ANTI_ALIASING_773:
		break;
	default:
		dev_err(dev->dev, "invalid accelerometer "
			"update bandwidth requested: %u\n", new_bandwidth);
		return -EINVAL;
	}

	err = dev->tf->read(dev->dev, status_registers.cntrl2.address, 1,
			    buf);
	if (err < 0)
		goto error;

	status_registers.cntrl2.resume_value = buf[0];
	updated_val = ((LSM9DS0_ACC_FILTER_MASK & new_bandwidth) |
		       (~LSM9DS0_ACC_FILTER_MASK & buf[0]));
	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.cntrl2.address, 1, buf);
	if (err < 0)
		goto error;
	status_registers.cntrl2.resume_value = updated_val;

	return err;

error:
	dev_err(dev->dev, "update accelerometer fs range failed "
		"0x%02x: %d\n", buf[0], err);
	return err;
}

static int lsm9ds0_acc_update_fs_range(struct lsm9ds0_dev *dev,
				       u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;
	u8 updated_val;
	u8 buf[1];

	switch (new_fs_range) {
	case LSM9DS0_ACC_FS_2G:
		sensitivity = SENSITIVITY_ACC_2G;
		break;
	case LSM9DS0_ACC_FS_4G:
		sensitivity = SENSITIVITY_ACC_4G;
		break;
	case LSM9DS0_ACC_FS_8G:
		sensitivity = SENSITIVITY_ACC_8G;
		break;
	case LSM9DS0_ACC_FS_16G:
		sensitivity = SENSITIVITY_ACC_16G;
		break;
	default:
		dev_err(dev->dev, "invalid accelerometer "
				"fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	err = dev->tf->read(dev->dev, status_registers.cntrl2.address, 1,
			    buf);
	if (err < 0)
		goto error;

	status_registers.cntrl2.resume_value = buf[0];
	updated_val = ((LSM9DS0_ACC_FS_MASK & new_fs_range) |
		       (~LSM9DS0_ACC_FS_MASK & buf[0]));
	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.cntrl2.address, 1, buf);
	if (err < 0)
		goto error;
	status_registers.cntrl2.resume_value = updated_val;
	dev->sensitivity_acc = sensitivity;

	return err;

error:
	dev_err(dev->dev, "update accelerometer fs range failed "
		"0x%02x %d\n", buf[0], err);
	return err;
}

static int lsm9ds0_mag_update_fs_range(struct lsm9ds0_dev *dev,
				       u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;
	u8 buf[1];

	switch (new_fs_range) {
	case LSM9DS0_MAG_FS_2G:
		sensitivity = SENSITIVITY_MAG_2G;
		break;
	case LSM9DS0_MAG_FS_4G:
		sensitivity = SENSITIVITY_MAG_4G;
		break;
	case LSM9DS0_MAG_FS_8G:
		sensitivity = SENSITIVITY_MAG_8G;
		break;
	case LSM9DS0_MAG_FS_12G:
		sensitivity = SENSITIVITY_MAG_12G;
		break;
	default:
		dev_err(dev->dev, "invalid magnetometer "
				"fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	err = dev->tf->read(dev->dev, status_registers.cntrl6.address, 1,
			    buf);
	if (err < 0)
		goto error;

	status_registers.cntrl6.resume_value = buf[0];
	buf[0] = (LSM9DS0_MAG_FS_MASK & new_fs_range);
	err = dev->tf->write(dev->dev, status_registers.cntrl6.address, 1, buf);
	if (err < 0)
		goto error;
	status_registers.cntrl6.resume_value = buf[0];
	dev->sensitivity_mag = sensitivity;

	return err;

error:
	dev_err(dev->dev, "update magnetometer fs range failed "
		"0x%02x: %d\n", buf[0], err);
	return err;
}

static int lsm9ds0_acc_update_odr(struct lsm9ds0_dev *dev,
				  unsigned int poll_ms)
{
	int i, err = -1;
	u8 config[1];

	for (i = ARRAY_SIZE(lsm9ds0_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm9ds0_acc_odr_table[i].cutoff_us <= poll_ms) ||
		    (i == 0))
			break;
	}

	config[0] = ((ODR_ACC_MASK & lsm9ds0_acc_odr_table[i].value) |
		     (~ODR_ACC_MASK & status_registers.cntrl1.resume_value));

	if (atomic_read(&dev->enabled_acc)) {
		err = dev->tf->write(dev->dev, status_registers.cntrl1.address, 1,
				 config);
		if (err < 0)
			goto error;
	}

	status_registers.cntrl1.resume_value = config[0];
	dev->ktime_acc = ktime_set(SEC_PORTION_FROM_MS(poll_ms),
				   NSEC_PORTION_FROM_MS(poll_ms));

	return err;

error:
	dev_err(dev->dev, "update accelerometer odr failed "
		"0x%02x: %d\n", config[0], err);

	return err;
}

static int lsm9ds0_mag_update_odr(struct lsm9ds0_dev *dev,
				  unsigned int poll_ms)
{
	int i, err = -1;
	u8 config[0];

	for (i = ARRAY_SIZE(lsm9ds0_mag_odr_table) - 1; i >= 0; i--) {
		if ((lsm9ds0_mag_odr_table[i].cutoff_us <= poll_ms) ||
		    (i == 0))
			break;
	}

	config[0] = ((ODR_MAG_MASK & lsm9ds0_mag_odr_table[i].value) |
		     (~ODR_MAG_MASK & status_registers.cntrl5.resume_value));

	if (atomic_read(&dev->enabled_mag)) {
		err = dev->tf->write(dev->dev, status_registers.cntrl5.address,
				     1, config);
		if (err < 0)
			goto error;
	}

	status_registers.cntrl5.resume_value = config[0];
	dev->ktime_mag = ktime_set(SEC_PORTION_FROM_MS(poll_ms),
				   NSEC_PORTION_FROM_MS(poll_ms));

	return err;

error:
	dev_err(dev->dev, "update magnetometer odr failed "
			"0x%02x,0x%02x: %d\n", config[0], config[1], err);

	return err;
}

static void lsm9ds0_validate_polling(unsigned int *min_interval,
				     unsigned int *poll_interval,
				     unsigned int min)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);
}

static int lsm9ds0_acc_validate_pdata(struct lsm9ds0_dev *dev)
{
	int res = -EINVAL;

	lsm9ds0_validate_polling(&dev->pdata_acc->min_interval,
				&dev->pdata_acc->poll_interval,
				(unsigned int)LSM9DS0_ACC_MIN_POLL_PERIOD_MS);

	switch (dev->pdata_acc->aa_filter_bandwidth) {
		case ANTI_ALIASING_50:
			res = 1;
			break;
		case ANTI_ALIASING_194:
			res = 1;
			break;
		case ANTI_ALIASING_362:
			res = 1;
			break;
		case ANTI_ALIASING_773:
			res = 1;
			break;
		default:
			dev_err(dev->dev, "invalid accelerometer "
				"bandwidth selected: %u\n",
					dev->pdata_acc->aa_filter_bandwidth);
	}

	return res;
}

static int lsm9ds0_mag_validate_pdata(struct lsm9ds0_dev *dev)
{
	lsm9ds0_validate_polling(&dev->pdata_mag->min_interval,
				&dev->pdata_mag->poll_interval,
				(unsigned int)LSM9DS0_MAG_MIN_POLL_PERIOD_MS);

	return 0;
}

static int lsm9ds0_acc_enable(struct lsm9ds0_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled_acc, 0, 1)) {
		int err;

		mutex_lock(&dev->lock);
		err = lsm9ds0_acc_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled_acc, 0);
			mutex_unlock(&dev->lock);
			return err;
		}
		hrtimer_start(&dev->hr_timer_acc, dev->ktime_acc,
			      HRTIMER_MODE_REL);
		if (!atomic_read(&dev->enabled_mag)) {
			if (dev->pdata_acc->gpio_int1 >= 0)
				enable_irq(dev->irq1);
			if (dev->pdata_acc->gpio_int2 >= 0)
				enable_irq(dev->irq2);
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

static int lsm9ds0_acc_disable(struct lsm9ds0_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_acc, 1, 0)) {
		cancel_work_sync(&dev->input_work_acc);

		mutex_lock(&dev->lock);
		hrtimer_cancel(&dev->hr_timer_acc);
		lsm9ds0_acc_device_power_off(dev);

		if (!atomic_read(&dev->enabled_mag)) {
			if (dev->pdata_acc->gpio_int1 >= 0)
				disable_irq_nosync(dev->irq1);
			if (dev->pdata_acc->gpio_int2 >= 0)
				disable_irq_nosync(dev->irq2);
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

static int lsm9ds0_mag_enable(struct lsm9ds0_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled_mag, 0, 1)) {
		int err;

		mutex_lock(&dev->lock);
		err = lsm9ds0_mag_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled_mag, 0);
			mutex_unlock(&dev->lock);
			return err;
		}
		if (!atomic_read(&dev->enabled_temp)) {
			hrtimer_start(&dev->hr_timer_mag, dev->ktime_mag,
				      HRTIMER_MODE_REL);
		}
		if (!atomic_read(&dev->enabled_acc)) {
			if (dev->pdata_acc->gpio_int1 >= 0)
				enable_irq(dev->irq1);
			if (dev->pdata_acc->gpio_int2 >= 0)
				enable_irq(dev->irq2);
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

static int lsm9ds0_mag_disable(struct lsm9ds0_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_mag, 1, 0)) {
		if (!atomic_read(&dev->enabled_temp)) {
			cancel_work_sync(&dev->input_work_mag);
			hrtimer_cancel(&dev->hr_timer_mag);
		}

		mutex_lock(&dev->lock);
		lsm9ds0_mag_device_power_off(dev);
		if (!atomic_read(&dev->enabled_acc)) {
			if (dev->pdata_acc->gpio_int1 >= 0)
				disable_irq(dev->irq1);
			if (dev->pdata_acc->gpio_int2 >= 0)
				disable_irq(dev->irq2);
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

static int lsm9ds0_temperature_enable(struct lsm9ds0_dev *dev)
{
	int err;
	u8 buf[1];
	u8 updated_val;

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, status_registers.cntrl5.address, 1,
			    buf);
	if (err < 0)
		goto out;

	status_registers.cntrl5.resume_value = buf[0];
	updated_val = ((TEMP_MASK & TEMP_ON) | (~TEMP_MASK & buf[0]));
	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.cntrl5.address, 1,
			     buf);
	if (err < 0)
		goto out;
	status_registers.cntrl5.resume_value = updated_val;

	if (!atomic_read(&dev->enabled_mag))
		hrtimer_start(&dev->hr_timer_mag, dev->ktime_mag,
			      HRTIMER_MODE_REL);

	atomic_set(&dev->enabled_temp, 1);

out:
	mutex_unlock(&dev->lock);

	return err;
}

static int lsm9ds0_temperature_disable(struct lsm9ds0_dev *dev)
{
	int err;
	u8 buf[1];
	u8 updated_val;

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, status_registers.cntrl5.address, 1,
			    buf);
	if (err < 0)
		goto err;

	status_registers.cntrl5.resume_value = buf[0];	
	updated_val = ((TEMP_MASK & TEMP_OFF) | (~TEMP_MASK & buf[0]));
	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.cntrl5.address, 1,
			     buf);
	if (err < 0)
		goto err;
	status_registers.cntrl5.resume_value = updated_val;
	mutex_unlock(&dev->lock);

	if (!atomic_read(&dev->enabled_mag)) {
		cancel_work_sync(&dev->input_work_mag);
		hrtimer_cancel(&dev->hr_timer_mag);
	}
	atomic_set(&dev->enabled_temp, 0);
	dev->temp_value_dec = NDTEMP;

	return 0;

err:
	mutex_unlock(&dev->lock);

	return err;
}

int lsm9ds0_enable(struct lsm9ds0_dev *dev)
{
	int err;

	err = lsm9ds0_acc_enable(dev);
	if (err < 0)
		return err;

	err = lsm9ds0_mag_enable(dev);
	if (err < 0)
		return err;

	return lsm9ds0_temperature_enable(dev);
}
EXPORT_SYMBOL(lsm9ds0_enable);

int lsm9ds0_disable(struct lsm9ds0_dev *dev)
{
	int err;

	err = lsm9ds0_acc_disable(dev);
	if (err < 0)
		return err;

	err = lsm9ds0_mag_disable(dev);
	if (err < 0)
		return err;

	return lsm9ds0_temperature_disable(dev);

}
EXPORT_SYMBOL(lsm9ds0_disable);

static void lsm9ds0_acc_input_cleanup(struct lsm9ds0_dev *dev)
{
	input_unregister_device(dev->input_dev_acc);
	input_free_device(dev->input_dev_acc);
}

static void lsm9ds0_mag_input_cleanup(struct lsm9ds0_dev *dev)
{
	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static ssize_t attr_get_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	unsigned int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	mutex_lock(&dev->lock);
	val = dev->pdata_acc->poll_interval;
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_get_polling_rate_mag(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	unsigned int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	mutex_lock(&dev->lock);
	val = dev->pdata_mag->poll_interval;
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate_acc(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;
	interval_ms = (unsigned int)max((unsigned int)interval_ms,
						dev->pdata_acc->min_interval);
	mutex_lock(&dev->lock);
	dev->pdata_acc->poll_interval = (unsigned int)interval_ms;
	lsm9ds0_acc_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_set_polling_rate_mag(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms) || !interval_ms)
		return -EINVAL;
	interval_ms = (unsigned int)max((unsigned int)interval_ms,
					dev->pdata_mag->min_interval);
	mutex_lock(&dev->lock);
	dev->pdata_mag->poll_interval = (unsigned int)interval_ms;
	lsm9ds0_mag_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_enable_acc(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", (int)atomic_read(&dev->enabled_acc));
}

static ssize_t attr_get_enable_mag(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", (int)atomic_read(&dev->enabled_mag));
}

static ssize_t attr_set_enable_acc(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm9ds0_acc_enable(dev);
	else
		lsm9ds0_acc_disable(dev);

	return size;
}

static ssize_t attr_set_enable_mag(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm9ds0_mag_enable(dev);
	else
		lsm9ds0_mag_disable(dev);

	return size;
}

static ssize_t attr_get_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	u8 val;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	int range = 2;
	mutex_lock(&dev->lock);
	val = dev->pdata_acc->fs_range ;
	switch (val) {
	case LSM9DS0_ACC_FS_2G:
		range = 2;
		break;
	case LSM9DS0_ACC_FS_4G:
		range = 4;
		break;
	case LSM9DS0_ACC_FS_8G:
		range = 8;
		break;
	case LSM9DS0_ACC_FS_16G:
		range = 16;
		break;
	}
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_get_range_mag(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	u8 val;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	int range = 2;
	mutex_lock(&dev->lock);
	val = dev->pdata_mag->fs_range ;
	switch (val) {
	case LSM9DS0_MAG_FS_2G:
		range = 2;
		break;
	case LSM9DS0_MAG_FS_4G:
		range = 4;
		break;
	case LSM9DS0_MAG_FS_8G:
		range = 8;
		break;
	case LSM9DS0_MAG_FS_12G:
		range = 12;
		break;
	}
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LSM9DS0_ACC_FS_2G;
		break;
	case 4:
		range = LSM9DS0_ACC_FS_4G;
		break;
	case 8:
		range = LSM9DS0_ACC_FS_8G;
		break;
	case 16:
		range = LSM9DS0_ACC_FS_16G;
		break;
	default:
		dev_err(dev->dev, "accelerometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&dev->lock);
	err = lsm9ds0_acc_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata_acc->fs_range = range;
	mutex_unlock(&dev->lock);
	dev_info(dev->dev, "accelerometer range set to:"
							" %lu g\n", val);

	return size;
}

static ssize_t attr_set_range_mag(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LSM9DS0_MAG_FS_2G;
		break;
	case 4:
		range = LSM9DS0_MAG_FS_4G;
		break;
	case 8:
		range = LSM9DS0_MAG_FS_8G;
		break;
	case 12:
		range = LSM9DS0_MAG_FS_12G;
		break;
	default:
		dev_err(dev->dev, "magnetometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&dev->lock);
	err = lsm9ds0_mag_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata_mag->fs_range = range;
	mutex_unlock(&dev->lock);
	dev_info(dev->dev, "magnetometer range set to:"
							" %lu g\n", val);

	return size;
}

static ssize_t attr_get_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	u8 val;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	int frequency=FILTER_773;
	mutex_lock(&dev->lock);
	val = dev->pdata_acc->aa_filter_bandwidth;
	switch (val) {
	case ANTI_ALIASING_50:
		frequency = FILTER_50;
		break;
	case ANTI_ALIASING_194:
		frequency = FILTER_194;
		break;
	case ANTI_ALIASING_362:
		frequency = FILTER_362;
		break;
	case ANTI_ALIASING_773:
		frequency = FILTER_773;
		break;
	}
	mutex_unlock(&dev->lock);
	return sprintf(buf, "%d\n", frequency);
}

static ssize_t attr_set_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 frequency;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case FILTER_50:
		frequency = ANTI_ALIASING_50;
		break;
	case FILTER_194:
		frequency = ANTI_ALIASING_194;
		break;
	case FILTER_362:
		frequency = ANTI_ALIASING_362;
		break;
	case FILTER_773:
		frequency = ANTI_ALIASING_773;
		break;
	default:
		dev_err(dev->dev, "accelerometer invalid filter "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&dev->lock);
	err = lsm9ds0_acc_update_filter(dev, frequency);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata_acc->aa_filter_bandwidth = frequency;
	mutex_unlock(&dev->lock);
	dev_info(dev->dev, "accelerometer anti-aliasing filter "
					"set to: %lu Hz\n", val);

	return size;
}

static ssize_t attr_get_temp_enable(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", (int)atomic_read(&dev->enabled_temp));
}

static ssize_t attr_set_temp_enable(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	unsigned long val;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	atomic_set(&dev->enabled_temp, (int)val);

	if (val > 0)
		lsm9ds0_temperature_enable(dev);
	else
		lsm9ds0_temperature_disable(dev);

	return size;
}

static ssize_t attr_get_temp(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	int dec;
	unsigned int flo;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	dec = dev->temp_value_dec;
	flo = dev->temp_value_flo;

	if(dec==NDTEMP)
		return sprintf(buf, "n.d.\n");

	return sprintf(buf, "%d.%u\n", dec, flo);
}

static struct kobj_attribute poll_attr_acc =
__ATTR(pollrate_ms, 0664, attr_get_polling_rate_acc, attr_set_polling_rate_acc);
static struct kobj_attribute enable_attr_acc =
__ATTR(enable_device, 0664, attr_get_enable_acc, attr_set_enable_acc);
static struct kobj_attribute fs_attr_acc =
__ATTR(full_scale, 0664, attr_get_range_acc, attr_set_range_acc);
static struct kobj_attribute aa_filter_attr  =
__ATTR(anti_aliasing_frequency, 0664, attr_get_aa_filter, attr_set_aa_filter);
static struct kobj_attribute poll_attr_mag =
__ATTR(pollrate_ms, 0664, attr_get_polling_rate_mag, attr_set_polling_rate_mag);
static struct kobj_attribute enable_attr_mag =
__ATTR(enable_device, 0664, attr_get_enable_mag, attr_set_enable_mag);
static struct kobj_attribute fs_attr_mag =
__ATTR(full_scale, 0664, attr_get_range_mag, attr_set_range_mag);

static int write_bit_on_register(struct lsm9ds0_dev *dev, u8 address,
				 u8 *resume_value, u8 mask, int value)
{
	int err;
	u8 updated_val, buf[1], val = 0;

	mutex_lock(&dev->lock);
	err = dev->tf->read(dev->dev, address, 1, buf);
	if (err < 0) {
		err = -1;
		goto out;
	}

	if (resume_value != NULL)
		*resume_value = buf[0];

	if (mask == 0) {
		updated_val = (u8)value;
	} else {
		if (value > 0)
			val = 0xFF;
		updated_val = (mask & val) | ((~mask) & buf[0]);
	}

	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, address, 1, buf);
	if (err < 0) {
		err = -1;
		goto out;
	}

	if (resume_value != NULL)
		*resume_value = updated_val;

out:
	mutex_unlock(&dev->lock);
	return err;
}

static int write_gen_int(struct lsm9ds0_dev *dev, struct interrupt_enable *ie,
			 int val)
{
	int err;

	if (val > 0)
		val = 1;
	else
		val = 0;

	err = write_bit_on_register(dev, ie->address, NULL, ie->mask, val);
	if(err < 0)
		return -1;

	atomic_set(&ie->enable, val);

	return err;
}

static int write_duration_threshold_int(struct lsm9ds0_dev *dev,
					struct interrupt_value *ie, int val)
{
	int err;

	if (val < 0)
		return -1;

	if (val > MAX_DUR_TH)
		return -1;

	err = write_bit_on_register(dev, ie->address, NULL, 0, val);
	if (err < 0)
		return -1;

	ie->value = val;

	return err;
}

static int write_threshold_mag_int(struct lsm9ds0_dev *dev,
				   struct interrupt_value *ie, int val)
{
	int err;
	u8 high;
	u8 low;

	if(val<0)
		return -1;

	if(val>MAX_TH_MAG)
		return -1;

	low = (u8)(0xff & val);

	err = write_bit_on_register(dev, ie->address, NULL, 0, low);
	if(err<0)
		return -1;

	high = (u8)(0xff & (val >> 8));

	err = write_bit_on_register(dev, (ie->address)+1, NULL, 0, high);
	if(err<0)
		return -1;

	ie->value = val;

	return err;
}

static ssize_t attr_get_gen1_status(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val = -1;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if(strcmp(attr->attr.name, "pin1_enable") == 0) {
		val = atomic_read(&dev->interrupt->gen1_pin1.enable);
	}
	if(strcmp(attr->attr.name, "pin2_enable") == 0) {
		val = atomic_read(&dev->interrupt->gen1_pin2.enable);
	}
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen1_status(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if(strcmp(attr->attr.name, "pin1_enable") == 0) {
		err = write_gen_int(dev,
					&dev->interrupt->gen1_pin1, (int)val);
	}
	if(strcmp(attr->attr.name, "pin2_enable") == 0) {
		err = write_gen_int(dev,
					&dev->interrupt->gen1_pin2, (int)val);
	}
	return size;
}

static ssize_t attr_get_gen2_status(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val = -1;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if(strcmp(attr->attr.name, "pin1_enable") == 0) {
		val = atomic_read(&dev->interrupt->gen2_pin1.enable);
	}
	if(strcmp(attr->attr.name, "pin2_enable") == 0) {
		val = atomic_read(&dev->interrupt->gen2_pin2.enable);
	}
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen2_status(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if(strcmp(attr->attr.name, "pin1_enable") == 0) {
		err = write_gen_int(dev,
					&dev->interrupt->gen2_pin1, (int)val);
	}
	if(strcmp(attr->attr.name, "pin2_enable") == 0) {
		err = write_gen_int(dev,
					&dev->interrupt->gen2_pin2, (int)val);
	}
	return size;
}

static ssize_t attr_get_gen1_duration(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = dev->interrupt->gen1_duration.value;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen1_duration(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_duration_threshold_int(dev,
				&dev->interrupt->gen1_duration, (int)val);

	return size;
}

static ssize_t attr_get_gen2_duration(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = dev->interrupt->gen2_duration.value;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen2_duration(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_duration_threshold_int(dev,
				&dev->interrupt->gen2_duration, (int)val);

	return size;
}

static ssize_t attr_get_gen1_threshold(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = dev->interrupt->gen1_threshold.value;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen1_threshold(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_duration_threshold_int(dev,
				&dev->interrupt->gen1_threshold, (int)val);

	return size;
}

static ssize_t attr_get_gen2_threshold(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = dev->interrupt->gen2_threshold.value;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen2_threshold(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_duration_threshold_int(dev,
				&dev->interrupt->gen2_threshold, (int)val);

	return size;
}

static ssize_t attr_get_gen_mag_status(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val = -1;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if(strcmp(attr->attr.name, "pin1_enable") == 0) {
		val = atomic_read(&dev->interrupt->gen_mag_pin1.enable);
	}
	if(strcmp(attr->attr.name, "pin2_enable") == 0) {
		val = atomic_read(&dev->interrupt->gen_mag_pin2.enable);
	}
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen_mag_status(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if(strcmp(attr->attr.name, "pin1_enable") == 0) {
		err = write_gen_int(dev,
				&dev->interrupt->gen_mag_pin1, (int)val);
		if(err >= 0) {
		if((atomic_read(&dev->interrupt->gen_mag_pin2.enable))==0)
			write_gen_int(dev,
					&dev->interrupt->gen_mag, (int)val);
		}
	}
	if(strcmp(attr->attr.name, "pin2_enable") == 0) {
		err = write_gen_int(dev,
				&dev->interrupt->gen_mag_pin2, (int)val);
		if(err >= 0) {
		if((atomic_read(&dev->interrupt->gen_mag_pin1.enable))==0)
			write_gen_int(dev,
					&dev->interrupt->gen_mag, (int)val);
		}
	}
	return size;
}

static ssize_t attr_get_gen_mag_threshold(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = dev->interrupt->gen_mag_threshold.value;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen_mag_threshold(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err = -1;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_threshold_mag_int(dev,
				&dev->interrupt->gen_mag_threshold, (int)val);

	return size;
}

static int get_axis(struct lsm9ds0_dev *dev,
					int generator, const char *name) {

	int val;
	int axis;

	if(strcmp(name, "x_high_enable") == 0) {
		axis = 3;
	}
	if(strcmp(name, "x_low_enable") == 0) {
		axis = 0;
	}
	if(strcmp(name, "y_high_enable") == 0) {
		axis = 4;
	}
	if(strcmp(name, "y_low_enable") == 0) {
		axis = 1;
	}
	if(strcmp(name, "z_high_enable") == 0) {
		axis = 5;
	}
	if(strcmp(name, "z_low_enable") == 0) {
		axis = 2;
	}

	if(generator == 1)
		val = atomic_read(&dev->interrupt->gen1_axis[axis].enable);
	else
		val = atomic_read(&dev->interrupt->gen2_axis[axis].enable);

	return val;
}

static int set_axis(struct lsm9ds0_dev *dev, int generator,
					const char *name, unsigned long value)
{
	int err = -1;
	int axis;

	if(strcmp(name, "x_high_enable") == 0) {
		axis = 3;
	}
	if((strcmp(name, "x_low_enable") == 0) ||
					(strcmp(name, "x_enable") == 0)) {
		axis = 0;
	}
	if(strcmp(name, "y_high_enable") == 0) {
		axis = 4;
	}
	if((strcmp(name, "y_low_enable") == 0) ||
					(strcmp(name, "y_enable") == 0)) {
		axis = 1;
	}
	if(strcmp(name, "z_high_enable") == 0) {
		axis = 5;
	}
	if((strcmp(name, "z_low_enable") == 0) ||
					(strcmp(name, "z_enable") == 0)) {
		axis = 2;
	}

	if(generator == 1)
		err = write_gen_int(dev,
			&(dev->interrupt->gen1_axis[axis]), (int)value);
	if(generator == 2)
		err = write_gen_int(dev,
			&(dev->interrupt->gen2_axis[axis]), (int)value);
	if(generator == 3)
		err = write_gen_int(dev,
			&(dev->interrupt->gen_mag_axis[axis]), (int)value);

	if(err < 0)
		return -1;

	return err;
}

static ssize_t attr_get_gen1_axis(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = get_axis(dev,1,attr->attr.name);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen1_axis(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = set_axis(dev, 1, attr->attr.name, val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_get_gen2_axis(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = get_axis(dev,2,attr->attr.name);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen2_axis(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = set_axis(dev, 2, attr->attr.name, val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_get_gen_mag_axis(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = get_axis(dev, 3, attr->attr.name);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen_mag_axis(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = set_axis(dev, 3, attr->attr.name, val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_get_gen1_and_or(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = atomic_read(&dev->interrupt->gen1_and_or.enable);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen1_and_or(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_gen_int(dev, &(dev->interrupt->gen1_and_or), (int)val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_get_gen2_and_or(struct kobject *kobj,
						struct kobj_attribute *attr,
						char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = atomic_read(&dev->interrupt->gen2_and_or.enable);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_gen2_and_or(struct kobject *kobj,
						struct kobj_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_gen_int(dev, &(dev->interrupt->gen2_and_or), (int)val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_set_pin_conf(struct device *device,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;

	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_gen_int(dev,
			&(dev->interrupt->interrupt_pin_conf), (int)val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_get_pin_conf(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	int val;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = atomic_read(&dev->interrupt->interrupt_pin_conf.enable);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_interrupt_polarity(struct device *device,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	unsigned long val;

	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	err = write_gen_int(dev,
			&(dev->interrupt->interrupt_polarity), (int)val);
	if(err < 0)
		return -1;

	return size;
}

static ssize_t attr_get_interrupt_polarity(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	int val;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	val = atomic_read(&dev->interrupt->interrupt_polarity.enable);
	return sprintf(buf, "%d\n", val);
}

static struct kobj_attribute gen1_interrupt_pin1_enable =
__ATTR(pin1_enable, 0664, attr_get_gen1_status, attr_set_gen1_status);
static struct kobj_attribute gen1_interrupt_pin2_enable =
__ATTR(pin2_enable, 0664, attr_get_gen1_status, attr_set_gen1_status);

static struct kobj_attribute gen2_interrupt_pin1_enable =
__ATTR(pin1_enable, 0664, attr_get_gen2_status, attr_set_gen2_status);
static struct kobj_attribute gen2_interrupt_pin2_enable =
__ATTR(pin2_enable, 0664, attr_get_gen2_status, attr_set_gen2_status);

static struct kobj_attribute gen1_duration =
__ATTR(duration, 0664, attr_get_gen1_duration, attr_set_gen1_duration);
static struct kobj_attribute gen2_duration =
__ATTR(duration, 0664, attr_get_gen2_duration, attr_set_gen2_duration);

static struct kobj_attribute gen1_threshold =
__ATTR(threshold, 0664, attr_get_gen1_threshold, attr_set_gen1_threshold);
static struct kobj_attribute gen2_threshold =
__ATTR(threshold, 0664, attr_get_gen2_threshold, attr_set_gen2_threshold);

static struct kobj_attribute mag_gen_interrupt_pin1 =
__ATTR(pin1_enable, 0664, attr_get_gen_mag_status, attr_set_gen_mag_status);
static struct kobj_attribute mag_gen_interrupt_pin2 =
__ATTR(pin2_enable, 0664, attr_get_gen_mag_status, attr_set_gen_mag_status);

static struct kobj_attribute mag_gen_threshold =
__ATTR(threshold, 0664, attr_get_gen_mag_threshold, attr_set_gen_mag_threshold);

static struct kobj_attribute gen1_x_high =
__ATTR(x_high_enable, 0664, attr_get_gen1_axis, attr_set_gen1_axis);
static struct kobj_attribute gen1_x_low =
__ATTR(x_low_enable, 0664, attr_get_gen1_axis, attr_set_gen1_axis);

static struct kobj_attribute gen2_x_high =
__ATTR(x_high_enable, 0664, attr_get_gen2_axis, attr_set_gen2_axis);
static struct kobj_attribute gen2_x_low =
__ATTR(x_low_enable, 0664, attr_get_gen2_axis, attr_set_gen2_axis);

static struct kobj_attribute gen1_y_high =
__ATTR(y_high_enable, 0664, attr_get_gen1_axis, attr_set_gen1_axis);
static struct kobj_attribute gen1_y_low =
__ATTR(y_low_enable, 0664, attr_get_gen1_axis, attr_set_gen1_axis);

static struct kobj_attribute gen2_y_high =
__ATTR(y_high_enable, 0664, attr_get_gen2_axis, attr_set_gen2_axis);
static struct kobj_attribute gen2_y_low =
__ATTR(y_low_enable, 0664, attr_get_gen2_axis, attr_set_gen2_axis);

static struct kobj_attribute gen1_z_high =
__ATTR(z_high_enable, 0664, attr_get_gen1_axis, attr_set_gen1_axis);
static struct kobj_attribute gen1_z_low =
__ATTR(z_low_enable, 0664, attr_get_gen1_axis, attr_set_gen1_axis);

static struct kobj_attribute gen2_z_high =
__ATTR(z_high_enable, 0664, attr_get_gen2_axis, attr_set_gen2_axis);
static struct kobj_attribute gen2_z_low =
__ATTR(z_low_enable, 0664, attr_get_gen2_axis, attr_set_gen2_axis);

static struct kobj_attribute gen_mag_x =
__ATTR(x_enable, 0664, attr_get_gen_mag_axis, attr_set_gen_mag_axis);
static struct kobj_attribute gen_mag_y =
__ATTR(y_enable, 0664, attr_get_gen_mag_axis, attr_set_gen_mag_axis);
static struct kobj_attribute gen_mag_z =
__ATTR(z_enable, 0664, attr_get_gen_mag_axis, attr_set_gen_mag_axis);

static struct kobj_attribute gen1_and_or =
__ATTR(and(1)_or(0)_combination, 0664, attr_get_gen1_and_or,
							attr_set_gen1_and_or);
static struct kobj_attribute gen2_and_or =
__ATTR(and(1)_or(0)_combination, 0664, attr_get_gen2_and_or,
							attr_set_gen2_and_or);


static struct attribute *attributes_acc_interrupt1[] = {
	&gen1_interrupt_pin1_enable.attr,
	&gen1_interrupt_pin2_enable.attr,
	&gen1_duration.attr,
	&gen1_threshold.attr,
	&gen1_x_high.attr,
	&gen1_x_low.attr,
	&gen1_y_high.attr,
	&gen1_y_low.attr,
	&gen1_z_high.attr,
	&gen1_z_low.attr,
	&gen1_and_or.attr,
	NULL,
};

static struct attribute *attributes_acc_interrupt2[] = {
	&gen2_interrupt_pin1_enable.attr,
	&gen2_interrupt_pin2_enable.attr,
	&gen2_duration.attr,
	&gen2_threshold.attr,
	&gen2_x_high.attr,
	&gen2_x_low.attr,
	&gen2_y_high.attr,
	&gen2_y_low.attr,
	&gen2_z_high.attr,
	&gen2_z_low.attr,
	&gen2_and_or.attr,
	NULL,
};

static struct attribute *attributes_mag_interrupt[] = {
	&mag_gen_interrupt_pin1.attr,
	&mag_gen_interrupt_pin2.attr,
	&mag_gen_threshold.attr,
	&gen_mag_x.attr,
	&gen_mag_y.attr,
	&gen_mag_z.attr,
	NULL,
};

static struct attribute *attributes_acc[] = {
	&poll_attr_acc.attr,
	&enable_attr_acc.attr,
	&fs_attr_acc.attr,
	&aa_filter_attr.attr,
	NULL,
};

static struct attribute *attributes_mag[] = {
	&poll_attr_mag.attr,
	&enable_attr_mag.attr,
	&fs_attr_mag.attr,
	NULL,
};

static struct attribute_group attr_group_acc = {
	.attrs = attributes_acc,
};

static struct attribute_group attr_group_mag = {
	.attrs = attributes_mag,
};

static struct attribute_group attr_group_int1_acc = {
	.attrs = attributes_acc_interrupt1,
	.name = "interrupt_generator1",
};

static struct attribute_group attr_group_int2_acc = {
	.attrs = attributes_acc_interrupt2,
	.name = "interrupt_generator2",
};

static struct attribute_group attr_group_int_mag = {
	.attrs = attributes_mag_interrupt,
	.name = "interrupt_generator",
};

static struct device_attribute attributes_com[] = {
	__ATTR(enable_temperature, 0664, attr_get_temp_enable,
							attr_set_temp_enable),
	__ATTR(read_temperature, 0444, attr_get_temp, NULL),
};

static struct device_attribute attributes_interrupt_com[] = {
	__ATTR(interrupt_pin_configuration, 0664, attr_get_pin_conf,
						attr_set_pin_conf),
	__ATTR(interrupt_polarity, 0664, attr_get_interrupt_polarity,
						attr_set_interrupt_polarity),
};

static int create_sysfs_interfaces(struct device *device)
{
	int err;
	int i,n;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);

	acc_kobj = kobject_create_and_add("accelerometer", &device->kobj);
	if(!acc_kobj)
		return -ENOMEM;

	mag_kobj = kobject_create_and_add("magnetometer", &device->kobj);
	if(!mag_kobj)
		return -ENOMEM;

	err = sysfs_create_group(acc_kobj, &attr_group_acc);
	if (err)
		kobject_put(acc_kobj);

	err = sysfs_create_group(mag_kobj, &attr_group_mag);
	if (err)
		kobject_put(mag_kobj);

	if((dev->pdata_acc->gpio_int1 >= 0)||
					(dev->pdata_acc->gpio_int2 >= 0)) {
		err = sysfs_create_group(acc_kobj, &attr_group_int1_acc);
		if (err)
			kobject_put(acc_kobj);

		err = sysfs_create_group(acc_kobj, &attr_group_int2_acc);
		if (err)
			kobject_put(acc_kobj);

		err = sysfs_create_group(mag_kobj, &attr_group_int_mag);
		if (err)
			kobject_put(mag_kobj);

		for (n = 0; n < ARRAY_SIZE(attributes_interrupt_com); n++)
		if (device_create_file(device, attributes_interrupt_com + n))
			goto error1;
	}

	for (i = 0; i < ARRAY_SIZE(attributes_com); i++)
		if (device_create_file(device, attributes_com + i))
			goto error;



	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(device, attributes_com + i);

error1:
	for ( ; n >= 0; n--)
		device_remove_file(device, attributes_interrupt_com + n);

	dev_err(device, "%s:Unable to create interface\n", __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *device)
{
	int i;
	struct lsm9ds0_dev *dev = dev_get_drvdata(device);
	kobject_put(acc_kobj);
	kobject_put(mag_kobj);
	for (i = 0; i < ARRAY_SIZE(attributes_com); i++)
		device_remove_file(device, attributes_com + i);
	if((dev->pdata_acc->gpio_int1 >= 0)||
					(dev->pdata_acc->gpio_int2 >= 0)) {
		for (i = 0; i < ARRAY_SIZE(attributes_interrupt_com); i++)
			device_remove_file(device, attributes_interrupt_com + i);
	}
}

static int lsm9ds0_acc_get_data(struct lsm9ds0_dev *dev, int *xyz)
{
	int i, err = -1;
	u8 acc_data[6];
	s32 hw_d[3] = { 0 };

	err = dev->tf->read(dev->dev, REG_ACC_DATA_ADDR, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s32)( (s16)((acc_data[1] << 8) | (acc_data[0]))));
	hw_d[1] = ((s32)( (s16)((acc_data[3] << 8) | (acc_data[2]))));
	hw_d[2] = ((s32)( (s16)((acc_data[5] << 8) | (acc_data[4]))));

#ifdef LSM9DS0_DEBUG
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM9DS0_ACC_DEV_NAME, acc_data[1], acc_data[0], hw_d[0]);
	pr_debug("%s read y=%X %X(regH regL), y=%d(dec) [ug]\n",
		LSM9DS0_ACC_DEV_NAME, acc_data[3], acc_data[2], hw_d[1]);
	pr_debug("%s read z=%X %X(regH regL), z=%d(dec) [ug]\n",
		LSM9DS0_ACC_DEV_NAME, acc_data[5], acc_data[4], hw_d[2]);
#endif

	hw_d[0] = hw_d[0] * dev->sensitivity_acc;
	hw_d[1] = hw_d[1] * dev->sensitivity_acc;
	hw_d[2] = hw_d[2] * dev->sensitivity_acc;

	for (i = 0; i < 3; i++) {
		xyz[i] = dev->pdata_acc->rot_matrix[0][i] * hw_d[0] +
				dev->pdata_acc->rot_matrix[1][i] * hw_d[1] +
				dev->pdata_acc->rot_matrix[2][i] * hw_d[2];
	}

	return err;
}

static int lsm9ds0_mag_get_data(struct lsm9ds0_dev *dev, int *xyz)
{
	int i, err = -1;
	u8 mag_data[6];
	s32 hw_d[3] = { 0 };

	err = dev->tf->read(dev->dev, REG_MAG_DATA_ADDR, 6, mag_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s32)( (s16)((mag_data[1] << 8) | (mag_data[0]))));
	hw_d[1] = ((s32)( (s16)((mag_data[3] << 8) | (mag_data[2]))));
	hw_d[2] = ((s32)( (s16)((mag_data[5] << 8) | (mag_data[4]))));

#ifdef LSM9DS0_DEBUG
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM9DS0_MAG_DEV_NAME, mag_data[1], mag_data[0], hw_d[0]);
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM9DS0_MAG_DEV_NAME, mag_data[3], mag_data[2], hw_d[1]);
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM9DS0_MAG_DEV_NAME, mag_data[5], mag_data[4], hw_d[2]);
#endif

	hw_d[0] = hw_d[0] * dev->sensitivity_mag;
	hw_d[1] = hw_d[1] * dev->sensitivity_mag;
	hw_d[2] = hw_d[2] * dev->sensitivity_mag;

	for (i = 0; i < 3; i++) {
		xyz[i] = dev->pdata_acc->rot_matrix[0][i] * hw_d[0] +
				dev->pdata_acc->rot_matrix[1][i] * hw_d[1] +
				dev->pdata_acc->rot_matrix[2][i] * hw_d[2];
	}

	return err;
}

static int lsm9ds0_temp_get_data(struct lsm9ds0_dev *dev,
				 int *dec, int *flo)
{
	int err = -1;
	u8 temp_data[2];
	s16 hw_d = 0;

	err = dev->tf->read(dev->dev, REG_TEMP_DATA_ADDR, 2, temp_data);
	if (err < 0)
		return err;

	hw_d = (s16)((temp_data[1] << 8) | (temp_data[0]));

#ifdef LSM9DS0_DEBUG
	pr_debug("%s read T=%X %X(regH regL), T=%d(dec) [C]\n",
		LSM9DS0_DEV_NAME, temp_data[1], temp_data[0], hw_d);
#endif

	*dec = (int)(hw_d/TEMP_SENSITIVITY) + OFFSET_TEMP;
	*flo = (((unsigned int)hw_d)%TEMP_SENSITIVITY);

	return err;
}

static void lsm9ds0_acc_report_values(struct lsm9ds0_dev *dev, int *xyz)
{
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_sync(dev->input_dev_acc);
}

static void lsm9ds0_mag_report_values(struct lsm9ds0_dev *dev, int *xyz)
{
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_mag, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_sync(dev->input_dev_mag);
}

static int lsm9ds0_acc_input_init(struct lsm9ds0_dev *dev)
{
	int err;

	dev->input_dev_acc = input_allocate_device();
	if (!dev->input_dev_acc) {
		dev_err(dev->dev, "acc input dev allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev_acc->name = LSM9DS0_ACC_DEV_NAME;
	dev->input_dev_acc->id.bustype = dev->bus_type;
	dev->input_dev_acc->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_acc, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev_acc->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev_acc->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev_acc->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev_acc->mscbit);

	err = input_register_device(dev->input_dev_acc);
	if (err) {
		dev_err(dev->dev,
			"unable to register accelerometer input device %s\n",
			dev->input_dev_acc->name);
		input_free_device(dev->input_dev_acc);
		return err;
	}

	return 0;
}

static int lsm9ds0_mag_input_init(struct lsm9ds0_dev *dev)
{
	int err;

	dev->input_dev_mag = input_allocate_device();
	if (!dev->input_dev_mag) {
		dev_err(dev->dev, "magn input dev allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev_mag->name = LSM9DS0_MAG_DEV_NAME;
	dev->input_dev_mag->id.bustype = dev->bus_type;
	dev->input_dev_mag->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_mag, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev_mag->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev_mag->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev_mag->mscbit);

	err = input_register_device(dev->input_dev_mag);
	if (err) {
		dev_err(dev->dev,
			"unable to register magnetometer input device %s\n",
			dev->input_dev_mag->name);
		input_free_device(dev->input_dev_mag);
		return err;
	}

	return 0;
}

static void lsm9ds0_input_cleanup(struct lsm9ds0_dev *dev)
{
	input_unregister_device(dev->input_dev_acc);
	input_free_device(dev->input_dev_acc);

	input_unregister_device(dev->input_dev_mag);
	input_free_device(dev->input_dev_mag);
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
	struct lsm9ds0_dev *dev;
	int xyz[3] = { 0 };
	int err;

	dev = container_of((struct work_struct *)input_work_acc,
			   struct lsm9ds0_dev, input_work_acc);

	mutex_lock(&dev->lock);
	err = lsm9ds0_acc_get_data(dev, xyz);
	if (err < 0)
		dev_err(dev->dev, "get_accelerometer_data failed\n");
	else
		lsm9ds0_acc_report_values(dev, xyz);

	mutex_unlock(&dev->lock);
	hrtimer_start(&dev->hr_timer_acc, dev->ktime_acc, HRTIMER_MODE_REL);
}

static void poll_function_work_mag(struct work_struct *input_work_mag)
{
	struct lsm9ds0_dev *dev;
	int xyz[3] = { 0 };
	int err;
	int dec;
	int flo;

	dev = container_of((struct work_struct *)input_work_mag,
			   struct lsm9ds0_dev, input_work_mag);

	mutex_lock(&dev->lock);

	if(atomic_read(&dev->enabled_temp)) {
		err = lsm9ds0_temp_get_data(dev, &dec, &flo);
		if (err < 0)
			dev_err(dev->dev, "get_temperature_data"
								" failed\n");
		else {
			dev->temp_value_dec = dec;
			dev->temp_value_flo = flo;
		}
	}

	if(atomic_read(&dev->enabled_mag)) {
		err = lsm9ds0_mag_get_data(dev, xyz);
		if (err < 0)
			dev_err(dev->dev, "get_magnetometer_data"
								" failed\n");
		else
			lsm9ds0_mag_report_values(dev, xyz);
	}

	mutex_unlock(&dev->lock);
	hrtimer_start(&dev->hr_timer_mag, dev->ktime_mag, HRTIMER_MODE_REL);
}

enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct lsm9ds0_dev *dev;


	dev = container_of((struct hrtimer *)timer,
			   struct lsm9ds0_dev, hr_timer_acc);

	queue_work(dev->data_workqueue, &dev->input_work_acc);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart poll_function_read_mag(struct hrtimer *timer)
{
	struct lsm9ds0_dev *dev;


	dev = container_of((struct hrtimer *)timer,
			   struct lsm9ds0_dev, hr_timer_mag);

	queue_work(dev->data_workqueue, &dev->input_work_mag);
	return HRTIMER_NORESTART;
}

int lsm9ds0_probe(struct lsm9ds0_dev *dev)
{
	int err;

	mutex_lock(&dev->lock);

	err = lsm9ds0_check_whoami(dev);
	if (err < 0)
		goto err_mutexunlock;

	dev->pdata_acc = kzalloc(sizeof(*dev->pdata_acc), GFP_KERNEL);
	dev->pdata_mag = kzalloc(sizeof(*dev->pdata_mag), GFP_KERNEL);
	if ((dev->pdata_acc == NULL) || (dev->pdata_mag == NULL)) {
		err = -ENOMEM;
		dev_err(dev->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (dev->dev->platform_data == NULL) {
		memcpy(dev->pdata_acc, &default_lsm9ds0_acc_pdata,
		       sizeof(*dev->pdata_acc));
		memcpy(dev->pdata_mag, &default_lsm9ds0_mag_pdata,
		       sizeof(*dev->pdata_mag));
		dev_info(dev->dev, "using def pdata for acc and magn\n");
	} else {
		struct lsm9ds0_main_platform_data *tmp;
		tmp = kzalloc(sizeof(struct lsm9ds0_main_platform_data),
			      GFP_KERNEL);
		if (tmp == NULL)
			goto exit_kfree_pdata;
		memcpy(tmp, dev->dev->platform_data, sizeof(*tmp));
		if (tmp->pdata_acc == NULL) {
			memcpy(dev->pdata_acc, &default_lsm9ds0_acc_pdata,
			       sizeof(*dev->pdata_acc));
			dev_info(dev->dev, "using def pdata for acc\n");
		} else {
			memcpy(dev->pdata_acc, tmp->pdata_acc,
			       sizeof(*dev->pdata_acc));
		}
		if(tmp->pdata_mag == NULL) {
			memcpy(dev->pdata_mag, &default_lsm9ds0_mag_pdata,
			       sizeof(*dev->pdata_mag));
			dev_info(dev->dev, "using def pdata for magn\n");
		} else {
			memcpy(dev->pdata_mag, tmp->pdata_mag,
			       sizeof(*dev->pdata_mag));
		}
		kfree(tmp);
	}

	err = lsm9ds0_acc_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate pdata for acc\n");
		goto exit_kfree_pdata;
	}

	err = lsm9ds0_mag_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate pdata for magn\n");
		goto exit_kfree_pdata;
	}

	if (dev->pdata_acc->init) {
		err = dev->pdata_acc->init();
		if (err < 0) {
			dev_err(dev->dev, "accelerometer init failed: %d\n",
				err);
			goto err_pdata_acc_init;
		}
	}

	if (dev->pdata_mag->init) {
		err = dev->pdata_mag->init();
		if (err < 0) {
			dev_err(dev->dev, "magnetometer init failed: %d\n",
				err);
			goto err_pdata_mag_init;
		}
	}

	if (dev->pdata_acc->gpio_int1 >= 0) {
		if (!gpio_is_valid(dev->pdata_acc->gpio_int1)) {
			dev_err(dev->dev, "The requested GPIO [%d] is not "
				"available\n", dev->pdata_acc->gpio_int1);
			err = -EINVAL;
			goto err_gpio1_valid;
		}

		err = gpio_request(dev->pdata_acc->gpio_int1,
				   "INTERRUPT_PIN1_LSM9DS0");
		if(err < 0) {
			dev_err(dev->dev, "Unable to request GPIO [%d].\n",
				dev->pdata_acc->gpio_int1);
			err = -EINVAL;
			goto err_gpio1_valid;
		}
		gpio_direction_input(dev->pdata_acc->gpio_int1);
		dev->irq1 = gpio_to_irq(dev->pdata_acc->gpio_int1);
		if(dev->irq1 < 0) {
			dev_err(dev->dev, "GPIO [%d] cannot be used as "
				"interrupt.\n", dev->pdata_acc->gpio_int1);
			err = -EINVAL;
			goto err_gpio1_irq;
		}
		pr_info("%s: %s has set irq1 to irq: %d, mapped on gpio:%d\n",
			LSM9DS0_DEV_NAME, __func__, dev->irq1,
			dev->pdata_acc->gpio_int1);
	}

	if (dev->pdata_acc->gpio_int2 >= 0) {
		if (!gpio_is_valid(dev->pdata_acc->gpio_int2)) {
			dev_err(dev->dev, "The requested GPIO [%d] is not "
				"available\n", dev->pdata_acc->gpio_int2);
			err = -EINVAL;
			goto err_gpio2_valid;
		}

		err = gpio_request(dev->pdata_acc->gpio_int2,
				   "INTERRUPT_PIN2_LSM9DS0");
		if(err < 0) {
			dev_err(dev->dev, "Unable to request GPIO [%d].\n",
				dev->pdata_acc->gpio_int2);
			err = -EINVAL;
			goto err_gpio2_valid;
		}
		gpio_direction_input(dev->pdata_acc->gpio_int2);
		dev->irq2 = gpio_to_irq(dev->pdata_acc->gpio_int2);
		if(dev->irq2 < 0) {
			dev_err(dev->dev, "GPIO [%d] cannot be used as "
				"interrupt.\n", dev->pdata_acc->gpio_int2);
			err = -EINVAL;
			goto err_gpio2_irq;
		}
		pr_info("%s: %s has set irq2 to irq: %d, mapped on gpio:%d\n",
			LSM9DS0_DEV_NAME, __func__, dev->irq2,
			dev->pdata_acc->gpio_int2);
	}

	err = lsm9ds0_hw_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm9ds0_acc_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "accelerometer power on failed: %d\n", err);
		goto err_pdata_init;
	}
	err = lsm9ds0_mag_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "magnetometer power on failed: %d\n", err);
		goto err_pdata_init;
	}

	err = lsm9ds0_acc_update_fs_range(dev, dev->pdata_acc->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range on accelerometer failed\n");
		goto  err_power_off_acc;
	}

	err = lsm9ds0_mag_update_fs_range(dev, dev->pdata_mag->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update_fs_range on magnetometer failed\n");
		goto  err_power_off_mag;
	}

	err = lsm9ds0_acc_update_odr(dev, dev->pdata_acc->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr on accelerometer failed\n");
		goto  err_power_off;
	}

	err = lsm9ds0_mag_update_odr(dev, dev->pdata_mag->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update_odr on magnetometer failed\n");
		goto  err_power_off;
	}

	err = lsm9ds0_acc_update_filter(dev, 
					dev->pdata_acc->aa_filter_bandwidth);
	if (err < 0) {
		dev_err(dev->dev, "update_filter on accelerometer failed\n");
		goto  err_power_off;
	}

	err = lsm9ds0_acc_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "accelerometer input init failed\n");
		goto err_power_off;
	}

	err = lsm9ds0_mag_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "magnetometer input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev,
		"device LSM9DS0_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm9ds0_acc_device_power_off(dev);
	lsm9ds0_mag_device_power_off(dev);

	if(dev->pdata_acc->gpio_int1 >= 0){
		INIT_WORK(&dev->irq1_work, lsm9ds0_irq1_work_func);
		dev->irq1_work_queue =
				create_singlethread_workqueue("lsm9ds0_wq1");
		if (!dev->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev, "cannot create work queue1: %d\n",
				err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(dev->irq1, lsm9ds0_isr1, IRQF_TRIGGER_RISING,
				  "lsm9ds0_irq1", dev);
		if (err < 0) {
			dev_err(dev->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(dev->irq1);
	}

	if(dev->pdata_acc->gpio_int2 >= 0){
		INIT_WORK(&dev->irq2_work, lsm9ds0_irq2_work_func);
		dev->irq2_work_queue =
				create_singlethread_workqueue("lsm9ds0_wq2");
		if (!dev->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(dev->dev, "cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(dev->irq2, lsm9ds0_isr2, IRQF_TRIGGER_RISING,
				  "lsm9ds0_irq2", dev);
		if (err < 0) {
			dev_err(dev->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(dev->irq2);
	}

	dev->data_workqueue = create_workqueue("lsm9ds0_workqueue");
	if (!dev->data_workqueue)
		goto err_destoyworkqueue2;

	hrtimer_init(&dev->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer_acc.function = &poll_function_read_acc;
	hrtimer_init(&dev->hr_timer_mag, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->hr_timer_mag.function = &poll_function_read_mag;

	INIT_WORK(&dev->input_work_acc, poll_function_work_acc);
	INIT_WORK(&dev->input_work_mag, poll_function_work_mag);

	mutex_unlock(&dev->lock);

	return 0;

err_destoyworkqueue2:
	destroy_workqueue(dev->irq2_work_queue);
err_free_irq1:
	free_irq(dev->irq1, dev);
err_destoyworkqueue1:
	destroy_workqueue(dev->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(dev->dev);
err_input_cleanup:
	lsm9ds0_input_cleanup(dev);
err_power_off:
err_power_off_mag:
	lsm9ds0_mag_device_power_off(dev);
err_power_off_acc:
	lsm9ds0_acc_device_power_off(dev);
	kfree(dev->interrupt);
err_hw_init:
err_gpio2_irq:
	gpio_free(dev->pdata_acc->gpio_int2);
err_gpio2_valid:
err_gpio1_irq:
	gpio_free(dev->pdata_acc->gpio_int1);
err_gpio1_valid:
err_pdata_init:
err_pdata_mag_init:
	if (dev->pdata_mag->exit)
		dev->pdata_mag->exit();
err_pdata_acc_init:
	if (dev->pdata_acc->exit)
		dev->pdata_acc->exit();
exit_kfree_pdata:
	kfree(dev->pdata_acc);
	kfree(dev->pdata_mag);
err_mutexunlock:
	mutex_unlock(&dev->lock);
	if (dev->data_workqueue)
		destroy_workqueue(dev->data_workqueue);

	return err;
}
EXPORT_SYMBOL(lsm9ds0_probe);

int lsm9ds0_remove(struct lsm9ds0_dev *dev)
{
	lsm9ds0_disable(dev);

	if (dev->pdata_acc->gpio_int1 >= 0) {
		free_irq(dev->irq1, dev);
		gpio_free(dev->pdata_acc->gpio_int1);
		destroy_workqueue(dev->irq1_work_queue);
	}

	if (dev->pdata_acc->gpio_int2 >= 0) {
		free_irq(dev->irq2, dev);
		gpio_free(dev->pdata_acc->gpio_int2);
		destroy_workqueue(dev->irq2_work_queue);
	}

	lsm9ds0_acc_input_cleanup(dev);
	lsm9ds0_mag_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	if (dev->pdata_acc->exit)
		dev->pdata_acc->exit();

	if (dev->pdata_mag->exit)
		dev->pdata_mag->exit();

	if ((dev->pdata_acc->gpio_int1 >= 0) ||
	    (dev->pdata_acc->gpio_int2 >= 0))
		kfree(dev->interrupt);

	if (dev->data_workqueue)
		destroy_workqueue(dev->data_workqueue);

	kfree(dev->pdata_acc);
	kfree(dev->pdata_mag);

	return 0;
}
EXPORT_SYMBOL(lsm9ds0_remove);

MODULE_DESCRIPTION("lsm9ds0 accelerometer and magnetometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");
