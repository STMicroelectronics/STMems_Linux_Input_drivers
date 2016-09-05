/******************** (C) COPYRIGHT 2016 STMicroelectronics ******************
*
* File Name		: lsm6ds0_core.c
* Author		: MSH - C&I BU - Application Team
*			: Giuseppe Barba (giuseppe.barba@st.com)
*			: Author is willing to be considered the contact
*			: and update point for the driver.
* Version		: V.1.1.0
* Date			: 2016/May/13
* Description		: LSM6DS0 driver
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
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
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
#endif

#include "lsm6ds0.h"

#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
#define COMP_X_FACTOR_ADDR		0x7F
#define COMP_Y_FACTOR_ADDR		0xF0
#define COMP_Z_FACTOR_ADDR		0xF1
#define COMP_HW_DISABLE_MASK		0xC0
#endif

#ifdef CONFIG_INPUT_LSM6DS0_LP
#define LSM6DS0_GYR_LPF1		0
#define LSM6DS0_GYR_LPF1_2		1
#define LSM6DS0_GYR_LP_TH_US		(LSM6DS0_ODR_US_100)

#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL
#define CONFIG_INPUT_LSM6DS0_S_MODEL_LP
#endif
#endif

#define MS_TO_US(x)			(x * 1000L)
#define US_TO_NS(x)			(x * 1000L)
#define MS_TO_NS(x)			(US_TO_NS(MS_TO_US(x)))
#define NS_TO_US(x)			(x / 1000)
#define US_TO_MS(x)			(x / 1000)
#define NS_TO_MS(x)			(US_TO_MS(NS_TO_US(x)))

#define REFERENCE_G			0x0B

/* TODO: check the following values */
/* Sensitivity */
#define SENSITIVITY_ACC_2G		61	/** ug/LSB */
#define SENSITIVITY_ACC_4G		122	/** ug/LSB */
#define SENSITIVITY_ACC_8G		244	/** ug/LSB */
#define SENSITIVITY_GYR_250		8750	/** udps/LSB */
#define SENSITIVITY_GYR_500		17500	/** udps/LSB */
#define SENSITIVITY_GYR_2000		70000	/** udps/LSB */
#define SENSITIVITY_TEMP		16	/** LSB/C */
#define OFFSET_TEMP			25	/** Offset temperature */

#define ACC_G_MAX_POS			1495040	/** max positive value acc [ug] */
#define ACC_G_MAX_NEG			1495770	/** max negative value acc [ug] */
#define GYR_FS_MAX			32768

#define FUZZ				0
#define FLAT				0

#define FILTER_50			50	/** Anti-Aliasing 50 Hz */
#define FILTER_100			100	/** Anti-Aliasing 105 Hz */
#define FILTER_200			200	/** Anti-Aliasing 211 Hz */
#define FILTER_400			400	/** Anti-Aliasing 408 Hz */

#define RANGE_245DPS			245
#define RANGE_500DPS			500
#define RANGE_2000DPS			2000

#define ACT_THS				0x04
#define ACT_DUR				0x05
#define WHO_AM_I			0x0F
#define WHO_AM_I_VAL			0x68

/* Angular rate sensor Control Register 1 */
#define CTRL_REG1_G			0x10

#define BW_G_SHIFT			0
#define BW_G_MASK			0x03

#define FS_G_SHIFT			3
#define FS_G_MASK			0x18

/* Angular rate sensor Control Register 2 */
#define CTRL_REG2_G			0x11

#define OUT_SEL_SHIFT			0
#define OUT_SEL_MASK			0x03

#define INT_SEL_SHIFT			2
#define INT_SEL_MASK			0x0C

#define CTRL_REG3_G			0x12
#ifdef CONFIG_INPUT_LSM6DS0_LP
#define CTRL_REG3_G_LP_MODE_MASK	0x80
#endif

/* Angular rate sensor sign and orientation register. */
#define ORIENT_CFG_G			0x13
#define ORIENT_CFG_G_SIGN_X_MASK	0x20
#define ORIENT_CFG_G_SIGN_Y_MASK	0x10
#define ORIENT_CFG_G_SIGN_Z_MASK	0x08
#define ORIENT_CFG_G_SIGN_ORIENT_MASK	0x07
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16
#define STATUS_REG1			0x17
#define	OUT_X_L_G			0x18 /* 1st AXIS OUT REG of 6 */
#define CTRL_REG4			0x1E
#define CTRL_REG4_DEF			0x38
#define CTRL_REG4_X_EN			0x08
#define CTRL_REG4_Y_EN			0x10
#define CTRL_REG4_Z_EN			0x20
#define CTRL_REG4_ALL_AXES_EN		0x38
#define CTRL_REG4_AXES_EN_MASK		0x38
#define CTRL_REG5_XL			0x1F
#define CTRL_REG5_XL_DEF		0x38
#define CTRL_REG6_XL			0x20
#define LSM6DS0_ACC_FS_DEF		LSM6DS0_ACC_FS_2G
#define BW_SCAL_ODR_SHIFT		2
#define BW_SCAL_ODR_MASK		0x04
#define BW_XL_50			0x0C
#define BW_XL_105			0x08
#define BW_XL_211			0x04
#define BW_XL_408			0x00
#define BW_XL_DEF			BW_XL_408
#define CTRL_REG7_XL			0x21
#define CTRL_REG8			0x22
#define CTRL_REG8_DEF			0x44
#define CTRL_REG9			0x23
#define CTRL_REG10			0x24
#define STATUS_REG2			0x27
#define OUT_X_L_XL			0x28 /* 1st AXIS OUT REG of 6 */
#define FIFO_CTRL			0x2E
#define FIFO_SRC			0x2F
/* INT1_A/G pin control register. */
#define INT1_CTRL			0x0C
#define INT1_CTRL_IG_G_MASK		0x80
#define INT1_CTRL_IG_XL_MASK		0x40
#define INT1_CTRL_FSS5_MASK		0x20
#define INT1_CTRL_OVR_MASK		0x10
#define INT1_CTRL_FTH_MASK		0x08
#define INT1_CTRL_BOOT_MASK		0x04
#define INT1_CTRL_DRDY_G_MASK		0x02
#define INT1_CTRL_DRDY_XL_MASK		0x01

/* INT2_A/G pin control register. */
#define INT2_CTRL			0x0D
#define INT2_CTRL_INACT_MASK		0x80
#define INT2_CTRL_FSS5_MASK		0x20
#define INT2_CTRL_OVR_MASK		0x10
#define INT2_CTRL_FTH_MASK		0x08
#define INT2_CTRL_DRDY_TEMP_MASK	0x04
#define INT2_CTRL_DRDY_G_MASK		0x02
#define INT2_CTRL_DRDY_XL_MASK		0x01

/* Linear acceleration sensor interrupt source register. */
#define INT_GEN_SRC_XL			0x26
#define INT_GEN_SRC_XL_IA_MASK		0x40
#define INT_GEN_SRC_XL_ZH_MASK		0x20
#define INT_GEN_SRC_XL_ZL_MASK		0x10
#define INT_GEN_SRC_XL_YH_MASK		0x08
#define INT_GEN_SRC_XL_YL_MASK		0x04
#define INT_GEN_SRC_XL_XH_MASK		0x02
#define INT_GEN_SRC_XL_XL_MASK		0x01

/* Linear acceleration sensor interrupt generator configuration register. */
#define INT_GEN_CFG_XL			0x06
#define INT_GEN_CFG_XL_AOI_MASK		0x80
#define INT_GEN_CFG_XL_6D_MASK		0x40
#define INT_GEN_CFG_XL_ZHIE_MASK	0x20
#define INT_GEN_CFG_XL_ZLIE_MASK	0x10
#define INT_GEN_CFG_XL_YHIE_MASK	0x08
#define INT_GEN_CFG_XL_YLIE_MASK	0x04
#define INT_GEN_CFG_XL_XHIE_MASK	0x02
#define INT_GEN_CFG_XL_XLIE_MASK	0x01

/* Linear acceleration sensor interrupt threshold registers. */
#define INT_GEN_THS_X_XL		0x07
#define INT_GEN_THS_Y_XL		0x08
#define INT_GEN_THS_Z_XL		0x09

/* Linear acceleration sensor interrupt duration register. */
#define INT_GEN_DUR_XL			0x0A
#define INT_GEN_DUR_XL_WAIT_MASK	0x80
#define INT_GEN_DUR_XL_DUR_MASK		0x7F

/* Angular rate sensor interrupt source register. */
#define INT_GEN_SRC_G			0x14
#define INT_GEN_SRC_G_IA_MASK		0x40
#define INT_GEN_SRC_G_ZH_MASK		0x20
#define INT_GEN_SRC_G_ZL_MASK		0x10
#define INT_GEN_SRC_G_YH_MASK		0x08
#define INT_GEN_SRC_G_YL_MASK		0x04
#define INT_GEN_SRC_G_XH_MASK		0x02
#define INT_GEN_SRC_G_XL_MASK		0x01

/* Angular rate sensor interrupt generator configuration register. */
#define INT_GEN_CFG_G			0x30
#define INT_GEN_CFG_G_AOI_MASK		0x80
#define INT_GEN_CFG_G_LIR_MASK		0x40
#define INT_GEN_CFG_G_ZHIE_MASK		0x20
#define INT_GEN_CFG_G_ZLIE_MASK		0x10
#define INT_GEN_CFG_G_YHIE_MASK		0x08
#define INT_GEN_CFG_G_YLIE_MASK		0x04
#define INT_GEN_CFG_G_XHIE_MASK		0x02
#define INT_GEN_CFG_G_XLIE_MASK		0x01

/* Angular rate sensor interrupt generator threshold registers. */
#define INT_GEN_THS_XH_G		0x31
#define INT_GEN_THS_XL_G		0x32
#define INT_GEN_THS_YH_G		0x33
#define INT_GEN_THS_YL_G		0x34
#define INT_GEN_THS_ZH_G		0x35
#define INT_GEN_THS_ZL_G		0x36

/* Angular rate sensor interrupt generator duration register. */
#define INT_GEN_DUR_G			0x37
#define INT_GEN_DUR_G_WAIT_MASK		0x80
#define INT_GEN_DUR_G_DUR_MASK		0x7F

#define DEF_ZERO			0x00
#define UNDEF				0x00

#define to_dev(obj) container_of(obj, struct device, kobj)
#define LSM6DS0_ATTTR(_name, _leaf, _mode, _show, _store) \
	struct device_attribute lsm6ds0_attr_##_name = __ATTR(_leaf, _mode, _show, _store)

static struct workqueue_struct *lsm6ds0_workqueue;


struct output_rate{
	uint32_t cutoff_us;
	uint8_t value;
};

static const struct output_rate lsm6ds0_gyr_odr_table[] = {
	{ LSM6DS0_ODR_US_110, (LSM6DS0_GYR_ODR_110 | (LSM6DS0_GYR_BW_11)) },
	{ LSM6DS0_ODR_US_101, (LSM6DS0_GYR_ODR_101 | (LSM6DS0_GYR_BW_11)) },
	{ LSM6DS0_ODR_US_100, (LSM6DS0_GYR_ODR_100 | (LSM6DS0_GYR_BW_11)) },
	{ LSM6DS0_ODR_US_011, (LSM6DS0_GYR_ODR_011 | (LSM6DS0_GYR_BW_11)) },
	{ LSM6DS0_ODR_US_010, (LSM6DS0_GYR_ODR_010 | (LSM6DS0_GYR_BW_11)) },
	{ LSM6DS0_ODR_US_001, (LSM6DS0_GYR_ODR_001) },
};

static const struct output_rate lsm6ds0_acc_odr_table[] = {
        { LSM6DS0_ODR_US_110, (LSM6DS0_GYR_ODR_110) },
        { LSM6DS0_ODR_US_101, (LSM6DS0_GYR_ODR_101) },
        { LSM6DS0_ODR_US_100, (LSM6DS0_GYR_ODR_100) },
        { LSM6DS0_ODR_US_011, (LSM6DS0_GYR_ODR_011) },
        { LSM6DS0_ODR_US_010, (LSM6DS0_GYR_ODR_010) },
        { LSM6DS0_ODR_US_001, (LSM6DS0_GYR_ODR_001) },
};

struct discard_list {
	uint32_t param;
	uint8_t count;
};

static const struct acc_turn_on_time {
	uint32_t odr_us;
	struct discard_list disc_list[4];
} lsm6ds0_acc_turn_on_time [] = {
	{ LSM6DS0_ODR_US_110,{	{ LSM6DS0_ACC_BW_400, 2},
				{ LSM6DS0_ACC_BW_200, 4},
				{ LSM6DS0_ACC_BW_100, 7},
				{ LSM6DS0_ACC_BW_50,  14},},},
	{ LSM6DS0_ODR_US_101,{	{ LSM6DS0_ACC_BW_400, 1},
				{ LSM6DS0_ACC_BW_200, 2},
				{ LSM6DS0_ACC_BW_100, 4},
				{ LSM6DS0_ACC_BW_50,  7},},},
	{ LSM6DS0_ODR_US_100,{	{ LSM6DS0_ACC_BW_400, 1},
				{ LSM6DS0_ACC_BW_200, 1},
				{ LSM6DS0_ACC_BW_100, 2},
				{ LSM6DS0_ACC_BW_50,  4},},},
	{ LSM6DS0_ODR_US_011,{	{ LSM6DS0_ACC_BW_400, 1},
				{ LSM6DS0_ACC_BW_200, 1},
				{ LSM6DS0_ACC_BW_100, 1},
				{ LSM6DS0_ACC_BW_50,  2},},},
	{ LSM6DS0_ODR_US_010,{	{ LSM6DS0_ACC_BW_400, 0},
				{ LSM6DS0_ACC_BW_200, 0},
				{ LSM6DS0_ACC_BW_100, 0},
				{ LSM6DS0_ACC_BW_50,  0},},},
	{ LSM6DS0_ODR_US_001,{	{ LSM6DS0_ACC_BW_400, 0},
				{ LSM6DS0_ACC_BW_200, 0},
				{ LSM6DS0_ACC_BW_100, 0},
				{ LSM6DS0_ACC_BW_50,  0},},},
};

#ifdef CONFIG_INPUT_LSM6DS0_LP
static const struct gyr_turn_on_time {
	uint32_t odr_us;
	struct discard_list disc_list[2];
} lsm6ds0_gyr_turn_on_time [] = {
	{ LSM6DS0_ODR_US_110,{	{ LSM6DS0_GYR_LPF1,   8},
				{ LSM6DS0_GYR_LPF1_2, 18},},},
	{ LSM6DS0_ODR_US_101,{	{ LSM6DS0_GYR_LPF1,   5},
				{ LSM6DS0_GYR_LPF1_2, 15},},},
	{ LSM6DS0_ODR_US_100,{	{ LSM6DS0_GYR_LPF1,   4},
				{ LSM6DS0_GYR_LPF1_2, 14},},},
	{ LSM6DS0_ODR_US_011,{	{ LSM6DS0_GYR_LPF1,   3},
				{ LSM6DS0_GYR_LPF1_2, 13},},},
	{ LSM6DS0_ODR_US_010,{	{ LSM6DS0_GYR_LPF1,   3},
				{ LSM6DS0_GYR_LPF1_2, 13},},},
	{ LSM6DS0_ODR_US_001,{	{ LSM6DS0_GYR_LPF1,   2},
				{ LSM6DS0_GYR_LPF1_2, 0},},},
};
#endif

static const struct lsm6ds0_acc_platform_data default_lsm6ds0_acc_pdata = {
	.fs_range = LSM6DS0_ACC_FS_2G,
	.poll_interval = LSM6DS0_ACC_POLL_INTERVAL_DEF,
	.min_interval = LSM6DS0_ACC_MIN_POLL_PERIOD_US,
	.aa_filter_bandwidth = LSM6DS0_ACC_BW_400,
};

static const struct lsm6ds0_gyr_platform_data default_lsm6ds0_gyr_pdata = {
	.fs_range = LSM6DS0_GYR_FS_245DPS,
	.poll_interval = LSM6DS0_GYR_POLL_INTERVAL_DEF,
	.min_interval = LSM6DS0_GYR_MIN_POLL_PERIOD_US,
};

struct lsm6ds0_main_platform_data default_lsm6ds0_main_platform_data = {
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
	.gpio_int1 = LSM6DS0_INT1_GPIO_DEF,
	.gpio_int2 = LSM6DS0_INT2_GPIO_DEF,
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


struct reg_rw {
	uint8_t address;
	uint8_t default_val;
	uint8_t resume_val;
};

struct reg_r {
	uint8_t address;
	uint8_t default_val;
};

static struct status_registers {
	struct reg_rw act_ths;
	struct reg_rw act_dur;
	struct reg_rw int_gen_cfg_xl;
	struct reg_rw int_gen_ths_x_xl;
	struct reg_rw int_gen_ths_y_xl;
	struct reg_rw int_gen_ths_z_xl;
	struct reg_rw int_gen_dur_xl;
	struct reg_rw reference_g;
	struct reg_rw int1_ctrl;
	struct reg_rw int2_ctrl;
	struct reg_r who_am_i;
	struct reg_rw ctrl_reg1_g;
	struct reg_rw ctrl_reg2_g;
	struct reg_rw ctrl_reg3_g;
	struct reg_rw orient_cfg_g;
	struct reg_r int_gen_src_g;
	struct reg_r status_reg1;
	struct reg_rw ctrl_reg4;
	struct reg_rw ctrl_reg5_xl;
	struct reg_rw ctrl_reg6_xl;
	struct reg_rw ctrl_reg7_xl;
	struct reg_rw ctrl_reg8;
	struct reg_rw ctrl_reg9;
	struct reg_rw ctrl_reg10;
	struct reg_r int_gen_src_xl;
	struct reg_r status_reg2;
	struct reg_rw fifo_ctrl;
	struct reg_r fifo_src;
	struct reg_rw int_gen_cfg_g;
	struct reg_rw int_gen_ths_xh_g;
	struct reg_rw int_gen_ths_xl_g;
	struct reg_rw int_gen_ths_yh_g;
	struct reg_rw int_gen_ths_yl_g;
	struct reg_rw int_gen_ths_zh_g;
	struct reg_rw int_gen_ths_zl_g;
	struct reg_rw int_gen_dur_g;
} status_registers = {
	.act_ths =
		{.address = ACT_THS, 		.default_val = DEF_ZERO,},
	.act_dur =
		{.address = ACT_DUR, 		.default_val = DEF_ZERO,},
	.int_gen_cfg_xl =
		{.address = INT_GEN_CFG_XL, 	.default_val = DEF_ZERO,},
	.int_gen_ths_x_xl =
		{.address = INT_GEN_THS_X_XL, 	.default_val = DEF_ZERO,},
	.int_gen_ths_y_xl =
		{.address = INT_GEN_THS_Y_XL, 	.default_val = DEF_ZERO,},
	.int_gen_ths_z_xl =
		{.address = INT_GEN_THS_Z_XL, 	.default_val = DEF_ZERO,},
	.int_gen_dur_xl =
		{.address = INT_GEN_DUR_XL, 	.default_val = DEF_ZERO,},
	.reference_g =
		{.address = REFERENCE_G, 	.default_val = DEF_ZERO,},
	.int1_ctrl =
		{.address = INT1_CTRL,		.default_val = DEF_ZERO,},
	.int2_ctrl =
		{.address = INT2_CTRL,		.default_val = DEF_ZERO,},
	.who_am_i =
		{.address = WHO_AM_I,		.default_val = WHO_AM_I_VAL,},
	.ctrl_reg1_g =
		{.address = CTRL_REG1_G,	.default_val = DEF_ZERO,},
	.ctrl_reg2_g =
		{.address = CTRL_REG2_G,	.default_val = DEF_ZERO,},
#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL_LP
		/** LSM6L0 require the LP bit to be set to 0 */
	.ctrl_reg3_g =
		{.address = CTRL_REG3_G,
				.default_val = CTRL_REG3_G_LP_MODE_MASK,},
#else
	.ctrl_reg3_g =
		{.address = CTRL_REG3_G,	.default_val = DEF_ZERO,},
#endif
	.orient_cfg_g =
		{.address = ORIENT_CFG_G,	.default_val = DEF_ZERO,},
	.int_gen_src_g =
		{.address = INT_GEN_SRC_G,	.default_val = UNDEF,},
	.status_reg1 =
		{.address = STATUS_REG1,	.default_val = UNDEF,},
	.ctrl_reg4 =
		{.address = CTRL_REG4,		.default_val = CTRL_REG4_DEF,},
	.ctrl_reg5_xl =
		{.address = CTRL_REG5_XL,	.default_val = CTRL_REG5_XL_DEF,},
	.ctrl_reg6_xl =
		{.address = CTRL_REG6_XL,	.default_val = DEF_ZERO,},
	.ctrl_reg7_xl =
		{.address = CTRL_REG7_XL,	.default_val = DEF_ZERO,},
	.ctrl_reg8 =
		{.address = CTRL_REG8,		.default_val = CTRL_REG8_DEF,},
	.ctrl_reg9 =
		{.address = CTRL_REG9,		.default_val = DEF_ZERO,},
	.ctrl_reg10 =
		{.address = CTRL_REG10,	.default_val = DEF_ZERO,},
	.int_gen_src_xl =
		{.address = INT_GEN_SRC_XL,	.default_val = DEF_ZERO,},
	.status_reg2 =
		{.address = STATUS_REG2,	.default_val = UNDEF,},
	.fifo_ctrl =
		{.address = FIFO_CTRL,		.default_val = DEF_ZERO,},
	.fifo_src =
		{.address = FIFO_SRC,		.default_val = UNDEF,},
	.int_gen_cfg_g =
		{.address = INT_GEN_CFG_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_xh_g =
		{.address = INT_GEN_THS_XH_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_xl_g =
		{.address = INT_GEN_THS_XL_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_yh_g =
		{.address = INT_GEN_THS_YH_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_yl_g =
		{.address = INT_GEN_THS_YL_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_zh_g =
		{.address = INT_GEN_THS_ZH_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_zl_g =
		{.address = INT_GEN_THS_ZL_G,	.default_val = DEF_ZERO,},
	.int_gen_dur_g =
		{.address = INT_GEN_DUR_G,	.default_val = DEF_ZERO,},
};

static int32_t _lsm6ds0_acc_device_power_off(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	uint8_t buf = (LSM6DS0_ACC_ODR_MASK & LSM6DS0_ACC_ODR_OFF) |
				   ((~LSM6DS0_ACC_ODR_MASK) &
				   status_registers.ctrl_reg6_xl.resume_val);

	err = stat->tf->write(stat, status_registers.ctrl_reg6_xl.address, 1, &buf);

	if (err < 0)
		dev_err(stat->dev, "accelerometer soft power off "
							"failed: %d\n", err);

	if (stat->pdata_acc->power_off) {
		stat->pdata_acc->power_off();
	}

	return 0;
}

static int32_t lsm6ds0_acc_device_power_off(struct lsm6ds0_status *stat)
{
	_lsm6ds0_acc_device_power_off(stat);

	atomic_set(&stat->enabled_acc, 0);
	dev_info(stat->dev, "accelerometer switched off.");

	return 0;
}

static int32_t _lsm6ds0_gyr_device_power_off(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	uint8_t buf = (LSM6DS0_GYR_ODR_MASK & LSM6DS0_GYR_ODR_OFF) |
				   ((~LSM6DS0_GYR_ODR_MASK) &
				   status_registers.ctrl_reg1_g.resume_val);

	err = stat->tf->write(stat, status_registers.ctrl_reg1_g.address, 1, &buf);

	if (err < 0)
		dev_err(stat->dev, "gyroscope soft power off "
							"failed: %d\n", err);

	if (stat->pdata_gyr->power_off) {
		stat->pdata_gyr->power_off();
	}

	return 0;
}

static int32_t lsm6ds0_gyr_device_power_off(struct lsm6ds0_status *stat)
{
	_lsm6ds0_gyr_device_power_off(stat);
	atomic_set(&stat->enabled_gyr, 0);
	dev_info(stat->dev, "gyroscope switched off.");

	return 0;
}

static int32_t lsm6ds0_gyr_disable(struct lsm6ds0_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_gyr, 1, 0)) {
		cancel_work_sync(&stat->input_work_gyr);
		hrtimer_cancel(&stat->hr_timer_gyr);
		lsm6ds0_gyr_device_power_off(stat);
	}

	return 0;
}

static int32_t lsm6ds0_acc_disable(struct lsm6ds0_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_acc, 1, 0)) {
		if (atomic_read(&stat->enabled_gyr) > 0)
			lsm6ds0_gyr_disable(stat);

		cancel_work_sync(&stat->input_work_acc);
		hrtimer_cancel(&stat->hr_timer_acc);
		lsm6ds0_acc_device_power_off(stat);
	}

	return 0;
}

static void lsm6ds0_acc_input_cleanup(struct lsm6ds0_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);
}

static void lsm6ds0_gyr_input_cleanup(struct lsm6ds0_status *stat)
{
	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

static enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct lsm6ds0_status *stat;


	stat = container_of((struct hrtimer *)timer,
				struct lsm6ds0_status, hr_timer_acc);

	queue_work(lsm6ds0_workqueue, &stat->input_work_acc);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart poll_function_read_gyr(struct hrtimer *timer)
{
	struct lsm6ds0_status *stat;


	stat = container_of((struct hrtimer *)timer,
				struct lsm6ds0_status, hr_timer_gyr);

	queue_work(lsm6ds0_workqueue, &stat->input_work_gyr);
	return HRTIMER_NORESTART;
}

static void lsm6ds0_validate_polling(uint32_t *min_interval,
					uint32_t *poll_interval,
					uint32_t min)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);
}

static int32_t lsm6ds0_acc_validate_pdata(struct lsm6ds0_status *stat)
{
	int32_t res = -EINVAL;

	lsm6ds0_validate_polling(&stat->pdata_acc->min_interval,
				 &stat->pdata_acc->poll_interval,
				(unsigned int)LSM6DS0_ACC_MIN_POLL_PERIOD_US);

	switch (stat->pdata_acc->aa_filter_bandwidth) {
	case LSM6DS0_ACC_BW_50:
		res = 1;
		break;
	case LSM6DS0_ACC_BW_100:
		res = 1;
		break;
	case LSM6DS0_ACC_BW_200:
		res = 1;
		break;
	case LSM6DS0_ACC_BW_400:
		res = 1;
		break;
	default:
		dev_err(stat->dev, "invalid accelerometer "
			"bandwidth selected: %u\n",
				stat->pdata_acc->aa_filter_bandwidth);
	}

	return res;
}

static int32_t lsm6ds0_gyr_validate_pdata(struct lsm6ds0_status *stat)
{
	/* checks for correctness of minimal polling period */
	lsm6ds0_validate_polling(&stat->pdata_gyr->min_interval,
				 &stat->pdata_gyr->poll_interval,
				(unsigned int)LSM6DS0_GYR_MIN_POLL_PERIOD_US);

	/* Enforce minimum polling interval */
	if (stat->pdata_gyr->poll_interval < stat->pdata_gyr->min_interval) {
		dev_err(stat->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	stat->compensation_temp = 0;
#endif
	return 0;
}


static int32_t lsm6ds0_hw_init(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	uint8_t buf;

	dev_info(stat->dev, "%s: hw init start\n",
						LSM6DS0_ACC_GYR_DEV_NAME);

	err = stat->tf->read(stat, status_registers.who_am_i.address, 1, &buf);
	if (err < 0) {
		dev_warn(stat->dev, "Error reading WHO_AM_I: is device"
		" available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf != status_registers.who_am_i.default_val) {
	dev_err(stat->dev,
		"device unknown. Expected: 0x%02x,"
		" Replies: 0x%02x\n", status_registers.who_am_i.default_val,
					buf);
		err = -1;
		goto err_unknown_device;
	}

	status_registers.act_ths.resume_val =
				status_registers.act_ths.default_val;
	status_registers.act_dur.resume_val =
				status_registers.act_dur.default_val;
	status_registers.int_gen_cfg_xl.resume_val =
				status_registers.int_gen_cfg_xl.default_val;
	status_registers.int_gen_ths_x_xl.resume_val =
				status_registers.int_gen_ths_x_xl.default_val;
	status_registers.int_gen_ths_y_xl.resume_val =
				status_registers.int_gen_ths_y_xl.default_val;
	status_registers.int_gen_ths_z_xl.resume_val =
				status_registers.int_gen_ths_z_xl.default_val;
	status_registers.int_gen_dur_xl.resume_val =
				status_registers.int_gen_dur_xl.default_val;
	status_registers.reference_g.resume_val =
				status_registers.reference_g.default_val;
	status_registers.int1_ctrl.resume_val =
				status_registers.int1_ctrl.default_val;
	status_registers.int2_ctrl.resume_val =
				status_registers.int2_ctrl.default_val;
	status_registers.ctrl_reg1_g.resume_val =
				status_registers.ctrl_reg1_g.default_val;
	status_registers.ctrl_reg2_g.resume_val =
				status_registers.ctrl_reg2_g.default_val;
	status_registers.ctrl_reg3_g.resume_val =
				status_registers.ctrl_reg3_g.default_val;
	status_registers.orient_cfg_g.resume_val =
				status_registers.orient_cfg_g.default_val;
	status_registers.ctrl_reg4.resume_val =
				status_registers.ctrl_reg4.default_val;
	status_registers.ctrl_reg5_xl.resume_val =
				status_registers.ctrl_reg5_xl.default_val;
	status_registers.ctrl_reg6_xl.resume_val =
				status_registers.ctrl_reg6_xl.default_val;
	status_registers.ctrl_reg7_xl.resume_val =
				status_registers.ctrl_reg7_xl.default_val;
	status_registers.ctrl_reg8.resume_val =
				status_registers.ctrl_reg8.default_val;
	status_registers.ctrl_reg9.resume_val =
				status_registers.ctrl_reg9.default_val;
	status_registers.ctrl_reg10.resume_val =
				status_registers.ctrl_reg10.default_val;
	status_registers.fifo_ctrl.resume_val =
				status_registers.fifo_ctrl.default_val;
	status_registers.int_gen_cfg_g.resume_val =
				status_registers.int_gen_cfg_g.default_val;
	status_registers.int_gen_ths_xh_g.resume_val =
				status_registers.int_gen_ths_xh_g.default_val;
	status_registers.int_gen_ths_xl_g.resume_val =
				status_registers.int_gen_ths_xl_g.default_val;
	status_registers.int_gen_ths_yh_g.resume_val =
				status_registers.int_gen_ths_yh_g.default_val;
	status_registers.int_gen_ths_yl_g.resume_val =
				status_registers.int_gen_ths_yl_g.default_val;
	status_registers.int_gen_ths_zh_g.resume_val =
				status_registers.int_gen_ths_zh_g.default_val;
	status_registers.int_gen_ths_zl_g.resume_val =
				status_registers.int_gen_ths_zl_g.default_val;
	status_registers.int_gen_dur_g.resume_val =
				status_registers.int_gen_dur_g.default_val;

#ifdef CONFIG_INPUT_LSM6DS0_LP
	atomic_set(&stat->low_power_state, 0);
#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL
	/** Disable Low Power mode at startup, device will be in normal mode */
	atomic_set(&stat->enabled_lp_mode, 0);
#else
	atomic_set(&stat->enabled_lp_mode, 1);
#endif
	stat->gyr_discard_samples = 0;
#endif
	stat->acc_discard_samples = 0;

	stat->hw_initialized = 1;
	dev_info(stat->dev, "%s: hw init done\n",
						LSM6DS0_ACC_GYR_DEV_NAME);

	return 0;

err_unknown_device:
err_firstread:
	stat->hw_working = 0;
	stat->hw_initialized = 0;

	return err;
}

static int32_t _lsm6ds0_acc_device_power_on(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	uint8_t buf[9] = { 0 };

	if (stat->pdata_acc->power_on) {
		err = stat->pdata_acc->power_on();
		if (err < 0) {
			dev_err(stat->dev,
				"accelerometer power_on failed: %d\n", err);
			return err;
		}
	}

 	buf[0] = status_registers.ctrl_reg4.resume_val;
 	buf[1] = status_registers.ctrl_reg5_xl.resume_val;
	buf[2] = status_registers.ctrl_reg6_xl.resume_val;
	buf[3] = status_registers.ctrl_reg7_xl.resume_val;
	buf[4] = status_registers.ctrl_reg8.resume_val;
	buf[5] = status_registers.ctrl_reg9.resume_val;
	buf[6] = status_registers.ctrl_reg10.resume_val;
	err = stat->tf->write(stat, status_registers.ctrl_reg4.address, 7, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen_cfg_xl.resume_val;
	buf[1] = status_registers.int_gen_ths_x_xl.resume_val;
	buf[2] = status_registers.int_gen_ths_y_xl.resume_val;
	buf[3] = status_registers.int_gen_ths_z_xl.resume_val;
	buf[4] = status_registers.int_gen_dur_xl.resume_val;
	err = stat->tf->write(stat, status_registers.int_gen_cfg_xl.address, 5, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.resume_val;
	buf[1] = status_registers.int2_ctrl.resume_val;
	err = stat->tf->write(stat, status_registers.int1_ctrl.address, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.fifo_ctrl.resume_val;
	err = stat->tf->write(stat, status_registers.fifo_ctrl.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg8.resume_val;
	buf[1] = status_registers.ctrl_reg9.resume_val;
	err = stat->tf->write(stat, status_registers.ctrl_reg8.address, 2, buf);
	if (err < 0)
		goto err_resume_state;

	return 0;

err_resume_state:
	dev_err(stat->dev, "accelerometer hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int32_t lsm6ds0_acc_device_power_on(struct lsm6ds0_status *stat)
{
	int32_t err = -1;

	err = _lsm6ds0_acc_device_power_on(stat);
	if (err < 0)
		atomic_set(&stat->enabled_acc, 0);
	else
		atomic_set(&stat->enabled_acc, 1);

	return err;
}

static int32_t _lsm6ds0_gyr_device_power_on(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	uint8_t buf[9] = { 0 };

	if (stat->pdata_gyr->power_on) {
		err = stat->pdata_gyr->power_on();
		if (err < 0) {
			dev_err(stat->dev,
				"gyroscope power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.act_ths.resume_val;
	err = stat->tf->write(stat, status_registers.act_ths.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.reference_g.resume_val;
	err = stat->tf->write(stat, status_registers.reference_g.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg1_g.resume_val;
	buf[1] = status_registers.ctrl_reg2_g.resume_val;
	buf[2] = status_registers.ctrl_reg3_g.resume_val;
	buf[3] = status_registers.orient_cfg_g.resume_val;
	err = stat->tf->write(stat, status_registers.ctrl_reg1_g.address, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg4.resume_val;
	err = stat->tf->write(stat, status_registers.ctrl_reg4.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen_cfg_g.resume_val;
	buf[1] = status_registers.int_gen_ths_xh_g.resume_val;
	buf[2] = status_registers.int_gen_ths_xl_g.resume_val;
	buf[3] = status_registers.int_gen_ths_yh_g.resume_val;
	buf[4] = status_registers.int_gen_ths_yl_g.resume_val;
	buf[5] = status_registers.int_gen_ths_zh_g.resume_val;
	buf[6] = status_registers.int_gen_ths_zl_g.resume_val;
	buf[7] = status_registers.int_gen_dur_g.resume_val;
	err = stat->tf->write(stat, status_registers.int_gen_cfg_g.address, 8, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.resume_val;
	buf[1] = status_registers.int2_ctrl.resume_val;
	err = stat->tf->write(stat, status_registers.int1_ctrl.address, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.fifo_ctrl.resume_val;
	err = stat->tf->write(stat, status_registers.fifo_ctrl.address, 1, buf);
	if (err < 0)
		goto err_resume_state;

	return 0;

err_resume_state:
	dev_err(stat->dev, "gyroscope hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);

	return err;
}

static int32_t lsm6ds0_gyr_device_power_on(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	
	err = _lsm6ds0_gyr_device_power_on(stat);
	if (err < 0)
		atomic_set(&stat->enabled_gyr, 0);
	else
		atomic_set(&stat->enabled_gyr, 1);

	return err;
}

static int32_t lsm6ds0_acc_update_fs_range(struct lsm6ds0_status *stat,
							uint8_t new_fs_range)
{
	int32_t sensitivity, err = -1;
	uint8_t val;

	switch (new_fs_range) {
	case LSM6DS0_ACC_FS_2G:
		sensitivity = SENSITIVITY_ACC_2G;
		break;
	case LSM6DS0_ACC_FS_4G:
		sensitivity = SENSITIVITY_ACC_4G;
		break;
	case LSM6DS0_ACC_FS_8G:
		sensitivity = SENSITIVITY_ACC_8G;
		break;
	default:
		dev_err(stat->dev, "invalid accelerometer "
				"fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	val = ((LSM6DS0_ACC_FS_MASK & new_fs_range) | ((~LSM6DS0_ACC_FS_MASK) &
				status_registers.ctrl_reg6_xl.resume_val));

	err = stat->tf->write(stat,  status_registers.ctrl_reg6_xl.address, 1, &val);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = val;

	mutex_lock(&stat->lock);
	stat->sensitivity_acc = sensitivity;
	mutex_unlock(&stat->lock);

	return err;

error:
	dev_err(stat->dev, "update accelerometer fs range failed "
			"0x%02x,0x%02x: %d\n", status_registers.ctrl_reg6_xl.address,
			val, err);
	return err;
}

static int32_t lsm6ds0_gyr_update_fs_range(struct lsm6ds0_status *stat,
							uint8_t new_fs_range)
{
	int32_t err = -1;
	uint8_t updated_val;
	u32 sensitivity;
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	int8_t k_fs;
#endif

	switch(new_fs_range) {
	case LSM6DS0_GYR_FS_245DPS:
		sensitivity = SENSITIVITY_GYR_250;
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
		k_fs = 8;
#endif
		break;
	case LSM6DS0_GYR_FS_500DPS:
		sensitivity = SENSITIVITY_GYR_500;
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
		k_fs = 4;
#endif
		break;
	case LSM6DS0_GYR_FS_2000DPS:
		sensitivity = SENSITIVITY_GYR_2000;
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
		k_fs = 1;
#endif
		break;
	default:
		dev_err(stat->dev, "invalid g range "
					"requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	err = stat->tf->read(stat, status_registers.ctrl_reg1_g.address, 1, &updated_val);
	if (err < 0)
		goto error;

	updated_val = ((LSM6DS0_GYR_FS_MASK & new_fs_range) |
					((~LSM6DS0_GYR_FS_MASK) & updated_val));

	err = stat->tf->write(stat, status_registers.ctrl_reg1_g.address, 1,
							&updated_val);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg1_g.resume_val = updated_val;

	mutex_lock(&stat->lock);
	stat->sensitivity_gyr = sensitivity;
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	stat->k_fs = k_fs;
#endif
	mutex_unlock(&stat->lock);

error:
	return err;
}

static int32_t lsm6ds0_acc_update_odr(struct lsm6ds0_status *stat,
					uint32_t poll_interval_us)
{
	int32_t err = -1;
	uint8_t buf;
	uint32_t i;

	for (i = ARRAY_SIZE(lsm6ds0_acc_odr_table) - 1; i >= 0; i--) {
		if (((uint32_t)lsm6ds0_acc_odr_table[i].cutoff_us <=
			poll_interval_us) || (i == 0))
			break;
	}

	if (atomic_read(&stat->enabled_acc)) {
		buf = LSM6DS0_ACC_ODR_MASK & lsm6ds0_acc_odr_table[i].value;
		buf |= (~LSM6DS0_ACC_ODR_MASK) &
				status_registers.ctrl_reg6_xl.resume_val;

		err = stat->tf->write(stat, status_registers.ctrl_reg6_xl.address, 1,
							  &buf);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg6_xl.resume_val = buf;
		mutex_lock(&stat->lock);
		stat->pdata_acc->poll_interval = poll_interval_us;
		stat->ktime_acc = ktime_set(0, US_TO_NS(poll_interval_us));
		mutex_unlock(&stat->lock);
		dev_info(stat->dev, "Acc Polling Rate (us)%d\n", stat->pdata_acc->poll_interval);
	}

	return err;

error:
	dev_err(stat->dev, "update accelerometer odr failed : %d\n", err);

	return err;
}

static int32_t lsm6ds0_gyr_update_odr(struct lsm6ds0_status *stat,
					uint32_t poll_interval_us)
{
	uint8_t buf;
	uint32_t val, i;
	int32_t err = -1;
#ifdef CONFIG_INPUT_LSM6DS0_LP
	uint8_t lp_mode;
#endif

	if (atomic_read(&stat->enabled_gyr)) {
		if (atomic_read(&stat->enabled_acc)) {
			val = min(poll_interval_us,
					stat->pdata_acc->poll_interval);
		} else {
			val = poll_interval_us;
		}

		for (i = ARRAY_SIZE(lsm6ds0_gyr_odr_table) - 1; i >= 0; i--) {
			if ((lsm6ds0_gyr_odr_table[i].cutoff_us <= val) ||
				(i == 0))
				break;
		}

		/* Set ODR value */
		buf = LSM6DS0_GYR_ODR_MASK & lsm6ds0_gyr_odr_table[i].value;
		buf |= (~LSM6DS0_GYR_ODR_MASK) &
				status_registers.ctrl_reg1_g.resume_val;

		err = stat->tf->write(stat, status_registers.ctrl_reg1_g.address, 1,
							  &buf);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg1_g.resume_val = buf;

#ifdef CONFIG_INPUT_LSM6DS0_LP
		if (lsm6ds0_gyr_odr_table[i].cutoff_us < LSM6DS0_GYR_LP_TH_US)
			lp_mode = 1;
		else
			lp_mode = 0;

		/* Discard samples only if switch between normal and low-power
		 * mode
		 */
		if (lp_mode != (uint8_t)atomic_read(&stat->low_power_state) &&
				atomic_read(&stat->enabled_lp_mode) == 1) {
			atomic_set(&stat->low_power_state, lp_mode);

			mutex_lock(&stat->lock);
			stat->gyr_discard_samples =
			lsm6ds0_gyr_turn_on_time[i].disc_list[LSM6DS0_GYR_LPF1].count;
		} else {
			mutex_lock(&stat->lock);
			stat->gyr_discard_samples = 0;
		}
		mutex_unlock(&stat->lock);
#endif
		/* Enable all axes */
		buf = CTRL_REG4_ALL_AXES_EN |
			  status_registers.ctrl_reg4.resume_val;

		err = stat->tf->write(stat, status_registers.ctrl_reg4.address, 1, &buf);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg4.resume_val = buf;

		mutex_lock(&stat->lock);
		stat->ktime_gyr = ktime_set(0, US_TO_NS(val));
		stat->pdata_gyr->poll_interval = val;
		mutex_unlock(&stat->lock);
	}
	return err;
error:
	dev_err(stat->dev, "update accelerometer odr failed: %d\n", err);

	return err;
}

static int32_t lsm6ds0_acc_update_filter(struct lsm6ds0_status *stat,
							uint8_t new_bandwidth)
{
	int32_t err = -1;
	uint8_t updated_val;

	switch (new_bandwidth) {
	case LSM6DS0_ACC_BW_50:
		break;
	case LSM6DS0_ACC_BW_100:
		break;
	case LSM6DS0_ACC_BW_200:
		break;
	case LSM6DS0_ACC_BW_400:
		break;
	default:
		dev_err(stat->dev, "invalid accelerometer "
			"update bandwidth requested: %u\n", new_bandwidth);
		return -EINVAL;
	}

	err = stat->tf->read(stat, status_registers.ctrl_reg6_xl.address, 1,
						 &updated_val);
	if (err < 0)
		goto error;

	updated_val = ((LSM6DS0_ACC_BW_MASK & new_bandwidth) |
					((~LSM6DS0_ACC_BW_MASK) & updated_val));

	err = stat->tf->write(stat, status_registers.ctrl_reg6_xl.address, 1,
						  &updated_val);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = updated_val;

	return err;

error:
	dev_err(stat->dev, "update accelerometer fs range failed : %d\n", err);
	return err;
}


#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL_LP
static int32_t lsm6ds0_acc_change_pm_state(struct lsm6ds0_status *stat,
							uint8_t new_pm_state)
{
	uint8_t val;
	int32_t err = -1, new_state = (new_pm_state != 0) ? 1 : 0;

	if (atomic_read(&stat->enabled_lp_mode) != new_state) {
		val = ((CTRL_REG3_G_LP_MODE_MASK & new_state) |
			((~CTRL_REG3_G_LP_MODE_MASK) &
			status_registers.ctrl_reg3_g.resume_val));

		err = stat->tf->write(stat, status_registers.ctrl_reg3_g.address, 1,
							  &val);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg3_g.resume_val = val;
		atomic_set(&stat->enabled_lp_mode, new_state);
	}
	return err;
error:
	dev_err(stat->dev, "enable pm bit failed : %d\n", err);
	return err;
}
#endif

static int32_t lsm6ds0_acc_enable(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	uint8_t j, bw;

	if (!atomic_cmpxchg(&stat->enabled_acc, 0, 1)) {
		err = lsm6ds0_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_acc, 0);
			dev_err(stat->dev, "enable accelerometer "
				      "failed");
			return err;
		}

		for (j = ARRAY_SIZE(lsm6ds0_acc_turn_on_time) - 1; j >= 0; j--) {
			if ((lsm6ds0_acc_turn_on_time[j].odr_us <=
			  stat->pdata_acc->poll_interval) || (j == 0))
				break;
		}
		bw = stat->pdata_acc->aa_filter_bandwidth;

		stat->acc_discard_samples =
			lsm6ds0_acc_turn_on_time[j].disc_list[bw].count;
		
		hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc,
							HRTIMER_MODE_REL);
	}

	return 0;
}

static int32_t lsm6ds0_gyr_enable(struct lsm6ds0_status *stat)
{
	int32_t err = -1;

	if (!atomic_cmpxchg(&stat->enabled_gyr, 0, 1)) {

		err = lsm6ds0_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_gyr, 0);
			return err;
		}

		hrtimer_start(&(stat->hr_timer_gyr), stat->ktime_gyr,
							HRTIMER_MODE_REL);
	}
	return 0;
}

#ifdef LSM6DS0_EN_ON_OPEN
int32_t lsm6ds0_acc_input_open(struct input_dev *input)
{
	struct lsm6ds0_status *stat = input_get_drvdata(input);

	return lsm6ds0_acc_enable(stat);
}

void lsm6ds0_acc_input_close(struct input_dev *dev)
{
	struct lsm6ds0_status *stat = input_get_drvdata(dev);

	lsm6ds0_acc_disable(stat);
}
#endif

int32_t lsm6ds0_gyr_input_open(struct input_dev *input)
{
	struct lsm6ds0_status *stat = input_get_drvdata(input);

	return lsm6ds0_gyr_enable(stat);
}

void lsm6ds0_gyr_input_close(struct input_dev *dev)
{
	struct lsm6ds0_status *stat = input_get_drvdata(dev);

	lsm6ds0_gyr_disable(stat);
}

static int32_t lsm6ds0_temp_get_data(struct lsm6ds0_status *stat, int32_t *data)
{
	int32_t err = -1;
	uint8_t temp_data[2] = { 0 };

	err = stat->tf->read(stat, OUT_TEMP_L, 2, temp_data);
	if (err < 0)
		return err;

	(*data) = (int32_t)((int16_t)((temp_data[1] << 8) | (temp_data[0])));

	return err;
}

static int32_t lsm6ds0_acc_get_data(struct lsm6ds0_status *stat, int32_t *xyz)
{
	int32_t i, err = -1, hw_d[3] = { 0 };
	uint8_t acc_data[6];

	err = stat->tf->read(stat, OUT_X_L_XL, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((int32_t)( (int16_t)((acc_data[1] << 8) | (acc_data[0]))));
	hw_d[1] = ((int32_t)( (int16_t)((acc_data[3] << 8) | (acc_data[2]))));
	hw_d[2] = ((int32_t)( (int16_t)((acc_data[5] << 8) | (acc_data[4]))));

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

static int32_t lsm6ds0_gyr_get_data(struct lsm6ds0_status *stat, int32_t *xyz)
{
	int32_t i, err = 1, hw_d[3] = { 0 };
	uint8_t gyro_data[6];
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	int32_t temp, abs_temp;
#endif

	err = stat->tf->read(stat, OUT_X_L_G, 6, gyro_data);

	if (err < 0)
		return err;

	hw_d[0] = (int32_t) ((int16_t)((gyro_data[1]) << 8) | gyro_data[0]);
	hw_d[1] = (int32_t) ((int16_t)((gyro_data[3]) << 8) | gyro_data[2]);
	hw_d[2] = (int32_t) ((int16_t)((gyro_data[5]) << 8) | gyro_data[4]);

	mutex_lock(&stat->lock);
#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	err = lsm6ds0_temp_get_data(stat, &temp);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}

	if (temp >= stat->compensation_temp) {
		abs_temp = temp - stat->compensation_temp;
	} else {
		abs_temp = stat->compensation_temp - temp;
	}

	if (abs_temp > SENSITIVITY_TEMP)
		stat->compensation_temp = temp;

	hw_d[0] -= 10 * stat->compensation_temp *
			((int32_t)(stat->k_fs * stat->k_coeff[1])) / 896;
	hw_d[1] += 10 * stat->compensation_temp *
			((int32_t)(stat->k_fs * stat->k_coeff[0])) / 896;
	hw_d[2] -= 10 * stat->compensation_temp *
			((int32_t)(stat->k_fs * stat->k_coeff[2])) / 896;
#endif

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

static void lsm6ds0_acc_report_values(struct lsm6ds0_status *stat,
								int32_t *xyz)
{
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(stat->input_dev_acc);
}

static void lsm6ds0_gyr_report_values(struct lsm6ds0_status *stat,
								int32_t *xyz)
{
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(stat->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(stat->input_dev_gyr);
}

static int32_t lsm6ds0_acc_input_init(struct lsm6ds0_status *stat)
{
	int32_t err = -1;

	mutex_lock(&stat->lock);
	stat->input_dev_acc = input_allocate_device();
	if (!stat->input_dev_acc) {
		err = -ENOMEM;
		dev_err(stat->dev, "accelerometer "
					"input device allocation failed\n");
		mutex_unlock(&stat->lock);
		return err;
	}

#ifdef LSM6DS0_EN_ON_OPEN
	stat->input_dev_acc->open = lsm6ds0_acc_input_open;
	stat->input_dev_acc->close = lsm6ds0_acc_input_close;
#endif
	stat->input_dev_acc->name = LSM6DS0_ACC_DEV_NAME;
	stat->input_dev_acc->id.bustype = stat->bustype;
	stat->input_dev_acc->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev_acc, stat);

	__set_bit(INPUT_EVENT_TYPE, stat->input_dev_acc->evbit );
	__set_bit(INPUT_EVENT_X, stat->input_dev_acc->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev_acc->mscbit);
	__set_bit(INPUT_EVENT_Z, stat->input_dev_acc->mscbit);

	err = input_register_device(stat->input_dev_acc);
	if (err) {
		dev_err(stat->dev,
			"unable to register accelerometer input device %s\n",
				stat->input_dev_acc->name);
		input_free_device(stat->input_dev_acc);
	}
	mutex_unlock(&stat->lock);

	return err;
}

static int32_t lsm6ds0_gyr_input_init(struct lsm6ds0_status *stat)
{
	int32_t err = -1;

	dev_dbg(stat->dev, "%s\n", __func__);

	mutex_lock(&stat->lock);
	stat->input_dev_gyr = input_allocate_device();
	if (!stat->input_dev_gyr) {
		err = -ENOMEM;
		dev_err(stat->dev,
			"input device allocation failed\n");
		mutex_unlock(&stat->lock);
		return err;
	}

#ifdef LSM6DS0_EN_ON_OPEN
	stat->input_dev_gyr->open = lsm6ds0_gyr_input_open;
	stat->input_dev_gyr->close = lsm6ds0_gyr_input_close;
#endif
	stat->input_dev_gyr->name = LSM6DS0_GYR_DEV_NAME;
	stat->input_dev_gyr->id.bustype = stat->bustype;
	stat->input_dev_gyr->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev_gyr, stat);

	__set_bit(INPUT_EVENT_TYPE, stat->input_dev_gyr->evbit );
	__set_bit(INPUT_EVENT_X, stat->input_dev_gyr->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev_gyr->mscbit);
	__set_bit(INPUT_EVENT_Z, stat->input_dev_gyr->mscbit);

	err = input_register_device(stat->input_dev_gyr);
	if (err) {
		dev_err(stat->dev,
			"unable to register input device %s\n",
			stat->input_dev_gyr->name);
		input_free_device(stat->input_dev_gyr);
	}
	mutex_unlock(&stat->lock);

	return err;
}
static void lsm6ds0_input_cleanup(struct lsm6ds0_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);

	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

static ssize_t set_polling_rate_acc(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned long)max((unsigned int)MS_TO_US(interval_ms),
						sdata->pdata_acc->min_interval);

	lsm6ds0_acc_update_odr(sdata, interval_us);

	return count;
}

static ssize_t get_polling_rate_acc(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	uint32_t val = 0;
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);

	mutex_lock(&sdata->lock);
	val = sdata->pdata_acc->poll_interval;
	mutex_unlock(&sdata->lock);

	return sprintf(buf, "%u\n", US_TO_MS(val));
}

static ssize_t get_enable_acc(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);

	int32_t val = (int)atomic_read(&sdata->enabled_acc);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_enable_acc(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6ds0_acc_enable(sdata);
	else
		lsm6ds0_acc_disable(sdata);

	return count;
}

static ssize_t get_range_acc(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	uint8_t val;
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	int32_t range = 2;

	mutex_lock(&sdata->lock);
	val = sdata->pdata_acc->fs_range;
	mutex_unlock(&sdata->lock);

	switch (val) {
	case LSM6DS0_ACC_FS_2G:
		range = 2;
		break;
	case LSM6DS0_ACC_FS_4G:
		range = 4;
		break;
	case LSM6DS0_ACC_FS_8G:
		range = 8;
		break;
	}
	return sprintf(buf, "%d\n", range);
}

static ssize_t set_range_acc(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM6DS0_ACC_FS_2G;
		break;
	case 4:
		range = LSM6DS0_ACC_FS_4G;
		break;
	case 8:
		range = LSM6DS0_ACC_FS_8G;
		break;
	default:
		dev_err(sdata->dev, "accelerometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = lsm6ds0_acc_update_fs_range(sdata, range);
	if (err < 0)
		return err;

	mutex_lock(&sdata->lock);
	sdata->pdata_acc->fs_range = range;
	mutex_unlock(&sdata->lock);

	dev_info(sdata->dev, "accelerometer range set to:"
							" %lu g\n", val);

	return size;
}

static ssize_t get_aa_filter(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	uint8_t val;
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	int32_t frequency = FILTER_400;

	mutex_lock(&sdata->lock);
	val = sdata->pdata_acc->aa_filter_bandwidth;
	mutex_unlock(&sdata->lock);

	switch (val) {
	case LSM6DS0_ACC_BW_50:
		frequency = FILTER_50;
		break;
	case LSM6DS0_ACC_BW_100:
		frequency = FILTER_100;
		break;
	case LSM6DS0_ACC_BW_200:
		frequency = FILTER_200;
		break;
	case LSM6DS0_ACC_BW_400:
		frequency = FILTER_400;
		break;
	}

	return sprintf(buf, "%d\n", frequency);
}

static ssize_t set_aa_filter(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t frequency;
	int32_t err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case FILTER_50:
		frequency = LSM6DS0_ACC_BW_50;
		break;
	case FILTER_100:
		frequency = LSM6DS0_ACC_BW_100;
		break;
	case FILTER_200:
		frequency = LSM6DS0_ACC_BW_200;
		break;
	case FILTER_400:
		frequency = LSM6DS0_ACC_BW_400;
		break;
	default:
		dev_err(sdata->dev, "accelerometer invalid filter "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = lsm6ds0_acc_update_filter(sdata, frequency);
	if (err < 0)
		return err;

	mutex_lock(&sdata->lock);
	sdata->pdata_acc->aa_filter_bandwidth = frequency;
	mutex_unlock(&sdata->lock);

	dev_info(sdata->dev, "accelerometer anti-aliasing filter "
					"set to: %lu Hz\n", val);

	return size;
}

static ssize_t get_temp(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	int32_t temp_decimal, temp_hw = 0, err = -1;
	uint32_t temp_float;

	err = lsm6ds0_temp_get_data(sdata, &temp_hw);
	if (err < 0)
		return sprintf(buf, "null\n");

	temp_decimal = (int32_t)(temp_hw / SENSITIVITY_TEMP) + OFFSET_TEMP;
	temp_float = (((uint32_t)temp_hw) % SENSITIVITY_TEMP);

	return sprintf(buf, "%d.%d\n", temp_decimal, temp_float);
}

static ssize_t get_polling_rate_gyr(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	uint32_t val;
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);

	mutex_lock(&sdata->lock);
	val = sdata->pdata_gyr->poll_interval;
	mutex_unlock(&sdata->lock);

	return sprintf(buf, "%d\n", US_TO_MS(val));
}

static ssize_t set_polling_rate_gyr(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned int)max((unsigned int)MS_TO_US(interval_ms),
					sdata->pdata_gyr->min_interval);

	lsm6ds0_gyr_update_odr(sdata, interval_us);

	return count;
}

static ssize_t get_enable_gyr(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	int32_t val = atomic_read(&sdata->enabled_gyr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_enable_gyr(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6ds0_gyr_enable(sdata);
	else
		lsm6ds0_gyr_disable(sdata);

	return count;
}

static ssize_t get_range_gyr(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	int32_t range = 0;
	uint8_t val;

	mutex_lock(&sdata->lock);
	val = sdata->pdata_gyr->fs_range;
	switch (val) {
	case LSM6DS0_GYR_FS_245DPS:
		range = RANGE_245DPS;
		break;
	case LSM6DS0_GYR_FS_500DPS:
		range = RANGE_500DPS;
		break;
	case LSM6DS0_GYR_FS_2000DPS:
		range = RANGE_2000DPS;
		break;
	}
	mutex_unlock(&sdata->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t set_range_gyr(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err = -1;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 245:
		range = LSM6DS0_GYR_FS_245DPS;
		break;
	case 500:
		range = LSM6DS0_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM6DS0_GYR_FS_2000DPS;
		break;
	default:
		dev_err(sdata->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}

	err = lsm6ds0_gyr_update_fs_range(sdata, range);
	if (err >= 0) {
		mutex_lock(&sdata->lock);
		sdata->pdata_gyr->fs_range = range;
		mutex_unlock(&sdata->lock);
	}

	dev_info(sdata->dev, "range set to: %lu dps\n", val);

	return count;
}

#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL_LP
static ssize_t get_pmode(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	uint8_t val;

	val = atomic_read(&sdata->enabled_lp_mode);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_pmode(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct lsm6ds0_status *sdata = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	lsm6ds0_acc_change_pm_state(sdata, val);

	return count;
}
#endif

LSM6DS0_ATTTR(enable_acc, enable_device, 0664, get_enable_acc, set_enable_acc);
LSM6DS0_ATTTR(enable_gyr, enable_device, 0664, get_enable_gyr, set_enable_gyr);
LSM6DS0_ATTTR(pollrate_acc, pollrate_ms, 0664, get_polling_rate_acc, set_polling_rate_acc);
LSM6DS0_ATTTR(pollrate_gyr, pollrate_ms, 0664, get_polling_rate_gyr, set_polling_rate_gyr);
LSM6DS0_ATTTR(range_acc, range, 0664, get_range_acc, set_range_acc);
LSM6DS0_ATTTR(range_gyr, range, 0664, get_range_gyr, set_range_gyr);
LSM6DS0_ATTTR(aa_freq, anti_aliasing_frequency, 0664, get_aa_filter, set_aa_filter);
#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL_LP
LSM6DS0_ATTTR(pmode, pmode, 0664, get_pmode, set_pmode);
#endif
LSM6DS0_ATTTR(temperature, temperature, 0444, get_temp, NULL);

static struct attribute *attributes_acc[] = {
	&lsm6ds0_attr_enable_acc.attr,
	&lsm6ds0_attr_pollrate_acc.attr,
	&lsm6ds0_attr_range_acc.attr,
	&lsm6ds0_attr_aa_freq.attr,
	&lsm6ds0_attr_temperature.attr,
#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL_LP
	&lsm6ds0_attr_pmode.attr,
#endif
	NULL,
};

static struct attribute *attributes_gyr[] = {
	&lsm6ds0_attr_enable_gyr.attr,
	&lsm6ds0_attr_pollrate_gyr.attr,
	&lsm6ds0_attr_range_gyr.attr,
	NULL,
};

static struct attribute_group attr_group_acc = {
	.attrs = attributes_acc,
};

static struct attribute_group attr_group_gyr = {
	.attrs = attributes_gyr,
};

static int32_t create_sysfs_interfaces(struct lsm6ds0_status *stat)
{
	int32_t err = -1;

	err = sysfs_create_group(&stat->input_dev_acc->dev.kobj, &attr_group_acc);
	if (err)
		return err;

	return sysfs_create_group(&stat->input_dev_gyr->dev.kobj, &attr_group_gyr);
}

static void remove_sysfs_interfaces(struct lsm6ds0_status *stat)
{
	sysfs_remove_group(&stat->input_dev_acc->dev.kobj, &attr_group_acc);
	sysfs_remove_group(&stat->input_dev_gyr->dev.kobj, &attr_group_gyr);
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
	struct lsm6ds0_status *stat;
	int32_t xyz[3] = { 0 }, err = -1;

	stat = container_of((struct work_struct *)input_work_acc,
			struct lsm6ds0_status, input_work_acc);

	err = lsm6ds0_acc_get_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get accelerometer data failed\n");
	else {
		mutex_lock(&stat->lock);
		if (stat->acc_discard_samples > 0) {
			stat->acc_discard_samples--;
			mutex_unlock(&stat->lock);
		} else {
			mutex_unlock(&stat->lock);
			lsm6ds0_acc_report_values(stat, xyz);
		}
	}

	hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc, HRTIMER_MODE_REL);
}

static void poll_function_work_gyr(struct work_struct *input_work_gyr)
{
	struct lsm6ds0_status *stat;
	int32_t xyz[3] = { 0 }, err = -1;

	stat = container_of((struct work_struct *)input_work_gyr,
			struct lsm6ds0_status, input_work_gyr);

	err = lsm6ds0_gyr_get_data(stat, xyz);
	if (err < 0)
		dev_err(stat->dev, "get gyroscope data failed.\n");
	else {
#ifdef CONFIG_INPUT_LSM6DS0_LP
		mutex_lock(&stat->lock);
		if (stat->gyr_discard_samples > 0) {
			stat->gyr_discard_samples--;
			mutex_unlock(&stat->lock);
		} else {
			mutex_unlock(&stat->lock);
			lsm6ds0_gyr_report_values(stat, xyz);
		}
#else
		lsm6ds0_gyr_report_values(stat, xyz);
#endif
	}

	hrtimer_start(&stat->hr_timer_gyr, stat->ktime_gyr, HRTIMER_MODE_REL);
}

#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
int32_t lsm6ds0_hw_comp_read_k(struct lsm6ds0_status *stat, uint8_t reg,
								int8_t *val)
{
	int32_t err = -1;

	/* Read hardcoded factor */
	(*val) = reg;
	err = stat->tf->read(stat, reg, 1, val);
	if (err < 0) {
		return err;
	}

	if ((*val) & (0x20)) {
		(*val) |= (0xE0);
		(*val) &= (0xFF);
	} else {
		(*val) &= (0x3F);
	}
	return err;
}

int32_t lsm6ds0_hw_compensation_disable(struct lsm6ds0_status *stat,
								uint8_t reg)
{
	int32_t err = -1;
	uint8_t tmp = 0;

	switch(reg) {
	case COMP_X_FACTOR_ADDR:
		tmp = stat->k_coeff[0];
		break;

	case COMP_Y_FACTOR_ADDR:
		tmp = stat->k_coeff[1];
		break;

	case COMP_Z_FACTOR_ADDR:
		tmp = stat->k_coeff[2];
		break;
	}

	/* Disable HW compensation */
	err = stat->tf->write(stat, reg, 1, &tmp);

	return err;
}

static int32_t lsm6ds0_gyr_temp_compensation_init(struct lsm6ds0_status *stat)
{
	int32_t err = -1;
	int8_t k_coeff[3] = { 0 };

	/* reset coefficients */
	mutex_lock(&stat->lock);
	stat->k_coeff[0] = 0;
	stat->k_coeff[1] = 0;
	stat->k_coeff[2] = 0;
	mutex_unlock(&stat->lock);

	err = lsm6ds0_hw_comp_read_k(stat, COMP_X_FACTOR_ADDR, &k_coeff[0]);
	if (err < 0) {
		return err;
	}

	err = lsm6ds0_hw_comp_read_k(stat, COMP_Y_FACTOR_ADDR, &k_coeff[1]);
	if (err < 0) {
		return err;
	}

	err = lsm6ds0_hw_comp_read_k(stat, COMP_Z_FACTOR_ADDR, &k_coeff[2]);
	if (err < 0) {
		return err;
	}

	mutex_lock(&stat->lock);
	err = lsm6ds0_hw_compensation_disable(stat, COMP_X_FACTOR_ADDR);
	if (err < 0) {
		goto err_dis_comp;
	}
	stat->k_coeff[0] = k_coeff[0];

	err = lsm6ds0_hw_compensation_disable(stat, COMP_Y_FACTOR_ADDR);
	if (err < 0) {
		goto err_dis_comp;
	}
	stat->k_coeff[1] = k_coeff[1];

	err = lsm6ds0_hw_compensation_disable(stat, COMP_Z_FACTOR_ADDR);
	if (err < 0) {
		goto err_dis_comp;
	}
	stat->k_coeff[2] = k_coeff[2];

err_dis_comp:
	mutex_unlock(&stat->lock);
	return err;
}
#endif

#ifdef CONFIG_OF
static int32_t lsm6ds0_parse_dt(struct lsm6ds0_status *stat,
                                        struct device* dev)
{
	struct device_node *dn;
	uint8_t i, j;
	uint32_t val;
	uint32_t vect[9] = { 0 };

	mutex_lock(&stat->lock);
	dn = dev->of_node;
	stat->pdata_main->of_node = dn;
	stat->pdata_main->gpio_int1 = of_get_gpio(dn, 0);
	if (!gpio_is_valid(stat->pdata_main->gpio_int1)) {
		dev_err(dev, "failed to get gpio_int1\n");
			stat->pdata_main->gpio_int1 = LSM6DS0_INT1_GPIO_DEF;
	}

	stat->pdata_main->gpio_int2 = of_get_gpio(dn, 1);
	if (!gpio_is_valid(stat->pdata_main->gpio_int2)) {
		dev_err(dev, "failed to get gpio_int2\n");
			stat->pdata_main->gpio_int2 = LSM6DS0_INT2_GPIO_DEF;
	}

	if (of_property_read_u32_array(dn, "rot-matrix", vect,
		      ARRAY_SIZE(vect)) >= 0) {
		for (j = 0; j < 3; j++) {
			for (i = 0; i < 3; i++) {
				stat->pdata_main->rot_matrix[i][j] = vect[3 * j + i];
			}
		}
	} else {
		for (j = 0; j < 3; j++) {
			for (i = 0; i < 3; i++) {
				stat->pdata_main->rot_matrix[i][j] =
		default_lsm6ds0_main_platform_data.rot_matrix[i][j];
			}
		}
	}

	if (!of_property_read_u32(dn, "g-poll-interval", &val)) {
		stat->pdata_gyr->poll_interval = MS_TO_US(val);
	} else {
		stat->pdata_gyr->poll_interval =
			LSM6DS0_GYR_POLL_INTERVAL_DEF;
	}

	if (!of_property_read_u32(dn, "g-min-interval", &val)) {
		stat->pdata_gyr->min_interval = val;
	} else {
		stat->pdata_gyr->min_interval =
			LSM6DS0_GYR_MIN_POLL_PERIOD_US;
	}
	
	if (!of_property_read_u32(dn, "g-fs-range", &val)) {
		stat->pdata_gyr->fs_range = val;
	} else {
	stat->pdata_gyr->fs_range = LSM6DS0_GYR_FS_245DPS;
	}

	if (!of_property_read_u32(dn, "x-poll-interval", &val)) {
		stat->pdata_acc->poll_interval = MS_TO_US(val);
	} else {
		stat->pdata_acc->poll_interval =
			LSM6DS0_ACC_MIN_POLL_PERIOD_US;
	}

	if (!of_property_read_u32(dn, "x-min-interval", &val)) {
		stat->pdata_acc->min_interval = MS_TO_US(val);
	} else {
		stat->pdata_acc->min_interval =
			LSM6DS0_ACC_MIN_POLL_PERIOD_US;
	}

	if (!of_property_read_u32(dn, "x-fs-range", &val)) {
		stat->pdata_acc->fs_range = val;
	} else {
		stat->pdata_acc->fs_range = LSM6DS0_ACC_FS_2G;
	}

	if (!of_property_read_u32(dn, "aa-filter-bw", &val)) {
		stat->pdata_acc->aa_filter_bandwidth = val;
	} else {
		stat->pdata_acc->aa_filter_bandwidth = LSM6DS0_ACC_BW_400;
	}
	mutex_unlock(&stat->lock);

	return 0;
}
#endif

int32_t lsm6ds0_common_probe(struct lsm6ds0_status *stat)
{
	int32_t err = -1;

	dev_info(stat->dev, "probe start.\n");

	mutex_init(&stat->lock);
	mutex_init(&stat->tb.buf_lock);
	if (!lsm6ds0_workqueue)
		lsm6ds0_workqueue = create_workqueue("lsm6ds0_workqueue");

	hrtimer_init(&stat->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_acc.function = &poll_function_read_acc;
	hrtimer_init(&stat->hr_timer_gyr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_gyr.function = &poll_function_read_gyr;

	stat->pdata_main = kzalloc(sizeof(*stat->pdata_main), GFP_KERNEL);
	stat->pdata_acc = kzalloc(sizeof(*stat->pdata_acc), GFP_KERNEL);
	stat->pdata_gyr = kzalloc(sizeof(*stat->pdata_gyr), GFP_KERNEL);

	if ((stat->pdata_main == NULL) || (stat->pdata_acc == NULL) ||
						  (stat->pdata_gyr == NULL)){
		err = -ENOMEM;
		dev_err(stat->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_memory_alloc;
	}

	stat->pdata_main->pdata_acc = stat->pdata_acc;
	stat->pdata_main->pdata_gyr = stat->pdata_gyr;

#ifdef CONFIG_OF
	lsm6ds0_parse_dt(stat, stat->dev);
#else
	if (stat->dev->platform_data == NULL) {
		memcpy(stat->pdata_main,
				&default_lsm6ds0_main_platform_data,
				sizeof(*stat->pdata_main));
		memcpy(stat->pdata_acc, &default_lsm6ds0_acc_pdata,
						sizeof(*stat->pdata_acc));
		memcpy(stat->pdata_gyr, &default_lsm6ds0_gyr_pdata,
						sizeof(*stat->pdata_gyr));
		dev_info(stat->dev, "using default plaform_data for "
					"accelerometer and gyroscope\n");
	} else {
		struct lsm6ds0_main_platform_data *platform_data;
		platform_data = stat->dev->platform_data;

		if (platform_data == NULL) {
			memcpy(stat->pdata_main,
				&default_lsm6ds0_main_platform_data,
				sizeof(*stat->pdata_main));
			dev_info(stat->dev, "using default plaform_data for "
							"accelerometer\n");
		} else {
			memcpy(stat->pdata_main, platform_data,
						sizeof(*stat->pdata_acc));
		}

		if (platform_data->pdata_acc == NULL) {
			memcpy(stat->pdata_acc, &default_lsm6ds0_acc_pdata,
						sizeof(*stat->pdata_acc));
			dev_info(stat->dev, "using default plaform_data for "
							"accelerometer\n");
		} else {
			memcpy(stat->pdata_acc, platform_data->pdata_acc,
						sizeof(*stat->pdata_acc));
		}

		if (platform_data->pdata_gyr == NULL) {
			memcpy(stat->pdata_gyr, &default_lsm6ds0_gyr_pdata,
						sizeof(*stat->pdata_gyr));
			dev_info(stat->dev, "using default plaform_data for "
							"gyroscope\n");
		} else {
			memcpy(stat->pdata_gyr, platform_data->pdata_gyr,
						sizeof(*stat->pdata_gyr));
		}
	}
#endif

	err = lsm6ds0_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data for "
							"accelerometer \n");
		goto exit_kfree_pdata;
	}

	err = lsm6ds0_gyr_validate_pdata(stat);
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

	err = lsm6ds0_hw_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm6ds0_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "accelerometer power on failed: "
								"%d\n", err);
		goto err_pdata_init;
	}

	err = lsm6ds0_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "gyroscope power on failed: "
								"%d\n", err);
		goto err_pdata_init;
	}

	err = lsm6ds0_acc_update_fs_range(stat, stat->pdata_acc->fs_range);
	if (err < 0) {
		dev_err(stat->dev, "update accelerometer full scale range "
								"failed\n");
		goto  err_power_off_acc;
	}

	err = lsm6ds0_gyr_update_fs_range(stat, stat->pdata_gyr->fs_range);
	if (err < 0) {
		dev_err(stat->dev, "update gyroscope full scale range "
								"failed\n");
		goto  err_power_off_gyr;
	}

	err = lsm6ds0_acc_update_odr(stat, stat->pdata_acc->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update accelerometer ODR failed\n");
		goto  err_power_off;
	}

	err = lsm6ds0_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update gyroscope ODR failed\n");
		goto  err_power_off;
	}

	err = lsm6ds0_acc_update_filter(stat,
					stat->pdata_acc->aa_filter_bandwidth);
	if (err < 0) {
		dev_err(stat->dev, "update accelerometer filter "
								"failed\n");
		goto  err_power_off;
	}

#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	err = lsm6ds0_gyr_temp_compensation_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "sw temperature compensation init "
								"failed\n");
		goto  err_power_off;
	}
#endif

	err = lsm6ds0_acc_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "accelerometer input init failed\n");
		goto err_power_off;
	}

	err = lsm6ds0_gyr_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "gyroscope input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(stat);
	if (err < 0) {
		dev_err(stat->dev, "device %s sysfs register failed\n",
			LSM6DS0_ACC_GYR_DEV_NAME);
		goto err_input_cleanup;
	}

	lsm6ds0_acc_device_power_off(stat);
	lsm6ds0_gyr_device_power_off(stat);
	INIT_WORK(&stat->input_work_acc, poll_function_work_acc);
	INIT_WORK(&stat->input_work_gyr, poll_function_work_gyr);
	dev_info(stat->dev, "%s: probed\n", LSM6DS0_ACC_GYR_DEV_NAME);

	return 0;

err_input_cleanup:
	lsm6ds0_input_cleanup(stat);

err_power_off:
err_power_off_gyr:
	lsm6ds0_gyr_device_power_off(stat);

err_power_off_acc:
	lsm6ds0_acc_device_power_off(stat);

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
	if (lsm6ds0_workqueue) {
		flush_workqueue(lsm6ds0_workqueue);
		destroy_workqueue(lsm6ds0_workqueue);
		lsm6ds0_workqueue = NULL;
	}

	dev_err(stat->dev,"%s: Driver Init failed\n",
						LSM6DS0_ACC_GYR_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lsm6ds0_common_probe);

int32_t lsm6ds0_common_remove(struct lsm6ds0_status *stat)
{
	remove_sysfs_interfaces(stat);
	if (atomic_read(&stat->enabled_gyr)) {
		lsm6ds0_gyr_disable(stat);
		lsm6ds0_gyr_input_cleanup(stat);

		if (stat->pdata_gyr->exit)
			stat->pdata_gyr->exit();
	}

	lsm6ds0_acc_disable(stat);
	lsm6ds0_acc_input_cleanup(stat);

	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();

	if(lsm6ds0_workqueue) {
		flush_workqueue(lsm6ds0_workqueue);
		destroy_workqueue(lsm6ds0_workqueue);
		lsm6ds0_workqueue = NULL;
	}

	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_main);

	return 0;
}
EXPORT_SYMBOL(lsm6ds0_common_remove);

#ifdef CONFIG_PM
int32_t lsm6ds0_common_suspend(struct lsm6ds0_status *stat)
{
	if (atomic_read(&stat->enabled_gyr) > 0) {
		cancel_work_sync(&stat->input_work_gyr);
		hrtimer_cancel(&stat->hr_timer_gyr);
		_lsm6ds0_gyr_device_power_off(stat);
	}

	if (atomic_read(&stat->enabled_acc) > 0) {
		cancel_work_sync(&stat->input_work_acc);
		hrtimer_cancel(&stat->hr_timer_acc);
		_lsm6ds0_acc_device_power_off(stat);
	}

	return 0;
}
EXPORT_SYMBOL(lsm6ds0_common_suspend);

int32_t lsm6ds0_common_resume(struct lsm6ds0_status *stat)
{
	if (atomic_read(&stat->enabled_acc) > 0) {
		_lsm6ds0_acc_device_power_on(stat);
		hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc,
							HRTIMER_MODE_REL);
	}

	if (atomic_read(&stat->enabled_gyr) > 0) {
		_lsm6ds0_gyr_device_power_on(stat);
		hrtimer_start(&(stat->hr_timer_gyr), stat->ktime_gyr,
							HRTIMER_MODE_REL);
	}

	return 0;
}
EXPORT_SYMBOL(lsm6ds0_common_resume);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION(LSM6DS0_MOD_DESCRIPTION);
MODULE_AUTHOR("Giuseppe Barba,STMicroelectronics");
MODULE_LICENSE("GPL");
