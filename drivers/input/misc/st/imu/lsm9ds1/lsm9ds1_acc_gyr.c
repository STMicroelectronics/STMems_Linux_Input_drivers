/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lsm9ds1_acc_gyr.c
* Authors            : MSH - C&I BU - Application Team
*                    : Giuseppe Barba (giuseppe.barba@st.com)
*                    : Matteo Dameno (matteo.dameno@st.com)
*                    : Denis Ciocca (denis.ciocca@st.com)
*                    : Lorenzo Bianconi (lorenzo.bianconi@st.com)
*                    : Authors are willing to be considered the contact
*                    : and update points for the driver.
* Version            : V.1.0.0
* Date               : 2016/May/16
* Description        : LSM9DS1 accelerometer & gyroscope driver
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
********************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif

#include "lsm9ds1.h"

#define MS_TO_NS(x)			(x*1000000L)
#define REFERENCE_G			(0x0B)

/* Sensitivity */
#define SENSITIVITY_ACC_2G		(60)	/** ug/LSB */
#define SENSITIVITY_ACC_4G		(120)	/** ug/LSB */
#define SENSITIVITY_ACC_8G		(240)	/** ug/LSB */
#define SENSITIVITY_GYR_250		(8750)	/** udps/LSB */
#define SENSITIVITY_GYR_500		(17500)	/** udps/LSB */
#define SENSITIVITY_GYR_2000		(70000)	/** udps/LSB */

#define FILTER_50			(50)/** Anti-Aliasing 50 Hz */
#define FILTER_105			(105)/** Anti-Aliasing 105 Hz */
#define FILTER_211			(211)/** Anti-Aliasing 211 Hz */
#define FILTER_408			(408)/** Anti-Aliasing 408 Hz */

#define RANGE_245DPS			(245)
#define RANGE_500DPS			(500)
#define RANGE_2000DPS			(2000)

#define ACT_THS				(0x04)
#define ACT_DUR				(0x05)
#define WHO_AM_I			(0x0F)
#define WHO_AM_I_VAL			(0x68)

/* Angular rate sensor Control Register 1 */
#define CTRL_REG1_G			(0x10)

#define BW_G_SHIFT			(0)
#define BW_G_MASK			(0x03)

#define FS_G_SHIFT			(3)
#define FS_G_MASK			(0x18) 

/* Angular rate sensor Control Register 2 */
#define CTRL_REG2_G			(0x11)

#define OUT_SEL_SHIFT			(0)
#define OUT_SEL_MASK			(0x03)

#define INT_SEL_SHIFT			(2)
#define INT_SEL_MASK			(0x0C)

#define SEL_LPF1			(0x00)
#define SEL_HPF				(0x01)
#define SEL_LPF2			(0x02)


#define CTRL_REG3_G			(0x12)

/* Angular rate sensor sign and orientation register. */
#define ORIENT_CFG_G			(0x13)
#define ORIENT_CFG_G_SIGN_X_MASK	(0x20)
#define ORIENT_CFG_G_SIGN_Y_MASK	(0x10)
#define ORIENT_CFG_G_SIGN_Z_MASK	(0x08)
#define ORIENT_CFG_G_SIGN_ORIENT_MASK	(0x07)

#define OUT_TEMP_L			(0x15)
#define OUT_TEMP_H			(0x16)
#define STATUS_REG1			(0x17)
#define	OUT_X_L_G			(0x18) /* 1st AXIS OUT REG of 6 */

#define CTRL_REG4			(0x1E)
#define CTRL_REG4_DEF			(0x38)
#define CTRL_REG4_X_EN			(0x08)
#define CTRL_REG4_Y_EN			(0x10)
#define CTRL_REG4_Z_EN			(0x20)
#define CTRL_REG4_ALL_AXES_EN		(0x38)
#define CTRL_REG4_AXES_EN_MASK		(0x38)

#define CTRL_REG5_XL			(0x1F)
#define CTRL_REG5_XL_DEF		(0x38)

/* Linear acceleration sensor Control Register 6 */
#define CTRL_REG6_XL			(0x20)

#define LSM9DS1_ACC_FS_DEF		(LSM9DS1_ACC_FS_2G)

#define BW_SCAL_ODR_SHIFT		(2)
#define BW_SCAL_ODR_MASK		(0x04)

#define BW_XL_50			(0x0C)
#define BW_XL_105			(0x08)
#define BW_XL_211			(0x04)
#define BW_XL_408			(0x00)
#define BW_XL_DEF			(BW_XL_408)


#define CTRL_REG7_XL			(0x21)

#define CTRL_REG8			(0x22)
#define CTRL_REG8_DEF			(0x44)

#define CTRL_REG9			(0x23)
#define CTRL_REG10			(0x24)


#define STATUS_REG2			(0x27)
#define OUT_X_L_XL			(0x28) /* 1st AXIS OUT REG of 6 */

#define FIFO_CTRL			(0x2E)
#define FIFO_SRC			(0x2F)

/* INT1_A/G pin control register. */
#define INT1_CTRL			(0x0C)
#define INT1_CTRL_IG_G_MASK		(0x80)
#define INT1_CTRL_IG_XL_MASK		(0x40)
#define INT1_CTRL_FSS5_MASK		(0x20)
#define INT1_CTRL_OVR_MASK		(0x10)
#define INT1_CTRL_FTH_MASK		(0x08)
#define INT1_CTRL_BOOT_MASK		(0x04)
#define INT1_CTRL_DRDY_G_MASK		(0x02)
#define INT1_CTRL_DRDY_XL_MASK		(0x01)
#define INT1_CTRL_DRDY_BOTH		(INT1_CTRL_DRDY_XL_MASK | \
					 INT1_CTRL_DRDY_G_MASK)

/* INT2_A/G pin control register. */
#define INT2_CTRL			(0x0D)
#define INT2_CTRL_INACT_MASK		(0x80)
#define INT2_CTRL_FSS5_MASK		(0x20)
#define INT2_CTRL_OVR_MASK		(0x10)
#define INT2_CTRL_FTH_MASK		(0x08)
#define INT2_CTRL_DRDY_TEMP_MASK	(0x04)
#define INT2_CTRL_DRDY_G_MASK		(0x02)
#define INT2_CTRL_DRDY_XL_MASK		(0x01)

/* Linear acceleration sensor interrupt source register. */
#define INT_GEN_SRC_XL			(0x26)
#define INT_GEN_SRC_XL_IA_MASK		(0x40)
#define INT_GEN_SRC_XL_ZH_MASK		(0x20)
#define INT_GEN_SRC_XL_ZL_MASK		(0x10)
#define INT_GEN_SRC_XL_YH_MASK		(0x08)
#define INT_GEN_SRC_XL_YL_MASK		(0x04)
#define INT_GEN_SRC_XL_XH_MASK		(0x02)
#define INT_GEN_SRC_XL_XL_MASK		(0x01)

/* Linear acceleration sensor interrupt generator configuration register. */
#define INT_GEN_CFG_XL			(0x06)
#define INT_GEN_CFG_XL_AOI_MASK	(0x80)
#define INT_GEN_CFG_XL_6D_MASK		(0x40)
#define INT_GEN_CFG_XL_ZHIE_MASK	(0x20)
#define INT_GEN_CFG_XL_ZLIE_MASK	(0x10)
#define INT_GEN_CFG_XL_YHIE_MASK	(0x08)
#define INT_GEN_CFG_XL_YLIE_MASK	(0x04)
#define INT_GEN_CFG_XL_XHIE_MASK	(0x02)
#define INT_GEN_CFG_XL_XLIE_MASK	(0x01)

/* Linear acceleration sensor interrupt threshold registers. */
#define INT_GEN_THS_X_XL		(0x07)
#define INT_GEN_THS_Y_XL		(0x08)
#define INT_GEN_THS_Z_XL		(0x09)

/* Linear acceleration sensor interrupt duration register. */
#define INT_GEN_DUR_XL			(0x0A)
#define INT_GEN_DUR_XL_WAIT_MASK	(0x80)
#define INT_GEN_DUR_XL_DUR_MASK	(0x7F)

/* Angular rate sensor interrupt source register. */
#define INT_GEN_SRC_G			(0x14)
#define INT_GEN_SRC_G_IA_MASK		(0x40)
#define INT_GEN_SRC_G_ZH_MASK		(0x20)
#define INT_GEN_SRC_G_ZL_MASK		(0x10)
#define INT_GEN_SRC_G_YH_MASK		(0x08)
#define INT_GEN_SRC_G_YL_MASK		(0x04)
#define INT_GEN_SRC_G_XH_MASK		(0x02)
#define INT_GEN_SRC_G_XL_MASK		(0x01)

/* Angular rate sensor interrupt generator configuration register. */
#define INT_GEN_CFG_G			(0x30)
#define INT_GEN_CFG_G_AOI_MASK		(0x80)
#define INT_GEN_CFG_G_LIR_MASK		(0x40)
#define INT_GEN_CFG_G_ZHIE_MASK	(0x20)
#define INT_GEN_CFG_G_ZLIE_MASK	(0x10)
#define INT_GEN_CFG_G_YHIE_MASK	(0x08)
#define INT_GEN_CFG_G_YLIE_MASK	(0x04)
#define INT_GEN_CFG_G_XHIE_MASK	(0x02)
#define INT_GEN_CFG_G_XLIE_MASK	(0x01)

/* Angular rate sensor interrupt generator threshold registers. */
#define INT_GEN_THS_XH_G		(0x31)
#define INT_GEN_THS_XL_G		(0x32)
#define INT_GEN_THS_YH_G		(0x33)
#define INT_GEN_THS_YL_G		(0x34)
#define INT_GEN_THS_ZH_G		(0x35)
#define INT_GEN_THS_ZL_G		(0x36)

/* Angular rate sensor interrupt generator duration register. */
#define INT_GEN_DUR_G			(0x37)
#define INT_GEN_DUR_G_WAIT_MASK	(0x80)
#define INT_GEN_DUR_G_DUR_MASK		(0x7F)

#define DEF_ZERO			(0x00)
#define UNDEF				(0x00)
#define NDTEMP				(1000)	/* Not Available temperature */

#define GET_BIT(reg,mask)		(((reg & mask) == mask) ? 1 : 0)
#define SET_BIT(reg,mask)		(reg | mask)
#define UNSET_BIT(reg,mask)		(reg & (~mask))

static struct kobject *acc_kobj;
static struct kobject *gyr_kobj;

#define to_dev(obj) container_of(obj, struct device, kobj)

struct output_rate{
	unsigned int cutoff_ms;
	u8 value;
}; 

static const struct output_rate lsm9ds1_gyr_odr_table[] = {
	{  1, (LSM9DS1_GYR_ODR_952 | (LSM9DS1_GYR_BW_3)) },
	{  2, (LSM9DS1_GYR_ODR_476 | (LSM9DS1_GYR_BW_3)) },
	{  4, (LSM9DS1_GYR_ODR_238 | (LSM9DS1_GYR_BW_3)) },
	{  8, (LSM9DS1_GYR_ODR_119 | (LSM9DS1_GYR_BW_3)) },
	{ 16, (LSM9DS1_GYR_ODR_59_5 | (LSM9DS1_GYR_BW_3)) },
	{ 67, (LSM9DS1_GYR_ODR_14_9) },
};

static const struct output_rate lsm9ds1_acc_odr_table[] = {
        {  1, (LSM9DS1_ACC_ODR_952) },
        {  2, (LSM9DS1_ACC_ODR_476) },
        {  4, (LSM9DS1_ACC_ODR_238) },
        {  8, (LSM9DS1_ACC_ODR_119) },
        { 20, (LSM9DS1_ACC_ODR_50) },
        { 100, (LSM9DS1_ACC_ODR_10) },
};

static const struct lsm9ds1_acc_platform_data default_lsm9ds1_acc_pdata = {
	.fs_range = LSM9DS1_ACC_FS_2G,
	.poll_interval = LSM9DS1_ACC_POLL_INTERVAL_DEF,
	.min_interval = LSM9DS1_ACC_MIN_POLL_PERIOD_MS,
	.aa_filter_bandwidth = LSM9DS1_ACC_BW_408,
};

static const struct lsm9ds1_gyr_platform_data default_lsm9ds1_gyr_pdata = {
	.fs_range = LSM9DS1_GYR_FS_245DPS,
	.poll_interval = LSM9DS1_GYR_POLL_INTERVAL_DEF,
	.min_interval = LSM9DS1_GYR_MIN_POLL_PERIOD_MS,
};

struct lsm9ds1_acc_gyr_main_platform_data default_lsm9ds1_main_platform_data = {
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
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

struct reg_rw {
	u8 address;
	u8 default_val;
	u8 resume_val;
};

struct reg_r {
	u8 address;
	u8 default_val;
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
		{.address = INT1_CTRL,		.default_val = INT1_CTRL_DRDY_BOTH,},
	.int2_ctrl =
		{.address = INT2_CTRL,		.default_val = DEF_ZERO,},
	.who_am_i =
		{.address = WHO_AM_I,		.default_val = WHO_AM_I_VAL,},
	.ctrl_reg1_g =
		{.address = CTRL_REG1_G,	.default_val = DEF_ZERO,},
	.ctrl_reg2_g =
		{.address = CTRL_REG2_G,	.default_val = DEF_ZERO,},
	.ctrl_reg3_g =
		{.address = CTRL_REG3_G,	.default_val = DEF_ZERO,},
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
/*****************************************************************************/

static int lsm9ds1_acc_device_power_off(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;
	u8 buf[1];

	buf[0] = (LSM9DS1_ACC_ODR_MASK & LSM9DS1_ACC_ODR_OFF) |
		 (~LSM9DS1_ACC_ODR_MASK &
		  status_registers.ctrl_reg6_xl.resume_val);

	err = dev->tf->write(dev->dev, status_registers.ctrl_reg6_xl.address,
			     1, buf);
	if (err < 0)
		dev_err(dev->dev, "accelerometer soft power off "
							"failed: %d\n", err);

	if (dev->pdata_acc->power_off) {
		dev->pdata_acc->power_off();
	}

	atomic_set(&dev->enabled_acc, 0);
	dev->acc_skip_cnt = 0;
	dev_info(dev->dev, "accelerometer switched off.");

	return 0;
}

static int lsm9ds1_gyr_device_power_off(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;
	u8 buf[1];

	buf[0] = (LSM9DS1_GYR_ODR_MASK & LSM9DS1_GYR_ODR_OFF) |
		 (~LSM9DS1_GYR_ODR_MASK &
		  status_registers.ctrl_reg1_g.resume_val);

	err = dev->tf->write(dev->dev, status_registers.ctrl_reg1_g.address,
			     1, buf);
	if (err < 0)
		dev_err(dev->dev, "gyroscope soft power off "
							"failed: %d\n", err);

	if (dev->pdata_gyr->power_off) {
		dev->pdata_gyr->power_off();
	}

	atomic_set(&dev->enabled_gyr, 0);
	dev->gyr_skip_cnt = 0;
	dev_info(dev->dev, "gyroscope switched off");

	return 0;
}

static int lsm9ds1_gyr_disable(struct lsm9ds1_acc_gyr_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_gyr, 1, 0))
		lsm9ds1_gyr_device_power_off(dev);

	return 0;
}

static int lsm9ds1_acc_disable(struct lsm9ds1_acc_gyr_dev *dev)
{
	if (atomic_cmpxchg(&dev->enabled_acc, 1, 0)) {
		if (atomic_read(&dev->enabled_gyr) > 0)
			lsm9ds1_gyr_disable(dev);

		lsm9ds1_acc_device_power_off(dev);

		if (dev->irq > 0)
			disable_irq(dev->irq);
	}

	return 0;
}

int lsm9ds1_acc_gyr_disable(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;

	err = lsm9ds1_acc_disable(dev);
	if (err < 0)
		return err;
	return lsm9ds1_gyr_disable(dev);
}
EXPORT_SYMBOL(lsm9ds1_acc_gyr_disable);

static void lsm9ds1_acc_input_cleanup(struct lsm9ds1_acc_gyr_dev *dev)
{
	input_unregister_device(dev->input_dev_acc);
	input_free_device(dev->input_dev_acc);
}

static void lsm9ds1_gyr_input_cleanup(struct lsm9ds1_acc_gyr_dev *dev)
{
	input_unregister_device(dev->input_dev_gyr);
	input_free_device(dev->input_dev_gyr);
}

static void lsm9ds1_validate_polling(unsigned int *min_interval,
					unsigned int *poll_interval,
					unsigned int min)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);
}

static int lsm9ds1_acc_validate_pdata(struct lsm9ds1_acc_gyr_dev *dev)
{
	int res = -EINVAL;

	lsm9ds1_validate_polling(&dev->pdata_acc->min_interval,
				 &dev->pdata_acc->poll_interval,
				(unsigned int)LSM9DS1_ACC_MIN_POLL_PERIOD_MS);

	switch (dev->pdata_acc->aa_filter_bandwidth) {
	case LSM9DS1_ACC_BW_50:
		res = 1;
		break;
	case LSM9DS1_ACC_BW_105:
		res = 1;
		break;
	case LSM9DS1_ACC_BW_211:
		res = 1;
		break;
	case LSM9DS1_ACC_BW_408:
		res = 1;
		break;
	default:
		dev_err(dev->dev, "invalid accelerometer "
			"bandwidth selected: %u\n",
				dev->pdata_acc->aa_filter_bandwidth);
	}

	return res;
}

static int lsm9ds1_gyr_validate_pdata(struct lsm9ds1_acc_gyr_dev *dev)
{
	/* checks for correctness of minimal polling period */
	lsm9ds1_validate_polling(&dev->pdata_gyr->min_interval, 
				 &dev->pdata_gyr->poll_interval,
				(unsigned int)LSM9DS1_GYR_MIN_POLL_PERIOD_MS);

	/* Enforce minimum polling interval */
	if (dev->pdata_gyr->poll_interval < dev->pdata_gyr->min_interval) {
		dev_err(dev->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int lsm9ds1_acc_gyr_check_whoami(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;
	u8 data;

	err = dev->tf->read(dev->dev, status_registers.who_am_i.address, 1,
			    &data);
	if (err < 0) {
		dev_warn(dev->dev, "Error reading WHO_AM_I\n");
		return err;
	}

	if (data != status_registers.who_am_i.default_val) {
		dev_err(dev->dev, "device unknown 0x%02x - 0x%02x\n",
			status_registers.who_am_i.default_val, data);
		return -1;
	}

	return 0;
}

static int lsm9ds1_acc_gyr_hw_init(struct lsm9ds1_acc_gyr_dev *dev)
{
#ifdef LSM9DS1_DEBUG
	dev_info(dev->dev, "%s: hw init start\n", LSM9DS1_ACC_GYR_DEV_NAME);
#endif

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

	dev->temp_value_dec = NDTEMP;

#ifdef LSM9DS1_DEBUG
	dev_info(dev->dev, "%s: hw init done\n", LSM9DS1_ACC_GYR_DEV_NAME);
#endif

	return 0;
}

static int lsm9ds1_acc_device_power_on(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err = -1;
	u8 buf[7];

	if (dev->pdata_acc->power_on) {
		err = dev->pdata_acc->power_on();
		if (err < 0) {
			dev_err(dev->dev,
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
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg4.address,
			     7, buf);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = status_registers.int_gen_cfg_xl.resume_val;
	buf[1] = status_registers.int_gen_ths_x_xl.resume_val;
	buf[2] = status_registers.int_gen_ths_y_xl.resume_val;
	buf[3] = status_registers.int_gen_ths_z_xl.resume_val;
	buf[4] = status_registers.int_gen_dur_xl.resume_val;
	err = dev->tf->write(dev->dev, status_registers.int_gen_cfg_xl.address,
			     5, buf);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = status_registers.int1_ctrl.resume_val;
	buf[1] = status_registers.int2_ctrl.resume_val;
	err = dev->tf->write(dev->dev, status_registers.int1_ctrl.address,
			     2, buf);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = status_registers.fifo_ctrl.resume_val;
	err = dev->tf->write(dev->dev, status_registers.fifo_ctrl.address,
			     1, buf);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = status_registers.ctrl_reg8.resume_val;
	buf[1] = status_registers.ctrl_reg9.resume_val;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg8.address,
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

static int lsm9ds1_gyr_device_power_on(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err = -1;
	u8 buf[8];

	if (dev->pdata_gyr->power_on) {
		err = dev->pdata_gyr->power_on();
		if (err < 0) {
			dev_err(dev->dev,
				"gyroscope power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.act_ths.resume_val;
	err = dev->tf->write(dev->dev, status_registers.act_ths.address,
			     1, buf);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = status_registers.reference_g.resume_val;
	err = dev->tf->write(dev->dev, status_registers.reference_g.address,
			     1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg1_g.resume_val;
	buf[1] = status_registers.ctrl_reg2_g.resume_val;
	buf[2] = status_registers.ctrl_reg3_g.resume_val;
	buf[3] = status_registers.orient_cfg_g.resume_val;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg1_g.address,
			     4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg4.resume_val;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg4.address,
			     1, buf);
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
	err = dev->tf->write(dev->dev, status_registers.int_gen_cfg_g.address,
			     8, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.resume_val;
	buf[1] = status_registers.int2_ctrl.resume_val;
	err = dev->tf->write(dev->dev, status_registers.int1_ctrl.address,
			     2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[1] = status_registers.fifo_ctrl.resume_val;
	err = dev->tf->write(dev->dev, status_registers.fifo_ctrl.address,
			     1, buf);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&dev->enabled_gyr, 1);

	return 0;

err_resume_state:
	atomic_set(&dev->enabled_gyr, 0);
	dev_err(dev->dev, "gyroscope hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int lsm9ds1_acc_update_fs_range(struct lsm9ds1_acc_gyr_dev *dev,
				       u8 new_fs_range)
{
	int err = -1;
	u16 sensitivity;
	u8 val, buf[1];

	switch (new_fs_range) {
	case LSM9DS1_ACC_FS_2G:
		sensitivity = SENSITIVITY_ACC_2G;
		break;
	case LSM9DS1_ACC_FS_4G:
		sensitivity = SENSITIVITY_ACC_4G;
		break;
	case LSM9DS1_ACC_FS_8G:
		sensitivity = SENSITIVITY_ACC_8G;
		break;
	default:
		dev_err(dev->dev, "invalid acc fs range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	val = ((LSM9DS1_ACC_FS_MASK & new_fs_range) |
	       (~LSM9DS1_ACC_FS_MASK &
		status_registers.ctrl_reg6_xl.resume_val));

	buf[0] = val;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg6_xl.address,
			     1, buf);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = val;
	dev->sensitivity_acc = sensitivity;

	return err;

error:
	dev_err(dev->dev, "update accelerometer fs range failed "
		"0x%02x: %d\n", buf[0], err);
	return err;
}

static int lsm9ds1_gyr_update_fs_range(struct lsm9ds1_acc_gyr_dev *dev,
				       u8 new_fs_range)
{
	int err = -1;
	u8 buf[1];
	u8 updated_val;

	u32 sensitivity;

	switch(new_fs_range) {
	case LSM9DS1_GYR_FS_245DPS:
		sensitivity = SENSITIVITY_GYR_250;
		break;
	case LSM9DS1_GYR_FS_500DPS:
		sensitivity = SENSITIVITY_GYR_500;
		break;
	case LSM9DS1_GYR_FS_2000DPS:
		sensitivity = SENSITIVITY_GYR_2000;
		break;
	default:
		dev_err(dev->dev, "invalid g range "
					"requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	updated_val = ((LSM9DS1_GYR_FS_MASK & new_fs_range) |
		       (~LSM9DS1_GYR_FS_MASK & status_registers.ctrl_reg1_g.resume_val));
	
	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg1_g.address,
			     1, buf);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg1_g.resume_val = updated_val;
	dev->sensitivity_gyr = sensitivity;

error:
	return err;
}

static int lsm9ds1_acc_update_odr(struct lsm9ds1_acc_gyr_dev *dev,
				  unsigned int poll_ms)
{
	int i, err = 0;
	u8 buf[1];

	for (i = ARRAY_SIZE(lsm9ds1_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm9ds1_acc_odr_table[i].cutoff_ms <= poll_ms) ||
		    (i == 0))
			break;
	}

	buf[0] = LSM9DS1_ACC_ODR_MASK & lsm9ds1_acc_odr_table[i].value;
	buf[0] |= (~LSM9DS1_ACC_ODR_MASK) &
		  status_registers.ctrl_reg6_xl.resume_val;

	if (atomic_read(&dev->enabled_acc)) {
		err = dev->tf->write(dev->dev,
				     status_registers.ctrl_reg6_xl.address,
				     1, buf);
		if (err < 0)
			goto error;
	}

	dev->acc_dec_cnt = poll_ms / lsm9ds1_acc_odr_table[i].cutoff_ms;
	dev->acc_skip_cnt = 0;

	status_registers.ctrl_reg6_xl.resume_val = buf[0];

	dev_info(dev->dev, "accelerometer odr set to %d\n", poll_ms);

	return err;

error:
	dev_err(dev->dev, "update accelerometer odr failed "
			"0x%02x: %d\n", buf[0], err);

	return err;
}

static int lsm9ds1_gyr_update_odr(struct lsm9ds1_acc_gyr_dev *dev,
				  unsigned int poll_ms)
{
	int err = 0;
	u8 buf[1];
	u8 val;
	int i;

	if (atomic_read(&dev->enabled_acc))
		val = min(poll_ms, dev->pdata_acc->poll_interval);
	else
		val = poll_ms;

	for (i = ARRAY_SIZE(lsm9ds1_gyr_odr_table) - 1; i >= 0; i--)
		if ((lsm9ds1_gyr_odr_table[i].cutoff_ms <= val) || (i == 0))
			break;

	buf[0] = LSM9DS1_GYR_ODR_MASK & lsm9ds1_gyr_odr_table[i].value;
	buf[0] |= (~LSM9DS1_GYR_ODR_MASK) &
		  status_registers.ctrl_reg1_g.resume_val;

	if (atomic_read(&dev->enabled_gyr)) {
		/* Set ODR value */
		err = dev->tf->write(dev->dev,
				     status_registers.ctrl_reg1_g.address,
				     1, buf);
		if (err < 0)
			goto error;
	}

	dev->gyr_dec_cnt = poll_ms / lsm9ds1_gyr_odr_table[i].cutoff_ms;
	dev->gyr_skip_cnt = 0;

	status_registers.ctrl_reg1_g.resume_val = buf[0];

	dev_info(dev->dev, "gyro odr set to %d\n", poll_ms);

	return err;

error:
	dev_err(dev->dev, "update accelerometer odr failed "
		"0x%02x: %d\n", buf[0], err);

	return err;
}

static int lsm9ds1_acc_update_filter(struct lsm9ds1_acc_gyr_dev *dev,
							u8 new_bandwidth)
{
	int err = -1;
	u8 updated_val;
	u8 buf[1];

	switch (new_bandwidth) {
	case LSM9DS1_ACC_BW_50:
		break;
	case LSM9DS1_ACC_BW_105:
		break;
	case LSM9DS1_ACC_BW_211:
		break;
	case LSM9DS1_ACC_BW_408:
		break;
	default:
		dev_err(dev->dev, "invalid accelerometer "
			"update bandwidth requested: %u\n", new_bandwidth);
		return -EINVAL;
	}

	err = dev->tf->read(dev->dev, status_registers.ctrl_reg6_xl.address, 1,
			    buf);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = buf[0];

	updated_val = ((LSM9DS1_ACC_BW_MASK & new_bandwidth) |
		       (~LSM9DS1_ACC_BW_MASK & buf[0]));

	buf[0] = updated_val;
	err = dev->tf->write(dev->dev, status_registers.ctrl_reg6_xl.address,
			     1, buf);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = updated_val;

	return err;

error:
	dev_err(dev->dev, "update accelerometer fs range failed "
		"0x%02x: %d\n", buf[0], err);
	return err;
}

static int lsm9ds1_acc_enable(struct lsm9ds1_acc_gyr_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled_acc, 0, 1)) {
		int err;

		err = lsm9ds1_acc_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled_acc, 0);
			dev_err(dev->dev, "enable accelerometer failed");
			return err;
		}

		if (dev->irq > 0)
			enable_irq(dev->irq);
	}

	return 0;
}

static int lsm9ds1_gyr_enable(struct lsm9ds1_acc_gyr_dev *dev)
{
	if (!atomic_cmpxchg(&dev->enabled_gyr, 0, 1)) {
		int err;

		if (atomic_read(&dev->enabled_acc) == 0)
			lsm9ds1_acc_enable(dev);

		err = lsm9ds1_gyr_device_power_on(dev);
		if (err < 0) {
			atomic_set(&dev->enabled_gyr, 0);
			return err;
		}
	}
	return 0;
}

int lsm9ds1_acc_gyr_enable(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;

	err = lsm9ds1_acc_enable(dev);
	if (err < 0)
		return err;
	return lsm9ds1_gyr_enable(dev);
}
EXPORT_SYMBOL(lsm9ds1_acc_gyr_enable);

static int lsm9ds1_acc_get_data(struct lsm9ds1_acc_gyr_dev *dev, int *xyz)
{
	int i, err = -1;
	u8 acc_data[6];
	s32 hw_d[3];

	err = dev->tf->read(dev->dev, OUT_X_L_XL, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s32)( (s16)((acc_data[1] << 8) | (acc_data[0]))));
	hw_d[1] = ((s32)( (s16)((acc_data[3] << 8) | (acc_data[2]))));
	hw_d[2] = ((s32)( (s16)((acc_data[5] << 8) | (acc_data[4]))));

#ifdef LSM9DS1_DEBUG
	dev_info(dev->dev, "%s read x=%X %X(regH regL), x=%d(dec) [udps]\n",
		 LSM9DS1_GYR_DEV_NAME, acc_data[1], acc_data[0], hw_d[0]);
	dev_info(dev->dev, "%s read y=%X %X(regH regL), y=%d(dec) [udps]\n",
		 LSM9DS1_GYR_DEV_NAME, acc_data[3], acc_data[2], hw_d[1]);
	dev_info(dev->dev, "%s read z=%X %X(regH regL), z=%d(dec) [udps]\n",
		 LSM9DS1_GYR_DEV_NAME, acc_data[5], acc_data[4], hw_d[2]);
#endif

	hw_d[0] = hw_d[0] * dev->sensitivity_acc;
	hw_d[1] = hw_d[1] * dev->sensitivity_acc;
	hw_d[2] = hw_d[2] * dev->sensitivity_acc;

	for (i = 0; i < 3; i++) {
		xyz[i] = dev->pdata_main->rot_matrix[0][i] * hw_d[0] +
				dev->pdata_main->rot_matrix[1][i] * hw_d[1] +
				dev->pdata_main->rot_matrix[2][i] * hw_d[2];
	}

	return err;
}

static int lsm9ds1_gyr_get_data(struct lsm9ds1_acc_gyr_dev *dev, int *xyz)
{
	int i, err = 1;
	u8 gyro_data[6];
	s32 hw_d[3];

	err = dev->tf->read(dev->dev, OUT_X_L_G, 6, gyro_data);
	if (err < 0)
		return err;

	hw_d[0] = (s32) ((s16)((gyro_data[1]) << 8) | gyro_data[0]);
	hw_d[1] = (s32) ((s16)((gyro_data[3]) << 8) | gyro_data[2]);
	hw_d[2] = (s32) ((s16)((gyro_data[5]) << 8) | gyro_data[4]);

#ifdef LSM9DS1_DEBUG
	dev_info(dev->dev, "%s read x=%X %X(regH regL), x=%d(dec) [udps]\n",
		 LSM9DS1_GYR_DEV_NAME, gyro_data[1], gyro_data[0], hw_d[0]);
	dev_info(dev->dev, "%s read y=%X %X(regH regL), y=%d(dec) [udps]\n",
		 LSM9DS1_GYR_DEV_NAME, gyro_data[3], gyro_data[2], hw_d[1]);
	dev_info(dev->dev, "%s read z=%X %X(regH regL), z=%d(dec) [udps]\n",
		 LSM9DS1_GYR_DEV_NAME, gyro_data[5], gyro_data[4], hw_d[2]);
#endif

	hw_d[0] = hw_d[0] * dev->sensitivity_gyr;
	hw_d[1] = hw_d[1] * dev->sensitivity_gyr;
	hw_d[2] = hw_d[2] * dev->sensitivity_gyr;

	for (i = 0; i < 3; i++) {
		xyz[i] = dev->pdata_main->rot_matrix[0][i] * hw_d[0] +
				dev->pdata_main->rot_matrix[1][i] * hw_d[1] +
				dev->pdata_main->rot_matrix[2][i] * hw_d[2];
	}

	return err;
}

static void lsm9ds1_acc_report_values(struct lsm9ds1_acc_gyr_dev *dev,
				      int *xyz, s64 timestamp)
{
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(dev->input_dev_acc, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(dev->input_dev_acc);
}

static void lsm9ds1_gyr_report_values(struct lsm9ds1_acc_gyr_dev *dev,
				      int *xyz, s64 timestamp)
{
	input_event(dev->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    xyz[0]);
	input_event(dev->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    xyz[1]);
	input_event(dev->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_Z,
		    xyz[2]);
	input_event(dev->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    timestamp >> 32);
	input_event(dev->input_dev_gyr, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    timestamp & 0xffffffff);
	input_sync(dev->input_dev_gyr);
}

static int lsm9ds1_acc_input_init(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;

	dev->input_dev_acc = input_allocate_device();
	if (!dev->input_dev_acc) {
		dev_err(dev->dev, "acc input allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev_acc->name = LSM9DS1_ACC_DEV_NAME;
	dev->input_dev_acc->id.bustype = dev->bus_type;
	dev->input_dev_acc->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_acc, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev_acc->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev_acc->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev_acc->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev_acc->mscbit);
	set_bit(INPUT_EVENT_TIME_MSB, dev->input_dev_acc->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, dev->input_dev_acc->mscbit);

	err = input_register_device(dev->input_dev_acc);
	if (err) {
		dev_err(dev->dev,
			"unable to register accelerometer input device %s\n",
			dev->input_dev_acc->name);
		input_free_device(dev->input_dev_acc);
	}

	return err;
}

static int lsm9ds1_gyr_input_init(struct lsm9ds1_acc_gyr_dev *dev)
{
	int err;

	dev->input_dev_gyr = input_allocate_device();
	if (!dev->input_dev_gyr) {
		dev_err(dev->dev, "input device allocation failed\n");
		return -ENOMEM;
	}

	dev->input_dev_gyr->name = LSM9DS1_GYR_DEV_NAME;
	dev->input_dev_gyr->id.bustype = dev->bus_type;
	dev->input_dev_gyr->dev.parent = dev->dev;

	input_set_drvdata(dev->input_dev_gyr, dev);

	set_bit(INPUT_EVENT_TYPE, dev->input_dev_gyr->evbit);
	set_bit(INPUT_EVENT_X, dev->input_dev_gyr->mscbit);
	set_bit(INPUT_EVENT_Y, dev->input_dev_gyr->mscbit);
	set_bit(INPUT_EVENT_Z, dev->input_dev_gyr->mscbit);
	set_bit(INPUT_EVENT_TIME_MSB, dev->input_dev_gyr->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, dev->input_dev_gyr->mscbit);

	err = input_register_device(dev->input_dev_gyr);
	if (err) {
		dev_err(dev->dev,
			"unable to register input device %s\n",
			dev->input_dev_gyr->name);
		input_free_device(dev->input_dev_gyr);
	}

	return err;
}
static void lsm9ds1_input_cleanup(struct lsm9ds1_acc_gyr_dev *dev)
{
	input_unregister_device(dev->input_dev_acc);
	input_free_device(dev->input_dev_acc);

	input_unregister_device(dev->input_dev_gyr);
	input_free_device(dev->input_dev_gyr);
}

static ssize_t attr_set_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_ms = (unsigned int)max((unsigned int)interval_ms,
						dev->pdata_acc->min_interval);

	mutex_lock(&dev->lock);
	dev->pdata_acc->poll_interval = (unsigned int)interval_ms;
	lsm9ds1_acc_update_odr(dev, interval_ms);
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	unsigned int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata_acc->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_get_enable_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);

	int val = (int)atomic_read(&dev->enabled_acc);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm9ds1_acc_enable(dev);
	else
		lsm9ds1_acc_disable(dev);

	return size;
}

static ssize_t attr_get_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	u8 val;
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	int range = 2;

	mutex_lock(&dev->lock);
	val = dev->pdata_acc->fs_range ;
	mutex_unlock(&dev->lock);

	switch (val) {
	case LSM9DS1_ACC_FS_2G:
		range = 2;
		break;
	case LSM9DS1_ACC_FS_4G:
		range = 4;
		break;
	case LSM9DS1_ACC_FS_8G:
		range = 8;
		break;
	}

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM9DS1_ACC_FS_2G;
		break;
	case 4:
		range = LSM9DS1_ACC_FS_4G;
		break;
	case 8:
		range = LSM9DS1_ACC_FS_8G;
		break;
	default:
		dev_err(dev->dev, "accelerometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lsm9ds1_acc_update_fs_range(dev, range);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}
	dev->pdata_acc->fs_range = range;
	mutex_unlock(&dev->lock);

	dev_info(dev->dev, "accelerometer range set to %lu g\n", val);

	return size;
}

static ssize_t attr_get_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	u8 val;
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	int frequency = FILTER_408;

	mutex_lock(&dev->lock);
	val = dev->pdata_acc->aa_filter_bandwidth;
	mutex_unlock(&dev->lock);

	switch (val) {
	case LSM9DS1_ACC_BW_50:
		frequency = FILTER_50;
		break;
	case LSM9DS1_ACC_BW_105:
		frequency = FILTER_105;
		break;
	case LSM9DS1_ACC_BW_211:
		frequency = FILTER_211;
		break;
	case LSM9DS1_ACC_BW_408:
		frequency = FILTER_408;
		break;
	}

	return sprintf(buf, "%d\n", frequency);
}

static ssize_t attr_set_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 frequency;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case FILTER_50:
		frequency = LSM9DS1_ACC_BW_50;
		break;
	case FILTER_105:
		frequency = LSM9DS1_ACC_BW_105;
		break;
	case FILTER_211:
		frequency = LSM9DS1_ACC_BW_211;
		break;
	case FILTER_408:
		frequency = LSM9DS1_ACC_BW_408;
		break;
	default:
		dev_err(dev->dev, "accelerometer invalid filter "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lsm9ds1_acc_update_filter(dev, frequency);
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

static ssize_t attr_get_polling_rate_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	int val;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);

	mutex_lock(&dev->lock);
	val = dev->pdata_gyr->poll_interval;
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	int err;
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	
	interval_ms = (unsigned int)max((unsigned int)interval_ms,
					dev->pdata_gyr->min_interval);

	mutex_lock(&dev->lock);
	err = lsm9ds1_gyr_update_odr(dev, interval_ms);
	if(err >= 0)
		dev->pdata_gyr->poll_interval = interval_ms;
	mutex_unlock(&dev->lock);

	return size;
}

static ssize_t attr_get_enable_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	int val = atomic_read(&dev->enabled_gyr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm9ds1_gyr_enable(dev);
	else
		lsm9ds1_gyr_disable(dev);

	return size;
}

static ssize_t attr_get_range_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	int range = 0;
	u8 val;

	mutex_lock(&dev->lock);
	val = dev->pdata_gyr->fs_range;
	switch (val) {
	case LSM9DS1_GYR_FS_245DPS:
		range = RANGE_245DPS;
		break;
	case LSM9DS1_GYR_FS_500DPS:
		range = RANGE_500DPS;
		break;
	case LSM9DS1_GYR_FS_2000DPS:
		range = RANGE_2000DPS;
		break;
	}
	mutex_unlock(&dev->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *device = to_dev(kobj->parent);
	struct lsm9ds1_acc_gyr_dev *dev = dev_get_drvdata(device);
	unsigned long val;
	u8 range;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 245:
		range = LSM9DS1_GYR_FS_245DPS;
		break;
	case 500:
		range = LSM9DS1_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM9DS1_GYR_FS_2000DPS;
		break;
	default:
		dev_err(dev->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&dev->lock);
	err = lsm9ds1_gyr_update_fs_range(dev, range);
	if (err >= 0)
		dev->pdata_gyr->fs_range = range;
	mutex_unlock(&dev->lock);

	dev_info(dev->dev, "range set to: %lu dps\n", val);

	return size;
}

static struct kobj_attribute poll_attr_acc =
	__ATTR(pollrate_ms, 0664, attr_get_polling_rate_acc, 
						attr_set_polling_rate_acc);
static struct kobj_attribute enable_attr_acc =
	__ATTR(enable_device, 0664, attr_get_enable_acc, attr_set_enable_acc);
static struct kobj_attribute fs_attr_acc =
	__ATTR(range, 0664, attr_get_range_acc, attr_set_range_acc);
static struct kobj_attribute aa_filter_attr  =
	__ATTR(anti_aliasing_frequency, 0664, attr_get_aa_filter, 
							attr_set_aa_filter);
static struct kobj_attribute poll_attr_gyr =
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_gyr, 
						attr_set_polling_rate_gyr);
static struct kobj_attribute enable_attr_gyr =
	__ATTR(enable_device, 0666, attr_get_enable_gyr, attr_set_enable_gyr);
static struct kobj_attribute range_attr_gyr =
	__ATTR(range, 0666, attr_get_range_gyr, attr_set_range_gyr);

static struct attribute *attributes_acc[] = {
	&poll_attr_acc.attr,
	&enable_attr_acc.attr,
	&fs_attr_acc.attr,
	&aa_filter_attr.attr,
	NULL,
};

static struct attribute *attributes_gyr[] = {
	&poll_attr_gyr.attr,
	&enable_attr_gyr.attr,
	&range_attr_gyr.attr,
	NULL,
};

static struct attribute_group attr_group_acc = {
	.attrs = attributes_acc,
};

static struct attribute_group attr_group_gyr = {
	.attrs = attributes_gyr,
};

static int create_sysfs_interfaces(struct device *dev)
{
	int err;

	acc_kobj = kobject_create_and_add("accelerometer", &dev->kobj);
	if(!acc_kobj)
		return -ENOMEM;

	gyr_kobj = kobject_create_and_add("gyroscope", &dev->kobj);
	if(!gyr_kobj)
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

#ifdef CONFIG_OF
static int lsm9ds1_acc_gyr_parse_dt(struct lsm9ds1_acc_gyr_dev *dev,
				    struct device* device)
{
	struct device_node *dn;
	uint8_t i, j;
	uint32_t val, vect[9];

	if (of_match_device(dev->acc_gyr_dt_id, device)) {
		dn = device->of_node;
		dev->pdata_main->of_node = dn;
		
		if (of_property_read_u32_array(dn, "rot-matrix", vect,
			      ARRAY_SIZE(vect)) >= 0) {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					dev->pdata_main->rot_matrix[i][j] =
						(short)vect[3 * j + i];
				}
			}
		} else {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					dev->pdata_main->rot_matrix[i][j] =
			default_lsm9ds1_main_platform_data.rot_matrix[i][j];
				}
			}
		}

		if (!of_property_read_u32(dn, "g-poll-interval", &val)) {
			dev->pdata_gyr->poll_interval = val;
		} else {
			dev->pdata_gyr->poll_interval =
				LSM9DS1_GYR_POLL_INTERVAL_DEF;
		}

		if (!of_property_read_u32(dn, "g-min-interval", &val)) {
			dev->pdata_gyr->min_interval = val;
		} else {
			dev->pdata_gyr->min_interval =
				LSM9DS1_GYR_MIN_POLL_PERIOD_MS;
		}

		if (!of_property_read_u32(dn, "g-fs-range", &val)) {
			dev->pdata_gyr->fs_range = val;
		} else {
			dev->pdata_gyr->fs_range = LSM9DS1_GYR_FS_245DPS;
		}

		if (!of_property_read_u32(dn, "x-poll-interval", &val)) {
			dev->pdata_acc->poll_interval = val;
		} else {
			dev->pdata_acc->poll_interval =
				LSM9DS1_ACC_POLL_INTERVAL_DEF;
		}

		if (!of_property_read_u32(dn, "x-min-interval", &val)) {
			dev->pdata_acc->min_interval = val;
		} else {
			dev->pdata_acc->min_interval =
				LSM9DS1_ACC_MIN_POLL_PERIOD_MS;
		}

		if (!of_property_read_u32(dn, "x-fs-range", &val)) {
			dev->pdata_acc->fs_range = val;
		} else {
			dev->pdata_acc->fs_range = LSM9DS1_ACC_FS_2G;
		}

		if (!of_property_read_u32(dn, "aa-filter-bw", &val)) {
			dev->pdata_acc->fs_range = val;
		} else {
			dev->pdata_acc->fs_range = LSM9DS1_ACC_BW_408;
		}
		return 0;
	}
	return -1;
}
#endif

static irqreturn_t lsm9ds1_acc_gyr_save_timestamp(int irq, void *private)
{
	struct lsm9ds1_acc_gyr_dev *dev;

	dev = (struct lsm9ds1_acc_gyr_dev *)private;

	disable_irq_nosync(irq);

	dev->timestamp = lsm9ds1_get_time_ns();

	return IRQ_WAKE_THREAD;
}

static irqreturn_t lsm9ds1_acc_gyr_thread_fn(int irq, void *private)
{
	u8 data;
	irqreturn_t ret = IRQ_NONE;
	struct lsm9ds1_acc_gyr_dev *dev;

	dev = (struct lsm9ds1_acc_gyr_dev *)private;

	if (dev->tf->read(dev->dev, status_registers.status_reg1.address, 1,
			  &data) < 0)
		goto out;

	if (data & 0x01) {
		int xyz[3] = {}, err;

		err = lsm9ds1_acc_get_data(dev, xyz);
		if (err < 0) {
			dev_err(dev->dev, "get accelerometer data failed\n");
		} else if (++dev->acc_skip_cnt >= dev->acc_dec_cnt) {
			lsm9ds1_acc_report_values(dev, xyz, dev->timestamp);
			dev->acc_skip_cnt = 0;
		}
		ret = IRQ_HANDLED;
	}

	if (data & 0x02) {
		int xyz[3] = {}, err;

		err = lsm9ds1_gyr_get_data(dev, xyz);
		if (err < 0) {
			dev_err(dev->dev, "get gyroscope data failed.\n");
		} else if (++dev->gyr_skip_cnt >= dev->gyr_dec_cnt) {
			lsm9ds1_gyr_report_values(dev, xyz, dev->timestamp);
			dev->gyr_skip_cnt = 0;
		}
		ret = IRQ_HANDLED;
	}

out:
	enable_irq(irq);
	return ret;
}

int lsm9ds1_acc_gyr_probe(struct lsm9ds1_acc_gyr_dev *dev, int irq)
{
	int err;

	mutex_lock(&dev->lock);

	err = lsm9ds1_acc_gyr_check_whoami(dev);
	if (err < 0) {
		mutex_unlock(&dev->lock);
		return err;
	}

	dev->pdata_main = kzalloc(sizeof(*dev->pdata_main), GFP_KERNEL);
	dev->pdata_acc = kzalloc(sizeof(*dev->pdata_acc), GFP_KERNEL);
	dev->pdata_gyr = kzalloc(sizeof(*dev->pdata_gyr), GFP_KERNEL);

	if ((dev->pdata_main == NULL) || (dev->pdata_acc == NULL) ||
	    (dev->pdata_gyr == NULL)){
		err = -ENOMEM;
		dev_err(dev->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}
	dev->pdata_main->pdata_acc = dev->pdata_acc;
	dev->pdata_main->pdata_gyr = dev->pdata_gyr;
#ifdef CONFIG_OF
	lsm9ds1_acc_gyr_parse_dt(dev, dev->dev);
#else
	if (dev->dev->platform_data == NULL) {
		memcpy(dev->pdata_main, 
				&default_lsm9ds1_main_platform_data,
				sizeof(*dev->pdata_main));
		memcpy(dev->pdata_acc, &default_lsm9ds1_acc_pdata,
						sizeof(*dev->pdata_acc));
		memcpy(dev->pdata_gyr, &default_lsm9ds1_gyr_pdata,
						sizeof(*dev->pdata_gyr));
		dev_info(dev->dev, "using default plaform_data for "
					"accelerometer and gyroscope\n");
	} else {
		struct lsm9ds1_acc_gyr_main_platform_data *platform_data;
		platform_data = dev->dev->platform_data;

		if(platform_data == NULL) {
			memcpy(dev->pdata_main,
				&default_lsm9ds1_main_platform_data,
				sizeof(*dev->pdata_main));
			dev_info(dev->dev, "using default plaform_data"
							" for accelerometer\n");
		} else {
			memcpy(dev->pdata_main, platform_data,
						sizeof(*dev->pdata_acc));
		}

		if(platform_data->pdata_acc == NULL) {
			memcpy(dev->pdata_acc, &default_lsm9ds1_acc_pdata,
						sizeof(*dev->pdata_acc));
			dev_info(dev->dev, "using default plaform_data"
							" for accelerometer\n");
		} else {
			memcpy(dev->pdata_acc, platform_data->pdata_acc,
						sizeof(*dev->pdata_acc));
		}

		if(platform_data->pdata_gyr == NULL) {
			memcpy(dev->pdata_gyr, &default_lsm9ds1_gyr_pdata,
						sizeof(*dev->pdata_gyr));
			dev_info(dev->dev, "using default plaform_data"
							" for gyroscope\n");
		} else {
			memcpy(dev->pdata_gyr, platform_data->pdata_gyr,
						sizeof(*dev->pdata_gyr));
		}
	}
#endif

	err = lsm9ds1_acc_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate platform data for "
							"accelerometer \n");
		goto exit_kfree_pdata;
	}

	err = lsm9ds1_gyr_validate_pdata(dev);
	if (err < 0) {
		dev_err(dev->dev, "failed to validate platform data for "
							"gyroscope\n");
		goto exit_kfree_pdata;
	}

	if (dev->pdata_acc->init) {
		err = dev->pdata_acc->init();
		if (err < 0) {
			dev_err(dev->dev, "accelerometer init failed: "
								"%d\n", err);
			goto err_pdata_acc_init;
		}
	}
	if (dev->pdata_gyr->init) {
		err = dev->pdata_gyr->init();
		if (err < 0) {
			dev_err(dev->dev, "magnetometer init failed: "
								"%d\n", err);
			goto err_pdata_gyr_init;
		}
	}

	err = lsm9ds1_acc_gyr_hw_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm9ds1_acc_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "accelerometer power on failed: "
								"%d\n", err);
		goto err_pdata_init;
	}

	err = lsm9ds1_gyr_device_power_on(dev);
	if (err < 0) {
		dev_err(dev->dev, "gyroscope power on failed: "
								"%d\n", err);
		goto err_pdata_init;
	}

	err = lsm9ds1_acc_update_fs_range(dev, dev->pdata_acc->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update accelerometer full scale range "
								"failed\n");
		goto  err_power_off_acc;
	}

	err = lsm9ds1_gyr_update_fs_range(dev, dev->pdata_gyr->fs_range);
	if (err < 0) {
		dev_err(dev->dev, "update gyroscope full scale range "
								"failed\n");
		goto  err_power_off_gyr;
	}

	err = lsm9ds1_acc_update_odr(dev, dev->pdata_acc->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update accelerometer ODR failed\n");
		goto  err_power_off;
	}

	err = lsm9ds1_gyr_update_odr(dev, dev->pdata_gyr->poll_interval);
	if (err < 0) {
		dev_err(dev->dev, "update gyroscope ODR failed\n");
		goto  err_power_off;
	}

	err = lsm9ds1_acc_update_filter(dev, 
					dev->pdata_acc->aa_filter_bandwidth);
	if (err < 0) {
		dev_err(dev->dev, "update accelerometer filter "
								"failed\n");
		goto  err_power_off;
	}

	err = lsm9ds1_acc_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "accelerometer input init failed\n");
		goto err_power_off;
	}

	err = lsm9ds1_gyr_input_init(dev);
	if (err < 0) {
		dev_err(dev->dev, "gyroscope input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(dev->dev);
	if (err < 0) {
		dev_err(dev->dev, "device %s sysfs register failed\n",
			LSM9DS1_ACC_GYR_DEV_NAME);
		goto err_input_cleanup;
	}

	lsm9ds1_acc_device_power_off(dev);
	lsm9ds1_gyr_device_power_off(dev);

	if (irq > 0) {
		dev->irq = irq;
		err = request_threaded_irq(irq, lsm9ds1_acc_gyr_save_timestamp,
					   lsm9ds1_acc_gyr_thread_fn,
					   IRQF_TRIGGER_HIGH, dev->name, dev);
		if (err)
			return err;

		disable_irq(irq);
	} else {
		dev_err(dev->dev, "%s: missing IRQ line\n", __func__);
		err = -EINVAL;
		goto err_input_cleanup;
	}

	mutex_unlock(&dev->lock);

	dev_info(dev->dev, "%s: probed\n", LSM9DS1_ACC_GYR_DEV_NAME);
	return 0;

err_input_cleanup:
	lsm9ds1_input_cleanup(dev);
err_power_off:
err_power_off_gyr:
	lsm9ds1_gyr_device_power_off(dev);
err_power_off_acc:
	lsm9ds1_acc_device_power_off(dev);
err_hw_init:
err_pdata_init:
err_pdata_gyr_init:
	if (dev->pdata_gyr->exit)
		dev->pdata_gyr->exit();
err_pdata_acc_init:
	if (dev->pdata_acc->exit)
		dev->pdata_acc->exit();
exit_kfree_pdata:
	kfree(dev->pdata_acc);
	kfree(dev->pdata_gyr);
err_mutexunlock:
	mutex_unlock(&dev->lock);

	return err;
}
EXPORT_SYMBOL(lsm9ds1_acc_gyr_probe);

int lsm9ds1_acc_gyr_remove(struct lsm9ds1_acc_gyr_dev *dev)
{
	if (atomic_read(&dev->enabled_gyr)) {
		lsm9ds1_gyr_disable(dev);
		lsm9ds1_gyr_input_cleanup(dev);

		if (dev->pdata_gyr->exit)
			dev->pdata_gyr->exit();
	}

	lsm9ds1_acc_disable(dev);
	lsm9ds1_acc_input_cleanup(dev);

	remove_sysfs_interfaces(dev->dev);

	if (dev->pdata_acc->exit)
		dev->pdata_acc->exit();

	kfree(dev->pdata_acc);
	kfree(dev->pdata_gyr);
	kfree(dev->pdata_main);

	return 0;
}
EXPORT_SYMBOL(lsm9ds1_acc_gyr_remove);

MODULE_DESCRIPTION("lsm9ds1 accelerometer and gyroscope driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");
