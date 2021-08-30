/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name		: lsm330_acc.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Lorenzo Bianconi (lorenzo.bianconi@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V.1.2.6.1
 * Date			: 2016/May/24
 * Description		: LSM330 accelerometer driver
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
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
	V 1.2.0		Registers names compliant to correct datasheet
	V.1.2.1		Removed enable_interrupt_output sysfs file, manages int1
			and int2, implements int1 isr.
	V.1.2.2		Added HR_Timer and custom sysfs path
	V.1.2.3		Ch state program codes and state prog parameters defines
	V.1.2.5		Changes create_sysfs_interfaces
	V.1.2.6		Changes resume and suspend functions
	V.1.2.6.1	Introduce SignMotion feat implementation and solves
			acc suspend/resume issue;
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include "lsm330.h"

//#define LSM330_DEBUG			1

#define LOAD_SM1_PROGRAM	0
#define LOAD_SM1_PARAMETERS	0
#define LOAD_SM2_PROGRAM	1
#define LOAD_SM2_PARAMETERS	1


#if ENABLE_SIGNIFICANT_MOTION > 0
/* customized thresholds for significant motion program*/
#define THRS1_2_02G 0x09
#define THRS1_2_04G 0x05
#define THRS1_2_06G 0x03
#define THRS1_2_08G 0x02
#define THRS1_2_16G 0x01
#endif

#define G_MAX			23920640	/* ug */
#define MS_TO_NS(x)		(x*1000000L)

#define SENSITIVITY_2G		60		/* ug/LSB	*/
#define SENSITIVITY_4G		120		/* ug/LSB	*/
#define SENSITIVITY_6G		180		/* ug/LSB	*/
#define SENSITIVITY_8G		240		/* ug/LSB	*/
#define SENSITIVITY_16G		730		/* ug/LSB	*/

#define	LSM330_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define LSM330_ODR_MASK		(0xF0)
#define LSM330_PM_OFF		(0x00)		/* OFF */
#define LSM330_ODR3_125		(0x10)		/*    3.125 Hz */
#define LSM330_ODR6_25		(0x20)		/*    6.25  Hz */
#define LSM330_ODR12_5		(0x30)		/*   12.5   Hz */
#define LSM330_ODR25		(0x40)		/*   25     Hz */
#define LSM330_ODR50		(0x50)		/*   50     Hz */
#define LSM330_ODR100		(0x60)		/*  100     Hz */
#define LSM330_ODR400		(0x70)		/*  400     Hz */
#define LSM330_ODR800		(0x80)		/*  800     Hz */
#define LSM330_ODR1600		(0x90)		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREGx */
#define LSM330_INTEN_MASK		(0x01)
#define LSM330_INTEN_OFF		(0x00)
#define LSM330_INTEN_ON			(0x01)

/* CTRLREG1 */
#define LSM330_HIST1_MASK		(0xE0)
#define LSM330_SM1INT_PIN_MASK		(0x08)
#define LSM330_SM1INT_PININT2		(0x08)
#define LSM330_SM1INT_PININT1		(0x00)
#define LSM330_SM1_EN_MASK		(0x01)
#define LSM330_SM1_EN_ON		(0x01)
#define LSM330_SM1_EN_OFF		(0x00)
/* */

/* CTRLREG2 */
#define LSM330_HIST2_MASK		(0xE0)
#define LSM330_SM2INT_PIN_MASK		(0x08)
#define LSM330_SM2INT_PININT2		(0x08)
#define LSM330_SM2INT_PININT1		(0x00)
#define LSM330_SM2_EN_MASK		(0x01)
#define LSM330_SM2_EN_ON		(0x01)
#define LSM330_SM2_EN_OFF		(0x00)
/* */

/* CTRLREG3 */
#define LSM330_INT_ACT_MASK		(0x01 << 6)
#define LSM330_INT_ACT_H		(0x01 << 6)
#define LSM330_INT_ACT_L		(0x00)

#define LSM330_INT2_EN_MASK		(0x01 << 4)
#define LSM330_INT2_EN_ON		(0x01 << 4)
#define LSM330_INT2_EN_OFF		(0x00)

#define LSM330_INT1_EN_MASK		(0x01 << 3)
#define LSM330_INT1_EN_ON		(0x01 << 3)
#define LSM330_INT1_EN_OFF		(0x00)
/* */

/* CTRLREG4 */
#define LSM330_BDU_EN			(0x08)
#define LSM330_ALL_AXES			(0x07)
/* */

/* STATUS REG BITS */
#define LSM330_STAT_INTSM1_BIT		(0x01 << 3)
#define LSM330_STAT_INTSM2_BIT		(0x01 << 2)

#define OUT_AXISDATA_REG		LSM330_OUTX_L
#define WHOAMI_LSM330_ACC		(0x40)	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define LSM330_WHO_AM_I			(0x0F)	/* WhoAmI register Address */

#define LSM330_OUTX_L			(0x28)	/* Output X LSByte */
#define LSM330_OUTX_H			(0x29)	/* Output X MSByte */
#define LSM330_OUTY_L			(0x2A)	/* Output Y LSByte */
#define LSM330_OUTY_H			(0x2B)	/* Output Y MSByte */
#define LSM330_OUTZ_L			(0x2C)	/* Output Z LSByte */
#define LSM330_OUTZ_H			(0x2D)	/* Output Z MSByte */
#define LSM330_LC_L			(0x16)	/* LSByte Long Counter Status */
#define LSM330_LC_H			(0x17)	/* MSByte Long Counter Status */

#define LSM330_INTERR_STAT		(0x18)	/* Interrupt Status */

#define LSM330_STATUS_REG		(0x27)	/* Status */

#define LSM330_CTRL_REG1		(0x21)	/* control reg 1 */
#define LSM330_CTRL_REG2		(0x22)	/* control reg 2 */
#define LSM330_CTRL_REG3		(0x23)	/* control reg 3 */
#define LSM330_CTRL_REG4		(0x20)	/* control reg 4 */
#define LSM330_CTRL_REG5		(0x24)	/* control reg 5 */
#define LSM330_CTRL_REG6		(0x25)	/* control reg 6 */

#define LSM330_OFF_X			(0x10)	/* Offset X Corr */
#define LSM330_OFF_Y			(0x11)	/* Offset Y Corr */
#define LSM330_OFF_Z			(0x12)	/* Offset Z Corr */

#define LSM330_CS_X			(0x13)	/* Const Shift X */
#define LSM330_CS_Y			(0x14)	/* Const Shift Y */
#define LSM330_CS_Z			(0x15)	/* Const Shift Z */

#define LSM330_VFC_1			(0x1B)	/* Vect Filter Coeff 1 */
#define LSM330_VFC_2			(0x1C)	/* Vect Filter Coeff 2 */
#define LSM330_VFC_3			(0x1D)	/* Vect Filter Coeff 3 */
#define LSM330_VFC_4			(0x1E)	/* Vect Filter Coeff 4 */


	/* state machine 1 program */
#define LSM330_STATEPR1		(0x40)	/*	State Program 1 16 bytes */
	/* state machine 1 params */
#define LSM330_TIM4_1		(0x50)	/*	SPr1 Timer4		*/
#define LSM330_TIM3_1		(0x51)	/*	SPr1 Timer3		*/
#define LSM330_TIM2_1		(0x52)	/*	SPr1 Timer2	2bytes	*/
//
#define LSM330_TIM1_1		(0x54)	/*	SPr1 Timer1	2bytes	*/
//
#define LSM330_THRS2_1		(0x56)	/*	SPr1 Threshold2		*/
#define LSM330_THRS1_1		(0x57)	/*	SPr1 Threshold1		*/
#define LSM330_SA_1		(0x59)	/*	SPr1 Swap Axis Sign Msk	*/
#define LSM330_MA_1		(0x5A)	/*	SPr1 Axis Sign Msk	*/
#define LSM330_SETT_1		(0x5B)	/*	SPr1 			*/
#define LSM330_PPRP_1		(0x5C)	/*	SPr1 ProgPointer ResetPointer */
#define LSM330_TC_1		(0x5D)	/*	SPr1 		2bytes	*/
#define LSM330_OUTS_1		(0x5F)	/*	SPr1 			*/

	/* state machine 2 program */
#define LSM330_STATEPR2	(0x60)	/*	State Program 2 16 bytes */
	/* state machine 2 params */
#define LSM330_TIM4_2		(0x70)	/*	SPr2 Timer4		*/
#define LSM330_TIM3_2		(0x71)	/*	SPr2 Timer3		*/
#define LSM330_TIM2_2		(0x72)	/*	SPr2 Timer2	2bytes	*/
//
#define LSM330_TIM1_2		(0x74)	/*	SPr2 Timer1	2bytes	*/
//
#define LSM330_THRS2_2		(0x76)	/*	SPr2 Threshold2		*/
#define LSM330_THRS1_2		(0x77)	/*	SPr2 Threshold1		*/
#define LSM330_DES_2		(0x78)	/*	SPr2 Decimation		*/
#define LSM330_SA_2		(0x79)	/*	SPr2 Swap Axis Sign Msk	*/
#define LSM330_MA_2		(0x7A)	/*	SPr2 Axis Sign Msk	*/
#define LSM330_SETT_2		(0x7B)	/*	SPr2 			*/
#define LSM330_PPRP_2		(0x7C)	/*	SPr2 ProgPointer ResetPointer */
#define LSM330_TC_2		(0x7D)	/*	SPr2 		2bytes	*/
//
#define LSM330_OUTS_2		(0x7F)	/*	SPr2 			*/
/*	end CONTROL REGISTRES	*/


/* RESUME STATE INDICES */
#define RES_LSM330_LC_L				0
#define RES_LSM330_LC_H				1

#define RES_LSM330_CTRL_REG4			2
#define RES_LSM330_CTRL_REG1			3
#define RES_LSM330_CTRL_REG2			4
#define RES_LSM330_CTRL_REG3			5
#define RES_LSM330_CTRL_REG5			6
#define RES_LSM330_CTRL_REG6			7

#define RES_LSM330_OFF_X			8
#define RES_LSM330_OFF_Y			9
#define RES_LSM330_OFF_Z			10

#define RES_LSM330_CS_X				11
#define RES_LSM330_CS_Y				12
#define RES_LSM330_CS_Z				13

#define RES_LSM330_VFC_1			14
#define RES_LSM330_VFC_2			15
#define RES_LSM330_VFC_3			16
#define RES_LSM330_VFC_4			17

#define RES_LSM330_THRS3			18

#define RES_LSM330_TIM4_1			20
#define RES_LSM330_TIM3_1			21
#define RES_LSM330_TIM2_1_L			22
#define RES_LSM330_TIM2_1_H			23
#define RES_LSM330_TIM1_1_L			24
#define RES_LSM330_TIM1_1_H			25

#define RES_LSM330_THRS2_1			26
#define RES_LSM330_THRS1_1			27
#define RES_LSM330_SA_1				28
#define RES_LSM330_MA_1				29
#define RES_LSM330_SETT_1			30

#define RES_LSM330_TIM4_2			31
#define RES_LSM330_TIM3_2			32
#define RES_LSM330_TIM2_2_L			33
#define RES_LSM330_TIM2_2_H			34
#define RES_LSM330_TIM1_2_L			35
#define RES_LSM330_TIM1_2_H			36

#define RES_LSM330_THRS2_2			37
#define RES_LSM330_THRS1_2			38
#define RES_LSM330_DES_2			39
#define RES_LSM330_SA_2				40
#define RES_LSM330_MA_2				41
#define RES_LSM330_SETT_2			42

/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define LSM330_SM1_DIS_SM2_DIS			(0x00)
#define LSM330_SM1_EN_SM2_DIS			(0x01)
#define LSM330_SM1_DIS_SM2_EN			(0x02)
#define LSM330_SM1_EN_SM2_EN			(0x03)

/* INTERRUPTS ENABLE CONTROLS */
#define LSM330_INT1_DIS_INT2_DIS		(0x00)
#define LSM330_INT1_EN_INT2_DIS			(0x01)
#define LSM330_INT1_DIS_INT2_EN			(0x02)
#define LSM330_INT1_EN_INT2_EN			(0x03)

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330_acc_odr_table[] = {
		{    1, LSM330_ODR1600 },
		{    3, LSM330_ODR400  },
		{   10, LSM330_ODR100  },
		{   20, LSM330_ODR50   },
		{   40, LSM330_ODR25   },
		{   80, LSM330_ODR12_5 },
		{  160, LSM330_ODR6_25 },
		{  320, LSM330_ODR3_125},
};

static struct lsm330_acc_platform_data default_lsm330_acc_pdata = {
	.fs_range = LSM330_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 10,
	.min_interval = LSM330_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM330_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_ACC_DEFAULT_INT2_GPIO,
};

static int int1_gpio = LSM330_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);
MODULE_PARM_DESC(int1_gpio, "integer: gpio number being assined to interrupt PIN1");
MODULE_PARM_DESC(int2_gpio, "integer: gpio number being assined to interrupt PIN2");

/* sets default init values to be written in registers at probe stage */
static void lsm330_acc_set_init_register_values(struct lsm330_acc_data *acc)
{
	acc->resume_state[RES_LSM330_LC_L] = 0x00;
	acc->resume_state[RES_LSM330_LC_H] = 0x00;

	acc->resume_state[RES_LSM330_CTRL_REG1] = (0x00 | LSM330_SM1INT_PININT1);
	acc->resume_state[RES_LSM330_CTRL_REG2] = (0x00 | LSM330_SM2INT_PININT1);
	acc->resume_state[RES_LSM330_CTRL_REG3] = LSM330_INT_ACT_H;
	if(acc->pdata->gpio_int1 >= 0)
		acc->resume_state[RES_LSM330_CTRL_REG3] =
				acc->resume_state[RES_LSM330_CTRL_REG3] | \
					LSM330_INT1_EN_ON;
	if(acc->pdata->gpio_int2 >= 0)
		acc->resume_state[RES_LSM330_CTRL_REG3] =
				acc->resume_state[RES_LSM330_CTRL_REG3] | \
					LSM330_INT2_EN_ON;

	acc->resume_state[RES_LSM330_CTRL_REG4] = (LSM330_BDU_EN |
							LSM330_ALL_AXES);
	acc->resume_state[RES_LSM330_CTRL_REG5] = 0x00;
	acc->resume_state[RES_LSM330_CTRL_REG6] = 0x10;

	acc->resume_state[RES_LSM330_THRS3] = 0x00;
	acc->resume_state[RES_LSM330_OFF_X] = 0x00;
	acc->resume_state[RES_LSM330_OFF_Y] = 0x00;
	acc->resume_state[RES_LSM330_OFF_Z] = 0x00;

	acc->resume_state[RES_LSM330_CS_X] = 0x00;
	acc->resume_state[RES_LSM330_CS_Y] = 0x00;
	acc->resume_state[RES_LSM330_CS_Z] = 0x00;

	acc->resume_state[RES_LSM330_VFC_1] = 0x00;
	acc->resume_state[RES_LSM330_VFC_2] = 0x00;
	acc->resume_state[RES_LSM330_VFC_3] = 0x00;
	acc->resume_state[RES_LSM330_VFC_4] = 0x00;
}

static void lsm330_acc_set_init_statepr1_inst(struct lsm330_acc_data *acc)
{
/* loads custom state program1 */
#if LOAD_SM1_PROGRAM > 0
	acc->resume_stmach_program1[0] = 0x00;
	acc->resume_stmach_program1[1] = 0x00;
	acc->resume_stmach_program1[2] = 0x00;
	acc->resume_stmach_program1[3] = 0x00;
	acc->resume_stmach_program1[4] = 0x00;
	acc->resume_stmach_program1[5] = 0x00;
	acc->resume_stmach_program1[6] = 0x00;
	acc->resume_stmach_program1[7] = 0x00;
	acc->resume_stmach_program1[8] = 0x00;
	acc->resume_stmach_program1[9] = 0x00;
	acc->resume_stmach_program1[10] = 0x00;
	acc->resume_stmach_program1[11] = 0x00;
	acc->resume_stmach_program1[12] = 0x00;
	acc->resume_stmach_program1[13] = 0x00;
	acc->resume_stmach_program1[14] = 0x00;
	acc->resume_stmach_program1[15] = 0x00;
#else /* loads default state program1 */
	acc->resume_stmach_program1[0] = 0x00;
	acc->resume_stmach_program1[1] = 0x00;
	acc->resume_stmach_program1[2] = 0x00;
	acc->resume_stmach_program1[3] = 0x00;
	acc->resume_stmach_program1[4] = 0x00;
	acc->resume_stmach_program1[5] = 0x00;
	acc->resume_stmach_program1[6] = 0x00;
	acc->resume_stmach_program1[7] = 0x00;
	acc->resume_stmach_program1[8] = 0x00;
	acc->resume_stmach_program1[9] = 0x00;
	acc->resume_stmach_program1[10] = 0x00;
	acc->resume_stmach_program1[11] = 0x00;
	acc->resume_stmach_program1[12] = 0x00;
	acc->resume_stmach_program1[13] = 0x00;
	acc->resume_stmach_program1[14] = 0x00;
	acc->resume_stmach_program1[15] = 0x00;
#endif /* LOAD_SM1_PROGRAM */
}

static void lsm330_acc_set_init_statepr2_inst(struct lsm330_acc_data *acc)
{
/* loads custom state program2 */
#if LOAD_SM2_PROGRAM > 0
#if ENABLE_SIGNIFICANT_MOTION > 0
	acc->resume_stmach_program2[0] = 0x05;
	acc->resume_stmach_program2[1] = 0x03;
	acc->resume_stmach_program2[2] = 0x04;
	acc->resume_stmach_program2[3] = 0x45;
	acc->resume_stmach_program2[4] = 0x00;
	acc->resume_stmach_program2[5] = 0x00;
	acc->resume_stmach_program2[6] = 0x00;
	acc->resume_stmach_program2[7] = 0x00;
	acc->resume_stmach_program2[8] = 0x00;
	acc->resume_stmach_program2[9] = 0x00;
	acc->resume_stmach_program2[10] = 0x00;
	acc->resume_stmach_program2[11] = 0x00;
	acc->resume_stmach_program2[12] = 0x00;
	acc->resume_stmach_program2[13] = 0x00;
	acc->resume_stmach_program2[14] = 0x00;
	acc->resume_stmach_program2[15] = 0x00;
#else
	acc->resume_stmach_program2[0] = 0x00;
	acc->resume_stmach_program2[1] = 0x00;
	acc->resume_stmach_program2[2] = 0x00;
	acc->resume_stmach_program2[3] = 0x00;
	acc->resume_stmach_program2[4] = 0x00;
	acc->resume_stmach_program2[5] = 0x00;
	acc->resume_stmach_program2[6] = 0x00;
	acc->resume_stmach_program2[7] = 0x00;
	acc->resume_stmach_program2[8] = 0x00;
	acc->resume_stmach_program2[9] = 0x00;
	acc->resume_stmach_program2[10] = 0x00;
	acc->resume_stmach_program2[11] = 0x00;
	acc->resume_stmach_program2[12] = 0x00;
	acc->resume_stmach_program2[13] = 0x00;
	acc->resume_stmach_program2[14] = 0x00;
	acc->resume_stmach_program2[15] = 0x00;
#endif /* ENABLE_SIGNIFICANT_MOTION */
#else /* loads default state program2 */
	acc->resume_stmach_program2[0] = 0x00;
	acc->resume_stmach_program2[1] = 0x00;
	acc->resume_stmach_program2[2] = 0x00;
	acc->resume_stmach_program2[3] = 0x00;
	acc->resume_stmach_program2[4] = 0x00;
	acc->resume_stmach_program2[5] = 0x00;
	acc->resume_stmach_program2[6] = 0x00;
	acc->resume_stmach_program2[7] = 0x00;
	acc->resume_stmach_program2[8] = 0x00;
	acc->resume_stmach_program2[9] = 0x00;
	acc->resume_stmach_program2[10] = 0x00;
	acc->resume_stmach_program2[11] = 0x00;
	acc->resume_stmach_program2[12] = 0x00;
	acc->resume_stmach_program2[13] = 0x00;
	acc->resume_stmach_program2[14] = 0x00;
	acc->resume_stmach_program2[15] = 0x00;
#endif /* LOAD_SM2_PROGRAM */
}

static void lsm330_acc_set_init_statepr1_param(struct lsm330_acc_data *acc)
{
/* loads custom state prog1 parameters */
#if LOAD_SM1_PARAMETERS > 0
	acc->resume_state[RES_LSM330_TIM4_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_1] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_1] = 0x00;
	/* DES1 not available*/
	acc->resume_state[RES_LSM330_SA_1] = 0x00;
	acc->resume_state[RES_LSM330_MA_1] = 0x00;
	acc->resume_state[RES_LSM330_SETT_1] = 0x00;
#else 	/* loads default state prog1 parameters */
	acc->resume_state[RES_LSM330_TIM4_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_1] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_1_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_1_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_1] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_1] = 0x00;
	/* DES1 not available*/
	acc->resume_state[RES_LSM330_SA_1] = 0x00;
	acc->resume_state[RES_LSM330_MA_1] = 0x00;
	acc->resume_state[RES_LSM330_SETT_1] = 0x00;
#endif
}

static void lsm330_acc_set_init_statepr2_param(struct lsm330_acc_data *acc)
{
/* loads custom state prog2 parameters */
#if LOAD_SM2_PARAMETERS > 0
#if ENABLE_SIGNIFICANT_MOTION > 0
	acc->resume_state[RES_LSM330_TIM4_2] = 0x64;
	acc->resume_state[RES_LSM330_TIM3_2] = 0xC8;
	acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_2] = THRS1_2_02G;
	acc->resume_state[RES_LSM330_DES_2] = 0x00;
	acc->resume_state[RES_LSM330_SA_2] = 0xA8;
	acc->resume_state[RES_LSM330_MA_2] = 0xA8;
	acc->resume_state[RES_LSM330_SETT_2] = 0x13;
#else
	acc->resume_state[RES_LSM330_TIM4_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_2] = 0x00;
	acc->resume_state[RES_LSM330_DES_2] = 0x00;
	acc->resume_state[RES_LSM330_SA_2] = 0x00;
	acc->resume_state[RES_LSM330_MA_2] = 0x00;
	acc->resume_state[RES_LSM330_SETT_2] = 0x00;
#endif  /* ENABLE_SIGNIFICANT_MOTION */
#else	/* loads default state prog2 parameters */
	acc->resume_state[RES_LSM330_TIM4_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM3_2] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LSM330_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LSM330_THRS2_2] = 0x00;
	acc->resume_state[RES_LSM330_THRS1_2] = 0x00;
	acc->resume_state[RES_LSM330_DES_2] = 0x00;
	acc->resume_state[RES_LSM330_SA_2] = 0x00;
	acc->resume_state[RES_LSM330_MA_2] = 0x00;
	acc->resume_state[RES_LSM330_SETT_2] = 0x00;
#endif
}

static int lsm330_acc_update(struct lsm330_acc_data *acc,
			     u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 rdbuf[1];
	u8 wrbuf[1];

	u8 init_val;
	u8 updated_val;
	err = acc->tf->read(acc->dev, reg_address, 1, rdbuf);
	if (!(err < 0)) {
		init_val = rdbuf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		wrbuf[0] = updated_val;
		err = acc->tf->write(acc->dev, reg_address, 1, wrbuf);
	}
	return err;
}

static int lsm330_acc_hw_init(struct lsm330_acc_data *acc)
{
	int i;
	int err = -1;
	u8 buf[17];

	pr_info("%s: hw init start\n", LSM330_ACC_DEV_NAME);

	err = acc->tf->read(acc->dev, LSM330_WHO_AM_I, 1, buf);
	if (err < 0) {
	dev_warn(acc->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_LSM330_ACC) {
	dev_err(acc->dev,
		"device unknown. Expected: 0x%02x,"
		" Replies: 0x%02x\n", WHOAMI_LSM330_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}


	buf[0] = acc->resume_state[RES_LSM330_LC_L];
	buf[1] = acc->resume_state[RES_LSM330_LC_H];
	err = acc->tf->write(acc->dev, LSM330_LC_L, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LSM330_TIM4_1];
	buf[1] = acc->resume_state[RES_LSM330_TIM3_1];
	buf[2] = acc->resume_state[RES_LSM330_TIM2_1_L];
	buf[3] = acc->resume_state[RES_LSM330_TIM2_1_H];
	buf[4] = acc->resume_state[RES_LSM330_TIM1_1_L];
	buf[5] = acc->resume_state[RES_LSM330_TIM1_1_H];
	buf[6] = acc->resume_state[RES_LSM330_THRS2_1];
	buf[7] = acc->resume_state[RES_LSM330_THRS1_1];
	err = acc->tf->write(acc->dev, LSM330_TIM4_1, 8, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LSM330_SA_1];
	buf[1] = acc->resume_state[RES_LSM330_MA_1];
	buf[2] = acc->resume_state[RES_LSM330_SETT_1];
	err = acc->tf->write(acc->dev, LSM330_SA_1, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LSM330_TIM4_2];
	buf[1] = acc->resume_state[RES_LSM330_TIM3_2];
	buf[2] = acc->resume_state[RES_LSM330_TIM2_2_L];
	buf[3] = acc->resume_state[RES_LSM330_TIM2_2_H];
	buf[4] = acc->resume_state[RES_LSM330_TIM1_2_L];
	buf[5] = acc->resume_state[RES_LSM330_TIM1_2_H];
	buf[6] = acc->resume_state[RES_LSM330_THRS2_2];
	buf[7] = acc->resume_state[RES_LSM330_THRS1_2];
	buf[8] = acc->resume_state[RES_LSM330_DES_2];
	buf[9] = acc->resume_state[RES_LSM330_SA_2];
	buf[10] = acc->resume_state[RES_LSM330_MA_2];
	buf[11] = acc->resume_state[RES_LSM330_SETT_2];
	err = acc->tf->write(acc->dev, LSM330_TIM4_2, 12, buf);
	if (err < 0)
		goto err_resume_state;

	/*	state program 1 */
	for (i = 0; i < LSM330_STATE_PR_SIZE; i++) {
		buf[i] = acc->resume_stmach_program1[i];
		pr_debug("i=%d,sm pr1 buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = acc->tf->write(acc->dev, LSM330_STATEPR1, LSM330_STATE_PR_SIZE,
			     buf);
	if (err < 0)
		goto err_resume_state;

	/*	state program 2 */
	for(i = 0; i < LSM330_STATE_PR_SIZE; i++){
		buf[i] = acc->resume_stmach_program2[i];
		pr_debug("i=%d,sm pr2 buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = acc->tf->write(acc->dev, LSM330_STATEPR2, LSM330_STATE_PR_SIZE,
			     buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LSM330_CTRL_REG5];
	buf[1] = acc->resume_state[RES_LSM330_CTRL_REG6];
	err = acc->tf->write(acc->dev, LSM330_CTRL_REG5, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LSM330_CTRL_REG1];
	buf[1] = acc->resume_state[RES_LSM330_CTRL_REG2];
	buf[2] = acc->resume_state[RES_LSM330_CTRL_REG3];
	err = acc->tf->write(acc->dev, LSM330_CTRL_REG1, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LSM330_CTRL_REG4];
	err = acc->tf->write(acc->dev, LSM330_CTRL_REG4, 1, buf);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM330_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(acc->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm330_acc_device_power_off(struct lsm330_acc_data *acc)
{
	int err;

	err = lsm330_acc_update(acc, LSM330_CTRL_REG4,
				LSM330_ODR_MASK, LSM330_PM_OFF);
	if (err < 0)
		dev_err(acc->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if(acc->pdata->gpio_int1)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lsm330_acc_device_power_on(struct lsm330_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(acc->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = lsm330_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lsm330_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

#if ENABLE_SIGNIFICANT_MOTION > 0
static void lsm330_acc_signMotion_interrupt_action(struct lsm330_acc_data *status)
{
	input_event(status->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_SM, 1);
	input_sync(status->input_dev);
	input_event(status->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_SM, 0);
	input_sync(status->input_dev);

	pr_debug("%s: sign Motion event\n", LSM330_ACC_DEV_NAME);
	atomic_set(&status->sign_mot_enabled, 0);
}
#endif

static irqreturn_t lsm330_acc_isr1(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	pr_debug("%s: isr1 queued\n", LSM330_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lsm330_acc_isr2(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	pr_debug("%s: isr2 queued\n", LSM330_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm330_acc_irq1_work_func(struct work_struct *work)
{

	int err = -1;
	u8 rbuf[2], status;
	struct lsm330_acc_data *acc;

	acc = container_of(work, struct lsm330_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_int_source(acc); */
	pr_debug("%s: IRQ1 triggered\n", LSM330_ACC_DEV_NAME);
	/*  */

	mutex_lock(&acc->lock);
	err = acc->tf->read(acc->dev, LSM330_INTERR_STAT, 1, rbuf);
	pr_debug("%s: INTERR_STAT_REG: 0x%02x\n",
					LSM330_ACC_DEV_NAME, rbuf[0]);
	status = rbuf[0];
	if(status & LSM330_STAT_INTSM1_BIT) {
		pr_debug("%s: SM1 interrupt \n", LSM330_ACC_DEV_NAME);

		/* mandatory to unlatch SM2 interrupt */
		err = acc->tf->read(acc->dev, LSM330_OUTS_1, 1, rbuf);
		pr_debug("%s: OUTS_1: 0x%02x\n",
					LSM330_ACC_DEV_NAME, rbuf[0]);

	}
	if(status & LSM330_STAT_INTSM2_BIT) {
		pr_debug("%s: SM2 interrupt \n", LSM330_ACC_DEV_NAME);

#if ENABLE_SIGNIFICANT_MOTION > 0
		lsm330_acc_signMotion_interrupt_action(acc);
#endif
		/* mandatory to unlatch SM2 interrupt */
		err = acc->tf->read(acc->dev, LSM330_OUTS_2, 1, rbuf);
		pr_debug("%s: OUTS_2: 0x%02x\n",
					LSM330_ACC_DEV_NAME, rbuf[0]);
	}
	pr_debug("%s: IRQ1 served\n", LSM330_ACC_DEV_NAME);
	mutex_unlock(&acc->lock);

	enable_irq(acc->irq1);
	pr_debug("%s: IRQ1 re-enabled\n", LSM330_ACC_DEV_NAME);
}

static void lsm330_acc_irq2_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;

	acc = container_of(work, struct lsm330_acc_data, irq2_work);
	pr_debug("%s: IRQ2 triggered\n", LSM330_ACC_DEV_NAME);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_stat_source(acc); */
	/* ; */
	pr_debug("%s: IRQ2 served\n", LSM330_ACC_DEV_NAME);

	enable_irq(acc->irq2);
	pr_debug("%s: IRQ2 re-enabled\n", LSM330_ACC_DEV_NAME);
}

static int lsm330_acc_register_masked_update(struct lsm330_acc_data *acc,
		u8 reg_address, u8 mask, u8 new_bit_values, int resume_index)
{
	u8 config[0];
	u8 init_val, updated_val;
	int err;
	int step = 0;

	err = acc->tf->read(acc->dev, reg_address, 1, config);
	if (err < 0)
		goto error;
	init_val = config[0];
	acc->resume_state[resume_index] = init_val;
	step = 1;
	updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
	config[0] = updated_val;
	err = acc->tf->write(acc->dev, reg_address, 1, config);
	if (err < 0)
		goto error;
	acc->resume_state[resume_index] = updated_val;

	return err;
	error:
		dev_err(acc->dev,
			"register 0x%02x update failed at step %d, error: %d\n",
				config[0], step, err);
	return err;
}

static int lsm330_acc_update_fs_range(struct lsm330_acc_data *acc,
								u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;
	u8 sigmot_threshold;
	u8 init_val, updated_val;

	switch (new_fs_range) {
	case LSM330_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		sigmot_threshold = THRS1_2_02G;
		break;
	case LSM330_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		sigmot_threshold = THRS1_2_04G;
		break;
	case LSM330_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		sigmot_threshold = THRS1_2_06G;
		break;
	case LSM330_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		sigmot_threshold = THRS1_2_08G;
		break;
	case LSM330_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		sigmot_threshold = THRS1_2_16G;
		break;
	default:
		dev_err(acc->dev, "invalid g range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}
	/* Updates configuration register 5,
	* which contains odr range setting if device is enabled,
	* otherwise updates just RES_CTRL5 for when it will */
	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 1,
		* which contains g range setting */
		err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,
			LSM330_ACC_FS_MASK, new_fs_range, RES_LSM330_CTRL_REG5);
		if(err < 0) {
			dev_err(acc->dev, "update g range failed\n");
			return err;
		} else
			acc->sensitivity = sensitivity;

#if ENABLE_SIGNIFICANT_MOTION > 0
		err = lsm330_acc_register_masked_update(acc, LSM330_THRS1_2,
			0xFF, sigmot_threshold, RES_LSM330_THRS1_2);
		if(err < 0)
			dev_err(acc->dev, "update sign motion theshold"
								" failed\n");
		return err;
#endif
	} else {
		init_val = acc->resume_state[RES_LSM330_CTRL_REG5];
		updated_val = ((LSM330_ACC_FS_MASK & new_fs_range) | ((~LSM330_ACC_FS_MASK) & init_val));
		acc->resume_state[RES_LSM330_CTRL_REG5] = updated_val;

#if ENABLE_SIGNIFICANT_MOTION > 0
		acc->resume_state[RES_LSM330_THRS1_2] = sigmot_threshold;
#endif
		return 0;
	}

	return err;
}


static int lsm330_acc_update_odr(struct lsm330_acc_data *acc,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 new_odr;
	u8 updated_val;
	u8 init_val;
	u8 mask = LSM330_ODR_MASK;

#if ENABLE_SIGNIFICANT_MOTION > 0
	if (poll_interval_ms == 0)
		poll_interval_ms = 10;
#endif

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i >= 0; i--) {
		if (lsm330_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	new_odr = lsm330_acc_odr_table[i].mask;

	/* Updates configuration register 4,
	* which contains odr range setting if device is enabled,
	* otherwise updates just RES_CTRL4 for when it will */
	if (atomic_read(&acc->enabled)) {
		err = lsm330_acc_register_masked_update(acc,
			LSM330_CTRL_REG4, LSM330_ODR_MASK, new_odr,
							RES_LSM330_CTRL_REG4);
		if(err < 0){
			dev_err(acc->dev, "update odr failed\n");
			return err;
		}
		acc->ktime_acc = ktime_set(poll_interval_ms / 1000,
					MS_TO_NS(poll_interval_ms % 1000));
	} else {
		init_val = acc->resume_state[RES_LSM330_CTRL_REG4];
		updated_val = ((mask & new_odr) | ((~mask) & init_val));
		acc->resume_state[RES_LSM330_CTRL_REG4] = updated_val;
		return 0;
	}

	return err;
}


#ifdef LSM330_DEBUG
static int lsm330_acc_register_write(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	mutex_lock(&acc->lock);
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	err = acc->tf->write(acc->dev, reg_address, 1, &new_value);
	mutex_unlock(&acc->lock);
	return err;
}

static int lsm330_acc_register_read(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err;

	mutex_lock(&acc->lock);
	err = acc->tf->read(acc->dev, reg_address, 1, buf);
	mutex_unlock(&acc->lock);

	return err;
}

static int lsm330_acc_register_update(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}
#endif


static int lsm330_acc_get_data(struct lsm330_acc_data *acc, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	err = acc->tf->read(acc->dev, OUT_AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

	pr_debug("%s read x=%d, y=%d, z=%d\n",
			LSM330_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

	return err;
}

static void lsm330_acc_report_values(struct lsm330_acc_data *acc,
					int *xyz)
{
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static void lsm330_acc_polling_manage(struct lsm330_acc_data *acc)
{
	if((acc->enable_polling) & (atomic_read(&acc->enabled))) {
			hrtimer_start(&acc->hr_timer_acc,
				acc->ktime_acc, HRTIMER_MODE_REL);
	} else
		hrtimer_cancel(&acc->hr_timer_acc);
}

int lsm330_acc_enable(struct lsm330_acc_data *acc)
{
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		int err;

		mutex_lock(&acc->lock);
		err = lsm330_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			mutex_unlock(&acc->lock);
			return err;
		}
		mutex_unlock(&acc->lock);
		lsm330_acc_polling_manage(acc);
	}

	return 0;
}
EXPORT_SYMBOL(lsm330_acc_enable);

int lsm330_acc_disable(struct lsm330_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_work_sync(&acc->input_work_acc);
		lsm330_acc_polling_manage(acc);

		mutex_lock(&acc->lock);
		lsm330_acc_device_power_off(acc);
		mutex_unlock(&acc->lock);
	}

	return 0;
}
EXPORT_SYMBOL(lsm330_acc_disable);

static ssize_t attr_get_enable_polling(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->enable_polling;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_polling(struct device *dev,
				struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long enable;

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;
	mutex_lock(&acc->lock);
	if (enable)
		acc->enable_polling = 1;
	else
		acc->enable_polling = 0;
	mutex_unlock(&acc->lock);
	lsm330_acc_polling_manage(acc);
	return size;
}

static ssize_t attr_get_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	err = lsm330_acc_update_odr(acc, interval_ms);
	if(err >= 0)
	{
		acc->pdata->poll_interval = interval_ms;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch(val) {
	case LSM330_ACC_G_2G:
		range = 2;
		break;
	case LSM330_ACC_G_4G:
		range = 4;
		break;
	case LSM330_ACC_G_6G:
		range = 6;
		break;
	case LSM330_ACC_G_8G:
		range = 8;
		break;
	case LSM330_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = LSM330_ACC_G_2G;
			break;
		case 4:
			range = LSM330_ACC_G_4G;
			break;
		case 6:
			range = LSM330_ACC_G_6G;
			break;
		case 8:
			range = LSM330_ACC_G_8G;
			break;
		case 16:
			range = LSM330_ACC_G_16G;
			break;
		default:
			return -1;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_update_fs_range(acc, range);
	if(err >= 0)
	{
		acc->pdata->fs_range = range;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_acc_enable(acc);
	else
		lsm330_acc_disable(acc);

	return size;
}

static int lsm330_acc_state_progrs_enable_control(
				struct lsm330_acc_data *acc, u8 settings)
{
	u8 val1, val2;
	int err = -1;
	settings = settings & 0x03;

	switch ( settings ) {
	case LSM330_SM1_DIS_SM2_DIS:
		val1 = LSM330_SM1_EN_OFF;
		val2 = LSM330_SM2_EN_OFF;
		break;
	case LSM330_SM1_DIS_SM2_EN:
		val1 = LSM330_SM1_EN_OFF;
		val2 = LSM330_SM2_EN_ON;
		break;
	case LSM330_SM1_EN_SM2_DIS:
		val1 = LSM330_SM1_EN_ON;
		val2 = LSM330_SM2_EN_OFF;
		break;
	case LSM330_SM1_EN_SM2_EN:
		val1 = LSM330_SM1_EN_ON;
		val2 = LSM330_SM2_EN_ON;
		break;
	default :
		pr_err("invalid state program setting : 0x%02x\n",settings);
		return err;
	}
	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG1, LSM330_SM1_EN_MASK, val1,
							RES_LSM330_CTRL_REG1);
	if (err < 0 )
		return err;

	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG2, LSM330_SM2_EN_MASK, val2,
							RES_LSM330_CTRL_REG2);
	if (err < 0 )
			return err;

#if ENABLE_SIGNIFICANT_MOTION > 0
	if (val2 == LSM330_SM2_EN_ON)
		atomic_set(&acc->sign_mot_enabled, 1);
#endif

	pr_debug("state program setting : 0x%02x\n", settings);


	return err;
}

static ssize_t attr_set_enable_state_prog(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int err = -1;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;


	if ( val < 0x00 || val > LSM330_SM1_EN_SM2_EN){
		pr_warn("invalid state program setting, val: %ld\n",val);
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_state_progrs_enable_control(acc, val);
	mutex_unlock(&acc->lock);
	if (err < 0)
		return err;
	return size;
}

static ssize_t attr_get_enable_state_prog(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	int err;
	u8 val, val1 = 0, val2 = 0, config[2];
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	err = acc->tf->read(acc->dev, LSM330_CTRL_REG1, 1, config);
	if (err < 0) {
		mutex_unlock(&acc->lock);
		return err;
	}
	val1 = (config[0] & LSM330_SM1_EN_MASK);

	err = acc->tf->read(acc->dev, LSM330_CTRL_REG2, 1, config);
	val2 = ((config[0] & LSM330_SM2_EN_MASK) << 1);

	val = (val1 | val2);
	mutex_unlock(&acc->lock);

	return sprintf(buf, "0x%02x\n", val);
}


#ifdef LSM330_DEBUG
/* PAY ATTENTION: These LSM330_DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	u8 x[1];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = val;
	rc = acc->tf->write(acc->dev, acc->reg_addr, 1, x);
	mutex_unlock(&acc->lock);

	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	rc = acc->tf->read(acc->dev, acc->reg_addr, 1, &data);
	mutex_unlock(&acc->lock);

	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0666, attr_get_polling_rate,
							attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
	__ATTR(enable_polling, 0666, attr_get_enable_polling, attr_set_enable_polling),
	__ATTR(enable_state_prog, 0666, attr_get_enable_state_prog,
						attr_set_enable_state_prog),
#ifdef LSM330_DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct lsm330_acc_data *acc)
{
	int i;

#ifdef CUSTOM_SYSFS_PATH
	acc->acc_class = class_create(THIS_MODULE, CUSTOM_SYSFS_CLASS_NAME_ACC);
	if (acc->acc_class == NULL)
		goto custom_class_error;

	acc->acc_dev = device_create(acc->acc_class, NULL, 0, "%s", "acc");
	if (acc->acc_dev == NULL)
		goto custom_class_error;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(acc->acc_dev, attributes + i))
			goto error;
#else
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(acc->dev, attributes + i))
			goto error;
#endif
	return 0;

error:
	for ( ; i >= 0; i--)
#ifdef CUSTOM_SYSFS_PATH
		device_remove_file(acc->acc_dev, attributes + i);
#else
		device_remove_file(acc->dev, attributes + i);
#endif

#ifdef CUSTOM_SYSFS_PATH
custom_class_error:
#endif
	dev_err(acc->dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int lsm330_acc_validate_pdata(struct lsm330_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(acc->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(acc->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(acc->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330_acc_input_init(struct lsm330_acc_data *acc)
{
	int err;

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(acc->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->name = LSM330_ACC_DEV_NAME;
	acc->input_dev->id.bustype = acc->bus_type;
	acc->input_dev->dev.parent = acc->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(INPUT_EVENT_TYPE, acc->input_dev->evbit);
	set_bit(INPUT_EVENT_X, acc->input_dev->mscbit);
	set_bit(INPUT_EVENT_Y, acc->input_dev->mscbit);
	set_bit(INPUT_EVENT_Z, acc->input_dev->mscbit);

	/*	next is used for interruptB sources data if the case */
#if ENABLE_SIGNIFICANT_MOTION > 0
	set_bit(INPUT_EVENT_SM, acc->input_dev->mscbit);
#endif

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(acc->dev, "unable to register input device %s\n",
			acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lsm330_acc_input_cleanup(struct lsm330_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
	struct lsm330_acc_data *acc;
	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct work_struct *)input_work_acc,
					struct lsm330_acc_data, input_work_acc);

	mutex_lock(&acc->lock);
	err = lsm330_acc_get_data(acc, xyz);
	if (err < 0)
		dev_err(acc->dev, "get_accelerometer_data failed\n");
	else
		lsm330_acc_report_values(acc, xyz);

	mutex_unlock(&acc->lock);

	lsm330_acc_polling_manage(acc);
}

static enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct lsm330_acc_data *acc;

	acc = container_of((struct hrtimer *)timer,
					struct lsm330_acc_data, hr_timer_acc);

	queue_work(acc->acc_workqueue, &acc->input_work_acc);
	return HRTIMER_NORESTART;
}

int lsm330_acc_probe(struct lsm330_acc_data *acc)
{
	int err = -1;

	mutex_lock(&acc->lock);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(acc->dev, "failed to allocate memory for pdata: %d\n",
			err);
		goto err_mutexunlock;
	}

	if(acc->dev->platform_data == NULL) {
		default_lsm330_acc_pdata.gpio_int1 = int1_gpio;
		default_lsm330_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(acc->pdata, &default_lsm330_acc_pdata,
							sizeof(*acc->pdata));
		dev_info(acc->dev, "using default platform_data\n");
	} else {
		memcpy(acc->pdata, acc->dev->platform_data,
		       sizeof(*acc->pdata));
	}

	err = lsm330_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(acc->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(acc->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	/* resume state init config */
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	lsm330_acc_set_init_register_values(acc);
	//init state program1 and params
	lsm330_acc_set_init_statepr1_param(acc);
	lsm330_acc_set_init_statepr1_inst(acc);
	//init state program2  and params
	lsm330_acc_set_init_statepr2_param(acc);
	lsm330_acc_set_init_statepr2_inst(acc);

	err = lsm330_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(acc->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	acc->enable_polling = 1;
	atomic_set(&acc->enabled, 1);

	err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		dev_err(acc->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(acc->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_input_init(acc);
	if (err < 0) {
		dev_err(acc->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(acc);
	if (err < 0) {
		dev_err(acc->dev,
		   "device LSM330_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

#ifdef CUSTOM_SYSFS_PATH
	dev_set_drvdata(acc->acc_dev, acc);
#endif
	lsm330_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

#if ENABLE_SIGNIFICANT_MOTION > 0
	atomic_set(&acc->sign_mot_enabled, 0);
#endif
	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lsm330_acc_isr1,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq1", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, lsm330_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lsm330_acc_isr2,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq2", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	acc->acc_workqueue = create_workqueue("lsm330_workqueue");
	if (!acc->acc_workqueue)
		goto err_destoyworkqueue2;

	hrtimer_init(&acc->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acc->hr_timer_acc.function = &poll_function_read_acc;
	INIT_WORK(&acc->input_work_acc, poll_function_work_acc);

	mutex_unlock(&acc->lock);

	dev_info(acc->dev, "%s: probed\n", LSM330_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(acc->dev);
err_input_cleanup:
	lsm330_acc_input_cleanup(acc);
err_power_off:
	lsm330_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	if(acc->acc_workqueue) {
		destroy_workqueue(acc->acc_workqueue);
	}

	return err;
}
EXPORT_SYMBOL(lsm330_acc_probe);

int lsm330_acc_remove(struct lsm330_acc_data *acc)
{
	if(acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if(acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	lsm330_acc_device_power_off(acc);
	lsm330_acc_input_cleanup(acc);
	remove_sysfs_interfaces(acc->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();

	if(acc->acc_workqueue) {
		destroy_workqueue(acc->acc_workqueue);
	}

	kfree(acc->pdata);
	kfree(acc);

	return 0;
}
EXPORT_SYMBOL(lsm330_acc_remove);

MODULE_DESCRIPTION("lsm330 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Lorenzo Bianconi");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL v2");

