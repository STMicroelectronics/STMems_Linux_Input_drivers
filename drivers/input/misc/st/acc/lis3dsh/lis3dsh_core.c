/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name		: lis3dsh_core.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Mario Tesi (mario.tesi@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V.1.2.3
 * Date			: 2016/May/16
 * Description		: LIS3DSH accelerometer driver
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
	V.1.2.1		Removed enable_interrupt_output sysfs file, manage int1
			and int2, implements int1 isr.
	V.1.2.2		Modified state program loadiing defines, removed
			state machine program.
	V.1.2.3		Added support to spi interface.
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

#include "lis3dsh.h"

//#define DEBUG

/* set to 1 to enable SM program and parameters loading */
/* set to 0 to leave unused */
#define LOAD_SM1_PROGRAM	1
#define LOAD_SP1_PARAMETERS	1
#define LOAD_SM2_PROGRAM	1
#define LOAD_SP2_PARAMETERS	1

#define G_MAX			23920640	/* ug */

#define SENSITIVITY_2G		60		/* ug/LSB	*/
#define SENSITIVITY_4G		120		/* ug/LSB	*/
#define SENSITIVITY_6G		180		/* ug/LSB	*/
#define SENSITIVITY_8G		240		/* ug/LSB	*/
#define SENSITIVITY_16G		730		/* ug/LSB	*/

#define LIS3DSH_FS_MASK		0x38

/* Output Data Rates ODR */
#define LIS3DSH_ODR_MASK	0XF0
#define LIS3DSH_PM_OFF		0x00		/* OFF */
#define LIS3DSH_ODR3_125	0x10		/*    3.125 Hz */
#define LIS3DSH_ODR6_25		0x20		/*    6.25  Hz */
#define LIS3DSH_ODR12_5		0x30		/*   12.5   Hz */
#define LIS3DSH_ODR25		0x40		/*   25     Hz */
#define LIS3DSH_ODR50		0x50		/*   50     Hz */
#define LIS3DSH_ODR100		0x60		/*  100     Hz */
#define LIS3DSH_ODR400		0x70		/*  400     Hz */
#define LIS3DSH_ODR800		0x80		/*  800     Hz */
#define LIS3DSH_ODR1600		0x90		/* 1600     Hz */

/* CTRLREG1 */
#define LIS3DSH_HIST1_MASK	0xE0
#define LIS3DSH_SM1INT_PIN_MASK	0x08
#define LIS3DSH_SM1INT_PININT2	0x08
#define LIS3DSH_SM1INT_PININT1	0x00
#define LIS3DSH_SM1_EN_MASK	0x01
#define LIS3DSH_SM1_EN_ON	0x01
#define LIS3DSH_SM1_EN_OFF	0x00
/* */

/* CTRLREG2 */
#define LIS3DSH_HIST2_MASK	0xE0
#define LIS3DSH_SM2INT_PIN_MASK	0x08
#define LIS3DSH_SM2INT_PININT2	0x08
#define LIS3DSH_SM2INT_PININT1	0x00
#define LIS3DSH_SM2_EN_MASK	0x01
#define LIS3DSH_SM2_EN_ON	0x01
#define LIS3DSH_SM2_EN_OFF	0x00
/* */

/* CTRLREG3 */
#define LIS3DSH_INT_ACT_MASK	(0x01 << 6)
#define LIS3DSH_INT_ACT_H	(0x01 << 6)
#define LIS3DSH_INT_ACT_L	0x00

#define LIS3DSH_INT2_EN_MASK	(0x01 << 4)
#define LIS3DSH_INT2_EN_ON	(0x01 << 4)
#define LIS3DSH_INT2_EN_OFF	0x00

#define LIS3DSH_INT1_EN_MASK	(0x01 << 3)
#define LIS3DSH_INT1_EN_ON	(0x01 << 3)
#define LIS3DSH_INT1_EN_OFF	0x00
/* */

/* CTRLREG4 */
#define LIS3DSH_BDU_EN		0x08
#define LIS3DSH_ALL_AXES	0x07
/* */

/* STATUS REG BITS */
#define LIS3DSH_STAT_INTSM1_BIT	(0x01 << 3)
#define LIS3DSH_STAT_INTSM2_BIT	(0x01 << 2)

#define OUT_AXISDATA_REG	LIS3DSH_OUTX_L
#define WHOAMI_LIS3DSH_ACC	0x3F	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define LIS3DSH_WHO_AM_I	0x0F	/* WhoAmI register Address */

#define LIS3DSH_OUTX_L		0x28	/* Output X LSByte */
#define LIS3DSH_OUTX_H		0x29	/* Output X MSByte */
#define LIS3DSH_OUTY_L		0x2A	/* Output Y LSByte */
#define LIS3DSH_OUTY_H		0x2B	/* Output Y MSByte */
#define LIS3DSH_OUTZ_L		0x2C	/* Output Z LSByte */
#define LIS3DSH_OUTZ_H		0x2D	/* Output Z MSByte */
#define LIS3DSH_LC_L		0x16	/* LSByte Long Counter Status */
#define LIS3DSH_LC_H		0x17	/* MSByte Long Counter Status */

#define LIS3DSH_INTERR_STAT	0x18	/* Interrupt Status */

#define LIS3DSH_STATUS_REG	0x27	/* Status */

#define LIS3DSH_CTRL_REG1	0x21	/* control reg 1 */
#define LIS3DSH_CTRL_REG2	0x22	/* control reg 2 */
#define LIS3DSH_CTRL_REG3	0x23	/* control reg 3 */
#define LIS3DSH_CTRL_REG4	0x20	/* control reg 4 */
#define LIS3DSH_CTRL_REG5	0x24	/* control reg 5 */
#define LIS3DSH_CTRL_REG6	0x25	/* control reg 6 */

#define LIS3DSH_OFF_X		0x10	/* Offset X Corr */
#define LIS3DSH_OFF_Y		0x11	/* Offset Y Corr */
#define LIS3DSH_OFF_Z		0x12	/* Offset Z Corr */

#define LIS3DSH_CS_X		0x13	/* Const Shift X */
#define LIS3DSH_CS_Y		0x14	/* Const Shift Y */
#define LIS3DSH_CS_Z		0x15	/* Const Shift Z */

#define LIS3DSH_VFC_1		0x1B	/* Vect Filter Coeff 1 */
#define LIS3DSH_VFC_2		0x1C	/* Vect Filter Coeff 2 */
#define LIS3DSH_VFC_3		0x1D	/* Vect Filter Coeff 3 */
#define LIS3DSH_VFC_4		0x1E	/* Vect Filter Coeff 4 */

	/* state program 1 */
#define LIS3DSH_STATEPR1	0X40	/*	State Program 1 16 bytes */

#define LIS3DSH_TIM4_1		0X50	/*	SPr1 Timer4		*/
#define LIS3DSH_TIM3_1		0X51	/*	SPr1 Timer3		*/
#define LIS3DSH_TIM2_1		0X52	/*	SPr1 Timer2	2bytes	*/
#define LIS3DSH_TIM1_1		0X54	/*	SPr1 Timer1	2bytes	*/

#define LIS3DSH_THRS2_1		0X56	/*	SPr1 Threshold1		*/
#define LIS3DSH_THRS1_1		0X57	/*	SPr1 Threshold2		*/
#define LIS3DSH_SA_1		0X59	/*	SPr1 Swap Axis Sign Msk	*/
#define LIS3DSH_MA_1		0X5A	/*	SPr1 Axis Sign Msk	*/
#define LIS3DSH_SETT_1		0X5B	/*	SPr1 			*/
#define LIS3DSH_PPRP_1		0X5C	/*	SPr1 ProgPointer ResetPointer */
#define LIS3DSH_TC_1		0X5D	/*	SPr1 		2bytes	*/
#define LIS3DSH_OUTS_1		0X5F	/*	SPr1 			*/

	/* state program 2 */
#define LIS3DSH_STATEPR2	0X60	/*	State Program 2 16 bytes */

#define LIS3DSH_TIM4_2		0X70	/*	SPr2 Timer4		*/
#define LIS3DSH_TIM3_2		0X71	/*	SPr2 Timer3		*/
#define LIS3DSH_TIM2_2		0X72	/*	SPr2 Timer2	2bytes	*/
#define LIS3DSH_TIM1_2		0X74	/*	SPr2 Timer1	2bytes	*/

#define LIS3DSH_THRS2_2		0X76	/*	SPr2 Threshold1		*/
#define LIS3DSH_THRS1_2		0X77	/*	SPr2 Threshold2		*/
#define LIS3DSH_DES_2		0X78	/*	SPr2 Decimation		*/
#define LIS3DSH_SA_2		0X79	/*	SPr2 Swap Axis Sign Msk	*/
#define LIS3DSH_MA_2		0X7A	/*	SPr2 Axis Sign Msk	*/
#define LIS3DSH_SETT_2		0X7B	/*	SPr2 			*/
#define LIS3DSH_PPRP_2		0X7C	/*	SPr2 ProgPointer ResetPointer */
#define LIS3DSH_TC_2		0X7D	/*	SPr2 		2bytes	*/
#define LIS3DSH_OUTS_2		0X7F	/*	SPr2 			*/
/*	end CONTROL REGISTRES	*/


/* RESUME STATE INDICES */
#define RES_LIS3DSH_LC_L		0
#define RES_LIS3DSH_LC_H		1

#define RES_LIS3DSH_CTRL_REG4		2
#define RES_LIS3DSH_CTRL_REG1		3
#define RES_LIS3DSH_CTRL_REG2		4
#define RES_LIS3DSH_CTRL_REG3		5
#define RES_LIS3DSH_CTRL_REG5		6
#define RES_LIS3DSH_CTRL_REG6		7

#define RES_LIS3DSH_OFF_X		8
#define RES_LIS3DSH_OFF_Y		9
#define RES_LIS3DSH_OFF_Z		10

#define RES_LIS3DSH_CS_X		11
#define RES_LIS3DSH_CS_Y		12
#define RES_LIS3DSH_CS_Z		13

#define RES_LIS3DSH_VFC_1		14
#define RES_LIS3DSH_VFC_2		15
#define RES_LIS3DSH_VFC_3		16
#define RES_LIS3DSH_VFC_4		17

#define RES_LIS3DSH_THRS3		18

#define RES_LIS3DSH_TIM4_1		20
#define RES_LIS3DSH_TIM3_1		21
#define RES_LIS3DSH_TIM2_1_L		22
#define RES_LIS3DSH_TIM2_1_H		23
#define RES_LIS3DSH_TIM1_1_L		24
#define RES_LIS3DSH_TIM1_1_H		25

#define RES_LIS3DSH_THRS2_1		26
#define RES_LIS3DSH_THRS1_1		27
#define RES_LIS3DSH_SA_1		28
#define RES_LIS3DSH_MA_1		29
#define RES_LIS3DSH_SETT_1		30

#define RES_LIS3DSH_TIM4_2		31
#define RES_LIS3DSH_TIM3_2		32
#define RES_LIS3DSH_TIM2_2_L		33
#define RES_LIS3DSH_TIM2_2_H		34
#define RES_LIS3DSH_TIM1_2_L		35
#define RES_LIS3DSH_TIM1_2_H		36

#define RES_LIS3DSH_THRS2_2		37
#define RES_LIS3DSH_THRS1_2		38
#define RES_LIS3DSH_DES_2		39
#define RES_LIS3DSH_SA_2		40
#define RES_LIS3DSH_MA_2		41
#define RES_LIS3DSH_SETT_2		42

/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define LIS3DSH_SM1_DIS_SM2_DIS		0x00
#define LIS3DSH_SM1_EN_SM2_DIS		0x01
#define LIS3DSH_SM1_DIS_SM2_EN		0x02
#define LIS3DSH_SM1_EN_SM2_EN		0x03

/* INTERRUPTS ENABLE CONTROLS */
#define LIS3DSH_INT1_DIS_INT2_DIS	0x00
#define LIS3DSH_INT1_EN_INT2_DIS	0x01
#define LIS3DSH_INT1_DIS_INT2_EN	0x02
#define LIS3DSH_INT1_EN_INT2_EN		0x03

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dsh_odr_table[] = {
		{ 1, LIS3DSH_ODR1600 },
		{ 3, LIS3DSH_ODR400 },
		{ 10, LIS3DSH_ODR100 },
		{ 20, LIS3DSH_ODR50 },
		{ 40, LIS3DSH_ODR25 },
		{ 80, LIS3DSH_ODR12_5 },
		{ 160, LIS3DSH_ODR6_25 },
		{ 320, LIS3DSH_ODR3_125 },
};

static struct lis3dsh_platform_data default_lis3dsh_pdata = {
	.fs_range = LIS3DSH_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 10,
	.min_interval = LIS3DSH_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LIS3DSH_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LIS3DSH_ACC_DEFAULT_INT2_GPIO,
};

static int int1_gpio = LIS3DSH_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LIS3DSH_ACC_DEFAULT_INT2_GPIO;

/* sets default init values to be written in registers at probe stage */
static void lis3dsh_set_init_register_values(struct lis3dsh_status *acc)
{
	acc->resume_state[RES_LIS3DSH_LC_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_LC_H] = 0x00;

	acc->resume_state[RES_LIS3DSH_CTRL_REG1] = (0x00 | LIS3DSH_SM1INT_PININT1);
	acc->resume_state[RES_LIS3DSH_CTRL_REG2] = (0x00 | LIS3DSH_SM2INT_PININT1);
	acc->resume_state[RES_LIS3DSH_CTRL_REG3] = LIS3DSH_INT_ACT_H;
	if(acc->pdata->gpio_int1 >= 0)
		acc->resume_state[RES_LIS3DSH_CTRL_REG3] =
				acc->resume_state[RES_LIS3DSH_CTRL_REG3] | \
					LIS3DSH_INT1_EN_ON;
	if(acc->pdata->gpio_int2 >= 0)
		acc->resume_state[RES_LIS3DSH_CTRL_REG3] =
				acc->resume_state[RES_LIS3DSH_CTRL_REG3] | \
					LIS3DSH_INT2_EN_ON;

	acc->resume_state[RES_LIS3DSH_CTRL_REG4] = (LIS3DSH_BDU_EN |
							LIS3DSH_ALL_AXES);
	acc->resume_state[RES_LIS3DSH_CTRL_REG5] = 0x00;
	acc->resume_state[RES_LIS3DSH_CTRL_REG6] = 0x10;

	acc->resume_state[RES_LIS3DSH_THRS3] = 0x00;
	acc->resume_state[RES_LIS3DSH_OFF_X] = 0x00;
	acc->resume_state[RES_LIS3DSH_OFF_Y] = 0x00;
	acc->resume_state[RES_LIS3DSH_OFF_Z] = 0x00;

	acc->resume_state[RES_LIS3DSH_CS_X] = 0x00;
	acc->resume_state[RES_LIS3DSH_CS_Y] = 0x00;
	acc->resume_state[RES_LIS3DSH_CS_Z] = 0x00;

	acc->resume_state[RES_LIS3DSH_VFC_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_VFC_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_VFC_3] = 0x00;
	acc->resume_state[RES_LIS3DSH_VFC_4] = 0x00;
}

static void lis3dsh_set_init_statepr1_inst(struct lis3dsh_status *acc)
{
#if (LOAD_SM1_PROGRAM == 1)
	/* Place here state machine 1 program */
	acc->resume_stmach_program1[0] = 0x00;
	acc->resume_stmach_program1[1] = 0x00;
	acc->resume_stmach_program1[2] = 0X00;
	acc->resume_stmach_program1[3] = 0X00;
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
#else
	acc->resume_stmach_program1[0] = 0x00;
	acc->resume_stmach_program1[1] = 0x00;
	acc->resume_stmach_program1[2] = 0X00;
	acc->resume_stmach_program1[3] = 0X00;
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

static void lis3dsh_set_init_statepr2_inst(struct lis3dsh_status *acc)
{
#if (LOAD_SM2_PROGRAM == 1)
	/* Place here state machine 2 program */
	acc->resume_stmach_program2[0] = 0x00;
	acc->resume_stmach_program2[1] = 0x00;
	acc->resume_stmach_program2[2] = 0X00;
	acc->resume_stmach_program2[3] = 0X00;
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
	acc->resume_stmach_program2[2] = 0X00;
	acc->resume_stmach_program2[3] = 0X00;
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

static void lis3dsh_set_init_statepr1_param(struct lis3dsh_status *acc)
{
#if (LOAD_SP1_PARAMETERS == 1)
	/* Place here state machine 1 parameters */
	acc->resume_state[RES_LIS3DSH_TIM4_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM3_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_1_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_1_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_1_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_1_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS2_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS1_1] = 0x00;
	/* DES1 not available*/
	acc->resume_state[RES_LIS3DSH_SA_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_MA_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_SETT_1] = 0x00;
#else
	acc->resume_state[RES_LIS3DSH_TIM4_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM3_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_1_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_1_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_1_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_1_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS2_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS1_1] = 0x00;
	/* DES1 not available*/
	acc->resume_state[RES_LIS3DSH_SA_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_MA_1] = 0x00;
	acc->resume_state[RES_LIS3DSH_SETT_1] = 0x00;
#endif
}

static void lis3dsh_set_init_statepr2_param(struct lis3dsh_status *acc)
{
#if (LOAD_SP2_PARAMETERS == 1)
	/* Place here state machine 2 parameters */
	acc->resume_state[RES_LIS3DSH_TIM4_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM3_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS2_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS1_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_DES_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_SA_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_MA_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_SETT_2] = 0x00;
#else
	acc->resume_state[RES_LIS3DSH_TIM4_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM3_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_2_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM2_2_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_2_L] = 0x00;
	acc->resume_state[RES_LIS3DSH_TIM1_2_H] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS2_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_THRS1_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_DES_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_SA_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_MA_2] = 0x00;
	acc->resume_state[RES_LIS3DSH_SETT_2] = 0x00;
#endif
}

static int lis3dsh_i2c_update(struct lis3dsh_status *acc,
			      u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;

	err = acc->tf->read(acc, reg_address, 1, &init_val);
	if (err >= 0) {
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = acc->tf->write(acc, reg_address, 1, &updated_val);
	}

	return err;
}

static int lis3dsh_hw_init(struct lis3dsh_status *acc)
{
	int i;
	int err = -1;
	u8 buf[17];

	err = acc->tf->read(acc, LIS3DSH_WHO_AM_I, 1, buf);
	if (err < 0) {
		dev_warn(acc->dev, "Error reading WHO_AM_I: is device "
			 "available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_LIS3DSH_ACC) {
		dev_err(acc->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n", WHOAMI_LIS3DSH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = acc->resume_state[RES_LIS3DSH_LC_L];
	buf[1] = acc->resume_state[RES_LIS3DSH_LC_H];
	err = acc->tf->write(acc, LIS3DSH_LC_L, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LIS3DSH_TIM4_1];
	buf[1] = acc->resume_state[RES_LIS3DSH_TIM3_1];
	buf[2] = acc->resume_state[RES_LIS3DSH_TIM2_1_L];
	buf[3] = acc->resume_state[RES_LIS3DSH_TIM2_1_H];
	buf[4] = acc->resume_state[RES_LIS3DSH_TIM1_1_L];
	buf[5] = acc->resume_state[RES_LIS3DSH_TIM1_1_H];
	buf[6] = acc->resume_state[RES_LIS3DSH_THRS2_1];
	buf[7] = acc->resume_state[RES_LIS3DSH_THRS1_1];
	err = acc->tf->write(acc, LIS3DSH_TIM4_1, 8, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LIS3DSH_SA_1];
	buf[1] = acc->resume_state[RES_LIS3DSH_MA_1];
	buf[2] = acc->resume_state[RES_LIS3DSH_SETT_1];
	err = acc->tf->write(acc, LIS3DSH_SA_1, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LIS3DSH_TIM4_2];
	buf[1] = acc->resume_state[RES_LIS3DSH_TIM3_2];
	buf[2] = acc->resume_state[RES_LIS3DSH_TIM2_2_L];
	buf[3] = acc->resume_state[RES_LIS3DSH_TIM2_2_H];
	buf[4] = acc->resume_state[RES_LIS3DSH_TIM1_2_L];
	buf[5] = acc->resume_state[RES_LIS3DSH_TIM1_2_H];
	buf[6] = acc->resume_state[RES_LIS3DSH_THRS2_2];
	buf[7] = acc->resume_state[RES_LIS3DSH_THRS1_2];
	buf[8] = acc->resume_state[RES_LIS3DSH_DES_2];
	buf[9] = acc->resume_state[RES_LIS3DSH_SA_2];
	buf[10] = acc->resume_state[RES_LIS3DSH_MA_2];
	buf[11] = acc->resume_state[RES_LIS3DSH_SETT_2];
	err = acc->tf->write(acc, LIS3DSH_TIM4_2, 12, buf);
	if (err < 0)
		goto err_resume_state;

	/*	state program 1 */
	for (i = 0; i <= LIS3DSH_STATE_PR_SIZE; i++) {
		buf[i] = acc->resume_stmach_program1[i-1];
		pr_debug("i=%d,sm pr1 buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = acc->tf->write(acc, LIS3DSH_STATEPR1, LIS3DSH_STATE_PR_SIZE, buf);
	if (err < 0)
		goto err_resume_state;

	/*	state program 2 */
	for(i = 0; i <= LIS3DSH_STATE_PR_SIZE; i++) {
		buf[i] = acc->resume_stmach_program2[i-1];
		pr_debug("i=%d,sm pr2 buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = acc->tf->write(acc, LIS3DSH_STATEPR2, LIS3DSH_STATE_PR_SIZE,
			     buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LIS3DSH_CTRL_REG5];
	buf[1] = acc->resume_state[RES_LIS3DSH_CTRL_REG6];
	err = acc->tf->write(acc, LIS3DSH_CTRL_REG5, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LIS3DSH_CTRL_REG1];
	buf[1] = acc->resume_state[RES_LIS3DSH_CTRL_REG2];
	buf[2] = acc->resume_state[RES_LIS3DSH_CTRL_REG3];
	err = acc->tf->write(acc, LIS3DSH_CTRL_REG1, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_LIS3DSH_CTRL_REG4];
	err = acc->tf->write(acc, LIS3DSH_CTRL_REG4, 1, buf);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
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

static void lis3dsh_device_power_off(struct lis3dsh_status *acc)
{
	int err;

	err = lis3dsh_i2c_update(acc, LIS3DSH_CTRL_REG4,
				 LIS3DSH_ODR_MASK, LIS3DSH_PM_OFF);
	if (err < 0)
		dev_err(acc->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if (acc->pdata->gpio_int1)
			disable_irq_nosync(acc->irq1);
		if (acc->pdata->gpio_int2)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if (acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lis3dsh_device_power_on(struct lis3dsh_status *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(acc->dev,
				"power_on failed: %d\n", err);
			return err;
		}
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = lis3dsh_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lis3dsh_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	return 0;
}

static irqreturn_t lis3dsh_isr1(int irq, void *dev)
{
	struct lis3dsh_status *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	return IRQ_HANDLED;
}

static irqreturn_t lis3dsh_isr2(int irq, void *dev)
{
	struct lis3dsh_status *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);

	return IRQ_HANDLED;
}

static void lis3dsh_irq1_work_func(struct work_struct *work)
{
	struct lis3dsh_status *acc;

	acc = container_of(work, struct lis3dsh_status, irq1_work);
	pr_debug("%s: IRQ1 triggered\n", LIS3DSH_ACC_DEV_NAME);
	/* TODO  add interrupt service procedure.
		 ie:lis3dsh_get_int_source(acc); */
	enable_irq(acc->irq1);
}

static void lis3dsh_irq2_work_func(struct work_struct *work)
{
	struct lis3dsh_status *acc;

	acc = container_of(work, struct lis3dsh_status, irq2_work);
	pr_debug("%s: IRQ2 triggered\n", LIS3DSH_ACC_DEV_NAME);
	/* TODO  add interrupt service procedure.
		 ie:lis3dsh_get_stat_source(acc); */
	/* ; */
	enable_irq(acc->irq2);
}

static int lis3dsh_register_masked_update(struct lis3dsh_status *acc,
					  u8 reg_address, u8 mask,
					  u8 new_bit_values, int resume_index)
{
	u8 init_val, updated_val;
	int err;

	err = acc->tf->read(acc, reg_address, 1, &init_val);
	if (err < 0)
		goto error;

	acc->resume_state[resume_index] = init_val;
	updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
	err = acc->tf->write(acc, reg_address, 1, &updated_val);
	if (err < 0)
		goto error;

	acc->resume_state[resume_index] = updated_val;

	return err;

error:
		dev_err(acc->dev,
			"register 0x%02x update failed error: %d\n",
			updated_val, err);
	return err;
}

static int lis3dsh_update_fs_range(struct lis3dsh_status *acc,
				   u8 new_fs_range)
{
	int err = -1;
	u16 sensitivity;

	switch (new_fs_range) {
	case LIS3DSH_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LIS3DSH_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LIS3DSH_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		break;
	case LIS3DSH_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LIS3DSH_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(acc->dev, "invalid g range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 1,
		* which contains g range setting */
		err = lis3dsh_register_masked_update(acc, LIS3DSH_CTRL_REG5,
		LIS3DSH_FS_MASK, new_fs_range, RES_LIS3DSH_CTRL_REG5);
		if (err < 0) {
			dev_err(acc->dev, "update g range failed\n");
			return err;
		} else
			acc->sensitivity = sensitivity;
	}

	if (err < 0)
		dev_err(acc->dev, "update g range not executed "
			"because the device is off\n");
	return err;
}

static int lis3dsh_update_odr(struct lis3dsh_status *acc,
			      int poll_interval_ms)
{
	int err = 0;
	int i;
	u8 new_odr, old_odr;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dsh_odr_table) - 1; i >= 0; i--)
		if (lis3dsh_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;

	old_odr = acc->pdata->poll_interval;
	new_odr = lis3dsh_odr_table[i].mask;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		err = lis3dsh_register_masked_update(acc, LIS3DSH_CTRL_REG4,
						     LIS3DSH_ODR_MASK, new_odr,
						     RES_LIS3DSH_CTRL_REG4);
		if (err >= 0)
			acc->pdata->poll_interval = lis3dsh_odr_table[i].cutoff_ms;
		else
			acc->pdata->poll_interval = old_odr;
	} else {
		/* Store new ODR for next activation */
		acc->pdata->poll_interval = lis3dsh_odr_table[i].cutoff_ms;
	}

	return err;
}


#ifdef DEBUG
static int lis3dsh_register_write(struct lis3dsh_status *acc, u8 *buf,
				  u8 reg_address, u8 new_value)
{
	buf[0] = new_value;
	
	return acc->tf->write(acc, reg_address, 1, buf);
}

static int lis3dsh_register_read(struct lis3dsh_status *acc, u8 *buf,
				 u8 reg_address)
{
	return acc->tf->read(acc, reg_address, 1, buf);
}

static int lis3dsh_register_update(struct lis3dsh_status *acc, u8 *buf,
				   u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;

	err = acc->tf->read(acc, reg_address, 1, &init_val);
	if (!(err < 0)) {
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = acc->tf->write(acc, reg_address, 1, &updated_val);
	}

	return err;
}
#endif

static int lis3dsh_get_acceleration_data(struct lis3dsh_status *acc,
					 int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	err = acc->tf->read(acc, OUT_AXISDATA_REG, 6, acc_data);
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

	pr_debug("%s read x=%d, y=%d, z=%d\n", LIS3DSH_ACC_DEV_NAME,
		 xyz[0], xyz[1], xyz[2]);

	return err;
}

static void lis3dsh_report_values(struct lis3dsh_status *acc, int *xyz)
{
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(acc->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int lis3dsh_enable(struct lis3dsh_status *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		dev_info(acc->dev, "Enabling Device %s\n",
			acc->input_dev->name);
		err = lis3dsh_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		/* Android:
		 * Udpate polling rate in case odr was changed while sensor disabled */
		lis3dsh_update_odr(acc, acc->pdata->poll_interval);
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lis3dsh_disable(struct lis3dsh_status *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		dev_info(acc->dev, "Disabling Device %s\n",
			acc->input_dev->name);
		cancel_delayed_work(&acc->input_work);
		lis3dsh_device_power_off(acc);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int val;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);

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
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&acc->lock);
	err = lis3dsh_update_odr(acc, interval_ms);
	if (err < 0)
		dev_err(acc->dev, "Failed to set odr\n");

	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_range(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	u8 val;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	int range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch(val) {
	case LIS3DSH_ACC_G_2G:
		range = 2;
		break;
	case LIS3DSH_ACC_G_4G:
		range = 4;
		break;
	case LIS3DSH_ACC_G_6G:
		range = 6;
		break;
	case LIS3DSH_ACC_G_8G:
		range = 8;
		break;
	case LIS3DSH_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t size)
{
	int err;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = LIS3DSH_ACC_G_2G;
			break;
		case 4:
			range = LIS3DSH_ACC_G_4G;
			break;
		case 6:
			range = LIS3DSH_ACC_G_6G;
			break;
		case 8:
			range = LIS3DSH_ACC_G_8G;
			break;
		case 16:
			range = LIS3DSH_ACC_G_16G;
			break;
		default:
			return -1;
	}

	mutex_lock(&acc->lock);
	err = lis3dsh_update_fs_range(acc, range);
	if (err >= 0)
		acc->pdata->fs_range = range;
	mutex_unlock(&acc->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis3dsh_enable(acc);
	else
		lis3dsh_disable(acc);

	return size;
}

static int lis3dsh_state_progrs_enable_control(struct lis3dsh_status *acc,
					       u8 settings)
{
	u8 val1, val2;
	int err = -1;

	switch ( settings ) {
	case LIS3DSH_SM1_DIS_SM2_DIS:
		val1 = LIS3DSH_SM1_EN_OFF;
		val2 = LIS3DSH_SM2_EN_OFF;
		break;
	case LIS3DSH_SM1_DIS_SM2_EN:
		val1 = LIS3DSH_SM1_EN_OFF;
		val2 = LIS3DSH_SM2_EN_ON;
		break;
	case LIS3DSH_SM1_EN_SM2_DIS:
		val1 = LIS3DSH_SM1_EN_ON;
		val2 = LIS3DSH_SM2_EN_OFF;
		break;
	case LIS3DSH_SM1_EN_SM2_EN:
		val1 = LIS3DSH_SM1_EN_ON;
		val2 = LIS3DSH_SM2_EN_ON;
		break;
	default :
		pr_err("invalid state program setting : 0x%02x\n",settings);
		return err;
	}
	err = lis3dsh_register_masked_update(acc, LIS3DSH_CTRL_REG1,
					     LIS3DSH_SM1_EN_MASK, val1,
					     RES_LIS3DSH_CTRL_REG1);
	if (err < 0 )
		return err;

	err = lis3dsh_register_masked_update(acc, LIS3DSH_CTRL_REG2,
					     LIS3DSH_SM2_EN_MASK, val2,
					     RES_LIS3DSH_CTRL_REG2);
	if (err < 0 )
		return err;

	acc->stateprogs_enable_setting = settings;

	pr_debug("state program setting : 0x%02x\n",
		 acc->stateprogs_enable_setting);

	return err;
}

static ssize_t attr_set_enable_state_prog(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	int err = -1;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	if (val < 0x00 || val > LIS3DSH_SM1_EN_SM2_EN) {
		pr_warn("invalid state program setting, val: %ld\n",val);
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = lis3dsh_state_progrs_enable_control(acc, val);
	mutex_unlock(&acc->lock);
	if (err < 0)
		return err;

	return size;
}

static ssize_t attr_get_enable_state_prog(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	u8 val;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->stateprogs_enable_setting;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "0x%02x\n", val);
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int rc;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = acc->tf->write(acc, &x[0], 1, &x[1]);

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
	int rc;
	u8 reg, data;

	mutex_lock(&acc->lock);
	reg = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = acc->tf->read(acc, reg, 1, &data);
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct lis3dsh_status *acc = dev_get_drvdata(dev);
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

	__ATTR(poll_period_ms, 0664, attr_get_polling_rate,
	       attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(enable_state_prog, 0664, attr_get_enable_state_prog,
	       attr_set_enable_state_prog),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;

	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);

	dev_err(dev, "%s:Unable to create interface\n", __func__);

	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);

	return 0;
}

static void lis3dsh_input_work_func(struct work_struct *work)
{
	struct lis3dsh_status *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			   struct lis3dsh_status, input_work);

	mutex_lock(&acc->lock);
	err = lis3dsh_get_acceleration_data(acc, xyz);
	if (err < 0)
		pr_debug("get_acceleration_data failed\n");
	else
		lis3dsh_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			      acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef LIS3DSH_EN_ON_OPEN
int lis3dsh_input_open(struct input_dev *input)
{
	struct lis3dsh_status *acc = input_get_drvdata(input);

	return lis3dsh_enable(acc);
}

void lis3dsh_input_close(struct input_dev *dev)
{
	struct lis3dsh_status *acc = input_get_drvdata(dev);

	lis3dsh_disable(acc);
}
#endif

static int lis3dsh_validate_pdata(struct lis3dsh_status *acc)
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

/* Report misc event type */
static int lis3dsh_input_init(struct lis3dsh_status *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lis3dsh_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		dev_err(acc->dev, "input device allocation failed\n");

		return -ENOMEM;
	}

	acc->input_dev->name = LIS3DSH_ACC_DEV_NAME;
	acc->input_dev->id.bustype = acc->bustype;
	acc->input_dev->dev.parent = acc->dev;
#ifdef LIS3DSH_EN_ON_OPEN
	acc->input_dev->open = lis3dsh_input_open;
	acc->input_dev->close = lis3dsh_input_close;
#endif
	input_set_drvdata(acc->input_dev, acc);

	__set_bit(INPUT_EVENT_TYPE, acc->input_dev->evbit);
	__set_bit(INPUT_EVENT_X, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, acc->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Z, acc->input_dev->mscbit);

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(acc->dev, "unable to register input device %s\n",
			acc->input_dev->name);
		input_free_device(acc->input_dev);
	}

	return err;
}

static void lis3dsh_input_cleanup(struct lis3dsh_status *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

int lis3dsh_common_probe(struct lis3dsh_status *acc)
{
	int err = -1;

	dev_info(acc->dev, "probe start.\n");
	mutex_init(&acc->lock);
	mutex_init(&acc->tb.buf_lock);
	mutex_lock(&acc->lock);

	acc->pdata = kmalloc(sizeof(struct lis3dsh_platform_data), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(acc->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (acc->dev->platform_data == NULL) {
		default_lis3dsh_pdata.gpio_int1 = int1_gpio;
		default_lis3dsh_pdata.gpio_int2 = int2_gpio;
		memcpy(acc->pdata, &default_lis3dsh_pdata,
							sizeof(*acc->pdata));
		dev_info(acc->dev, "using default platform_data\n");
	} else {
		memcpy(acc->pdata, acc->dev->platform_data,
		       sizeof(*acc->pdata));
	}

	err = lis3dsh_validate_pdata(acc);
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

	if (acc->pdata->gpio_int1 >= 0) {
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d "
			"mapped on gpio:%d\n",
			LIS3DSH_ACC_DEV_NAME, __func__, acc->irq1,
			acc->pdata->gpio_int1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d "
			"mapped on gpio:%d\n",
			LIS3DSH_ACC_DEV_NAME, __func__, acc->irq2,
			acc->pdata->gpio_int2);
	}

	/* resume state init config */
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	lis3dsh_set_init_register_values(acc);

	/* init state program1 and params */
	lis3dsh_set_init_statepr1_param(acc);
	lis3dsh_set_init_statepr1_inst(acc);

	/* init state program2  and params */
	lis3dsh_set_init_statepr2_param(acc);
	lis3dsh_set_init_statepr2_inst(acc);

	err = lis3dsh_device_power_on(acc);
	if (err < 0) {
		dev_err(acc->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lis3dsh_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		dev_err(acc->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lis3dsh_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(acc->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis3dsh_input_init(acc);
	if (err < 0) {
		dev_err(acc->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(acc->dev);
	if (err < 0) {
		dev_err(acc->dev,
		   	"device LIS3DSH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lis3dsh_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if (acc->pdata->gpio_int1 >= 0) {
		INIT_WORK(&acc->irq1_work, lis3dsh_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lis3dsh_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev, "cannot create work queue1: %d\n",
				err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lis3dsh_isr1,
				  IRQF_TRIGGER_RISING, "lis3dsh_irq1", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		INIT_WORK(&acc->irq2_work, lis3dsh_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lis3dsh_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(acc->dev, "cannot create work queue2: %d\n",
				err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lis3dsh_isr2,
				  IRQF_TRIGGER_RISING, "lis3dsh_irq2", acc);
		if (err < 0) {
			dev_err(acc->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	mutex_unlock(&acc->lock);

	dev_info(acc->dev, "%s: probed\n", LIS3DSH_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if (acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if (acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(acc->dev);
err_input_cleanup:
	lis3dsh_input_cleanup(acc);
err_power_off:
	lis3dsh_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	pr_err("%s: Driver Init failed\n", LIS3DSH_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lis3dsh_common_probe);

int lis3dsh_common_remove(struct lis3dsh_status *acc)
{
	if (acc->pdata->gpio_int1 >= 0) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if (acc->pdata->gpio_int2 >= 0) {
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	if (atomic_cmpxchg(&acc->enabled, 1, 0))
		cancel_delayed_work_sync(&acc->input_work);

	lis3dsh_device_power_off(acc);
	lis3dsh_input_cleanup(acc);
	remove_sysfs_interfaces(acc->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();

	kfree(acc->pdata);

	return 0;
}
EXPORT_SYMBOL(lis3dsh_common_remove);

#ifdef CONFIG_PM
int lis3dsh_common_resume(struct lis3dsh_status *acc)
{
	if (acc->on_before_suspend)
		return lis3dsh_enable(acc);

	return 0;
}
EXPORT_SYMBOL(lis3dsh_common_resume);

int lis3dsh_common_suspend(struct lis3dsh_status *acc)
{
	acc->on_before_suspend = atomic_read(&acc->enabled);

	return lis3dsh_disable(acc);
}
EXPORT_SYMBOL(lis3dsh_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("lis3dsh accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, Mario Tesi, STMicroelectronics");
MODULE_LICENSE("GPL v2");

