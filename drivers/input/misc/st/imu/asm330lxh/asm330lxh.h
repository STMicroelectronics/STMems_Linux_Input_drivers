/******************** (C) COPYRIGHT 2014 STMicroelectronics ******************
*
* File Name		: asm330lxh.h
* Author		: AMS - Motion Mems Division - Application Team
*			: Giuseppe Barba (giuseppe.barba@st.com)
*			: Alberto MARINONI (alberto.marinoni@st.com)
*			: Mario Tesi (mario.tesi@st.com)
* Version		: V.1.0.1
* Date			: 2016/Jul/07
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

#ifndef	__ASM330LXH_H__
#define	__ASM330LXH_H__

/* Enable open/close input function
#define ASM330LXH_EN_ON_OPEN
*/
/**********************************************/
/* 	Acc Gyro section define		      */
/**********************************************/
#define ASM330LXH_ACC_GYR_DEV_NAME	"asm330lxh"
#define ASM330LXH_ACC_DEV_NAME		"asm330lxh_acc"
#define ASM330LXH_GYR_DEV_NAME		"asm330lxh_gyr"

/** ODR periods in usec */
#define ASM330LXH_ODR_PERIOD_US_12_5_HZ	80000
#define ASM330LXH_ODR_PERIOD_US_50_HZ	20000
#define ASM330LXH_ODR_PERIOD_US_100_HZ	10000
#define ASM330LXH_ODR_PERIOD_US_200_HZ	5000
#define ASM330LXH_ODR_PERIOD_US_400_HZ	2500
#define ASM330LXH_ODR_PERIOD_US_800_HZ	1250

#define ASM330LXH_GYR_MIN_POLL_PERIOD_US	(ASM330LXH_ODR_PERIOD_US_800_HZ)
#define ASM330LXH_ACC_MIN_POLL_PERIOD_US	(ASM330LXH_ODR_PERIOD_US_800_HZ)
#define ASM330LXH_GYR_POLL_INTERVAL_DEF	(ASM330LXH_ODR_PERIOD_US_12_5_HZ)
#define ASM330LXH_ACC_POLL_INTERVAL_DEF	(ASM330LXH_ODR_PERIOD_US_12_5_HZ)

#define ASM330LXH_ACC_ODR_OFF		0x00
#define ASM330LXH_ACC_ODR_MASK		0xE0

#define ASM330LXH_GYR_ODR_OFF		0x00
#define ASM330LXH_GYR_ODR_MASK		0xE0
#define ASM330LXH_GYR_ODR_MASK_SHIFT	5
#define ASM330LXH_GYR_ODR_001		(1 << ASM330LXH_GYR_ODR_MASK_SHIFT)
#define ASM330LXH_GYR_ODR_010		(2 << ASM330LXH_GYR_ODR_MASK_SHIFT)
#define ASM330LXH_GYR_ODR_011		(3 << ASM330LXH_GYR_ODR_MASK_SHIFT)
#define ASM330LXH_GYR_ODR_100		(4 << ASM330LXH_GYR_ODR_MASK_SHIFT)
#define ASM330LXH_GYR_ODR_101		(5 << ASM330LXH_GYR_ODR_MASK_SHIFT)
#define ASM330LXH_GYR_ODR_110		(6 << ASM330LXH_GYR_ODR_MASK_SHIFT)

/* Accelerometer Sensor Full Scale */
#define ASM330LXH_ACC_FS_MASK		0x18
#define ASM330LXH_ACC_FS_2G 		0x00	/* Full scale 2g */
#define ASM330LXH_ACC_FS_4G 		0x10	/* Full scale 4g */
#define ASM330LXH_ACC_FS_8G 		0x18	/* Full scale 8g */
#define ASM330LXH_ACC_FS_16G 		0x08	/* Full scale 16g */

#define RANGE_2G			2
#define RANGE_4G			4
#define RANGE_8G			8
#define RANGE_16G			16

#define ASM330LXH_INT1_GPIO_DEF		(-EINVAL)
#define ASM330LXH_INT2_GPIO_DEF		(-EINVAL)

#define ASM330LXH_GYR_FS_MASK		0x18
#define ASM330LXH_GYR_FS_245DPS		0x00
#define ASM330LXH_GYR_FS_500DPS		0x08
#define ASM330LXH_GYR_FS_1000DPS	0x10
#define ASM330LXH_GYR_FS_2000DPS	0x18

#define RANGE_245DPS			245
#define RANGE_500DPS			500
#define RANGE_1000DPS			1000
#define RANGE_2000DPS			2000

#define ASM330LXH_GYR_BW_00		0x00
#define ASM330LXH_GYR_BW_01		0x01
#define ASM330LXH_GYR_BW_10		0x02
#define ASM330LXH_GYR_BW_11		0x03

#define ASM330LXH_RX_MAX_LENGTH		500
#define ASM330LXH_TX_MAX_LENGTH		500

struct asm330lxh_status;

struct asm330lxh_acc_platform_data {
	uint32_t poll_interval;
	uint32_t min_interval;
	uint8_t fs_range;

	int32_t (*init)(void);
	void (*exit)(void);
	int32_t (*power_on)(void);
	int32_t (*power_off)(void);
};

struct asm330lxh_gyr_platform_data {
	uint32_t poll_interval;
	uint32_t min_interval;
	uint8_t fs_range;

	int32_t (*init)(void);
	void (*exit)(void);
	int32_t (*power_on)(void);
	int32_t (*power_off)(void);
};

struct asm330lxh_main_platform_data {
	int32_t gpio_int1;
	int32_t gpio_int2;
	short rot_matrix[3][3];
	struct asm330lxh_acc_platform_data *pdata_acc;
	struct asm330lxh_gyr_platform_data *pdata_gyr;
#ifdef CONFIG_OF
	struct device_node *of_node;
#endif
};

struct asm330lxh_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ASM330LXH_RX_MAX_LENGTH];
	u8 tx_buf[ASM330LXH_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct asm330lxh_transfer_function {
	int (*write)(struct asm330lxh_status *cdata, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct asm330lxh_status *cdata, u8 reg_addr, int len,
		    u8 *data);
};

struct asm330lxh_status {
	struct asm330lxh_main_platform_data *pdata_main;
	struct asm330lxh_acc_platform_data *pdata_acc;
	struct asm330lxh_gyr_platform_data *pdata_gyr;

	struct mutex lock;
	struct work_struct input_work_acc;
	struct work_struct input_work_gyr;

	struct hrtimer hr_timer_acc;
	ktime_t ktime_acc;
	struct hrtimer hr_timer_gyr;
	ktime_t ktime_gyr;

	struct input_dev *input_dev_acc;
	struct input_dev *input_dev_gyr;
	struct device *dev;

	u16 bustype;
	atomic_t enabled_acc;
	atomic_t enabled_gyr;
	int32_t irq;

	atomic_t on_before_suspend;

	uint32_t sensitivity_acc;
	uint32_t sensitivity_gyr;

	s64 acc_ts, gyr_ts;

	/** acc_discard_samples count the number of samples to be discarded
	 after switching between power-down mode and normal mode */
	uint8_t acc_discard_samples;
	uint8_t gyr_discard_samples;
	uint32_t gyr_current_ODR_interval_us;

	/* Serial BUS */
	const struct asm330lxh_transfer_function *tf;
	struct asm330lxh_transfer_buffer tb;
};

#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

int32_t asm330lxh_common_probe(struct asm330lxh_status *stat);
int32_t asm330lxh_common_remove(struct asm330lxh_status *stat);

#ifdef CONFIG_PM
int32_t asm330lxh_common_suspend(struct asm330lxh_status *stat);
int32_t asm330lxh_common_resume(struct asm330lxh_status *stat);
#endif /* CONFIG_PM */

#endif	/* __ASM330LXH_H__ */
