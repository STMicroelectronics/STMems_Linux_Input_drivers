/******************** (C) COPYRIGHT 2016 STMicroelectronics ******************
*
* File Name		: lsm6ds0.h
* Author		: AMS - Motion Mems Division - Application Team
*			: Giuseppe Barba (giuseppe.barba@st.com)
* Version		: V.1.1.0
* Date			: 2016/May/13
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

#ifndef	__LSM6DS0_H__
#define	__LSM6DS0_H__

/** LSM6DS0 model */
#define LSM6DS0_ACC_GYR_DEV_NAME	"lsm6ds0"
#define LSM6DS0_ACC_DEV_NAME		"lsm6ds0_acc"
#define LSM6DS0_GYR_DEV_NAME    	"lsm6ds0_gyr"
#define LSM6DS0_MOD_DESCRIPTION	"lsm6ds0 driver"

/**********************************************/
/* 	Accelerometer section defines	 	*/
/**********************************************/

/* Accelerometer Sensor Full Scale */
#define LSM6DS0_ACC_FS_MASK		0x18
#define LSM6DS0_ACC_FS_2G 		0x00	/* Full scale 2g */
#define LSM6DS0_ACC_FS_4G 		0x08	/* Full scale 4g */
#define LSM6DS0_ACC_FS_8G 		0x10	/* Full scale 8g */

/* Accelerometer Anti-Aliasing Filter */
#define LSM6DS0_ACC_BW_400		0X00
#define LSM6DS0_ACC_BW_200		0X01
#define LSM6DS0_ACC_BW_100		0X02
#define LSM6DS0_ACC_BW_50		0X03
#define LSM6DS0_ACC_BW_MASK		0X03

#define LSM6DS0_INT1_GPIO_DEF		(-EINVAL)
#define LSM6DS0_INT2_GPIO_DEF		(-EINVAL)

#define LSM6DS0_ACC_ODR_OFF		0x00
#define LSM6DS0_ACC_ODR_MASK		0xE0
#define LSM6DS0_ACC_ODR_14_9		0x20
#define LSM6DS0_ACC_ODR_59_5		0x40
#define LSM6DS0_ACC_ODR_119		0x60
#define LSM6DS0_ACC_ODR_238		0x80
#define LSM6DS0_ACC_ODR_476		0xA0
#define LSM6DS0_ACC_ODR_952		0xC0

#define LSM6DL0_ACC_ODR_20		0x20
#define LSM6DL0_ACC_ODR_80		0x40
#define LSM6DL0_ACC_ODR_160		0x60
#define LSM6DL0_ACC_ODR_320		0x80
#define LSM6DL0_ACC_ODR_640		0xA0
#define LSM6DL0_ACC_ODR_1280		0xC0

/**********************************************/
/* 	Gyroscope section defines	 	*/
/**********************************************/


#define LSM6DS0_GYR_FS_MASK		0x18
#define LSM6DS0_GYR_FS_245DPS		0x00
#define LSM6DS0_GYR_FS_500DPS		0x08
#define LSM6DS0_GYR_FS_2000DPS		0x18

#define LSM6DS0_GYR_ODR_OFF		0x00
#define LSM6DS0_GYR_ODR_MASK		0xE0
#define LSM6DS0_GYR_ODR_MASK_SHIFT	5
#define LSM6DS0_GYR_ODR_001		(1 << LSM6DS0_GYR_ODR_MASK_SHIFT)
#define LSM6DS0_GYR_ODR_010		(2 << LSM6DS0_GYR_ODR_MASK_SHIFT)
#define LSM6DS0_GYR_ODR_011		(3 << LSM6DS0_GYR_ODR_MASK_SHIFT)
#define LSM6DS0_GYR_ODR_100		(4 << LSM6DS0_GYR_ODR_MASK_SHIFT)
#define LSM6DS0_GYR_ODR_101		(5 << LSM6DS0_GYR_ODR_MASK_SHIFT)
#define LSM6DS0_GYR_ODR_110		(6 << LSM6DS0_GYR_ODR_MASK_SHIFT)

#ifdef CONFIG_INPUT_LSM6DS0_LP
#define LSM6DS0_GYR_MAX_LP_ODR_US	8
#endif

#define LSM6DS0_GYR_BW_00		0x00
#define LSM6DS0_GYR_BW_01		0x01
#define LSM6DS0_GYR_BW_10		0x02
#define LSM6DS0_GYR_BW_11		0x03

/** ODR periods in msec */
#ifdef CONFIG_INPUT_LSM6DS0_S_MODEL
/** LSM6DS0 model */
#define LSM6DS0_ODR_14_9_US		67115
#define LSM6DS0_ODR_59_5_US		16807
#define LSM6DS0_ODR_119_US		 8404
#define LSM6DS0_ODR_238_US		 4202
#define LSM6DS0_ODR_476_US		 2101
#define LSM6DS0_ODR_952_US		 1051

#define LSM6DS0_ODR_US_001		LSM6DS0_ODR_14_9_US
#define LSM6DS0_ODR_US_010		LSM6DS0_ODR_59_5_US
#define LSM6DS0_ODR_US_011		LSM6DS0_ODR_119_US
#define LSM6DS0_ODR_US_100		LSM6DS0_ODR_238_US
#define LSM6DS0_ODR_US_101		LSM6DS0_ODR_476_US
#define LSM6DS0_ODR_US_110		LSM6DS0_ODR_952_US
#else
/** LSM6DL0 model */
#define LSM6DL0_ODR_20_US		50000
#define LSM6DL0_ODR_80_US		12500
#define LSM6DL0_ODR_160_US		 6250
#define LSM6DL0_ODR_320_US		 3125
#define LSM6DL0_ODR_640_US		 1563
#define LSM6DL0_ODR_1280_US		  782

#define LSM6DS0_ODR_US_001		LSM6DL0_ODR_20_US
#define LSM6DS0_ODR_US_010		LSM6DL0_ODR_80_US
#define LSM6DS0_ODR_US_011		LSM6DL0_ODR_160_US
#define LSM6DS0_ODR_US_100		LSM6DL0_ODR_320_US
#define LSM6DS0_ODR_US_101		LSM6DL0_ODR_640_US
#define LSM6DS0_ODR_US_110		LSM6DL0_ODR_1280_US
#endif

#define LSM6DS0_GYR_MIN_POLL_PERIOD_US	LSM6DS0_ODR_US_110
#define LSM6DS0_ACC_MIN_POLL_PERIOD_US	LSM6DS0_ODR_US_110
#define LSM6DS0_GYR_POLL_INTERVAL_DEF	LSM6DS0_ODR_US_001
#define LSM6DS0_ACC_POLL_INTERVAL_DEF	LSM6DS0_ODR_US_001

#define LSM6DS0_RX_MAX_LENGTH		500
#define LSM6DS0_TX_MAX_LENGTH		500

struct lsm6ds0_status;

struct lsm6ds0_acc_platform_data {
	uint32_t poll_interval;
	uint32_t min_interval;
	uint8_t fs_range;
	uint8_t aa_filter_bandwidth;

	int32_t (*init)(void);
	void (*exit)(void);
	int32_t (*power_on)(void);
	int32_t (*power_off)(void);
};

struct lsm6ds0_gyr_platform_data {
	uint32_t poll_interval;
	uint32_t min_interval;
	uint8_t fs_range;

	int32_t (*init)(void);
	void (*exit)(void);
	int32_t (*power_on)(void);
	int32_t (*power_off)(void);
};

struct lsm6ds0_main_platform_data {
	int32_t gpio_int1;
	int32_t gpio_int2;
	short rot_matrix[3][3];
	struct lsm6ds0_acc_platform_data *pdata_acc;
	struct lsm6ds0_gyr_platform_data *pdata_gyr;
#ifdef CONFIG_OF
	struct device_node	*of_node;
#endif
};

struct lsm6ds0_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LSM6DS0_RX_MAX_LENGTH];
	u8 tx_buf[LSM6DS0_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lsm6ds0_transfer_function {
	int (*write) (struct lsm6ds0_status *cdata, u8 reg_addr, int len, u8 *data);
	int (*read) (struct lsm6ds0_status *cdata, u8 reg_addr, int len, u8 *data);
};

struct lsm6ds0_status {
	struct lsm6ds0_main_platform_data *pdata_main;
	struct lsm6ds0_acc_platform_data *pdata_acc;
	struct lsm6ds0_gyr_platform_data *pdata_gyr;

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

	int8_t hw_initialized;

	/* hw_working=-1 means not tested yet */
	int8_t hw_working;

	atomic_t enabled_acc;
	atomic_t enabled_gyr;
	atomic_t enabled_temp;

	int32_t on_before_suspend;

	uint32_t sensitivity_acc;
	uint32_t sensitivity_gyr;
	int32_t irq;

	int32_t irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;

	int32_t irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef CONFIG_INPUT_LSM6DS0_SW_COMP
	int8_t k_fs;
	int8_t k_coeff[3];
	int32_t compensation_temp;
#endif
#ifdef CONFIG_INPUT_LSM6DS0_LP
	atomic_t low_power_state;
	atomic_t enabled_lp_mode;

	/** gyr_discard_samples count the number of samples to be discarded
	    after switching between low-power and normal mode */
	uint8_t gyr_discard_samples;
#endif
	/** acc_discard_samples count the number of samples to be discarded
	    after switching between power-down mode and normal mode */
	uint8_t acc_discard_samples;

	const struct lsm6ds0_transfer_function *tf;
	struct lsm6ds0_transfer_buffer tb;
};

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

int32_t lsm6ds0_common_probe(struct lsm6ds0_status *stat);
int32_t lsm6ds0_common_remove(struct lsm6ds0_status *stat);

#ifdef CONFIG_PM
int32_t lsm6ds0_common_suspend(struct lsm6ds0_status *stat);
int32_t lsm6ds0_common_resume(struct lsm6ds0_status *stat);
#endif /* CONFIG_PM */

#endif	/* __LSM6DS0_H__ */
