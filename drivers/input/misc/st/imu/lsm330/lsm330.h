/*
********************* (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: lsm330.h
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Lorenzo Bianconi (lorenzo.bianconi@st.com)
* Version		: V.1.2.6.1
* Date			: 2016/May/24
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
********************************************************************************

********************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
	V 1.2.0		Registers names compliant to correct datasheet
	V.1.2.1		Removed enable_interrupt_output sysfs file, manage int1
			and int2, implements int1 isr.
	V.1.2.2		Added HR_Timer and custom sysfs path
	V.1.2.3		Ch state program codes and state prog parameters define,
			gyro: corrects remove and poll_function_work
	V.1.2.5		Changes create_sysfs_interfaces
	V.1.2.6		Changes resume and suspend functions
	V.1.2.6.1	Introduce SignMotion feat implementation and solves
			acc suspend/resume issue;

********************************************************************************
SYSFS interface
- range: set full scale
	-> accelerometer: 	2,4,6,8,16 				[g]
	-> gyroscope:		250,500,2000				[dps]
- poll_period_ms: set 1/ODR
	-> accelerometer:	LSM330_ACC_MIN_POLL_PERIOD_MS < t	[ms]
	-> gyroscope:		LSM330_GYR_MIN_POLL_PERIOD_MS < t	[ms]
- enable_device: enable/disable sensor					[1/0]
- enable_polling: enable data polling/enable interrupt driven data acq	[1/0]

INPUT subsystem: NOTE-> output data INCLUDE the sensitivity in accelerometer,
			but NOT INCLUDE the sensitivity in gyroscope.
*******************************************************************************/

#ifndef __LSM330_H__
#define __LSM330_H__


#define LSM330_ACC_DEV_NAME		"lsm330_acc"
#define LSM330_GYR_DEV_NAME		"lsm330_gyr"

//#define CUSTOM_SYSFS_PATH
#define CUSTOM_SYSFS_CLASS_NAME_GYR	"ST_gyr"
#define CUSTOM_SYSFS_CLASS_NAME_ACC	"ST_acc"

#define LSM330_GYR_SAD0L		(0x00)
#define LSM330_ACC_SAD0L		(0x02)
#define LSM330_SAD0H			(0x01)
#define LSM330_ACC_I2C_SADROOT		(0x07)
#define LSM330_ACC_I2C_SAD_L		((LSM330_ACC_I2C_SADROOT<<2) | \
							LSM330_ACC_SAD0L)
#define LSM330_ACC_I2C_SAD_H		((LSM330_ACC_I2C_SADROOT<<2) | \
							LSM330_SAD0H)

#define LSM330_GYR_I2C_SADROOT		(0x35)
#define LSM330_GYR_I2C_SAD_L		((LSM330_GYR_I2C_SADROOT<<1)| \
							LSM330_GYR_SAD0L)
#define LSM330_GYR_I2C_SAD_H		((LSM330_GYR_I2C_SADROOT<<1)| \
							LSM330_SAD0H)

/* Poll Interval */
#define LSM330_ACC_MIN_POLL_PERIOD_MS		1

#define LSM330_GYR_MIN_POLL_PERIOD_MS		2


#ifdef __KERNEL__
/* enable significan motion program options
 * and configurations on INT2
 * */
#define ENABLE_SIGNIFICANT_MOTION	1

/* Interrupt */
#define LSM330_ACC_DEFAULT_INT1_GPIO		(-EINVAL)
#define LSM330_ACC_DEFAULT_INT2_GPIO		(-EINVAL)

/* replace previous defines with something like following
 * if you like to have default platform_data to
 * have gpios for interrupt pins defined.
#define LSM330_ACC_DEFAULT_INT1_GPIO		134
#define LSM330_ACC_DEFAULT_INT2_GPIO		39
*/

#define LSM330_GYR_DEFAULT_INT1_GPIO		(-EINVAL)
#define LSM330_GYR_DEFAULT_INT2_GPIO		(-EINVAL)


/* Accelerometer Sensor Full Scale */
#define LSM330_ACC_G_2G				(0x00)
#define LSM330_ACC_G_4G				(0x08)
#define LSM330_ACC_G_6G				(0x10)
#define LSM330_ACC_G_8G				(0x18)
#define LSM330_ACC_G_16G			(0x20)

/* Gyroscope Sensor Full Scale */
#define LSM330_GYR_FS_250DPS			(0x00)
#define LSM330_GYR_FS_500DPS			(0x10)
#define LSM330_GYR_FS_2000DPS			(0x30)

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_SM			MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

struct lsm330_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

#if defined(CONFIG_INPUT_LSM330_SPI) || \
    defined(CONFIG_INPUT_LSM330_SPI_MODULE)
#define LSM330_RX_MAX_LENGTH		500
#define LSM330_TX_MAX_LENGTH		500

struct lsm330_transfer_buffer {
	u8 rx_buf[LSM330_RX_MAX_LENGTH];
	u8 tx_buf[LSM330_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_LSM330_SPI */

struct lsm330_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

#define LSM330_ACC_RESUME_ENTRIES		43
#define LSM330_STATE_PR_SIZE			16

struct lsm330_acc_data {
	const char *name;
	u16 bus_type;
	struct device *dev;
	struct lsm330_acc_platform_data *pdata;

	struct mutex lock;
	struct work_struct input_work_acc;
	struct hrtimer hr_timer_acc;
	struct workqueue_struct *acc_workqueue;
	ktime_t ktime_acc;

	struct input_dev *input_dev;

#ifdef CUSTOM_SYSFS_PATH
	struct class *acc_class;
	struct device *acc_dev;
#endif

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	atomic_t sign_mot_enabled;
	int enable_polling;

	u16 sensitivity;

	u8 resume_state[LSM330_ACC_RESUME_ENTRIES];
	u8 resume_stmach_program1[LSM330_STATE_PR_SIZE];
	u8 resume_stmach_program2[LSM330_STATE_PR_SIZE];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef LSM330_DEBUG
	u8 reg_addr;
#endif
	struct lsm330_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM330_SPI) || \
    defined(CONFIG_INPUT_LSM330_SPI_MODULE)
	struct lsm330_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM330_SPI */
};

struct lsm330_gyr_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	/* fifo related */
	u8 watermark;
	u8 fifomode;

	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */

	/* axis mapping */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

#define LSM330_GYR_RESUME_ENTRIES		6

struct lsm330_gyr_status {
	const char *name;
	u16 bus_type;
	struct device *dev;
	struct lsm330_gyr_platform_data *pdata;

	struct mutex lock;
	struct workqueue_struct *gyr_workqueue;

	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[LSM330_GYR_RESUME_ENTRIES];

	u32 sensitivity;

#ifdef CUSTOM_SYSFS_PATH
	struct class *gyr_class;
	struct device *gyr_dev;
#endif

	/* interrupt related */
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	bool polling_enabled;
	/* fifo related */
	u8 watermark;
	u8 fifomode;

	struct hrtimer hr_timer;
	ktime_t ktime;
	struct work_struct polling_task;

	struct lsm330_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM330_SPI) || \
    defined(CONFIG_INPUT_LSM330_SPI_MODULE)
	struct lsm330_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM330_SPI */
};

int lsm330_acc_probe(struct lsm330_acc_data *acc);
int lsm330_acc_remove(struct lsm330_acc_data *acc);
int lsm330_acc_enable(struct lsm330_acc_data *acc);
int lsm330_acc_disable(struct lsm330_acc_data *acc);

int lsm330_gyr_probe(struct lsm330_gyr_status *stat);
int lsm330_gyr_remove(struct lsm330_gyr_status *stat);
int lsm330_gyr_enable(struct lsm330_gyr_status *stat);
int lsm330_gyr_disable(struct lsm330_gyr_status *stat);

#endif	/* __KERNEL__ */

#endif	/* __LSM330_H__ */
