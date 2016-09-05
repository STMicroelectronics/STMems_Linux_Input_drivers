/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lsm9ds0.h
* Authors            : MSH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
* Version            : V.1.0.2
* Date               : 2013/Oct/23
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
* REVISON HISTORY
* 1.0.1		| 2012/Aug/30	| Denis Ciocca    | corrects gyr_get_data func
* 1.0.2		| 2013/Oct/23	| Matteo Dameno	  | introduced acc_mag 1.0.5
*******************************************************************************/

#ifndef	__LSM9DS0_H__
#define	__LSM9DS0_H__

#define	LSM9DS0_DEV_NAME	"lsm9ds0_acc_mag"
#define	LSM9DS0_ACC_DEV_NAME	"lsm9ds0_acc"
#define	LSM9DS0_MAG_DEV_NAME	"lsm9ds0_mag"
#define LSM9DS0_GYR_DEV_NAME	"lsm9ds0_gyr"

#define LSM9DS0_SAD0L_ACC_MAG		0x02
#define LSM9DS0_SAD0H_ACC_MAG		0x01
#define LSM9DS0_SAD0L_GYR		0x00
#define LSM9DS0_SAD0H_GYR		0x01

/************************************************/
/* 	Output data			 	*/
/*************************************************
accelerometer: ug
magnetometer: ugauss
gyroscope: udps
*************************************************/

/************************************************/
/* 	sysfs data			 	*/
/*************************************************
accelerometer:
	- pollrate->ms
	- fullscale->g
magnetometer:
	- pollrate->ms
	- fullscale->gauss
gyroscope:
	- pollrate->ms
	- fullscale->dps
*************************************************/

#define LSM9DS0_ACC_MAG_I2C_SADROOT	0x07

/* I2C address if gyr SA0 pin to GND */
#define LSM9DS0_ACC_MAG_I2C_SAD_L	((LSM9DS0_ACC_MAG_I2C_SADROOT << 2) | \
					 LSM9DS0_SAD0L_ACC_MAG)
/* I2C address if gyr SA0 pin to Vdd */
#define LSM9DS0_ACC_MAG_I2C_SAD_H	((LSM9DS0_ACC_MAG_I2C_SADROOT << 2) | \
					 LSM9DS0_SAD0H_ACC_MAG)

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	LSM9DS0_ACC_FS_MASK	0x18
#define LSM9DS0_ACC_FS_2G 	0x00	/* Full scale 2g */
#define LSM9DS0_ACC_FS_4G 	0x08	/* Full scale 4g */
#define LSM9DS0_ACC_FS_8G 	0x10	/* Full scale 8g */
#define LSM9DS0_ACC_FS_16G	0x18	/* Full scale 16g */

/* Accelerometer Anti-Aliasing Filter */
#define ANTI_ALIASING_773	0X00
#define ANTI_ALIASING_362	0X40
#define ANTI_ALIASING_194	0X80
#define ANTI_ALIASING_50	0XC0

/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

/* Magnetometer Sensor Full Scale */
#define LSM9DS0_MAG_FS_MASK	0x60
#define LSM9DS0_MAG_FS_2G	0x00	/* Full scale 2 gauss */
#define LSM9DS0_MAG_FS_4G	0x20	/* Full scale 4 gauss */
#define LSM9DS0_MAG_FS_8G	0x40	/* Full scale 8 gauss */
#define LSM9DS0_MAG_FS_12G	0x60	/* Full scale 12 gauss */

/************************************************/
/* 	Gyroscope section defines	 	*/
/************************************************/

#define LSM9DS0_GYR_I2C_SADROOT		0x35

/* I2C address if gyr SA0 pin to GND */
#define LSM9DS0_GYR_I2C_SAD_L		((LSM9DS0_GYR_I2C_SADROOT << 1) | \
					 LSM9DS0_SAD0L_GYR)
/* I2C address if gyr SA0 pin to Vdd */
#define LSM9DS0_GYR_I2C_SAD_H		((LSM9DS0_GYR_I2C_SADROOT << 1) | \
					 LSM9DS0_SAD0H_GYR)

/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones havew to be set to -EINVAL
 */
#define DEFAULT_INT1_GPIO		-EINVAL
#define DEFAULT_INT2_GPIO		-EINVAL

#define	LSM9DS0_ACC_MIN_POLL_PERIOD_MS	1
#define LSM9DS0_MAG_MIN_POLL_PERIOD_MS	5

#define LSM9DS0_GYR_DEFAULT_INT1_GPIO	-EINVAL
#define LSM9DS0_GYR_DEFAULT_INT2_GPIO	-EINVAL

#define LSM9DS0_GYR_MIN_POLL_PERIOD_MS	2

#define LSM9DS0_GYR_FS_250DPS		0x00
#define LSM9DS0_GYR_FS_500DPS		0x10
#define LSM9DS0_GYR_FS_2000DPS		0x30

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

struct lsm9ds0_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

#if defined(CONFIG_INPUT_LSM9DS0_SPI) || \
    defined(CONFIG_INPUT_LSM9DS0_SPI_MODULE)
#define LSM9DS0_RX_MAX_LENGTH		500
#define LSM9DS0_TX_MAX_LENGTH		500

struct lsm9ds0_transfer_buffer {
	u8 rx_buf[LSM9DS0_RX_MAX_LENGTH];
	u8 tx_buf[LSM9DS0_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_LSM9DS0_SPI */

struct lsm9ds0_dev {
	u16 bus_type;
	const char *name;
	struct device *dev;
	struct lsm9ds0_acc_platform_data *pdata_acc;
	struct lsm9ds0_mag_platform_data *pdata_mag;

	struct mutex lock;
	struct work_struct input_work_acc;
	struct work_struct input_work_mag;

	struct hrtimer hr_timer_acc;
	ktime_t ktime_acc;
	struct hrtimer hr_timer_mag;
	ktime_t ktime_mag;
	struct workqueue_struct *data_workqueue;

	struct input_dev *input_dev_acc;
	struct input_dev *input_dev_mag;
	struct input_dev *input_dev_temp;

	struct lsm9ds0_interrupt *interrupt;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;

	atomic_t enabled_acc;
	atomic_t enabled_mag;
	atomic_t enabled_temp;

	int temp_value_dec;
	unsigned int temp_value_flo;

	int on_before_suspend;

	u16 sensitivity_acc;
	u16 sensitivity_mag;

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct lsm9ds0_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM9DS0_SPI) || \
    defined(CONFIG_INPUT_LSM9DS0_SPI_MODULE)
	struct lsm9ds0_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM9DS0_SPI */
};

struct lsm9ds0_acc_platform_data {
	
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	short rot_matrix[3][3];

	u8 aa_filter_bandwidth;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio_int1;
	int gpio_int2;
};

#define LSM9DS0_GYR_RESUME_ENTRIES	6

struct lsm9ds0_mag_platform_data {

	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	short rot_matrix[3][3];

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct lsm9ds0_gyr_dev {
	u16 bus_type;
	const char *name;

	struct device *dev;
	struct lsm9ds0_gyr_platform_data *pdata;

	struct mutex lock;
	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[LSM9DS0_GYR_RESUME_ENTRIES];

	u32 sensitivity;

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
	struct workqueue_struct *gyr_workqueue;

	struct lsm9ds0_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM9DS0_SPI) || \
    defined(CONFIG_INPUT_LSM9DS0_SPI_MODULE)
	struct lsm9ds0_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM9DS0_SPI */
};

struct lsm9ds0_gyr_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

struct lsm9ds0_main_platform_data {
	
	struct lsm9ds0_acc_platform_data *pdata_acc;
	struct lsm9ds0_mag_platform_data *pdata_mag;
};

int lsm9ds0_probe(struct lsm9ds0_dev *dev);
int lsm9ds0_remove(struct lsm9ds0_dev *dev);
int lsm9ds0_enable(struct lsm9ds0_dev *dev);
int lsm9ds0_disable(struct lsm9ds0_dev *dev);

int lsm9ds0_gyr_probe(struct lsm9ds0_gyr_dev *dev);
int lsm9ds0_gyr_remove(struct lsm9ds0_gyr_dev *dev);
int lsm9ds0_gyr_resume(struct lsm9ds0_gyr_dev *dev);
int lsm9ds0_gyr_suspend(struct lsm9ds0_gyr_dev *dev);

#endif	/* __LSM9DS0_H__ */
