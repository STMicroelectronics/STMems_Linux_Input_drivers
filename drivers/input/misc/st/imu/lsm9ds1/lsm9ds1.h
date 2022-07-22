/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lsm9ds1.h
* Authors	     : AMS - Motion Mems Division - Application Team
*		     : Giuseppe Barba (giuseppe.barba@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
*		     : Lorenzo Bianconi (lorenzo.bianconi@st.com)
* Version            : V.1.0.0
* Date               : 2016/May/13
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

#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#define LSM9DS1_ACC_GYR_DEV_NAME	"lsm9ds1_acc_gyr"
#define LSM9DS1_MAG_DEV_NAME		"lsm9ds1_mag"
#define LSM9DS1_ACC_DEV_NAME		"lsm9ds1_acc"
#define LSM9DS1_GYR_DEV_NAME    	"lsm9ds1_gyr"


/************************************************/
/* 	Magnetometer Section  		        */
/************************************************/
/* Magnetometer Sensor Full Scale */
#define LSM9DS1_MAG_FS_MASK		0x60
#define LSM9DS1_MAG_FS_4G		0x00	/* Full scale 4 Gauss */
#define LSM9DS1_MAG_FS_8G		0x20	/* Full scale 8 Gauss */
#define LSM9DS1_MAG_FS_12G		0x40	/* Full scale 10 Gauss */
#define LSM9DS1_MAG_FS_16G		0x60	/* Full scale 16 Gauss */

/* ODR */
#define ODR_MAG_MASK			0x1C	/* Mask for odr change on mag */
#define LSM9DS1_MAG_ODR0_625		0x00	/* 0.625Hz output data rate */
#define LSM9DS1_MAG_ODR1_25		0x04	/* 1.25Hz output data rate */
#define LSM9DS1_MAG_ODR2_5		0x08	/* 2.5Hz output data rate */
#define LSM9DS1_MAG_ODR5		0x0C	/* 5Hz output data rate */
#define LSM9DS1_MAG_ODR10		0x10	/* 10Hz output data rate */
#define LSM9DS1_MAG_ODR20		0x14	/* 20Hz output data rate */
#define LSM9DS1_MAG_ODR40		0x18	/* 40Hz output data rate */
#define LSM9DS1_MAG_ODR80		0x1C	/* 80Hz output data rate */

#define MAG_ENABLE_ON_INPUT_OPEN 	0
#define LSM9DS1_MAG_MIN_POLL_PERIOD_MS	13
#define LSM9DS1_INT_M_GPIO_DEF		-EINVAL
#define LSM9DS1_M_POLL_INTERVAL_DEF	100


/************************************************/
/* 	Accelerometer section defines	        */
/************************************************/
#define LSM9DS1_ACC_MIN_POLL_PERIOD_MS	1

/* Accelerometer Sensor Full Scale */
#define LSM9DS1_ACC_FS_MASK		0x18
#define LSM9DS1_ACC_FS_2G 		0x00	/* Full scale 2g */
#define LSM9DS1_ACC_FS_4G 		0x10	/* Full scale 4g */
#define LSM9DS1_ACC_FS_8G 		0x18	/* Full scale 8g */

/* Accelerometer Anti-Aliasing Filter */
#define LSM9DS1_ACC_BW_408		0X00
#define LSM9DS1_ACC_BW_211		0X01
#define LSM9DS1_ACC_BW_105		0X02
#define LSM9DS1_ACC_BW_50		0X03
#define LSM9DS1_ACC_BW_MASK		0X03

#define LSM9DS1_INT1_GPIO_DEF		-EINVAL
#define LSM9DS1_INT2_GPIO_DEF		-EINVAL

#define LSM9DS1_ACC_ODR_OFF		0x00
#define LSM9DS1_ACC_ODR_MASK		0xE0
#define LSM9DS1_ACC_ODR_10		0x20
#define LSM9DS1_ACC_ODR_50		0x40
#define LSM9DS1_ACC_ODR_119		0x60
#define LSM9DS1_ACC_ODR_238		0x80
#define LSM9DS1_ACC_ODR_476		0xA0
#define LSM9DS1_ACC_ODR_952		0xC0

/************************************************/
/* 	Gyroscope section defines	 	*/
/************************************************/
#define LSM9DS1_GYR_MIN_POLL_PERIOD_MS	1

#define LSM9DS1_GYR_FS_MASK		0x18
#define LSM9DS1_GYR_FS_245DPS		0x00
#define LSM9DS1_GYR_FS_500DPS		0x08
#define LSM9DS1_GYR_FS_2000DPS		0x18

#define LSM9DS1_GYR_ODR_OFF		0x00
#define LSM9DS1_GYR_ODR_MASK		0xE0
#define LSM9DS1_GYR_ODR_14_9		0x20
#define LSM9DS1_GYR_ODR_59_5		0x40
#define LSM9DS1_GYR_ODR_119		0x60
#define LSM9DS1_GYR_ODR_238		0x80
#define LSM9DS1_GYR_ODR_476		0xA0
#define LSM9DS1_GYR_ODR_952		0xC0

#define LSM9DS1_GYR_BW_0		0x00
#define LSM9DS1_GYR_BW_1		0x01
#define LSM9DS1_GYR_BW_2		0x02
#define LSM9DS1_GYR_BW_3		0x03

#define LSM9DS1_GYR_POLL_INTERVAL_DEF	100
#define LSM9DS1_ACC_POLL_INTERVAL_DEF	100

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX


#if defined(CONFIG_INPUT_LSM9DS1_SPI) || \
    defined(CONFIG_INPUT_LSM9DS1_SPI_MODULE)
#define LSM9DS1_RX_MAX_LENGTH		500
#define LSM9DS1_TX_MAX_LENGTH		500

struct lsm9ds1_transfer_buffer {
	u8 rx_buf[LSM9DS1_RX_MAX_LENGTH];
	u8 tx_buf[LSM9DS1_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_LSMDS1_SPI */

struct lsm9ds1_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

struct lsm9ds1_mag_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	short rot_matrix[3][3];
	int gpio_int_m;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
#ifdef CONFIG_OF
	struct device_node	*of_node;
#endif
};

struct lsm9ds1_mag_dev {
	const char *name;
	u16 bus_type;
	struct device *dev;
	const struct of_device_id *mag_dt_id;
	struct lsm9ds1_mag_platform_data *pdata_mag;

	struct mutex lock;
	struct work_struct input_work_mag;
	struct workqueue_struct *mag_workqueue;

	struct hrtimer hr_timer_mag;
	ktime_t ktime_mag;

	struct input_dev *input_dev_mag;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;

	atomic_t enabled_mag;

	u32 sensitivity_mag;

	u8 xy_mode;
	u8 z_mode;
	struct lsm9ds1_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM9DS1_SPI) || \
    defined(CONFIG_INPUT_LSM9DS1_SPI_MODULE)
	struct lsm9ds1_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSMDS1_SPI */
};

struct lsm9ds1_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	u8 aa_filter_bandwidth;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct lsm9ds1_gyr_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct lsm9ds1_acc_gyr_dev {
	const char *name;
	u16 bus_type;
	const struct of_device_id *acc_gyr_dt_id;
	struct device *dev;
	struct lsm9ds1_acc_gyr_main_platform_data *pdata_main;
	struct lsm9ds1_acc_platform_data *pdata_acc;
	struct lsm9ds1_gyr_platform_data *pdata_gyr;

	struct mutex lock;

	int irq;
	s64 timestamp;

	u16 acc_skip_cnt, acc_dec_cnt;
	u16 gyr_skip_cnt, gyr_dec_cnt;

	ktime_t ktime_acc;

	struct input_dev *input_dev_acc;
	struct input_dev *input_dev_gyr;
	//struct input_dev *input_dev_temp;

	atomic_t enabled_acc;
        atomic_t enabled_gyr;
        atomic_t enabled_temp;

	int32_t temp_value_dec;
	uint32_t temp_value_flo;

	int32_t sensitivity_acc;
	int32_t sensitivity_gyr;

	struct lsm9ds1_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM9DS1_SPI) || \
    defined(CONFIG_INPUT_LSM9DS1_SPI_MODULE)
	struct lsm9ds1_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSMDS1_SPI */
};

struct lsm9ds1_acc_gyr_main_platform_data {
	short rot_matrix[3][3];
	struct lsm9ds1_acc_platform_data *pdata_acc;
	struct lsm9ds1_gyr_platform_data *pdata_gyr;
#ifdef CONFIG_OF
	struct device_node	*of_node;
#endif
};

static inline s64 lsm9ds1_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

int lsm9ds1_acc_gyr_probe(struct lsm9ds1_acc_gyr_dev *dev, int irq);
int lsm9ds1_acc_gyr_remove(struct lsm9ds1_acc_gyr_dev *dev);
int lsm9ds1_acc_gyr_enable(struct lsm9ds1_acc_gyr_dev *dev);
int lsm9ds1_acc_gyr_disable(struct lsm9ds1_acc_gyr_dev *dev);

int lsm9ds1_mag_probe(struct lsm9ds1_mag_dev *dev);
int lsm9ds1_mag_remove(struct lsm9ds1_mag_dev *dev);
int lsm9ds1_mag_enable(struct lsm9ds1_mag_dev *dev);
int lsm9ds1_mag_disable(struct lsm9ds1_mag_dev *dev);

#endif	/* __LSM9DS1_H__ */
