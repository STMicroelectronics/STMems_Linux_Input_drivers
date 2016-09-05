/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lsm303c.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
*		: Lorenzo Bianconi (lorenzo.bianconi@st.com)
* Version	: V.1.0.3
* Date		: 2013/Dec/18
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
*******************************************************************************/
/******************************************************************************/

#ifndef	__LSM303C_H__
#define	__LSM303C_H__


#define LSM303C_ACC_DEV_NAME			"lsm303c_acc"
#define	LSM303C_MAG_DEV_NAME			"lsm303c_mag"


/* to set gpios numb connected to interrupt pins,
 * the unused ones have to be set to -EINVAL
 */
#define LSM303C_ACC_DEFAULT_INT1_GPIO		(-EINVAL)

#define LSM303C_MAG_DEFAULT_INT1_GPIO		(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define LSM303C_ACC_FS_MASK			(0x30)
#define LSM303C_ACC_FS_2G			(0x00)
#define LSM303C_ACC_FS_4G			(0x20)
#define LSM303C_ACC_FS_8G			(0x30)

/* Magnetometer Sensor Full Scale */
#define LSM303C_MAG_FS_MASK			(0x60)
#define LSM303C_MAG_FS_4G			(0x00)	/* Full scale 4 G */
#define LSM303C_MAG_FS_8G			(0x20)	/* Full scale 8 G */
#define LSM303C_MAG_FS_10G			(0x40)	/* Full scale 10 G */
#define LSM303C_MAG_FS_16G			(0x60)	/* Full scale 16 G */

#define LSM303C_ACC_MIN_POLL_PERIOD_MS		2
#define LSM303C_MAG_MIN_POLL_PERIOD_MS		13


struct lsm303c_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

#if defined(CONFIG_INPUT_LSM303C_SPI) || \
    defined(CONFIG_INPUT_LSM303C_SPI_MODULE)
#define LSM303C_RX_MAX_LENGTH		500
#define LSM303C_TX_MAX_LENGTH		500

struct lsm303c_transfer_buffer {
	u8 rx_buf[LSM303C_RX_MAX_LENGTH];
	u8 tx_buf[LSM303C_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE

#define LSM303C_ACC_RESUME_ENTRIES		18

struct lsm303c_acc_platform_data {
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

	/* set gpio_int[1] to choose gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
};

struct lsm303c_acc_dev {
	const char *name;
	u16 bus_type;
	struct device *dev;
	struct lsm303c_acc_platform_data *pdata;

	struct mutex lock;
	struct work_struct input_poll_work;
	struct hrtimer hr_timer_poll;
	ktime_t polling_ktime;
	struct workqueue_struct *hr_timer_poll_work_queue;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	u8 sensitivity;

	u8 resume_state[LSM303C_ACC_RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef LSM303C_DEBUG
	u8 reg_addr;
#endif

	struct lsm303c_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM303C_SPI) || \
    defined(CONFIG_INPUT_LSM303C_SPI_MODULE)
	struct lsm303c_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM303C_SPI */
};

struct lsm303c_mag_platform_data {

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
};

struct lsm303c_mag_dev {
	const char *name;
	u16 bus_type;
	struct device *dev;
	struct i2c_client *client;
	struct lsm303c_mag_platform_data *pdata_mag;

	struct mutex lock;
	struct work_struct input_work_mag;

	struct hrtimer hr_timer_mag;
	ktime_t ktime_mag;
	struct workqueue_struct *mag_workqueue;

	struct input_dev *input_dev_mag;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;

	atomic_t enabled_mag;

	int on_before_suspend;
	int use_smbus;

	u32 sensitivity_mag;

	u8 xy_mode;
	u8 z_mode;

	struct lsm303c_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM303C_SPI) || \
    defined(CONFIG_INPUT_LSM303C_SPI_MODULE)
	struct lsm303c_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM303C_SPI */
};

int lsm303c_acc_probe(struct lsm303c_acc_dev *dev);
int lsm303c_acc_remove(struct lsm303c_acc_dev *dev);
int lsm303c_acc_enable(struct lsm303c_acc_dev *dev);
int lsm303c_acc_disable(struct lsm303c_acc_dev *dev);

int lsm303c_mag_probe(struct lsm303c_mag_dev *dev);
int lsm303c_mag_remove(struct lsm303c_mag_dev *dev);
int lsm303c_mag_enable(struct lsm303c_mag_dev *dev);
int lsm303c_mag_disable(struct lsm303c_mag_dev *dev);

#endif	/* __LSM303C_H__ */

