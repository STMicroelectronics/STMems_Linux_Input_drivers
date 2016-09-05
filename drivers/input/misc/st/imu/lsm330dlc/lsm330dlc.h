/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lsm330dlc_sysfs.h
* Authors            : MH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Carmine Iascone (carmine.iascone@st.com)
		     : Lorenzo Bianconi (lorenzo.bianconi@st.com)
* Version            : V.1.0.12
* Date               : 2016/May/06
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
/*******************************************************************************
Version History.

 Revision 1.0.10: 2011/Aug/16
  merges release 1.0.10 acc + 1.1.5.3 gyr
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  renamed field g_range to fs_range in lsm330dlc_acc_platform_data
  replaced defines SA0L and SA0H with LSM330DLC_SAD0x
*******************************************************************************/

#ifndef	__LSM330DLC_H__
#define	__LSM330DLC_H__


#define	LSM330DLC_ACC_DEV_NAME	"lsm330dlc_acc"
#define LSM330DLC_GYR_DEV_NAME	"lsm330dlc_gyr"

#define LSM330DLC_ACC_RESUME_ENTRIES	17
#define LSM330DLC_GYR_RESUME_ENTRIES	6

/* Input events used by lsm303agr driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

struct lsm330dlc_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

#if defined(CONFIG_INPUT_LSM330DLC_SPI) || \
    defined(CONFIG_INPUT_LSM330DLC_SPI_MODULE)
#define LSM330DLC_RX_MAX_LENGTH		500
#define LSM330DLC_TX_MAX_LENGTH		500

struct lsm330dlc_transfer_buffer {
	u8 rx_buf[LSM330DLC_RX_MAX_LENGTH];
	u8 tx_buf[LSM330DLC_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_LSM330DLC_SPI */

struct lsm330dlc_acc_dev {
	struct device *dev;
	const char *name;
	u16 bus_type;

	int hw_initialized;
	/* hw_working = -1 means not tested yet */
	int hw_working;
	atomic_t enabled;

	u8 resume_state[LSM330DLC_ACC_RESUME_ENTRIES];
	u8 sensitivity;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;

	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	const struct lsm330dlc_transfer_function *tf;

	struct lsm330dlc_acc_platform_data *pdata;
#if defined(CONFIG_INPUT_LSM330DLC_SPI) || \
    defined(CONFIG_INPUT_LSM330DLC_SPI_MODULE)
	struct lsm330dlc_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM330DLC_SPI */

#ifdef LSM330DLC_DEBUG
	u8 reg_addr;
#endif
};

struct lsm330dlc_gyr_dev {
	struct device *dev;
	const char *name;
	u16 bus_type;

	struct lsm330dlc_gyr_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[LSM330DLC_GYR_RESUME_ENTRIES];

	/* interrupt related */
	int irq;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;

	/* fifo related */
	u8 watermark;
	u8 fifomode;

	const struct lsm330dlc_transfer_function *tf;
#if defined(CONFIG_INPUT_LSM330DLC_SPI) || \
    defined(CONFIG_INPUT_LSM330DLC_SPI_MODULE)
	struct lsm330dlc_transfer_buffer tb;
#endif /* CONFIG_INPUT_LSM330DLC_SPI */
};

struct lsm330dlc_acc_platform_data {
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

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

struct lsm330dlc_gyr_platform_data {
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

int lsm330dlc_acc_probe(struct lsm330dlc_acc_dev *dev);
int lsm330dlc_acc_remove(struct lsm330dlc_acc_dev *dev);
int lsm330dlc_acc_enable(struct lsm330dlc_acc_dev *dev);
int lsm330dlc_acc_disable(struct lsm330dlc_acc_dev *dev);

int lsm330dlc_gyr_probe(struct lsm330dlc_gyr_dev *dev);
int lsm330dlc_gyr_remove(struct lsm330dlc_gyr_dev *dev);
int lsm330dlc_gyr_resume(struct lsm330dlc_gyr_dev *dev);
int lsm330dlc_gyr_suspend(struct lsm330dlc_gyr_dev *dev);

#endif	/* __LSM330DLC_H__ */

