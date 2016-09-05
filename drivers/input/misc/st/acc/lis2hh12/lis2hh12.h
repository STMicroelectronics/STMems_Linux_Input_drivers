
/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis2hh12.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Mario Tesi (mario.tesi@st.com)
* Version	: V.1.1.1
* Date		: 2016/May/05
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

 Revision 1.0.0 25/Feb/2013
  first revision
  supports sysfs;
 Revision 1.1.0 28/Mar/2013
  introduces hr_timers for polling;
 Revision 1.1.1 05/May/2016
  introduces transfer function block
*******************************************************************************/

#ifndef	__LIS2HH12_H__
#define	__LIS2HH12_H__

/* Uncomment if want Enable/Disable sensor on Open/Close input device
 * #define LIS2HH_EN_OPEN
 */

#define	LIS2HH12_ACC_DEV_NAME		"lis2hh12_acc"

#define	LIS2HH12_ACC_MIN_POLL_PERIOD_MS	2

#ifdef __KERNEL__

#define LIS2HH12_ACC_SAD0L		0x10
#define LIS2HH12_ACC_SAD0H		0x01
#define LIS2HH12_ACC_I2C_SADROOT	0x07

/* I2C address if acc SA0 pin to GND */
#define LIS2HH12_ACC_I2C_SAD_L		((LIS2HH12_ACC_I2C_SADROOT<<2)| \
					 LIS2HH12_ACC_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define LIS2HH12_ACC_I2C_SAD_H		((LIS2HH12_ACC_I2C_SADROOT<<2)| \
					 LIS2HH12_ACC_SAD0H)

/* to set gpios numb connected to interrupt pins,
 * the unused ones have to be set to -EINVAL
 */
#define LIS2HH12_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define LIS2HH12_ACC_DEFAULT_INT2_GPIO	(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define	LIS2HH12_ACC_FS_MASK		0x30
#define LIS2HH12_ACC_FS_2G		0x00
#define LIS2HH12_ACC_FS_4G		0x20
#define LIS2HH12_ACC_FS_8G		0x30

#define BUFF_RX_MAX_LENGTH		500
#define BUFF_TX_MAX_LENGTH		500

#define RESUME_ENTRIES			18
struct lis2hh12_status;

struct lis2hh12_platform_data {
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

struct lis2hh12_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct lis2hh12_transfer_function {
	int (*write)(struct lis2hh12_status *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct lis2hh12_status *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct lis2hh12_status {
	const char *name;
	struct lis2hh12_platform_data *pdata;
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
	int on_before_suspend;
	struct device *dev;
	u16 bustype;
	u8 sensitivity;
	u8 resume_state[RESUME_ENTRIES];
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct lis2hh12_transfer_function *tf;
	struct lis2hh12_transfer_buffer tb;

#ifdef DEBUG
	u8 reg_addr;
#endif
};

/* Input events used by lis2dh driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

int lis2hh12_common_probe(struct lis2hh12_status *stat);
int lis2hh12_common_remove(struct lis2hh12_status *stat);

#ifdef CONFIG_PM
int lis2hh12_common_resume(struct lis2hh12_status *stat);
int lis2hh12_common_suspend(struct lis2hh12_status *stat);
#endif /* CONFIG_PM */

#endif	/* __KERNEL__ */

#endif	/* __LIS2HH12_H__ */

