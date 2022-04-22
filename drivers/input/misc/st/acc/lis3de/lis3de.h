/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
*
* File Name	: lis3de.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
*		: Mario Tesi (mario.tesi@st.com)
*		: Both authors are willing to be considered the contact
*		: and update points for the driver.
* Version	: V.1.0.2
* Date		: 2013/Jun/17
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

#ifndef	__LIS3DE_H__
#define	__LIS3DE_H__

/* Uncomment if want Enable/Disable sensor on Open/Close input device
 * #define LIS3DE_EN_OPEN
 */

#define	LIS3DE_ACC_DEV_NAME		"lis3de"

/************************************************/
/* 	Output data: ug				*/
/*	Sysfs enable: enable_device		*/
/*	Sysfs odr: pollrate_ms			*/
/*	Sysfs full scale: range			*/
/************************************************/

#define	LIS3DE_ACC_MIN_POLL_PERIOD_MS	1

#ifdef __KERNEL__

/* to set gpios numb connected to interrupt pins,
* the unused ones have to be set to -EINVAL
*/
#define LIS3DE_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define LIS3DE_ACC_DEFAULT_INT2_GPIO	(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define	LIS3DE_ACC_FS_MASK	0x30
#define LIS3DE_ACC_G_2G		0x00
#define LIS3DE_ACC_G_4G		0x10
#define LIS3DE_ACC_G_8G		0x20
#define LIS3DE_ACC_G_16G	0x30

#define BUFF_RX_MAX_LENGTH	500
#define BUFF_TX_MAX_LENGTH	500

#define	RESUME_ENTRIES		17

struct lis3de_status;

struct lis3de_platform_data {
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

struct lis3de_transfer_buffer {
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct lis3de_transfer_function {
	int (*write)(struct lis3de_status *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct lis3de_status *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct lis3de_status {
	const char *name;
	struct lis3de_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	struct device *dev;
	u16 bustype;

	unsigned int sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct lis3de_transfer_function *tf;
	struct lis3de_transfer_buffer tb;

#ifdef DEBUG
	u8 reg_addr;
#endif
};

/* Input events used by lis2dh driver */
#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

int lis3de_common_probe(struct lis3de_status *stat);
int lis3de_common_remove(struct lis3de_status *stat);

#ifdef CONFIG_PM_SLEEP
int lis3de_common_resume(struct lis3de_status *stat);
int lis3de_common_suspend(struct lis3de_status *stat);
#endif /* CONFIG_PM_SLEEP */

#endif	/* __KERNEL__ */

#endif	/* __LIS3DE_H__ */

