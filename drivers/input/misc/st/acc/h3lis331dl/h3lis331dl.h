/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name		: h3lis331dl.h
* Authors		: MSH - Motion Mems BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Mario Tesi (mario.tesi@st.com)
* Version		: V 1.7.1
* Date			: 2012/10/07
* Description		: H3LIS331DL 3D accelerometer sensor API
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

#ifndef __H3LIS331DL_H__
#define __H3LIS331DL_H__

/*
#define	DEBUG
*/

/* Uncomment if want Enable/Disable sensor on Open/Close input device
#define H3LIS331DL_EN_OPEN
*/

/* Device support autoincrement address on spi/i2c interface */
#define HAS_IF_AUTO_INCREMENT

#define	H3LIS331DL_ACC_DEV_NAME	"h3lis331dl"

#define	H3LIS331DL_ACC_MIN_POLL_PERIOD_MS 1

/* to set gpios numb connected to interrupt pins,
* the unused ones have to be set to -EINVAL
*/
#define H3LIS331DL_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define H3LIS331DL_ACC_DEFAULT_INT2_GPIO	(-EINVAL)

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	H3LIS331DL_ACC_FS_MASK	0x30
#define H3LIS331DL_ACC_G_100G 	0x00
#define H3LIS331DL_ACC_G_200G 	0x10
#define H3LIS331DL_ACC_G_400G 	0x30

/* Accelerometer Sensor Operating Mode */
#define H3LIS331DL_ACC_ENABLE	0x01
#define H3LIS331DL_ACC_DISABLE	0x00
#define H3LIS331DL_ACC_PM_NORMAL	0x20
#define H3LIS331DL_ACC_PM_OFF	H3LIS331DL_ACC_DISABLE

#define BUFF_RX_MAX_LENGTH	500
#define BUFF_TX_MAX_LENGTH	500

#define	RESUME_ENTRIES		12

#ifdef __KERNEL__

struct h3lis331dl_data;

struct h3lis331dl_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

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

struct h3lis331dl_transfer_buffer {
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct h3lis331dl_transfer_function {
	int (*write)(struct h3lis331dl_data *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct h3lis331dl_data *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct h3lis331dl_data {
	const char *name;
	struct h3lis331dl_platform_data *pdata;

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
	u8 sensitivity;
	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct h3lis331dl_transfer_function *tf;
	struct h3lis331dl_transfer_buffer tb;

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

int h3lis331dl_common_probe(struct h3lis331dl_data *stat);
int h3lis331dl_common_remove(struct h3lis331dl_data *stat);

#ifdef CONFIG_PM_SLEEP
int h3lis331dl_common_resume(struct h3lis331dl_data *stat);
int h3lis331dl_common_suspend(struct h3lis331dl_data *stat);
#endif /* CONFIG_PM_SLEEP */

#endif /* __KERNEL__ */

#endif  /* __H3LIS331DL_H__ */
