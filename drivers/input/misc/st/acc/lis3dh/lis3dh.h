/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis3dh_acc.h
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Mario Tesi <mario.tesi@st.com>
* Version	: V.1.0.14
* Date		: 2016/Apr/26
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

 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
  renamed field g_range to fs_range in lis3dh_acc_platform_data;
  replaced defines SA0L and SA0H with LIS3DH_SAD0x
 Revision 1.0.13: 2013/Feb/25
  modified acc_remove function;
 Revision 1.0.14: 2016/Apr/26
  added new i2c and spi interface
*******************************************************************************/
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#ifndef	__LIS3DH_H__
#define	__LIS3DH_H__

/* Uncomment if want enable/disable on open/close input device */
//#define LIS3DH_EN_OPEN_CLOSE

#define	LIS3DH_ACC_DEV_NAME		"lis3dh_acc"

#define	LIS3DH_ACC_MIN_POLL_PERIOD_MS	1

#ifdef __KERNEL__

#define LIS3DH_SAD0L			0x00
#define LIS3DH_SAD0H			0x01

#define LIS3DH_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define LIS3DH_ACC_DEFAULT_INT2_GPIO	(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define	LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G			0x00
#define LIS3DH_ACC_G_4G			0x10
#define LIS3DH_ACC_G_8G			0x20
#define LIS3DH_ACC_G_16G		0x30

#define BUFF_RX_MAX_LENGTH		500
#define BUFF_TX_MAX_LENGTH		500

#define	RESUME_ENTRIES			17

struct lis3dh_acc_status;

struct lis3dh_acc_platform_data {
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

struct lis3dh_acc_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct lis3dh_acc_transfer_function {
	int (*write)(struct lis3dh_acc_status *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct lis3dh_acc_status *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct lis3dh_acc_status {
	const char *name;
	struct lis3dh_acc_platform_data *pdata;

	struct mutex lock;
	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;
	struct device *dev;
	u16 bustype;
	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq;
	int64_t timestamp;

	struct lis3dh_acc_transfer_function *tf;
	struct lis3dh_acc_transfer_buffer tb;
};

static inline s64 lis3dh_acc_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

/* Input events used by lis3dh driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

int lis3dh_acc_probe(struct lis3dh_acc_status *stat, int irq);
int lis3dh_acc_remove(struct lis3dh_acc_status *stat);

#ifdef CONFIG_PM_SLEEP
int lis3dh_acc_common_resume(struct lis3dh_acc_status *stat);
int lis3dh_acc_common_suspend(struct lis3dh_acc_status *stat);
#endif /* CONFIG_PM_SLEEP */

#endif	/* __KERNEL__ */

#endif	/* __LIS3DH_H__ */
