/*
********************* (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name          : lis3dsh.h
* Authors            : MH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
* Version            : V.1.2.2
* Date               : 2012/Dec/15
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
	V.1.2.2		Modified state program loadiing defines, removed
			state machine program.
********************************************************************************
SYSFS interface
- range: set full scale
	-> accelerometer: 	2,4,6,8,16 				[g]
- poll_period_ms: set 1/ODR
	-> accelerometer:	LIS3DH_ACC_MIN_POLL_PERIOD_MS < t	[ms]
- enable_device: enable/disable sensor					[1/0]


INPUT subsystem: NOTE-> output data INCLUDE the sensitivity in accelerometer.
- accelerometer:	abs_x, abs_y, abs_z		[ug]
*******************************************************************************/

#ifndef __LIS3DSH_H__
#define __LIS3DSH_H__


#define LIS3DSH_ACC_DEV_NAME		"lis3dsh_acc"


/* Poll Interval */
#define LIS3DSH_ACC_MIN_POLL_PERIOD_MS	1


#ifdef __KERNEL__

/* Interrupt */
#define LIS3DSH_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define LIS3DSH_ACC_DEFAULT_INT2_GPIO	(-EINVAL)


/* Accelerometer Sensor Full Scale */
#define LIS3DSH_ACC_G_2G		0x00
#define LIS3DSH_ACC_G_4G		0x08
#define LIS3DSH_ACC_G_6G		0x10
#define LIS3DSH_ACC_G_8G		0x18
#define LIS3DSH_ACC_G_16G		0x20

#define LIS3DSH_RESUME_ENTRIES		43
#define LIS3DSH_STATE_PR_SIZE		16

#define BUFF_RX_MAX_LENGTH		500
#define BUFF_TX_MAX_LENGTH		500

/* Input Event Type */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE

#define BUFF_RX_MAX_LENGTH		500
#define BUFF_TX_MAX_LENGTH		500

struct lis3dsh_status;

struct lis3dsh_platform_data {
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

struct lis3dsh_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct lis3dsh_transfer_function {
	int (*write)(struct lis3dsh_status *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct lis3dsh_status *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct lis3dsh_status {
	const char *name;
	struct lis3dsh_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	u16 bustype;
	u16 sensitivity;
	struct device *dev;
	u8 stateprogs_enable_setting;
	u8 resume_state[LIS3DSH_RESUME_ENTRIES];
	u8 resume_stmach_program1[LIS3DSH_STATE_PR_SIZE];
	u8 resume_stmach_program2[LIS3DSH_STATE_PR_SIZE];
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct lis3dsh_transfer_function *tf;
	struct lis3dsh_transfer_buffer tb;

#ifdef DEBUG
	u8 reg_addr;
#endif
};

#ifdef CONFIG_PM_SLEEP
int lis3dsh_common_resume(struct lis3dsh_status *acc);
int lis3dsh_common_suspend(struct lis3dsh_status *acc);
#endif

int lis3dsh_common_remove(struct lis3dsh_status *acc);
int lis3dsh_common_probe(struct lis3dsh_status *acc);

#endif /* __KERNEL__ */

#endif /* __LIS3DSH_H__ */
