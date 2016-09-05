/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: a3g4250d.h
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Mario Tesi (mario.tesi@st.com)
*			: Authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.3 sysfs
* Date			: 2016/May/23
* Description		: A3G4250D digital output gyroscope sensor API
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
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0.0		| 2016/05/23	| Mario Tesi		| ported from l3g4200d
*******************************************************************************/

#ifndef __A3G4250D_H__
#define __A3G4250D_H__

#define A3G4250D_MIN_POLL_PERIOD_MS	2

#define A3G4250D_DEV_NAME	"a3g4250d_gyr"

#define A3G4250D_GYR_ENABLED	1
#define A3G4250D_GYR_DISABLED	0

#ifdef __KERNEL__

#define BUFF_RX_MAX_LENGTH	500
#define BUFF_TX_MAX_LENGTH	500

#define	RESUME_ENTRIES		5

struct a3g4250d_data;

struct a3g4250d_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

struct a3g4250d_triple {
	short x, y, z;
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

struct a3g4250d_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct a3g4250d_transfer_function {
	int (*write)(struct a3g4250d_data *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct a3g4250d_data *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct a3g4250d_data {
	const char *name;
	struct device *dev;
	struct a3g4250d_platform_data *pdata;
	struct mutex lock;
	u16 bustype;
	int hw_initialized;
	int selftest_enabled;
	atomic_t enabled;

	s64 timestamp;
	struct hrtimer hr_timer;
	ktime_t poll_ktime;
	struct workqueue_struct *work_queue;
	struct work_struct poll_work;
	struct input_dev *input_dev;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];
	struct a3g4250d_transfer_function *tf;
	struct a3g4250d_transfer_buffer tb;
};

static inline s64 a3g4250d_get_time_ns(void)
{
	struct timespec ts;

	/*
	 * calls getnstimeofday.
	 * If hrtimers then up to ns accurate, if not microsecond.
	 */
	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

/* Input events used by a3g4250d driver */
#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

int a3g4250d_common_probe(struct a3g4250d_data *gyro);
int a3g4250d_common_remove(struct a3g4250d_data *gyro);

#ifdef CONFIG_PM
int a3g4250d_common_suspend(struct a3g4250d_data *gyro);
int a3g4250d_common_resume(struct a3g4250d_data *gyro);
#endif /* CONFIG_PM */

#endif /* __KERNEL__ */

#endif  /* __A3G4250D_H__ */
