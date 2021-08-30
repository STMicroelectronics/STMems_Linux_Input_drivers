/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: l3g4200d.h
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.3 sysfs
* Date			: 2011/Sep/24
* Description		: L3G4200D digital output gyroscope sensor API
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
* 1.0		| 2010/Aug/19	| Carmine Iascone	| First Release
* 1.1.0		| 2011/02/28	| Matteo Dameno		| Self Test Added
* 1.1.1		| 2011/05/25	| Matteo Dameno		| Corrects Polling Bug
* 1.1.2		| 2011/05/30	| Matteo Dameno		| Corrects ODR Bug
* 1.1.3		| 2011/06/24	| Matteo Dameno		| Corrects ODR Bug
* 1.1.4		| 2011/09/24	| Matteo Dameno		| forces BDU setting
*******************************************************************************/

#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#define L3G4200D_MIN_POLL_PERIOD_MS	2

#define L3G4200D_DEV_NAME	"l3g4200d_gyr"

#define L3G4200D_GYR_FS_250DPS	0x00
#define L3G4200D_GYR_FS_500DPS	0x10
#define L3G4200D_GYR_FS_2000DPS	0x30

#define L3G4200D_GYR_ENABLED	1
#define L3G4200D_GYR_DISABLED	0

#ifdef __KERNEL__

#define BUFF_RX_MAX_LENGTH	500
#define BUFF_TX_MAX_LENGTH	500

#define	RESUME_ENTRIES		5

struct l3g4200d_data;

struct l3g4200d_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

struct l3g4200d_triple {
	short x, y, z;
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

struct l3g4200d_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct l3g4200d_transfer_function {
	int (*write)(struct l3g4200d_data *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct l3g4200d_data *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct l3g4200d_data {
	const char *name;
	struct device *dev;
	struct l3g4200d_platform_data *pdata;
	struct mutex lock;
	u16 bustype;
	struct input_polled_dev *input_poll_dev;
	int hw_initialized;
	int selftest_enabled;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];
	struct l3g4200d_transfer_function *tf;
	struct l3g4200d_transfer_buffer tb;
};

/* Input events used by l3g4200d driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

int l3g4200d_common_probe(struct l3g4200d_data *gyro);
int l3g4200d_common_remove(struct l3g4200d_data *gyro);

#ifdef CONFIG_PM
int l3g4200d_common_suspend(struct l3g4200d_data *gyro);
int l3g4200d_common_resume(struct l3g4200d_data *gyro);
#endif /* CONFIG_PM */

#endif /* __KERNEL__ */

#endif  /* __L3G4200D_H__ */
