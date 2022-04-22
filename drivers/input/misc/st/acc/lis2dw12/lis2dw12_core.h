/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name          : lis2dw12_core.h
* Authors            : AMG - Application Team
*		     : Mario Tesi <mario.tesi@st.com>
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Author is willing to be considered the contact and update
*		     : point for the driver.
* Version            : V.1.0
* Date               : 2016/Oct/18
* Description        : LIS2DW12 driver
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

#ifndef DRIVERS_INPUT_MISC_LIS2DW12_CORE_H_
#define DRIVERS_INPUT_MISC_LIS2DW12_CORE_H_

#define LIS2DW12_DEV_NAME		"lis2dw12"

#define HZ_TO_PERIOD_NSEC(hz)	(1000 * 1000 * 1000 / ((u32)(hz)))
#define MS_TO_US(x)		({ typeof(x) _x = (x); ((_x) * \
				 ((typeof(x)) 1000));})
#define US_TO_NS(x)		(MS_TO_US(x))
#define MS_TO_NS(x)		(US_TO_NS(MS_TO_US(x)))
#define US_TO_MS(x)		({ typeof(x) _x = (x); ((_x) / \
				 ((typeof(x)) 1000));})
#define NS_TO_US(x)		(US_TO_MS(x))
#define NS_TO_MS(x)		(US_TO_MS(NS_TO_US(x)))

enum {
	LIS2DW12_ACCEL = 0,
	LIS2DW12_FF,
	LIS2DW12_TAP,
	LIS2DW12_DOUBLE_TAP,
	LIS2DW12_WAKEUP,
	LIS2DW12_SENSORS_NUMB,
};

#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

#define LIS2DW12_RX_MAX_LENGTH	16
#define LIS2DW12_TX_MAX_LENGTH	16

#define to_dev(obj) container_of(obj, struct device, kobj)

struct reg_rw {
	u8 const address;
	u8 const init_val;
	u8 resume_val;
};

struct reg_r {
	const u8 address;
	const u8 init_val;
};

struct lis2dw12_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LIS2DW12_RX_MAX_LENGTH];
	u8 tx_buf[LIS2DW12_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lis2dw12_data;

struct lis2dw12_transfer_function {
	int (*write)(struct lis2dw12_data *cdata, u8 reg_addr,
		     int len, u8 *data, bool b_lock);
	int (*read)(struct lis2dw12_data *cdata, u8 reg_addr, int len, u8 *data,
		    bool b_lock);
};

struct lis2dw12_sensor_data {
	struct lis2dw12_data *cdata;
	const char* name;
	s64 timestamp;
	u8 enabled;
	u32 c_odr;
	u32 c_gain;
	u8 sindex;
	unsigned int poll_ms;
	struct input_dev *input_dev;
	struct hrtimer hr_timer;
	struct work_struct input_work;
	ktime_t oldktime;
};

struct lis2dw12_data {
	const char *name;
	u8 drdy_int_pin;
	u8 selftest_status;
	u8 power_mode;
	int irq;
	s64 timestamp;
	struct device *dev;
	struct lis2dw12_sensor_data sensors[LIS2DW12_SENSORS_NUMB];
	struct mutex bank_registers_lock;
	const struct lis2dw12_transfer_function *tf;
	struct lis2dw12_transfer_buffer tb;
};

int lis2dw12_common_probe(struct lis2dw12_data *cdata, int irq, u16 bustype);
void lis2dw12_common_remove(struct lis2dw12_data *cdata, int irq);

#ifdef CONFIG_PM_SLEEP
int lis2dw12_common_suspend(struct lis2dw12_data *cdata);
int lis2dw12_common_resume(struct lis2dw12_data *cdata);
#endif /* CONFIG_PM_SLEEP */

#endif /* DRIVERS_INPUT_MISC_LIS2DW12_CORE_H_ */
