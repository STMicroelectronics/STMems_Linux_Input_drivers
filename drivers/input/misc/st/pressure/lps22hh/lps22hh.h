/*
 * STMicroelectronics lps22hh driver
 *
 * Copyright 2020 STMicroelectronics Inc.
 *
 * Authors: AMG MSD DIVISION
 *        : Mario Tesi (mario.tesi@st.com)
 *
 * Version: 1.0.0
 *
 *******************************************************************************
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
 */
#ifndef	__LPS22HH_H__
#define	__LPS22HH_H__

#define LPS22HH_MODULE_VERSION		"1.0.0"

/* 5ms is poll rate for 200Hz */
#define LPS22HH_MIN_POLL_PERIOD_MS	5
#define	LPS22HH_DEV_NAME		"lps22hh"
#define	LPS22CH_DEV_NAME		"lps22ch"
#define	LPS27HHW_DEV_NAME		"lps27hhw"
#define	LPS27HHTW_DEV_NAME		"lps27hhtw"


#define LPS22HH_RX_MAX_LENGTH		64
#define LPS22HH_TX_MAX_LENGTH		64

typedef enum {
	RES_CTRL_REG1 = 0,
	RES_CTRL_REG2,
	RES_CTRL_REG3,
	RESUME_ENTRIES
} resume_entries;

struct lps22hh_data;

struct lps22hh_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

struct lps22hh_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LPS22HH_RX_MAX_LENGTH];
	u8 tx_buf[LPS22HH_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lps22hh_transfer_function {
	int (*write) (struct lps22hh_data *cdata, u8 reg_addr, int len, u8 *data);
	int (*read) (struct lps22hh_data *cdata, u8 reg_addr, int len, u8 *data);
};

struct lps22hh_data {
	const char *name;
	struct lps22hh_platform_data *pdata;

	struct mutex lock;
	struct workqueue_struct *workqueue;
	struct hrtimer hr_timer;
	ktime_t delta_ts;
	struct work_struct input_work;
	struct input_dev *input_dev_pres;
	struct device *dev;

	int64_t timestamp;

	int hw_initialized;
	int hw_working;
	u16 bustype;
	int irq;
	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

	s32 press;
	s16 temperature;

	const struct lps22hh_transfer_function *tf;
	struct lps22hh_transfer_buffer tb;
};

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

int lps22hh_common_probe(struct lps22hh_data *prs);
int lps22hh_common_remove(struct lps22hh_data *prs);

#ifdef CONFIG_PM_SLEEP
int lps22hh_common_resume(struct lps22hh_data *prs);
int lps22hh_common_suspend(struct lps22hh_data *prs);
#endif /* CONFIG_PM_SLEEP */

#endif  /* __LPS22HH_H__ */
