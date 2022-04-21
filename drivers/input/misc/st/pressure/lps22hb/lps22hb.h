/*
* STMicroelectronics lps22hb driver
*
* Copyright 2016 STMicroelectronics Inc.
*
* Authors: HESA BU - Application Team
*        : Adalberto Muhuho (adalberto.muhuho@st.com)
*        : Mario Tesi (mario.tesi@st.com)
*
* The structure of this driver is based on reference code previously delivered
* by Lorenzo Sarchi
*
* Version: 0.0.3
* Date   : 2016/May/16
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
*/
/******************************************************************************
 Revision history:

 Revision 0.0.1 2015/Dec/11:
	first (BETA) release

 Revision 0.0.2 2016/Apr/19:
	Revision 0.0.2 downgrades previous License to GPLv2

 Revision 0.0.3 2016/May/16:
	Added SPI support
******************************************************************************/
#ifndef	__LPS22_H__
#define	__LPS22_H__

#define LPS22_PRS_MIN_POLL_PERIOD_MS	13
#define	LPS22_PRS_DEV_NAME		"lps22hb"
#define	LPS22_HD_PRS_DEV_NAME		"lps22hd"

/* input define mappings */
#define ABS_PR		ABS_PRESSURE
#define ABS_TEMP	ABS_GAS

/* Output conversion factors */
#define	LPS22HB_SENSITIVITY_T	100	/* = LSB/degrC */
#define	LPS22HB_SENSITIVITY_P	4096	/* = LSB/mbar */

#define	LPS22HB_TEMPERATURE_OFFSET	0

#define LPS22HB_RX_MAX_LENGTH	500
#define LPS22HB_TX_MAX_LENGTH	500

#define	RESUME_ENTRIES		16

//#define	DEBUG

#ifdef __KERNEL__
struct lps22_prs_data;

struct lps22_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

struct lps22_prs_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LPS22HB_RX_MAX_LENGTH];
	u8 tx_buf[LPS22HB_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lps22_prs_transfer_function {
	int (*write) (struct lps22_prs_data *cdata, u8 reg_addr, int len, u8 *data);
	int (*read) (struct lps22_prs_data *cdata, u8 reg_addr, int len, u8 *data);
};

struct lps22_prs_data {
	const char *name;
	struct lps22_prs_platform_data *pdata;

	struct mutex lock;
	struct workqueue_struct *workqueue;
	struct hrtimer hr_timer;
	ktime_t delta_ts;
	struct work_struct input_work;
	struct input_dev *input_dev_pres;

	struct device *dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	u16 bustype;
	int irq;
	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef DEBUG
	u8 reg_addr;
#endif
	const struct lps22_prs_transfer_function *tf;
	struct lps22_prs_transfer_buffer tb;
};

#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

int lps22hb_common_probe(struct lps22_prs_data *prs);
int lps22hb_common_remove(struct lps22_prs_data *prs);

#ifdef CONFIG_PM_SLEEP
int lps22hb_common_resume(struct lps22_prs_data *prs);
int lps22hb_common_suspend(struct lps22_prs_data *prs);
#endif /* CONFIG_PM_SLEEP */
#endif /* __KERNEL__ */

#endif  /* __LPS22_H__ */
