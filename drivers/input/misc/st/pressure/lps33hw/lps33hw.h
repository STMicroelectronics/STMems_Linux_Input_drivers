/*
 * STMicroelectronics lps33hw common define
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef	__LPS33_H__
#define	__LPS33_H__

#define LPS33_PRS_MIN_POLL_PERIOD_MS	13
#define	LPS33_PRS_DEV_NAME		"lps33hw"
#define	LPS35_PRS_DEV_NAME		"lps35hw"

/* input define mappings */
#define ABS_PR		ABS_PRESSURE
#define ABS_TEMP	ABS_GAS

/* Output conversion factors */
#define	LPS33HW_SENSITIVITY_T	100	/* = LSB/degrC */
#define	LPS33HW_SENSITIVITY_P	4096	/* = LSB/mbar */

#define	LPS33HW_TEMPERATURE_OFFSET	0

#define LPS33HW_RX_MAX_LENGTH	500
#define LPS33HW_TX_MAX_LENGTH	500

#define	RESUME_ENTRIES		16

//#define	DEBUG

#ifdef __KERNEL__
struct lps33_prs_data;

struct lps33_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

struct lps33_prs_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LPS33HW_RX_MAX_LENGTH];
	u8 tx_buf[LPS33HW_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lps33_prs_transfer_function {
	int (*write) (struct lps33_prs_data *cdata,
		      u8 reg_addr, int len, u8 *data);
	int (*read) (struct lps33_prs_data *cdata,
		     u8 reg_addr, int len, u8 *data);
};

struct lps33_prs_data {
	const char *name;
	struct lps33_prs_platform_data *pdata;

	struct mutex lock;
	struct workqueue_struct *workqueue;
	struct hrtimer hr_timer;
	ktime_t delta_ts;
	struct work_struct input_work;
	struct input_dev *input_dev_pres;

	struct device *dev;

	int hw_initialized;

	/* hw_working =-1 means not tested yet */
	int hw_working;
	u16 bustype;
	int irq;
	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef DEBUG
	u8 reg_addr;
#endif
	const struct lps33_prs_transfer_function *tf;
	struct lps33_prs_transfer_buffer tb;
};

#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX

int lps33hw_common_probe(struct lps33_prs_data *prs);
int lps33hw_common_remove(struct lps33_prs_data *prs);

#ifdef CONFIG_PM_SLEEP
int lps33hw_common_resume(struct lps33_prs_data *prs);
int lps33hw_common_suspend(struct lps33_prs_data *prs);
#endif /* CONFIG_PM_SLEEP */
#endif /* __KERNEL__ */

#endif  /* __LPS33_H__ */
