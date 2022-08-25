// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Matteo Dameno <matteo.dameno@st.com>
 *
 * Licensed under the GPL-2.
 */


#ifndef __LPS22DF__
#define __LPS22DF__

#define LPS22DF_PRS_MIN_POLL_PERIOD_MS	5
#define	LPS22DF_PRS_DEV_NAME		"lps22df"

#define LPS22DF_RX_MAX_LENGTH	64
#define LPS22DF_TX_MAX_LENGTH	64


/* RESUME STATE INDICES */
enum resume_entries {
	RES_CTRL_REG1 = 0,
	RES_CTRL_REG2,
	RES_CTRL_REG3,
	RES_CTRL_REG4,
	RESUME_ENTRIES
};

struct lps22df_prs_data;

struct lps22df_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

struct lps22df_prs_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LPS22DF_RX_MAX_LENGTH];
	u8 tx_buf[LPS22DF_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lps22df_prs_transfer_function {
	int (*write)(struct lps22df_prs_data *cdata, u8 reg_addr,
							int len, u8 *data);
	int (*read)(struct lps22df_prs_data *cdata, u8 reg_addr,
							int len, u8 *data);
};

struct lps22df_prs_data {
	const char *name;
	struct lps22df_prs_platform_data *pdata;

	struct mutex lock;
	struct workqueue_struct *workqueue;
	struct hrtimer hr_timer;
	ktime_t delta_ts;
	struct work_struct input_work;
	struct input_dev *input_dev_pres;
	struct device *dev;

	int64_t timestamp;

	int hw_initialized;
	/* hw_working =-1 means not answering on bus */
	int hw_working;
	u16 bustype;
	int irq;
	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef DEBUG
	u8 reg_addr;
#endif
	const struct lps22df_prs_transfer_function *tf;
	struct lps22df_prs_transfer_buffer tb;
};

#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED
#define INPUT_EVENT_Z		MSC_GESTURE

#ifdef LPS22DF_TIMESTAMP
#define INPUT_EVENT_TIME_MSB	MSC_SCAN
#define INPUT_EVENT_TIME_LSB	MSC_MAX
#endif /* LPS22DF_TIMESTAMP */

int lps22df_common_probe(struct lps22df_prs_data *prs);
int lps22df_common_remove(struct lps22df_prs_data *prs);

#ifdef CONFIG_PM_SLEEP
int lps22df_common_resume(struct lps22df_prs_data *prs);
int lps22df_common_suspend(struct lps22df_prs_data *prs);
#endif /* CONFIG_PM_SLEEP */

#endif  /* __LPS22DF__ */
