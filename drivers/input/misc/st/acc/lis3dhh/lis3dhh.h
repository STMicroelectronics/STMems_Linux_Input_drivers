/*
 * STMicroelectronics lis3dhh commons define
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 * 
 * Version 1.0.0
 *
 * Licensed under the GPL-2.
 */

#include <linux/spi/spi.h>

#ifndef	__LIS3DHH_H__
#define	__LIS3DHH_H__

#define	LIS3DHH_ACC_DEV_NAME		"lis3dhh"
#define IIS3DHHC_ACC_DEV_NAME		"iis3dhhc"

/* Acc. sensor ODR fixed to 1.1 kHz */
#define	LIS3DHH_MIN_POLL_PERIOD_NS	909000

#ifdef __KERNEL__

#define BUFF_RX_MAX_LENGTH		32
#define BUFF_TX_MAX_LENGTH		32

struct lis3dhh_acc_status;

struct lis3dhh_acc_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};

struct lis3dhh_acc_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[BUFF_RX_MAX_LENGTH];
	u8 tx_buf[BUFF_TX_MAX_LENGTH] ____cacheline_aligned;
};

/* specific bus I/O functions */
struct lis3dhh_acc_transfer_function {
	int (*write)(struct lis3dhh_acc_status *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct lis3dhh_acc_status *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct lis3dhh_acc_status {
	const char *name;
	struct lis3dhh_acc_platform_data *pdata;
	struct mutex lock;
	struct work_struct input_poll_work;
	struct input_dev *input_dev;
	struct hrtimer hr_timer_poll;
	struct workqueue_struct *hr_timer_poll_work_queue;
	ktime_t polling_ktime;

	int hw_initialized;
	/* hw_working < 0 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	struct device *dev;
	u16 bustype;
	u8 resume_state;

	int irq;
	int64_t timestamp;

	/* Number of initial samples to discard every cycle power */
	u8 sample_to_discard;

	struct lis3dhh_acc_transfer_function *tf;
	struct lis3dhh_acc_transfer_buffer tb;
};

/* Input events used by lis3dh driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

int lis3dhh_common_probe(struct lis3dhh_acc_status *stat);
int lis3dhh_common_remove(struct lis3dhh_acc_status *stat);

#ifdef CONFIG_PM
int lis3dhh_common_resume(struct lis3dhh_acc_status *stat);
int lis3dhh_common_suspend(struct lis3dhh_acc_status *stat);
#endif /* CONFIG_PM */

#endif	/* __KERNEL__ */

#endif	/* __LIS3DHH_H__ */
