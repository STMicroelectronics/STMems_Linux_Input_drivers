 /*
  * STMicroelectronics h3lis100dl Accelerometer driver
  * Based on lis331dlh input driver
  *
  * Copyright 2016 STMicroelectronics Inc.
  *
  * Armando Visconti <armando.visconti@st.com>
  *
  * Licensed under the GPL-2.
  */

#ifndef __H3LIS100DL_H__
#define __H3LIS100DL_H__

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define SAD0L				0x00
#define SAD0H				0x01
#define H3LIS100DL_I2C_SADROOT		0x0C
#define H3LIS100DL_I2C_SAD_L		((H3LIS100DL_I2C_SADROOT<<1)|SAD0L)
#define H3LIS100DL_I2C_SAD_H		((H3LIS100DL_I2C_SADROOT<<1)|SAD0H)
#define	H3LIS100DL_DEV_NAME		"h3lis100dl"

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	H3LIS100DL_FS_MASK		0x30
#define H3LIS100DL_G_100G 		0x00

/* Accelerometer Sensor Operating Mode */
#define H3LIS100DL_ENABLE		0x01
#define H3LIS100DL_DISABLE		0x00
#define H3LIS100DL_PM_NORMAL		0x20
#define H3LIS100DL_PM_OFF		H3LIS100DL_DISABLE

#define H3LIS100DL_RX_MAX_LENGTH	500
#define H3LIS100DL_TX_MAX_LENGTH	500

#define AUTO_INCREMENT			0x80

#define	RESUME_ENTRIES			12

#ifdef __KERNEL__

struct h3lis100dl_data;

struct h3lis100dl_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[H3LIS100DL_RX_MAX_LENGTH];
	u8 tx_buf[H3LIS100DL_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct h3lis100dl_transfer_function {
	int (*write)(struct h3lis100dl_data *cdata, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct h3lis100dl_data *cdata, u8 reg_addr, int len,
		    u8 *data);
};

struct h3lis100dl_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct h3lis100dl_data {
	const char *name;
	struct device *dev;
	struct h3lis100dl_platform_data *pdata;

	struct mutex lock;
	struct work_struct input_work;

	struct hrtimer hr_timer;
	ktime_t ktime;

	struct input_dev *input_dev;
	u16 bustype;

	int64_t timestamp;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	int selftest_enabled;

	atomic_t enabled;
	int on_before_suspend;

	u16 sensitivity;

	u8 resume_state[RESUME_ENTRIES];
	const struct h3lis100dl_transfer_function *tf;
	struct h3lis100dl_transfer_buffer tb;

#ifdef DEBUG
	u8 reg_addr;
#endif
};

int h3lis100dl_common_probe(struct h3lis100dl_data *acc);
int h3lis100dl_common_remove(struct h3lis100dl_data *acc);

#ifdef CONFIG_PM
int h3lis100dl_common_resume(struct h3lis100dl_data *cdata);
int h3lis100dl_common_suspend(struct h3lis100dl_data *cdata);
#endif /* CONFIG_PM */

#endif /* __KERNEL__ */
#endif  /* __H3LIS100DL_H__ */
