/*
 * STMicroelectronics lsm6dsm driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Mario Tesi <giuseppe.barba@st.com>
 * v 1.2.2
 * Licensed under the GPL-2.
 */

#ifndef DRIVERS_INPUT_MISC_LSM6DSM_CORE_H_
#define DRIVERS_INPUT_MISC_LSM6DSM_CORE_H_

#define LSM6DSM_DEV_NAME		"lsm6dsm"

#define HZ_TO_PERIOD_NSEC(hz)		(1000 * 1000 * 1000 / ((u32)(hz)))
#define MS_TO_US(x)			({ typeof(x) _x = (x); ((_x) * \
							((typeof(x)) 1000));})
#define US_TO_NS(x)			(MS_TO_US(x))
#define MS_TO_NS(x)			(US_TO_NS(MS_TO_US(x)))
#define US_TO_MS(x)			({ typeof(x) _x = (x); ((_x) / \
							((typeof(x)) 1000));})
#define NS_TO_US(x)			(US_TO_MS(x))
#define NS_TO_MS(x)			(US_TO_MS(NS_TO_US(x)))

enum {
	LSM6DSM_ACCEL = 0,
	LSM6DSM_GYRO,
	LSM6DSM_SIGN_MOTION,
	LSM6DSM_STEP_COUNTER,
	LSM6DSM_STEP_DETECTOR,
	LSM6DSM_TILT,
	LSM6DSM_WRIST_TILT ,
	LSM6DSM_SENSORS_NUMB,
};

#define DEF_ZERO			0x00

/* First Out register for Acc and Gyro */
#define LSM6DSM_ACC_OUT_X_L_ADDR	0x28
#define LSM6DSM_GYR_OUT_X_L_ADDR	0x22

/* Output data rate registers */
#define LSM6DSM_ACC_ODR_ADDR		CTRL1_ADDR
#define LSM6DSM_ACC_ODR_MASK		0xf0
#define LSM6DSM_GYR_ODR_ADDR		CTRL2_ADDR
#define LSM6DSM_GYR_ODR_MASK		0xf0

#define LSM6DSM_ACC_FS_ADDR		CTRL1_ADDR
#define LSM6DSM_GYR_FS_ADDR		CTRL2_ADDR

#define LSM6DSM_IF_INC_MASK		0x04

#define LSM6DSM_HPERF_GYR_ADDR		CTRL7_ADDR
#define LSM6DSM_HPERF_GYR_MASK		0x80

#define LSM6DSM_HPERF_ACC_ADDR		CTRL6_ADDR
#define LSM6DSM_HPERF_ACC_MASK		0x10
#define LSM6DSM_HPERF_ACC_ENABLE	0x00

#define CTRL1_ADDR			0x10
#define CTRL2_ADDR			0x11
#define CTRL3_ADDR			0x12
#define CTRL6_ADDR			0x15
#define CTRL7_ADDR			0x16

/* Sensitivity Acc */
#define SENSITIVITY_ACC_2G		61	/** ug/LSB */
#define SENSITIVITY_ACC_4G		122	/** ug/LSB */
#define SENSITIVITY_ACC_8G		244	/** ug/LSB */
#define SENSITIVITY_ACC_16G		488	/** ug/LSB */
/* Sensitivity Gyr */
#define SENSITIVITY_GYR_125		437	/** 10udps/LSB */
#define SENSITIVITY_GYR_245		875	/** 10udps/LSB */
#define SENSITIVITY_GYR_500		1750	/** 10udps/LSB */
#define SENSITIVITY_GYR_1000		3500	/** 10udps/LSB */
#define SENSITIVITY_GYR_2000		7000	/** 10udps/LSB */

#define FUZZ				0
#define FLAT				0

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define LSM6DSM_RX_MAX_LENGTH		500
#define LSM6DSM_TX_MAX_LENGTH		500

#define to_dev(obj) 			container_of(obj, struct device, kobj)

struct reg_rw {
	u8 const address;
	u8 const init_val;
	u8 resume_val;
};

struct reg_r {
	const u8 address;
	const u8 init_val;
};

struct lsm6dsm_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LSM6DSM_RX_MAX_LENGTH];
	u8 tx_buf[LSM6DSM_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lsm6dsm_data;

struct lsm6dsm_transfer_function {
	int (*write) (struct lsm6dsm_data *cdata, u8 reg_addr, int len, u8 *data,
		      bool b_lock);
	int (*read) (struct lsm6dsm_data *cdata, u8 reg_addr, int len, u8 *data,
		     bool b_lock);
};

struct lsm6dsm_sensor_data {
	struct lsm6dsm_data *cdata;
	const char* name;
	int64_t timestamp;
	u8 enabled;
	u32 c_odr;
	u32 c_gain;
	u8 sindex;
	u8 sample_to_discard;
	struct input_dev *input_dev;
	unsigned int poll_interval;
	struct hrtimer hr_timer;
	struct work_struct input_work;
	ktime_t oldktime;
};

struct lsm6dsm_data {
	const char *name;
	bool reset_steps;
	bool sign_motion_event_ready;
	u16 steps_c;
	u8 drdy_int_pin;
	int irq;
	int64_t timestamp;
	struct work_struct input_work;
	struct device *dev;
	struct lsm6dsm_sensor_data sensors[LSM6DSM_SENSORS_NUMB];
	struct mutex bank_registers_lock;
	const struct lsm6dsm_transfer_function *tf;
	struct lsm6dsm_transfer_buffer tb;
};

int lsm6dsm_common_probe(struct lsm6dsm_data *cdata, int irq, u16 bustype);
void lsm6dsm_common_remove(struct lsm6dsm_data *cdata, int irq);

#ifdef CONFIG_PM
int lsm6dsm_common_suspend(struct lsm6dsm_data *cdata);
int lsm6dsm_common_resume(struct lsm6dsm_data *cdata);
#endif /* CONFIG_PM */

#endif /* DRIVERS_INPUT_MISC_LSM6DSM_CORE_H_ */
