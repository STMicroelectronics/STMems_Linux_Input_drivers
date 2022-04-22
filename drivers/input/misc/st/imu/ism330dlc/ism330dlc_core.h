/*
 * STMicroelectronics ism330dlc driver
 *
 * Copyright 2018 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Mario Tesi <giuseppe.barba@st.com>
 * v 1.2.2
 * Licensed under the GPL-2.
 */

#ifndef DRIVERS_INPUT_MISC_ISM330DLC_CORE_H_
#define DRIVERS_INPUT_MISC_ISM330DLC_CORE_H_

#define ISM330DLC_DEV_NAME		"ism330dlc"

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
	ISM330DLC_ACCEL = 0,
	ISM330DLC_GYRO,
	ISM330DLC_TILT,
	ISM330DLC_WAKEUP,
	ISM330DLC_SENSORS_NUMB,
};

#define DEF_ZERO			0x00

/* First Out register for Acc and Gyro */
#define ISM330DLC_ACC_OUT_X_L_ADDR	0x28
#define ISM330DLC_GYR_OUT_X_L_ADDR	0x22

/* Output data rate registers */
#define ISM330DLC_ACC_ODR_ADDR		CTRL1_ADDR
#define ISM330DLC_ACC_ODR_MASK		0xf0
#define ISM330DLC_GYR_ODR_ADDR		CTRL2_ADDR
#define ISM330DLC_GYR_ODR_MASK		0xf0

#define ISM330DLC_ACC_FS_ADDR		CTRL1_ADDR
#define ISM330DLC_GYR_FS_ADDR		CTRL2_ADDR

#define ISM330DLC_IF_INC_MASK		0x04

#define ISM330DLC_HPERF_GYR_ADDR	CTRL7_ADDR
#define ISM330DLC_HPERF_GYR_MASK	0x80

#define ISM330DLC_HPERF_ACC_ADDR	CTRL6_ADDR
#define ISM330DLC_HPERF_ACC_MASK	0x10
#define ISM330DLC_HPERF_ACC_ENABLE	0x00

#define CTRL1_ADDR			0x10
#define CTRL2_ADDR			0x11
#define CTRL3_ADDR			0x12
#define CTRL6_ADDR			0x15
#define CTRL7_ADDR			0x16

#define FUZZ				0
#define FLAT				0

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define ISM330DLC_RX_MAX_LENGTH		32
#define ISM330DLC_TX_MAX_LENGTH		32

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

struct ism330dlc_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ISM330DLC_RX_MAX_LENGTH];
	u8 tx_buf[ISM330DLC_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct ism330dlc_data;

struct ism330dlc_transfer_function {
	int (*write) (struct ism330dlc_data *cdata, u8 reg_addr, int len, u8 *data,
		      bool b_lock);
	int (*read) (struct ism330dlc_data *cdata, u8 reg_addr, int len, u8 *data,
		     bool b_lock);
};

struct ism330dlc_sensor_data {
	struct ism330dlc_data *cdata;
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

struct ism330dlc_data {
	const char *name;
	u8 drdy_int_pin;
	int irq;
	int64_t timestamp;
	struct work_struct input_work;
	struct device *dev;
	struct ism330dlc_sensor_data sensors[ISM330DLC_SENSORS_NUMB];
	struct mutex bank_registers_lock;
	const struct ism330dlc_transfer_function *tf;
	struct ism330dlc_transfer_buffer tb;
};

int ism330dlc_common_probe(struct ism330dlc_data *cdata, int irq, u16 bustype);
void ism330dlc_common_remove(struct ism330dlc_data *cdata, int irq);

#ifdef CONFIG_PM_SLEEP
int ism330dlc_common_suspend(struct ism330dlc_data *cdata);
int ism330dlc_common_resume(struct ism330dlc_data *cdata);
#endif /* CONFIG_PM_SLEEP */

#endif /* DRIVERS_INPUT_MISC_ISM330DLC_CORE_H_ */
