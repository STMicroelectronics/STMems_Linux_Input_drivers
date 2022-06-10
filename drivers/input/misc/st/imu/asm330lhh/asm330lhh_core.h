/*
 * STMicroelectronics asm330lhh driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <giuseppe.barba@st.com>
 * v 1.0
 * Licensed under the GPL-2.
 */

#ifndef DRIVERS_INPUT_MISC_ASM330LHH_CORE_H_
#define DRIVERS_INPUT_MISC_ASM330LHH_CORE_H_

#define ASM330LHH_DEV_NAME		"asm330lhh"
#define LSM6DSR_DEV_NAME		"lsm6dsr"

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
	ASM330LHH_ACCEL = 0,
	ASM330LHH_GYRO,
	ASM330LHH_TEMP,
	ASM330LHH_MAX_ID,
};

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define ASM330LHH_RX_MAX_LENGTH		64
#define ASM330LHH_TX_MAX_LENGTH		64

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

struct asm330lhh_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ASM330LHH_RX_MAX_LENGTH];
	u8 tx_buf[ASM330LHH_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct asm330lhh_data;

struct asm330lhh_transfer_function {
	int (*write) (struct asm330lhh_data *cdata, u8 reg_addr, int len, u8 *data,
		      bool b_lock);
	int (*read) (struct asm330lhh_data *cdata, u8 reg_addr, int len, u8 *data,
		     bool b_lock);
};

struct asm330lhh_sensor_data {
	struct asm330lhh_data *cdata;
	const char* name;
	int64_t timestamp;
	u8 enabled;
	u32 c_odr;
	u32 c_gain;
	u8 sindex;
	u8 sample_to_discard;
	struct input_dev *input_dev;
	unsigned int poll_interval;
#ifndef CONFIG_ASM330LHH_IRQ_THREAD
	struct hrtimer hr_timer;
	struct work_struct input_work;
	ktime_t oldktime;
#endif /* CONFIG_ASM330LHH_IRQ_THREAD */
};

struct asm330lhh_data {
	const char *name;
	u8 drdy_int_pin;
	int irq;
	int64_t timestamp;
	struct device *dev;
	struct asm330lhh_sensor_data sensors[ASM330LHH_MAX_ID];
	struct mutex bank_registers_lock;
	const struct asm330lhh_transfer_function *tf;
	struct asm330lhh_transfer_buffer tb;
};

int asm330lhh_common_probe(struct asm330lhh_data *cdata, int irq, u16 bustype);
void asm330lhh_common_remove(struct asm330lhh_data *cdata, int irq);

#ifdef CONFIG_PM_SLEEP
int asm330lhh_common_suspend(struct asm330lhh_data *cdata);
int asm330lhh_common_resume(struct asm330lhh_data *cdata);
#endif /* CONFIG_PM_SLEEP */

#endif /* DRIVERS_INPUT_MISC_ASM330LHH_CORE_H_ */
