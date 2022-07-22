/*
 * STMicroelectronics lsm303ah_core.h driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef DRIVERS_INPUT_MISC_LSM303AH_CORE_H_
#define DRIVERS_INPUT_MISC_LSM303AH_CORE_H_

#define LSM303AH_OUT_XYZ_SIZE		8

#define HZ_TO_NSEC(hz)			(1000 * 1000 * 1000 / ((u32)(hz)))
#define HZ_TO_MSEC(hz)			(1000 / ((u32)x))
#define MS_TO_US(x)			({ typeof(x) _x = (x); ((_x) * \
							((typeof(x)) 1000));})
#define US_TO_NS(x)			(MS_TO_US(x))
#define MS_TO_NS(x)			(US_TO_NS(MS_TO_US(x)))
#define US_TO_MS(x)			({ typeof(x) _x = (x); ((_x) / \
							((typeof(x)) 1000));})
#define NS_TO_US(x)			(US_TO_MS(x))
#define NS_TO_MS(x)			(US_TO_MS(NS_TO_US(x)))
#define DEF_ZERO			(0x00)

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX
#define LSM303AH_EN_BIT			0x01
#define LSM303AH_DIS_BIT		0x00

#define LSM303AH_RX_MAX_LENGTH		(500)
#define LSM303AH_TX_MAX_LENGTH		(500)

#define ADD_DEVICE_ENABLE_ATTR		static \
					DEVICE_ATTR(enable, \
						S_IWUSR | S_IRUGO, \
						st_sensor_get_enable, \
						st_sensor_set_enable);

#define ADD_DEVICE_POLLING_ATTR		static \
						DEVICE_ATTR(polling_rate, \
						S_IWUSR | S_IRUGO, \
						st_sensor_get_polling_rate, \
						st_sensor_set_polling_rate);
#define LSM303AH_CONCAT(a, b)		(a b)

#define to_dev(obj) container_of(obj, struct device, kobj)

#define LSM303AH_DEV_NAME			"lsm303ah"
#define LSM303AH_ACC_INPUT_DEV_NAME		"lsm303ah_acc"
#define LSM303AH_I2C_ADDR			0x1e

struct lsm303ah_platform_data {
	u8 drdy_int_pin;
};

struct reg_rw {
	u8 const address;
	u8 const init_val;
	u8 resume_val;
};

struct reg_r {
	const u8 address;
	const u8 init_val;
};

struct lsm303ah_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LSM303AH_RX_MAX_LENGTH];
	u8 tx_buf[LSM303AH_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_common_data;

struct st_sensor_transfer_function {
	int (*write) (struct st_common_data *cdata, u8 addr, int len, u8 *data);
	int (*read) (struct st_common_data *cdata, u8 addr, int len, u8 *data);
};

enum {
	LSM303AH_ACCEL = 0,
	LSM303AH_STEP_C,
	LSM303AH_FF,
	LSM303AH_TAP,
	LSM303AH_DOUBLE_TAP,
	LSM303AH_STEP_D,
	LSM303AH_TILT,
	LSM303AH_SIGN_M,
	LSM303AH_WAKEUP,
	LSM303AH_ACTIVITY,
	LSM303AH_SENSORS_NUMB,
};

typedef struct {
	u8 power_mode;
	u8 enabled_sensor;
	u32 common_odr;
	s64 timestamp;
	struct work_struct irq_work;
	struct st_common_data *cdata; /* link back to common data */
} priv_data_t;

struct st_sensor_data {
	struct st_common_data *cdata;
	const char* name;
	s64 timestamp;
	u8 enabled;
	u32 c_odr;
	u32 c_gain;
	u8 sindex;
	u8 sample_to_discard;

	struct input_dev *input_dev;
	struct hrtimer hr_timer;
	struct work_struct input_work;
	ktime_t ktime;
	int (*enable)(struct st_sensor_data *sdata);
	int (*disable)(struct st_sensor_data *sdata);
	int (*write_odr)(struct st_sensor_data *sdata);
};

struct st_common_data {
	const char *name;
	u16 bus_type;
	u8 drdy_int_pin;
	u8 wai_addr;
	u8 wai_val;
	int irq;
	uint32_t sensors_count;

	void *priv_data;

	struct mutex lock;
	struct workqueue_struct *workqueue;
	struct device *dev;
	struct st_sensor_data *sensors;
	const struct st_sensor_transfer_function *tf;
	struct lsm303ah_transfer_buffer tb;
};

static inline s64 st_sensor_get_time_ns(void) {
	return ktime_to_ns(ktime_get_boottime());
}

enum hrtimer_restart st_sensor_poll_function(struct hrtimer *timer);
int st_sensor_common_probe(struct st_common_data *cdata, int irq);
void st_sensor_common_remove(struct st_common_data *cdata);
int st_sensor_common_resume(struct st_common_data *cdata);
int st_sensor_common_suspend(struct st_common_data *cdata);
int st_sensor_input_init(struct st_sensor_data *sdata, const char* description,
								bool is_3_axis);
int st_sensor_write_data(struct st_common_data *cdata, u8 reg_addr,
							u8 mask, u8 data);
void st_sensor_input_cleanup(struct st_sensor_data *sdata);
void st_sensor_report_3axes_event(struct st_sensor_data *sdata, s32 *xyz,
								s64 timestamp);
void st_sensor_report_single_event(struct st_sensor_data *sdata, s32 data);

ssize_t st_sensor_get_enable(struct device *dev,
				    struct device_attribute *attr,
				    char *buf);
ssize_t st_sensor_set_enable(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);
ssize_t st_sensor_get_polling_rate(struct device *dev,
				   struct device_attribute *attr, char *buf);
ssize_t st_sensor_set_polling_rate(struct device *dev,
				   struct device_attribute *attr, const char *buf,
				   size_t count);

int lsm303ah_acc_enable(struct st_common_data *cdata);
int lsm303ah_acc_disable(struct st_common_data *cdata);
int lsm303ah_acc_probe(struct st_common_data *cdata);
int lsm303ah_acc_remove(struct st_common_data *cdata);

int lsm303ah_mag_enable(struct st_common_data *cdata);
int lsm303ah_mag_disable(struct st_common_data *cdata);
int lsm303ah_mag_probe(struct st_common_data *cdata);
int lsm303ah_mag_remove(struct st_common_data *cdata);

#endif /* DRIVERS_INPUT_MISC_LSM303AH_CORE_H_ */
