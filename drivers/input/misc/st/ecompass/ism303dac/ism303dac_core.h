/*
 * STMicroelectronics ism303dac_core.h driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef DRIVERS_INPUT_MISC_ISM303DAC_CORE_H_
#define DRIVERS_INPUT_MISC_ISM303DAC_CORE_H_

#define ISM303DAC_OUT_XYZ_SIZE		8

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
#define ISM303DAC_EN_BIT			0x01
#define ISM303DAC_DIS_BIT		0x00

#define ISM303DAC_RX_MAX_LENGTH		(500)
#define ISM303DAC_TX_MAX_LENGTH		(500)

#define ADD_DEVICE_ENABLE_ATTR		static \
					DEVICE_ATTR(enable, \
						S_IWUSR | S_IRUGO, \
						st_ism303dac_sensor_get_enable, \
						st_ism303dac_sensor_set_enable);

#define ADD_DEVICE_POLLING_ATTR		static \
						DEVICE_ATTR(polling_rate, \
						S_IWUSR | S_IRUGO, \
						st_ism303dac_sensor_get_polling_rate, \
						st_ism303dac_sensor_set_polling_rate);
#define ISM303DAC_CONCAT(a, b)		(a b)

#define to_dev(obj) container_of(obj, struct device, kobj)

#define ISM303DAC_DEV_NAME			"ism303dac"
#define ISM303DAC_ACC_INPUT_DEV_NAME		"ism303dac_acc"
#define ISM303DAC_I2C_ADDR			0x1e

struct ism303dac_platform_data {
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

struct ism303dac_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ISM303DAC_RX_MAX_LENGTH];
	u8 tx_buf[ISM303DAC_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_common_data;

struct st_sensor_transfer_function {
	int (*write) (struct st_common_data *cdata, u8 addr, int len, u8 *data);
	int (*read) (struct st_common_data *cdata, u8 addr, int len, u8 *data);
};

enum {
	ISM303DAC_ACCEL = 0,
	ISM303DAC_FF,
	ISM303DAC_TAP,
	ISM303DAC_DOUBLE_TAP,
	ISM303DAC_WAKEUP,
	ISM303DAC_ACTIVITY,
	ISM303DAC_SENSORS_NUMB,
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
	struct ism303dac_transfer_buffer tb;
};

static inline s64 st_sensor_get_time_ns(void) {
	return ktime_to_ns(ktime_get_boottime());
}

enum hrtimer_restart st_ism303dac_sensor_poll_function(struct hrtimer *timer);
int st_ism303dac_sensor_common_probe(struct st_common_data *cdata, int irq);
void st_ism303dac_sensor_common_remove(struct st_common_data *cdata);
int st_ism303dac_sensor_common_resume(struct st_common_data *cdata);
int st_ism303dac_sensor_common_suspend(struct st_common_data *cdata);
int st_ism303dac_sensor_input_init(struct st_sensor_data *sdata, const char* description,
								bool is_3_axis);
int st_ism303dac_sensor_write_data(struct st_common_data *cdata, u8 reg_addr,
							u8 mask, u8 data);
void st_ism303dac_sensor_input_cleanup(struct st_sensor_data *sdata);
void st_ism303dac_sensor_report_3axes_event(struct st_sensor_data *sdata, s32 *xyz,
								s64 timestamp);
void st_ism303dac_sensor_report_single_event(struct st_sensor_data *sdata, s32 data);

ssize_t st_ism303dac_sensor_get_enable(struct device *dev,
				    struct device_attribute *attr,
				    char *buf);
ssize_t st_ism303dac_sensor_set_enable(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);
ssize_t st_ism303dac_sensor_get_polling_rate(struct device *dev,
				   struct device_attribute *attr, char *buf);
ssize_t st_ism303dac_sensor_set_polling_rate(struct device *dev,
				   struct device_attribute *attr, const char *buf,
				   size_t count);

int ism303dac_acc_enable(struct st_common_data *cdata);
int ism303dac_acc_disable(struct st_common_data *cdata);
int ism303dac_acc_probe(struct st_common_data *cdata);
int ism303dac_acc_remove(struct st_common_data *cdata);

int ism303dac_mag_enable(struct st_common_data *cdata);
int ism303dac_mag_disable(struct st_common_data *cdata);
int ism303dac_mag_probe(struct st_common_data *cdata);
int ism303dac_mag_remove(struct st_common_data *cdata);

#endif /* DRIVERS_INPUT_MISC_ISM303DAC_CORE_H_ */
