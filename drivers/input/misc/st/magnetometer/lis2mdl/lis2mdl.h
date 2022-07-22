/*
 * STMicroelectronics lis2mdl.h driver
 *
 * Copyright 2015 STMicroelectronics Inc.
 *
 * Armando Visconti <armando.visconti@st.com>
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef DRIVERS_INPUT_MISC_LIS2MDL_CORE_H_
#define DRIVERS_INPUT_MISC_LIS2MDL_CORE_H_

#include <linux/types.h>

#define LIS2MDL_OUT_XYZ_SIZE		8

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
#define LIS2MDL_EN_BIT			0x01
#define LIS2MDL_DIS_BIT		0x00

#define LIS2MDL_RX_MAX_LENGTH		(500)
#define LIS2MDL_TX_MAX_LENGTH		(500)

#define ADD_DEVICE_ENABLE_ATTR		static \
					DEVICE_ATTR(enable, \
						S_IWUSR | S_IRUGO, \
						lis2mdl_get_enable, \
						lis2mdl_set_enable);

#define ADD_DEVICE_POLLING_ATTR		static \
						DEVICE_ATTR(polling_rate, \
						S_IWUSR | S_IRUGO, \
						lis2mdl_get_polling_rate, \
						lis2mdl_set_polling_rate);
#define LIS2MDL_CONCAT(a, b)		(a b)

#define to_dev(obj) container_of(obj, struct device, kobj)

#define LIS2MDL_DEV_NAME			"lis2mdl"
#define IIS2MDC_DEV_NAME			"iis2mdc"
#define LIS2MDL_I2C_ADDR			0x1e

struct lis2mdl_platform_data {
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

struct lis2mdl_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LIS2MDL_RX_MAX_LENGTH];
	u8 tx_buf[LIS2MDL_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_common_data;

struct lis2mdl_transfer_function {
	int (*write) (struct st_common_data *cdata, u8 addr, int len, u8 *data);
	int (*read) (struct st_common_data *cdata, u8 addr, int len, u8 *data);
};

typedef struct {
	u8 power_mode;
	u8 enabled_sensor;
	u32 common_odr;
	s64 timestamp;
	struct work_struct irq_work;
	struct st_common_data *cdata; /* link back to common data */
} priv_data_t;

struct lis2mdl_data {
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
	int (*enable)(struct lis2mdl_data *sdata);
	int (*disable)(struct lis2mdl_data *sdata);
	int (*write_odr)(struct lis2mdl_data *sdata);
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
	struct lis2mdl_data *sensors;
	const struct lis2mdl_transfer_function *tf;
	struct lis2mdl_transfer_buffer tb;
};

static inline s64 lis2mdl_get_time_ns(void) {
	return ktime_to_ns(ktime_get_boottime());
}

int lis2mdl_enable(struct st_common_data *cdata);
int lis2mdl_disable(struct st_common_data *cdata);
int lis2mdl_probe(struct st_common_data *cdata);
int lis2mdl_remove(struct st_common_data *cdata);

#endif /* DRIVERS_INPUT_MISC_LIS2MDL_CORE_H_ */
