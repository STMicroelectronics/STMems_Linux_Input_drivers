/*
 * STMicroelectronics hts221_core driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef _HTS221_H_
#define _HTS221_H_

#include <linux/device.h>
#include <linux/module.h>

/* Input events used by lsm303agr driver */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#if defined(CONFIG_INPUT_HTS221_SPI) || \
    defined(CONFIG_INPUT_HTS221_SPI_MODULE)
#define HTS221_RX_MAX_LENGTH		500
#define HTS221_TX_MAX_LENGTH		500

struct hts221_transfer_buffer {
	u8 rx_buf[HTS221_RX_MAX_LENGTH];
	u8 tx_buf[HTS221_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_HTS221_SPI */

struct hts221_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

enum hts221_sensor_type {
	HTS221_SENSOR_T,
	HTS221_SENSOR_H,
	HTS221_SENSOR_MAX,
};

struct hts221_sensor {
	u8 res;
	int slope;
	int b_gen;
};

struct hts221_dev {
	const char *name;
	u16 bus_type;
	struct mutex lock;
	struct device *dev;
	struct input_dev *input_dev;
	struct delayed_work input_work;
	const struct hts221_transfer_function *tf;
#if defined(CONFIG_INPUT_HTS221_SPI) || \
    defined(CONFIG_INPUT_HTS221_SPI_MODULE)
	struct hts221_transfer_buffer tb;
#endif /* CONFIG_INPUT_HTS221_SPI */

	u8 odr;
	u8 poll_interval;
	bool heater;
        bool enabled;
	struct hts221_sensor sensors[HTS221_SENSOR_MAX];
};

static inline s64 hts221_get_time_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

int hts221_probe(struct hts221_dev *dev);
void hts221_remove(struct hts221_dev *dev);
int hts221_enable(struct hts221_dev *dev);
int hts221_disable(struct hts221_dev *dev);

#endif /* _HTS221_H_ */
