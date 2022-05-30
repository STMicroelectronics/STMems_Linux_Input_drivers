/*
 * STMicroelectronics stts22h_core driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Authors: AMG MSD DIVISION
 *        : Mario Tesi (mario.tesi@st.com)
 *
 * Licensed under the GPL-2.
 */

#ifndef _STTS22H_H_
#define _STTS22H_H_

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/module.h>

#define STTS22H_DEVICE_NAME		"stts22h"

#define STTS22H_WHOAMI_ADDR		0x01
#define STTS22H_WHOAMI_VAL		0xA0

#define STTS22H_CTRL_ADDR		0x04
#define STTS22H_LOW_ODR_START_MASK	BIT(7)
#define STTS22H_BDU_MASK		BIT(6)
#define STTS22H_AVG_MASK		GENMASK(5, 4)
#define STTS22H_IF_ADD_INC_MASK		BIT(3)
#define STTS22H_FREERUN_MASK		BIT(2)
#define STTS22H_ONE_SHOT_MASK		BIT(0)

#define STTS22H_STATUS_ADDR		0x05
#define STTS22H_BUSY_MASK		BIT(0)

#define STTS22H_TEMP_L_OUT_ADDR		0x06

#define STTS22H_SOFTWARE_RESET_ADDR	0x0C
#define STTS22H_LOW_ODR_ENABLE_MASK	BIT(6)
#define STTS22H_SW_RESET_MASK		BIT(1)

/* input events associated to sensor events */
#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define ODR_TO_MS(x)			(1000 / (x))
#define MS_TO_ODR(x)			(1000 / (x))
#define MS_TO_NS(x)			((x) * 1000000L)

/**
 * struct stts22h_sensor - Sensor data instance
 *
 * @input_work: Sensor delayed workqueue.
 * @input_dev: Input device.
 * @dev: Linux Device.
 * @lock: Mutex to lock device registers access.
 * @name: Sensor name.
 * @poll_index: Poll rate index.
 * @enabled: Enable sensor flag.
 * @low_odr: Flag for low odr configuration.
 * @poll_ms: Poll rate in ms.
 */
struct stts22h_sensor {
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct device *dev;
	struct mutex lock;
	const char *name;
	int poll_index;
	bool enabled;
	bool low_odr;
	u32 poll_ms;
};

#endif /* _STTS22H_H_ */
