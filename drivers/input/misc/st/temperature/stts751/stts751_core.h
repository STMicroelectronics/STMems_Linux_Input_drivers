/*
 * STMicroelectronics stts751_core driver
 *
 * Copyright 2020 STMicroelectronics Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _STTS751_H_
#define _STTS751_H_

#include <linux/device.h>
#include <linux/module.h>

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define MS_TO_NS(x)			((x) * 1000000L)

#define STTS751_DEFAULT_POLL_PERIOD_MS	10

#define REG_VAL_HIGH_ADDR		0x00
#define REG_STATUS_ADDR			0x01
#define REG_VAL_LOW_ADDR		0x02
#define REG_CONF_ADDR			0x03
#define REG_CONV_RATE_ADDR		0x04

#define REG_PROD_ID_ADDR		0xfd
#define REG_MAN_ID_ADDR			0xfe
#define REG_REV_NUM_ADDR		0xff

#define PROD_ID0_VALUE			0x00
#define PROD_ID1_VALUE			0x01

#define MASK_ENABLE			0x80
#define ENABLE_SENSOR			0x80
#define DISABLE_SENSOR			0x00

struct stts751_dev {
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct device *dev;
	struct mutex lock;
	const char *name;

	u8 poll_interval;
	bool enabled;
	u8 odr, res;
};

#endif /* _STTS751_H_ */
