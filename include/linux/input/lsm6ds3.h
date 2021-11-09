/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.2.1
 * Licensed under the GPL-2.
 */

#ifndef __LSM6DS3_PLATFORM_DATA_
#define __LSM6DS3_PLATFORM_DATA_
struct lsm6ds3_platform_data {
	int drdy_int_pin;
};
#endif /* __LSM6DS3_PLATFORM_DATA_ */

