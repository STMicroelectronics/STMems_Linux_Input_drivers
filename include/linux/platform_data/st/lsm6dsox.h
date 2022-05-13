/*
 * STMicroelectronics lsm6dsox driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef __LSM6DSOX_PLATFORM_DATA_
#define __LSM6DSOX_PLATFORM_DATA_
struct lsm6dsox_platform_data {
	int drdy_int_pin;
};
#endif /* __LSM6DSOX_PLATFORM_DATA_ */

