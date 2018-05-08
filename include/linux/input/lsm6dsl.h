/*
 * STMicroelectronics lsm6dsl driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.0.1
 * Licensed under the GPL-2.
 */

#ifndef __LSM6DSL_PLATFORM_DATA_
#define __LSM6DSL_PLATFORM_DATA_
struct lsm6dsl_platform_data {
	int drdy_int_pin;
};
#endif /* __LSM6DSL_PLATFORM_DATA_ */
