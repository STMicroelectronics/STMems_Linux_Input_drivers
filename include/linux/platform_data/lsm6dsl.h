/*
 * STMicroelectronics lsm6dsl driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 */
 
#ifndef __LSM6DSL_H__
#define __LSM6DSL_H__
 
struct lsm6dsl_platform_data {
        u8 drdy_int_pin;
};

#endif /* __LSM6DSL_H__ */
