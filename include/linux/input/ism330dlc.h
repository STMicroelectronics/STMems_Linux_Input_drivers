/*
 * STMicroelectronics ism330dlc driver
 *
 * Copyright 2018 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */


#ifndef __ISM330DLC_H__
#define __ISM330DLC_H__

#define ISM330DLC_DEV_NAME			"ism330dlc"

struct ism330dlc_platform_data {
	u8 drdy_int_pin;
};

#endif /* __ISM330DLC_H__ */
