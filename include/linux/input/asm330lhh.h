/*
 * STMicroelectronics asm330lhh driver
 *
 * Copyright 2019 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com> 
 * v 1.0.0
 * Licensed under the GPL-2.
 */

#ifndef __ASM330LHH_PLATFORM_DATA_
#define __ASM330LHH_PLATFORM_DATA_
struct asm330lhh_platform_data {
        int drdy_int_pin;
};
#endif /* __ASM330LHH_PLATFORM_DATA_ */

