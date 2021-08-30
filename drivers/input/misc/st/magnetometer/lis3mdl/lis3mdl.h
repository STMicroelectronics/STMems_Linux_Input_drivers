/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis3mdl.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
*		: Lorenzo Bianconi (lorenzo.bianconi@st.com)
* Version	: V.1.0.2
* Date		: 2016/May/2
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

#ifndef __LIS3MDL_H__
#define __LIS3MDL_H__

#include <linux/module.h>
#include <linux/version.h>

struct lis3mdl_transfer_function {
	int (*write) (struct device *dev, u8 addr, int len, u8 *data);
	int (*read) (struct device *dev, u8 addr, int len, u8 *data);
};

#if defined(CONFIG_INPUT_LIS3MDL_SPI) || \
    defined(CONFIG_INPUT_LIS3MDL_SPI_MODULE)
#define LIS3MDL_RX_MAX_LENGTH		500
#define LIS3MDL_TX_MAX_LENGTH		500

struct lis3mdl_transfer_buffer {
	u8 rx_buf[LIS3MDL_RX_MAX_LENGTH];
	u8 tx_buf[LIS3MDL_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif /* CONFIG_INPUT_LIS3MDL_SPI */

struct lis3mdl_dev {
	const char *name;
	struct device *dev;
	struct mutex lock;
	u16 bus_type;
	struct work_struct input_work_mag;

	struct hrtimer hr_timer_mag;
	ktime_t ktime;

	struct input_dev *input_dev_mag;

	atomic_t enabled_mag;
	int64_t timestamp;

	u32 sensitivity_mag;
	u32 poll_interval;

	u8 xy_mode;
	u8 z_mode;
	u8 fs_range;

	const struct lis3mdl_transfer_function *tf;
#if defined(CONFIG_INPUT_LIS3MDL_SPI) || \
    defined(CONFIG_INPUT_LIS3MDL_SPI_MODULE)
	struct lis3mdl_transfer_buffer tb;
#endif /* CONFIG_INPUT_LIS3MDL_SPI */
};

int lis3mdl_mag_enable(struct lis3mdl_dev *dev);
int lis3mdl_mag_disable(struct lis3mdl_dev *dev);
int lis3mdl_mag_probe(struct lis3mdl_dev *dev);
int lis3mdl_mag_remove(struct lis3mdl_dev *dev);
#endif	/* __LIS3MDL_H__ */
