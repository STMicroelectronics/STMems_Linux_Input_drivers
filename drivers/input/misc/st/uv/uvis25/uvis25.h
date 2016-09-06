/* 
 Copyright (C) 2016 STMicroelectronics
 
 AMS - Motion Mems Division - Application Team

 Mario Tesi (mario.tesi@st.com)

 Authors is willing to be considered the contact and update points for
 the driver.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 02110-1301 USA
*/
/******************************************************************************
 Revision 1.0.0 2016/May/21:
	first release
******************************************************************************/

#ifndef	__UVIS25_H__
#define	__UVIS25_H__

//#define DEBUG 0

#define	UVIS25_DEV_NAME		"uvis25"
#define HAS_IF_AUTO_INCREMENT

#ifdef __KERNEL__

/* Sensor Operating Mode */
#define UVIS25_MIN_POLL_PERIOD_MS	1000
#define UVIS25_MIN_UV		0
#define UVIS25_MAX_UV		15


#define	RESUME_ENTRIES		5

#define UVIS25_RX_MAX_LENGTH	100
#define UVIS25_TX_MAX_LENGTH	100

struct uvis25_data;

struct uvis25_transfer_buffer {
	u8 rx_buf[UVIS25_RX_MAX_LENGTH];
	u8 tx_buf[UVIS25_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct uvis25_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	unsigned int ev_type;
};

/* specific bus I/O functions */
struct uvis25_transfer_function {
	int (*write)(struct uvis25_data *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct uvis25_data *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct uvis25_data {
	const char *name;
	struct uvis25_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;
	struct device *dev;
	struct input_dev *input_dev;

	int hw_initialized;
	int hw_working;
	u16 bustype;

	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];
	struct uvis25_transfer_buffer tb;
	struct uvis25_transfer_function *tf;
};

/* Input Event Type */
#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL

int uvis25_common_probe(struct uvis25_data *stat);
int uvis25_common_remove(struct uvis25_data *stat);

#ifdef CONFIG_PM
int uvis25_common_resume(struct uvis25_data *stat);
int uvis25_common_suspend(struct uvis25_data *stat);
#endif /* CONFIG_PM */
#endif /* __KERNEL__ */

#endif  /* __UVIS25_H__ */
