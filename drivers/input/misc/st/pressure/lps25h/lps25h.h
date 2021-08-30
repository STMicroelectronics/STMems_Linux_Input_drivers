/*
* drivers/misc/lps25h.h
*
* STMicroelectronics LPS25H Pressure / Temperature Sensor module driver
*
* Copyright (C) 2012 STMicroelectronics
* AMS - Motion Mems Division - Application Team
* Matteo Dameno (matteo.dameno@st.com)
* Mario Tesi (mario.tesi@st.com)
*
* Authors is willing to be considered the contact and update points for
* the driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 1.0.0 2012/Nov/21:
	first release
	moved to input/misc
 Revision 1.0.1 2016/May/21:
	Added spi support
******************************************************************************/

#ifndef	__LPS25H_H__
#define	__LPS25H_H__

//#define DEBUG 0

#define	LPS25H_PRS_DEV_NAME	"lps25h_prs"
#define HAS_IF_AUTO_INCREMENT

/* Output conversion factors */
#define	SENSITIVITY_T		480	/* =	480 digit/degrC	*/
#define	SENSITIVITY_P		4096	/* =	digit/mbar	*/
#define	SENSITIVITY_P_SHIFT	12	/* =	4096 digit/mbar	*/
#define	TEMPERATURE_OFFSET	42.5f	/* =	42.5 degrC	*/

/* input define mappings */
#define ABS_PR		ABS_PRESSURE
#define ABS_TEMP	ABS_GAS
#define ABS_DLTPR	ABS_MISC

#ifdef __KERNEL__
/* Pressure Sensor Operating Mode */
#define	LPS25H_PRS_ENABLE	0x01
#define	LPS25H_PRS_DISABLE	0x00


#define LPS25H_PRS_MIN_POLL_PERIOD_MS	1

#define	LPS25H_PRS_SAD0L	0x00
#define	LPS25H_PRS_SAD0H	0x01
#define	LPS25H_PRS_I2C_SADROOT	0x2E
#define	LPS25H_PRS_I2C_SAD_L	((LPS25H_PRS_I2C_SADROOT<<1)| \
				 LPS25H_PRS_SAD0L)
#define	LPS25H_PRS_I2C_SAD_H	((LPS25H_PRS_I2C_SADROOT<<1)| \
				 LPS25H_PRS_SAD0H)

#define	RESUME_ENTRIES		16

#define LPS25H_RX_MAX_LENGTH	500
#define LPS25H_TX_MAX_LENGTH	500

struct lps25h_prs_transfer_buffer {
	u8 rx_buf[LPS25H_RX_MAX_LENGTH];
	u8 tx_buf[LPS25H_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lps25h_prs_data;

struct lps25h_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

/* specific bus I/O functions */
struct lps25h_prs_transfer_function {
	int (*write)(struct lps25h_prs_data *stat, u8 reg_addr, int len,
		     u8 *data);
	int (*read)(struct lps25h_prs_data *stat, u8 reg_addr, int len,
		    u8 *data);
};

struct lps25h_prs_data {
	const char *name;
	struct lps25h_prs_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;
	struct device *dev;
	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	u16 bustype;

	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef DEBUG
	u8 reg_addr;
#endif
	struct lps25h_prs_transfer_buffer tb;
	struct lps25h_prs_transfer_function *tf;
};

/* Input Event Type */
#define INPUT_EVENT_TYPE	EV_MSC
#define INPUT_EVENT_X		MSC_SERIAL
#define INPUT_EVENT_Y		MSC_PULSELED

int lps25h_common_probe(struct lps25h_prs_data *stat);
int lps25h_common_remove(struct lps25h_prs_data *stat);

#ifdef CONFIG_PM
int lps25h_common_resume(struct lps25h_prs_data *stat);
int lps25h_common_suspend(struct lps25h_prs_data *stat);
#endif /* CONFIG_PM */
#endif /* __KERNEL__ */

#endif  /* __LPS25H_H__ */
