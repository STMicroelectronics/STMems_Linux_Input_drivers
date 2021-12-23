/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name		: l3gd20.h
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Mario Tesi (mario.tesi@st.com)
*			: Authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.2.2 sysfs
* Date			: 2016/Jul/19
* Description		: L3GD20 digital output gyroscope sensor API
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
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS	  | DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone | First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno	  | Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno	  | SMB Bus Mng,
*		|		|		  | forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno	  | Introduces FIFO Feat.
* 1.1.5.2	| 2011/Nov/11	| Matteo Dameno	  | enable gpio_int to be
*		|		|		  | passed as parameter at
*		|		|		  | module loading time;
*		|		|		  | corrects polling
*		|		|		  | bug at end of probing;
* 1.1.5.3	| 2011/Dec/20	| Matteo Dameno	  | corrects error in
*		|		|		  | I2C SADROOT; Modifies
*		|		|		  | resume suspend func.
* 1.1.5.4	| 2012/Jan/09	| Matteo Dameno	  | moved under input/misc;
* 1.1.5.5	| 2012/Mar/30	| Matteo Dameno	  | moved watermark, use_smbus,
*		|		|		  | fifomode @ struct foo_status
*		|		|		  | sysfs range input format
*		|		|		  | changed to decimal
* 1.2		| 2012/Jul/10	| Denis Ciocca	  | input_poll_dev removal
* 1.2.1		| 2012/Jul/10	| Denis Ciocca	  | added high resolution timers
* 1.2.2		| 2016/Jul/19	| Mario Tesi	  | added timestamp for cts
*******************************************************************************/

#ifndef __L3GD20_GYR_H__
#define __L3GD20_GYR_H__

#define L3GD20_GYR_DEV_NAME		"l3gd20_gyr"

#define L3GD20_GYR_ENABLED		1
#define L3GD20_GYR_DISABLED		0


/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones havew to be set to -EINVAL
 */
#define L3GD20_GYR_DEFAULT_INT1_GPIO		(-EINVAL)
#define L3GD20_GYR_DEFAULT_INT2_GPIO		(-EINVAL)

#define L3GD20_GYR_MIN_POLL_PERIOD_MS	2

#define L3GD20_GYR_FS_250DPS		(0x00)
#define L3GD20_GYR_FS_500DPS		(0x10)
#define L3GD20_GYR_FS_2000DPS	(0x30)

struct l3gd20_gyr_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */

	/* axis mapping */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

#define	RESUME_ENTRIES		6

#if defined(CONFIG_INPUT_L3GD20_SPI) || \
    defined(CONFIG_INPUT_L3GD20_SPI_MODULE)
#define L3GD20_RX_MAX_LENGTH		500
#define L3GD20_TX_MAX_LENGTH		500

struct l3gd20_gyr_transfer_buffer {
	u8 rx_buf[L3GD20_RX_MAX_LENGTH];
	u8 tx_buf[L3GD20_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif

struct l3gd20_gyr_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

struct l3gd20_gyr_status {
	const char *name;
	struct device *dev;
	u16 bus_type;
	struct i2c_client *client;
	struct l3gd20_gyr_platform_data *pdata;

	struct mutex lock;
	struct workqueue_struct *gyr_workqueue;
	struct input_dev *input_dev;
	int64_t timestamp;

	int hw_initialized;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];

	u32 sensitivity;

	/* interrupt related */
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	bool polling_enabled;
	/* fifo related */
	u8 watermark;
	u8 fifomode;

	struct hrtimer hr_timer;
	ktime_t ktime;
	struct work_struct polling_task;

	struct l3gd20_gyr_transfer_function *tf;
#if defined(CONFIG_INPUT_L3GD20_SPI) || \
    defined(CONFIG_INPUT_L3GD20_SPI_MODULE)
	struct l3gd20_gyr_transfer_buffer tb;
#endif /* CONFIG_INPUT_L3GD20_SPI */
};

int l3gd20_gyr_probe(struct l3gd20_gyr_status *stat);
int l3gd20_gyr_remove(struct l3gd20_gyr_status *stat);
int l3gd20_gyr_suspend(struct l3gd20_gyr_status *stat);
int l3gd20_gyr_resume(struct l3gd20_gyr_status *stat);

#endif  /* __L3GD20_GYR_H__ */
