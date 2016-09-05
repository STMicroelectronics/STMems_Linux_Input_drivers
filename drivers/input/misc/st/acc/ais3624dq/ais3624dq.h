/* Copyright (C) 2014 STMicroelectronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * File Name		: ais3624dq.h
 * Authors		: VMA - Volume Mems & Analog Division
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V 1.0.1
 * Date			: 2012/10/07
 * Description		: AIS3624DQ 3D accelerometer sensor LDD
 *
 */

#ifndef __AIS3624DQ_H__
#define __AIS3624DQ_H__

#define	AIS3624DQ_ACC_DEV_NAME	"ais3624dq_acc"

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale bit */
#define AIS3624DQ_ACC_FS_MASK		0x30
#define AIS3624DQ_ACC_G_6G 		0x00
#define AIS3624DQ_ACC_G_12G 		0x10
#define AIS3624DQ_ACC_G_24G 		0x30

/* Accelerometer Sensor Operating Mode */
#define AIS3624DQ_ACC_ENABLE		0x01
#define AIS3624DQ_ACC_DISABLE		0x00
#define AIS3624DQ_ACC_PM_NORMAL		0x20
#define AIS3624DQ_ACC_PM_OFF		AIS3624DQ_ACC_DISABLE

#ifdef __KERNEL__

#if defined(CONFIG_INPUT_AIS3624DQ_SPI) || \
    defined(CONFIG_INPUT_AIS3624DQ_SPI_MODULE)
#define AIS3624DQ_RX_MAX_LENGTH	500
#define AIS3624DQ_TX_MAX_LENGTH	500

struct ais3624dq_transfer_buffer {
	u8 rx_buf[AIS3624DQ_RX_MAX_LENGTH];
	u8 tx_buf[AIS3624DQ_TX_MAX_LENGTH] ____cacheline_aligned;
};
#endif

struct ais3624dq_transfer_function {
	int (*write)(struct device *dev, u8 reg_addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

struct ais3624dq_acc_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

#define	RESUME_ENTRIES		12

struct ais3624dq_acc_data {
	const char *name;
	struct device *dev;
	u16 bus_type;
	struct ais3624dq_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	int selftest_enabled;

	atomic_t enabled;
	u8 sensitivity;
	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef AIS3624DQ_DEBUG
	u8 reg_addr;
#endif
#if defined(CONFIG_INPUT_AIS3624DQ_SPI) || \
    defined(CONFIG_INPUT_AIS3624DQ_SPI_MODULE)
	struct ais3624dq_transfer_buffer tb;
#endif
	struct ais3624dq_transfer_function *tf;
};

int ais3624dq_acc_probe(struct ais3624dq_acc_data *acc);
int ais3624dq_acc_remove(struct ais3624dq_acc_data *acc);
#ifdef CONFIG_PM
int ais3624dq_acc_enable(struct ais3624dq_acc_data *acc);
int ais3624dq_acc_disable(struct ais3624dq_acc_data *acc);
#endif

#endif /* __KERNEL__ */

#endif  /* __AIS3624DQ_H__ */
