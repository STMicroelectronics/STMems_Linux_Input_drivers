/*
 * STMicroelectronics lsm6dsox driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef _LSM6DSOX_CORE_H_
#define _LSM6DSOX_CORE_H_

#define LSM6DSO_DEV_NAME			"lsm6dso"
#define LSM6DSOX_DEV_NAME			"lsm6dsox"
#define LSM6DSO32_DEV_NAME			"lsm6dso32"
#define LSM6DSO32X_DEV_NAME			"lsm6dso32x"

#define LSM6DSOX_FUNC_CFG_ACCESS_ADDR		0x01
#define LSM6DSOX_FUNC_CFG_REG_MASK		BIT(7)

#define LSM6DSOX_WHO_AM_I			0x0f
#define LSM6DSOX_WHO_AM_I_VAL			0x6c

#define LSM6DSOX_CTRL1_XL_ADDR			0x10
#define LSM6DSOX_ODR_XL_MASK			GENMASK(7, 4)
#define LSM6DSOX_FS_XL_MASK			GENMASK(3, 2)

#define LSM6DSOX_CTRL2_G_ADDR			0x11
#define LSM6DSOX_ODR_G_MASK			GENMASK(7, 4)
#define LSM6DSOX_FS_G_MASK			GENMASK(3, 2)

#define LSM6DSOX_CTRL3_C_ADDR			0x12
#define LSM6DSOX_BDU_MASK			BIT(6)
#define LSM6DSOX_H_LACTIVE_MASK			BIT(5)
#define LSM6DSOX_IF_INC_MASK			BIT(1)
#define LSM6DSOX_SW_RESET_MASK			BIT(0)

#define LSM6DSOX_CTRL4_C_ADDR			0x13
#define LSM6DSOX_INT2_ON_INT1_MASK		BIT(5)

#define LSM6DSOX_CTRL5_C_ADDR			0x14
#define LSM6DSOX_ROUNDING_MASK			GENMASK(6, 5)

#define LSM6DSOX_CTRL10_C_ADDR			0x19
#define LSM6DSOX_TIMESTAMP_EN_MASK		BIT(5)

#define LSM6DSOX_OUTX_L_G_ADDR			0x22
#define LSM6DSOX_OUTX_L_A_ADDR			0x28

#define LSM6DSOX_EMB_FUNC_STATUS_MAINPAGE_ADDR	0x35
#define LSM6DSOX_IS_SIGMOT_MASK			BIT(5)
#define LSM6DSOX_IS_TILT_MASK			BIT(4)
#define LSM6DSOX_IS_STEP_DET_MASK		BIT(3)

#define LSM6DSOX_TAP_CFG0_ADDR			0x56
#define LSM6DSOX_LIR_MASK			BIT(0)

#define LSM6DSOX_MD1_CFG_ADDR			0x5e
#define LSM6DSOX_INT1_EMB_FUNC_MASK		BIT(1)

#define LSM6DSOX_MD2_CFG_ADDR			0x5f
#define LSM6DSOX_INT2_EMB_FUNC_MASK		BIT(1)

/* embedded function registers */
#define LSM6DSOX_EMB_FUNC_EN_A_ADDR		0x04
#define LSM6DSOX_SIGN_MOTION_EN_MASK		BIT(5)
#define LSM6DSOX_TILT_EN_MASK			BIT(4)
#define LSM6DSOX_PEDO_EN_MASK			BIT(3)

#define LSM6DSOX_EMB_FUNC_INT1			0x0a
#define LSM6DSOX_INT1_SIG_MOT_MASK		BIT(5)
#define LSM6DSOX_INT1_TILT_MASK			BIT(4)
#define LSM6DSOX_INT1_STEP_DETECTOR_MASK	BIT(3)

#define LSM6DSOX_EMB_FUNC_INT2			0x0e
#define LSM6DSOX_INT2_SIG_MOT_MASK		BIT(5)
#define LSM6DSOX_INT2_TILT_MASK			BIT(4)
#define LSM6DSOX_INT2_STEP_DETECTOR_MASK	BIT(3)

#define LSM6DSOX_PAGE_RW_ADDR			0x17
#define LSM6DSOX_EMB_FUNC_LIR_MASK		BIT(7)

#define LSM6DSOX_STEP_COUNTER_L_ADDR		0x62

#define LSM6DSOX_EMB_FUNC_SRC_ADDR		0x64
#define LSM6DSOX_PEDO_RST_STEP_MASK		BIT(7)

/* default configuration */
#define LSM6DSOX_MIN_DURATION_MS		1638
#define LSM6DSOX_ACCEL_STD			1
#define LSM6DSOX_ACCEL_STD_FROM_PD		2
#define LSM6DSOX_GYRO_STD			6
#define LSM6DSOX_GYRO_STD_FROM_PD		2

#define LSM6DSOX_OUT_XYZ_SIZE			6

#define HZ_TO_PERIOD_NSEC(hz)			(1000 * 1000 * 1000 / ((u32)(hz)))
#define MS_TO_US(x)				({ typeof(x) _x = (x); ((_x) * \
							((typeof(x)) 1000)); })
#define US_TO_NS(x)				(MS_TO_US(x))
#define MS_TO_NS(x)				(US_TO_NS(MS_TO_US(x)))
#define US_TO_MS(x)				({ typeof(x) _x = (x); ((_x) / \
							((typeof(x)) 1000)); })
#define NS_TO_US(x)				(US_TO_MS(x))
#define NS_TO_MS(x)				(US_TO_MS(NS_TO_US(x)))

#define LSM6DSOX_ODR_LIST_NUM			6
#define LSM6DSOX_FS_LIST_NUM			4

#define INPUT_EVENT_TYPE			EV_MSC
#define INPUT_EVENT_X				MSC_SERIAL
#define INPUT_EVENT_Y				MSC_PULSELED
#define INPUT_EVENT_Z				MSC_GESTURE
#define INPUT_EVENT_TIME_MSB			MSC_SCAN
#define INPUT_EVENT_TIME_LSB			MSC_MAX

#define LSM6DSOX_RX_MAX_LENGTH			500
#define LSM6DSOX_TX_MAX_LENGTH			500

struct lsm6dsox_data;

enum lsm6dsox_sensor_id {
	LSM6DSOX_ACCEL = 0,
	LSM6DSOX_GYRO,
	LSM6DSOX_SIGN_MOTION,
	LSM6DSOX_STEP_COUNTER,
	LSM6DSOX_STEP_DETECTOR,
	LSM6DSOX_TILT,
	LSM6DSOX_MAX_SENSOR,
};

enum lsm6dsox_hw_id {
	LSM6DSO_ID,
	LSM6DSOX_ID,
	LSM6DSO32_ID,
	LSM6DSO32X_ID,
	LSM6DSOX_MAX_ID,
};

struct lsm6dsox_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[LSM6DSOX_RX_MAX_LENGTH];
	u8 tx_buf[LSM6DSOX_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct lsm6dsox_transfer_function {
	int (*write)(struct lsm6dsox_data *cdata, u8 reg_addr, int len,
		     u8 *data, bool b_lock);
	int (*read)(struct lsm6dsox_data *cdata, u8 reg_addr, int len,
		    u8 *data, bool b_lock);
};

struct lsm6dsox_odr_reg {
	u32 hz;
	u8 value;
};

struct lsm6dsox_odr_table_t {
	u8 addr[2];
	u8 mask[2];
	struct lsm6dsox_odr_reg odr_avl[LSM6DSOX_ODR_LIST_NUM];
};

struct lsm6dsox_fs_reg {
	unsigned int gain;
	u8 value;
	int urv;
};

struct lsm6dsox_fs_table {
	u8 addr;
	u8 mask;
	struct lsm6dsox_fs_reg fs_avl[LSM6DSOX_FS_LIST_NUM];
};

struct lsm6dsox_settings {
	struct {
		enum lsm6dsox_hw_id hw_id;
		const char *name;
	} id[LSM6DSOX_MAX_ID];
	struct lsm6dsox_fs_table fs_table[LSM6DSOX_MAX_ID];
};

struct lsm6dsox_sensor_data {
	struct lsm6dsox_data *cdata;
	const char *name;
	int64_t timestamp;
	u8 enabled;
	u32 c_odr;
	u32 c_gain;
	u8 sindex;
	u8 sample_to_discard;
	struct input_dev *input_dev;
	unsigned int poll_interval;
	struct hrtimer hr_timer;
	struct work_struct input_work;
	ktime_t oldktime;
};

struct lsm6dsox_data {
	const char *name;
	char dev_name[16];
	bool reset_steps;
	bool sign_motion_event_ready;
	u16 steps_c;
	u8 drdy_int_pin;
	int irq;
	int64_t timestamp;
	struct work_struct input_work;
	struct device *dev;
	struct lsm6dsox_sensor_data sensors[LSM6DSOX_MAX_SENSOR];
	struct mutex reg_lock;
	const struct lsm6dsox_transfer_function *tf;
	struct lsm6dsox_transfer_buffer tb;
	const struct lsm6dsox_settings *settings;
};

int lsm6dsox_common_probe(struct lsm6dsox_data *cdata, int irq,
			  int hw_id, u16 bustype);
void lsm6dsox_common_remove(struct lsm6dsox_data *cdata, int irq);

#ifdef CONFIG_PM_SLEEP
int lsm6dsox_common_suspend(struct lsm6dsox_data *cdata);
int lsm6dsox_common_resume(struct lsm6dsox_data *cdata);
#endif /* CONFIG_PM_SLEEP */

#endif /* _LSM6DSOX_CORE_H_ */
