/*
* Copyright (C) 2016 STMicroelectronics
*
* AMS - Motion Mems Division - Application Team
* Matteo Dameno (matteo.dameno@st.com)
* Mario Tesi (mario.tesi@st.com)
*
* Authors is willing to be considered the contact and update points for
* the driver.
*
* Read pressures and temperatures output can be converted in units of
* measurement by dividing them respectively for SENSITIVITY_P and SENSITIVITY_T.
* Temperature values must then be added by the constant float TEMPERATURE_OFFSET
* expressed as Celsius degrees.
*
* Obtained values are then expessed as
* mbar (=0.1 kPa) and Celsius degrees (T(°C) = 42.5 + (TEMP_OUT / 480)).
*
* To use autozero feature you can write 0 zero or 1 to its corresponding sysfs
* file. This lets you to write current temperature and pressure into reference
* registers or to reset them.
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
 Revision 1.0.0 2016/May/20:
	add spi support
******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "lps25h.h"

#define	PR_ABS_MAX	8388607		/* 24 bit 2'compl */
#define	PR_ABS_MIN	-8388608

#ifdef SHRT_MAX
#define	TEMP_MAX	SHRT_MAX
#define TEMP_MIN	SHRT_MIN
#else
#define	TEMP_MAX	SHORT_MAX
#define TEMP_MIN	SHORT_MIN
#endif

#ifdef SHRT_MAX
#define PRESS_OFFSET_MAX	SHRT_MAX
#define PRESS_OFFSET_MIN	SHRT_MIN
#else
#define PRESS_OFFSET_MAX	SHORT_MAX
#define PRESS_OFFSET_MIN	SHORT_MIN
#endif

#define	WHOAMI_LPS25H_PRS	0xBD	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define	REF_P_XL	0x08		/*	pressure reference	*/
#define	REF_P_L		0x09		/*	pressure reference	*/
#define	REF_P_H		0x0A		/*	pressure reference	*/

#define	WHO_AM_I_REG	0x0F		/*	WhoAmI register		*/
#define	RESOL_CONF	0x10		/*	Pres Temp resolution set*/

#define	CTRL_REG1	0x20		/*	power / ODR control reg	*/
#define	CTRL_REG2	0x21		/*	boot reg		*/
#define	CTRL_REG3	0x22		/*	interrupt control reg	*/
#define	CTRL_REG4	0x23		/*	interrupt control reg	*/
#define	INT_CFG_REG	0x24		/*	interrupt config reg	*/
#define	INT_SRC_REG	0x25		/*	interrupt source reg	*/

#define	STATUS_REG	0x27		/*	status reg		*/

#define	PRESS_OUT_XL	0x28		/*	press output (3 regs)	*/
#define	TEMP_OUT_L	0x2B		/*	temper output (2 regs)	*/

#define	FIFO_CTRL	0x2E		/*				*/
#define	FIFO_STAT	0x2F		/*				*/

#define	THS_P_L		0x30		/*	pressure threshold	*/
#define	THS_P_H		0x31		/*	pressure threshold	*/

#define	RPDS_TRIM_L	0x39		/*	pressure offset		*/
#define	RPDS_TRIM_H	0x3A		/*	pressure offset		*/

/*	REGISTERS ALIASES	*/
#define	P_REF_INDATA_REG	REF_P_XL
#define	P_OFFSET_INDATA_REG	RPDS_TRIM_L
#define	P_THS_INDATA_REG	THS_P_L
#define	P_OUTDATA_REG		PRESS_OUT_XL
#define	T_OUTDATA_REG		TEMP_OUT_L
#define	OUTDATA_REG		PRESS_OUT_XL

/* */
#define	ENABLE_MASK		0x80	/*  ctrl_reg1 */
#define	ODR_MASK		0x70	/*  ctrl_reg1 */
#define	DIFF_MASK		0x08	/*  ctrl_reg1 */
#define	BDU_MASK		0x04	/*  ctrl_reg1 */
#define	RESET_AZ		0x02	/*  ctrl_reg1 */

#define	AUTOZ_MASK		0x02	/*  ctrl_reg2 */

#define	PM_NORMAL		0x80	/* Power Normal Mode*/
#define	PM_OFF			0x00	/* Power Down */

#define	AUTOZ_ON		0x02	/* Enab AutoZero Function */
#define	AUTOZ_OFF		0x00	/* Disab Difference Function */

#define	BDU_ON			0x04	/* En BDU Block Data Upd */


#define	RES_AVGTEMP_064		0x0C
#define	RES_AVGTEMP_032		0x08

#define	RES_AVGPRES_512		0x03
#define	RES_AVGPRES_128		0x02

#define	RES_MAX	(RES_AVGTEMP_064 | RES_AVGPRES_512)	/* Max Resol. */

#define	FUZZ			0
#define	FLAT			0

#define	I2C_AUTO_INCREMENT	0x80
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5

/* RESUME STATE INDICES */
#define	RES_REF_P_XL		0
#define	RES_REF_P_L		1
#define	RES_REF_P_H		2
#define	RES_RESOL_CONF		5
#define	RES_CTRL_REG1		6
#define	RES_CTRL_REG2		7
#define	RES_CTRL_REG3		8
#define	RES_CTRL_REG4		9
#define	RES_INT_CFG_REG		10
#define	RES_FIFO_CTRL		11
#define	RES_THS_P_L		12
#define	RES_THS_P_H		13
#define	RES_RPSD_TRIM_L		14
#define	RES_RPSD_TRIM_H		15

/* end RESUME STATE INDICES */

/* Pressure Sensor Operating Mode */
#define	AUTOZ_ENABLE	1
#define	AUTOZ_DISABLE	0

/* Barometer and Termometer output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot both		*/
#define	ODR_1_1		0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	ODR_7_7		0x20	/*  7  Hz baro,  7  Hz term ODR	*/
#define	ODR_12_12	0x30	/* 12.5Hz baro, 12.5Hz term ODR	*/
#define	ODR_25_25	0x40	/* 25  Hz baro, 25  Hz term ODR	*/

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lps25h_prs_odr_table[] = {
	{   40, ODR_25_25 },
	{   80, ODR_12_12 },
	{  143, ODR_7_7   },
	{ 1000, ODR_1_1   },
};

static struct lps25h_prs_platform_data default_lps25h_prs_pdata = {
	.poll_interval = 1000,
	.min_interval = LPS25H_PRS_MIN_POLL_PERIOD_MS,
};

struct outputdata {
	s32 press;
	s16 temperature;
};

static int lps25h_prs_register_write(struct lps25h_prs_data *stat, u8 *buf,
				     u8 reg_address, u8 new_value)
{
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	return stat->tf->write(stat, reg_address, 1, &new_value);
}

static int lps25h_prs_register_read(struct lps25h_prs_data *stat, u8 *buf,
				    u8 reg_address)
{
	return stat->tf->read(stat, reg_address, 1, buf);
}

static int lps25h_prs_register_update(struct lps25h_prs_data *stat, u8 *buf,
				      u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err;
	u8 init_val;
	u8 updated_val;
	err = lps25h_prs_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lps25h_prs_register_write(stat, buf, reg_address, updated_val);
	}
	return err;
}

static int lps25h_prs_hw_init(struct lps25h_prs_data *stat)
{
	int err;
	u8 buf[6];

	dev_dbg(stat->dev, "%s: hw init start\n", LPS25H_PRS_DEV_NAME);

	buf[0] = stat->resume_state[RES_REF_P_XL];
	buf[1] = stat->resume_state[RES_REF_P_L];
	buf[2] = stat->resume_state[RES_REF_P_H];
	err = stat->tf->write(stat, P_REF_INDATA_REG, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_RESOL_CONF];
	err = stat->tf->write(stat, RESOL_CONF, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_THS_P_L];
	buf[1] = stat->resume_state[RES_THS_P_H];
	err = stat->tf->write(stat, P_THS_INDATA_REG, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL_REG2];
	buf[1] = stat->resume_state[RES_CTRL_REG3];
	buf[2] = stat->resume_state[RES_CTRL_REG4];
	err = stat->tf->write(stat, CTRL_REG2, 3, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_CFG_REG];
	err = stat->tf->write(stat, INT_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL_REG1];
	err = stat->tf->write(stat, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	dev_dbg(stat->dev, "%s: hw init done\n", LPS25H_PRS_DEV_NAME);
	return 0;

err_resume_state:
	stat->hw_initialized = 0;
	dev_err(stat->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lps25h_prs_device_power_off(struct lps25h_prs_data *stat)
{
	int err;
	u8 buf = PM_OFF;

	err = stat->tf->write(stat, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off)
		stat->pdata->power_off();

	stat->hw_initialized = 0;
}

static int lps25h_prs_device_power_on(struct lps25h_prs_data *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(stat->dev, "power_on failed: %d\n", err);
			return err;
		}
	}

	if (!stat->hw_initialized) {
		err = lps25h_prs_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lps25h_prs_device_power_off(stat);
			return err;
		}
	}

	return 0;
}

int lps25h_prs_update_odr(struct lps25h_prs_data *stat, int poll_period_ms)
{
	int err = -1;
	int i;
	u8 init_val, updated_val;
	u8 curr_val, new_val;
	u8 mask = ODR_MASK;
	u8 resol = RES_MAX;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (longest period) backward (shortest
	 * period), to support the poll_interval requested by the system.
	 * It must be the longest period shorter then the set poll period.*/
	for (i = ARRAY_SIZE(lps25h_prs_odr_table) - 1; i >= 0; i--) {
		if ((lps25h_prs_odr_table[i].cutoff_ms <= poll_period_ms) ||
		    (i == 0))
			break;
	}

	new_val = lps25h_prs_odr_table[i].mask;

	/* before to change the ODR it is mandatory to power down the device */
	err = stat->tf->read(stat, CTRL_REG1, 1, &init_val);
	if (err < 0)
		goto error;

	/* work on all but ENABLE bits power down */
	stat->resume_state[RES_CTRL_REG1] = init_val ;
	curr_val = ((ENABLE_MASK & PM_OFF) | ((~ENABLE_MASK) & init_val));
	err = stat->tf->write(stat, CTRL_REG1, 1, &curr_val);
	if (err < 0)
		goto error;

	/* set new ODR*/
	updated_val = ((mask & new_val) | ((~mask) & curr_val));
	err = stat->tf->write(stat, CTRL_REG1, 1, &updated_val);
	if (err < 0)
		goto error;

	/* power up */
	curr_val = ((ENABLE_MASK & PM_NORMAL) |
		    ((~ENABLE_MASK) & updated_val));
	err = stat->tf->write(stat, CTRL_REG1, 1, &curr_val);
	if (err < 0)
		goto error;

	err = stat->tf->write(stat, RESOL_CONF, 1, &resol);
	if (err < 0)
		goto error;

	stat->resume_state[RES_CTRL_REG1] = curr_val;
	stat->resume_state[RES_RESOL_CONF] = resol;

	return err;

error:
	dev_err(stat->dev, "update odr failed: %d\n", err);

	return err;
}

static int lps25h_prs_set_press_reference(struct lps25h_prs_data *stat,
					  s32 new_reference)
{
	int err;
	u8 bit_valuesXL, bit_valuesL, bit_valuesH;
	u8 buf[4];

	bit_valuesXL = (u8) (new_reference & 0x0000FF);
	bit_valuesL = (u8)((new_reference & 0x00FF00) >> 8);
	bit_valuesH = (u8)((new_reference & 0xFF0000) >> 16);

	buf[0] = bit_valuesXL;
	buf[1] = bit_valuesL;
	buf[2] = bit_valuesH;

	err = stat->tf->write(stat, P_REF_INDATA_REG, 3, buf);
	if (err < 0)
		return err;

	stat->resume_state[RES_REF_P_XL] = bit_valuesXL;
	stat->resume_state[RES_REF_P_L] = bit_valuesL;
	stat->resume_state[RES_REF_P_H] = bit_valuesH;

	return err;
}

static int lps25h_prs_get_press_reference(struct lps25h_prs_data *stat,
					  s32 *buf32)
{
	int err;
	u8 bit_valuesXL, bit_valuesL, bit_valuesH;
	u8 buf[3];
	u16 temp = 0;

	err = stat->tf->read(stat, P_REF_INDATA_REG, 3, buf);
	if (err < 0)
		return err;

	bit_valuesXL = buf[0];
	bit_valuesL = buf[1];
	bit_valuesH = buf[2];

	temp = ((bit_valuesH) << 8) | bit_valuesL;
	*buf32 = (s32)((((s16) temp) << 8) | bit_valuesXL);

	dev_dbg(stat->dev, "pressure_reference val: %+d", *buf32);

	return err;
}

static int lps25h_prs_set_press_offset(struct lps25h_prs_data *stat,
				       s16 new_offset)
{
	int err;
	u8 bit_valuesL, bit_valuesH;
	u8 buf[3];

	bit_valuesL = (u8)(new_offset & 0x00FF);
	bit_valuesH = (u8)((new_offset & 0xFF00) >> 8);

	buf[0] = bit_valuesL;
	buf[1] = bit_valuesH;
	err = stat->tf->write(stat, P_OFFSET_INDATA_REG, 2, buf);

	if (err < 0)
		return err;

	stat->resume_state[RES_RPSD_TRIM_L] = bit_valuesL;
	stat->resume_state[RES_RPSD_TRIM_L] = bit_valuesH;

	return err;
}

static int lps25h_prs_get_press_offset(struct lps25h_prs_data *stat,
				       s16 *buf16)
{
	int err;
	u8 bit_valuesL, bit_valuesH;
	u8 buf[2] = {0};
	u16 offset_pres = 0;

	err = stat->tf->read(stat, P_OFFSET_INDATA_REG, 2, buf);
	if (err < 0)
		return err;

	bit_valuesL = buf[0];
	bit_valuesH = buf[1];

	offset_pres = (((u16)bit_valuesH) << 8);
	*buf16 = (s16)(offset_pres | ((u16)bit_valuesL));

	return err;
}

static int lps25h_prs_autozero_manage(struct lps25h_prs_data *stat,
				      u8 control)
{
	int err;
	u8 buf[5];
	u8 bit_values = AUTOZ_OFF;
	u8 init_val;

	if (control >= AUTOZ_ENABLE) {
		bit_values = AUTOZ_ON;
		buf[0] = CTRL_REG2;
		err = stat->tf->read(stat, CTRL_REG2, 1, &init_val);
		if (err < 0)
			return err;

		stat->resume_state[RES_CTRL_REG2] = init_val;
		err = lps25h_prs_register_update(stat, buf, CTRL_REG2,
						 AUTOZ_MASK, bit_values);
		if (err < 0)
			return err;
	} else {
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		buf[4] = 0;
		err = stat->tf->write(stat, P_REF_INDATA_REG, 3, buf);
		if (err < 0)
			return err;

		stat->resume_state[RES_REF_P_XL] = 0;
		stat->resume_state[RES_REF_P_L] = 0;
		stat->resume_state[RES_REF_P_H] = 0;
	}

	return 0;
}

static int lps25h_prs_get_presstemp_data(struct lps25h_prs_data *stat,
					 struct outputdata *out)
{
	int err;
	u8 prs_data[5];
	s32 pressure;
	s16 temperature;

	err = stat->tf->read(stat, OUTDATA_REG, 5, prs_data);
	if (err < 0)
		return err;

	dev_dbg(stat->dev, "temp out tH = 0x%02x, tL = 0x%02x."
		"press_out: pH = 0x%02x, pL = 0x%02x, pXL= 0x%02x\n",
		prs_data[4], prs_data[3], prs_data[2], prs_data[1],
		prs_data[0]);

	pressure = (s32)((((s8)prs_data[2]) << 16) |
			 (prs_data[1] << 8) | (prs_data[0]));
	temperature = (s16)((((s8) prs_data[4]) << 8) | prs_data[3]);

	dev_dbg(stat->dev, "temp out pressure = %d, temperature = %d\n",
		pressure, temperature);

	out->press = pressure;
	out->temperature = temperature;

	return err;
}

static void lps25h_prs_report_values(struct lps25h_prs_data *stat,
				     struct outputdata *out)
{
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_X,
		    out->press);
	input_event(stat->input_dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y,
		    out->temperature);
	input_sync(stat->input_dev);
}

static int lps25h_prs_enable(struct lps25h_prs_data *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lps25h_prs_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		schedule_delayed_work(&stat->input_work,
			msecs_to_jiffies(stat->pdata->poll_interval));
	}

	return 0;
}

static int lps25h_prs_disable(struct lps25h_prs_data *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_delayed_work_sync(&stat->input_work);
		lps25h_prs_device_power_off(stat);
	}

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lps25h_prs_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lps25h_prs_enable(stat);
	else
		lps25h_prs_disable(stat);

	return size;
}

static ssize_t attr_get_press_ref(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int err;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	s32 val = 0;

	mutex_lock(&stat->lock);
	err = lps25h_prs_get_press_reference(stat, &val);
	mutex_unlock(&stat->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_press_ref(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int err = -1;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	long val = 0;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	if (val < PR_ABS_MIN || val > PR_ABS_MAX) {
		dev_err(stat->dev, " Value discarded: pressure_reference %+li"
			" is outside the valid interval: %+i , %+i.\n",
			val, (PR_ABS_MIN), (PR_ABS_MAX));
		return -EINVAL;
	}

	mutex_lock(&stat->lock);
	err = lps25h_prs_set_press_reference(stat, val);
	mutex_unlock(&stat->lock);
	if (err < 0)
		return err;

	return size;
}

static ssize_t attr_get_press_offset(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int err;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	s16 val = 0;

	mutex_lock(&stat->lock);
	err = lps25h_prs_get_press_offset(stat, &val);
	mutex_unlock(&stat->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_press_offset(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err = -1;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	long val = 0;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	if (val < PRESS_OFFSET_MIN || val > PRESS_OFFSET_MAX) {
		dev_err(stat->dev, " Value discarded: pressure_offset %+li"
			" is outside the valid interval: %+i , %+i.\n",
			val, (PRESS_OFFSET_MIN), (PRESS_OFFSET_MAX));
		return -EINVAL;
	}

	mutex_lock(&stat->lock);
	err = lps25h_prs_set_press_offset(stat, val);
	mutex_unlock(&stat->lock);
	if (err < 0)
		return err;

	return size;
}

static ssize_t attr_set_autozero(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int err;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	err = lps25h_prs_autozero_manage(stat, (u8) val);
	mutex_unlock(&stat->lock);
	if (err < 0)
		return err;

	return size;
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int rc;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	u8 reg, data;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	reg = stat->reg_addr;
	mutex_unlock(&stat->lock);
	data = val;
	rc = stat->tf->write(stat, &reg, 1), &data;

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	int rc;
	u8 reg, data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = stat->tf->read(stat, &reg, 1, &data);
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct lps25h_prs_data *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);

	return size;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(poll_period_ms, 0664, attr_get_polling_rate,
	       attr_set_polling_rate),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(pressure_reference_level, 0664, attr_get_press_ref,
	       attr_set_press_ref),
	__ATTR(pressure_offset, 0660, attr_get_press_offset,
	       attr_set_press_offset),
	__ATTR(enable_autozero, 0220, NULL, attr_set_autozero),
#ifdef DEBUG
	__ATTR(reg_value, 0664, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0220, NULL, attr_addr_set),
#endif
};

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int create_sysfs_interfaces(struct device *dev)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(dev, attributes + i);
		if (ret < 0)
			goto error;
	}
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);

	return ret;
}

static void lps25h_prs_input_work_func(struct work_struct *work)
{
	struct lps25h_prs_data *stat = container_of(
			(struct delayed_work *)work,
			struct lps25h_prs_data,
			input_work);

	static struct outputdata output;
	int err;

	mutex_lock(&stat->lock);
	err = lps25h_prs_get_presstemp_data(stat, &output);
	if (err < 0)
		dev_err(stat->dev, "get_pressure_data failed\n");
	else
		lps25h_prs_report_values(stat, &output);

	schedule_delayed_work(&stat->input_work,
			      msecs_to_jiffies(stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);
}

int lps25h_prs_input_open(struct input_dev *input)
{
	struct lps25h_prs_data *stat = input_get_drvdata(input);

	return lps25h_prs_enable(stat);
}

void lps25h_prs_input_close(struct input_dev *dev)
{
	lps25h_prs_disable(input_get_drvdata(dev));
}

static int lps25h_prs_validate_pdata(struct lps25h_prs_data *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval = max((unsigned int)LPS25H_PRS_MIN_POLL_PERIOD_MS,
					stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
					 stat->pdata->min_interval);

	/* Checks polling interval relative to minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(stat->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lps25h_prs_input_init(struct lps25h_prs_data *stat)
{
	int err;

	INIT_DELAYED_WORK(&stat->input_work, lps25h_prs_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		dev_err(stat->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	stat->input_dev->open = lps25h_prs_input_open;
	stat->input_dev->close = lps25h_prs_input_close;
	stat->input_dev->name = stat->name;
	stat->input_dev->id.bustype = stat->bustype;
	stat->input_dev->dev.parent = stat->dev;
	input_set_drvdata(stat->input_dev, stat);

	__set_bit(INPUT_EVENT_TYPE, stat->input_dev->evbit);
	__set_bit(INPUT_EVENT_X, stat->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, stat->input_dev->mscbit);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(stat->dev,
			"unable to register input polled device %s\n",
			stat->input_dev->name);
		input_free_device(stat->input_dev);
		return err;
	}

	return 0;
}

static void lps25h_prs_input_cleanup(struct lps25h_prs_data *stat)
{
	input_unregister_device(stat->input_dev);
}

int lps25h_common_probe(struct lps25h_prs_data *stat)
{
	int err = -1;
	u8 tempvalue;

	dev_info(stat->dev, "probe start.\n");

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	/* read chip WHO_AM_I_REG */
	err = lps25h_prs_register_read(stat, &tempvalue, WHO_AM_I_REG);
	if (tempvalue == WHOAMI_LPS25H_PRS) {
		dev_info(stat->dev, "device recognized\n");
	} else {
			/* TODO: discriminate between errors */
			err = -ENODEV;
			dev_err(stat->dev, "Not replying or not recognized\n");
			goto err_mutexunlock;
	}

	stat->pdata = kzalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(stat->dev,
			"failed to allocate memory for pdata: %d\n",
			err);
		goto err_mutexunlock;
	}

	/* select default or custom platform_data */
	if (stat->dev->platform_data == NULL) {
		memcpy(stat->pdata, &default_lps25h_prs_pdata,
		       sizeof(*stat->pdata));
		dev_info(stat->dev, "using default platform_data\n");
	} else {
		memcpy(stat->pdata, stat->dev->platform_data,
		       sizeof(*stat->pdata));
	}

	err = lps25h_prs_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data\n");
		goto err_exit_kfree_pdata;
	}

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(stat->dev, "init failed: %d\n", err);
			goto err_exit_pointer;
		}
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));
	/* init registers which need values different from zero */
	stat->resume_state[RES_CTRL_REG1] = ((ENABLE_MASK & PM_NORMAL) |
		(ODR_MASK & ODR_1_1) | (BDU_MASK & BDU_ON));

	stat->resume_state[RES_RESOL_CONF] = RES_MAX;
	err = lps25h_prs_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto err_exit_pointer;
	}

	atomic_set(&stat->enabled, 1);

	err = lps25h_prs_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = lps25h_prs_input_init(stat);
	if (err < 0) {
		dev_err(stat->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(stat->dev);
	if (err < 0) {
		dev_err(stat->dev,
			"device LPS25H_PRS_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lps25h_prs_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	mutex_unlock(&stat->lock);

	dev_info(stat->dev, "%s: probed\n", LPS25H_PRS_DEV_NAME);

	return 0;

err_input_cleanup:
	lps25h_prs_input_cleanup(stat);
err_power_off:
	lps25h_prs_device_power_off(stat);
err_exit_pointer:
	if (stat->pdata->exit)
		stat->pdata->exit();
err_exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);

	pr_err("%s: Driver Init failed\n", LPS25H_PRS_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lps25h_common_probe);

int lps25h_common_remove(struct lps25h_prs_data *stat)
{
	lps25h_prs_input_cleanup(stat);
	lps25h_prs_device_power_off(stat);
	remove_sysfs_interfaces(stat->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}
EXPORT_SYMBOL(lps25h_common_remove);

#ifdef CONFIG_PM
int lps25h_common_resume(struct lps25h_prs_data *stat)
{
	if (stat->on_before_suspend)
		return lps25h_prs_enable(stat);

	return 0;
}
EXPORT_SYMBOL(lps25h_common_resume);

int lps25h_common_suspend(struct lps25h_prs_data *stat)
{
	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lps25h_prs_disable(stat);
}
EXPORT_SYMBOL(lps25h_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("STMicrolelectronics lps25h pressure sensor driver");
MODULE_AUTHOR("Matteo Dameno, STMicroelectronics");
MODULE_LICENSE("GPL v2");
