/*
 * STMicroelectronics lsm6dsox spi driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "lsm6dsox_core.h"

#define SENSORS_SPI_READ 0x80

static int lsm6dsox_spi_read(struct lsm6dsox_data *cdata, u8 reg_addr,
			     int len, u8 *data, bool b_lock)
{
	struct spi_transfer xfers[] = {
		{
			.tx_buf = cdata->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = cdata->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};
	struct spi_message msg;
	int err;

	if (b_lock)
		mutex_lock(&cdata->reg_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->reg_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->reg_lock);

	return err;
}

static int lsm6dsox_spi_write(struct lsm6dsox_data *cdata, u8 reg_addr,
			      int len, u8 *data, bool b_lock)
{
	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};
	struct spi_message msg;
	int err;

	if (len >= LSM6DSOX_RX_MAX_LENGTH)
		return -ENOMEM;

	if (b_lock)
		mutex_lock(&cdata->reg_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->reg_lock);

	return err;
}

static const struct lsm6dsox_transfer_function lsm6dsox_tf_spi = {
	.write = lsm6dsox_spi_write,
	.read = lsm6dsox_spi_read,
};

static int lsm6dsox_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	int hw_id = id->driver_data;
	struct lsm6dsox_data *cdata;
	int err;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &lsm6dsox_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = lsm6dsox_common_probe(cdata, spi->irq, hw_id, BUS_SPI);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lsm6dsox_spi_remove(struct spi_device *spi)
{
	struct lsm6dsox_data *cdata = spi_get_drvdata(spi);

	lsm6dsox_common_remove(cdata, spi->irq);
	dev_info(cdata->dev, "%s: removed\n", LSM6DSOX_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lsm6dsox_suspend(struct device *dev)
{
	struct lsm6dsox_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6dsox_common_suspend(cdata);
}

static int lsm6dsox_resume(struct device *dev)
{
	struct lsm6dsox_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6dsox_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lsm6dsox_pm_ops, lsm6dsox_suspend,
			 lsm6dsox_resume);

#define LSM6DSOX_PM_OPS		(&lsm6dsox_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LSM6DSOX_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lsm6dsox_ids[] = {
	{ LSM6DSO_DEV_NAME, LSM6DSO_ID },
	{ LSM6DSOX_DEV_NAME, LSM6DSOX_ID },
	{ LSM6DSO32_DEV_NAME, LSM6DSO32_ID },
	{ LSM6DSO32X_DEV_NAME, LSM6DSO32X_ID },
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm6dsox_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6dsox_id_table[] = {
	{
		.compatible = "st,lsm6dso",
		.data = (void *)LSM6DSO_ID,
	},
	{
		.compatible = "st,lsm6dsox",
		.data = (void *)LSM6DSOX_ID,
	},
	{
		.compatible = "st,lsm6dso32",
		.data = (void *)LSM6DSO32_ID,
	},
	{
		.compatible = "st,lsm6dso32x",
		.data = (void *)LSM6DSO32X_ID,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6dsox_id_table);
#endif

static struct spi_driver lsm6dsox_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DSOX_DEV_NAME,
		.pm = LSM6DSOX_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6dsox_id_table,
#endif
	},
	.probe    = lsm6dsox_spi_probe,
	.remove   = lsm6dsox_spi_remove,
	.id_table = lsm6dsox_ids,
};

module_spi_driver(lsm6dsox_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6dsox spi driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
