/*
 * STMicroelectronics lsm6dsm spi driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * Mario Tesi <mario.tesi@st.com>
 * v 1.2.2
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "lsm6dsm_core.h"

#define SENSORS_SPI_READ 0x80

static int lsm6dsm_spi_read(struct lsm6dsm_data *cdata, u8 reg_addr, int len,
			    u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;

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

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

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
		mutex_unlock(&cdata->bank_registers_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int lsm6dsm_spi_write(struct lsm6dsm_data *cdata, u8 reg_addr, int len,
			     u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM6DSM_RX_MAX_LENGTH)
		return -ENOMEM;

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static const struct lsm6dsm_transfer_function lsm6dsm_tf_spi = {
	.write = lsm6dsm_spi_write,
	.read = lsm6dsm_spi_read,
};

static int lsm6dsm_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm6dsm_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &lsm6dsm_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = lsm6dsm_common_probe(cdata, spi->irq, BUS_SPI);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lsm6dsm_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lsm6dsm_data *cdata = spi_get_drvdata(spi);

	lsm6dsm_common_remove(cdata, spi->irq);
	dev_info(cdata->dev, "%s: removed\n", LSM6DSM_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm6dsm_suspend(struct device *dev)
{
	struct lsm6dsm_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6dsm_common_suspend(cdata);
}

static int lsm6dsm_resume(struct device *dev)
{
	struct lsm6dsm_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6dsm_common_resume(cdata);
}

static const struct dev_pm_ops lsm6dsm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6dsm_suspend, lsm6dsm_resume)
};

#define LSM6DSM_PM_OPS		(&lsm6dsm_pm_ops)
#else /* CONFIG_PM */
#define LSM6DSM_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id lsm6dsm_ids[] = {
	{ LSM6DSM_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm6dsm_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6dsm_id_table[] = {
	{ .compatible = "st,lsm6dsm", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6dsm_id_table);
#endif

static struct spi_driver lsm6dsm_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DSM_DEV_NAME,
		.pm = LSM6DSM_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lsm6dsm_id_table,
#endif
	},
	.probe    = lsm6dsm_spi_probe,
	.remove   = lsm6dsm_spi_remove,
	.id_table = lsm6dsm_ids,
};

module_spi_driver(lsm6dsm_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6dsm spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
