/*
 * STMicroelectronics lsm6ds0 spi driver
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

#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include "lsm6ds0.h"

#define SENSORS_SPI_READ 0x80

static int lsm6ds0_spi_read(struct lsm6ds0_status *cdata, u8 reg_addr, int len,
			    u8 *data)
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

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(to_spi_device(cdata->dev), &msg);
	if (!err)
		memcpy(data, cdata->tb.rx_buf, len);
		
	mutex_unlock(&cdata->tb.buf_lock);

	return err;
}

static int lsm6ds0_spi_write(struct lsm6ds0_status *cdata, u8 reg_addr, int len,
			     u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LSM6DS0_RX_MAX_LENGTH)
		return -ENOMEM;

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(to_spi_device(cdata->dev), &msg);
	mutex_unlock(&cdata->tb.buf_lock);

	return err;
}

static const struct lsm6ds0_transfer_function lsm6ds0_tf_spi = {
	.write = lsm6ds0_spi_write,
	.read = lsm6ds0_spi_read,
};

static int lsm6ds0_spi_probe(struct spi_device *spi)
{
	int err;
	struct lsm6ds0_status *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->irq = spi->irq;
	cdata->tf = &lsm6ds0_tf_spi;
	cdata->bustype = BUS_SPI;
	spi_set_drvdata(spi, cdata);

	err = lsm6ds0_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lsm6ds0_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lsm6ds0_status *cdata = spi_get_drvdata(spi);

	lsm6ds0_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", LSM6DS0_ACC_GYR_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm6ds0_suspend(struct device *dev)
{
	struct lsm6ds0_status *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6ds0_common_suspend(cdata);
}

static int lsm6ds0_resume(struct device *dev)
{
	struct lsm6ds0_status *cdata = spi_get_drvdata(to_spi_device(dev));

	return lsm6ds0_common_resume(cdata);
}

static const struct dev_pm_ops lsm6ds0_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6ds0_suspend, lsm6ds0_resume)
};

#define LSM6DS0_PM_OPS		(&lsm6ds0_pm_ops)
#else /* CONFIG_PM */
#define LSM6DS0_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id lsm6ds0_ids[] = {
	{ LSM6DS0_ACC_GYR_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lsm6ds0_ids);

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds0_id_table[] = {
	{ .compatible = "st,lsm6ds0", },
	{ },
};
MODULE_DEVICE_TABLE(of, lsm6ds0_id_table);
#endif

static struct spi_driver lsm6ds0_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LSM6DS0_ACC_GYR_DEV_NAME,
		.pm = LSM6DS0_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lsm6ds0_id_table),
#endif
	},
	.probe    = lsm6ds0_spi_probe,
	.remove   = lsm6ds0_spi_remove,
	.id_table = lsm6ds0_ids,
};

module_spi_driver(lsm6ds0_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lsm6ds0 spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
