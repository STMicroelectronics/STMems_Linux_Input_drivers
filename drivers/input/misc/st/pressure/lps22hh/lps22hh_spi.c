/*
 * STMicroelectronics lps22hh spi driver
 *
 * Copyright 2020 STMicroelectronics Inc.
 *
 * Authors: AMG MSD DIVISION
 *        : Mario Tesi (mario.tesi@st.com)
 *
 * Version: 1.0.0
 *
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

#include "lps22hh.h"

#define SENSORS_SPI_READ 0x80

static int lps22hh_spi_read(struct lps22hh_data *cdata, u8 reg_addr, int len,
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

static int lps22hh_spi_write(struct lps22hh_data *cdata, u8 reg_addr, int len,
			     u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LPS22HH_RX_MAX_LENGTH)
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

static const struct lps22hh_transfer_function lps22hh_tf_spi = {
	.write = lps22hh_spi_write,
	.read = lps22hh_spi_read,
};

static int lps22hh_spi_probe(struct spi_device *spi)
{
	int err;
	struct lps22hh_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->irq = spi->irq;
	cdata->tf = &lps22hh_tf_spi;
	cdata->name = spi->modalias;
	cdata->bustype = BUS_SPI;
	spi_set_drvdata(spi, cdata);

	err = lps22hh_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lps22hh_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lps22hh_data *cdata = spi_get_drvdata(spi);

	lps22hh_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", LPS22HH_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lps22hh_suspend(struct device *dev)
{
	struct lps22hh_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lps22hh_common_suspend(cdata);
}

static int lps22hh_resume(struct device *dev)
{
	struct lps22hh_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lps22hh_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lps22hh_pm_ops, lps22hh_suspend, lps22hh_resume);

#define LPS22HH_PM_OPS		(&lps22hh_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LPS22HH_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lps22hh_ids[] = {
	{ LPS22HH_DEV_NAME },
	{ LPS22CH_DEV_NAME },
	{ LPS27HHW_DEV_NAME },
	{ LPS27HHTW_DEV_NAME },
	{}
};
MODULE_DEVICE_TABLE(spi, lps22hh_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps22hh_id_table[] = {
	{
		.compatible = "st,lps22hh",
		.data = LPS22HH_DEV_NAME,
	},
	{
		.compatible = "st,lps22ch",
		.data = LPS22CH_DEV_NAME,
	},
	{
		.compatible = "st,lps27hhw",
		.data = LPS27HHW_DEV_NAME,
	},
	{
		.compatible = "st,lps27hhtw",
		.data = LPS27HHTW_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, lps22hh_id_table);
#endif

static struct spi_driver lps22hh_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st_lps22hh_spi_input_drv",
		.pm = LPS22HH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lps22hh_id_table),
#endif
	},
	.probe    = lps22hh_spi_probe,
	.remove   = lps22hh_spi_remove,
	.id_table = lps22hh_ids,
};

module_spi_driver(lps22hh_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22hh spi driver");
MODULE_AUTHOR("AMG MSD DIVISION, STMicroelectronics");
MODULE_VERSION(LPS22HH_MODULE_VERSION);
MODULE_LICENSE("GPL v2");
