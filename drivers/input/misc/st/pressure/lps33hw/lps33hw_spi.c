/*
 * STMicroelectronics lps33hw spi driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 * v 1.0.0
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

#include "lps33hw.h"

#define SENSORS_SPI_READ 0x80

static int lps33hw_spi_read(struct lps33_prs_data *cdata, u8 reg_addr, int len,
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

static int lps33hw_spi_write(struct lps33_prs_data *cdata, u8 reg_addr, int len,
			     u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LPS33HW_RX_MAX_LENGTH)
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

static const struct lps33_prs_transfer_function lps33hw_tf_spi = {
	.write = lps33hw_spi_write,
	.read = lps33hw_spi_read,
};

static int lps33hw_spi_probe(struct spi_device *spi)
{
	int err;
	struct lps33_prs_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->irq = spi->irq;
	cdata->tf = &lps33hw_tf_spi;
	cdata->name = spi->modalias;
	cdata->bustype = BUS_SPI;
	spi_set_drvdata(spi, cdata);

	err = lps33hw_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lps33hw_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lps33_prs_data *cdata = spi_get_drvdata(spi);

	lps33hw_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", LPS33_PRS_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int lps33hw_suspend(struct device *dev)
{
	struct lps33_prs_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lps33hw_common_suspend(cdata);
}

static int lps33hw_resume(struct device *dev)
{
	struct lps33_prs_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lps33hw_common_resume(cdata);
}

static const struct dev_pm_ops lps33hw_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lps33hw_suspend, lps33hw_resume)
};

#define LPS33HW_PM_OPS		(&lps33hw_pm_ops)
#else /* CONFIG_PM */
#define LPS33HW_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id lps33hw_ids[] = {
	{ LPS33_PRS_DEV_NAME },
	{ LPS35_PRS_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(spi, lps33hw_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps33hw_id_table[] = {
	{
		.compatible = "st,lps33hw",
		.data = LPS33_PRS_DEV_NAME,
	},
	{
		.compatible = "st,lps35hw",
		.data = LPS35_PRS_DEV_NAME,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lps33hw_id_table);
#endif

static struct spi_driver lps33hw_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LPS33_PRS_DEV_NAME,
		.pm = LPS33HW_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lps33hw_id_table),
#endif
	},
	.probe    = lps33hw_spi_probe,
	.remove   = lps33hw_spi_remove,
	.id_table = lps33hw_ids,
};

module_spi_driver(lps33hw_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lps33hw spi driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
