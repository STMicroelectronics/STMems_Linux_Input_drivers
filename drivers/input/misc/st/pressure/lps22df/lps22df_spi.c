// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Matteo Dameno <matteo.dameno@st.com>
 *
 * Licensed under the GPL-2.
 */

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include "lps22df.h"

#define SENSORS_SPI_READ 0x80

static int lps22df_spi_read(struct lps22df_prs_data *cdata, u8 reg_addr,
							int len, u8 *data)
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

static int lps22df_spi_write(struct lps22df_prs_data *cdata, u8 reg_addr,
							int len, u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LPS22DF_RX_MAX_LENGTH)
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

static const struct lps22df_prs_transfer_function lps22df_tf_spi = {
	.write = lps22df_spi_write,
	.read = lps22df_spi_read,
};

static int lps22df_spi_probe(struct spi_device *spi)
{
	int err;
	struct lps22df_prs_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->irq = spi->irq;
	cdata->tf = &lps22df_tf_spi;
	cdata->name = spi->modalias;
	cdata->bustype = BUS_SPI;
	spi_set_drvdata(spi, cdata);

	err = lps22df_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int lps22df_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct lps22df_prs_data *cdata = spi_get_drvdata(spi);

	lps22df_common_remove(cdata);
	pr_info("%s: removed\n", LPS22DF_PRS_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lps22df_suspend(struct device *dev)
{
	struct lps22df_prs_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lps22df_common_suspend(cdata);
}

static int lps22df_resume(struct device *dev)
{
	struct lps22df_prs_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lps22df_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(lps22df_pm_ops, lps22df_suspend, lps22df_resume);

#define LPS22DF_PM_OPS		(&lps22df_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LPS22DF_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lps22df_ids[] = {
	{ LPS22DF_PRS_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(spi, lps22df_ids);

#ifdef CONFIG_OF
static const struct of_device_id lps22df_id_table[] = {
	{
		.compatible = "st,lps22df",
		.data = LPS22DF_PRS_DEV_NAME,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, lps22df_id_table);
#endif

static struct spi_driver lps22df_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LPS22DF_PRS_DEV_NAME,
		.pm = LPS22DF_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lps22df_id_table),
#endif
	},
	.probe    = lps22df_spi_probe,
	.remove   = lps22df_spi_remove,
	.id_table = lps22df_ids,
};

module_spi_driver(lps22df_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22df spi driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_LICENSE("GPL v2");
