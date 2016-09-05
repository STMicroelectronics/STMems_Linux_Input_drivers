/*
 * STMicroelectronics asm330lxh spi driver
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

#include "asm330lxh.h"

#define SENSORS_SPI_READ 0x80

static int asm330lxh_spi_read(struct asm330lxh_status *cdata, u8 reg_addr, int len,
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

static int asm330lxh_spi_write(struct asm330lxh_status *cdata, u8 reg_addr, int len,
			       u8 *data)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= ASM330LXH_RX_MAX_LENGTH)
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

static const struct asm330lxh_transfer_function asm330lxh_tf_spi = {
	.write = asm330lxh_spi_write,
	.read = asm330lxh_spi_read,
};

static int asm330lxh_spi_probe(struct spi_device *spi)
{
	int err;
	struct asm330lxh_status *cdata;

	cdata = kzalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->irq = spi->irq;
	cdata->tf = &asm330lxh_tf_spi;
	cdata->bustype = BUS_SPI;
	spi_set_drvdata(spi, cdata);

	err = asm330lxh_common_probe(cdata);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int asm330lxh_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct asm330lxh_status *cdata = spi_get_drvdata(spi);

	asm330lxh_common_remove(cdata);
	dev_info(cdata->dev, "%s: removed\n", ASM330LXH_ACC_GYR_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int asm330lxh_suspend(struct device *dev)
{
	struct asm330lxh_status *cdata = spi_get_drvdata(to_spi_device(dev));

	return asm330lxh_common_suspend(cdata);
}

static int asm330lxh_resume(struct device *dev)
{
	struct asm330lxh_status *cdata = spi_get_drvdata(to_spi_device(dev));

	return asm330lxh_common_resume(cdata);
}

static const struct dev_pm_ops asm330lxh_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(asm330lxh_suspend, asm330lxh_resume)
};

#define ASM330LXH_PM_OPS	(&asm330lxh_pm_ops)
#else /* CONFIG_PM */
#define ASM330LXH_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct spi_device_id asm330lxh_ids[] = {
	{ ASM330LXH_ACC_GYR_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, asm330lxh_ids);

#ifdef CONFIG_OF
static const struct of_device_id asm330lxh_id_table[] = {
	{ .compatible = "st,asm330lxh", },
	{ },
};
MODULE_DEVICE_TABLE(of, asm330lxh_id_table);
#endif

static struct spi_driver asm330lxh_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ASM330LXH_ACC_GYR_DEV_NAME,
		.pm = ASM330LXH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(asm330lxh_id_table),
#endif
	},
	.probe    = asm330lxh_spi_probe,
	.remove   = asm330lxh_spi_remove,
	.id_table = asm330lxh_ids,
};

module_spi_driver(asm330lxh_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics asm330lxh spi driver");
MODULE_AUTHOR("Giuseppe Barba");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
