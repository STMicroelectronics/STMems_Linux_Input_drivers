/*
 * STMicroelectronics asm330lhh spi driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 * v 1.0
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>

#include "asm330lhh_core.h"

#define SENSORS_SPI_READ 0x80

static int asm330lhh_spi_read(struct asm330lhh_data *cdata, u8 reg_addr, int len,
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

static int asm330lhh_spi_write(struct asm330lhh_data *cdata, u8 reg_addr, int len,
			     u8 *data, bool b_lock)
{
	int err;
	struct spi_message msg;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= ASM330LHH_RX_MAX_LENGTH)
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

static const struct asm330lhh_transfer_function asm330lhh_tf_spi = {
	.write = asm330lhh_spi_write,
	.read = asm330lhh_spi_read,
};

static int asm330lhh_spi_probe(struct spi_device *spi)
{
	int err;
	struct asm330lhh_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &asm330lhh_tf_spi;
	spi_set_drvdata(spi, cdata);

	err = asm330lhh_common_probe(cdata, spi->irq, BUS_SPI);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);

	return err;
}

static int asm330lhh_spi_remove(struct spi_device *spi)
{
	/* TODO: check the function */
	struct asm330lhh_data *cdata = spi_get_drvdata(spi);

	asm330lhh_common_remove(cdata, spi->irq);
	dev_info(cdata->dev, "%s: removed\n", ASM330LHH_DEV_NAME);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int asm330lhh_suspend(struct device *dev)
{
	struct asm330lhh_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return asm330lhh_common_suspend(cdata);
}

static int asm330lhh_resume(struct device *dev)
{
	struct asm330lhh_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return asm330lhh_common_resume(cdata);
}

static SIMPLE_DEV_PM_OPS(asm330lhh_pm_ops, asm330lhh_suspend, asm330lhh_resume);

#define ASM330LHH_PM_OPS		(&asm330lhh_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define ASM330LHH_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id asm330lhh_ids[] = {
	{ ASM330LHH_DEV_NAME, 0 },
	{ LSM6DSR_DEV_NAME, 1 },
	{ ISM330DHCX_DEV_NAME, 2 },
	{ }
};
MODULE_DEVICE_TABLE(spi, asm330lhh_ids);

#ifdef CONFIG_OF
static const struct of_device_id asm330lhh_id_table[] = {
	{ .compatible = "st," ASM330LHH_DEV_NAME, },
	{ .compatible = "st," LSM6DSR_DEV_NAME, },
	{ .compatible = "st," ISM330DHCX_DEV_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, asm330lhh_id_table);
#endif

static struct spi_driver asm330lhh_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ASM330LHH_DEV_NAME,
		.pm = ASM330LHH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = asm330lhh_id_table,
#endif
	},
	.probe    = asm330lhh_spi_probe,
	.remove   = asm330lhh_spi_remove,
	.id_table = asm330lhh_ids,
};

module_spi_driver(asm330lhh_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics asm330lhh spi driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
