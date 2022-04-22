/*
 * STMicroelectronics lis3dhh spi driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Version 1.0.0
 *
 * Licensed under the GPL-2.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/input.h>

#include "lis3dhh.h"

#define SENSORS_SPI_READ	0x80

static int lis3dhh_acc_spi_read(struct lis3dhh_acc_status *stat, u8 reg_addr, int len,
			       u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(stat->dev);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = stat->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = stat->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	mutex_lock(&stat->tb.buf_lock);
	stat->tb.tx_buf[0] = reg_addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		goto acc_spi_read_error;

	memcpy(data, stat->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&stat->tb.buf_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&stat->tb.buf_lock);

	return err;
}

static int lis3dhh_acc_spi_write(struct lis3dhh_acc_status *stat, u8 reg_addr, int len,
			       u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(stat->dev);

	struct spi_transfer xfers = {
		.tx_buf = stat->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= BUFF_RX_MAX_LENGTH)
		return -ENOMEM;

	mutex_lock(&stat->tb.buf_lock);
	stat->tb.tx_buf[0] = reg_addr;

	memcpy(&stat->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	err = spi_sync(spi, &msg);
	mutex_unlock(&stat->tb.buf_lock);

	return err;
}

static struct lis3dhh_acc_transfer_function lis3dhh_acc_tf_spi = {
	.write = lis3dhh_acc_spi_write,
	.read = lis3dhh_acc_spi_read,
};

static int lis3dhh_acc_spi_probe(struct spi_device *spi)
{
	int err;
	struct lis3dhh_acc_status *stat;

	stat = kzalloc(sizeof(struct lis3dhh_acc_status), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &spi->dev;
	stat->name = spi->modalias;
	stat->bustype = BUS_SPI;
	stat->tf = &lis3dhh_acc_tf_spi;
	spi_set_drvdata(spi, stat);

	err = lis3dhh_common_probe(stat);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(stat);

	return err;
}

static int lis3dhh_acc_spi_remove(struct spi_device *spi)
{
	struct lis3dhh_acc_status *stat = spi_get_drvdata(spi);

	lis3dhh_common_remove(stat);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lis3dhh_acc_suspend(struct device *dev)
{
	struct lis3dhh_acc_status *stat= spi_get_drvdata(to_spi_device(dev));

	return lis3dhh_common_suspend(stat);
}

static int lis3dhh_acc_resume(struct device *dev)
{
	struct lis3dhh_acc_status *stat = spi_get_drvdata(to_spi_device(dev));

	return lis3dhh_common_resume(stat);
}

static SIMPLE_DEV_PM_OPS(lis3dhh_acc_pm_ops,
				lis3dhh_acc_suspend,
				lis3dhh_acc_resume);

#define LIS3DHH_PM_OPS		(&lis3dhh_acc_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define LIS3DHH_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static const struct spi_device_id lis3dhh_acc_ids[] = {
	{ LIS3DHH_ACC_DEV_NAME, 0 },
	{ IIS3DHHC_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lis3dhh_acc_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis3dhh_acc_id_table[] = {
	{ .compatible = "st,lis3dhh", },
	{ .compatible = "st,iis3dhhc", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis3dhh_acc_id_table);
#endif

static struct spi_driver lis3dhh_acc_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DHH_ACC_DEV_NAME,
		.pm = LIS3DHH_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = lis3dhh_acc_id_table,
#endif
	},
	.remove = lis3dhh_acc_spi_remove,
	.probe = lis3dhh_acc_spi_probe,
	.id_table = lis3dhh_acc_ids,
};

module_spi_driver(lis3dhh_acc_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis3dhh spi driver");
MODULE_AUTHOR("Mario Tesi");
MODULE_LICENSE("GPL v2");
