/*
 * STMicroelectronics ais328dq_spi driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/module.h>

#include "ais328dq.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold cdata->lock */
static int ais328dq_spi_read(struct device *device, u8 addr,
				    int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct ais328dq_acc_data *acc = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = acc->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = acc->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	acc->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, acc->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold cdata->lock */
static int ais328dq_spi_write(struct device *device, u8 addr,
				     int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct ais328dq_acc_data *acc = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = acc->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= AIS328DQ_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	acc->tb.tx_buf[0] = addr;

	memcpy(&acc->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static struct ais328dq_transfer_function ais328dq_spi_tf = {
	.write = ais328dq_spi_write,
	.read = ais328dq_spi_read,
};

#ifdef CONFIG_PM_SLEEP
static int ais328dq_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct ais328dq_acc_data *acc = spi_get_drvdata(spi);

	return ais328dq_acc_enable(acc);
}

static int ais328dq_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct ais328dq_acc_data *acc = spi_get_drvdata(spi);

	return ais328dq_acc_disable(acc);
}

static SIMPLE_DEV_PM_OPS(ais328dq_pm_ops,
			ais328dq_suspend,
			ais328dq_resume);

#define AIS328DQ_PM_OPS		(&ais328dq_pm_ops)
#else /* CONFIG_PM_SLEEP */
#define AIS328DQ_PM_OPS		NULL
#endif /* CONFIG_PM_SLEEP */

static int ais328dq_spi_probe(struct spi_device *spi)
{
	int err;
	struct ais328dq_acc_data *acc;

#ifdef AIS328DQ_DEBUG
	dev_info(&spi->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	acc = kzalloc(sizeof(struct ais328dq_acc_data), GFP_KERNEL);
	if (!acc) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	acc->name = spi->modalias;
	acc->bus_type = BUS_SPI;
	acc->tf = &ais328dq_spi_tf;
	acc->dev = &spi->dev;
	spi_set_drvdata(spi, acc);

	mutex_init(&acc->lock);

	err = ais328dq_acc_probe(acc);
	if (err < 0) {
		kfree(acc);
		return err;
	}

	return 0;
}

int ais328dq_spi_remove(struct spi_device *spi)
{
	struct ais328dq_acc_data *acc = spi_get_drvdata(spi);

#ifdef AIS328DQ_DEBUG
	dev_info(acc->dev, "driver removing\n");
#endif

	ais328dq_acc_remove(acc);
	kfree(acc);

	return 0;
}

static const struct spi_device_id ais328dq_spi_id[] = {
	{ "ais328dq", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ais328dq_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id ais328dq_spi_id_table[] = {
	{ .compatible = "st,ais328dq" },
	{ },
};
MODULE_DEVICE_TABLE(of, ais328dq_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver ais328dq_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ais328dq_spi",
		.pm = AIS328DQ_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = ais328dq_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = ais328dq_spi_probe,
	.remove = ais328dq_spi_remove,
	.id_table = ais328dq_spi_id,
};

module_spi_driver(ais328dq_spi_driver);

MODULE_DESCRIPTION("ais328dq acc spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

