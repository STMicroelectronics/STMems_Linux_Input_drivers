/*
 * STMicroelectronics hts221_spi driver
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

#include "hts221_core.h"

#define SENSORS_SPI_READ	0x80
#define SPI_AUTO_INCREMENT	0x40

/* XXX: caller must hold cdata->lock */
static int hts221_spi_read(struct device *device, u8 addr, int len, u8 *data)
{
	int err;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct hts221_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers[] = {
		{
			.tx_buf = dev->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = dev->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr | SENSORS_SPI_READ;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	err = spi_sync(spi, &msg);
	if (err)
		return err;

	memcpy(data, dev->tb.rx_buf, len * sizeof(u8));

	return len;
}

/* XXX: caller must hold cdata->lock */
static int hts221_spi_write(struct device *device, u8 addr, int len, u8 *data)
{
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(device);
	struct hts221_dev *dev = spi_get_drvdata(spi);

	struct spi_transfer xfers = {
		.tx_buf = dev->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= HTS221_TX_MAX_LENGTH)
		return -ENOMEM;

	if (len > 1)
		addr |= SPI_AUTO_INCREMENT;

	dev->tb.tx_buf[0] = addr;

	memcpy(&dev->tb.tx_buf[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	return spi_sync(spi, &msg);
}

static const struct hts221_transfer_function hts221_spi_tf = {
	.write = hts221_spi_write,
	.read = hts221_spi_read,
};

#ifdef CONFIG_PM
static int hts221_spi_resume(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct hts221_dev *dev = spi_get_drvdata(spi);

	return hts221_enable(dev);
}

static int hts221_spi_suspend(struct device *device)
{
	struct spi_device *spi = to_spi_device(device);
	struct hts221_dev *dev = spi_get_drvdata(spi);

	return hts221_disable(dev);
}

static const struct dev_pm_ops hts221_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hts221_spi_suspend,
				hts221_spi_resume)
};
#endif /* CONFIG_PM */

static int hts221_spi_probe(struct spi_device *spi)
{
	int err;
	struct hts221_dev *dev;

#ifdef HTS221_DEBUG
	dev_info(&client->dev, "probe start.\n");
#endif

	/* Alloc Common data structure */
	dev = kzalloc(sizeof(struct hts221_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&spi->dev, "failed to allocate module data\n");
		return -ENOMEM;
	}

	dev->name = spi->modalias;
	dev->bus_type = BUS_SPI;
	dev->tf = &hts221_spi_tf;
	dev->dev = &spi->dev;
	spi_set_drvdata(spi, dev);

	mutex_init(&dev->lock);

	err = hts221_probe(dev);
	if (err < 0) {
		kfree(dev);
		return err;
	}

	return 0;
}

int hts221_spi_remove(struct spi_device *spi)
{
	struct hts221_dev *dev = spi_get_drvdata(spi);

#ifdef HTS221_DEBUG
	dev_info(&client->dev, "driver removing\n");
#endif

	hts221_remove(dev);
	kfree(dev);

	return 0;
}

static const struct spi_device_id hts221_spi_id[] = {
	{ "hts221", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, hts221_spi_id);

#ifdef CONFIG_OF
static const struct of_device_id hts221_spi_id_table[] = {
	{ .compatible = "st,hts221" },
	{ },
};
MODULE_DEVICE_TABLE(of, hts221_spi_id_table);
#endif /* CONFIG_OF */

static struct spi_driver hts221_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "hts221_spi",
#ifdef CONFIG_PM
		.pm = &hts221_spi_pm_ops,
#endif /* CONFIG_PM */
#ifdef CONFIG_OF
		.of_match_table = hts221_spi_id_table,
#endif /* CONFIG_OF */
	},
	.probe = hts221_spi_probe,
	.remove = hts221_spi_remove,
	.id_table = hts221_spi_id,
};

module_spi_driver(hts221_spi_driver);

MODULE_DESCRIPTION("hts221 spi driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");

