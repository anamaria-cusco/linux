// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. AD5592R Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
static const struct iio_info ad5592r_info = {
};

static int ad5592r_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	indio_dev=devm_iio_device_alloc(&spi->dev,0);
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->name="iio-ad5592r";
	indio_dev->info=&ad5592r_info;

	dev_info(&spi->dev, "AD5592R Driver Probed!");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad5592r_spi_driver = {
	.driver = {
		.name = "ad5592r_summer",
	},
	.probe=ad5592r_probe,
};


module_spi_driver(ad5592r_spi_driver);

MODULE_AUTHOR("Cusco Ana-Maria <anamaria_cusco@yahoo.com>");
MODULE_DESCRIPTION("Summer Practice AD5592R ADC Driver");
MODULE_LICENSE("GPL v2");