// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. AD5592R Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>



static struct spi_driver ad5592r_spi_driver = {
	.driver = {
		.name = "ad5592r_summer",
	}
};
module_spi_driver(ad5592r_spi_driver);

MODULE_AUTHOR("Cusco Ana-Maria <anamaria_cusco@yahoo.com>");
MODULE_DESCRIPTION("Summer Practice AD5592R ADC Driver");
MODULE_LICENSE("GPL v2");