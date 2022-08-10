// SPDX-License-Identifier: GPL-2.0+
/*
 * IIO Analog Devices Emulator Driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

 #include<linux/module.h>
 #include<linux/spi/spi.h>
 

static struct spi_driver adi_emu_driver = {
    .driver={
        .name="iio-adi-emu",
    }
};


module_spi_driver(adi_emu_driver);


MODULE_AUTHOR("Ana-Maria Cusco <anamaria_cusco@yahoo.com>");
MODULE_DESCRIPTION("IIO Analog Devices Emulator Driver");
MODULE_LICENSE("GPL v2");



