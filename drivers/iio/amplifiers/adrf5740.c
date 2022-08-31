// SPDX-License-Identifier: GPL-2.0
/*
 * ADRF5740 2 dB LSB, 4-Bit, Silicon Digital Attenuator,
 * 10 MHz to 60 GHz
 *
 * Copyright 2022 Analog Devices Inc.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/platform_device.h>


static const struct of_device_id adrf5740_of_match[] = {
        {.compatible = "adi,adrf5740"},
        {},

};
MODULE_DEVICE_TABLE(of, adrf5740_of_match);

static int adrf5740_probe(struct platform_device *pdev){
       struct iio_dev *indio_dev;
        indio_dev = devm_iio_device_alloc(&pdev->dev, 0);
        if(!indio_dev)
                return -ENOMEM;
        
        indio_dev->name = "adrf5740";
        indio_dev->info = &adrf5740_info;
        dev_info(&pdev->dev, "adrf5740 Probed");
        return devm_iio_device_register(&pdev->dev, indio_dev);
}
	

static struct platform_driver adrf5740_driver = {
        .driver = {
                .name = "adrf5740",
                .of_match_table = adrf5740_of_match,
        },
        .probe = adrf5740_probe,
        
};

module_platform_driver(adrf5740_driver);

MODULE_AUTHOR("Ana-Maria Cusco <anamaria_cusco@yahoo.com>");
MODULE_DESCRIPTION("ADRF5740 Digital Attenuator Driver");
MODULE_LICENSE("GPL v2");

