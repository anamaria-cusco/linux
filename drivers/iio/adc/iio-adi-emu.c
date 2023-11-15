// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. Emulator Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
/* Regmap support */
#include <linux/regmap.h>

#define REG_DEVICE_CONFIG 	0x02
#define POWER_DOWN		BIT(5)

struct adi_emu_priv {
	bool enable;
	struct regmap 	*regmap;
};

static int adi_emu_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = priv->enable;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->channel)
			*val = 1;
		else
			*val = 0;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adi_emu_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_write(priv->regmap, REG_DEVICE_CONFIG,
			!val ? POWER_DOWN : 0);
		if (ret)
			return ret;
		priv->enable = val;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct iio_info adi_emu_info = {
	.read_raw = &adi_emu_read_raw,
	.write_raw = &adi_emu_write_raw,
};

static const struct regmap_config adi_emu_regmap_config = {
	.reg_bits = 8, /*  number of bits used for the register addresses in the device */
	.val_bits = 8, /* number of bits used for the register values in the device */
	.max_register = 0x8, /* highest register address */
};

/*
* This device (ADC) has 2 input voltage channel (voltage measurement channels)
* Each channel has 1 attribute (raw) and 1 shared attribute (enable)
* Raw attribute is used to read the voltage value
* Enable attribute is used to enable/disable the channel
*/
static const struct iio_chan_spec adi_emu_channels[] = {
	{
		.type = IIO_VOLTAGE,
		/* attributes will be specific to this channel */ 
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		/* attributes are shared by all channels */
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0, /* Channel is output. */
		.indexed = 1, /* Specify the channel has a numerical index. */
		.channel = 0, /* Index of the channel */
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0, 
		.indexed = 1,
		.channel = 1, 
	}
};

static int adi_emu_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adi_emu_priv *priv;

	/* allocate an iio_dev from a driver, sizeof priv is zero - no internal structure */
	indio_dev = devm_iio_device_alloc(&spi->dev,  sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->enable = false;
	/* initializes a regmap for a SPI device */
	priv->regmap = devm_regmap_init_spi(spi, &adi_emu_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	indio_dev->name = "iio-adi-emu";
	indio_dev->channels = adi_emu_channels;
	indio_dev->num_channels = ARRAY_SIZE(adi_emu_channels);
	indio_dev->info = &adi_emu_info;

	/*  register a device with the IIO subsystem */
	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver adi_emu_driver = {
	.driver = {
		.name = "iio-adi-emu",
	},
	.probe = adi_emu_probe,
};
module_spi_driver(adi_emu_driver);

MODULE_AUTHOR("Ana-Maria Cusco <ana-maria.cusco@analog.com>");
MODULE_DESCRIPTION("IIO ADI Emulator Driver");
MODULE_LICENSE("GPL v2");