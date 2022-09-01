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
#include <linux/err.h>
#include <linux/gpio/consumer.h>

struct  adrf5740_chip_info {
        const char			*name;
	const struct iio_chan_spec	*channels;
	unsigned int			num_channels;
	unsigned int			num_gpios;
	int				gain_min;
	int				gain_max;
	int				default_gain;
};

struct  adrf5740_state {
	struct	regulator *reg;
	// struct mutex lock; /* protect sensor state */
	struct	adrf5740_chip_info *chip_info;
	struct	gpio_descs *gpios;
	u32	raw_gain;
};

static const struct iio_chan_spec adrf5740_channels[] = {
	{
		.type = IIO_VOLTAGE,						\
	        .output = 1,							\
	        .indexed = 1,							\
	        .channel = 0,						        \
	        .info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	}, 
};

static struct adrf5740_chip_info adrf5740_chip_info = {
		.name = "adrf5740",
		.channels = adrf5740_channels,
		.num_channels = ARRAY_SIZE(adrf5740_channels),
		.num_gpios = 4,
		.gain_min = 0,
		.gain_max = 22,
		.default_gain = 0xB, /* set default gain -22dB*/
};


static int adrf5740_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long mask)
{
        struct adrf5740_state *st = iio_priv(indio_dev);
        int db_gain = 0;
	

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
                db_gain = st->raw_gain * -2; /* 2dB/LSB */
                *val = db_gain;
                *val2 = 0;
                return IIO_VAL_INT_PLUS_MICRO_DB;
		
	}

	return -EINVAL;
}

static int adrf5740_write(struct iio_dev *indio_dev, u32 value)
{
	struct adrf5740_state *st = iio_priv(indio_dev);
        // unsigned long values[BITS_PER_TYPE(value)=32];
	DECLARE_BITMAP(values, BITS_PER_TYPE(value));

	values[0] = value;

        /* memory mapped vs bus controlled */
	gpiod_set_array_value_cansleep(st->gpios->ndescs, st->gpios->desc,
				       NULL, values);
	return 0;
}

static int adrf5740_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val,
				 int val2,
				 long mask)
{
        struct adrf5740_state *st = iio_priv(indio_dev);
        struct adrf5740_chip_info *inf = st->chip_info;
        int db_gain;
	
        db_gain = val;

	if (abs(db_gain) > inf->gain_max || abs(db_gain) < inf->gain_min)
		return -EINVAL;
        
        
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
                st->raw_gain = (abs(db_gain) / 2) & 0xF;
                adrf5740_write(indio_dev, st->raw_gain);
		return 0;
	}

	return -EINVAL;
}

static int adrf5740_write_raw_get_fmt(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	default:
		return -EINVAL;
	}
}

static const struct iio_info adrf5740_info = {
	.read_raw = &adrf5740_read_raw,
	.write_raw = &adrf5740_write_raw,
	.write_raw_get_fmt = &adrf5740_write_raw_get_fmt,
};


static const struct of_device_id adrf5740_of_match[] = {
        {.compatible = "adi,adrf5740"},
        {},

};
MODULE_DEVICE_TABLE(of, adrf5740_of_match);

static int adrf5740_probe(struct platform_device *pdev){
       struct iio_dev *indio_dev;
       struct adrf5740_state *st;

        indio_dev = devm_iio_device_alloc(&pdev->dev, 0);
        if(!indio_dev)
                return -ENOMEM;

        st = iio_priv(indio_dev);
        st->chip_info = &adrf5740_chip_info;
        st->raw_gain = st->chip_info->default_gain;
       
        indio_dev->name = st->chip_info->name;
        indio_dev->info = &adrf5740_info;
        indio_dev->num_channels = st->chip_info->num_channels;
        indio_dev->channels = st->chip_info->channels;
        indio_dev->info = &adrf5740_info;

        st->gpios = devm_gpiod_get_array(&pdev->dev, "ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpios))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->gpios),
				     "failed to get gpios\n");

	if (st->gpios->ndescs != st->chip_info->num_gpios) {
		dev_err(&pdev->dev, "%d GPIOs needed to operate\n",
			st->chip_info->num_gpios);
		return -ENODEV;
	}

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

