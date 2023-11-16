// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. Emulator Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
/* IIO sysfs support */
#include <linux/iio/sysfs.h> 
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
/* Regmap support */
#include <linux/regmap.h>

#define REG_SCRATCH_PAD 	0x01

#define REG_DEVICE_CONFIG 	0x02
#define POWER_DOWN		BIT(5)

#define REG_CNVST 		0x03
#define CNVST			BIT(0)

#define REG_CH0_DATA_HIGH	0x04
#define REG_CH0_DATA_LOW	0x05
#define REG_CH1_DATA_HIGH	0x06
#define REG_CH1_DATA_LOW	0x07

struct adi_emu_priv {
	bool enable;
	struct regmap 	*regmap;
};

static int adi_emu_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int high, low;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = priv->enable;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		/* A Logic 1 in this bit position starts a single conversion,
		and this bit is automatically reset to 0 at the end of conversion.*/
		ret = regmap_write(priv->regmap, REG_CNVST, CNVST);
		if (ret)
			return ret;
		/* Read MSB and LSB of data for a specific channel */
		if (chan->channel) {
			ret = regmap_read(priv->regmap, REG_CH1_DATA_HIGH, &high);
			if (ret)
				return ret;
			ret = regmap_read(priv->regmap, REG_CH1_DATA_LOW, &low);
			if (ret)
				return ret;
		} else {
			ret = regmap_read(priv->regmap, REG_CH0_DATA_HIGH, &high);
			if (ret)
				return ret;
			ret = regmap_read(priv->regmap, REG_CH0_DATA_LOW, &low);
			if (ret)
				return ret;
		}
		*val = (high << 8) | low;
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

static int adi_emu_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);

	if (readval)
		return regmap_read(priv->regmap, reg, readval);

	return regmap_write(priv->regmap, reg, writeval);
}

enum scratch_pad_iio_dev_attr {
	SCRATCH_PAD,
};

static ssize_t adi_emu_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	/* Retrieve iio_dev using container of */
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	/* Retrieve priv field*/
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	switch (this_attr->address) {
	case SCRATCH_PAD:
		regmap_read(priv->regmap, REG_SCRATCH_PAD, &reg_val);
		ret = sprintf(buf, "0x%x\n", reg_val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t adi_emu_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	switch (this_attr->address) {
	case SCRATCH_PAD:
		/* Convert string to unsigned int in base 16 (hexadecimal)*/
		ret = kstrtouint(buf, 16, &reg_val);
		if (ret)
			break;
		ret = regmap_write(priv->regmap, REG_SCRATCH_PAD, reg_val);
		if (ret < 0)
			break;
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;

}

static IIO_DEVICE_ATTR(scratch_pad, S_IRUGO | S_IWUSR, 
			adi_emu_show, 
			adi_emu_store, SCRATCH_PAD);

static struct attribute *adi_emu_attributes[] = {
	&iio_dev_attr_scratch_pad.dev_attr.attr,
	NULL,
};


static const struct attribute_group adi_emu_attribute_group = {
	.attrs = adi_emu_attributes,
};

static const struct iio_info adi_emu_info = {
	.read_raw = &adi_emu_read_raw,
	.write_raw = &adi_emu_write_raw,
	.debugfs_reg_access = &adi_emu_reg_access,
	.attrs = &adi_emu_attribute_group,
};

static const struct regmap_config adi_emu_regmap_config = {
	.reg_bits = 8, /*  number of bits used for the register addresses in the device */
	.val_bits = 8, /* number of bits used for the register values in the device */
	.max_register = 0x8, /* highest register address */
};

/* This function is called for every trigger event that occurs
* Software trigger - sysfs write
*/
static irqreturn_t adi_emu_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int high, low;
	u16 buf[2];
	int i = 0, ret;

	ret = regmap_write(priv->regmap, REG_CNVST, CNVST);
	if (ret)
		return ret;
	if (*indio_dev->active_scan_mask & BIT(0)) {
		ret = regmap_read(priv->regmap,
			REG_CH0_DATA_HIGH, &high);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap,
			REG_CH0_DATA_LOW, &low);
		if (ret)
			return ret;
		buf[i++] = (high << 8) | low;
	}
	if (*indio_dev->active_scan_mask & BIT(1)) {
		ret = regmap_read(priv->regmap,
			REG_CH1_DATA_HIGH, &high);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap,
			REG_CH1_DATA_LOW, &low);
		if (ret)
			return ret;
		buf[i] = (high << 8) | low;
	}

	/* push the data to the buffers - making it available to userspace*/
	iio_push_to_buffers(indio_dev, buf);

	/* finish processing the triggers*/
	iio_trigger_notify_done(indio_dev->trig);

	/* interrupt has been handled*/
	return IRQ_HANDLED;
}

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
		/* Meta information associated with a channel reading placed in
		* buffer is called a scan element.
		*/
		.scan_index = 0, /* Index of the channel in scan mask */
		.scan_type = {
			.sign = 'u', /* Unsigned */
			.realbits = 12, /* 12 bits */
			.storagebits = 16, /* 16 bits */
			.shift = 0, /* No shift */
			.endianness = IIO_LE, /* Endianness - Little */
		},
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0, 
		.indexed = 1,
		.channel = 1, 
		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 12,
			.shift = 0,
			.endianness = IIO_LE,
		},
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

	/* set up an IIO device with a triggered buffer
	* NULL - for trigger
	* adi_emu_trigger_handler - handler 
	* NULL- buffer setup operations pre/post 
	Note that no trigger is set at this time
	It can be set after device setup through sysfs trigger
	*/
	devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL, 
					&adi_emu_trigger_handler, NULL);
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