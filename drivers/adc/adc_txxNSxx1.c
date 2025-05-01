/*
 * Copyright (c) 2025 Andreas Wolf
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ADC driver for the TI xxNSxx1 ADCs.
 *
 * This driver supports multiple variants of the Texas Instrument ADC chip.
 * There are differences in the number of channels, the resolution, and the 
 * sampling speed between the versions of this chip:
 *
 *  ADCxx1Sxx1 : 1 channel
 *  ADCxx2Sxx1 : 2 channels
 *  ADCxx4Sxx1 : 4 channels
 *
 *  ADC08NSxx1 : 8-bit resolution
 *  ADC10NSxx1 : 10-bit resolution
 *  ADC12NSxx1 : 12-bit resolution
 *
 *  ADCxxNS021 : Up to 200 ksps
 *  ADCxxNS051 : Up to 500 ksps
 *  ADCxxNS101 : Up to 1 Msps
 *
 */

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(adc_txxNSxx1, CONFIG_ADC_LOG_LEVEL);

#define MIN_RESOLUTION 8U
#define MAX_RESOLUTION 12U

struct txxNSxx1_config {
	struct spi_dt_spec bus;
	uint8_t channels;
	uint8_t resolution;
};

struct txxNSxx1_data {
	const struct device *dev;
	uint16_t *buffer;
	uint8_t channels;
	uint8_t resolution;
};

static int txxNSxx1_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	const struct txxNSxx1_config *config = dev->config;
	struct txxNSxx1_data *data = dev->data;

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_EXTERNAL0) {
		LOG_ERR("unsupported channel reference '%d'",
			channel_cfg->reference);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'",
			channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	data->resolution = config->resolution;

	return 0;
}

static int txxNSxx1_read_channel(const struct device *dev, uint8_t channel,
				 uint16_t *result)
{
	const struct txxNSxx1_config *config = dev->config;

        /* The ADC frame length is 16 bits, but only a subset is used for real data.  */
        /* The first frame sets the channel number for the conversion, and the second */
        /* frame contains the obtained data for that channel. The data written in the */
        /* second frame is no useful (selecting channel 0), and the data read in the  */
        /* first frame might originate from an unknown channel and is ignored.        */

	uint8_t tx1_bytes[2];
	uint8_t rx1_bytes[2];
	uint8_t tx2_bytes[2];
	uint8_t rx2_bytes[2];
	int err;
	const struct spi_buf tx_buf[2] = {
		{
			.buf = tx1_bytes,
			.len = sizeof(tx1_bytes)
		},
		{
			.buf = tx2_bytes,
			.len = sizeof(tx2_bytes)
		}
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = rx1_bytes,
			.len = sizeof(rx1_bytes)
		},
		{
			.buf = rx2_bytes,
			.len = sizeof(rx2_bytes)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf)
	};

	/*
	 * Configuration byte consists of: 2 dummy bits + D2 + D1 + D0 + 3 dummy bits
	 */
	tx1_bytes[0] = channel << 3;
	tx1_bytes[1] = 0;
	/*
	 * Just sent zeroes
	 */
	tx2_bytes[0] = 0;
	tx2_bytes[1] = 0;

	err = spi_transceive_dt(&config->bus, &tx, &rx);
	if (err) {
		return err;
	}

	/*
	 * Read data from second frame
	 */
	*result = sys_get_be16(rx2_bytes);

	return 0;
}

static int txxNSxx1_init(const struct device *dev)
{
	const struct txxNSxx1_config *config = dev->config;
	struct txxNSxx1_data *data = dev->data;

	data->dev = dev;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	return 0;
}

static int txxNSxx1_read(const struct device *dev,
			 const struct adc_sequence *sequence)
{
	struct txxNSxx1_data *data = dev->data;
	uint16_t result = 0;
	uint8_t channel;
	int err;

	data->buffer = sequence->buffer;
	data->channels = sequence->channels;

	while (data->channels) {
		channel = find_lsb_set(data->channels) - 1;

		LOG_DBG("reading channel %d", channel);

		err = txxNSxx1_read_channel(data->dev, channel, &result);
		if (err) {
			LOG_ERR("failed to read channel %d (err %d)",
				channel, err);
			break;
		}

		result &= (BIT_MASK(sequence->resolution) << 2);
		int shift = MAX_RESOLUTION - sequence->resolution;
		result >>= shift;

		LOG_DBG("read channel %d, result = %d", channel,
			result);

		*data->buffer++ = result;
		WRITE_BIT(data->channels, channel, 0);
	}

	return err;
}

static DEVICE_API(adc, txxNSxx1_adc_api) = {
	.channel_setup = txxNSxx1_channel_setup,
	.read = txxNSxx1_read,
};

#define INST_DT_ADCXXNSXX1(inst, t) DT_INST(inst, ti_##t)

#define ADCXXNSXX1_DEVICE(t, n, ch) \
	static struct txxNSxx1_data ti_##t##_data_##n = { \
		.channels = ch,                       \
	}; \
	static const struct txxNSxx1_config ti_##t##_config_##n = { \
		.bus = SPI_DT_SPEC_GET(INST_DT_ADCXXNSXX1(n, t), \
					 SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
					 SPI_MODE_CPOL | SPI_MODE_CPHA | \
					 SPI_WORD_SET(16), 0), \
		.channels = ch, \
	}; \
	DEVICE_DT_DEFINE(INST_DT_ADCXXNSXX1(n, t), \
			 &txxNSxx1_init, NULL, \
			 &ti_##t##_data_##n, \
			 &ti_##t##_config_##n, POST_KERNEL, \
			 CONFIG_ADC_INIT_PRIORITY, \
			 &txxNSxx1_adc_api)

/*
 * ADCxx1Sxx1: 1 channels
 */
#define ADCXX1SXX1_DEVICE(n) ADCXXNSXX1_DEVICE(adcxx1sxx1, n, 1)

/*
 * ADCxx2Sxx1: 2 channels
 */
#define ADCXX2SXX1_DEVICE(n) ADCXXNSXX1_DEVICE(adcxx2sxx1, n, 2)

/*
 * ADCxx4Sxx1: 4 channels
 */
#define ADCXX4SXX1_DEVICE(n) ADCXXNSXX1_DEVICE(adcxx4sxx1, n, 4)

#define CALL_WITH_ARG(arg, expr) expr(arg)

#define INST_DT_ADCXXNSXX1_FOREACH(t, inst_expr)	\
	LISTIFY(DT_NUM_INST_STATUS_OKAY(ti_adcxx##t##sxx1),	\
		CALL_WITH_ARG, (;), inst_expr)

INST_DT_ADCXXNSXX1_FOREACH(4, ADCXX4SXX1_DEVICE);
INST_DT_ADCXXNSXX1_FOREACH(2, ADCXX2SXX1_DEVICE);
INST_DT_ADCXXNSXX1_FOREACH(1, ADCXX1SXX1_DEVICE);
