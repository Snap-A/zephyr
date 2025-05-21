/*
 * Copyright (c) 2025  Andreas Wolf <awolf002@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dac_dacx311, CONFIG_DAC_LOG_LEVEL);

#define DACX311_MAX_RESOLUTION 14
#define DACX311_MAX_CHANNEL    1

struct dacx311_config {
	struct spi_dt_spec bus;
	uint8_t resolution;
};

struct dacx311_data {
	uint8_t resolution;
	uint8_t configured;
};

static int dacx311_reg_write(const struct device *dev, uint16_t val)
{
	const struct dacx311_config *cfg = dev->config;
	uint8_t tx_bytes[2];

        /* Construct write buffer for SPI API */
	const struct spi_buf tx_buf[2] = {
		{
			.buf = tx_bytes,
			.len = sizeof(tx_bytes)
		},
		{
			.buf = NULL,
			.len = 1
		}
        };
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)-1
	};

        /* Set register bits */
        tx_bytes[0] = val >> 8;
        tx_bytes[1] = val & 0xFF;

        /* Write to bus */
	return spi_write_dt(&cfg->bus, &tx);
}

static int dacx311_channel_setup(const struct device *dev,
				const struct dac_channel_cfg *channel_cfg)
{
	const struct dacx311_config *config = dev->config;
	struct dacx311_data *data = dev->data;

	if (channel_cfg->channel_id > 0) {
		LOG_ERR("Unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->internal) {
		LOG_ERR("Internal channels not supported");
		return -ENOTSUP;
	}

	if (data->configured & BIT(channel_cfg->channel_id)) {
		LOG_DBG("Channel %d already configured", channel_cfg->channel_id);
		return 0;
	}

	data->configured |= BIT(channel_cfg->channel_id);

	/* set bit resultion for this chip variant */
	data->resolution = config->resolution;

	LOG_DBG("Channel %d initialized", channel_cfg->channel_id);

	return 0;
}

static int dacx311_write_value(const struct device *dev, uint8_t channel,
				uint32_t value)
{
	struct dacx311_data *data = dev->data;
	uint16_t regval;
        uint8_t shift;
	int ret;

	const bool brdcast = (channel == DAC_CHANNEL_BROADCAST) ? 1 : 0;

	if (!brdcast && (channel > (DACX311_MAX_CHANNEL - 1))) {
		LOG_ERR("Unsupported channel %d", channel);
		return -ENOTSUP;
	}

	/*
	 * Check if channel is initialized
	 * If broadcast channel is used, check if any channel is initialized
	 */
	if ((brdcast && !data->configured) ||
	    (channel < DACX311_MAX_CHANNEL && !(data->configured & BIT(channel)))) {
		LOG_ERR("Channel %d not initialized", channel);
		return -EINVAL;
	}

	if (value >= (1 << (data->resolution))) {
		LOG_ERR("Value %d out of range", value);
		return -EINVAL;
	}

        shift = DACX311_MAX_RESOLUTION - data->resolution;
	/*
	 * Shift passed value to align MSB bit position to register bit 13.
	 *
	 * DAC output register format:
	 *
	 * | 15 14 | 13 12 11 10  9  8  7  6  5  4  3  2  1  0         |
	 * |-------|---------------------------------------------------|
	 * | Mode  | 8311[13:0] / 7311[13:2] / 6311[13:4] / 5311[13:6] |
	 */
	regval = value << shift;

        /*
         * Set mode bits to '0 0' to enable normal output drivers
         */
	regval &= 0x3FFF;

	ret = dacx311_reg_write(dev, regval);
	if (ret) {
		LOG_ERR("Unable to set value %d on channel %d", value, channel);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(dac, dacx311_driver_api) = {
	.channel_setup = dacx311_channel_setup,
	.write_value = dacx311_write_value,
};

#define INST_DT_DACX311(inst, t) DT_INST(inst, ti_dac##t)

#define DACX311_DEVICE(t, n, res) \
	static struct dacx311_data dac##t##_data_##n; \
	static const struct dacx311_config dac##t##_config_##n = { \
		.bus = SPI_DT_SPEC_GET(INST_DT_DACX311(n, t), \
					 SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
					 SPI_MODE_CPOL | SPI_MODE_CPHA | \
					 SPI_WORD_SET(16), 0),  \
		.resolution = res, \
	}; \
	DEVICE_DT_DEFINE(INST_DT_DACX311(n, t), \
				NULL, NULL, \
				&dac##t##_data_##n, \
				&dac##t##_config_##n, POST_KERNEL, \
				CONFIG_DAC_INIT_PRIORITY, \
				&dacx311_driver_api)

/*
 * DAC8311: 14-bit
 */
#define DAC8311_DEVICE(n) DACX311_DEVICE(8311, n, 14)

/*
 * DAC7311: 12-bit
 */
#define DAC7311_DEVICE(n) DACX311_DEVICE(7311, n, 12)

/*
 * DAC6311: 10-bit
 */
#define DAC6311_DEVICE(n) DACX311_DEVICE(6311, n, 10)

/*
 * DAC5311: 8-bit
 */
#define DAC5311_DEVICE(n) DACX311_DEVICE(5311, n, 8)


#define CALL_WITH_ARG(arg, expr) expr(arg)

#define INST_DT_DACX311_FOREACH(t, inst_expr) \
	LISTIFY(DT_NUM_INST_STATUS_OKAY(ti_dac##t), \
		     CALL_WITH_ARG, (), inst_expr)

INST_DT_DACX311_FOREACH(8311, DAC8311_DEVICE);
INST_DT_DACX311_FOREACH(7311, DAC7311_DEVICE);
INST_DT_DACX311_FOREACH(6311, DAC6311_DEVICE);
INST_DT_DACX311_FOREACH(5311, DAC5311_DEVICE);
