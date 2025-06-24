/*
 * Copyright (c) 2025 Andreas Wolf <awolf002@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Driver for 74LVC1G373 D-type latch
 */

#define DT_DRV_COMPAT ti_sn74lvc1g373

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_sn74lvc1g373, CONFIG_GPIO_LOG_LEVEL);

#if CONFIG_SPI_INIT_PRIORITY >= CONFIG_GPIO_SN74LVC1G373_INIT_PRIORITY
#error SPI_INIT_PRIORITY must be lower than SN74LVC1G373_INIT_PRIORITY
#endif

struct gpio_sn74lvc1g373_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config config;

	struct spi_dt_spec bus;
};

struct gpio_sn74lvc1g373_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data data;

	struct k_mutex lock;
	uint8_t output;
};

static int sn74lvc1g373_spi_write(const struct device *dev, uint8_t data)
{
	const struct gpio_sn74lvc1g373_config *config = dev->config;
#if 0
	const struct spi_config *spi = &config->bus.config;
	const struct gpio_dt_spec *cs = &spi->cs.gpio;
	int ret = 0;

	// Lock SPI and set 'MOSI' to value

	// Set CS after data pin to high
	ret = gpio_pin_set_dt(cs, 1);
	if (!ret) {
		LOG_ERR("Not able to change CS to '1'");
		return ret;
	}

	// Sleep 1 us
	k_usleep(1);

	// Set CS after data pin to low
	ret = gpio_pin_set_dt(cs, 0);
	if (!ret) {
		LOG_ERR("Not able to change CS to '0'");
		return ret;
	}

	// Reset 'MOSI' to '0' and unlock SPI

	return ret;
#else
	uint8_t buf;
	struct spi_buf tx_buf[] = { { .buf = &buf, .len = 1 } };
	const struct spi_buf_set tx = { .buffers = tx_buf, .count = 1 };

	if (data)
		buf = 0xFF;
	else
		buf = 0;

	return spi_write_dt(&config->bus, &tx);
#endif
}

static int gpio_sn74lvc1g373_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(flags);
	return 0;
}

static int gpio_sn74lvc1g373_port_get_raw(const struct device *dev, uint32_t *value)
{
	struct gpio_sn74lvc1g373_drv_data *drv_data = dev->data;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	*value = drv_data->output;

	k_mutex_unlock(&drv_data->lock);

	return 0;
}

static int gpio_sn74lvc1g373_port_set_masked_raw(const struct device *dev, uint32_t mask,
					      uint32_t value)
{
	struct gpio_sn74lvc1g373_drv_data *drv_data = dev->data;
	int ret = 0;
	uint8_t output;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	/* check if we need to do something at all      */
	/* current output differs from new masked value */
	if ((drv_data->output & mask) != (mask & value)) {
		output = (drv_data->output & ~mask) | (mask & value);

		ret = sn74lvc1g373_spi_write(dev, output);
		if (ret < 0) {
			goto unlock;
		}

		drv_data->output = output;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int gpio_sn74lvc1g373_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	return gpio_sn74lvc1g373_port_set_masked_raw(dev, mask, mask);
}

static int gpio_sn74lvc1g373_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	return gpio_sn74lvc1g373_port_set_masked_raw(dev, mask, 0U);
}

static int gpio_sn74lvc1g373_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	struct gpio_sn74lvc1g373_drv_data *drv_data = dev->data;
	int ret;
	uint8_t toggled_output;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	toggled_output = drv_data->output ^ mask;

	ret = sn74lvc1g373_spi_write(dev, toggled_output);
	if (ret < 0) {
		goto unlock;
	}

	drv_data->output ^= mask;

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

/**
 * @brief Initialization function of sn74lvc1g373
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_sn74lvc1g373_init(const struct device *dev)
{
	const struct gpio_sn74lvc1g373_config *config = dev->config;
	struct gpio_sn74lvc1g373_drv_data *drv_data = dev->data;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	drv_data->output = 0U;
	return 0;
}

static DEVICE_API(gpio, gpio_sn74lvc1g373_drv_api_funcs) = {
	.pin_configure = gpio_sn74lvc1g373_config,
	.port_get_raw = gpio_sn74lvc1g373_port_get_raw,
	.port_set_masked_raw = gpio_sn74lvc1g373_port_set_masked_raw,
	.port_set_bits_raw = gpio_sn74lvc1g373_port_set_bits_raw,
	.port_clear_bits_raw = gpio_sn74lvc1g373_port_clear_bits_raw,
	.port_toggle_bits = gpio_sn74lvc1g373_port_toggle_bits,
};

#define SN74LVC1G373_SPI_OPERATION								\
	((uint16_t)(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)))

#define SN74LVC1G373_INIT(n)									\
	static struct gpio_sn74lvc1g373_drv_data sn74lvc1g373_data_##n = {			\
		.output = 0,									\
		.lock = Z_MUTEX_INITIALIZER(sn74lvc1g373_data_##n.lock),			\
	};											\
												\
	static const struct gpio_sn74lvc1g373_config sn74lvc1g373_config_##n = {	       	\
		.bus = SPI_DT_SPEC_INST_GET(n, SN74LVC1G373_SPI_OPERATION, 0),			\
	};											\
												\
	DEVICE_DT_DEFINE(DT_DRV_INST(n), &gpio_sn74lvc1g373_init, NULL,				\
			 &sn74lvc1g373_data_##n, &sn74lvc1g373_config_##n, POST_KERNEL,		\
			 CONFIG_GPIO_SN74LVC1G373_INIT_PRIORITY, &gpio_sn74lvc1g373_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(SN74LVC1G373_INIT)
