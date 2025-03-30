/*
 * Copyright (c) 2025, Andreas Wolf
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief AIROC Wi-Fi GPIO interface.
 */

#include <errno.h>
#include <whd_buffer_api.h>
#include <zephyr/drivers/gpio.h>

#include <airoc_whd_hal_common.h>

#define IOVAR_STR_GPIOOUT "gpioout"

extern whd_interface_t airoc_wifi_get_whd_interface(void);

static whd_result_t whd_wifi_set_gpioout(uint32_t gpio_bit, uint32_t gpio_val)
{
    uint32_t buf[2];
    uint16_t buf_len = sizeof(buf);

    /* The data payload for the GPIOOUT command are the two 32 bit values */
    buf[0] = gpio_bit;
    buf[1] = gpio_val;

    whd_interface_t ifp = airoc_wifi_get_whd_interface();
    if (ifp)
    {
    	return whd_wifi_set_iovar_buffer(ifp, IOVAR_STR_GPIOOUT, buf, buf_len);
    }
    return -1;
}

/*---------------------------------------------------------*/

//#ifdef DT_DRV_COMPAT
//#undef DT_DRV_COMPAT
//#endif
#define DT_DRV_COMPAT airoc_gpio

struct gpio_airoc_data {
   whd_interface_t ifp;
   struct k_mutex gpio_mutex;
   uint32_t allowed_mask;
   uint32_t pin_value;
};

static int gpio_airoc_bus_init(const struct device *dev)
{
   struct gpio_airoc_data *data = dev->data;
   k_mutex_init(&(data->gpio_mutex));

   return 0;
}

static int gpio_airoc_configure(const struct device *dev,
				gpio_pin_t pin,
				gpio_flags_t flags)
{
   struct gpio_airoc_data *data = dev->data;
   uint32_t value;
   whd_result_t res;

   k_mutex_lock(&(data->gpio_mutex), K_FOREVER);
   value = data->pin_value;
   res = WHD_SUCCESS;

   if (flags & GPIO_OUTPUT)
   {
      data->allowed_mask |= BIT(pin);

      if (flags & GPIO_OUTPUT_INIT_HIGH)
      {
         value |= BIT(pin);
         res = whd_wifi_set_gpioout(data->allowed_mask, value);
      }
      else if (flags & GPIO_OUTPUT_INIT_LOW)
      {
         value &= ~BIT(pin);
         res = whd_wifi_set_gpioout(data->allowed_mask, value);
      }

      data->pin_value = value;
   }

   if (res == WHD_SUCCESS)
   {
      data->pin_value = value;
      k_mutex_unlock(&(data->gpio_mutex));
      return 0;
   }

   k_mutex_unlock(&(data->gpio_mutex));
   return -1;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_airoc_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *flags)
{
   struct gpio_airoc_data *data = dev->data;
   k_mutex_lock(&(data->gpio_mutex), K_FOREVER);
   uint32_t is_valid_pin = BIT(pin) & data->allowed_mask;

   *flags = 0;
   if (is_valid_pin)
   {
      /* Always the same value? */
      *flags |= GPIO_OUTPUT_HIGH;
      *flags |= GPIO_OUTPUT;
   }

   k_mutex_unlock(&(data->gpio_mutex));
   return 0;
}
#endif

static int gpio_airoc_port_get_raw(const struct device *port, uint32_t *value)
{
   struct gpio_airoc_data *data = port->data;
   *value = data->pin_value;
   return 0;
}

static int gpio_airoc_port_set_masked_raw(const struct device *port,
                                          uint32_t mask, uint32_t value)
{
   struct gpio_airoc_data *data = port->data;
   k_mutex_lock(&(data->gpio_mutex), K_FOREVER);

   uint32_t value_new = data->pin_value | (value & (mask & data->allowed_mask));

   whd_result_t res = whd_wifi_set_gpioout((mask & data->allowed_mask), value_new);
   if (res == WHD_SUCCESS)
   {
      data->pin_value = value_new;
      k_mutex_unlock(&(data->gpio_mutex));
      return 0;
   }

   k_mutex_unlock(&(data->gpio_mutex));
   return -1;
}

static int gpio_airoc_port_set_bits_raw(const struct device *port,
                                        uint32_t pins)
{
   struct gpio_airoc_data *data = port->data;
   k_mutex_lock(&(data->gpio_mutex), K_FOREVER);

   uint32_t value = data->pin_value | (pins & data->allowed_mask);

   whd_result_t res = whd_wifi_set_gpioout((pins & data->allowed_mask), value);
   if (res == WHD_SUCCESS)
   {
      data->pin_value = value;
      k_mutex_unlock(&(data->gpio_mutex));
      return 0;
   }
   k_mutex_unlock(&(data->gpio_mutex));
   return -1;
}

static int gpio_airoc_port_clear_bits_raw(const struct device *port,
                                          uint32_t pins)
{
   struct gpio_airoc_data *data = port->data;
   k_mutex_lock(&(data->gpio_mutex), K_FOREVER);

   uint32_t value = data->pin_value & ~(pins & data->allowed_mask);

   whd_result_t res = whd_wifi_set_gpioout((pins & data->allowed_mask), value);
   if (res == WHD_SUCCESS)
   {
      data->pin_value = value;
      k_mutex_unlock(&(data->gpio_mutex));
      return 0;
   }

   k_mutex_unlock(&(data->gpio_mutex));
   return -1;
}

static int gpio_airoc_port_toggle_bits(const struct device *port,
                                       uint32_t pins)
{
   struct gpio_airoc_data *data = port->data;
   k_mutex_lock(&(data->gpio_mutex), K_FOREVER);

   uint32_t value = data->pin_value ^ (pins & data->allowed_mask);

   whd_result_t res = whd_wifi_set_gpioout((pins & data->allowed_mask), value);
   if (res == WHD_SUCCESS)
   {
      data->pin_value = value;
      k_mutex_unlock(&(data->gpio_mutex));
      return 0;
   }

   k_mutex_unlock(&(data->gpio_mutex));
   return -1;
}

static int gpio_airoc_pic_not_supported(const struct device *,
                                        gpio_pin_t,
                                        enum gpio_int_mode,
                                        enum gpio_int_trig)
{
   /* Always return error code for invalid */
   return -EINVAL;
}

static int gpio_airoc_mc_not_supported(const struct device *,
                                       struct gpio_callback *,
                                       _Bool)
{
   /* Always return error code for invalid */
   return -EINVAL;
}

static uint32_t gpio_airoc_gpi_not_supported(const struct device *)
{
   /* Always return error code for invalid */
   return -EINVAL;
}

/*---------------------------------------------------------*/

static DEVICE_API(gpio, gpio_airoc_driver_api) = {
	.pin_configure = gpio_airoc_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_airoc_get_config,
#endif
	.port_get_raw = gpio_airoc_port_get_raw,
	.port_set_masked_raw = gpio_airoc_port_set_masked_raw,
	.port_set_bits_raw = gpio_airoc_port_set_bits_raw,
	.port_clear_bits_raw = gpio_airoc_port_clear_bits_raw,
	.port_toggle_bits = gpio_airoc_port_toggle_bits,
	.pin_interrupt_configure = gpio_airoc_pic_not_supported,
	.manage_callback = gpio_airoc_mc_not_supported,
	.get_pending_int = gpio_airoc_gpi_not_supported,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_airoc_port_get_direction,
#endif
};

/*---------------------------------------------------------*/

int airoc_gpio_set(uint8_t gpio, uint8_t on)
{
   /* The GPIO pin is encoded as a bit in a 32bit integer */
   whd_result_t res = whd_wifi_set_gpioout(BIT(gpio), on?BIT(gpio):0);
   return res;
}

/*---------------------------------------------------------*/

static struct gpio_airoc_data gpio_airoc_data = { 0 };

DEVICE_DT_DEFINE(DT_N_INST_0_infineon_airoc_gpio, gpio_airoc_bus_init, NULL, &gpio_airoc_data, NULL,
                 POST_KERNEL, CONFIG_WIFI_INIT_PRIORITY,
                 &gpio_airoc_driver_api);
