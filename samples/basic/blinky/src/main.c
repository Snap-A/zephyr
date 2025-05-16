/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#include <zephyr/autoconf.h>

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, AIROC_GPIO_LED_PIN will be defined
#if CONFIG_BOARD_RPI_PICO_RP2040_W
#include <airoc_gpio.h>
#else
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

int main(void)
{
	int ret;
	bool led_state = true;

#if defined(LED0_NODE)
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
#endif

	while (1) {
#if defined(LED0_NODE)
		ret = gpio_pin_toggle_dt(&led);
#elif defined(AIROC_GPIO_LED_PIN)
		led_state = !led_state;
		// Ask the wifi "driver" to set the GPIO on or off
		ret = airoc_gpio_set(AIROC_GPIO_LED_PIN, led_state);
#endif
		if (ret < 0) {
			return 0;
		}

		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
