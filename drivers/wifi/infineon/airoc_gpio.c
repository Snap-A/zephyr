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
#include <airoc_whd_hal_common.h>

#define IOVAR_STR_GPIOOUT "gpioout"

extern whd_interface_t airoc_wifi_get_whd_interface(void);

static whd_result_t whd_wifi_set_gpioout(uint32_t gpio_bit, uint32_t gpio_val) {
	uint32_t buf[2];
	uint16_t buf_len = sizeof(buf);

	/* The data payload for the GPIOOUT command are the two 32 bit values */
	buf[0] = gpio_bit;
	buf[1] = gpio_val;

	whd_interface_t ifp = airoc_wifi_get_whd_interface();
	if (ifp) {
		return whd_wifi_set_iovar_buffer(ifp, IOVAR_STR_GPIOOUT, buf, buf_len);
	}
	return -1;
}

/*---------------------------------------------------------*/

int airoc_gpio_set(uint8_t gpio, uint8_t on) {
	/* The GPIO pin is encoded as a bit in a 32bit integer */
	whd_result_t res = whd_wifi_set_gpioout(BIT(gpio), on ? BIT(gpio) : 0);
	return res;
}
