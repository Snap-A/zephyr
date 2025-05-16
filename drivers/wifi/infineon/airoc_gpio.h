/*
 * Copyright 2025, Andreas Wolf (awolf002@gmail.com)
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 *  Defines API and constants to perform GPIO operations on Wifi chip
 */

#ifndef INCLUDED_AIROC_GPIO_H_
#define INCLUDED_AIROC_GPIO_H_

#define AIROC_GPIO_LED_PIN   0

#ifdef __cplusplus
extern "C"
{
#endif

/** Sends an IOVAR command to set a given GPIO pin to the specified level.
 *
 *  @param  gpio    : The pin number as seen by the Wifi chip. (See defines)
 *  @param  on      : The logic state the pin should be set to:
 *                    0  => Low/Off
 *                    >0 => High/On
 *  @return 0 (SUCCESS) or Error code
 */
extern int airoc_gpio_set(uint8_t gpio, uint8_t on);

#ifdef __cplusplus
}     /* extern "C" */
#endif
#endif /*  INCLUDED_AIROC_GPIO_H_ */
