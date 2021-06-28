/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-27     kyle         first version
 */

#ifndef __BSP_DEVICE_H__
#define __BSP_DEVICE_H__

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef BSP_USING_I2C_TLC59108
#include "tlc59108/tlc59108.h"
#endif /* BSP_USING_I2C_TLC59108 */

#ifdef BSP_USING_I2C_VL53L0X
#include "vl53l0x_dev_lib/vl53l0x_dev.h"
#endif /* BSP_USING_I2C_TLC59108 */

#ifdef BSP_USING_I2C_PCF8574
#include "pcf8574/pcf8574.h"
#endif /* BSP_USING_I2C_PCF8574 */

#ifdef BSP_USING_PWM_WS2812
#include "ws2812/ws2812.h"
#endif /* BSP_USING_PWM_WS2812 */

#endif /* __BSP_DEVICE_H__ */
