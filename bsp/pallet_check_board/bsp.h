/**
 * @file            bsp.c
 * @author          philZeng@outlook.com
 * @version         V0.0.1
 * @date            2018年8月6号
 * @brief           功能板板级支持包
 * @attention       
 */
#ifndef __BSP_H__
#define __BSP_H__
#include <rtconfig.h>
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <soc.h>

uint8_t get_pallet_id_low( void );
uint8_t is_pallet_installed(void);

#ifdef USING_WS2812_LED_DEV
#include "ws2812.h"
#endif


#ifdef RT_USING_I2C

#ifdef RT_USING_I2C_BITOPS
#include "soft_i2c.h"
#endif

#endif

#ifdef USING_PALLET_LED_BAR_DEV
#include "pallet_led_bar.h"
#endif

#ifdef USING_HEARTBEAT_LED_DEV
#include "heartbeat_led.h"
#endif

void rt_hw_board_init(void);

#endif
