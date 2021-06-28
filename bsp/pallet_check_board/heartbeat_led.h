/**
 * @file heartbeat_led.h
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-10-29
 * 
 * 
 */
#ifndef __HEARTBEAT_LED_H__
#define __HEARTBEAT_LED_H__

#include <rtthread.h>
#include <rtdevice.h>

rt_err_t init_heartbeat_led_dev( rt_device_t dev, const char *dev_name );

#endif
