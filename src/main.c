/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>

#include "includes.h"

#define PAN_PIN       28
#define DIP_2POS_PIN0 3
#define DIP_2POS_PIN1 2

static uint8_t init_state = 0;
static uint8_t pallet_id = 0;
static rt_device_t watchdog_device;

void set_init_state( uint8_t state )
{
    init_state = state;
}

uint8_t get_init_state( void )
{
    return init_state;
}

uint8_t get_pallet_id( void )
{
    return pallet_id;
}

uint8_t is_pallet_installed(void)
{
    if (rt_pin_read(PAN_PIN))
    {
        return 0;
    }

    return 1;
}


uint8_t get_pallet_id_low( void )
{
    uint8_t id = 0;

    if (rt_pin_read(DIP_2POS_PIN0))
    {
        id |= 0x01;
    }

    if (rt_pin_read(DIP_2POS_PIN1))
    {
        id |= 0x02;
    }

    return id;
}

static void bsp_gpio_init(void)
{
    rt_pin_mode(PAN_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(DIP_2POS_PIN0, PIN_MODE_INPUT);
    rt_pin_mode(DIP_2POS_PIN1, PIN_MODE_INPUT);
}

static void app_thread_startup(void)
{
    rt_err_t err_result;

    bsp_gpio_init();
    
    set_init_state( 0 );
    init_heartbeat_led();
    set_heartbeat_intval( 400 );
    pallet_id =  get_pallet_id_low();
    protocol_init();
    init_sensors();
    init_led_bars();
    set_init_state( 0xFF );
    set_heartbeat_intval( 4000 );
}

void watch_dog_timer_timeout( void *arg )
{
    if (watchdog_device != RT_NULL)
    {
        rt_device_control(watchdog_device,RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
    }
}

int main(void)
{
    rt_timer_t watchdog_timer;
    
    watchdog_device = rt_device_find("wdt");
    if (watchdog_device != RT_NULL)
    {
        rt_device_init(watchdog_device);
        rt_device_open(watchdog_device, RT_DEVICE_FLAG_WRONLY);
        rt_device_control(watchdog_device,RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
        rt_device_control(watchdog_device,RT_DEVICE_CTRL_WDT_START, RT_NULL);
    }

    watchdog_timer = rt_timer_create("watchdog", 
                                     watch_dog_timer_timeout,
                                     RT_NULL, 
                                     RT_TICK_PER_SECOND, 
                                     RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    if (watchdog_timer != RT_NULL)
    {
        rt_timer_start(watchdog_timer); //启动喂狗定时器
    }
    
    app_thread_startup();

    return 0;
}




