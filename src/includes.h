#ifndef __INCLUDES_H__
#define __INCLUDES_H__
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <config.h>
//#include <soc.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <rtdbg.h>

#ifdef RT_USING_FINSH
#include <shell.h>
#include <finsh.h>
#endif

#include <bsp_device.h>

#include "constant.h"
#include "protocol/protocol.h"
#include "sensors/sensors.h"
#include "led_bar/led_bar.h"

uint8_t get_init_state( void );

uint8_t get_pallet_id( void );

uint8_t is_pallet_installed(void);
#endif
