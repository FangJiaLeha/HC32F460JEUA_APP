/**
 * @file pallet_detect.h
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-09
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
#ifndef __PALLET_DETECT_H__
#define __PALLET_DETECT_H__
#include <stdint.h>
#include <rtthread.h>
#include "constant.h"

#define PF574_1             0
#define PF574_2             1
#define PF574_3             2
#define PF574_4             3

#define INSTALLED_STATE         0x80

#define PF574_CONNECT_STATE     0x8000

#define LAYER_SW_NONE           ((int8_t)-1)
#define LAYER_SW_ON             ((int8_t)1)
#define LAYER_SW_OFF            ((int8_t)0)

#define LAYER_DISABLED          ((int8_t)0)
#define LAYER_ENABLED           ((int8_t)1)

#define IS_LAYER_ENABLED( layer )      ( (layer)->layer_enabled == LAYER_ENABLED )     

//#define CLR_INSTALLED_STATE( s )    do{(s) &= 0xFF7F;}while(0)

//#define SET_INSTALLED_STATE( s )    do{(s) |= 0x0080;}while(0)

//#define CLR_PF574_CONNECTED_STATE( s ) do{ (s) &= 0x7FFF;}while(0)

//#define SET_PF574_CONNECTED_STATE( s ) do{ (s) |= 0x8000;}while(0)

/**
 * @brief 托盘检测模块初始化
 * 
 * @return RT_EOK 
 */
err_t pallet_init( void );


err_t update_pallet_state( void );

/**
 * @brief 获取托盘状态：
 *   bit0       bit1  bit2  bit3 bit4  bit5  bit6     bit7
 * 物品放置状态	  x	     x	  x	    x	 x	    x	托盘安装状态
 * @param[in] state 用于存储状态的buff，该buff需要大于等于4个字节，buff[idx], idx+1 为托盘所在层号
 */
void get_pallet_state( void );

err_t init_layer_vl53l0x_dev( void );

err_t deinit_layer_vl53l0x_dev( void );

void layer_switch( uint8_t val );

void layer_fun_switcher( void );

#endif

