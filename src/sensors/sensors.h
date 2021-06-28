/**
 * @file sensors.h
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-16
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
#ifndef __SENSORS_H__
#define __SENSORS_H__
#include <stdint.h>
#include "constant.h"
#include "pallet_detect.h"

/**
 * @brief 按键触摸状态
 * 
 */
union TOUCH_STATE
{
    uint8_t state;
    struct {
        uint8_t left_ear:1;  //!< 左耳 触摸状态，0：未触摸，1：触摸
        uint8_t head:1;  //!< 额头触摸状态，0：为触摸，1：触摸
        uint8_t right_ear:1; //!< 右耳触摸状态，0：未触摸，1：触摸
        uint8_t key:1;       //!< 按键触摸
        uint8_t resv:4;
    } bits;
};

void init_sensors( void );

/**
 * @brief 获取托盘状态信息
 * 
 * @param[in] id        can设备id 
 * @param[in] req       请求数据
 * @param[in] req_len   请求数据长度
 * @param[in] ret       响应返回数据
 * @param[in] ret_len   响应返回数据长度
 */
void protocol_pallet_state( uint16_t id, 
                         uint8_t *req,
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len );

#endif
