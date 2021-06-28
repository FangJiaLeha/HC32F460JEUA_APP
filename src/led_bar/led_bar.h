/**
 * @file led_bar.h
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-15
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
#ifndef __LED_BAR_H__
#define __LED_BAR_H__
#include <stdint.h>

#define COLOR_BLACK_IDX         0
#define COLOR_WHITE_IDX         1
#define COLOR_RED_IDX           2
#define COLOR_GREEN_IDX         3
#define COLOR_BLUE_IDX          4
#define COLOR_ORANGE_IDX        5
#define COLOR_YELLOW_IDX        6

#define PWM_LIGHT_START         (2880 - 1)

#define PWM_LIGHT_SHUTDOWN      (2880 - 1)

void led_bar_control(uint16_t id, 
                         uint8_t *req, 
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len);

/**
 * @brief led分段式控制协议
 * 
 * @param[in] id 
 * @param[in] req 
 * @param[in] req_len 
 * @param[in] ret 
 * @param[in] ret_len 
 */
void led_bar_sectrl_control(uint16_t id, 
                         uint8_t *req, 
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len);

void init_led_bars( void );

void set_heartbeat_intval( uint16_t time );

void init_heartbeat_led( void );

//void led_bar_active( uint8_t layer );

void led_bars_power_on( void );

void led_bars_power_off( void );

#endif
