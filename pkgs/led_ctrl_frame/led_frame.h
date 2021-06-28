/**
 * @file led_frame.h
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-10-27
 * 
 * 
 */
#ifndef __LED_FRAME_H__
#define __LED_FRAME_H__
#include <stdint.h>
#include <rtthread.h>

/**
 * @brief 关闭模式
 * 
 */
#define LEDBAR_CTRL_MODE_OFF           0

/**
 * @brief 常亮显示
 * 
 */
#define LEDBAR_CTRL_MODE_CONST_ON      1

/**
 * @brief 闪烁显示
 * 
 */
#define LEDBAR_CTRL_MODE_BLINK_OFF         2


#define LEDBAR_CTRL_MODE_BLINK_ON         4

/**
 * @brief 呼吸模式
 * 
 */
#define LEDBAR_CTRL_MODE_BREATHING        3

/**
 * @brief 分段控制模式
 * 
 */
#define LEDBAR_CTRL_MODE_SECTRL        6

/**
 * @brief 灯条工作时运行各项参数
 * 
 */
struct render_param{
    uint8_t ctrl_mode;
    float color1[3];
    float color2[3];
    float breath_step[3];
    uint16_t intval;
    uint16_t times;
    //uint16_t cnt;
    uint16_t loop_delay;  //!< 渲染引擎渲染间隔时间，单位10ms
};

/**
 * @brief 灯条动画效果渲染
 * 
 */
struct render_state{
    uint8_t action_done;
    uint16_t looped; //!< 已循环的次数或运行的次数
};

struct LED_BAR{
    uint8_t id;

    struct render_param param;
    struct render_state state;
    //rt_thread_t thread_handler;
    rt_timer_t timer;

    //err_t (*init)( struct LED_BAR *bar );
    rt_err_t (*set_color) (  struct LED_BAR *bar, float *color );
    rt_err_t (*on)( struct LED_BAR *bar, float *color );
    rt_err_t (*off)( struct LED_BAR *bar );
    rt_err_t (*blink_and_on )( struct LED_BAR *bar, float *color, uint16_t intval, uint8_t times );
    rt_err_t (*blink_and_off )( struct LED_BAR *bar, float *color, uint16_t intval, uint8_t times );
    rt_err_t (*section_ctrl)( struct LED_BAR *bar, float *start_color,float *end_color, uint16_t time );
    //void (*render_thread)( void *arg );
    struct rt_mutex lock;
    void *private;
};

rt_err_t init_led_bar( struct LED_BAR *bar, 
                    uint8_t id, 
                    rt_err_t (*set_color)( struct LED_BAR *bar, float *color ), void *priv_data  );

#endif
