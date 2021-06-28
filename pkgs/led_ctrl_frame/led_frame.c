/**
 * @file led_bar.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-15
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */

#include "led_frame.h"

static rt_err_t on( struct LED_BAR *bar, float *color )
{
    rt_err_t ret;
    rt_mutex_take( &bar->lock, RT_WAITING_FOREVER );
    bar->param.ctrl_mode = LEDBAR_CTRL_MODE_CONST_ON;
    bar->param.color1[0] = color[0];
    bar->param.color1[1] = color[1];
    bar->param.color1[2] = color[2];
    ret = bar->set_color( bar, color );
    rt_mutex_release( &bar->lock );
    return ret;
}

static rt_err_t off( struct LED_BAR *bar )
{
    rt_err_t ret;
    rt_mutex_take( &bar->lock, RT_WAITING_FOREVER );
    bar->param.ctrl_mode = LEDBAR_CTRL_MODE_OFF;
    bar->param.color1[0] = 0;
    bar->param.color1[1] = 0;
    bar->param.color1[2] = 0;
    ret =  bar->set_color( bar, bar->param.color1 );
    rt_mutex_release( &bar->lock );
    return ret;
}

static rt_err_t blink_and_on( struct LED_BAR *bar, float *color, uint16_t intval, uint8_t times )
{
    if( intval < 10 )
    {
        return -RT_ERROR;
    }
    rt_mutex_take( &bar->lock, RT_WAITING_FOREVER );

    bar->param.color1[0] = color[0];
    bar->param.color1[1] = color[1];
    bar->param.color1[2] = color[2];

    bar->param.color2[0] = 0;
    bar->param.color2[1] = 0;
    bar->param.color2[2] = 0;

    bar->param.ctrl_mode = LEDBAR_CTRL_MODE_BLINK_ON;
    bar->param.times = times;
    bar->param.intval = intval;

    
    bar->param.loop_delay = intval  / 10;

    bar->state.looped = 0;
    bar->state.action_done = 0;
    {
        rt_tick_t tick = bar->param.loop_delay;
        // 开始执行，设定频次
        rt_timer_control( bar->timer, RT_TIMER_CTRL_SET_TIME, &tick );
        rt_timer_stop( bar->timer );
        rt_timer_start( bar->timer );
    }
    rt_mutex_release( &bar->lock );
    return RT_EOK;
}

static rt_err_t blink_and_off( struct LED_BAR *bar, float *color, uint16_t intval, uint8_t times )
{
    if( intval < 10 )
    {
        return -RT_ERROR;
    }
    rt_mutex_take( &bar->lock, RT_WAITING_FOREVER );

    bar->param.color1[0] = color[0];
    bar->param.color1[1] = color[1];
    bar->param.color1[2] = color[2];

    bar->param.color2[0] = 0;
    bar->param.color2[1] = 0;
    bar->param.color2[2] = 0;

    bar->param.ctrl_mode = LEDBAR_CTRL_MODE_BLINK_OFF;
    bar->param.times = times;
    bar->param.intval = intval;

    
    bar->param.loop_delay = intval  / 10;

    bar->state.looped = 0;
    bar->state.action_done = 0;
    {
        rt_tick_t tick = bar->param.loop_delay;
        // 开始执行，设定频次
        rt_timer_control( bar->timer, RT_TIMER_CTRL_SET_TIME, &tick );
        rt_timer_stop( bar->timer );
        rt_timer_start( bar->timer );
    }
    rt_mutex_release( &bar->lock );
    return RT_EOK;
}

static rt_err_t section_ctrl( struct LED_BAR *bar, float *start_color, float *end_color, uint16_t time )
{
    struct render_param *param;
    if( bar == RT_NULL || time < 100 )
    {
        return -RT_ERROR;
    }
    param = &bar->param;

    rt_mutex_take( &bar->lock, RT_WAITING_FOREVER );
    param->ctrl_mode = LEDBAR_CTRL_MODE_SECTRL;

    param->times = time / 10; 

    param->color1[0] = start_color[0];
    param->color1[1] = start_color[1];
    param->color1[2] = start_color[2];

    param->color2[0] = end_color[0];
    param->color2[1] = end_color[1];
    param->color2[2] = end_color[2];

    param->breath_step[0] = ( end_color[0] - start_color[0] ) / param->times;
    param->breath_step[1] = ( end_color[1] - start_color[1] ) / param->times;
    param->breath_step[2] = ( end_color[2] - start_color[2] ) / param->times;
    param->loop_delay = 1;
    bar->state.action_done = 0;
    bar->state.looped = 0;
    {
        rt_tick_t tick = param->loop_delay;
        // 开始执行，设定频次
        rt_timer_control( bar->timer, RT_TIMER_CTRL_SET_TIME, &tick );
        rt_timer_stop( bar->timer );
        rt_timer_start( bar->timer );
    }
    rt_mutex_release( &bar->lock );
    return RT_EOK;
}

static void render( void *arg )
{
    struct LED_BAR *bar = arg;

    if( bar == RT_NULL )
    {
        return;
    }

    rt_mutex_take( &bar->lock, RT_WAITING_FOREVER );
    if( bar->param.ctrl_mode == LEDBAR_CTRL_MODE_BLINK_OFF
        || bar->param.ctrl_mode == LEDBAR_CTRL_MODE_BLINK_ON )
    {
        if( bar->state.action_done == 0 )
        {
            if( bar->state.looped & 1 )
            {
                bar->set_color( bar, bar->param.color1 );
            } else {
                bar->set_color( bar, bar->param.color2 );
            }
            bar->state.looped++;
            if( 
                bar->param.times > 0 
                && ( bar->state.looped >> 1 ) >= bar->param.times
            ){
                rt_tick_t tick = RT_TICK_PER_SECOND * 5;
                
                if( bar->param.ctrl_mode == LEDBAR_CTRL_MODE_BLINK_OFF )
                {
                    bar->set_color( bar, bar->param.color2 );
                } else {
                    bar->set_color( bar, bar->param.color1 );
                }
                bar->state.action_done = 1; // 执行完成
                bar->param.ctrl_mode = 0;
                // 任务完成，减缓轮询频次
                rt_timer_control( bar->timer, RT_TIMER_CTRL_SET_TIME, &tick );
                rt_timer_stop( bar->timer );
                rt_timer_start( bar->timer );
            }
        }
    } else if( bar->param.ctrl_mode == LEDBAR_CTRL_MODE_SECTRL ) {
        if( bar->state.action_done == 0 )
        {
            float out[3];
            out[0] = bar->param.color1[0] 
                    + bar->param.breath_step[0] * bar->state.looped;
            out[1] = bar->param.color1[1] 
                    + bar->param.breath_step[1] * bar->state.looped;
            out[2] = bar->param.color1[2] 
                    + bar->param.breath_step[2] * bar->state.looped;
            bar->state.looped++;
            bar->set_color( bar, out );
            if( bar->state.looped >= bar->param.times )
            {
                rt_tick_t tick = RT_TICK_PER_SECOND * 5;
                
                bar->state.action_done = 1;
                bar->param.ctrl_mode = 0;
                // 任务完成，减缓轮询频次
                rt_timer_control( bar->timer, RT_TIMER_CTRL_SET_TIME, &tick );
                rt_timer_stop( bar->timer );
                rt_timer_start( bar->timer );
            }
        }
    }
    rt_mutex_release( &bar->lock );
}

rt_err_t init_led_bar( struct LED_BAR *bar, 
                    uint8_t id, 
                    rt_err_t (*set_color)( struct LED_BAR *bar, float *color ), void *priv_data  )
{
    rt_thread_t tid;
    uint8_t name[9] = {'L', 'E', 'D', 'R', 'e', 'n','d', 0, 0 };

    bar->id = id;
    bar->set_color = set_color;
    bar->on = on;
    bar->off = off;
    bar->blink_and_on = blink_and_on;
    bar->blink_and_off = blink_and_off;
    bar->section_ctrl = section_ctrl;
    //bar->render_thread = render;
    bar->private = priv_data;
    
    rt_memset( &bar->param, 0, sizeof( bar->param) + sizeof( bar->state ) );
    if( id > 9 )
    {
        name[7] = 'A' + ( id - 10 );
    } else {
        name[7] = '0' + id;
    }

    rt_mutex_init( &bar->lock, (const char *)name, RT_IPC_FLAG_FIFO );

    // tid = rt_thread_create( (const char *)name, render, bar, 512, RT_THREAD_PRIORITY_MAX / 3 - 2, 20 );
    // if( tid == RT_NULL )
    // {
    //     return -RT_ERROR;
    // }
    // rt_thread_startup( tid );
    // bar->thread_handler = tid;
    bar->timer = rt_timer_create( (const char *)name, render, 
                        bar, RT_TICK_PER_SECOND * 5, 
                        RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER );
    if( bar->timer == RT_NULL )
    {
        return -RT_ENOMEM;
    } else {
        rt_timer_start( bar->timer );
    }

    return RT_EOK;
}

