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
 
#include <rtthread.h>
#include <led_frame.h>
#include "bsp_device.h"
#include "constant.h"
#include "led_bar.h"

struct WS2812_BAR{
    struct LED_BAR parent;
    uint32_t led_num;
    uint32_t start;
    void * (*get_disbuff)( struct WS2812_BAR *wbar );
};

const uint8_t color_table[8][3] = {
    {0, 0, 0 }, // 黑色 --> 0
    { 0x7F, 0x7F, 0x7F }, // 白色 --> 1
    { 0x7F, 0, 0 }, // 红色 --> 2
    { 0, 0x7F, 0 }, // 绿色 --> 3
    { 0, 0, 0x7F }, // 蓝色 --> 4
    { 0x7F, 0x56, 0}, //橙色 --> 5
    { 0x7F, 0x7F, 0}, // 黄色 --> 6
    {0x2F, 0x2F,0x2F }
};

static struct WS2812_BAR led_bar9;
static struct WS2812_BAR led_bar10;
static struct WS2812_BAR led_bar11;
static struct LED_BAR led_bar_pallet;
// static struct LED_BAR led_bar2;
// static struct LED_BAR led_bar3;
// static struct LED_BAR led_bar4;
// static struct LED_BAR led_bar5;
// static struct LED_BAR led_bar6;
// static struct LED_BAR led_bar7;
// static struct LED_BAR led_bar8;

static struct LED_BAR *Lbar[4] = { /*&led_bar1, &led_bar2, &led_bar3, &led_bar4, &                                       led_bar5, &led_bar6, &led_bar7, &led_bar8, */
                                 (struct LED_BAR *)&led_bar9, (struct LED_BAR *)&led_bar10,  (struct LED_BAR *)&led_bar11, &led_bar_pallet
                                };

volatile static uint8_t power_state = 1;  //!< 电源状态：1，正常运行，2，进入低功耗模式

static rt_device_t tlc59108 = RT_NULL;
struct TLC59108REGS regs = {0};
static uint8_t tlc59108_led_key_installed = 0; //!< 带3色LED按键板是否安装
// /**
//  * @brief 电源事件监听器回调处理函数
//  * 
//  * @param[in] type     事件类型 
//  * @param[in] arg_len   事件数据长度，单位字节
//  * @param[in] arg       事件数据指针
//  */
// static void power_event_listener_cb( uint16_t type, uint16_t arg_len, void* arg )
// {
//     uint8_t *data = (uint8_t *)arg;
//     struct LED_BAR *bar;
//     int i;

//     if( data == RT_NULL )
//     {
//         return;
//     }
//     if( 1 == *data )
//     {
//         // 进入低功耗模式
//         power_state = 2;
//         for( i = 0; i < 11; i++ )
//         {
//             Lbar[i]->off( Lbar[i] );
//         }
//     } else if ( 2 == *data ) {
//         power_state = 1;
//     }
// }

// /**
//  * @brief 电源事件监听器
//  * 
//  */
// static struct LISTENER power_event_listener = {
//     .cb = power_event_listener_cb,
//     .next = RT_NULL,
// };

static rt_err_t tlc59108_set_color( struct LED_BAR *bar, float *color )
{
    unsigned char buff[] = {0x82, (unsigned char)color[2], (unsigned char)color[0], (unsigned char)color[1]};
    rt_device_t dev = (rt_device_t)bar->private;

    rt_device_write( dev, 0, buff, sizeof(buff) );

    return RT_EOK;
}

static rt_err_t ws2812_bar_set_color( struct LED_BAR *bar, float *color )
{
    struct WS2812_BAR *wbar = (struct WS2812_BAR *)bar;
    struct WS2812_BAR_CTRLPACK pack;
    rt_device_t dev = (rt_device_t)bar->private;

    pack.color[0] = (uint8_t)color[0];
    pack.color[1] = (uint8_t)color[1];
    pack.color[2] = (uint8_t)color[2];

    pack.start = wbar->start;
    pack.count = wbar->led_num;
    rt_device_control( dev, WS2812_CTRL_BAR_COLOR, &pack );
    return RT_EOK;
}

static err_t init_ws2812_bar(struct WS2812_BAR *wbar, uint8_t id, 
                    rt_err_t (*set_color)( struct LED_BAR *bar, float *color ),
                    void * priv_data, uint32_t led_num, uint32_t start 
                )
{
    struct LED_BAR *bar = (struct LED_BAR *)wbar;

    if( bar == RT_NULL || set_color == RT_NULL )
    {
        return -FAILED;
    }
    init_led_bar( bar, id, set_color, priv_data );
    wbar->start = start;
    wbar->led_num = led_num;
    return OK;
}

static rt_err_t pallet_bar_set_color( struct LED_BAR *bar, float *color )
{
    rt_device_t dev = (rt_device_t)bar->private;
    float color_av = ( color[0] + color[1] + color[2] ) / 3;
    uint8_t pos = ( bar->id + 1 ) / 2;
    
    rt_device_write( dev, pos, &color_av, sizeof(color_av) );
    return RT_EOK;

    #if 0
    uint16_t val;

    val = (uint16_t)( PWM_LIGHT_START * color_av / 255 );
    //val = 7300 - (uint16_t)( 7300 * color_av / 255 );

    rt_device_write( dev, pos, &val, 2 );
    return RT_EOK;
    #endif
}

static struct LED_BAR *_get_led_bar_obj( uint8_t bar_id )
{
    uint8_t pallet_id = get_pallet_id();
    struct LED_BAR *bar = RT_NULL;

    if( pallet_id == 0 ) //第一层托盘支持头部LED灯控制
    {
        if( bar_id > 8 && bar_id < 12 )
        {
            bar = Lbar[bar_id-9];
        }
    }

    if( bar_id == ( pallet_id * 2+1 ) || bar_id == ( pallet_id * 2+2 ) )
    {
        bar = &led_bar_pallet;
    }
    return bar;
}

void led_bar_control( uint16_t id, 
                         uint8_t *req, 
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len )
{
    struct LED_BAR *bar = RT_NULL;
    float color[3];
    uint32_t arg;

    *ret_len = 0;
    if( ( req_len < 8 ) || ( req[2] > 7 ) || ( req[1] > 11 ) || (req[1] == 0 )
            /* || ( BCC_CheckSum( req, 7 ) != req[7] ) */ )
    {
        return;
    }
    
    if( get_init_state() != 0xFF || power_state == 2 )
    {
        return ;
    }

    bar = _get_led_bar_obj( req[1] );

    if( bar == RT_NULL )
    {
        return;
    }

    //bar = Lbar[ req[1] - 9 ];
    //pack.id = req[1];
    if( req[3] == 5 )
    {
        //rgb 颜色控制模式

        color[0] = (float)req[4];
        color[1] = (float)req[5];
        color[2] = (float)req[6];

        bar->on( bar, color );

    } else {

        color[0] = (float)color_table[req[2]][0];
        color[1] = (float)color_table[req[2]][1];
        color[2] = (float)color_table[req[2]][2];

        if( req[3] == LEDBAR_CTRL_MODE_OFF )
        {
            bar->off( bar );
        } else if ( req[3] == LEDBAR_CTRL_MODE_CONST_ON ) {
            bar->on( bar, color );
        } else if ( req[3] == LEDBAR_CTRL_MODE_BLINK_OFF ) {
            uint32_t intval;
            uint8_t times = req[4];

            intval = req[5];
            intval <<= 8;
            intval |= req[6];

            bar->blink_and_off( bar, color, intval, times );
        } else if ( req[3] == LEDBAR_CTRL_MODE_BLINK_ON ) {
            uint32_t intval;
            uint8_t times = req[4];

            intval = req[5];
            intval <<= 8;
            intval |= req[6];

            bar->blink_and_on( bar, color, intval, times );
        }
    }
    
}

/*灯呼吸模式控制*/
void led_bar_sectrl_control(uint16_t id, 
                         uint8_t *req, 
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len)
{
    struct LED_BAR *bar = RT_NULL;
    float color1[3], color2[3];
    uint32_t time;

    *ret_len = 0;
    // 非法数据帧
    if( ( req_len < 8 ) || ( ( req[1] & 0x0F ) == 0 ) /* || ( BCC_CheckSum( req, 7 ) != req[7] ) */ )
    {
        return;
    }
    
    if( get_init_state() != 0xFF )
    {
        return ;
    }
    
    bar = _get_led_bar_obj( req[1] >> 4 );

    if( bar == RT_NULL )
    {
        return;
    }


    time = ( ( req[1] & 0x0F ) + 1 ) * 100;
    color1[0] = req[2];
    color1[1] = req[3];
    color1[2] = req[4];

    color2[0] = req[5];
    color2[1] = req[6];
    color2[2] = req[7];

    //bar = Lbar[bar_id-9];
    bar->section_ctrl( bar, color1, color2, time );
}

// void led_bar_active( uint8_t layer )
// {
//     uint8_t bar_id = 0;
//     int cmd = 0;
//     static uint8_t bar_state = 0;

//     if( is_pallet_installed( layer ) )
//     { 
//         if( ( ( 1 << layer ) & bar_state ) == 0 )
//         {
//             cmd = DEVICE_CTRL_ACTIVE_RGBLED_BAR;
//             bar_state |= ( 1 << layer );
//         }
        
//     } else {
//         if(  ( 1 << layer ) & bar_state  )
//         {
//             cmd = DEVICE_CTRL_DISACTIVE_RGBLED_BAR;
//             bar_state &= ~( 1 << layer );
//         }
//     }
//     if( cmd != 0 )
//     {
//         bar_id =  (layer+1) * 2 - 1;
//         rt_device_control( ws_led, cmd, &bar_id );
//         bar_id =  (layer+1) * 2;
//         rt_device_control( ws_led, cmd, &bar_id );
//     }
// }

void init_led_bars( void )
{
    uint8_t bar_id = 0;
    uint8_t i;
    float color[3] = { 0x7F, 0x7F, 0x7F };
    rt_device_t ws_led_dev;
    rt_device_t pled_dev;
    uint32_t ws_led_num = 11;
    uint8_t pallet_id = get_pallet_id();
    rt_err_t ret;

    if( pallet_id == 0 )
    {
        tlc59108 = rt_device_find( "tlc59108" );
        if( tlc59108 != RT_NULL )
        {
            ret = rt_device_open( tlc59108, RT_DEVICE_OFLAG_RDWR );
            if( ret == RT_EOK )
            {
                uint8_t txBuffer[2] = {0x0C, 0x2A };
                tlc59108_led_key_installed = 1;
                rt_device_write( tlc59108, 0, &regs, 18 );
                rt_device_write( tlc59108, 0, txBuffer, 2 );
            }
        }
    } else {
        tlc59108_led_key_installed = 0;
    }
    
    ws_led_dev = rt_device_find( "ws2812" );
    
    if( ws_led_dev != RT_NULL )
    {
        rt_device_open( ws_led_dev, RT_DEVICE_OFLAG_RDWR );
        rt_device_control( ws_led_dev, WS2812_CTRL_INIT, &ws_led_num );
        init_ws2812_bar( &led_bar9, 9, ws2812_bar_set_color, ws_led_dev, 4, 0 );
        init_ws2812_bar( &led_bar10, 10, ws2812_bar_set_color, ws_led_dev, 4, 4 );
        init_ws2812_bar( &led_bar11, 11, ws2812_bar_set_color, ws_led_dev, 3, 8 );
    }

    if( tlc59108_led_key_installed ) 
    { // 如果安装了PJ0002_TF4_A04带tlc59108彩色led灯按键板，则重置11号灯条控制方法为tlc59108
        Lbar[2]->private = tlc59108;
        Lbar[2]->set_color = tlc59108_set_color;
    }

    pled_dev = rt_device_find( "PLedBar" );
    if( pled_dev != RT_NULL )
    {
        rt_device_open( pled_dev, RT_DEVICE_OFLAG_RDWR );
        init_led_bar( &led_bar_pallet, 1, pallet_bar_set_color, pled_dev );
    }

    for( i = 0; i < 3; i++ )
    {
        if( pallet_id == 0 )
        {
            Lbar[i]->on( Lbar[i], (float []){ (float)22, (float)160, (float)255} );
        } else {
            Lbar[i]->off( Lbar[i] );
        }
    }
    
    Lbar[3]->on( Lbar[3], color );
    
    //listen_event( &power_event_listener, PD_EVENT_POWER );
}

static rt_err_t heartbeat_set_color( struct LED_BAR *bar, float *color )
{
    rt_device_t dev = (rt_device_t)bar->private;
    uint8_t val = 0;

    if( color == RT_NULL )
    {
        return -RT_ERROR;
    }
    
    if( ( (uint8_t)color[0] ) > 0 )
    {
        val = 1;
    }

    rt_device_write( dev, 0, &val, 1 );
    return RT_EOK;
}

static struct LED_BAR heartbeat_led;

void set_heartbeat_intval( uint16_t time )
{
    float color[3] = { 0xFF, 0xFF, 0xFF };
    if( time == 0 )
    {
        return;
    }
    heartbeat_led.blink_and_on( &heartbeat_led,  color, time, 0 );
}

void init_heartbeat_led( void )
{
    rt_device_t _dev;

    _dev = rt_device_find( "HeartBeat" );
    if( _dev != RT_NULL )
    {
        rt_device_open( _dev, RT_DEVICE_OFLAG_RDWR );
        init_led_bar( &heartbeat_led, 0x80, heartbeat_set_color, _dev );
    }
}


// int active_led_bar( uint8_t id, uint8_t enable )
// {
//     if( enable )
//     {
//         rt_device_control( ws_led, DEVICE_CTRL_ACTIVE_RGBLED_BAR, &id );
//     } else {
//         rt_device_control( ws_led, DEVICE_CTRL_DISACTIVE_RGBLED_BAR, &id );
//     }
    
//     return 0;
// }

// FINSH_FUNCTION_EXPORT( active_led_bar, active_led_bar id enable );

int ctrl_led_bar( uint8_t id, uint8_t r, uint8_t g, uint8_t b )
{
    float color[3];
    struct LED_BAR *bar;

    if( id == 0 || id > 11 || id < 9 )
    {
        return -1;
    }

    bar = Lbar[ id - 9 ];

    color[0] = (float)r;
    color[1] = (float)g;
    color[2] = (float)b;

    bar->on( bar, color );

    return 0;
}

FINSH_FUNCTION_EXPORT( ctrl_led_bar, ctrl_led_bar id r g b );
