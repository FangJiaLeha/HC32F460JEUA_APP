#include "ws2812.h"
#include <rtthread.h>

#ifdef BSP_USING_PWM_WS2812

#include <rtdevice.h>

//#include <drivers/rt_drv_onewire.h>
#include "rt_drv_onewire.h"

#define DBG_TAG               "bsp.ws2812"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_ERROR
#endif
#include <rtdbg.h>


#ifdef WS2812_600K
#define TIMING_WIDTH          1666
#define RESET_BITS_COUNT      ((60000 + TIMING_WIDTH - 1) / TIMING_WIDTH)
#define TIMING_ONE  1133
#define TIMING_ZERO 533
#else
#define TIMING_WIDTH          1250
#define RESET_BITS_COUNT      ((60000 + TIMING_WIDTH - 1) / TIMING_WIDTH)
#define TIMING_ONE  850
#define TIMING_ZERO 400
//#define TIMING_ONE  958
//#define TIMING_ZERO 319
#endif

static rt_err_t _update_led_data( struct WS2812_DEV *ws_dev, uint32_t pos, uint32_t count )
{
    uint8_t i;
    uint8_t j;
    uint8_t tmp;
    uint16_t idx = 0;
    rt_err_t result = RT_EOK;
    struct rt_device_onewire *device = ws_dev->onewire_dev;
    uint16_t *pulse_buff;
    uint32_t pulse_size;

    if( pos >= ws_dev->led_num )
    {
        return -RT_EINVAL;
    }
   
    if( ( pos + count ) > ws_dev->led_num )
    {
        count = ws_dev->led_num - pos;
    }

    pulse_size = RESET_BITS_COUNT + ws_dev->led_num * 3 * 8;
    pulse_buff = (uint16_t *)rt_malloc(sizeof(uint16_t) * pulse_size);
    rt_memset(pulse_buff, 0x00, sizeof(uint16_t) * pulse_size);
    LOG_I("color[%d~%d]:", pos, pos + count);
    for (idx = 0; idx < ws_dev->led_num; idx++)
    {
        LOG_I("r-%d,g-%d,b-%d", ws_dev->render_buff[idx][0],
                                ws_dev->render_buff[idx][1],
                                ws_dev->render_buff[idx][2]);
        //GRB <==> RGB
        for (j = 0; j < 8; j++)
        {
            if (ws_dev->render_buff[idx][1] & (1U << (7 - j)))
            {
                pulse_buff[RESET_BITS_COUNT + (3*idx + 0)*8 + j] = TIMING_ONE;
            }
            else {
                pulse_buff[RESET_BITS_COUNT + (3*idx + 0)*8 + j] = TIMING_ZERO;
            }
        }
        for (j = 0; j < 8; j++)
        {
            if (ws_dev->render_buff[idx][0] & (1U << (7 - j)))
            {
                pulse_buff[RESET_BITS_COUNT + (3*idx + 1)*8 + j] = TIMING_ONE;
            }
            else {
                pulse_buff[RESET_BITS_COUNT + (3*idx + 1)*8 + j] = TIMING_ZERO;
            }
        }
        for (j = 0; j < 8; j++)
        {
            if (ws_dev->render_buff[idx][2] & (1U << (7 - j)))
            {
                pulse_buff[RESET_BITS_COUNT + (3*idx + 2)*8 + j] = TIMING_ONE;
            }
            else {
                pulse_buff[RESET_BITS_COUNT + (3*idx + 2)*8 + j] = TIMING_ZERO;
            }
        }
    }
    result = rt_device_write(&device->parent, 0, pulse_buff, pulse_size);
    rt_free(pulse_buff);

    return result;
}

static rt_err_t init_ws2812_hardware( struct WS2812_DEV *ws_dev )
{
    rt_err_t result = RT_EOK;
    struct rt_device_onewire *device = ws_dev->onewire_dev;
    struct rt_pwm_configuration configuration = {0};
    #if 0
    uint16_t *pulse_buff;
    uint32_t pulse_size;
    uint8_t res_num;
    uint32_t i;
    uint8_t  j;
    static const uint8_t buff[] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
    static const uint8_t size = sizeof(buff);
    #endif

    if (!device)
    {
        return -RT_EIO;
    }

    result = rt_device_open(&device->parent, RT_DEVICE_FLAG_WRONLY);
    if (result != RT_EOK)
    {
        return result;
    }

    configuration.channel = ws_dev->onewire_channel;
    configuration.period = 1250;
    configuration.pulse  = 0;
    result = rt_device_control(&device->parent, ONEWIRE_CMD_SET, &configuration);    
    if (result != RT_EOK)
    {
        return result;
    }

    #if 0
    res_num = ((60000 + 1250 - 1) / 1250);
    pulse_size = res_num + 8*size;
    pulse_buff = (uint16_t *)rt_malloc(sizeof(uint16_t) * pulse_size);
    rt_memset(pulse_buff, 0x00, sizeof(uint16_t) * pulse_size);
    for (i = 0; i < size; i++)
    {
        for (j = 0; j < 8; j++)
        {
            if (buff[i] & (1U << (7 - j)))
            {
                pulse_buff[res_num + i*8 + j] = 850;
            }
            else {
                pulse_buff[res_num + i*8 + j] = 400;
            }
        }
    }
    result = rt_device_write(&device->parent, 0, pulse_buff, pulse_size);
    rt_free(pulse_buff);
    #endif

    return result;
}

static void update_timeout( void *arg )
{
    struct WS2812_DEV *ws_dev = (struct WS2812_DEV *)arg;

    if ( !ws_dev->need_update )
    {
        return;
    }
    rt_mutex_take( &ws_dev->lock, RT_WAITING_FOREVER );
    _update_led_data( ws_dev, 0, ws_dev->led_num );
    ws_dev->need_update = 0;
    rt_mutex_release( &ws_dev->lock );
}

static rt_err_t _init( rt_device_t dev )
{
    struct WS2812_DEV *ws_dev = (struct WS2812_DEV *)dev;
    rt_thread_t tid;

    if ( rt_mutex_init( &ws_dev->lock, ws_dev->parent.parent.name, RT_IPC_FLAG_FIFO  ) != RT_EOK )
    {
        return -RT_ENOSYS;
    }

    ws_dev->update_timer = rt_timer_create( "WSUpdate", update_timeout, ws_dev, RT_TICK_PER_SECOND / 20 , 
                                RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER );
    if( ws_dev->update_timer != RT_NULL )
    {
        rt_timer_start( ws_dev->update_timer );
    }
    init_ws2812_hardware( ws_dev );
    //rt_memset( LED_DMA_Buffer, 0, sizeof(LED_DMA_Buffer) );
    //rt_memset( display_buffer, 0, sizeof( display_buffer ) );
    //rt_memset( render_buffer, 0, sizeof( render_buffer ) );

    //tid = rt_thread_create( "RendENG", led_bar_render, ws_dev, 1024, RT_THREAD_PRIORITY_MAX / 3 - 2, 20 );

//    if( tid == RT_NULL )
//    {
//        return -RT_ERROR;
//    }
//    rt_thread_startup( tid );
    return RT_EOK;
}

static rt_size_t _write( rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct WS2812_DEV *ws_dev = (struct WS2812_DEV *)dev;
    uint8_t (*dis_buff)[3] = (uint8_t (*)[3])buffer;
    int i;

    if( pos >= ws_dev->led_num || size  == 0 )
    {
        return 0;
    }

    if( ( pos + size ) > ws_dev->led_num )
    {
        size = ws_dev->led_num - pos;
    }

    rt_mutex_take( &ws_dev->lock, RT_WAITING_FOREVER );
    for( i = pos; i < ( pos + size ); i++ )
    {
        ws_dev->render_buff[i][0] = dis_buff[i-pos][0];
        ws_dev->render_buff[i][1] = dis_buff[i-pos][1];
        ws_dev->render_buff[i][2] = dis_buff[i-pos][2];
    }
    _update_led_data( ws_dev, pos, size );
    rt_mutex_release( &ws_dev->lock );
    return size;
}

static rt_err_t _control( rt_device_t dev, int cmd, void *arg )
{
    struct WS2812_DEV *ws_dev = (struct WS2812_DEV *)dev;
    uint8_t bar_buff_idx;
    uint8_t led_count   = 0;
    uint8_t led_bar_idx = 0;
    int i = 0;
    uint32_t *led_num = (uint32_t *)arg;

    if( cmd == WS2812_CTRL_INIT )
    {
        if( led_num == RT_NULL || *led_num == 0 )
        {
            return -RT_EINVAL;
        }
        rt_mutex_take( &ws_dev->lock, RT_WAITING_FOREVER );
        if( ws_dev->render_buff != RT_NULL )
        {
            rt_free( ws_dev->render_buff );
            ws_dev->render_buff = RT_NULL;
        }
        ws_dev->render_buff = (uint8_t (*)[3])rt_malloc( ( *led_num ) * 3  );
        if( ws_dev->render_buff == RT_NULL )
        {
            rt_mutex_release( &ws_dev->lock );
            return -RT_ENOMEM;
        }
        ws_dev->led_num = *led_num;
        rt_memset( ws_dev->render_buff, 0, ( *led_num ) * 3 );
        _update_led_data( ws_dev, 0, ws_dev->led_num );
        rt_mutex_release( &ws_dev->lock );

    } else if ( cmd == WS2812_CTRL_GET_DISBUFF ) {
        uint8_t (**dis_buff)[3] = arg;
        *dis_buff = ws_dev->render_buff;
    } else if( cmd == WS2812_CTRL_UPDATE_DEVDATA ) {
        rt_mutex_take( &ws_dev->lock, RT_WAITING_FOREVER );
//        _update_led_data( ws_dev, 0, ws_dev->led_num );
        ws_dev->need_update = 1;
        rt_mutex_release( &ws_dev->lock );
    } else if( cmd == WS2812_CTRL_BAR_COLOR ) {
        struct WS2812_BAR_CTRLPACK *pack = arg;
        uint32_t count;
        uint32_t i;
        
        if( pack->count == 0 || pack->start >= ws_dev->led_num )
        {
            return -RT_EINVAL;
        }
        count = pack->count;
        if( pack->start + pack->count > ws_dev->led_num )
        {
            count = ws_dev->led_num - pack->start;
        }
        rt_mutex_take( &ws_dev->lock, RT_WAITING_FOREVER );
        for( i = pack->start; i < ( count + pack->start); i++ )
        {
            ws_dev->render_buff[i][0] = pack->color[0];
            ws_dev->render_buff[i][1] = pack->color[1];
            ws_dev->render_buff[i][2] = pack->color[2];
        }
//        _update_led_data( ws_dev, pack->start, count );
        ws_dev->need_update = 1;
        rt_mutex_release( &ws_dev->lock );
    } 

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops ws2812_ops =
{
    .init    = _init,
    .open    = RT_NULL,
    .close   = RT_NULL,
    .read    = RT_NULL,
    .write   = _write,
    .control = _control,
};
#endif

rt_err_t ws2812_register(struct WS2812_DEV *wsdev, 
                         const char *dev_name,
                         const char *pwm_name,
                         const int  onewire_channel)
{
    rt_device_t dev = &wsdev->parent;

    wsdev->onewire_dev = (struct rt_device_onewire *)rt_device_find(pwm_name);
    if (wsdev->onewire_dev == RT_NULL)
    {
        LOG_E("can't find %s device", pwm_name);
        return RT_ERROR;
    }

    wsdev->onewire_channel = onewire_channel;

    dev->type        = RT_Device_Class_Char;
    dev->rx_indicate = RT_NULL;
    dev->tx_complete = RT_NULL;
#ifdef RT_USING_DEVICE_OPS
    dev->ops         = &ws2812_ops;
#else                
    dev->init        = _init;
    dev->open        = RT_NULL;
    dev->close       = RT_NULL;
    dev->read        = RT_NULL;
    dev->write       = _write;
    dev->control     = _control;
#endif

    return rt_device_register(dev, dev_name, RT_DEVICE_FLAG_RDWR);
}

static int ws2812_device_init(void)
{
    static struct WS2812_DEV ws2812_dev;
    rt_err_t ret;

    ret = ws2812_register(&ws2812_dev, BSP_PWM_WS2812_DEV, BSP_PWM_WS2812_PWMX, BSP_PWM_WS2812_PWM_CHANNEL);
    if (RT_EOK != ret)
    {
        LOG_E("init ws2812 device failed");
        return -1;
    }
    return 0;
}
INIT_DEVICE_EXPORT(ws2812_device_init);

#endif /* BSP_USING_PWM_WS2812 */
