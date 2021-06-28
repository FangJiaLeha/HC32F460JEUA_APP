#include "ws2812.h"
#include <rtthread.h>

#ifdef BSP_USING_PWM_WS2812

#include <rtdevice.h>

#define DBG_TAG               "bsp.ws2812"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

#define RESET_BITS_COUNT            24

#ifdef WS2812_600K
#define TIMING_ONE  91
#define TIMING_ZERO 29
#else
#define TIMING_ONE  68
#define TIMING_ZERO 22
#endif


static struct WS2812_DEV *ws_dev;

static void dma_configure( const void *dma_buff, uint32_t len )
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit( DMA1_Channel7 );
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM4->CCR3);	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dma_buff;		
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						     
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA1_Channel7, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;      
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );


    DMA_ITConfig( DMA1_Channel7, DMA_IT_TC, ENABLE );

    // RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE);
    // DMA_ClearFlag( DMA2_Stream5, DMA_FLAG_TCIF5);
    // DMA_ClearFlag( DMA2_Stream5, DMA_FLAG_TEIF5 );

    // DMA_Cmd( DMA2_Stream5, DISABLE );

    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM1->CCR1);	
    // DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma_buff;		
    // DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						     
    // DMA_InitStructure.DMA_BufferSize = len;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							
    // DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    
    // DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    // DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full ;
    // DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    // DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    // DMA_InitStructure.DMA_Channel = DMA_Channel_6;
    
    // //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    // DMA_Init( DMA2_Stream5, &DMA_InitStructure );

    // NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;      
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init( &NVIC_InitStructure );

    // DMA_ClearFlag( DMA2_Stream5, DMA_FLAG_TCIF5 );
    // DMA_ITConfig( DMA2_Stream5, DMA_IT_TC, ENABLE );

}

static rt_err_t _update_led_data( struct WS2812_DEV *ws_dev, uint32_t pos, uint32_t count )
{
    uint8_t i;
    uint8_t j;
    uint8_t tmp;
    uint16_t memaddr;
    uint16_t idx = 0;
    uint32_t dma_buff_len;
    uint32_t watch_set = 1;
    uint32_t evt = 0;
    rt_err_t ret;
    
    memaddr = 0;				//reset buffer memory index

    if( pos >= ws_dev->led_num )
    {
        return -RT_EINVAL;
    }
   
    if( ( pos + count ) > ws_dev->led_num )
    {
        count = ws_dev->led_num - pos;
    }

    memaddr = pos * 24;
    dma_buff_len = ws_dev->led_num * 24 + RESET_BITS_COUNT;

    for(j = pos; j < ( pos +count ); j++ )
    {
        idx = j; //单个灯珠编号
        // 写入red值
        for( i = 0; i < 8; i++)
        {
            ws_dev->dma_buff[memaddr] = ( ( ws_dev->render_buff[ idx ][ 1 ] << i ) & 0x0080 ) ? TIMING_ONE : TIMING_ZERO;
            memaddr++;
        }
        // 写入green值
        for( i = 0; i < 8; i++ )
        {
            ws_dev->dma_buff[memaddr] = ( (ws_dev->render_buff[ idx ][ 0 ] << i ) & 0x0080 ) ? TIMING_ONE :TIMING_ZERO;
            memaddr++;
        }
        // 写入blue值
        for( i = 0; i < 8; i++)
        {
            ws_dev->dma_buff[memaddr] = ( ( ws_dev->render_buff[ idx ][ 2 ] << i ) & 0x0080 ) ? TIMING_ONE : TIMING_ZERO;
            memaddr++;
        }
    }

    while( memaddr < dma_buff_len )
    {
        ws_dev->dma_buff[ memaddr ] = 0;
        //LED_DMA_Buffer[ memaddr ] = TIMING_ZERO;
        memaddr++;
    }
    TIM4->CNT = 300; // 此值一定要大于pwm周期值，否则第一个bit数据丢失，即整体数据向左移动一位；
    DMA_SetCurrDataCounter( DMA1_Channel7, dma_buff_len ); 	
    DMA_Cmd( DMA1_Channel7, ENABLE); 			                   
    TIM_Cmd( TIM4, ENABLE); 
    ret = rt_event_recv( &ws_dev->dma_evt, watch_set, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 1 * RT_TICK_PER_SECOND, &evt );
    if( ret == -RT_ETIMEOUT )
    {
        return -RT_EIO;
    }
    return RT_EOK;
}

void DMA1_Channel7_IRQHandler( void )
{
    if ( DMA_GetITStatus( DMA1_IT_TC7 ) != RESET )  
    {
        DMA_Cmd( DMA1_Channel7, DISABLE );    	               
        DMA_ClearITPendingBit( DMA1_IT_TC7 );
        TIM_Cmd( TIM4, DISABLE );
        //DelayNus(6);
        rt_event_send( &ws_dev->dma_evt, 1 );
    }
    
    if ( DMA_GetITStatus( DMA1_IT_TE7 ) != RESET ) {
        DMA_ClearITPendingBit( DMA1_IT_TE7 );
    }
    
    if ( DMA_GetITStatus( DMA1_IT_GL7 ) != RESET ) {
        DMA_ClearITPendingBit( DMA1_IT_GL7 );
    }
    
}

static rt_err_t init_ws2812_hardware( struct WS2812_DEV *ws_dev )
{
    rt_err_t ret = RT_EOK;

    if (ws_dev->pwm_dev != RT_NULL)
    {
        ret = rt_pwm_set(ws_dev->pwm_dev, ws_dev->pwm_channel, 1250, 0);
        rt_pwm_enable(ws_dev->pwm_dev, ws_dev->pwm_channel);
    }
    else
    {
        ret = RT_ERROR;
    }
    
    return ret;
}

static void update_timeout( void *arg )
{
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
    rt_event_init( &ws_dev->dma_evt, "ws2812RD", RT_IPC_FLAG_FIFO );
    ws_dev->update_timer = rt_timer_create( "WSUpdate", update_timeout, RT_NULL, 1 , 
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
    uint8_t led_count = 0;
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
        if( ws_dev->dma_buff != RT_NULL )
        {
            rt_free( ws_dev->dma_buff );
            ws_dev->dma_buff = RT_NULL;
        }
        if( ws_dev->render_buff != RT_NULL )
        {
            rt_free( ws_dev->render_buff );
            ws_dev->render_buff = RT_NULL;
        }
        ws_dev->dma_buff = rt_malloc( ( (*led_num) * 24 + RESET_BITS_COUNT ) * 2 );
        if( ws_dev->dma_buff == RT_NULL )
        {
            rt_mutex_release( &ws_dev->lock );
            return -RT_ENOMEM;
        }
        ws_dev->render_buff = (uint8_t (*)[3])rt_malloc( ( *led_num ) * 3  );
        if( ws_dev->render_buff == RT_NULL )
        {
            rt_free( ws_dev->dma_buff );
            ws_dev->dma_buff = RT_NULL;
            rt_mutex_release( &ws_dev->lock );
            return -RT_ENOMEM;
        }
        ws_dev->led_num = *led_num;
        dma_configure( ws_dev->dma_buff,  ws_dev->led_num * 3 + RESET_BITS_COUNT );
        rt_memset( ws_dev->render_buff, 0, ( *led_num ) * 3 );
        //_reset_dma_buff( ws_dev );
        _update_led_data( ws_dev, 0, ws_dev->led_num );
        rt_mutex_release( &ws_dev->lock );

    } else if ( cmd == WS2812_CTRL_GET_DISBUFF ) {
        uint8_t (**dis_buff)[3] = arg;
        *dis_buff = ws_dev->render_buff;
    } else if( cmd == WS2812_CTRL_UPDATE_DEVDATA ) {
        rt_mutex_take( &ws_dev->lock, RT_WAITING_FOREVER );
        //_update_led_data( ws_dev, 0, ws_dev->led_num );
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
        //_update_led_data( ws_dev, pack->start, count );
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
                         const char *pwm_name
                         const int  pwm_channel)
{
    rt_device_t dev = &wsdev->parent;

    wsdev->pwm_dev = (struct rt_device_pwm *)rt_device_find(pwm_name);
    if (wsdev->pwm_dev == RT_NULL)
    {
        LOG_E("can't find %s device", pwm_name);
        return RT_ERROR;
    }
    wsdev->pwm_channel = pwm_channel;

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
