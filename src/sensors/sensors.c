/**
 * @file sensors.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-16
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
#include "includes.h"

/**
 * @brief 触摸传感器pcf8574设备
 * 
 */
static rt_device_t touch_dev;

/**
 * @brief 触摸传感器上一次状态
 * 
 */
static union TOUCH_STATE t_old_state;

/**
 * @brief 触摸传感器当前状态
 * 
 */
union TOUCH_STATE tstate;


/**
 * @brief 
 * 
 * @return err_t 
 */
static struct rt_i2c_bus_device* i2c_bus;


static err_t TFminiPlus_checksum( uint8_t* data )
{
    int i;
    uint8_t calcu_sum = 0;
    for( i = 0; i < 8; i++ )
    {
        calcu_sum += data[i];
    }
    if( calcu_sum != data[8] )
    {
        return -FAILED;
    }
    return OK;
}

static err_t get_TFminiPlus_data( uint8_t slave_addr, uint8_t *data )
{
    uint8_t wbuf[5] = {0x5A, 0x05, 0x00, 0x01, 0x60};
    uint8_t rbuf[9];
    rt_size_t rett;
    struct rt_i2c_msg m[2];

    if( ( RT_NULL == i2c_bus ) || ( RT_NULL == data )  )
    {
        return -FAILED;
    }

    m[0].addr = slave_addr;
    m[0].flags = 0;
    m[0].len = 5;
    m[0].buf = wbuf;

    m[1].addr = slave_addr;
    m[1].flags = RT_I2C_RD;
    m[1].len = 9;
    m[1].buf = rbuf;

    rett = rt_i2c_transfer( i2c_bus, m, 2);
    if( rett == 0 )
    {
        return -FAILED;
    }

    rbuf[0] = rbuf[1];
    if( TFminiPlus_checksum( rbuf ) != OK )
    {
        return -FAILED;
    }

    data[2] = rbuf[3];
    data[3] = rbuf[2];
    data[4] = rbuf[5];
    data[5] = rbuf[4];
    data[6] = rbuf[7];
    data[7] = rbuf[6];

    return OK;

}

void protocol_pallet_state( uint16_t id, 
                         uint8_t *req,
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len )
{
    *ret_len = 0;
    if( ( req_len < 8 )  /* || ( BCC_CheckSum( req, 7 ) != req[7] ) */ )
    {
        
        return;
    }
    
    if( get_init_state() != 0xFF )
    {
        return ;
    }
    
    if( 0 == req[1] )
    {
        get_pallet_state();
    } else if( 3 == req[1] ) { // 使能托盘检测功能，将会将托盘物品检测传感器处于低功耗状态
        if( req[2] != ( get_pallet_id() + 1 ) )
        {
            return;
        } else {
            layer_switch( req[3] );
        }
    }
}

static void process_touch_event( void )
{
    rt_device_read( touch_dev, 0, &tstate, sizeof( tstate ) );
    if( t_old_state.bits.head != tstate.bits.head ) //额头触摸
    {
        if( tstate.bits.head )
        {
            touch_feedback(4, 0 );
        } else {
            touch_feedback(4, 1 );
        }
    }

    if( t_old_state.bits.left_ear != tstate.bits.left_ear ) //左耳触摸
    {
        if( tstate.bits.left_ear )
        {
            touch_feedback(1, 0 );
        } else {
            touch_feedback(1, 1 );
        }
    }
    if( t_old_state.bits.right_ear != tstate.bits.right_ear ) //右耳触摸
    {
        if( tstate.bits.right_ear )
        {
            touch_feedback(2, 0 );
        }  else {
            touch_feedback(2, 1 );
        }
    }
    if( t_old_state.bits.key != tstate.bits.key ) //实体按键触摸
    {
        if( tstate.bits.key )
        {
            touch_feedback(3, 0 );
        } else {
            touch_feedback(3, 1 );

            // 开启振动马达振动
            //rt_timer_start( shake_motor_timer );
            //shake_motor_switcher = 1;
        }
    }
    t_old_state = tstate;
}


void sensor_process_entry( void *arg )
{
    uint32_t cnt = 0;

    //rt_device_read( touch_dev, 0, &tstate, sizeof( tstate ) );
    //t_old_state = tstate;

    while( get_init_state() != 0xFF )
    {
        // 等待初始完成
        rt_thread_delay( RT_TICK_PER_SECOND / 10 );
    }
    while(1)
    {
        
        // if( 2 == power_state && 2 == power_state_switcher ) //为低功耗模式
        // {
        //     rt_thread_delay( RT_TICK_PER_SECOND / 10 );
        //     continue;
        // } else if( 1 == power_state && 2 == power_state_switcher ) {
        //     rt_thread_delay( RT_TICK_PER_SECOND / 20 );
        //     motor_stop();
        //     power_state = power_state_switcher;
        //     continue;
        // } else if( 2 == power_state && 1 == power_state_switcher ) {
        //     power_state = power_state_switcher;
        // }
        cnt++;
        rt_thread_delay( RT_TICK_PER_SECOND / 50 );
        if( (cnt % 5 ) == 0 )
        {
            //if( ( cnt % 10 ) == 0 )
            //{
            update_pallet_state();
            //}
            //process_touch_event();
            // if( shake_motor_switcher == 1 && shake_motor_state == 0  )
            // {
            //     ret = motor_shake();
            //     if( ret == OK )
            //     {
            //         shake_motor_state = 1;
            //     }
            // } else if( shake_motor_switcher == 0 && shake_motor_state == 1 ) {
            //     ret = motor_stop();
            //     if( ret == OK )
            //     {
            //         shake_motor_state = 0;
            //     }
            // }
        }

        layer_fun_switcher();
        

//        if( get_TFminiPlus_data( 0x10, send_msg.data ) == OK )
//        {
//            TFminiPlus_feedback( 1, &send_msg );
//        }

//        if( get_TFminiPlus_data( 0x11, send_msg.data ) == OK )
//        {
//            TFminiPlus_feedback( 2, &send_msg );
//        }
        
        //rt_device_control( io_dev, DEVICE_CTRL_IO_GET_DATA, &extend_io );
        
    }
}

void touch_process_entry( void *arg )
{
    
    uint32_t cnt = 0;
    while( get_init_state() != 0xFF )
    {
        // 等待初始完成
        rt_thread_delay( RT_TICK_PER_SECOND / 10 );
    }
    while(1)
    {
        cnt++;
        rt_thread_delay( RT_TICK_PER_SECOND / 100 );
        if( (cnt % 3 ) == 0 )
        {
            process_touch_event();
        }
    }
}

void init_sensors( void )
{
    rt_thread_t tid;
    uint8_t pallet_id = get_pallet_id();

    pallet_init();

    tid = rt_thread_create("sensors", sensor_process_entry, RT_NULL, 1024, RT_THREAD_PRIORITY_MAX / 3, 20 );
    if( tid != RT_NULL )
    {
        rt_thread_startup(tid);
    }

    if( pallet_id == 0 ) // 第一层托盘连接触摸传感器
    {
        touch_dev = rt_device_find( "touch_io" );
        if( touch_dev != RT_NULL )
        {
            uint8_t data = 0x0F;
            rt_device_open( touch_dev, RT_DEVICE_OFLAG_RDWR );
            rt_device_write( touch_dev, 0, &data, 1 );
            //rt_device_read( touch_dev, 0, &t_old_state, sizeof(tstate) );
        }
        tid = rt_thread_create("touchproc", touch_process_entry, RT_NULL, 1024, RT_THREAD_PRIORITY_MAX / 3, 20 );
        if( tid != RT_NULL )
        {
            rt_thread_startup(tid);
        }
    }

}

