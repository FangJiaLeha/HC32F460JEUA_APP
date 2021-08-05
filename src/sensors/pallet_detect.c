/**
 * @file pallet_detect.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-09
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
 #define DBG_SECTION_NAME    "[pallet]"
#include "includes.h"

#define VL53L0X_DISTANCE_THRESHOLD      380

#define _CONSOLE_COLOR(n)       rt_kprintf("\033["#n"m")

uint8_t vl53l0x_cali_state = 0;    //!< vl53l0x????,0:???????,1:???????
uint8_t i2c_bus_used = 0;  //!< i2c ??????,0,????,1:???;


struct PALLET_INFO{
    //rt_device_t pcf_dev;
    //uint16_t addr;
    uint8_t placed;
    uint8_t installed;
    uint16_t distance[4];
    rt_device_t vl53l0xdev[4];

    /**
     * @brief ????????, 
     * -1,???????, 
     *  1:??????????, 
     *  0:??????????
     */
    int8_t layer_switcher;
    uint8_t layer_enabled;
};


static struct PALLET_INFO pallet = { /*0x26,*/  0, 0, { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },  { 0, 0, 0, 0 }, -1, 0 };

struct vl53l0x_calibration vl53l0x_cali[4];

void set_vl53l0x_cali_state( uint8_t state )
{
    vl53l0x_cali_state = state;
}

uint8_t is_vl53l0x_cali( void )
{
    return ( vl53l0x_cali_state == 0 )? 0:1;
}

void set_i2c_bus_state( uint8_t state )
{
    i2c_bus_used = state;
}

uint8_t is_i2c_bus_used( void )
{
    return (i2c_bus_used == 0)? 0:1;
}

// uint8_t is_pallet_installed( void )
// {
//     return pallet.installed;
// }

int pallet_state( void )
{
    int i,m;
    _CONSOLE_COLOR(33);
    for( i=0; i < 4; i++ )
    {
        rt_kprintf( "layer:%d\n", i+1 );
        rt_kprintf( "pallet installed:%d, placed:%d\n", pallet.installed, pallet.placed );
        rt_kprintf( "    sensor distance:\n");
        for( m = 0; m < 4; m++ )
        {
            rt_kprintf( "        sensor:%d distance:%d mm\n", i+1, pallet.distance[m]);
        }
    }
    rt_kprintf( "\n");
    _CONSOLE_COLOR(0);
    return 0;
}

FINSH_FUNCTION_EXPORT( pallet_state, get pallet status.)

int vl53_offsetcali( uint8_t layer, uint8_t pos )
{
    rt_err_t ret;
    dbg_log( DBG_INFO, "waiting for i2c bus free...\n");
    set_vl53l0x_cali_state( 1 );
    while( is_i2c_bus_used() );
    dbg_log( DBG_INFO, "i2c free now!\n");
    ret = rt_device_control( pallet.vl53l0xdev[pos], VL53L0X_CTRL_OFFSET_CALIBRATION, &vl53l0x_cali[pos] );
    if( ret == RT_EOK )
    {
        dbg_log( DBG_INFO, "offset calibration success, please do cross talk calibration next!\n");
    } else {
        dbg_log( DBG_INFO, "offset calibration failed!\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT( vl53_offsetcali, vl53_offsetcali: layer pos.)

int vl53_crosscacli( uint8_t layer, uint8_t pos )
{
    rt_err_t ret;
    ret = rt_device_control( pallet.vl53l0xdev[pos], VL53L0X_CTRL_CROSSTALK_CALIBRATION, &vl53l0x_cali[pos] );
    if( ret == RT_EOK )
    {
        //save_vl53l0x_calibration( &vl53l0x_cali[0], sizeof(vl53l0x_cali) );
        dbg_log( DBG_INFO, "cross talk calibration success!\n");
    } else {
        dbg_log( DBG_INFO, "cross talk calibration failed!\n");
    }
    set_vl53l0x_cali_state( 0 );
    //save_vl53l0x_calibration( &vl53l0x_cali[0][0], sizeof(vl53l0x_cali) );
    return 0;
}
FINSH_FUNCTION_EXPORT( vl53_crosscacli, vl53_crosscacli: layer pos.)

uint16_t min_dis[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
volatile uint8_t trigged_count = 0;

static err_t update_layer_vl53l0x_distance( void )
{
    int i;
    rt_err_t ret;
    uint8_t placed = 0;
    uint16_t distance_threshold = VL53L0X_DISTANCE_THRESHOLD;

    if( get_pallet_id() == 0 ) //??????????????
    {
        distance_threshold = 300;
    }
	
    if( pallet.layer_enabled == 0 )
    {
        for( i = 0; i < 4; i++ )
        {
            pallet.distance[i] = 0xFFFF;
        }
        return -RT_ENOSYS;
    }

    for( i = 0; i < 4; i++ )
    {
        if( pallet.vl53l0xdev[i] == RT_NULL )
        {
            continue;
        }
        
        ret = rt_device_control( pallet.vl53l0xdev[i], 
                                    VL53L0X_CTRL_MEASUREMENT, 
                                    &pallet.distance[i] );
        if ( ret == RT_EOK )
        {
            if( pallet.distance[i] < min_dis[i] )
            {
                min_dis[i] = pallet.distance[i];
            }
        }
        if( ( ret == RT_EOK) 
            && ( pallet.distance[i] != 0xFFFF )
            && ( pallet.distance[i] < distance_threshold ) 
						&& ( pallet.distance[i] != 0 )
				){
            placed = 1;
        } else if( ret != RT_EOK ) {
            pallet.distance[i] = 0xFFFF;
        }
    }
    
    if( ( placed == 1 && pallet.placed == 0 ) 
        || ( placed == 0 && pallet.placed == 1 ) 
    ){
        trigged_count++;
    } else {
        trigged_count = 0;
    }
    
    if( pallet.placed == 0 && placed == 1 && trigged_count > 5 )
    {
        pallet.placed = placed;
        trigged_count = 0;
        pallet_feedback( get_pallet_id() + 1, 2, 1 );
    } else if( pallet.placed == 1 && placed == 0 && trigged_count > 5 ) {
        pallet.placed = placed;
        pallet_feedback( get_pallet_id() + 1, 2, 0 );
        trigged_count = 0;
    }
    
    return OK;
}

void get_pallet_state( void )
{
    uint8_t i;

//    *state = ( pallet.installed << 7 ) | ( pallet.placed & 1 ) 
//                | ( pallet.layer_enabled << 6 );
    pallet_feedback( get_pallet_id() + 1, 1, pallet.installed );
    pallet_feedback( get_pallet_id() + 1, 2, pallet.placed );
    pallet_feedback( get_pallet_id() + 1, 3, pallet.layer_enabled );
}

err_t update_pallet_state( void )
{
    int i;
    uint8_t read_data;
    rt_size_t ret;
    if( is_vl53l0x_cali() ) 
    {  //vl53l0x ????????i2c????
        return RT_EOK;
    }
    set_i2c_bus_state( 1 );
    //pallet.installed = is_pallet_installed();
    

    if( is_pallet_installed() )
    {
        if( pallet.installed == 0 )
        {
            pallet.installed = 1;
            pallet_feedback( get_pallet_id() + 1, 1, 1 );
        }
        update_layer_vl53l0x_distance( );
    } else {
        if(  pallet.installed == 1 )
        {
            pallet.installed = 0;
            pallet_feedback( get_pallet_id() + 1, 1, 0 );
        }
        rt_memset( pallet.distance, 0xFF, sizeof(pallet.distance) );
    }
    set_i2c_bus_state( 0 );
    return OK;
}

void layer_switch( uint8_t val )
{
    rt_base_t base;
    
    if( val != LAYER_DISABLED && val != LAYER_ENABLED )
    {
        return;
    }
    
    base = rt_hw_interrupt_disable();
    pallet.layer_switcher = val;
    rt_hw_interrupt_enable(base);
}

void layer_fun_switcher( void )
{
    int8_t i, sw;
    rt_base_t base;
    err_t res;

    base = rt_hw_interrupt_disable();
    sw = pallet.layer_switcher;
    if( sw != -1 )
    { 
        pallet.layer_switcher = -1;
    }
    rt_hw_interrupt_enable(base);
    if( sw == LAYER_SW_NONE ) // ????
    {
        
    } else if( sw == LAYER_SW_ON && IS_LAYER_ENABLED( &pallet) ) {
        pallet_feedback( get_pallet_id() + 1, 3, LAYER_ENABLED );
    } else if ( ( sw == LAYER_SW_OFF ) 
                && ( !IS_LAYER_ENABLED( &pallet )) ) {
        pallet_feedback( get_pallet_id() + 1, 3, LAYER_DISABLED );
    } else if( sw == LAYER_SW_ON ) {
         res = init_layer_vl53l0x_dev( );
         if( res == OK )
         {
             pallet_feedback( get_pallet_id() + 1, 3, LAYER_ENABLED );
         } else {
             pallet_feedback( get_pallet_id() + 1, 3, LAYER_DISABLED );
         }

    } else if ( sw == LAYER_SW_OFF ) {
        deinit_layer_vl53l0x_dev( );
        pallet_feedback( get_pallet_id() + 1, 3, LAYER_DISABLED );
    }
}

err_t deinit_layer_vl53l0x_dev( void )
{
    int i;

    for( i = 0; i < 4; i++ )
    {
        rt_device_control( pallet.vl53l0xdev[i], 
                            VL53L0X_CTRL_DEINIT_DEV, RT_NULL );
        pallet.layer_enabled = LAYER_DISABLED;
    }
    return OK;
}

err_t init_layer_vl53l0x_dev( void )
{
    int i;
    char name[RT_NAME_MAX] = "vl53l0x";//{  'v','l','5','3','l','0','x', 0, '\0' };
    uint8_t pos = 0;
    uint8_t success_count = 0;
    rt_err_t ret;
    for( i = 0; i < 4; i++ )
    {
        if( pallet.vl53l0xdev[i] == RT_NULL )
        {
            name[7] = '0' + i;
          
            pallet.vl53l0xdev[i] = rt_device_find( name );
        }
        
        if( pallet.vl53l0xdev[i] != RT_NULL )
        {
            rt_thread_delay(4);
            if( ( pallet.vl53l0xdev[i]->open_flag & RT_DEVICE_OFLAG_OPEN ) 
                != RT_DEVICE_OFLAG_OPEN )
            {
                rt_device_open( pallet.vl53l0xdev[i], RT_DEVICE_FLAG_RDWR );
                if( pos == 0 ) // 二次打开，解决部分需要二次打开才能成功的问题
                {
                    rt_thread_delay(1);
                    rt_device_open( pallet.vl53l0xdev[i], RT_DEVICE_FLAG_RDWR );
                }
            }
            
            ret = rt_device_control( pallet.vl53l0xdev[i], VL53L0X_CTRL_INIT_DEV, &vl53l0x_cali[i] );
            if( ret != RT_EOK )
            {
                pallet.layer_enabled = LAYER_DISABLED;
            } else {
                pallet.layer_enabled = LAYER_ENABLED;
                success_count++;
            }
        }
        pos++;
    }
//    if( success_count < 3 )
    if( success_count < 2 )
    {
        pallet.layer_enabled = LAYER_DISABLED;
        return -FAILED;
    }
    pallet.layer_enabled = LAYER_ENABLED;
    return OK;
}

err_t pallet_init( void )
{
    uint8_t read_data;
    rt_size_t ret;
    int i;

    set_i2c_bus_state( 1 );

    //get_vl53l0x_calibration( (void*)&vl53l0x_cali[0], sizeof(vl53l0x_cali) );

    pallet.installed = is_pallet_installed();

    init_layer_vl53l0x_dev( );
    
    set_i2c_bus_state( 0 );

    return RT_EOK;
}

// err_t pallet_install_check_init( void )
// {
//     //rt_device_t dev;
//     uint8_t read_data;
//     rt_size_t ret;
//     int i;
//     char name[8] = { 'p', 'f', '5', '7', '4', '_', 0, 0 };

//     set_i2c_bus_state( 1 );
//     for( i = 0; i < 4; i++ )
//     {
//         name[6] = '1' + i;
//         pallet[i].pcfdev = rt_device_find( name );
//         if( pallet[i].pcfdev != RT_NULL )
//         {
//             rt_device_open( pallet[i].pcfdev, RT_DEVICE_FLAG_RDWR );
//             rt_thread_delay(1);
//             ret = rt_device_write( pallet[i].pcfdev, 0, (void *)&pallet[i].pcf_data, 1 );
//             if( i == 0 )
//             {
//                 ret = rt_device_write( pallet[i].pcfdev, 0, (void *)&pallet[i].pcf_data, 1 );
//             }
//             if( ret == 0 )
//             {
//                 pallet[i].pcf_connect_state = 0;
//             } else {
//                 pallet[i].pcf_connect_state = 1;
                
//                 ret = rt_device_read( pallet[i].pcfdev, 0, &read_data, 1 );
//                 if( ret > 0 && ( read_data & 0x80 ) == 0 )
//                 {
//                     pallet[i].installed = 1;
//                 } else {
//                     pallet[i].installed = 0;
//                 }
//             }
//         }
//     }
//     set_i2c_bus_state( 0 );
    
//     //init_a_vl53l0x();

//     return RT_EOK;
// }


