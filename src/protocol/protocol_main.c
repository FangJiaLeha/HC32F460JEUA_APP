/**
 * @file protocol_main.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-15
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
#include "includes.h"

static rt_device_t can_dev = RT_NULL;
static struct rt_can_filter_item filter_item[1];
static struct rt_can_filter_config can_filter_cfg;
static struct rt_semaphore recv_sem;
static struct rt_mutex recv_mtx;

static void request_server(uint16_t id, 
                         uint8_t *req, 
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len)
{
    *ret_len = 0;
    if( req[1] == 0x13 ) // 获取版本号
    {
        *ret_len = 8;
        ret[0] = 0x13;
        ret[1] =( PROGRAM_VERSION >> 16 ) & 0xFF; // Major，主版本号
        ret[2] = (  PROGRAM_VERSION >> 8 ) & 0xFF; // Minor, 次版本号
        ret[3] = PROGRAM_VERSION & 0xFF; // Revision，修订版本号
        ret[4] = 0;
        ret[5] = 0;
        ret[6] = get_pallet_id() + 7;
        ret[7] = BCC_CheckSum( ret, 7 );
    }
}

const struct PCMD cmds[4] = {

    { 0x00, request_server },
    // /**
    //  * 升级支持协议
    //  */
    // { 0x16, iap_control },

    /**
     * @brief 灯条控制协议
     * 
     */
    { 0x90, led_bar_control }, 
    /**
     * @brief 灯条分段控制协议
     */
    { 0xA0, led_bar_sectrl_control },
    { 0x91, protocol_pallet_state }
    // /**
    //  * @brief  动画切换协议
    //  * 
    //  */
    // { 0x92, animation_control },

    // /**
    //  * @brief 动画更新协议
    //  * 
    //  */
    // { 0x93, protocol_animation_update_entry },

    // /**
    //  * @brief 字库更新协议
    //  * 
    //  */
    // { 0x95, protocol_font_update_entry }
};

uint8_t BCC_CheckSum(const uint8_t *buf, uint8_t len)
{
	uint8_t i;
	uint8_t checksum = 0;
	for( i = 0; i < len; i++ )
	{
		checksum ^= *buf++;
	}
	return checksum;
}


static const struct PCMD *find_processor( uint16_t can_id, uint8_t cid )
{
    int i;
    uint8_t cmds_count = sizeof( cmds ) / sizeof( struct PCMD );

    for( i = 0; i < cmds_count; i++ )
    {
        if( cmds[i].cid == cid )
        {
            return &cmds[i];
        }
    }
    return NULL;
}

static void can_lock( void )
{
    rt_mutex_take( &recv_mtx, RT_WAITING_FOREVER );
}

static void can_unlock( void )
{
    rt_mutex_release( &recv_mtx );
}
uint32_t t_cnt = 0;
int touch_feedback( uint8_t touch_id, uint8_t state )
{
    struct rt_can_msg send_msg;
    t_cnt++;
    send_msg.id = 7+get_pallet_id();
    send_msg.ide = 0;
    send_msg.rtr = 0;
    send_msg.len = 8;
    rt_memset( send_msg.data, 0, 8 );
    send_msg.data[0] = 0x94;
    send_msg.data[1] = touch_id;
    send_msg.data[2] = state;
    send_msg.data[7] = BCC_CheckSum( send_msg.data, 7 );
    can_lock();
    rt_device_write( can_dev, 0, (void *)&send_msg, sizeof(send_msg) );
    can_unlock();
    return 0;
}

int pallet_feedback( uint8_t pallet_id, uint8_t stype, uint8_t state )
{
    struct rt_can_msg send_msg;

    rt_memset( (void *)&send_msg, 0, sizeof(send_msg) ); // Can single FIFO transmission. Solves the problem that can data frames sometimes fail to be sent
    send_msg.id = get_pallet_id() + 7;
    send_msg.ide = 0;
    send_msg.rtr = 0;
    send_msg.len = 8;
    //rt_memset( send_msg.data, 0, 8 );
    send_msg.data[0] = 0x91;
    send_msg.data[1] = 2;
    send_msg.data[2] = pallet_id;
    send_msg.data[3] = stype;
    send_msg.data[4] = state;
    send_msg.data[7] = BCC_CheckSum( send_msg.data, 7 );
    can_lock();
    rt_device_write( can_dev, 0, (void *)&send_msg, sizeof(send_msg) );
    can_unlock();
    dbg_log( DBG_LOG, "pallet evt, id:%d, stype:%d, state:%d\n", pallet_id, stype, state );
    return 0;
}


uint32_t can_send( uint8_t *data, uint8_t len )
{
    struct rt_can_msg send_msg;
    rt_size_t wsize;

    if( len > 8 )
    {
        len = 8;
    }

    send_msg.id = get_pallet_id() + 7;
    send_msg.ide = 0;
    send_msg.rtr = 0;
    send_msg.len = len;
    rt_memset( send_msg.data, 0, 8 );
    rt_memcpy( send_msg.data, data, len );
    can_lock();
    wsize = rt_device_write( can_dev, 0, (void *)&send_msg, sizeof(send_msg) );
    can_unlock();
    return wsize;
}

static rt_err_t can_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release( &recv_sem );
    return RT_EOK;
}

#ifdef RT_CAN_USING_HDR
static rt_err_t can_filter_cb( rt_device_t dev, void *arg, rt_int32_t hdr, rt_size_t size )
{
    //rt_device_t can_dev = ( rt_device_t )arg;
    rt_sem_release( &recv_sem );
    return RT_EOK;
}
#endif

void protocol_process_entry( void* arg )
{
    rt_uint32_t watch_set = 1;
    rt_uint32_t evt;
    rt_size_t read_size = 0;
    rt_err_t ret;
    volatile struct rt_can_msg msg, send_msg;
    const struct PCMD *cmd;
    volatile uint32_t ret_len;

    can_dev = rt_device_find( "can0" );
    if( can_dev == RT_NULL )
    {
        return ;
    }

    rt_sem_init( &recv_sem, "CanSem", 0, RT_IPC_FLAG_FIFO );
    rt_mutex_init( &recv_mtx, "CanMtx", RT_IPC_FLAG_FIFO );
    memset((void *)&send_msg, 0, sizeof(struct rt_can_msg)); // Can single FIFO transmission. Solves the problem that can data frames sometimes fail to be sent

    filter_item[0].id = get_pallet_id() + 7;
    filter_item[0].ide = 0;
    filter_item[0].rtr = 0;
    filter_item[0].mode = 1;
    filter_item[0].mask = 0x0;
    filter_item[0].hdr = -1;
#ifdef RT_CAN_USING_HDR
    filter_item[0].ind = can_filter_cb;
    filter_item[0].args = can_dev;
#endif /*RT_CAN_USING_HDR*/
    rt_device_set_rx_indicate(can_dev, can_rx_ind);
    
    can_filter_cfg.count = 1;
    can_filter_cfg.actived = 1;
    can_filter_cfg.items = filter_item;
    rt_device_open( can_dev, ( RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX ) );
    rt_device_control( can_dev, RT_CAN_CMD_SET_FILTER, &can_filter_cfg );

    rt_thread_delay( RT_TICK_PER_SECOND );
    //can_send( data, 8 );
    
    while( 1 ) 
    {
        ret = rt_sem_take( &recv_sem, RT_WAITING_FOREVER );
        if( ret == -RT_ETIMEOUT )
        {
            continue;
        }
        can_lock();
        msg.hdr = filter_item[0].hdr;
        read_size = rt_device_read( can_dev, 0, (void*)&msg, sizeof( msg ) );
        can_unlock();
        if( ( read_size != sizeof( msg ) ) || ( msg.len == 0 ) )
        {
            continue;
        }
        cmd = find_processor( msg.id, msg.data[0] );
        if( cmd == RT_NULL )
        {
            continue;
        }
        ret_len = 0;
        cmd->callback( msg.id, (uint8_t *)msg.data, msg.len, (uint8_t *)send_msg.data, (uint32_t*)&ret_len );
        if( ret_len > 0 )
        {
            send_msg.id = get_pallet_id() + 7;
            send_msg.ide = 0;
            send_msg.rtr = 0;
            send_msg.len = ret_len;
            can_lock();
            rt_device_write( can_dev, 0, (void *)&send_msg, sizeof(msg) );
            can_unlock();
        }
    }
}

int protocol_init( void )
{
    rt_thread_t tid;
    tid = rt_thread_create("protocol", protocol_process_entry, RT_NULL, 1024, RT_THREAD_PRIORITY_MAX / 3 - 3, 20 );
    if( tid != RT_NULL )
    {
        rt_thread_startup(tid);
    }
    return 0;
}
