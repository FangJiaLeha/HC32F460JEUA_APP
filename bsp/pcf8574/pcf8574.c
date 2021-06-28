/**
 * @file pcf8574.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-08
 * 
 */
 
#include "pcf8574.h"
#include <rtthread.h>

#ifdef BSP_USING_I2C_PCF8574

#include <rtdevice.h>

#define DBG_TAG               "bsp.pcf8574"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

static rt_err_t pcf8574_open( rt_device_t dev, rt_uint16_t oflag )
{
    struct PCF8574_DEV *pcf_dev = (struct PCF8574_DEV *)dev;
    if( pcf_dev->bus->parent.ref_count == 0 )
    {
        rt_device_open( &(pcf_dev->bus->parent), RT_NULL );
    }
    return RT_EOK;
}

static rt_size_t pcf8574_read( rt_device_t dev, 
                                rt_off_t pos, 
                                void *buff, 
                                rt_size_t count )
{
    struct PCF8574_DEV *pcf_dev = (struct PCF8574_DEV *)dev;
    rt_size_t ret;
    uint8_t *p = (uint8_t *)buff;
    uint16_t start_data = pcf_dev->address;
    uint8_t recv_data[2];
   
    ret = rt_i2c_master_recv( pcf_dev->bus, start_data, 0, recv_data, 2 );
    rt_thread_delay(1);
    if( ret == 0 )
    {
        return ret;
    }
    if( p != RT_NULL )
    {
        *p = recv_data[1];
    }
    
    return ret;
}


static rt_size_t pcf8574_write( rt_device_t dev,
                                rt_off_t    pos,
                                const void *buffer,
                                rt_size_t   count)
{
    struct PCF8574_DEV *pcf_dev = (struct PCF8574_DEV *)dev;
    rt_size_t ret;
    uint8_t *p = (uint8_t *)buffer;
    uint8_t send_data[2];
    uint16_t start_data = pcf_dev->address;
    send_data[0] = p[0];
    //send_data[1] = p[0];
    
    ret = rt_i2c_master_send(  pcf_dev->bus, start_data, 0, send_data, 1);
    pcf_dev->out_data = p[0];
    rt_thread_delay(1);
    return ret;
}

static rt_err_t  pcf8574_control( rt_device_t dev, int cmd, void *args )
{
    struct PCF8574_DEV *pcf_dev = (struct PCF8574_DEV *)dev;
    rt_size_t ret;
    uint8_t data = pcf_dev->out_data;
    
    if( cmd == PCF8574_CTRL_SETBIT 
        || cmd == PCF8574_CTRL_CLRBIT )
    {
        uint8_t bitnum;
        
        
        if( args == RT_NULL )
        {
            return -RT_EINVAL;
        }
        bitnum = *((uint8_t *)args);
        if( bitnum > 7 )
        {
            return -RT_EINVAL;
        }
        
        if( cmd == PCF8574_CTRL_SETBIT )
        {
            data |= ( 1 << bitnum );
        } else if( cmd == PCF8574_CTRL_CLRBIT ) {
            data &= (~( 1 << bitnum ));
        }
        ret = pcf8574_write( dev, 0, &data, 1 );
        return ret>0? RT_EOK:( -RT_ERROR );
    } else if ( cmd == PCF8574_CTRL_GETOUTDATA ) {
        
        if( args == RT_NULL )
        {
            return -RT_EINVAL;
        }
        
        *((uint8_t *)args) = data;
        return RT_EOK;
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops pcf8574_ops =
{
    .init    = RT_NULL,
    .open    = pcf8574_open,
    .close   = RT_NULL,
    .read    = pcf8574_read,
    .write   = pcf8574_write,
    .control = pcf8574_control,
};
#endif

rt_err_t pcf8574_regist_and_connnect_i2c( struct PCF8574_DEV * dev,
                                           const char *pcf_dev_name,
                                           const char *i2c_bus_name,
                                           uint16_t address )
{
    struct rt_i2c_bus_device *bus_dev;

    if (dev == RT_NULL ||
        pcf_dev_name == RT_NULL ||
        i2c_bus_name == RT_NULL)
    {
        return -RT_EINVAL;
    }

    bus_dev = rt_i2c_bus_device_find( i2c_bus_name );
    if (bus_dev == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    dev->bus            = bus_dev;
    dev->address        = address;
    dev->parent.type    = RT_Device_Class_Miscellaneous;
#ifdef RT_USING_DEVICE_OPS
    dev->parent.ops     = &pcf8574_ops;
#else
    dev->parent.init    = RT_NULL;
    dev->parent.open    = pcf8574_open;
    dev->parent.close   = RT_NULL;
    dev->parent.read    = pcf8574_read;
    dev->parent.write   = pcf8574_write;
    dev->parent.control = pcf8574_control;
#endif

    return rt_device_register( &(dev->parent), 
                               pcf_dev_name, 
                               RT_DEVICE_FLAG_RDWR );
}

static int pcf8574_device_init(void)
{
    static struct PCF8574_DEV touch_oi;
    rt_err_t ret;

    ret = pcf8574_regist_and_connnect_i2c( &touch_oi, BSP_I2C_PCF8574_DEV, BSP_I2C_PCF8574_BUS, BSP_I2C_PCF8574_ADDRESS);
    if (RT_EOK != ret)
    {
        LOG_E("init pcf8574 device failed");
        return -1;
    }
    return 0;
}
INIT_DEVICE_EXPORT(pcf8574_device_init);

#endif /* BSP_USING_I2C_PCF8574 */
