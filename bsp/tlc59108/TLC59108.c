/**
 * @file pcf8574.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-08
 * 
 */
 
#include "tlc59108.h"
#include <rtthread.h>

#ifdef BSP_USING_I2C_TLC59108

#include <rtdevice.h>

#define DBG_TAG               "bsp.tlc59108"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

static rt_size_t _read_regs( rt_device_t dev, 
                                rt_off_t pos, 
                                void *buff, 
                                rt_size_t count )
{
    struct TLC59108_DEV *tlc59108_dev = (struct TLC59108_DEV *)dev;
    rt_size_t ret;
    uint8_t ctrl_reg = 0x80 | ( pos & 0x1F );
    struct rt_i2c_msg *m = tlc59108_dev->msg;
    uint8_t reg_addr = pos & 0x1F;

    if( ( count + reg_addr ) > 18 || buff == RT_NULL || reg_addr > 18 )
    { 
        return 0;
    }

    m[0].addr = tlc59108_dev->address;
    m[0].flags = 0;
    m[0].len = 1;
    m[0].buf = &ctrl_reg;

    m[1].addr = tlc59108_dev->address;
    m[1].flags = RT_I2C_RD;
    m[1].len = count;
    m[1].buf = buff;

    ret = rt_i2c_transfer( tlc59108_dev->bus, m, 2 );

    return ret;
}

static rt_err_t tlc59108_open( rt_device_t dev, rt_uint16_t oflag )
{
    struct TLC59108_DEV *tlc59108_dev = (struct TLC59108_DEV *)dev;
    rt_size_t ret = 0;
    if( tlc59108_dev->bus->parent.ref_count == 0 )
    {
        rt_device_open( &(tlc59108_dev->bus->parent), RT_NULL );
    }

    ret  = _read_regs( dev, 0, (uint8_t*)&tlc59108_dev->regs, 18 );
    if( ret == 0 )
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}



static rt_size_t tlc59108_read( rt_device_t dev, 
                                rt_off_t pos, 
                                void *buff, 
                                rt_size_t count )
{
    struct TLC59108_DEV *tlc59108_dev = (struct TLC59108_DEV *)dev;
    rt_size_t ret;
    uint8_t ctrl_reg = 0x80 | ( pos & 0x1F );
    uint16_t start_data = tlc59108_dev->address;
    uint8_t *recv = RT_NULL;
    uint8_t reg_addr = pos & 0x1F;

    if( ( count + reg_addr ) > 18 || buff == RT_NULL || reg_addr > 18 )
    { 
        return 0;
    }

    recv = ( (uint8_t *)&(tlc59108_dev->regs) ) + ( pos & 0x1F ); // 偏移 寄存地址

    rt_memcpy( buff, recv, count );
    
    return count;
}

static rt_size_t tlc59108_write( rt_device_t dev,
                                rt_off_t    pos,
                                const void *buffer,
                                rt_size_t   count)
{
    struct TLC59108_DEV *tlc59108_dev = (struct TLC59108_DEV *)dev;
    rt_size_t ret;
    uint8_t *p;
    uint8_t reg_addr = pos & 0x1F;

    //send_data[1] = p[0];
    if( buffer == RT_NULL || ( count + reg_addr ) > 18 || reg_addr > 18 )
    {
        return 0;
    }
    
    p = ( (uint8_t *)&(tlc59108_dev->regs) ) + reg_addr; // 偏移 寄存地址

    ret = rt_i2c_master_send(  tlc59108_dev->bus, tlc59108_dev->address, 0, buffer, count );
    if( ret > 0 )
    {
        rt_memcpy( p, buffer, count );
    }
    rt_thread_delay(1);
    return ret;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops tlc59108_ops =
{
    .init    = RT_NULL,
    .open    = tlc59108_open,
    .close   = RT_NULL,
    .read    = tlc59108_read,
    .write   = tlc59108_write,
    .control = RT_NULL,
};
#endif

rt_err_t tlc59108_regist_and_connect_i2c( struct TLC59108_DEV * dev,
                                           const char *tlc59108_dev_name,
                                           const char *i2c_bus_name,
                                           uint16_t address )
{
    struct rt_i2c_bus_device *bus_dev;
    
    if (dev == RT_NULL ||
        tlc59108_dev_name == RT_NULL ||
        i2c_bus_name == RT_NULL)
    {
        return -RT_EINVAL;
    }

    bus_dev = rt_i2c_bus_device_find(i2c_bus_name);
    if (bus_dev == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    dev->bus            = bus_dev;
    dev->address        = address;
    dev->parent.type    = RT_Device_Class_Miscellaneous;
#ifdef RT_USING_DEVICE_OPS
    dev->parent.ops     = &tlc59108_ops;
#else
    dev->parent.init    = RT_NULL;
    dev->parent.open    = tlc59108_open;
    dev->parent.close   = RT_NULL;
    dev->parent.read    = tlc59108_read;
    dev->parent.write   = tlc59108_write;
    dev->parent.control = RT_NULL;
#endif

    return rt_device_register( &(dev->parent), 
                               tlc59108_dev_name, 
                               RT_DEVICE_FLAG_RDWR );
}

static int tlc59108_device_init(void)
{
    static struct TLC59108_DEV tlc59108_dev;
    rt_err_t ret;

    ret = tlc59108_regist_and_connect_i2c( &tlc59108_dev, BSP_I2C_TLC59108_DEV, BSP_I2C_TLC59108_BUS, BSP_I2C_TLC59108_ADDRESS);
    if (RT_EOK != ret)
    {
        LOG_E("init tlc59108 device failed");
        return -1;
    }
    return 0;
}
INIT_DEVICE_EXPORT(tlc59108_device_init);

#endif /* BSP_USING_I2C_TLC59108 */
