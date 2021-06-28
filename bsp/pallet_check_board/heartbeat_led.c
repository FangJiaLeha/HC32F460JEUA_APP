/**
 * @file heartbeat_led.c
 * @author phil (zengfei@pudutech.com)
 * @brief 功能板卡心跳灯驱动
 * @version 0.1
 * @date 2019-10-29
 * 
 */
 
#include "heartbeat_led.h"
#include <rtthread.h>

#ifdef BSP_USING_HEARTBEAT_LED

#include <rtdevice.h>

#define DBG_TAG               "bsp.heartbeat_led"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

struct heartbeat_led {
    struct rt_device parent;
    
    rt_uint32_t pin;
};

static rt_err_t _open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_err_t result = RT_EOK;
    struct heartbeat_led *led_dev = (struct heartbeat_led *)dev;
    
    rt_pin_mode(led_dev->pin, PIN_MODE_OUTPUT);
    rt_pin_write(led_dev->pin, PIN_HIGH);

    return result;
}

static rt_size_t _write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct heartbeat_led *led_dev = (struct heartbeat_led *)dev;
    uint8_t *val = (uint8_t *)buffer;

    rt_pin_write(led_dev->pin, (*val) ? PIN_HIGH : PIN_LOW);

    return size;
}


#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops heartbeat_led_ops =
{
    .init    = RT_NULL,
    .open    = _open,
    .close   = RT_NULL,
    .read    = RT_NULL,
    .write   = _write,
    .control = RT_NULL,
};
#endif

rt_err_t heartbeat_led_register(struct heartbeat_led *led_dev, 
                         const char *dev_name,
                         rt_uint32_t pin)
{
    rt_device_t dev = &led_dev->parent;

    led_dev->pin     = pin;

    dev->type        = RT_Device_Class_Char;
    dev->rx_indicate = RT_NULL;
    dev->tx_complete = RT_NULL;
#ifdef RT_USING_DEVICE_OPS
    dev->ops         = &heartbeat_led_ops;
#else                
    dev->init        = RT_NULL;
    dev->open        = _open;
    dev->close       = RT_NULL;
    dev->read        = RT_NULL;
    dev->write       = _write;
    dev->control     = RT_NULL;
#endif

    return rt_device_register(dev, dev_name, RT_DEVICE_FLAG_WRONLY);
}

static int heartbeat_led_device_init(void)
{
    static struct heartbeat_led heartbeat_led_dev;
    rt_err_t ret;

    ret = heartbeat_led_register(&heartbeat_led_dev, "HeartBeat", BSP_HEARTBEAT_PIN);
    if (RT_EOK != ret)
    {
        LOG_E("init heartbeat_led device failed");
        return -1;
    }
    return 0;
}
INIT_DEVICE_EXPORT(heartbeat_led_device_init);

#endif /* BSP_USING_HEARTBEAT_LED */
