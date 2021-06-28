#include "tlc59108.h"
#include <rtthread.h>

#ifdef BSP_USING_PALLET_LED_BAR

#include <rtdevice.h>

#define DBG_TAG               "bsp.pallet_led_bar"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>


#define LED_BAR_PWM_PERIOD    40000 //ns

struct led_bar_dev {
    struct rt_device parent;
    
    struct rt_device_pwm *pwm_dev;
    int pwm_channel;
};

static rt_err_t _open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_err_t result = RT_EOK;
    struct led_bar_dev *led_dev = (struct led_bar_dev *)dev;
    struct rt_device_pwm *device = led_dev->pwm_dev;
    struct rt_pwm_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }

    result = rt_device_open(&device->parent, RT_DEVICE_FLAG_WRONLY);
    if (result != RT_EOK)
    {
        return result;
    }

    configuration.channel = led_dev->pwm_channel;
    configuration.period  = LED_BAR_PWM_PERIOD;
    configuration.pulse   = 0;
    result = rt_device_control(&device->parent, PWM_CMD_SET, &configuration);    
    if (result != RT_EOK)
    {
        return result;
    }

    result = rt_device_control(&device->parent, PWM_CMD_ENABLE, &configuration);
    if (result != RT_EOK)
    {
        return result;
    }

    return result;
}

static rt_size_t _write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_err_t result = RT_EOK;
    struct led_bar_dev *led_dev = (struct led_bar_dev *)dev;
    struct rt_device_pwm *device = led_dev->pwm_dev;
    struct rt_pwm_configuration configuration = {0};
    float color_av = *(float *)buffer;

    if (!device)
    {
        return 0;
    }

    configuration.channel = led_dev->pwm_channel;
    configuration.period  = LED_BAR_PWM_PERIOD;
    configuration.pulse   = (LED_BAR_PWM_PERIOD * (255 - color_av)) / 255;
    result = rt_device_control(&device->parent, PWM_CMD_SET, &configuration);    
    if (result != RT_EOK)
    {
        return 0;
    }

    return size;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops pallet_led_bar_ops =
{
    .init    = RT_NULL,
    .open    = _open,
    .close   = RT_NULL,
    .read    = RT_NULL,
    .write   = _write,
    .control = RT_NULL,
};
#endif

rt_err_t pallet_led_bar_register(struct led_bar_dev *led_dev, 
                         const char *dev_name,
                         const char *pwm_name,
                         const int  pwm_channel)
{
    rt_device_t dev = &led_dev->parent;

    led_dev->pwm_dev = (struct rt_device_pwm *)rt_device_find(pwm_name);
    if (led_dev->pwm_dev == RT_NULL)
    {
        LOG_E("can't find %s device", pwm_name);
        return RT_ERROR;
    }

    led_dev->pwm_channel = pwm_channel;

    dev->type        = RT_Device_Class_Char;
    dev->rx_indicate = RT_NULL;
    dev->tx_complete = RT_NULL;
#ifdef RT_USING_DEVICE_OPS
    dev->ops         = &pallet_led_bar_ops;
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

static int pallet_led_bar_device_init(void)
{
    static struct led_bar_dev pallet_led_bar_dev;
    rt_err_t ret;

    ret = pallet_led_bar_register(&pallet_led_bar_dev, BSP_PALLET_LED_BAR_DEV, BSP_PALLET_LED_BAR_PWMX, BSP_PALLET_LED_BAR_CHANNEL);
    if (RT_EOK != ret)
    {
        LOG_E("init pallet_led_bar device failed");
        return -1;
    }
    return 0;
}
INIT_DEVICE_EXPORT(pallet_led_bar_device_init);

#endif /* BSP_USING_PALLET_LED_BAR */
