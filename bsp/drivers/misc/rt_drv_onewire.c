/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-11     kyle         the first version
 */

#include <string.h>

//#include <drivers/rt_drv_onewire.h>
#include "rt_drv_onewire.h"

#define DBG_TAG               "ONEWIRE"
#ifdef RT_ONEWIRE_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

static rt_err_t _onewire_init(rt_device_t dev)
{
    rt_err_t result = RT_EOK;
    struct rt_device_onewire *onewire = (struct rt_device_onewire *)dev;

    if (onewire->ops->init)
    {
        result = onewire->ops->init(onewire);
    }

    return result;
}

static rt_size_t _onewire_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_err_t result = RT_EOK;
    struct rt_device_onewire *onewire = (struct rt_device_onewire *)dev;

    if (onewire->ops->read)
    {
        result = onewire->ops->read(onewire, pos, buffer, size);
        if (result != RT_EOK)
        {
            return 0;
        }
    }

    return size;
}

static rt_size_t _onewire_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_err_t result = RT_EOK;
    struct rt_device_onewire *onewire = (struct rt_device_onewire *)dev;

    if (onewire->ops->write)
    {
        result = onewire->ops->write(onewire, pos, buffer, size);
        if (result != RT_EOK)
        {
            return 0;
        }
    }

    return size;
}

static rt_err_t _onewire_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct rt_device_onewire *onewire = (struct rt_device_onewire *)dev;

    if (onewire->ops->control)
    {
        result = onewire->ops->control(onewire, cmd, args);
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops onewire_device_ops =
{
    .init    = _onewire_init,
    .open    = RT_NULL,
    .close   = RT_NULL,
    .read    = _onewire_read,
    .write   = _onewire_write,
    .control = _onewire_control
};
#endif /* RT_USING_DEVICE_OPS */

rt_err_t rt_device_onewire_register(struct rt_device_onewire *device, const char *name, const struct rt_onewire_ops *ops, const void *user_data)
{
    rt_err_t result = RT_EOK;

    memset(device, 0, sizeof(struct rt_device_onewire));

#ifdef RT_USING_DEVICE_OPS
    device->parent.ops = &onewire_device_ops;
#else
    device->parent.init  = _onewire_init;
    device->parent.open  = RT_NULL;
    device->parent.close = RT_NULL;
    device->parent.read  = _onewire_read;
    device->parent.write = _onewire_write;
    device->parent.control = _onewire_control;
#endif /* RT_USING_DEVICE_OPS */

    device->parent.type      = RT_Device_Class_Miscellaneous;
    device->ops              = ops;
    device->parent.user_data = (void *)user_data;

    result = rt_device_register(&device->parent, name, RT_DEVICE_FLAG_WRONLY);

    return result;
}

rt_err_t rt_onewire_enable(struct rt_device_onewire *device, int channel)
{
    rt_err_t result = RT_EOK;
    struct rt_onewire_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }
    
    configuration.channel = channel;
    result = rt_device_control(&device->parent, ONEWIRE_CMD_ENABLE, &configuration);

    return result;
}

rt_err_t rt_onewire_disable(struct rt_device_onewire *device, int channel)
{
    rt_err_t result = RT_EOK;
    struct rt_onewire_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }

    configuration.channel = channel;
    result = rt_device_control(&device->parent, ONEWIRE_CMD_DISABLE, &configuration);

    return result;
}

rt_err_t rt_onewire_set(struct rt_device_onewire *device, int channel, rt_uint32_t period, rt_uint32_t pulse)
{
    rt_err_t result = RT_EOK;
    struct rt_onewire_configuration configuration = {0};

    if (!device)
    {
        return -RT_EIO;
    }
    
    result = rt_device_init(&device->parent);
    if (result != RT_EOK)
    {
        return result;
    }

    configuration.channel = channel;
    configuration.period = period;
    configuration.pulse = pulse;
    result = rt_device_control(&device->parent, ONEWIRE_CMD_SET, &configuration);

    return result;
}

rt_err_t rt_onewire_write(struct rt_device_onewire *device, int channel, const uint8_t *buff, uint32_t size)
{
    rt_err_t result = RT_EOK;
    struct rt_onewire_configuration configuration = {0};
    uint16_t *pulse_buff;
    uint32_t pulse_size;
    uint8_t res_num;
    uint32_t i;
    uint8_t  j;

    if (!device)
    {
        return -RT_EIO;
    }

    result = rt_device_open(&device->parent, RT_DEVICE_FLAG_WRONLY);
    if (result != RT_EOK)
    {
        return result;
    }

    configuration.channel = channel;
    configuration.period = 1250;
    configuration.pulse  = 0;
    result = rt_device_control(&device->parent, ONEWIRE_CMD_SET, &configuration);    
    if (result != RT_EOK)
    {
        return result;
    }

    LOG_HEX(DBG_TAG, 3, (rt_uint8_t *)buff, size);

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

    LOG_HEX(DBG_TAG, 6, (rt_uint8_t *)pulse_buff, sizeof(uint16_t) * pulse_size);

    result = rt_device_write(&device->parent, 0, pulse_buff, pulse_size);
    rt_free(pulse_buff);

    return result;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

FINSH_FUNCTION_EXPORT_ALIAS(rt_onewire_enable, onewire_enable, enable pwm by channel.);
FINSH_FUNCTION_EXPORT_ALIAS(rt_onewire_disable, onewire_disable, disable pwm by channel.);
FINSH_FUNCTION_EXPORT_ALIAS(rt_onewire_set, onewire_set, set pwm.);
FINSH_FUNCTION_EXPORT_ALIAS(rt_onewire_write, onewire_write, write pwm data.);


#ifdef FINSH_USING_MSH
static int onewire_enable(int argc, char **argv)
{
    int result = 0;
    struct rt_device_onewire *device = RT_NULL;

    if (argc != 3)
    {
        rt_kprintf("Usage: onewire_enable onewire1 1\n");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_onewire *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }

    result = rt_onewire_enable(device, atoi(argv[2]));

_exit:
    return result;
}
MSH_CMD_EXPORT(onewire_enable, onewire_enable onewire1 1);

static int onewire_disable(int argc, char **argv)
{
    int result = 0;
    struct rt_device_onewire *device = RT_NULL;

    if (argc != 3)
    {
        rt_kprintf("Usage: onewire_disable onewire1 1\n");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_onewire *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }

    result = rt_onewire_disable(device, atoi(argv[2]));

_exit:
    return result;
}
MSH_CMD_EXPORT(onewire_disable, onewire_disable onewire1 1);

static int onewire_set(int argc, char **argv)
{
    int result = 0;
    struct rt_device_onewire *device = RT_NULL;

    if (argc != 5)
    {
        rt_kprintf("Usage: onewire_set onewire1 1 100 50\n");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_onewire *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }

    result = rt_onewire_set(device, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

_exit:
    return result;
}
MSH_CMD_EXPORT(onewire_set, onewire_set onewire1 1 100 50);

static int onewire_write(int argc, char **argv)
{
    int result = 0;
    struct rt_device_onewire *device = RT_NULL;
    uint8_t *buff;
    uint32_t i;
    uint32_t size;

    if (argc < 4)
    {
        rt_kprintf("Usage: onewire_write onewire1 1 0x01 0x02\n");
        result = -RT_ERROR;
        goto _exit;
    }

    device = (struct rt_device_onewire *)rt_device_find(argv[1]);
    if (!device)
    {
        result = -RT_EIO;
        goto _exit;
    }

    size = argc - 3;
    buff = (uint8_t *)rt_malloc(sizeof(uint8_t) * size);
    for (i = 0; i < size; i++)
    {
        buff[i] = atoi(argv[3 + i]);
    }

    result = rt_onewire_write(device, atoi(argv[2]), buff, size);
    rt_free(buff);

_exit:
    return result;
}
MSH_CMD_EXPORT(onewire_write, onewire_write onewire1 1 0x01 0x02);

#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */
