/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-11     kyle         the first version
 */

#ifndef __DRV_ONEWIRE_H_INCLUDE__
#define __DRV_ONEWIRE_H_INCLUDE__

#include <rtthread.h>
#include <rtdevice.h>

enum 
{
    ONEWIRE_CMD_RESET = 0U,
    ONEWIRE_CMD_SEARCH,
    ONEWIRE_CMD_RESET_SEARCH,
    ONEWIRE_CMD_FIRST,
    ONEWIRE_CMD_NEXT,
    ONEWIRE_CMD_GETROM,
    ONEWIRE_CMD_GETFULLROM,
    ONEWIRE_CMD_SELECT_WITH_POINTER,
    
    ONEWIRE_CMD_ENABLE,
    ONEWIRE_CMD_DISABLE,
    ONEWIRE_CMD_SET,
    ONEWIRE_CMD_GET,
};

struct rt_onewire_configuration
{
    rt_uint32_t channel; /* 0-n */
    rt_uint32_t period;  /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
    rt_uint32_t pulse;   /* unit:ns (pulse<=period) */
};

struct rt_onewire_config
{
    rt_uint8_t last_discrepancy; /* search private */
    rt_uint8_t last_family_discrepancy; /* search private */
    rt_uint8_t last_device_flag; /* search private */
    rt_uint8_t rom_no[8]; /* 8-byte address of last search device */
};

struct rt_device_onewire;
struct rt_onewire_ops
{
    rt_err_t  (*init)   (struct rt_device_onewire *device);
    rt_size_t (*read)   (struct rt_device_onewire *device, rt_off_t pos, void *buffer, rt_size_t size);
    rt_size_t (*write)  (struct rt_device_onewire *device, rt_off_t pos, const void *buffer, rt_size_t size);
    rt_err_t  (*control)(struct rt_device_onewire *device, int cmd, void *arg);
};

struct rt_device_onewire
{
    struct rt_device parent;
    const struct rt_onewire_ops *ops;
};

rt_err_t rt_device_onewire_register(struct rt_device_onewire *device, const char *name, const struct rt_onewire_ops *ops, const void *user_data);


#endif /* __DRV_ONEWIRE_H_INCLUDE__ */
