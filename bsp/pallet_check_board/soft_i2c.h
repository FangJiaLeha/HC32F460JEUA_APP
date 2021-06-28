#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#include <rtdevice.h>
#include "stm32f10x.h"

/**
 * @brief 软i2c总线驱动初始化方法
 * 
 * @param[in] bus_dev 
 * @param[in] bus_name 
 * @return rt_err_t 
 */
rt_err_t rt_soft_i2c_bus_init( struct rt_i2c_bus_device* bus_dev, 
                                const char *bus_name );

rt_err_t touch_soft_i2c_bus_init( struct rt_i2c_bus_device* bus_dev, const char *bus_name );

#endif // STM32_SPI_H_INCLUDED
