/**
 * @file pcf8574.h
 * @author phil (zengfei@pudutech.com)
 * @brief i2c端口扩展芯片pcf8574接口文档
 * @version 0.1
 * @date 2019-08-08
 * 
 */

#ifndef __PCF8574_H__
#define __PCF8574_H__

#include <rtdevice.h>

#define PCF8574_CTRL_SETBIT         0x380
#define PCF8574_CTRL_CLRBIT         0x381
#define PCF8574_CTRL_GETOUTDATA     0x382

struct PCF8574_DEV{
    struct rt_device    parent;        //!< 父对象
    struct rt_i2c_bus_device *bus;     //!< 总线设备
    uint16_t             address;      //!< 设备i2c地址
    uint8_t             out_data;      //!< 输出值
};

#endif /* __PCF8574_H__ */
