/**
 * @file tlc59108.h
 * @author phil (zengfei@pudutech.com)
 * @brief i2c端口扩展芯片pcf8574接口文档
 * @version 0.1
 * @date 2019-08-08
 * 
 */

#ifndef __TLC59108_H__
#define __TLC59108_H__

#include <rtdevice.h>


struct TLC59108REGS
{
    union {
        uint8_t mode1;
        struct {
            uint8_t ALLCALL:1; 
            uint8_t SUB3:1;
            uint8_t SUB2:1;
            uint8_t SUB1:1;
            uint8_t OSC:1;
            uint8_t AI0:1;
            uint8_t AI1:1;
            uint8_t AI2:1;   
        } bit;
    } MODE1;
    union {
        uint8_t mode2;
        struct {
            uint8_t Resv:3; 
            uint8_t OCH:1;
            uint8_t Resv1:1;
            uint8_t DMBLNK:1;
            uint8_t Resv2:2;
        } bit;
    } MODE2;    
    uint8_t PWM0;
    uint8_t PWM1;
    uint8_t PWM2;
    uint8_t PWM3;
    uint8_t PWM4;
    uint8_t PWM5;
    uint8_t PWM6;
    uint8_t PWM7;
    uint8_t GRPPWM;
    uint8_t GRPFREQ;
    union {
        uint8_t ledout0;
        struct {
            uint8_t LDR0:2; 
            uint8_t LDR1:2; 
            uint8_t LDR2:2; 
            uint8_t LDR3:2;  
        } bit;
    } LEDOUT0;
    union {
        uint8_t ledout1;
        struct {
            uint8_t LDR4:2; 
            uint8_t LDR5:2; 
            uint8_t LDR6:2; 
            uint8_t LDR7:2;  
        } bit;
    } LEDOUT1;    
    uint8_t SUBADR1;
    uint8_t SUBADR2;
    uint8_t SUBADR3;
    uint8_t ALLCALLADR;

};

struct TLC59108_DEV{
    struct rt_device    parent;        //!< 父对象
    struct rt_i2c_bus_device *bus;     //!< 总线设备
    uint16_t             address;      //!< 设备i2c地址
    struct rt_i2c_msg msg[2];
    struct TLC59108REGS regs;            //!< tlc 寄存器列表
};

#endif /* __TLC59108_H__ */
