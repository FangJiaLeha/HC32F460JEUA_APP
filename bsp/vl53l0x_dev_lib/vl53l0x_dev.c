/**
 * @file vl53l0x_dev.c
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-21
 *
 * 
 */

#include "vl53l0x_dev.h"
#include <rtthread.h>

#ifdef BSP_USING_I2C_VL53L0X

#include <rtdevice.h>

#define DBG_TAG               "bsp.vl53l0x"
#ifdef RT_I2C_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>


#define VL53L0X_ID_BASE             0x30

static const rt_base_t xshut_pins[] = 
{
    BSP_I2C_VL53L0X_XSHUT1_PIN,
    BSP_I2C_VL53L0X_XSHUT2_PIN,
    BSP_I2C_VL53L0X_XSHUT3_PIN,
    BSP_I2C_VL53L0X_XSHUT4_PIN,
};
static const rt_uint8_t xshut_pin_num = sizeof(xshut_pins) / sizeof(xshut_pins[0]);

//VL53L0X各测量模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//默认
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//高精度
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//长距离
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//高速
		
};

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t wsize = 0;
    rt_base_t level;
    struct rt_i2c_msg *m = Dev->msg;

    //rt_thread_delay( 1 );
    Dev->buff[0] = index;
    if( count > VL53L0X_MAX_I2C_XFER_SIZE )
    {
        count = VL53L0X_MAX_I2C_XFER_SIZE;
    }

    rt_memcpy( &(Dev->buff[1]), pdata, count );
#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    m->addr  = Dev->I2cDevAddr;
    m->flags = 0;
    m->len   = count + 1; // 包含寄存器地址字节数量
    m->buf   = (rt_uint8_t *)Dev->buff; // 

    wsize = Dev->bus->ops->master_xfer( Dev->bus, m, 1 );

    //wsize = rt_i2c_transfer( Dev->bus, &m, 1 );
    

    //wsize = rt_i2c_master_send( Dev->bus, Dev->I2cDevAddr, 0, Dev->buff, count + 1 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif
    if( wsize == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t ret = 0;
    struct rt_i2c_msg *m = Dev->msg;
    rt_base_t level;


    //rt_thread_delay( 1 );
    if( count > VL53L0X_MAX_I2C_XFER_SIZE )
    {
        count = VL53L0X_MAX_I2C_XFER_SIZE;
    }

    m[0].addr = Dev->I2cDevAddr;
    m[0].flags = 0;
    m[0].len = 1;
    m[0].buf = Dev->buff;
    Dev->buff[0] = index;

    m[1].addr = Dev->I2cDevAddr;
    m[1].flags = RT_I2C_RD;
    m[1].len = count;
    m[1].buf = pdata;

#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    //ret = rt_i2c_transfer( Dev->bus, m, 2 );
    ret = Dev->bus->ops->master_xfer( Dev->bus, m, 2 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif

    if( ret == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t ret = 0;
    struct rt_i2c_msg *m = Dev->msg;
    rt_base_t level;

    //rt_thread_delay( 1 );
    m[0].addr = Dev->I2cDevAddr;
    m[0].flags = 0;
    m[0].len = 1;
    m[0].buf = Dev->buff;
    Dev->buff[0] = index;

    m[1].addr = Dev->I2cDevAddr;
    m[1].flags = RT_I2C_RD;
    m[1].len = 1;
    m[1].buf = data;

#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    //ret = rt_i2c_transfer( Dev->bus, m, 2 );
    ret = Dev->bus->ops->master_xfer( Dev->bus, m, 2 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif

    if( ret == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t ret = 0;
    struct rt_i2c_msg *m = Dev->msg;
    rt_base_t level;

    //rt_thread_delay( 1 );
    m[0].addr = Dev->I2cDevAddr;
    m[0].flags = 0;
    m[0].len = 1;
    m[0].buf = Dev->buff;
    Dev->buff[0] = index;

    m[1].addr = Dev->I2cDevAddr;
    m[1].flags = RT_I2C_RD;
    m[1].len = 2;
    m[1].buf = &(Dev->buff[1]);

#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    //ret = rt_i2c_transfer( Dev->bus, m, 2 );
    ret = Dev->bus->ops->master_xfer( Dev->bus, m, 2 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif

    if( ret == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    } else { 
        *data = ( (uint16_t)Dev->buff[1] << 8 ) | (uint16_t)Dev->buff[2];
    }
    return Status;
}

VL53L0X_Error  VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_base_t level;
    rt_size_t ret = 0;
    struct rt_i2c_msg *m = Dev->msg;
    //rt_thread_delay( 1 );
    m[0].addr = Dev->I2cDevAddr;
    m[0].flags = 0;
    m[0].len = 1;
    m[0].buf = Dev->buff;
    Dev->buff[0] = index;

    m[1].addr = Dev->I2cDevAddr;
    m[1].flags = RT_I2C_RD;
    m[1].len = 4;
    m[1].buf = &(Dev->buff[1]);

#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    //ret = rt_i2c_transfer( Dev->bus, m, 2 );
    ret = Dev->bus->ops->master_xfer( Dev->bus, m, 2 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif

    if( ret == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    } else {
        *data = ( (uint32_t)Dev->buff[1]<<24 )
                +((uint32_t)Dev->buff[2]<<16)
                +((uint32_t)Dev->buff[3]<<8)
                +((uint32_t)Dev->buff[4]);
    }
    return Status;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t wsize = 0;
    rt_base_t level;
    struct rt_i2c_msg *m = Dev->msg;

    //rt_thread_delay( 1 );
    Dev->buff[0] = index;
    Dev->buff[1] = data;

    m->addr  = Dev->I2cDevAddr;
    m->flags = 0;
    m->len   = 2; // 包含寄存器地址字节数量
    m->buf   = (rt_uint8_t *)Dev->buff; // 

#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    wsize = Dev->bus->ops->master_xfer( Dev->bus, m, 1 );
    //wsize = rt_i2c_master_send( Dev->bus, Dev->I2cDevAddr, 0, Dev->buff, 2 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif

    if( wsize == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t wsize = 0;
    rt_base_t level;
    struct rt_i2c_msg *m = Dev->msg;

    //rt_thread_delay( 1 );
    Dev->buff[0] = index;
    Dev->buff[1] = data >> 8;
    Dev->buff[2] = data & 0xFF;

    m->addr  = Dev->I2cDevAddr;
    m->flags = 0;
    m->len   = 3; // 包含寄存器地址字节数量
    m->buf   = (rt_uint8_t *)Dev->buff; // 
#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    wsize = Dev->bus->ops->master_xfer( Dev->bus, m, 1 );
    //wsize = rt_i2c_master_send( Dev->bus, Dev->I2cDevAddr, 0, Dev->buff, 3 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif
    if( wsize == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    rt_size_t wsize = 0;
    rt_base_t level;
    struct rt_i2c_msg *m = Dev->msg;

    //rt_thread_delay( 1 );
    Dev->buff[0] = index;
    Dev->buff[1] = ( data >> 24 );
	Dev->buff[2] = ( ( data & 0xff0000 ) >> 16 );
	Dev->buff[3] = ( ( data & 0xff00 ) >> 8 );
	Dev->buff[4] = ( data & 0xff );

    m->addr  = Dev->I2cDevAddr;
    m->flags = 0;
    m->len   = 5; // 包含寄存器地址字节数量
    m->buf   = (rt_uint8_t *)Dev->buff; // 
#ifdef DISABLE_INTERRUPT
    level = rt_hw_interrupt_disable();
#endif
    wsize = Dev->bus->ops->master_xfer( Dev->bus, m, 1 );
    //wsize = rt_i2c_master_send( Dev->bus, Dev->I2cDevAddr, 0, Dev->buff, 5 );
#ifdef DISABLE_INTERRUPT
    rt_hw_interrupt_enable(level);
#endif

    if( wsize == 0 )
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;
    //rt_thread_delay( 1 );
    Status = VL53L0X_RdByte( Dev, index, &data );
    if( Status == VL53L0X_ERROR_NONE )
    {
        data = (data & AndData) | OrData;
        Status = VL53L0X_WrByte( Dev, index, data );
    }
    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    rt_thread_delay( 1 );
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
    uint16_t Id = 0;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t sta = 0x00;

    if( newaddr == dev->I2cDevAddr )
    {
        return VL53L0X_ERROR_NONE;
    }
        
    //在进行第一个寄存器访问之前设置I2C标准模式
    Status = VL53L0X_WrByte(dev,0x88,0x00);
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        sta=0x01;//设置I2C标准模式出错
        goto set_error;
    }
    rt_thread_delay(1);
    //尝试使用默认的0x52地址读取一个寄存器
    Status = VL53L0X_RdWord( dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id );
    if(Status!=VL53L0X_ERROR_NONE) 
    {
        sta=0x02;//读取寄存器出错
        goto set_error;
    }

    if( Id == 0xEEAA )
    {
        //设置设备新的I2C地址
        Status = VL53L0X_SetDeviceAddress( dev, newaddr );
        if(Status!=VL53L0X_ERROR_NONE) 
        {
            sta=0x03;//设置I2C地址出错
            goto set_error;
        }
        //修改参数结构体的I2C地址
        dev->I2cDevAddr = newaddr;
        //检查新的I2C地址读写是否正常
        Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
        if(Status!=VL53L0X_ERROR_NONE) 
        {
            sta=0x04;//新I2C地址读写出错
            goto set_error;
        }	
    }
set_error:
    return Status;
}

static rt_err_t _open( rt_device_t dev, rt_uint16_t oflag )
{
    VL53L0X_DEV vl53l0x_dev =  (VL53L0X_DEV) dev;
    if( vl53l0x_dev->bus->parent.ref_count == 0 )
    {
        rt_device_open( &(vl53l0x_dev->bus->parent), RT_NULL );
    }
    return RT_EOK;
}

static rt_size_t _read( rt_device_t dev, 
                        rt_off_t pos, 
                        void *buff, 
                        rt_size_t count )
{
    return 0;
}

static rt_size_t _write(rt_device_t dev,
                        rt_off_t    pos,
                        const void *buffer,
                        rt_size_t   count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    return 0;
}

static rt_err_t _cross_talk_calibration( VL53L0X_DEV dev, struct vl53l0x_calibration *cdata )
{
    uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    
    if( cdata == RT_NULL || cdata->calibrationed != 1 )
    {
        return -RT_EOK;
    }
    Status = VL53L0X_PerformXTalkCalibration( dev, XTalkCalDistance, &XTalkCompensationRateMegaCps);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    cdata->XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
    cdata->XTalkCalDistance = XTalkCalDistance;
    cdata->calibrationed = 0x55;
    return RT_EOK;
}

static rt_err_t _offset_calibration(  VL53L0X_DEV dev, struct vl53l0x_calibration *cdata )
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount = 7;
    uint32_t CalDistanceMilliMeter = 100 << 16;
	int32_t  OffsetMicroMeter = 30000;
    uint8_t isApertureSpads = 0;
    uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;

    if( cdata == RT_NULL )
    {
        return -RT_ERROR;
    }

    Status = VL53L0X_DataInit(dev);	
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    Status = VL53L0X_StaticInit( dev );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    rt_thread_delay(2);
    //执行参考Spad管理
    Status = VL53L0X_PerformRefSpadManagement( dev, &refSpadCount, &isApertureSpads );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    cdata->refSpadCount = refSpadCount;
    cdata->isApertureSpads = isApertureSpads;
    rt_thread_delay(2);
    //Ref参考校准
    Status = VL53L0X_PerformRefCalibration( dev, &VhvSettings, &PhaseCal );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    cdata->VhvSettings = VhvSettings;
    cdata->PhaseCal = PhaseCal;
    rt_thread_delay(2);

    //偏移校准
    Status = VL53L0X_PerformOffsetCalibration(dev, CalDistanceMilliMeter, &OffsetMicroMeter );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    cdata->CalDistanceMilliMeter = CalDistanceMilliMeter;
    cdata->OffsetMicroMeter = OffsetMicroMeter;
    cdata->calibrationed = 1; // 偏移校准完成
    return RT_EOK;
}

rt_tick_t stamp[10] = {0,0,0, 0,0,0,0,0, 0,0};
/**
 * @brief 初始化vl53L0x测距设备，并初始化为高精度测量模式
 * 
 * @param[in] dev 
 * @param[in] dev_addr 
 * @return 无
 */
static rt_err_t _dev_init( VL53L0X_DEV dev, struct vl53l0x_calibration *cdata )
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int delay = RT_TICK_PER_SECOND * 10 / 1000; // 延时50ms
    uint8_t VhvSettings;
	uint8_t PhaseCal;
    uint8_t isApertureSpads;
    uint32_t refSpadCount;
	uint32_t stopComplete = 0;
    stamp[0] = rt_tick_get();
    dev->I2cDevAddr = BSP_I2C_VL53L0X_ADDRESS; //初始默认地址
    if( dev->dev_enable != RT_NULL )
    {
        dev->dev_enable( dev->id, 0 ); // 禁用芯片
        rt_thread_delay( delay );
        dev->dev_enable( dev->id, 1 ); //使能芯片 
        rt_thread_delay( delay );
    }
    if( dev->id != 0 )
    {
        Status = vl53l0x_Addr_set( dev, dev->id);//设置VL53L0X传感器I2C地址
        if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 4 );
    }
    
    Status = VL53L0X_DataInit(dev);	
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }

    Status = VL53L0X_GetDeviceInfo( dev, &dev->info );//获取设备ID信息
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }

    Status = VL53L0X_StaticInit(dev);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    if( cdata != RT_NULL && cdata->calibrationed == 0x55 ) //已校准
    {
        //设定Spads校准值
        Status = VL53L0X_SetReferenceSpads( dev, cdata->refSpadCount, cdata->isApertureSpads );
		if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 1 );
        //设定Ref校准值
		Status = VL53L0X_SetRefCalibration( dev, cdata->VhvSettings, cdata->PhaseCal );
		if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 1 );
        //设定偏移校准值
		Status = VL53L0X_SetOffsetCalibrationDataMicroMeter( dev, cdata->OffsetMicroMeter );
		if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 1 );
        //设定串扰校准值
		Status = VL53L0X_SetXTalkCompensationRateMegaCps( dev, cdata->XTalkCompensationRateMegaCps );
		if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 1 );
    } else {
        // 开始自校准初始化参数
        Status = VL53L0X_PerformRefCalibration( dev, &VhvSettings, &PhaseCal );//Ref参考校准
        if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 1 );
        //执行参考SPAD管理
        Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads); 
        if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        //rt_thread_delay( 1 );
    }
    
    //使能单次测量模式
    //Status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);
    Status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //设置内部周期测量时间
    Status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,Mode_data[VL53L0X_DEFAULT_MEASURE_MODE].timingBudget); 
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );

    //使能SIGMA范围检查
    Status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //使能信号速率范围检查
    Status = VL53L0X_SetLimitCheckEnable(dev,
                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //设定SIGMA范围
    Status = VL53L0X_SetLimitCheckValue( dev,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
            Mode_data[VL53L0X_DEFAULT_MEASURE_MODE].sigmaLimit);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //设定信号速率范围范围
    Status = VL53L0X_SetLimitCheckValue(dev,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            Mode_data[VL53L0X_DEFAULT_MEASURE_MODE].signalLimit);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //设定完整测距最长时间
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,
            Mode_data[VL53L0X_DEFAULT_MEASURE_MODE].timingBudget);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //设定VCSEL脉冲周期
    Status = VL53L0X_SetVcselPulsePeriod(dev, 
        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 
        Mode_data[VL53L0X_DEFAULT_MEASURE_MODE].preRangeVcselPeriod);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( 1 );
    //设定VCSEL脉冲周期范围
    Status = VL53L0X_SetVcselPulsePeriod(   dev,
        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 
        Mode_data[VL53L0X_DEFAULT_MEASURE_MODE].finalRangeVcselPeriod);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    //rt_thread_delay( RT_TICK_PER_SECOND / 10 );
//    Status = VL53L0X_StopMeasurement(dev);//停止测量
//    if( Status != VL53L0X_ERROR_NONE )
//    {
//        return -RT_ERROR;
//    }
//    while( 1 )
//    {
//        Status = VL53L0X_GetStopCompletedStatus( dev, &stopComplete );
//        if( Status != VL53L0X_ERROR_NONE )
//        {
//            return -RT_ERROR;
//        }
//        if( stopComplete )
//        {
//            break;
//        }
//    }
    
    //rt_thread_delay( 1 );
    //设定触发中断上、下限值
//    Status = VL53L0X_SetInterruptThresholds(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 60, 150 );
//	if( Status != VL53L0X_ERROR_NONE )
//    {
//        return -RT_ERROR;
//    }
//	rt_thread_delay( 1 );
//	Status = VL53L0X_SetGpioConfig(dev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT, VL53L0X_INTERRUPTPOLARITY_LOW);//设定触发中断模式 下降沿
//	if( Status != VL53L0X_ERROR_NONE )
//    {
//        return -RT_ERROR;
//    }
//	rt_thread_delay( 1 );
//	Status = VL53L0X_ClearInterruptMask(dev,0);//清除VL53L0X中断标志位
    
    Status = VL53L0X_StartMeasurement( dev );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    dev->state = VL53L0X_STATE_INWORK; // 初始化完成
    stamp[9] = rt_tick_get();
    return RT_EOK;
}

static rt_err_t _high_accuracy_measure( VL53L0X_DEV dev, uint16_t *distance )
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t ready = 0;
    VL53L0X_RangingMeasurementData_t measure_data;
    
    Status = VL53L0X_GetMeasurementDataReady( dev, &ready);
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }
    if( ready )
    {
        Status = VL53L0X_GetRangingMeasurementData(dev, &measure_data);//
        if( Status != VL53L0X_ERROR_NONE )
        {
            return -RT_ERROR;
        }
        VL53L0X_ClearInterruptMask(dev,0);
        if( measure_data.RangeStatus == 0 )
        {
            *distance = measure_data.RangeMilliMeter;
        } else {
            return -RT_ERROR;
        }
        VL53L0X_ClearInterruptMask(dev,0);
    } else {
        return -RT_ENOSYS;
    }
    /*
    Status = VL53L0X_PerformSingleRangingMeasurement( dev, &measure_data );
    if( Status != VL53L0X_ERROR_NONE )
    {
        return -RT_ERROR;
    }

    if( measure_data.RangeStatus == 0 )
    {
        *distance = measure_data.RangeMilliMeter;
    } else {
        return -RT_ERROR;
    }*/
    return RT_EOK;
}

static rt_err_t  _control(rt_device_t dev, int cmd, void *args)
{
    VL53L0X_DEV vl53l0x_dev =  (VL53L0X_DEV) dev;
    rt_err_t ret = RT_EOK;

    if( cmd == VL53L0X_CTRL_SET_DEV_ENABLE_HANLDER )
    {
        vl53l0x_dev->dev_enable = args;
    } else if( cmd == VL53L0X_CTRL_INIT_DEV ) {
        struct vl53l0x_calibration *cdata = (struct vl53l0x_calibration *)args;
        rt_mutex_take( &vl53l0x_dev->bus->lock, RT_WAITING_FOREVER );
        ret = _dev_init( vl53l0x_dev, cdata );
        rt_mutex_release( &vl53l0x_dev->bus->lock );
    } else if(  VL53L0X_CTRL_DEINIT_DEV == cmd ) {
        vl53l0x_dev->dev_enable( vl53l0x_dev->id, 0 ); // 禁用芯片
        rt_thread_delay( RT_TICK_PER_SECOND / 10 );
    } else if ( cmd == VL53L0X_CTRL_MEASUREMENT ) {
        if( vl53l0x_dev->state != VL53L0X_STATE_INWORK )
        {
            return -RT_ERROR;
        }
        rt_mutex_take( &vl53l0x_dev->bus->lock, RT_WAITING_FOREVER );
        ret = _high_accuracy_measure( vl53l0x_dev, (uint16_t *)args );
        rt_mutex_release( &vl53l0x_dev->bus->lock );
    } else if( cmd == VL53L0X_CTRL_SET_ID ) {
        uint8_t *dev_id = (uint8_t*)args;
        if(  *dev_id != vl53l0x_dev->id )
        {
            vl53l0x_dev->id = *dev_id;
        }
    } else if ( cmd == VL53L0X_CTRL_GET_ID ) {
        uint8_t *dev_id = (uint8_t*)args;
        *dev_id = vl53l0x_dev->id;
    } else if( cmd == VL53L0X_CTRL_OFFSET_CALIBRATION ) {
        struct vl53l0x_calibration *cdata = (struct vl53l0x_calibration *)args;

        vl53l0x_dev->state = VL53L0X_STATE_CALIBRATION;
        rt_thread_delay( RT_TICK_PER_SECOND / 2 );
        rt_mutex_take( &vl53l0x_dev->bus->lock, RT_WAITING_FOREVER );
        ret = _offset_calibration( vl53l0x_dev, cdata );
        rt_mutex_release( &vl53l0x_dev->bus->lock );
    } else if( cmd == VL53L0X_CTRL_CROSSTALK_CALIBRATION ) {
        struct vl53l0x_calibration *cdata = (struct vl53l0x_calibration *)args;
        rt_mutex_take( &vl53l0x_dev->bus->lock, RT_WAITING_FOREVER );
        ret = _cross_talk_calibration( vl53l0x_dev, cdata );
        if( ret == RT_EOK )
        {
            _dev_init( vl53l0x_dev, cdata );
        }
        rt_mutex_release( &vl53l0x_dev->bus->lock );
    }
    return ret;
}

static rt_err_t _init( rt_device_t dev )
{
    VL53L0X_DEV vl53l0x_dev =  (VL53L0X_DEV) dev;
    rt_err_t ret = RT_EOK;
    
    //ret = _dev_init( vl53l0x_dev );
    return ret;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops vl53l0x_ops =
{
    .init    = _init,
    .open    = _open,
    .close   = RT_NULL,
    .read    = _read,
    .write   = _write,
    .control = _control,
};
#endif

rt_err_t  vl53l0x_regist_and_connect_i2c(   VL53L0X_DEV dev, 
                                            uint8_t dev_id,
                                            const char *dev_name, 
                                            const char *i2c_bus_name, 
                                            void (* dev_enable)( uint8_t id, uint8_t enable ) )
{
    struct rt_i2c_bus_device *bus;
    VL53L0X_DEV vl53l0x_dev =  dev;

    if (dev == RT_NULL || dev_name == RT_NULL || i2c_bus_name == RT_NULL)
    {
        return -RT_EINVAL;
    }

    bus = rt_i2c_bus_device_find( i2c_bus_name );
    if (bus == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    vl53l0x_dev->id             = dev_id;
    vl53l0x_dev->dev_enable     = dev_enable;
    vl53l0x_dev->bus            = bus;
    vl53l0x_dev->I2cDevAddr     = BSP_I2C_VL53L0X_ADDRESS; //初始默认地址
    vl53l0x_dev->parent.type    = RT_Device_Class_Miscellaneous;
#ifdef RT_USING_DEVICE_OPS
    vl53l0x_dev->parent.ops     = &vl53l0x_ops;
#else
    vl53l0x_dev->parent.init    = _init;
    vl53l0x_dev->parent.open    = _open;
    vl53l0x_dev->parent.close   = RT_NULL;
    vl53l0x_dev->parent.read    = _read;
    vl53l0x_dev->parent.write   = _write;
    vl53l0x_dev->parent.control = _control;
#endif

    return rt_device_register( &(vl53l0x_dev->parent), 
                               dev_name, 
                               RT_DEVICE_FLAG_RDWR );
}

static void vl53l0x_xshut_pin_init(void)
{
    rt_uint8_t i;
    
    for (i = 0; i < xshut_pin_num; i++)
    {
        if (xshut_pins[i])
        {
            rt_pin_mode(xshut_pins[i], PIN_MODE_OUTPUT_OD);
        }
    }
}

static void vl53l0x_xshut_pin_enable(uint8_t id, uint8_t enable)
{
    rt_base_t num = (id - VL53L0X_ID_BASE) % xshut_pin_num;
    
    if (num >= xshut_pin_num)
    {
        return;
    }
    
    rt_pin_write(xshut_pins[num], enable);
}

static int vl53l0x_device_init(void)
{
    rt_uint8_t i;
    rt_err_t ret;
    static VL53L0X_Dev_t vl53l0x[xshut_pin_num];
    char name[RT_NAME_MAX] = BSP_I2C_VL53L0X_DEV;

    vl53l0x_xshut_pin_init();
    for (i = 0; i < xshut_pin_num; i++)
    {
        name[rt_strlen(BSP_I2C_VL53L0X_DEV)] = '0' + i;
        ret = vl53l0x_regist_and_connect_i2c((VL53L0X_DEV)&vl53l0x[i],
                                             (VL53L0X_ID_BASE + i),
                                             name,
                                             BSP_I2C_VL53L0X_BUS,
                                             vl53l0x_xshut_pin_enable);
    }
    if (RT_EOK != ret)
    {
        LOG_E("init vl53l0x device failed");
        return -1;
    }
    return 0;
}
INIT_DEVICE_EXPORT(vl53l0x_device_init);

#endif /* BSP_USING_I2C_VL53L0X */
