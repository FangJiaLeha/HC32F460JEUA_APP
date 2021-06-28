/**
 * @file vl53l0x_dev.h
 * @author phil (zengfei@pudutech.com)
 * @brief 激光测距芯片vl53l0x设备驱动
 * @version 0.1
 * @date 2019-08-21 
 * 
 */
#ifndef __VL53L0X_DEV_H__
#define __VL53L0X_DEV_H__

#include <stdint.h>
#include "vl53l0x_api.h"

#define VL53L0X_DEFAULT_MEASURE_MODE            0
#define VL53L0X_HIGH_ACCURANCY_MEASURE_MODE     1
#define VL53L0X_LONG_MEASURE_MODE               2
#define VL53L0X_HIGH_SPEED_MEASURE_MODE         3 

#define VL53L0X_CTRL_SET_DEV_ENABLE_HANLDER      0x301
#define VL53L0X_CTRL_INIT_DEV                    0x302
#define VL53L0X_CTRL_DEINIT_DEV				     0x303
#define VL53L0X_CTRL_MEASUREMENT                 0x304
#define VL53L0X_CTRL_SET_ID                      0x305
#define VL53L0X_CTRL_GET_ID                      0x306
#define VL53L0X_CTRL_OFFSET_CALIBRATION          0x307
#define VL53L0X_CTRL_CROSSTALK_CALIBRATION       0x308

#define VL53L0X_STATE_UNINIT                    0x00
#define VL53L0X_STATE_INITING                   0x01
#define VL53L0X_STATE_CALIBRATION               0x5A
#define VL53L0X_STATE_INWORK                    0x5A


/**
 * @brief 校准参数
 * 
 */
struct vl53l0x_calibration
{
	uint8_t  calibrationed;               //!< 校准完成标志，0x55:已校准;其他，未校准
	uint8_t  isApertureSpads;             //!< ApertureSpads值
	uint8_t  VhvSettings;                 //!< VhvSettings值
	uint8_t  PhaseCal;                    //!< PhaseCal值
	uint32_t XTalkCalDistance;            //!< XTalkCalDistance值
	uint32_t XTalkCompensationRateMegaCps;//!< XTalkCompensationRateMegaCps值
	uint32_t CalDistanceMilliMeter;       //!< CalDistanceMilliMeter值
	int32_t  OffsetMicroMeter;            //!< OffsetMicroMeter值
	uint32_t refSpadCount;                //!< refSpadCount值
};
                                            
#endif /* __VL53L0X_DEV_H__ */
