/**
 * @file protocol.h
 * @author phil (zengfei@pudutech.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-15
 * 
 * @copyright Copyright 2019 (c) Pudu Technology Inc. All Rights Reserved. 
 * 
 */
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include <stdint.h>

struct PCMD{
    uint8_t cid; //!< command id;
    void (*callback)( uint16_t id, 
                         uint8_t *req,
                         uint8_t req_len, 
                         uint8_t *ret, 
                         uint32_t *ret_len );
};

int protocol_init( void );

/**
 * @brief 计算和校验码
 * 算法为：  
 * CheckSum = X1 ^ X2 ^ X3 ... ^Xn  
 * 
 * @param[in] buf 待校验数据
 * @param[in] len 待校验数据长度
 * @return 校验和
 */
uint8_t BCC_CheckSum(const uint8_t *buf, uint8_t len);

/**
 * @brief 触摸反馈，如果硬件检测到触摸事件 可使用该接口反馈给上位机
 * 
 * @param[in] touch_id 触摸传感器编号
 * @param[in] state    触摸状态
 * @return 0
 */
int touch_feedback( uint8_t touch_id, uint8_t state );

/**
 * @brief 托盘状态反馈
 * 
 * @param[in] pallet_id     托盘id
 * @param[in] stype         状态类型：1，托盘安装状态；2，托盘物品放置状态
 * @param[in] state         状态：状态类型为1的情况下：0，托盘未安装；1：托盘已安装；
 *                                状态类型为2的情况下：0:，托盘已放置物品；1：托盘物品已拿走；
 * @return int ok
 */
int pallet_feedback( uint8_t pallet_id, uint8_t stype, uint8_t state );


/**
 * @brief 发送can 消息指令
 * 
 * @param[in] data 待发送数据buffer指针，
 * @param[in] len  待发送数据长度，最大8字节
 * @return  返回已发送字节数成功返回 sizeof(rt_can_msg), 0 失败 
 */
uint32_t can_send( uint8_t *data, uint8_t len );

#endif
