/**
 * @file    CAN_Diagnostic.h
 * @brief   CAN通信链路检测和电磁阀诊断工具头文件
 * @date    2026-09-04
 */

#ifndef __CAN_DIAGNOSTIC_H
#define __CAN_DIAGNOSTIC_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ==================== 函数接口 ==================== */

/**
 * @brief  初始化CAN诊断模块
 */
void CAN_Diag_Init(void);

/**
 * @brief  CAN接收事件统计更新（在接收中断中调用）
 * @param  hcan: CAN句柄
 * @param  rxHeader: 接收头
 * @param  rxData: 接收数据
 */
void CAN_Diag_UpdateRx(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);

/**
 * @brief  CAN发送事件统计更新（在发送函数中调用）
 * @param  hcan: CAN句柄
 */
void CAN_Diag_UpdateTx(CAN_HandleTypeDef *hcan);

/**
 * @brief  CAN错误统计更新（在错误回调中调用）
 * @param  hcan: CAN句柄
 * @param  error_code: 错误代码
 */
void CAN_Diag_UpdateError(CAN_HandleTypeDef *hcan, uint32_t error_code);

/**
 * @brief  电磁阀状态统计更新
 * @param  channel: 通道号 (1-3)
 * @param  cmd: 命令字节
 */
void Solenoid_Diag_Update(uint8_t channel, uint8_t cmd);

/**
 * @brief  系统时钟更新（每1ms在定时器中断中调用）
 */
void CAN_Diag_TickUpdate(void);

/**
 * @brief  打印完整诊断报告
 */
void CAN_Diag_PrintReport(void);

/**
 * @brief  实时监控模式（循环调用，终端显示实时状态）
 */
void CAN_Diag_LiveMonitor(void);

/**
 * @brief  重置所有统计数据
 */
void CAN_Diag_ResetStats(void);

/**
 * @brief  启用/禁用诊断功能
 * @param  enable: 1=启用, 0=禁用
 */
void CAN_Diag_Enable(uint8_t enable);

/**
 * @brief  查询诊断是否启用
 * @return 1=已启用, 0=已禁用
 */
uint8_t CAN_Diag_IsEnabled(void);

#endif /* __CAN_DIAGNOSTIC_H */
