#include "CAN_IRQHandler.h"
#include "ZDrive.h"
#include "motor.h"
#include "Gripper.h"
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
/* 全局命令区，CAN 中断写，主循环读 */
volatile uint8_t g_beep_new = 0;   /* 1 = 来了新的蜂鸣器命令 */
volatile uint8_t g_beep_count = 0; /* 要响几声 */

volatile uint8_t g_led_new = 0; /* 1 = 来了新的流水灯命令 */
volatile uint8_t g_led_cmd = 0; /* 0 关灯，1 开灯 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
         if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            if (RxHeader.IDE == CAN_ID_EXT)
            {
                if (RxHeader.ExtId == 0x01020101) // 蜂鸣器
                {
                    g_beep_count = RxData[0];
                    g_beep_new = 1;
                }
                else if (RxHeader.ExtId == 0x01020201) // 流水灯
                {
                    g_led_cmd = RxData[0];
                    g_led_new = 1;
                }
                else if ((RxHeader.ExtId & 0xFFFFFF00) == 0x01010500)
                {
                    /* 中断内只入队,夹爪/电磁阀动作由主循环 Gripper_Task 执行,
                       避免在中断上下文做 GPIO 时序/电机控制/发CAN反馈 */
                    uint8_t cmd = (uint8_t)(RxHeader.ExtId & 0xFF);
                    Gripper_RecvCmd(cmd, RxData, RxHeader.DLC);
                }
            }
        }
    }
    else if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            if (RxHeader.IDE == CAN_ID_STD && (RxHeader.StdId >= 0x201U && RxHeader.StdId <= 0x208U))
            {
                DJmotor_Receive(RxHeader, RxData);
            }
        }
    }
}