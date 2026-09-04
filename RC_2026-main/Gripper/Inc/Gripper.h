#ifndef __GRIPPER_H
#define __GRIPPER_H

#include <stdint.h>
#include "Solenoid.h"

typedef enum
{
    IDLE,

    EXTEND_PICK,      // 取球过程中，M3508大角度转动，爪子伸出
    RETRACT_PICK,     // 取球过程中，M3508大角度转动，爪子缩回

    EXTEND_PLACE,     // 放球过程中，M3508小角度转动，爪子伸出
    RETRACT_PLACE,    // 放球过程中，M3508小角度转动，爪子缩回

    GRIP_OPEN,        // 爪子张开
    GRIP_CLOSE        // 爪子抓紧
}Gripper_state;

/* CAN 接收中断中调用:仅入队,不执行动作(执行会阻塞中断) */
void Gripper_RecvCmd(uint8_t cmd, uint8_t *data, uint8_t len);
/* 主循环低速任务(10ms)调用:出队并执行夹爪/电磁阀动作 */
void Gripper_Task(void);

#endif
