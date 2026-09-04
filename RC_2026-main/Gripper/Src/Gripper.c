#include "Gripper.h"
#include "motor.h"
#include "PID.h"
#include "can.h"
#include <math.h>

/* ==================== 命令队列 ====================
 * CAN 接收中断(Gripper_RecvCmd)只负责入队,绝不执行动作;
 * 主循环 10ms 任务(Gripper_Task)出队执行,避免在中断里做
 * GPIO 时序/电机控制等业务导致时序错乱。
 * 单生产者(ISR写head)单消费者(任务读tail),无需加锁。 */
#define GRIPPER_FIFO_SIZE 8U

typedef struct
{
    uint8_t cmd;  /* 命令号(0x00~0x0C) */
    uint8_t data; /* 载荷首字节(大多数命令只用 data[0]) */
    uint8_t len;  /* DLC */
} GripperCmd_t;

static GripperCmd_t g_cmd_fifo[GRIPPER_FIFO_SIZE];
static volatile uint8_t g_fifo_head = 0; /* 写指针,仅ISR修改 */
static volatile uint8_t g_fifo_tail = 0; /* 读指针,仅主循环修改 */
static volatile uint8_t g_fifo_overflow = 0; /* 队列满丢帧计数 */

static uint8_t grip_flag = 0;   // 0=松开, 1=夹紧(0x0A切换用)
static uint8_t move_flag = 0;   // 0=水平, 1=抬起(0x0B切换用)

static void Send_Feedback(uint8_t cmd, uint8_t status)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t data[2] = {status, 0x00};
    uint32_t mailbox;

    tx.IDE = CAN_ID_EXT;                   // 扩展帧
    tx.ExtId = (uint32_t)0x05010100 | cmd; // 0x050101xx
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 2;
    tx.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

/* 中断侧:仅把命令压入 FIFO,不执行任何动作 */
void Gripper_RecvCmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t next = (uint8_t)((g_fifo_head + 1U) % GRIPPER_FIFO_SIZE);

    if (next == g_fifo_tail) /* FIFO 满:丢弃新帧并计数(命令太密,主循环消化不了) */
    {
        g_fifo_overflow++;
        return;
    }

    g_cmd_fifo[g_fifo_head].cmd = cmd;
    g_cmd_fifo[g_fifo_head].data = (data != NULL) ? data[0] : 0;
    g_cmd_fifo[g_fifo_head].len = len;
    g_fifo_head = next;
}

/* 任务侧:执行单条命令(在主循环/低速任务上下文调用,可安全操作 GPIO/电机/发CAN) */
static void Gripper_Execute(uint8_t cmd, uint8_t data0, uint8_t len)
{
    switch (cmd)
    {
    case 0x00: // 主控自检广播
        Send_Feedback(cmd, 0x00);
        break;

    case 0x01:
        if (data0 == 1) // 使能
        {
            DJmotor[2].Begin = 1;
            DJmotor[2].MODE_Set = DJ_Position;
        }
        else
        {
            DJmotor[2].Begin = 0;             // 失能
            DJmotor[2].MODE_Set = DJ_Disable;
        }
        Send_Feedback(cmd, data0);
        break;

    case 0xFF:
        DJmotor[2].Begin = 1;          // 复位需要重新使能,否则切换无效
        DJmotor[2].MODE_Set = DJ_Zero; // 重新寻零,角度回到机械零点
        solenoid_on(solenoid_num, 0x00); // 爪子松开
        Send_Feedback(cmd, data0);
        break;

    case 0x09:                              // 爪子准备
        DJmotor[2].valSet.angle_deg = 0.0f; // 横着
        solenoid_on(solenoid_num, 0x00);    // 爪子松开
        Send_Feedback(cmd, data0);
        break;

    case 0x0A: // 爪子抓/放切换
        if (grip_flag == 0)
        {
            solenoid_on(solenoid_num, 0x01); // 夹紧
            grip_flag = 1;
        }
        else
        {
            solenoid_on(solenoid_num, 0x00); // 松开
            grip_flag = 0;
        }
        Send_Feedback(cmd, data0);
        break;

    case 0x0B: // 机械臂移动
        if (move_flag == 0)
        {
            DJmotor[2].valSet.angle_deg = 0.0f;
            move_flag = 1;
        }
        else
        {
            DJmotor[2].valSet.angle_deg = -70.0f;
            move_flag = 0;
        }
        Send_Feedback(cmd, data0);
        break;

    case 0x0C:                              // 爪子回零
        solenoid_on(solenoid_num, 0x00);    // 松开爪子
        DJmotor[2].valSet.angle_deg = 0.0f; // 回到水平角度
        Send_Feedback(cmd, data0);
        break;

    default:
        break;
    }
}

/* 主循环 10ms 任务:逐条消费命令队列 */
void Gripper_Task(void)
{
    while (g_fifo_tail != g_fifo_head)
    {
        GripperCmd_t cmd = g_cmd_fifo[g_fifo_tail];
        g_fifo_tail = (uint8_t)((g_fifo_tail + 1U) % GRIPPER_FIFO_SIZE);
        Gripper_Execute(cmd.cmd, cmd.data, cmd.len);
    }
}
