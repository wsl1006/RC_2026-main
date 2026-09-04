#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "PID.h"

#define M2006_RATIO 36.0f
#define M3508_RATIO 19.2f
#define USE_DJ 1
#define M2006_NUM 0
#define M3508_NUM 4
#define USE_DJNUM (M2006_NUM + M3508_NUM)
#define Zero_Distance   100
/* 寻零堵转判定:连续150个控制周期(2ms×150=300ms)编码器近乎不动且电流顶到限幅附近 */
#define ZERO_STALL_TICKS   150U
/* 寻零超时保护:超过5秒未完成则失能并置 ZeroFailFlag(2ms×2500=5s) */
#define ZERO_TIMEOUT_TICKS 2500U

#define PIDPOS  0   // 位置式 PID
#define PIDINC  1   // 增量式 PID

typedef enum
{
    DJ_Disable = 0,  /* 关:transmit 0current */
    DJ_RPM = 1,      /*速度mode*/
    DJ_Position = 2, /*位置mode*/
    DJ_Zero = 3,     /*寻零mode*/
    DJ_Current = 4,  /*电流/扭矩*/
} DJmotor_mode_t;

/* ========== 反馈值结构体 ========== */
typedef struct
{
    volatile int16_t current_raw;  // 直接设置电流
    volatile float angle_deg;      // 输出角度，degree
    volatile int16_t speed_rpm;    // valSet：输出轴rpm;vaLNow：转子rpm（原始反馈）
    volatile float current_A;      // 反馈电流，A
    volatile int16_t PulseRead;    // rawencoder pulse
    volatile int16_t PulseGap;     // pulse delta
    volatile int32_t PulseTotal;   // accumulated pulse
    volatile int8_t temperature_C; //
} DJmotorVal;

/* ========== 参数结构体 ========== */
typedef struct
{
    uint16_t PulsePerRound;   // 8191
    float Gear_ratio;         // mechanism ratio
    float Reduction_ratio;    // motor reducer ratio
    uint32_t ParamID;         // CAN receive ID base
    int16_t CurrentLimit_raw; // output current limit，raw
} DJmotorParam;

typedef struct
{
    int32_t pulseLock;
    int32_t zeroCnt;
    int32_t GapCnt;
    uint32_t zeroTimeoutCnt;   // 寻零超时计数(每个控制周期+1)
} DJmotorArgum;

typedef struct 
{   uint32_t lastRxTime;
    uint32_t stuckCount;
    uint32_t timeoutCount;
}DJmotorError;

typedef struct
{
    bool RPMLimitFlag;
    bool PosAngleLimitFlag;
    bool PosRPMFlag;
    bool CurrentLimitFlag;        // degree
    float MaxAngle_deg;           // degree
    float MinAngle_deg;           // rmp
    int16_t SpeedRPMLimit;        // rpm
    int32_t PosRPMLimit;          // rpm
    int16_t ZeroRPMLimit;         // rpm
    int8_t  ZeroDir;              // 寻零方向:+1或-1,若撞的方向反了改这里
    int16_t ZeroCurrentLimit_raw; // raw
    bool IsLooseStuck;
} DJmotorLimit;

typedef struct
{
    bool IsSetZero;    // 是否已设置零点（是否处于寻零状态）
    bool Overtimeflag; // 通信超时标志
    bool StuckFlag;    // 堵转标志
    bool ZeroFlag;     // 寻零完成标志
    bool ZeroFailFlag; // 寻零超时/失败标志
    bool ZeroValid;    // 零点有效标志:寻零完成过才允许位置模式
    bool PosLimitFlag; // 实际角度超限急停标志
} DJmotorStatus;

/* ========== 主电机结构体 ========== */
typedef struct
{
    uint8_t ID;
    volatile bool Begin;              // true运行MoDE：false失能
    volatile DJmotor_mode_t MODE_Set;  // DJDisable即失能（发0电流
    volatile DJmotor_mode_t MODE_Cur; // 实际运行模式，任务层可读

    DJmotorParam param;
    DJmotorVal valSet;
    DJmotorVal valNow;
    DJmotorVal valPre;
    DJmotorStatus statusFlag;
    DJmotorLimit limit;
    DJmotorArgum argum;
    DJmotorError error;
    PIDType posPID;
    PIDType velPID;
} DJMotor, *DJMotorPointer;

#if USE_DJ
extern DJMotor DJmotor[USE_DJNUM];

void DJmotor_Init(void);
void DJmotor_Func(void);
void DJmotor_Receive(CAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);
void DJmotor_PID_Relogd(DJMotorPointer motor, DJmotorPID pid_reload);
void DJmotor_CurrentTransmit(DJMotorPointer motor);
// void DJmotor_SwitchMode(DJMotorPointer motor);
void DJmotor_SpeedMode(DJMotorPointer motor);
 void DJmotor_PositionMode(DJMotorPointer motor);
 void DJmotor_ZeroMode(DJMotorPointer motor);

 void DJmotor_SetZero(DJMotorPointer motor);

#endif
#endif /*__MOTOR_H*/
