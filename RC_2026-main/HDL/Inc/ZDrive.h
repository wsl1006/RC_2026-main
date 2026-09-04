/**
 * @file    ZDrive.h
 * @brief   ZDrive J60/Z-Smart motor CAN driver, ported from R2 chassis.
 */
#ifndef ZDRIVE_H
#define ZDRIVE_H

#include <stdbool.h>
#include <stdlib.h>
#include "main.h"
#include "can.h"
// #include "FD_Canqueue.h"
// #include "motor_config.h"
#define USE_ZMDR 1
#define USE_ZDRIVE_NUM 0            // 就一台电机
#define N2DEG(x)  ((x) * 360.0f)    // 圈→度（自己补，原版在 includes.h 里）
#define DEG2N(x)  ((x) / 360.0f)    // 度→圈
#define MOTOR_ZDRIVE_SPLIT_COUNT 0  // 不分流（=0 全走第一路，正好绕开分流逻辑）
#define MOTOR_ZDRIVE_COUNT 1
#define MOTOR_ZDRIVE_CAN_BUS_1 0    
#define MOTOR_ZDRIVE_CAN_BUS_2 1

#ifdef __cplusplus
extern "C"
{
#endif

// #define USE_ZDRIVE_NUM MOTOR_ZDRIVE_COUNT
#define Zdrive_Hz 10

#define POU 10000.f
#define POD -10000.f
#define Velocity_Limit 150.f
#define Current_Limit 40.f

#define PID_POS_P 0x12
#define PID_POS_I 0x13
#define PID_VEL_P 0x14
#define PID_VEL_I 0x15

    typedef enum
    {
        Zdrive_Disable = 0,
        Zdrive_Current,
        Zdrive_Speed,
        Zdrive_Postion,
        Zdrive_Test,
        Zdrive_RVCalibration,
        Zdrive_EncoderLineCalibration,
        Zdrive_EncoudeOffsetCalibration,
        Zdrive_VKCalibration,
        Zdrive_SaveSetting,
        Zdrive_EraseSetting,
        Zdrive_ClearErr,
        Zdrive_Brake
    } ZdriveMode;

    typedef enum
    {
        Zdrive_Well = 0,
        Zdrive_InsufficientVoltage,
        Zdrive_OverVoltage,
        Zdrive_InstabilityCurrent,
        Zdrive_OverCurrent,
        Zdrive_OverSpeed,
        Zdrive_ExcessiveR,
        Zdrive_ExcessiveInductence,
        Zdrive_LoseEncoder1,
        Zdrive_PolesErr,
        Zdrive_VKCalibrationErr,
        Zdrive_ModeErr,
        Zdrive_ParameterErr,
        Zdrive_Hot
    } ZdriveErr;

    typedef enum
    {
        SES = 0x01,
        ENCODER,
        ENL,
        NodeID,
        CAN_HZ,
        HEARTBEAT,
        Volta_LL,
        Pos_UL,
        Pos_LL,
        Vel_Limit,
        Poles_Num,
        CurLimit,
        CurrCAL,
        Start_Mode,
        Answer_Mode,
        FilterCoeff,
        ToleranceCoeff,
        Pos_PID_P,
        Pos_PID_I,
        Vel_PID_P,
        Vel_PID_I,
        Acc_Acu,
        Acc_Dec,
        Pos_Vel_TimeGap,
        Mode = 0x1F,
        Warning,
        Err,
        CurIn,
        VelIn,
        PosIn,
        LIF,
        Cur_M = 0x2B,
        Vel = 0x2D, // 当前速度
        Pur = 0x2E, // 当前位置,可以设置
        PVT_Frame = 0x39,
    } ZdriveCmd;

    typedef struct
    {
        float speed_rpm;    /* 速度(rpm) */
        float pos_deg;      /* 位置(deg) */
        float posIn_deg;    /* 当前位置(角度,deg,Pur 命令码) */
        float current_A;    /* 电流(A) */
        float torque_Nm;    /* 力矩(N·m) */
        float velLimit_rpm; /* 速度限制(rpm) */
        float accAcu_rps2;  /* 加速度(rps²) */
        float accDec_rps2;  /* 减速度(rps²) */
    } ZdriveValue;

    typedef struct
    {
        uint16_t GapCnt;
        uint32_t lastRxtime;
        uint32_t timeoutTicks;
        uint32_t stuckCnt;
        float lockAngle;
    } ZdriveArgum;

    typedef struct
    {
        float GearRatio;
        float ReductionRatio;
        float kpPos; /* 位置环 P */
        float kiPos; /* 位置环 I */
        float kpVel; /* 速度环 P */
        float kiVel; /* 速度环 I */
    } ZdriveParam;

    typedef struct
    {
        bool timeoutCheck;
        bool stuckCheck;
        bool Arriveflag;
        bool Zeroflag;
        float ZeroPoint;
        ZdriveErr err;
    } ZdriveStatus;

    typedef struct
    {
        bool PVTflag;
        float Total_Time;
    } ZdrivePVTParam;

    typedef struct
    {
        ZdriveMode mode;     /* 目标模式,任务层写;Disable 即停止 */
        ZdriveMode modeRead; /* 驱动确认的当前模式,由 RX 更新 */
        ZdriveParam param;
        ZdriveArgum argum;
        ZdriveValue valReal;
        ZdriveValue valPre;
        ZdriveValue valSetNow;
        ZdriveValue valSetPre;
        ZdriveStatus statusFlag;
        ZdrivePVTParam pvtparam;
        volatile bool Begin; /* 初始化完成标志:false 时 Func 跳过该电机,由任务层置 true */
    } Zdrive;

// #if USE_ZMDR
    extern Zdrive Zmotor[USE_ZDRIVE_NUM];

    void ZdriveInit(void);
    void ZdriveFunc(void);
    void ZdriveReceive(CAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data, uint8_t bus);
    void ZdriveSet(float data, uint8_t id, uint8_t set_code);
    void ZdriveAsk(uint8_t id, uint8_t ask_code);
    void ZdriveSetPVT(float speed, float angle, uint8_t id);
    void ZdriveSetMIT(uint8_t id);
    void ZdriveClearErr(uint8_t id);
    void ZdriveSetVelLimit(float vel, uint8_t id);
    void ZdriveSetPID(float value, uint8_t id, uint8_t pid_code);
    void ZdriveSetPosVelLimit(float vel_limit, uint8_t id);
    void ZdriveSetAccel(float ace, uint8_t id);
    /** 覆盖指定电机的 param,位置环/速度环 PID 哪一项改动就下发哪一项寄存器 */
    void ZdriveParamConfig(uint8_t id, ZdriveParam param);
// #endif /* USE_ZMDR */

#ifdef __cplusplus
}
#endif

void CLK_set(void);
void ctrl(uint8_t cmd);

#endif /* ZDRIVE_H */
