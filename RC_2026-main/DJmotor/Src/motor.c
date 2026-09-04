/**
 * @file    DJmotor.c
 * @brief   DJI M2006/M3508 CAN motor driver.
 *
 * Ported from R2_Chassis-chassis_main/Motor/src/DJmotor.c.  The control flow
 * and PID structure are kept, only the bus selection and a few safety details
 * were generalized for this template.
 */
#include "motor.h"
#include "MathFunc.h"

DJMotor DJmotor[USE_DJNUM];

/* ==================== 上电自动寻零配置 ==================== */
/* 需要上电自动寻零的电机 ID 列表(1-based,对应反馈帧 0x200+ID) */
static const uint8_t s_auto_zero_id[] = {3U}; /* 机械臂电机 ID=3,按实际机构改 */
#define AUTO_ZERO_COUNT (sizeof(s_auto_zero_id) / sizeof(s_auto_zero_id[0]))

static bool DJmotor_NeedAutoZero(uint8_t id)
{
    for (uint32_t i = 0U; i < AUTO_ZERO_COUNT; i++)
    {
        if (s_auto_zero_id[i] == id)
        {
            return true;
        }
    }
    return false;
}

void ChangeDataByte(uint8_t *a, uint8_t *b)
{
    uint8_t temp = *a;
    *a = *b;
    *b = temp;
}

/* 按 MOTOR_DJI_CAN_BUS 取总线句柄(0=CAN1,1=CAN2) */
static inline CAN_HandleTypeDef *DJmotor_GetCanHandle(void)
{
    return &hcan2;
}

static inline float Get_Total_Ratio(DJMotorPointer motor)
{
    return motor->param.Gear_ratio * motor->param.Reduction_ratio;
}
#if M3508_NUM > 0
static inline bool is_M3508(uint8_t ID)
{
    return (ID > M2006_NUM);
}
#else
static inline bool is_M3508(uint8_t ID)
{
    return false;
}
#endif
void DJmotor_Init(void)
{
    DJmotorParam dj2006_param;
    DJmotorParam dj3508_param;
    DJmotorLimit limit;
    DJmotorStatus statusFlag;
    DJmotorArgum argum;
    DJmotorError error;

    dj2006_param.ParamID = 0x200U;
    dj2006_param.Gear_ratio = 1.0f;
    dj2006_param.Reduction_ratio = M2006_RATIO;
    dj2006_param.PulsePerRound = 8192U;
    dj2006_param.CurrentLimit_raw = 4500;

    dj3508_param.ParamID = 0x1ffU;
    dj3508_param.Gear_ratio = 1.0f;
    dj3508_param.Reduction_ratio = M3508_RATIO;
    dj3508_param.PulsePerRound = 8192U;
    dj3508_param.CurrentLimit_raw = 10000;

    limit.CurrentLimitFlag = true;
    limit.IsLooseStuck = true; /* 堵转自动失能:防止顶住障碍/限位后持续输出 */

    limit.MaxAngle_deg = 270.0f;
    limit.MinAngle_deg = -270.0f;
    limit.PosAngleLimitFlag = true; /* 启用位置角度限位(目标限幅+实际角度急停) */
    limit.PosRPMFlag = true;
    limit.PosRPMLimit = 430;

    limit.RPMLimitFlag = false;
    limit.SpeedRPMLimit = 400;
    limit.ZeroCurrentLimit_raw = 3000;
    limit.ZeroRPMLimit = 10;
    limit.ZeroDir = 1; /* 寻零方向:+1或-1,若撞的方向反了改这里 */

    statusFlag.IsSetZero = false; /* 上电不再就地清零,零点由寻零流程在机械限位处建立 */
    statusFlag.Overtimeflag = false;
    statusFlag.StuckFlag = false;
    statusFlag.ZeroFlag = false;
    statusFlag.ZeroFailFlag = false;
    statusFlag.ZeroValid = false;
    statusFlag.PosLimitFlag = false;

    argum.pulseLock = 0;
    argum.zeroCnt = 0;
    argum.GapCnt = 0;
    argum.zeroTimeoutCnt = 0;

    error.lastRxTime = 0;
    error.stuckCount = 0;
    error.timeoutCount = 0;

    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        DJmotor[i].Begin = true;
        DJmotor[i].MODE_Set = DJ_Disable; /* 上电失能:发 0 电流 */
        DJmotor[i].statusFlag = statusFlag;
        DJmotor[i].limit = limit;
        DJmotor[i].argum = argum;
        DJmotor[i].error = error;
        DJmotor[i].valSet.current_raw = 0;
        DJmotor[i].valSet.angle_deg = 0.0f;
        DJmotor[i].valSet.speed_rpm = 0;
        DJmotor[i].valSet.PulseTotal = 0;
        DJmotor[i].valNow.PulseTotal = 0;
        DJmotor[i].valPre.PulseRead = 0;
    }

    // 这里可以使用表封装的参数进行替换赋值
    for (uint32_t i = 0; i < M2006_NUM; i++)
    {
        DJmotor[i].ID = (uint8_t)(i + 1U);
        DJmotor[i].param = dj2006_param;
    }

    for (uint32_t i = 0; i < M3508_NUM; i++)
    {
        DJmotor[i + M2006_NUM].ID = (uint8_t)(i + M2006_NUM + 1U);
        DJmotor[i + M2006_NUM].param = dj3508_param;
    }

    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        PIDInit(&DJmotor[i].posPID, 0.07f, 0.005f, 0.0f, PIDPOS);
        PIDInit(&DJmotor[i].velPID, 5.5f, 0.3f, 0.01f, PIDPOS);
    }

    /* 上电自动寻零:列表内的电机直接进入寻零模式,其余保持失能 */
    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        DJmotor[i].MODE_Set = DJmotor_NeedAutoZero(DJmotor[i].ID) ? DJ_Zero : DJ_Disable;
    }
}

void DJmotor_PID_Reload(DJMotorPointer motor, DJmotorPID pid_reload)
{
    PIDInit(&motor->posPID, pid_reload.posKp, pid_reload.posKi, pid_reload.posKd, 0);
    PIDInit(&motor->velPID, pid_reload.posKp, pid_reload.posKi, pid_reload.posKd, 1);
}

void DJmotor_SetZero(DJMotorPointer motor)
{
    motor->statusFlag.IsSetZero = false;
    motor->valNow.angle_deg = 0.0f;
    motor->valNow.PulseTotal = 0;
    motor->argum.pulseLock = 0;
}

void DJmotor_AngleCalculate(DJMotorPointer motor)
{
    motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseRead - motor->valPre.PulseRead);

    if (ABS(motor->valNow.PulseGap) > 4096)
    {
        motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseGap -
                                           GetSign(motor->valNow.PulseGap) *
                                               (int32_t)motor->param.PulsePerRound);
    }

    motor->valNow.PulseTotal += motor->valNow.PulseGap;
    motor->valNow.angle_deg = (float)motor->valNow.PulseTotal * 360.0f /
                              ((float)motor->param.PulsePerRound * Get_Total_Ratio(motor));

    if (motor->Begin) // 废弃字段
    {
        motor->argum.pulseLock = motor->valNow.PulseTotal;
    }

    if (motor->statusFlag.IsSetZero)
    {
        DJmotor_SetZero(motor);
        motor->statusFlag.IsSetZero = false;
    }

    motor->valPre = motor->valNow;
}

void DJmotor_Receive(CAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    if ((Rxheader.IDE != CAN_ID_STD) ||
        (Rxheader.RTR != CAN_RTR_DATA) ||
        (Rxheader.StdId < 0x201U) || (Rxheader.StdId > 0x208U))
    {
        return;
    }

    uint8_t card_id = (uint8_t)(Rxheader.StdId - 0x200U); /* 1..8 */

    /* Init 保证 ID = 索引 + 1,直接索引免循环查找 */
    if (card_id > USE_DJNUM)
    {
        return;
    }

    DJMotorPointer motor = &DJmotor[card_id - 1U];

    motor->valNow.PulseRead = (int16_t)(((uint16_t)Rx_data[0] << 8) | Rx_data[1]);
    int16_t speed_raw = (int16_t)(((uint16_t)Rx_data[2] << 8) | Rx_data[3]);
    motor->valNow.current_raw = (int16_t)(((uint16_t)Rx_data[4] << 8) | Rx_data[5]);

    if (is_M3508(motor->ID))
    {
        motor->valNow.temperature_C = (int8_t)Rx_data[6];
        motor->valNow.current_A = (float)motor->valNow.current_raw * 0.0012207f;
    }
    else
    {
        motor->valNow.current_A = (float)motor->valNow.current_raw / 10000.0f * 10.0f;
    }

    motor->valNow.speed_rpm = (float)speed_raw / Get_Total_Ratio(motor);

    motor->error.lastRxTime = 0;
    DJmotor_AngleCalculate(motor);
}

void DJmotor_CurrentTransmit(DJMotorPointer motor)
{
    static uint8_t tx_data[8] = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;
    uint8_t tag = 0;

    /* 电流限幅由各模式函数负责,此处只打包发送 */
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    if (motor->ID <= 4U)
    {
        tx_header.StdId = 0x200U;
        tag = (uint8_t)((motor->ID - 1U) * 2U);
    }
    else
    {
        tx_header.StdId = 0x1FFU;
        tag = (uint8_t)((motor->ID - 5U) * 2U);
    }

    EncodeS16Data(&motor->valSet.current_raw, &tx_data[tag]);
    ChangeDataByte(&tx_data[tag], &tx_data[tag + 1U]);

    if (motor->ID == 4U || motor->ID == 8U)
    {
        HAL_CAN_AddTxMessage(DJmotor_GetCanHandle(), &tx_header, tx_data, &tx_mailbox);
    }
}

void DJmotor_SpeedMode(DJMotorPointer motor)
{
    if (motor->limit.RPMLimitFlag)
    {
        motor->valSet.speed_rpm = ClampPeak(motor->velPID.SetVal, motor->limit.SpeedRPMLimit);
    }
    motor->velPID.SetVal = (float)motor->valSet.speed_rpm * Get_Total_Ratio(motor);
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * Get_Total_Ratio(motor);
    motor->valSet.current_raw = PID_Calculate(&motor->velPID);
    motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

void DJmotor_PositionMode(DJMotorPointer motor)
{
    /* 实际角度硬保护:越过机械行程立即急停失能,防止位置环跑飞顶坏机构 */
    if (motor->valNow.angle_deg > motor->limit.MaxAngle_deg ||
        motor->valNow.angle_deg < motor->limit.MinAngle_deg)
    {
        motor->valSet.current_raw = 0;
        motor->MODE_Set = DJ_Disable;
        motor->Begin = false;
        motor->statusFlag.PosLimitFlag = true;
        /* 清除PID积分项，防止重新启用时产生冲击 */
        PID_Reset(&motor->posPID);
        PID_Reset(&motor->velPID);
        return;
    }

    if (motor->limit.PosAngleLimitFlag)
    {
        motor->valSet.angle_deg = Clamp(motor->valSet.angle_deg, motor->limit.MinAngle_deg, motor->limit.MaxAngle_deg);
    }

    motor->valSet.PulseTotal = (int32_t)(motor->valSet.angle_deg * Get_Total_Ratio(motor) * (float)motor->param.PulsePerRound / 360.0f);
    motor->posPID.SetVal = (float)motor->valSet.PulseTotal;
    motor->posPID.CurVal = (float)motor->valNow.PulseTotal;

    float pos_output = PID_Calculate(&motor->posPID);

    /* 抗积分饱和：如果输出被限幅，则回退位置环积分项 */
    if (motor->limit.PosRPMFlag)
    {
        float output_limit = motor->limit.PosRPMLimit * Get_Total_Ratio(motor);
        if (pos_output > output_limit)
        {
            /* 输出饱和，回退积分累加 */
            motor->posPID.err[2] *= 0.9f;
            pos_output = output_limit;
        }
        else if (pos_output < -output_limit)
        {
            motor->posPID.err[2] *= 0.9f;
            pos_output = -output_limit;
        }
    }

    motor->velPID.SetVal = pos_output;
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * Get_Total_Ratio(motor);

    motor->valSet.current_raw = PID_Calculate(&motor->velPID);
    motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

void DJmotor_ZeroMode(DJMotorPointer motor)
{
    /* 朝限位方向低速推(方向由 limit.ZeroDir 决定,撞反了改 Init 里的值) */
    motor->velPID.SetVal = (float)motor->limit.ZeroRPMLimit * Get_Total_Ratio(motor) * (float)motor->limit.ZeroDir;
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * Get_Total_Ratio(motor);
    motor->valSet.current_raw = PID_Calculate(&motor->velPID);
    motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->limit.ZeroCurrentLimit_raw);

    /* 堵转判定:把寻零目标转速折算成每毫秒编码器增量,实际增量不足其 1/3 即视为堵转。
       阈值随 ZeroRPMLimit 自动缩放,寻零速度调快调慢都有效(固定增量阈值在低速下会误判) */
    float exp_gap = (float)motor->limit.ZeroRPMLimit * Get_Total_Ratio(motor) *
                    (float)motor->param.PulsePerRound / 60000.0f;
    float stall_gap = exp_gap / 3.0f;
    if (stall_gap < 2.0f)
    {
        stall_gap = 2.0f;
    }

    if ((float)ABS(motor->valNow.PulseGap) < stall_gap)
    {
        if (++motor->argum.zeroCnt > ZERO_STALL_TICKS)
        {
            /* 寻零完成:清 PID 历史,在机械限位处清零,松电失能,
               等待任务层看到 ZeroFlag 后重新使能 */
            PID_Reset(&motor->posPID);
            PID_Reset(&motor->velPID);
            DJmotor_SetZero(motor);
            motor->valSet.current_raw = 0;
            motor->statusFlag.ZeroFlag = true;
            motor->statusFlag.ZeroValid = true; /* 零点建立,此后才允许位置模式 */
            motor->Begin = false;
            motor->argum.zeroCnt = 0;
            motor->argum.zeroTimeoutCnt = 0;
        }
    }
    else
    {
        motor->argum.zeroCnt = 0;
    }

    /* 超时保护:规定时间内未完成寻零 → 失能并置失败标志,避免无限顶墙 */
    if (++motor->argum.zeroTimeoutCnt > ZERO_TIMEOUT_TICKS)
    {
        PID_Reset(&motor->posPID);
        PID_Reset(&motor->velPID);
        motor->valSet.current_raw = 0;
        motor->MODE_Set = DJ_Disable;
        motor->statusFlag.ZeroFailFlag = true;
        motor->Begin = false;
    }
}

static void DJmotor_Monitor(DJMotorPointer motor)
{

    if (motor->valNow.PulseGap < 5 && motor->valNow.current_raw > 3000)
    {
        if (motor->error.stuckCount++ > 100U) /* 连续1秒堵转判定 */
        {
            motor->error.stuckCount = 0;
            motor->statusFlag.StuckFlag = true;
            if (motor->limit.IsLooseStuck)
            {
                motor->MODE_Set = DJ_Disable;
            }
        }
    }
    else
    {
        motor->error.stuckCount = 0;
    }

    if (motor->error.lastRxTime++ > 50U)
    {
        if (motor->error.timeoutCount++ > 20U)
        {
            motor->error.timeoutCount = 0;
            motor->MODE_Set = DJ_Disable;
            motor->statusFlag.Overtimeflag = true;
        }
    }
}

static void DJmotor_SwitchMode(DJMotorPointer motor)
{
    if (motor->MODE_Set != motor->MODE_Cur)
    {
        /* 零点未建立时拒绝位置模式:错误零点下任何绝对位置指令都可能全速跑飞 */
        if (motor->MODE_Set == DJ_Position && !motor->statusFlag.ZeroValid)
        {
            return;
        }
        motor->MODE_Cur = motor->MODE_Set;
        motor->valSet.current_raw = 0;
        motor->valSet.speed_rpm = 0;
        motor->valSet.angle_deg = motor->valNow.angle_deg;
        /* 清误差历史与位置环累加的目标速度(velPID.SetVal),避免残留值冲击新模式 */
        PID_Reset(&motor->posPID);
        PID_Reset(&motor->velPID);
        motor->statusFlag.ZeroFlag = false;
        motor->statusFlag.Overtimeflag = false;
        motor->statusFlag.StuckFlag = false;
    }
}

void DJmotor_Func(void)
{
    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {

        if (DJmotor[i].Begin)
        {
            DJmotor_Monitor(&DJmotor[i]); /* 堵转+通信超时保护 */
            DJmotor_SwitchMode(&DJmotor[i]);

            switch (DJmotor[i].MODE_Cur)
            {
            case DJ_Disable:
                DJmotor[i].valSet.current_raw = 0;
                DJmotor_CurrentTransmit(&DJmotor[i]);
                continue;
                break;
            case DJ_RPM:
                DJmotor_SpeedMode(&DJmotor[i]);
                break;
            case DJ_Position:
                DJmotor_PositionMode(&DJmotor[i]);
                break;
            case DJ_Zero:
                DJmotor_ZeroMode(&DJmotor[i]);
                break;
            case DJ_Current:
                /* 直通电流:任务层每周期写 valSet.current_raw,这里补限幅 */
                ClampPeak(DJmotor[i].valSet.current_raw, DJmotor[i].param.CurrentLimit_raw);
                break;
            default:
                break;
            }
        }
        else
        {
            /* Begin=false(未初始化/寻零完成):强制 0 电流,防止残留累加电流持续输出 */
            DJmotor[i].valSet.current_raw = 0;
        }

        DJmotor_CurrentTransmit(&DJmotor[i]);
    }
}
