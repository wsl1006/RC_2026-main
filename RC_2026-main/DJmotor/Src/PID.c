#include "PID.h"
#include "motor.h"

float PID_Calculate(PIDType *pid)
{
    pid->err[0] = pid->SetVal - pid->CurVal;
    switch (pid->mode)
    {
    case PIDINC:
        pid->output = pid->Kp * (pid->err[0] - pid->err[1]) + pid->Ki * pid->err[0] +
                      pid->Kd * (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);
        pid->err[2] = pid->err[1];
        pid->err[1] = pid->err[0];
        break;

    case PIDPOS:
        pid->err[2] = 0.5f * pid->err[0] + 0.5f * pid->err[2];

        /* 积分项限幅：防止积分饱和导致超调/反转 */
        #define INTEGRAL_LIMIT 10000.0f
        if (pid->err[2] > INTEGRAL_LIMIT)
            pid->err[2] = INTEGRAL_LIMIT;
        else if (pid->err[2] < -INTEGRAL_LIMIT)
            pid->err[2] = -INTEGRAL_LIMIT;

        pid->output = pid->Kp * pid->err[0] +
                      pid->Ki * pid->err[2] +
                      pid->Kd * ((pid->err[0]) - pid->err[1]);
        pid->err[1] = pid->err[0];
        break;
    default:
        break;
    }
    return pid->output;
}

/* ==================== PID 函数实现 ==================== */

void PIDInit(PIDType *pid, float kp, float ki, float kd, uint8_t mode)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->mode = mode;
    pid->SetVal = 0.0f;
    pid->CurVal = 0.0f;
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->output = 0.0f;
}

void PID_Reset(PIDType *pid)
{
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->output = 0.0f;
}