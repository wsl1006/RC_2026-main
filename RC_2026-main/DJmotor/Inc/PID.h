#ifndef __PID_H
#define __PID_H
#define ABS(x) ((x) > 0 ? (x) : (-x))
#include <stdint.h>

typedef struct
{
    uint8_t mode;       // 0 = PIDINC, 1 = PIDPOS
    float err[3];       // 误差历史，arr[i]递增为更老的

    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数

    float SetVal;       // 目标值（设定值）
    float CurVal;       // 当前值（反馈值）

    float integral;      // 积分累加值（位置式用）
    float prevError;     // 上一次误差（微分项/增量式用）
    float output;        // 当前输出值（可选）
} PIDType;

typedef struct
{
    float posKp; // 位置环 Kp
    float posKi; // 位置环 Ki
    float posKd; // 位置环 Kd

    float velKp; // 速度环 Kp
    float velKi; // 速度环 Ki
    float velKd; // 速度环 Kd
} DJmotorPID;

float PID_Calculate(PIDType *pid);
void PIDInit(PIDType *pid, float kp, float ki, float kd, uint8_t mode);
void PID_Reset(PIDType *pid);

#endif /*__PID_H*/
