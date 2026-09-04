/**
 * @file    CAN_Diagnostic.c
 * @brief   CAN通信链路检测和电磁阀诊断工具
 * @date    2026-09-04
 */

#include "CAN_Diagnostic.h"
#include "can.h"
#include "motor.h"
#include "Solenoid.h"
#include <stdio.h>
#include <string.h>

/* ==================== 诊断数据结构 ==================== */
typedef struct {
    uint32_t rx_count;          // 接收计数
    uint32_t tx_count;          // 发送计数
    uint32_t error_count;       // 错误计数
    uint32_t last_rx_time;      // 上次接收时间(ms)
    uint32_t bus_off_count;     // 总线关闭计数
    uint32_t last_error_code;   // 最后错误码
} CAN_Stats_t;

typedef struct {
    uint8_t online;             // 设备在线标志
    uint32_t rx_count;          // 接收帧计数
    uint32_t last_rx_time;      // 上次接收时间
    int16_t last_pulse;         // 最后编码器值
    int16_t last_current;       // 最后电流值
    uint32_t timeout_count;     // 超时计数
    uint32_t pulse_jump_count;  // 编码器跳变计数
} Motor_Stats_t;

typedef struct {
    uint8_t state;              // 当前状态 (0-15)
    uint8_t last_cmd;           // 最后命令
    uint32_t switch_count;      // 切换次数
    uint32_t last_switch_time;  // 最后切换时间
} Solenoid_Stats_t;

/* ==================== 全局变量 ==================== */
static CAN_Stats_t g_can1_stats = {0};
static CAN_Stats_t g_can2_stats = {0};
static Motor_Stats_t g_motor_stats[8] = {0};  // 最多8个电机
static Solenoid_Stats_t g_solenoid_stats[3] = {0};  // 3个电磁阀通道

static uint8_t g_diagnostic_enable = 0;  // 诊断使能标志
static uint32_t g_system_tick = 0;       // 系统时钟(ms)

/* ==================== 函数声明 ==================== */
void CAN_Diag_Init(void);
void CAN_Diag_Update(void);
void CAN_Diag_PrintReport(void);
void CAN_Diag_ResetStats(void);

/* ==================== CAN统计更新 ==================== */
void CAN_Diag_UpdateRx(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    if (!g_diagnostic_enable) return;

    CAN_Stats_t *stats = (hcan->Instance == CAN1) ? &g_can1_stats : &g_can2_stats;
    stats->rx_count++;
    stats->last_rx_time = g_system_tick;

    // 电机反馈统计
    if (hcan->Instance == CAN2 && rxHeader->IDE == CAN_ID_STD)
    {
        if (rxHeader->StdId >= 0x201 && rxHeader->StdId <= 0x208)
        {
            uint8_t motor_id = (uint8_t)(rxHeader->StdId - 0x200);
            if (motor_id <= 8)
            {
                Motor_Stats_t *motor = &g_motor_stats[motor_id - 1];
                motor->online = 1;
                motor->rx_count++;
                motor->last_rx_time = g_system_tick;

                int16_t pulse = (int16_t)(((uint16_t)rxData[0] << 8) | rxData[1]);
                int16_t current = (int16_t)(((uint16_t)rxData[4] << 8) | rxData[5]);

                // 检测编码器跳变
                if (motor->rx_count > 1)
                {
                    int16_t pulse_gap = pulse - motor->last_pulse;
                    if (pulse_gap > 4096 || pulse_gap < -4096)
                    {
                        // 正常跨零不算，异常大跳变才计数
                        if ((pulse_gap > 5000 || pulse_gap < -5000) &&
                            (pulse_gap < 7000 && pulse_gap > -7000))
                        {
                            motor->pulse_jump_count++;
                        }
                    }
                }
                motor->last_pulse = pulse;
                motor->last_current = current;
            }
        }
    }
}

void CAN_Diag_UpdateTx(CAN_HandleTypeDef *hcan)
{
    if (!g_diagnostic_enable) return;

    CAN_Stats_t *stats = (hcan->Instance == CAN1) ? &g_can1_stats : &g_can2_stats;
    stats->tx_count++;
}

void CAN_Diag_UpdateError(CAN_HandleTypeDef *hcan, uint32_t error_code)
{
    if (!g_diagnostic_enable) return;

    CAN_Stats_t *stats = (hcan->Instance == CAN1) ? &g_can1_stats : &g_can2_stats;
    stats->error_count++;
    stats->last_error_code = error_code;

    if (error_code & HAL_CAN_ERROR_BOF)
    {
        stats->bus_off_count++;
    }
}

/* ==================== 电磁阀统计更新 ==================== */
void Solenoid_Diag_Update(uint8_t channel, uint8_t cmd)
{
    if (!g_diagnostic_enable || channel == 0 || channel > 3) return;

    Solenoid_Stats_t *sol = &g_solenoid_stats[channel - 1];

    if (cmd != sol->last_cmd)
    {
        sol->switch_count++;
        sol->last_switch_time = g_system_tick;
    }
    sol->state = cmd & 0x0F;
    sol->last_cmd = cmd;
}

/* ==================== 定时更新(每1ms调用) ==================== */
void CAN_Diag_TickUpdate(void)
{
    g_system_tick++;

    if (!g_diagnostic_enable) return;

    // 检测电机超时
    for (uint8_t i = 0; i < 8; i++)
    {
        if (g_motor_stats[i].online)
        {
            if ((g_system_tick - g_motor_stats[i].last_rx_time) > 100)  // 100ms超时
            {
                g_motor_stats[i].timeout_count++;
                g_motor_stats[i].online = 0;
            }
        }
    }
}

/* ==================== 初始化 ==================== */
void CAN_Diag_Init(void)
{
    memset(&g_can1_stats, 0, sizeof(CAN_Stats_t));
    memset(&g_can2_stats, 0, sizeof(CAN_Stats_t));
    memset(g_motor_stats, 0, sizeof(g_motor_stats));
    memset(g_solenoid_stats, 0, sizeof(g_solenoid_stats));
    g_diagnostic_enable = 1;
    g_system_tick = 0;
}

/* ==================== 重置统计 ==================== */
void CAN_Diag_ResetStats(void)
{
    CAN_Diag_Init();
}

/* ==================== 打印诊断报告 ==================== */
void CAN_Diag_PrintReport(void)
{
    printf("\n========== CAN 通信诊断报告 ==========\n");
    printf("系统运行时间: %lu ms\n\n", g_system_tick);

    // CAN1 统计
    printf("--- CAN1 统计 ---\n");
    printf("  接收帧数: %lu\n", g_can1_stats.rx_count);
    printf("  发送帧数: %lu\n", g_can1_stats.tx_count);
    printf("  错误次数: %lu\n", g_can1_stats.error_count);
    printf("  总线关闭: %lu 次\n", g_can1_stats.bus_off_count);
    printf("  最后错误码: 0x%08lX\n", g_can1_stats.last_error_code);
    if (g_can1_stats.rx_count > 0)
    {
        printf("  最后接收: %lu ms 前\n", g_system_tick - g_can1_stats.last_rx_time);
    }

    // CAN2 统计
    printf("\n--- CAN2 统计 (电机总线) ---\n");
    printf("  接收帧数: %lu\n", g_can2_stats.rx_count);
    printf("  发送帧数: %lu\n", g_can2_stats.tx_count);
    printf("  错误次数: %lu\n", g_can2_stats.error_count);
    printf("  总线关闭: %lu 次\n", g_can2_stats.bus_off_count);
    printf("  最后错误码: 0x%08lX\n", g_can2_stats.last_error_code);

    // 电机状态
    printf("\n--- 电机状态 ---\n");
    printf("ID | 状态 | 接收帧数 | 超时次数 | 跳变次数 | 编码器 | 电流\n");
    printf("---|------|----------|----------|----------|--------|------\n");
    for (uint8_t i = 0; i < USE_DJNUM && i < 8; i++)
    {
        Motor_Stats_t *m = &g_motor_stats[i];
        printf("%2d | %s | %8lu | %8lu | %8lu | %6d | %5d\n",
               i + 1,
               m->online ? "在线" : "离线",
               m->rx_count,
               m->timeout_count,
               m->pulse_jump_count,
               m->last_pulse,
               m->last_current);
    }

    // 电磁阀状态
    printf("\n--- 电磁阀状态 ---\n");
    printf("通道 | 状态 | 切换次数 | 最后切换时间\n");
    printf("-----|------|----------|-------------\n");
    for (uint8_t i = 0; i < 3; i++)
    {
        Solenoid_Stats_t *s = &g_solenoid_stats[i];
        printf("  %d  | 0x%02X | %8lu | %lu ms\n",
               i + 1,
               s->state,
               s->switch_count,
               s->last_switch_time);
    }

    printf("\n========== 诊断建议 ==========\n");

    // CAN总线健康检查
    if (g_can2_stats.error_count > 10)
    {
        printf("⚠️  CAN2错误率高 (%lu次)，检查:\n", g_can2_stats.error_count);
        printf("   - CAN终端电阻(120Ω)\n");
        printf("   - 线缆屏蔽和接地\n");
        printf("   - 波特率配置(当前1Mbps)\n");
    }

    if (g_can2_stats.bus_off_count > 0)
    {
        printf("❌ CAN2总线关闭过 %lu 次！严重故障，立即检查硬件\n", g_can2_stats.bus_off_count);
    }

    // 电机通信检查
    uint8_t motor_offline = 0;
    for (uint8_t i = 0; i < USE_DJNUM && i < 8; i++)
    {
        if (g_motor_stats[i].rx_count == 0)
        {
            motor_offline++;
        }
        else if (g_motor_stats[i].pulse_jump_count > 5)
        {
            printf("⚠️  电机ID%d 编码器跳变异常(%lu次)，检查:\n",
                   i + 1, g_motor_stats[i].pulse_jump_count);
            printf("   - 电机连接线缆\n");
            printf("   - 电磁干扰\n");
        }
        if (g_motor_stats[i].timeout_count > 10)
        {
            printf("⚠️  电机ID%d 频繁超时(%lu次)\n", i + 1, g_motor_stats[i].timeout_count);
        }
    }

    if (motor_offline == USE_DJNUM)
    {
        printf("❌ 所有电机离线！检查:\n");
        printf("   - CAN2物理连接\n");
        printf("   - 电机电源供电\n");
        printf("   - 过滤器配置\n");
    }
    else if (motor_offline > 0)
    {
        printf("⚠️  %d个电机离线\n", motor_offline);
    }

    // 电磁阀检查
    uint8_t solenoid_inactive = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (g_solenoid_stats[i].switch_count == 0)
        {
            solenoid_inactive++;
        }
    }
    if (solenoid_inactive == 3)
    {
        printf("ℹ️  所有电磁阀未激活(正常若未使用)\n");
    }

    printf("\n=======================================\n\n");
}

/* ==================== 实时监控模式 ==================== */
void CAN_Diag_LiveMonitor(void)
{
    static uint32_t last_print = 0;

    if (g_system_tick - last_print >= 1000)  // 每秒更新
    {
        last_print = g_system_tick;

        printf("\033[2J\033[H");  // 清屏并回到首行
        printf("========== CAN 实时监控 ==========\n");
        printf("时间: %lu s | CAN1_RX: %lu | CAN2_RX: %lu | CAN2_ERR: %lu\n\n",
               g_system_tick / 1000,
               g_can1_stats.rx_count,
               g_can2_stats.rx_count,
               g_can2_stats.error_count);

        printf("电机状态:\n");
        for (uint8_t i = 0; i < USE_DJNUM && i < 8; i++)
        {
            Motor_Stats_t *m = &g_motor_stats[i];
            printf("M%d:%s ", i + 1, m->online ? "✓" : "✗");
            if ((i + 1) % 4 == 0) printf("\n");
        }
        printf("\n");

        printf("按任意键退出监控模式...\n");
    }
}

/* ==================== 启用/禁用诊断 ==================== */
void CAN_Diag_Enable(uint8_t enable)
{
    g_diagnostic_enable = enable;
}

uint8_t CAN_Diag_IsEnabled(void)
{
    return g_diagnostic_enable;
}
