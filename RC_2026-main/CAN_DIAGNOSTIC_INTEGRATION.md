# CAN通信链路检测和电磁阀诊断工具 - 集成指南

## 概述

本工具用于检测CAN通信质量、电机反馈状态和电磁阀工作情况，帮助快速定位硬件和通信问题。

---

## 功能特性

✅ **CAN总线监控**
- 实时统计接收/发送帧数
- 错误计数和总线关闭检测
- 通信质量评估

✅ **电机状态诊断**
- 在线/离线检测
- 编码器跳变异常监测
- 通信超时统计
- 反馈数据实时跟踪

✅ **电磁阀状态监控**
- 切换次数统计
- 状态变化跟踪
- GPIO输出验证

---

## 集成步骤

### 1. 添加文件到工程

已创建的文件：
- `Core/Inc/CAN_Diagnostic.h`
- `Core/Src/CAN_Diagnostic.c`

在Keil/IAR工程中添加这两个文件。

---

### 2. 修改 CAN_IRQHandler.c

在 `IRQ/Src/CAN_IRQHandler.c` 中添加诊断钩子：

```c
#include "CAN_IRQHandler.h"
#include "CAN_Diagnostic.h"  // 添加这行
#include "motor.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 添加诊断统计
            CAN_Diag_UpdateRx(hcan, &RxHeader, RxData);
            
            // 原有业务逻辑...
            if (RxHeader.IDE == CAN_ID_EXT)
            {
                // ...
            }
        }
    }
    else if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 添加诊断统计
            CAN_Diag_UpdateRx(hcan, &RxHeader, RxData);
            
            if (RxHeader.IDE == CAN_ID_STD && (RxHeader.StdId >= 0x201U && RxHeader.StdId <= 0x208U))
            {
                DJmotor_Receive(RxHeader, RxData);
            }
        }
    }
}
```

---

### 3. 修改 motor.c

在 `DJmotor/Src/motor.c` 的发送函数中添加统计：

```c
#include "motor.h"
#include "CAN_Diagnostic.h"  // 添加这行

void DJmotor_CurrentTransmit(DJMotorPointer motor)
{
    static uint8_t tx_data[8] = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;
    // ... 原有代码 ...

    if (motor->ID == 4U || motor->ID == 8U)
    {
        CAN_HandleTypeDef *hcan = DJmotor_GetCanHandle();
        if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox) == HAL_OK)
        {
            // 添加发送统计
            CAN_Diag_UpdateTx(hcan);
        }
    }
}
```

---

### 4. 修改 Solenoid.c

在 `HDL/Src/Solenoid.c` 的控制函数中添加统计：

```c
#include "Solenoid.h"
#include "CAN_Diagnostic.h"  // 添加这行

void solenoid_on(uint8_t usart_channel, uint8_t cmd)
{
    uint8_t data = cmd & 0x0f;
    
    // 添加诊断统计
    Solenoid_Diag_Update(usart_channel, cmd);
    
    switch (usart_channel)
    {
    case 1:
        register_updata(&solenoid_Channel1, &data);
        break;
    // ... 其余代码不变
    }
}
```

---

### 5. 修改定时器中断

在 `IRQ/Src/TIM_IRQHandler.c` 或 `stm32f4xx_it.c` 的1ms定时器中断中添加：

```c
#include "CAN_Diagnostic.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)  // 假设TIM2是1ms定时器
    {
        // 添加诊断时钟更新
        CAN_Diag_TickUpdate();
        
        // 原有业务逻辑...
    }
}
```

---

### 6. 修改 main.c

在 `Core/Src/main.c` 中初始化和使用诊断功能：

```c
#include "main.h"
#include "CAN_Diagnostic.h"  // 添加这行

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    
    DJmotor_Init();
    
    // 初始化诊断模块
    CAN_Diag_Init();
    
    HAL_TIM_Base_Start_IT(&htim2);  // 启动1ms定时器
    
    uint32_t last_report = 0;
    
    while (1)
    {
        // 每5秒打印一次诊断报告
        if (HAL_GetTick() - last_report >= 5000)
        {
            last_report = HAL_GetTick();
            CAN_Diag_PrintReport();
        }
        
        // 或使用实时监控模式（注释掉上面的，启用这个）
        // CAN_Diag_LiveMonitor();
        
        // 原有业务逻辑...
    }
}
```

---

## 使用方法

### 方法1：定期报告模式

主循环每5秒打印一次完整报告：

```c
while (1)
{
    if (HAL_GetTick() - last_report >= 5000)
    {
        last_report = HAL_GetTick();
        CAN_Diag_PrintReport();
    }
}
```

**输出示例：**
```
========== CAN 通信诊断报告 ==========
系统运行时间: 15342 ms

--- CAN1 统计 ---
  接收帧数: 25
  发送帧数: 0
  错误次数: 0
  总线关闭: 0 次
  最后错误码: 0x00000000
  最后接收: 125 ms 前

--- CAN2 统计 (电机总线) ---
  接收帧数: 6136
  发送帧数: 1534
  错误次数: 0

--- 电机状态 ---
ID | 状态 | 接收帧数 | 超时次数 | 跳变次数 | 编码器 | 电流
---|------|----------|----------|----------|--------|------
 1 | 在线 |     1534 |        0 |        0 |   2156 |   324
 2 | 在线 |     1534 |        0 |        0 |  -1024 |  -156
 3 | 在线 |     1534 |        0 |        2 |   8156 |   892
 4 | 在线 |     1534 |        0 |        0 |    512 |   125

--- 电磁阀状态 ---
通道 | 状态 | 切换次数 | 最后切换时间
-----|------|----------|-------------
  1  | 0x03 |       12 | 8234 ms
  2  | 0x00 |        0 | 0 ms
  3  | 0x00 |        0 | 0 ms

========== 诊断建议 ==========
⚠️  电机ID3 编码器跳变异常(2次)，检查:
   - 电机连接线缆
   - 电磁干扰
```

---

### 方法2：实时监控模式

实时刷新显示（需要串口终端支持ANSI转义）：

```c
while (1)
{
    CAN_Diag_LiveMonitor();
    HAL_Delay(10);
}
```

---

### 方法3：手动触发报告

通过串口命令或按键触发：

```c
// 在串口接收回调中
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (rx_buffer[0] == 'd')  // 按'd'键打印诊断
    {
        CAN_Diag_PrintReport();
    }
    else if (rx_buffer[0] == 'r')  // 按'r'键重置统计
    {
        CAN_Diag_ResetStats();
        printf("诊断统计已重置\n");
    }
}
```

---

## 常见问题诊断

### 问题1：所有电机离线

**可能原因：**
1. CAN2物理连接断开
2. 终端电阻未接或阻值错误（应为120Ω）
3. 电机电源未供电
4. 过滤器配置错误

**检查步骤：**
```c
// 1. 检查CAN2状态寄存器
uint32_t esr = hcan2.Instance->ESR;
printf("CAN2 ESR: 0x%08lX\n", esr);

// 2. 手动发送测试帧
CAN_TxHeaderTypeDef tx;
uint8_t data[8] = {0};
tx.StdId = 0x200;
tx.IDE = CAN_ID_STD;
tx.RTR = CAN_RTR_DATA;
tx.DLC = 8;
uint32_t mailbox;
HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan2, &tx, data, &mailbox);
printf("发送结果: %d\n", ret);
```

---

### 问题2：编码器值异常跳变

**症状：** `pulse_jump_count` 持续增加

**可能原因：**
1. CAN线缆质量差或接触不良
2. 电磁干扰（靠近大功率电机驱动）
3. 地线回流路径不当

**解决方案：**
- 使用屏蔽双绞线
- 加强接地
- CAN线远离功率线

---

### 问题3：电磁阀不动作

**检查步骤：**

1. **验证GPIO输出**
```c
// 在 register_updata() 中添加调试
printf("GPIO写入: Port=%p Pin=0x%04X Level=%d\n", 
       solenoid->gpio_port, 
       solenoid->gpio_pin_sda, 
       (*data & 0x08) ? 1 : 0);
```

2. **用示波器/逻辑分析仪检测**
   - SDA引脚应输出时钟数据
   - CLK引脚应有脉冲

3. **测试GPIO强制输出**
```c
// 强制拉高测试
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
HAL_Delay(1000);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
```

---

## 性能影响

- **内存占用：** 约200字节（统计结构体）
- **CPU占用：** < 1%（仅在事件发生时统计）
- **实时性影响：** 可忽略（每次统计<10条指令）

---

## 禁用诊断

如果需要在发布版本中禁用诊断功能：

```c
// 方法1：在main.c中不调用 CAN_Diag_Init()

// 方法2：运行时禁用
CAN_Diag_Enable(0);  // 0=禁用, 1=启用
```

或者在 `CAN_Diagnostic.h` 中添加编译开关：

```c
#ifndef ENABLE_CAN_DIAGNOSTIC
#define ENABLE_CAN_DIAGNOSTIC 1  // 改为0可完全禁用
#endif
```

---

## 调试技巧

### 1. 查看CAN总线波特率

```c
// 当前配置: Prescaler=3, BS1=9TQ, BS2=4TQ, SJW=1TQ
// APB1时钟42MHz (STM32F4通常配置)
// 波特率 = 42MHz / (3 * (1 + 9 + 4)) = 1 Mbps
```

### 2. 使能CAN错误中断

在 `stm32f4xx_it.c` 中添加：

```c
void CAN2_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);
}

// 在初始化时使能
HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
```

### 3. 错误回调

```c
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);
    CAN_Diag_UpdateError(hcan, err);
    printf("CAN错误: 0x%08lX\n", err);
}
```

---

## 总结

本诊断工具帮助你：
1. ✅ 实时监控CAN通信健康状态
2. ✅ 快速定位电机离线/反馈异常
3. ✅ 验证电磁阀控制信号
4. ✅ 提供问题诊断建议

集成后，在遇到通信问题时，先运行诊断报告，根据输出的建议逐项排查。

---

**创建时间：** 2026-09-04  
**适用工程：** RC_2026 STM32F4 + DJI电机 + 电磁阀控制
