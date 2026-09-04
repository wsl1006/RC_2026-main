/*
 * @Author: Frt001 2067314783@qq.com
 * @Date: 2026-08-24 16:51:06
 * @LastEditors: Frt001 2067314783@qq.com
 * @LastEditTime: 2026-08-24 16:51:57
 * @FilePath: \f4_show\FML\Src\Solenoid.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Solenoid.h"

Solenoid_t solenoid_Channel1 = {0};
Solenoid_t solenoid_Channel2 = {0};
Solenoid_t solenoid_Channel3 = {0};

void solenoid_channel_init(Solenoid_t *solenoid, GPIO_TypeDef *gpio_port, uint16_t gpio_pin_sda, uint16_t gpio_pin_clk)
{
    solenoid->gpio_port = gpio_port;      // 保存端口
    solenoid->gpio_pin_sda = gpio_pin_sda; // 保存数据引脚
    solenoid->gpio_pin_clk = gpio_pin_clk; // 保存时钟引脚
    solenoid->data_prve = 0xF0;            // 初始化为非有效值（高4位为1），保证第一次调用会发送数据
}
// usart_channel=串口号 不需要在cube中配置 直接调用即可
void solenoid_init(uint8_t usart_channel)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    switch (usart_channel)
    {
    case 1:
        // 绑定通道1结构体到 GPIOA 的 PIN9(SDA) 和 PIN10(CLK)
        solenoid_channel_init(&solenoid_Channel1, GPIOA, GPIO_PIN_9, GPIO_PIN_10);
        __HAL_RCC_GPIOA_CLK_ENABLE();          // 使能 GPIOA 时钟
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET); // 初始拉低 SDA 和 CLK
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;   // 配置这两个引脚
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // 推挽输出
        GPIO_InitStruct.Pull = GPIO_NOPULL;               // 无上下拉
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;   // 中等速度
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);           // 调用 HAL 库初始化
        break;
    case 2:
        solenoid_channel_init(&solenoid_Channel2, GPIOA, GPIO_PIN_2, GPIO_PIN_3);
        __HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
    case 3:
        solenoid_channel_init(&solenoid_Channel3, GPIOC, GPIO_PIN_10, GPIO_PIN_11);
        __HAL_RCC_GPIOC_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        break;
    default:
        break;
    }
    solenoid_on(usart_channel, 0);
}
void register_updata(Solenoid_t *solenoid, uint8_t *data)
{
    if (*data == solenoid->data_prve)
        return;
    solenoid->data_prve = *data;
    /* 关中断保护整个移位窗口(微秒级),防止 SDA/CLK 时序被高优先级中断打断
       导致电磁阀板寄存器数据错位 */
    __disable_irq();
    for (int i = 0; i < 4; i++)
    {
        if ((*data & 0x08) == 0x08)
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_RESET);
        }
        *data <<= 1;
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
        __NOP();
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
    __NOP();
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
    __enable_irq();
}
// usart_channel=串口号, cmd=命令(低4位控制) 0000 4321 （4321为对应通道）
void solenoid_on(uint8_t usart_channel, uint8_t cmd)
{
    uint8_t data = cmd & 0x0f;
    switch (usart_channel)
    {
    case 1:
        register_updata(&solenoid_Channel1, &data);
        break;
    case 2:
        register_updata(&solenoid_Channel2, &data);
        break;
    case 3:
        register_updata(&solenoid_Channel3, &data);
        break;
    default:
        break;
    }
}