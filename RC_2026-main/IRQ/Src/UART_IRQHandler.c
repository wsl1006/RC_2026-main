#include "UART_IRQHandler.h"
uint8_t rx_buffer[5] = {0};
uint8_t tx_buffer[5] = {0};
volatile uint16_t Beep_Trigger;
void UART_Start_Receive()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, 5);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart->RxEventType == HAL_UART_RXEVENT_HT)
        return;                       // 半传输，不是完整命令，忽略
    if (huart->Instance == USART1)
    {
        
        if (rx_buffer[0] == 0xFF)
        {
            for (uint8_t i = 1; i < Size; i++)  //这里不要读固定位，不然残留也一并读了，响很多次
            {
                if (rx_buffer[i] == 1)
                    Beep_Trigger++;
            }
            // HAL_UART_Transmit(&huart1,tx_buffer,5,1);
        }
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, 5);
    }
}
