#ifndef UART_IRQHandler_H
#define UART_IRQHandler_H
#include "usart.h"
#include "main.h"
extern volatile uint16_t Beep_Trigger;

void UART_Start_Receive();
#endif/*UART_IRQHandler_H*/