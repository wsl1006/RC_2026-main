#ifndef BEEPTASK_H
#define BEEPTASK_H

#include "main.h"

/* 主循环调用：蜂鸣器状态机 */
void BeepTask(void);

/* 这些变量由 BeepTask.c 定义，TIM 中断里需要使用，所以在这里 extern 出去 */
extern uint16_t beep_count_on;
extern uint16_t beep_count_off;
extern volatile uint8_t bp_done;
extern volatile uint8_t bp_total;

#endif /* BEEPTASK_H */
