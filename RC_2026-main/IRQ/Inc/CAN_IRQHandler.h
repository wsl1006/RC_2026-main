#ifndef CAN_IRQHANDLER_H
#define CAN_IRQHANDLER_H
#include"can.h"
#include"main.h"
extern volatile uint8_t g_beep_new;   /* 1 = 来了新的蜂鸣器命令 */
extern volatile uint8_t g_beep_count; /* 要响几声 */

extern volatile uint8_t g_led_new;    /* 1 = 来了新的流水灯命令 */
extern volatile uint8_t g_led_cmd;    /* 0 关灯，1 开灯 */
#endif /*CAN_IRQHANDLER_H*/