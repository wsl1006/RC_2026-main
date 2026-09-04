#include "TIM_IRQHandler.h"
#include "UART_IRQHandler.h"
#include "BeepTask.h"
#include "gpio.h"
#include "ZDrive.h"
#include "motor.h"
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == TIM2) /* TIM2 = 2ms */
  {
    /* 电机控制每 2ms 一次(500Hz),满足 DJI C620 电调 ≥500Hz 控制率要求 */
    DJmotor_Func();
    if (beep_count_on > 0)
    {
      beep_count_on--;
      if (beep_count_on == 0)
      {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        beep_count_off = 50;
      }
    }
    else if (beep_count_off > 0)
    {
      beep_count_off--;
      if (beep_count_off == 0)
      {
        bp_done++;
        if (bp_done < bp_total)
        {
          beep_count_on = 50;
          HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        }
      }
    }
  }
}
