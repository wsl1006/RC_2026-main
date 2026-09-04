#ifndef TIM_IRQHandler_H
#define TIM_IRQHandler_H
/*-----------------------Include---------------------------*/
#include "main.h"
#include "tim.h"


/*-----------------------Variable---------------------------*/

/*-----------------------Function--------------------------*/
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif /*TIM_IRQHandler_H*/
