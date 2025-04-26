#include "delay.h"
#include "stm32f1xx_hal.h"

void delay_us (TIM_HandleTypeDef htim, uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim,0);
    while ((__HAL_TIM_GET_COUNTER(&htim)) < us);
}
