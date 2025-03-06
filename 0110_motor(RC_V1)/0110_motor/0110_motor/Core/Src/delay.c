/*
 * dealayus.c
 *
 *  Created on: Jan 8, 2025
 *      Author: user
 */


#include "delay.h"



void delay_us(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim11);  // 또는 다른 적절한 타이머
    while ((__HAL_TIM_GET_COUNTER(&htim11) - start) < us);
}
