/*
 * global_c.c
 *
 *  Created on: Jan 14, 2023
 *      Author: johnt
 */
#include <Global.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim6;

// Max 65,535us
void usDelay(const uint16_t delay){
  if (delay == 1) {
    volatile uint32_t x = 0;
    (void)x;
  }
  else {
    __HAL_TIM_DISABLE(&htim6);
    volatile uint16_t ccr1 = htim6.Instance->CCR1; // Read to clear CC1IF flag in SR
    (void)ccr1;
    htim6.Instance->CNT = 0;
    htim6.Instance->ARR = delay;
    htim6.Instance->CCR1 = 0b0000000000001001; // Stops counting when reaches ARR value, starts timer
    while (0 == (htim6.Instance->SR & 1));
    __HAL_TIM_DISABLE(&htim6);
  }
  uint32_t end = delay; // APB2 clock
  if (end > 0xFFFF) {
    end = 0xFFFF;
  }
}
