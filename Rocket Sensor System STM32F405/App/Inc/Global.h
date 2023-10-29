#ifndef __GLOBAL_C_H__
#define __GLOBAL_C_H__

#include <stdarg.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "main.h"
#include <atomic.h>

extern volatile uint32_t timer13Overflow; // Increments every 65,535us
extern TIM_HandleTypeDef htim13;
enum {
  LogicAnalyzerTimedValue = 66351705
};

#ifdef 	DEBUG
#ifdef __cplusplus
extern "C" {
#endif
  void thread_safe_printf(const char* format, ...);
  void thread_safe_printf_newline(const char* format, ...);
  void initialize_debug_printf();

  // max 4,681us
  void usDelay(uint16_t delay_time);

  __attribute__((always_inline)) static inline uint64_t GetTimeInNanoSeconds(){
    ATOMIC_ENTER_CRITICAL();
    uint64_t overflowMicroseconds = timer13Overflow;
    uint16_t currentCnt = htim13.Instance->CNT;
    ATOMIC_EXIT_CRITICAL();
    overflowMicroseconds *= LogicAnalyzerTimedValue;
    overflowMicroseconds += currentCnt;
    return overflowMicroseconds;
  }

  __attribute__((always_inline)) static inline uint8_t InReceiveMode(){
    return GPIO_PIN_SET == HAL_GPIO_ReadPin(DIP_SW_GPIO_Port, DIP_SW_Pin);
  }

  // LED 1 maps to LED 7 on board silkscreen
  __attribute__((always_inline)) static inline
  void OnLedOne(){
    HAL_GPIO_WritePin(LED1_MCU_GPIO_Port, LED1_MCU_Pin, GPIO_PIN_SET);
  }

  __attribute__((always_inline)) static inline
  void OffLedOne(){
    HAL_GPIO_WritePin(LED1_MCU_GPIO_Port, LED1_MCU_Pin, GPIO_PIN_RESET);
  }

  __attribute__((always_inline)) static inline
  void ToggleLedOne(){
    HAL_GPIO_TogglePin(LED1_MCU_GPIO_Port, LED1_MCU_Pin);
  }

  // LED 2 maps to LED 8 on board silkscreen
  __attribute__((always_inline)) static inline
  void OnLedTwo(){
    HAL_GPIO_WritePin(LED2_MCU_GPIO_Port, LED2_MCU_Pin, GPIO_PIN_SET);
  }

  __attribute__((always_inline)) static inline
  void OffLedTwo(){
    HAL_GPIO_WritePin(LED2_MCU_GPIO_Port, LED2_MCU_Pin, GPIO_PIN_RESET);
  }

  __attribute__((always_inline)) static inline
  void ToggleLedTwo(){
    HAL_GPIO_TogglePin(LED2_MCU_GPIO_Port, LED2_MCU_Pin);
  }

  __attribute__((always_inline)) static inline
  void TP0_On(){
    HAL_GPIO_WritePin(TEST_PIN0_GPIO_Port, TEST_PIN0_Pin, GPIO_PIN_RESET);
  }

  __attribute__((always_inline)) static inline
  void TP0_Off(){
    HAL_GPIO_WritePin(TEST_PIN0_GPIO_Port, TEST_PIN0_Pin, GPIO_PIN_SET);
  }

  __attribute__((always_inline)) static inline
  void TP1_On(){
    HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port, TEST_PIN1_Pin, GPIO_PIN_RESET);
  }

  __attribute__((always_inline)) static inline
  void TP1_Off(){
    HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port, TEST_PIN1_Pin, GPIO_PIN_SET);
  }

  __attribute__((always_inline)) static inline
  void TP2_On(){
    HAL_GPIO_WritePin(TEST_PIN2_GPIO_Port, TEST_PIN2_Pin, GPIO_PIN_RESET);
  }

  __attribute__((always_inline)) static inline
  void TP2_Off(){
    HAL_GPIO_WritePin(TEST_PIN2_GPIO_Port, TEST_PIN2_Pin, GPIO_PIN_SET);
  }

  __attribute__((always_inline)) static inline
  void TP3_On(){
    HAL_GPIO_WritePin(TEST_PIN3_GPIO_Port, TEST_PIN3_Pin, GPIO_PIN_RESET);
  }

  __attribute__((always_inline)) static inline
  void TP3_Off(){
    HAL_GPIO_WritePin(TEST_PIN3_GPIO_Port, TEST_PIN3_Pin, GPIO_PIN_SET);
  }

#ifdef __cplusplus
}
#endif

#define PRINT(...) thread_safe_printf(__VA_ARGS__)
#define PRINTLN(...) thread_safe_printf_newline(__VA_ARGS__)
#else
#define PRINT(...)
#define PRINTLN(...)
#endif
#endif //__GLOBAL_C_H__
