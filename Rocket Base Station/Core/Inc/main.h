/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void StartSampleTimer();
void StopSampleTimer();
uint32_t SampleTick();

void StartPrintStatsTimer();
void StopPrintStatsTimer();
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_1
#define LED_BLUE_GPIO_Port GPIOC
#define USER_PIN_3_Pin GPIO_PIN_7
#define USER_PIN_3_GPIO_Port GPIOA
#define SPI2_CS_RADIO_Pin GPIO_PIN_4
#define SPI2_CS_RADIO_GPIO_Port GPIOC
#define RADIO_PACKET_RECEIVED_INTERRUPT_Pin GPIO_PIN_5
#define RADIO_PACKET_RECEIVED_INTERRUPT_GPIO_Port GPIOC
#define RADIO_PACKET_RECEIVED_INTERRUPT_EXTI_IRQn EXTI9_5_IRQn
#define SENSOR_SPI2_SCK_Pin GPIO_PIN_13
#define SENSOR_SPI2_SCK_GPIO_Port GPIOB
#define SENSOR_SPI2_MISO_Pin GPIO_PIN_14
#define SENSOR_SPI2_MISO_GPIO_Port GPIOB
#define SENSOR_SPI2_MOSI_Pin GPIO_PIN_15
#define SENSOR_SPI2_MOSI_GPIO_Port GPIOB
#define RADIO_RESET_Pin GPIO_PIN_7
#define RADIO_RESET_GPIO_Port GPIOC
#define USER_PIN_2_Pin GPIO_PIN_9
#define USER_PIN_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
uint16_t sample_timer_ticks();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
