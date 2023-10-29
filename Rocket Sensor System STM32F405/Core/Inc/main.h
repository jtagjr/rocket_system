/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define RADIO_RESET_Pin GPIO_PIN_2
#define RADIO_RESET_GPIO_Port GPIOE
#define SPI2_CS_RADIO_Pin GPIO_PIN_3
#define SPI2_CS_RADIO_GPIO_Port GPIOE
#define RADIO_INT2_Pin GPIO_PIN_6
#define RADIO_INT2_GPIO_Port GPIOE
#define RADIO_INT1_Pin GPIO_PIN_13
#define RADIO_INT1_GPIO_Port GPIOC
#define RADIO_INT1_EXTI_IRQn EXTI15_10_IRQn
#define I2C2_SDA_GPS_Pin GPIO_PIN_0
#define I2C2_SDA_GPS_GPIO_Port GPIOF
#define I2C2_SCL_GPS_Pin GPIO_PIN_1
#define I2C2_SCL_GPS_GPIO_Port GPIOF
#define TEST_PIN0_Pin GPIO_PIN_6
#define TEST_PIN0_GPIO_Port GPIOF
#define TEST_PIN1_Pin GPIO_PIN_7
#define TEST_PIN1_GPIO_Port GPIOF
#define TEST_PIN2_Pin GPIO_PIN_8
#define TEST_PIN2_GPIO_Port GPIOF
#define TEST_PIN3_Pin GPIO_PIN_9
#define TEST_PIN3_GPIO_Port GPIOF
#define ACCEL1_CS_Pin GPIO_PIN_4
#define ACCEL1_CS_GPIO_Port GPIOA
#define ACCEL1_INT1_Pin GPIO_PIN_4
#define ACCEL1_INT1_GPIO_Port GPIOC
#define ACCEL1_INT1_EXTI_IRQn EXTI4_IRQn
#define ACCEL2_CS_Pin GPIO_PIN_5
#define ACCEL2_CS_GPIO_Port GPIOC
#define ACCEL2_INT1_Pin GPIO_PIN_0
#define ACCEL2_INT1_GPIO_Port GPIOB
#define ACCEL2_INT1_EXTI_IRQn EXTI0_IRQn
#define BAROMETER_INT_Pin GPIO_PIN_2
#define BAROMETER_INT_GPIO_Port GPIOB
#define BAROMETER_INT_EXTI_IRQn EXTI2_IRQn
#define ACCEL1_INT2_Pin GPIO_PIN_14
#define ACCEL1_INT2_GPIO_Port GPIOF
#define ACCEL1_INT2_EXTI_IRQn EXTI15_10_IRQn
#define BAROMETER_CS_Pin GPIO_PIN_1
#define BAROMETER_CS_GPIO_Port GPIOG
#define ACCEL2_INT2_INPUT_Pin GPIO_PIN_12
#define ACCEL2_INT2_INPUT_GPIO_Port GPIOE
#define DIP_SW_Pin GPIO_PIN_12
#define DIP_SW_GPIO_Port GPIOB
#define LED1_MCU_Pin GPIO_PIN_13
#define LED1_MCU_GPIO_Port GPIOB
#define UART3_TX_TO_GPS_Pin GPIO_PIN_8
#define UART3_TX_TO_GPS_GPIO_Port GPIOD
#define UART3_RX_FROM_GPS_Pin GPIO_PIN_9
#define UART3_RX_FROM_GPS_GPIO_Port GPIOD
#define LED2_MCU_Pin GPIO_PIN_10
#define LED2_MCU_GPIO_Port GPIOD
#define GPS_INT_Pin GPIO_PIN_12
#define GPS_INT_GPIO_Port GPIOD
#define GPS_INT_EXTI_IRQn EXTI15_10_IRQn
#define GPS_RESET_Pin GPIO_PIN_14
#define GPS_RESET_GPIO_Port GPIOD
#define GPS_PPS_INT_Pin GPIO_PIN_2
#define GPS_PPS_INT_GPIO_Port GPIOG
#define UART2_TX_DEBUG_TEXT_Pin GPIO_PIN_6
#define UART2_TX_DEBUG_TEXT_GPIO_Port GPIOC
#define DEPLOY_SWITCH_2_Pin GPIO_PIN_9
#define DEPLOY_SWITCH_2_GPIO_Port GPIOA
#define DEPLOY_SWITCH_1_Pin GPIO_PIN_10
#define DEPLOY_SWITCH_1_GPIO_Port GPIOA
#define SDIO_CLK_Pin GPIO_PIN_12
#define SDIO_CLK_GPIO_Port GPIOC
#define UART1_TEXT_DEBUG_Pin GPIO_PIN_6
#define UART1_TEXT_DEBUG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
