/*
 * GpioIrqHandlers.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: johnt
 */

#include <Global.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os2.h"

extern "C" {

  void HandleIrq_ACCEL1_INT1_Pin();
  void HandleIrq_ACCEL2_INT1_Pin();
  void HandleIrq_BAROMETER_INT_Pin();
  void HandleIrq_RADIO_INT1_Pin();
  void HandleIrq_RADIO_INT2_Pin();
  void HandleIrq_GPS_INT_Pin();
  void HandleIrq_ACCEL1_INT2_Pin();

  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    switch (GPIO_Pin) {
      case ACCEL1_INT1_Pin:
        HandleIrq_ACCEL1_INT1_Pin();
        break;

      case ACCEL1_INT2_Pin:
        HandleIrq_ACCEL1_INT2_Pin();
        break;

      case ACCEL2_INT1_Pin:
        HandleIrq_ACCEL2_INT1_Pin();
        break;

      case BAROMETER_INT_Pin:
        HandleIrq_BAROMETER_INT_Pin();
        break;

      case RADIO_INT1_Pin:
        HandleIrq_RADIO_INT1_Pin();
        break;

      case RADIO_INT2_Pin:
        // Currently this pin is set as input only not an interrupt
        HandleIrq_RADIO_INT2_Pin();
        break;

      case GPS_INT_Pin:
        HandleIrq_GPS_INT_Pin();
        break;

      default:
        // Log something
        break;
    }
  }
}

