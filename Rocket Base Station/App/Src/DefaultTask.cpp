
#include <Global.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include "task.h"
#include "Clock.h"
#include "Messages.h"

extern "C" {
void TransmitData(uint32_t timeout);
uint16_t DebugHighwaterMark();

void RunDefaultTask(){
  static char taskListBuf[500];
  static char taskStatsBuf[500];
  Clock& testClock = Clock::GetInstance();
  testClock.SetTime(1577865600);

  // Let system know that this task is ready to for low power sleep
  const uint16_t taskDelay = 100; // milliseconds
  const uint16_t STATS_DELAY = 10000 / taskDelay;
  const uint16_t blinkDelay = 5;
  uint16_t count = 0;

  while (true) {
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    osDelay(taskDelay);
    TransmitData(1000);
    ++count;
    if (count % blinkDelay) {
      HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }

    if (count == STATS_DELAY) {
      count = 0;
      PRINTLN("\nDebug printf high water mark: %u", DebugHighwaterMark());
      PRINTLN("Kernel Ticks: %lu", osKernelGetTickCount());
      vTaskList(taskListBuf);
      PRINTLN("Name          State  Priority   Stack   Num");
      PRINTLN("*******************************************");
      PRINTLN(taskListBuf);

      vTaskGetRunTimeStats(taskStatsBuf);
      PRINTLN("Name             Abs Time       %% Time");
      PRINTLN("*******************************************");
      PRINTLN(taskStatsBuf);

      time_t epochTime = testClock.GetCurrentTimeEpoch();
      tm testTime = testClock.GetTime(epochTime);

      PRINTLN("Clock Time - %d/%d/%d %02d:%02d:%02d UTC (epochTime: %ld)", testTime.tm_mon + 1, testTime.tm_mday, testTime.tm_year + 1900,
              testTime.tm_hour, testTime.tm_min, testTime.tm_sec, (uint32_t )epochTime);
    }
  }
}

}


