#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "task.h"

#include "main.h"

#include "Clock.h"
#include "Global.h"
#include "DataSample.h"

#ifdef DEBUG
// In PollingPrintf.c
extern "C" void TransmitData(uint32_t timeout);
extern "C" uint16_t DebugPrintfHighwaterMark();
constexpr uint32_t usb_buffer_length{2048};
uint32_t UserTxBufferFSLen = 0;
extern uint8_t* UserTxBufferFS;
extern "C" uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

static constexpr uint16_t transmit_timeout{1000};
static constexpr uint16_t buffer_task_length { 8192 };
static uint8_t buffer[buffer_task_length];
static TaskIndex task_index { TaskIndex::DEFAULT_APP };

extern osMessageQueueId_t DefaultTaskQueueHandle;

extern "C"
void TransmitBuffer(uint8_t* buffer, uint16_t length, uint32_t timeout);

extern "C" {
void StartDefaultTask(){
  Clock& testClock = Clock::GetInstance();
  testClock.SetTime(1577865600);

  static char taskListBuf[500];
  static char taskStatsBuf[500];
  const uint16_t taskDelay = 100; // milliseconds
  const uint16_t STATS_DELAY = 10000 / taskDelay;
  uint16_t count = 0;
  osStatus_t status;
  uint8_t priority;
  TaskMessage queue_item;

  while (true) {
    status = osMessageQueueGet(DefaultTaskQueueHandle, &queue_item.data, &priority, taskDelay);
    if (status != osOK && status != osErrorTimeout) {
      PRINTLN("DefaultTask acquire msg queue not ok Task");
      continue;
    }

    // Timeout is expected
    // Flush any buffered log data.
    TransmitData(transmit_timeout);
    ++count;

    // Check if it's time to print stats and to send out stats buffers
    if (count == STATS_DELAY) {
      count = 0;
      PRINTLN("\nDebug printf high water mark: %u", DebugPrintfHighwaterMark());

      PRINTLN("Kernel Ticks: %lu", osKernelGetTickCount());
      uint64_t cpuTime = GetTimeInNanoSeconds();
      uint64_t seconds = cpuTime / 1000000000;
      uint64_t nano_seconds = cpuTime % 1000000000;
      PRINTLN("CPU Run Time in Seconds: %llu.%llu", seconds, nano_seconds);

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

      TransmitData(transmit_timeout);

      if (task_index == TaskIndex::DEFAULT_APP) {
        task_index = TaskIndex::DATA_COLLECTOR;
        SendStatsDataCollector(buffer_task_length, buffer);
      }
    }

    // Process any incoming messages
    if (status == osOK){
      switch (queue_item.msg.id) {
      case static_cast<uint8_t>(TaskMsg::STATS_BUFFER):
        UserTxBufferFSLen = queue_item.msg.length > usb_buffer_length ? usb_buffer_length : queue_item.msg.length;
        memcpy(UserTxBufferFS, (uint8_t*)queue_item.msg.buffer, UserTxBufferFSLen);
        CDC_Transmit_FS(UserTxBufferFS, UserTxBufferFSLen);
        TransmitBuffer((uint8_t*)queue_item.msg.buffer, queue_item.msg.length, transmit_timeout);
        switch (task_index) {
        case TaskIndex::DATA_COLLECTOR:
          task_index = TaskIndex::DATA_WRITER;
          SendStatsDataWriter(buffer_task_length, buffer);
          break;
        case TaskIndex::DATA_WRITER:
          task_index = TaskIndex::RADIO;
          SendStatsRadioComms(buffer_task_length, buffer);
          break;
        case TaskIndex::RADIO:
          task_index = TaskIndex::ROCKET_APP;
          SendStatsRocketApp(buffer_task_length, buffer);
          break;
        case TaskIndex::ROCKET_APP:
          task_index = TaskIndex::DEFAULT_APP;
          break;
        default:
          PRINTLN("Error task_index = %u", uint32_t(task_index));
          break;
        }
        break;
      default:
        PRINTLN("Error default task: default case msg.id = %u", uint32_t(queue_item.msg.id));
        break;
      }
    }
  }
}

#else
void RunDefaultTask()
{
	while (1)
	{
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    osDelay(1000);
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    osDelay(1000);
	}
}
#endif

} /* End extern C" */
