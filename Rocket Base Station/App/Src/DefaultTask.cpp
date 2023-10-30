
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

void ProcessUsbRxData();
void TransmitData(uint32_t timeout);
void HandleUsbMessage(uint8_t id);

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
    ProcessUsbRxData();

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

constexpr uint32_t usb_buffer_length { 64 };
extern uint8_t* UserRxBufferFS; // Data received during ISR is stored here during the callback function CDC_Receive_FS
extern uint8_t* UserTxBufferFS; // Data to send is store here and retrieved in the callback function CDC_Transmit_FS

volatile bool rx_packet_length = false;
uint16_t rx_write_index{0};
uint16_t rx_read_index{0};
uint8_t rx_next_length{0};
uint8_t rx_packet[usb_buffer_length];
uint8_t tx_packet[usb_buffer_length];
bool msg_received{false};

extern void CDC_Transmit_FS(uint8_t*, uint16_t);

using ParseFunc = void (*)(const uint8_t);

enum class ParseState : uint8_t {
  Start,
  Length,
  Id,
  Payload
};

void UsbParseLength(const uint8_t byte);

ParseState parse_state{ParseState::Start};
ParseFunc parser_function = UsbParseLength;


void UsbParseData(const uint8_t byte) {
  rx_packet[rx_write_index++] = byte;
  --rx_next_length;
  if (rx_next_length == 0) {
    parser_function = UsbParseLength;
    msg_received = true;
  }
}

void UsbParseLength(const uint8_t byte) {
  if (byte > 0) {
    rx_next_length = rx_packet[rx_write_index++] = byte;
    parser_function = UsbParseData;
  }
}

void UsbParsePacket(const uint8_t byte) {
  (*parser_function)(byte);
  if (rx_write_index >= usb_buffer_length) {
    rx_write_index = 0;
  }
}

// Callback from USB library during ISR
void UpperLayerRxPacket(uint8_t* packet) {
  for(uint16_t i=0; i<usb_buffer_length; ++i) {
    UsbParsePacket(packet[i]);
  }
}

void UpperLayerTxComplete(uint8_t* packet, uint16_t len) {

}

void ProcessPacket() {
  uint8_t length = rx_packet[rx_read_index++];
  if (rx_read_index >= usb_buffer_length) {
    rx_read_index = 0;
  }

  auto id = rx_packet[rx_read_index++];
  if (rx_read_index >= usb_buffer_length) {
    rx_read_index = 0;
  }

  HandleUsbMessage(id);
}

void ProcessUsbRxData() {
  if (msg_received) {
    msg_received = false;
    ProcessPacket();
  }
}

}


