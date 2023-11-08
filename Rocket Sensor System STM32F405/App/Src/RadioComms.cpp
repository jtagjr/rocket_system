/*
 * RadioComms.cpp
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */

#include "string.h"

#include "cmsis_os2.h"

#include "main.h"
#include <Global.h>

#include "ExternalMessages.h"
#include "DataSample.h"
#include "RadioComms.h"

enum class RadioInternalCommands : uint8_t {
  NONE,
  START_RADIO,
  STOP_RADIO,
  SEND_SENSOR_DATA,
  SEND_GPS_DATA,
  RADIO_INTERRUPT_1,
  RADIO_INTERRUPT_2,
  PRINT_STATS,
};

extern osMessageQueueId_t RadioCommsQueueHandle;

static uint8_t received_packet[RadioPacketLength];
uint8_t packet_count = 1;

RadioComms radio;

void PrintStatusRegisters(RFM69& radio){
  uint8_t value = 0;
  if (radio.readRegister(1, value)) {
    PRINTLN("[0x01]:[RegOpMode]:0x%X", value);
  }
  if (radio.readRegister(2, value)) {
    PRINTLN("[0x02]:[RegDataModul]: 0x%X", value);
  }
  if (radio.readRegister(0x24, value)) {
    PRINTLN("[0x24]:[RegRssiValue]: 0x%X dB=%i", value, value / -2);
  }
  if (radio.readRegister(0x25, value)) {
    PRINTLN("[0x25]:[RegDioMapping1]: 0x%X", value);
  }
  if (radio.readRegister(0x26, value)) {
    PRINTLN("[0x26]:[RegDioMapping2]: 0x%X", value);
  }
  if (radio.readRegister(0x27, value)) {
    PRINTLN("[0x27]:[RegIrqFlags1]: 0x%X", value);
  }
  if (radio.readRegister(0x28, value)) {
    PRINTLN("[0x28]:[RegIrqFlags2]: 0x%X", value);
  }
}

RadioComms::RadioComms(){

}

RadioComms::~RadioComms(){

}

void RadioComms::Initialize(){
  radio.powerCycle();
  osDelay(1000);
  PRINTLN("Dumping Radio Reg 1");
  radio.dumpRegisters();
  radio.init();
  // GPS and Radio share interrupt
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  radio.clearFIFO();
  PRINTLN("Dumping Radio Reg 2 INIT");
  osDelay(500);
  radio.dumpRegisters();
  osDelay(500);
  radio.setMode(RFM69_MODE_RX);
}

void RadioComms::HandleRadioPacketReceived(){
  OnLedTwo();
  memset(received_packet, 0, RadioPacketLength);
  if (radio.receive(received_packet, RadioPacketLength)) {
    for (int i = 0; i < RadioPacketLength - 1; ++i) {
      PRINT("0x%X, ", received_packet[i]);
    }
    PRINTLN("0x%X", received_packet[RadioPacketLength - 1]);
  }
  else {
    PRINTLN("Radio Receiver read packet failed");
    radio.clearFIFO();
  }
  ProcessIncomingRadioPacket(RadioPacketLength, received_packet);
  OffLedTwo();
}

void RadioComms::HandleSendOutgoingMessage(uint8_t* buffer, uint16_t length){
  osDelay(50);
  radio.setMode(RFM69_MODE_TX);
  radio.send(buffer, length);
  bool sent = false;
  if (radio.packetSent(sent, 2000)) {
    PRINTLN("Outgoing Packet Sent");
  }
  radio.setMode(RFM69_MODE_RX);
}

void RadioComms::HandleSendOutgoingSensorData(uint8_t* buffer, uint16_t length) {
  radio.setMode(RFM69_MODE_TX);
  radio.send(buffer, length);
  bool sent = false;
  if (radio.packetSent(sent, 2000)) {
    PRINTLN("Sensor Packet Sent");
  }
  radio.setMode(RFM69_MODE_RX);
}

void RadioComms::HandleTxReady(){

}

void RadioComms::HandleRadioInterrupt()
{
  // Read register to see what to do next
  PRINTLN("HandleRadioInterrupt");

  uint8_t irq1 = 0;
  uint8_t irq2 = 0;

  if (radio.readRegIrqFlags(irq1, irq2)) {
    PRINTLN("irq1 0x%X irq2 0x%X", irq1, irq2);

    if (irq1 & RF_IRQFLAGS1_MODEREADY) {
      PRINTLN("Mode Ready");
    }

    if (irq1 & RF_IRQFLAGS1_RXREADY) {
      PRINTLN("Rx Ready");
    }

    if (irq1 & RF_IRQFLAGS1_TXREADY) {
      PRINTLN("Tx Ready");
    }

    if (irq1 & RF_IRQFLAGS1_PLLLOCK) {
      PRINTLN("PLL Lock");
    }

    if (irq1 & RF_IRQFLAGS1_RSSI) {
      PRINTLN("RSSI");
    }

    if (irq1 & RF_IRQFLAGS1_TIMEOUT) {
      PRINTLN("Timeout");
    }

    if (irq1 & RF_IRQFLAGS1_AUTOMODE) {
      PRINTLN("Auto Mode");
    }

    if (irq1 & RF_IRQFLAGS1_SYNCADDRESSMATCH) {
      PRINTLN("Sync Address Match");
    }

    if (irq2 & RF_IRQFLAGS2_FIFOFULL) {
      PRINTLN("FIFO Full");
    }

    if (irq2 & RF_IRQFLAGS2_FIFONOTEMPTY) {
      PRINTLN("FIFO Full");
    }

    if (irq2 & RF_IRQFLAGS2_FIFOLEVEL) {
      PRINTLN("FIFO Level");
    }

    if (irq2 & RF_IRQFLAGS2_FIFOOVERRUN) {
      PRINTLN("FIFO Overrun");
    }

    if (irq2 & RF_IRQFLAGS2_PACKETSENT) {
      radio.setMode(RFM69_MODE_STANDBY);
      radio.setMode(RFM69_MODE_RX);
      PRINTLN("Packet Sent");
    }

    if (irq2 & RF_IRQFLAGS2_PAYLOADREADY) {
      PRINTLN("Payload Ready");
    }

    if (irq2 & RF_IRQFLAGS2_CRCOK) {
      PRINTLN("CRC OK");
      HandleRadioPacketReceived();
    }

    if (irq2 & RF_IRQFLAGS2_LOWBAT) {
      PRINTLN("Low Bat");
    }
  }
}

extern "C" void StartRadioCommsTask(){
  radio.Initialize();
  radio.Run();
}

void RadioComms::Run(){
  osStatus_t status;
  uint8_t priority;
  TaskMessage queue_item;
  uint32_t queueWait { osWaitForever };

  while (true) {
    // Wait for accelerometer event
    status = osMessageQueueGet(RadioCommsQueueHandle, &queue_item.data, &priority, queueWait);
    if (status == osErrorTimeout) {
      PRINTLN("RadioComms queue timeout");
      continue;
    }
    else if (status != osOK) {
      PRINTLN("RadioComms acquire msg queue not ok Task");
      continue;
    }

    switch (queue_item.msg.id) {
      case static_cast<uint8_t>(TaskMsg::RADIO_INTERRUPT_1):
      case static_cast<uint8_t>(TaskMsg::RADIO_INTERRUPT_2):
      /*
       * Three interrupt cases all on the same pin
       * 1) Radio entered TX state, write packet to radio and clear the flags usingAccRadioMsg, usingAltRadioMsg or usingGpsRadioMsg
       * 2) Packet sent, configure for RX mode
       * 3) Packet Ready, read from FIFO and process
      */
      HandleRadioInterrupt();
      break;

      case static_cast<uint8_t>(TaskMsg::SEND_OUTGOING_MSG):
      HandleSendOutgoingMessage(static_cast<uint8_t*>(queue_item.msg.buffer), queue_item.msg.length);
      break;

      case static_cast<uint8_t>(TaskMsg::SEND_OUTGOING_SENSOR_DATA):
      HandleSendOutgoingSensorData(static_cast<uint8_t*>(queue_item.msg.buffer), queue_item.msg.length);
      break;

      case static_cast<uint8_t>(TaskMsg::SEND_STATS):
      WriteStatsRadioComms(0, queue_item.msg.buffer);
      break;

      case static_cast<uint8_t>(TaskMsg::START):

      break;

      case static_cast<uint8_t>(TaskMsg::STOP):

      break;

      default:
      break;
    }
  }
}

