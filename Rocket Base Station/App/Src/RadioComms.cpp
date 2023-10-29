/*
 * RadioComms.cpp
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */
#include <Global.h>
#include "RadioComms.h"
#include "cmsis_os2.h"

class Message
{
public:
	enum MessageID
	{
		START_RADIO,
		STOP_RADIO,
		RADIO_INTERRUPT,
		PRINT_STATS,
		SEND_START_DATA_STREAM,
	};
};

osMessageQueueId_t queueHandle;

extern "C"
{
void StartRadioCommsTask(osMessageQueueId_t handle)
{
	queueHandle = handle;
	RadioComms comms;
	comms.Run();
}

void RadioCommsStartRadio()
{
	constexpr uint8_t msg = Message::START_RADIO;
	osMessageQueuePut(queueHandle, &msg, 0, 0);
}

void RadioCommsStopRadio()
{
	constexpr uint8_t msg = Message::STOP_RADIO;
	osMessageQueuePut(queueHandle, &msg, 0, 0);
}

void HandleRadioISR()
{
	constexpr uint8_t msg = Message::RADIO_INTERRUPT;
	osMessageQueuePut(queueHandle, &msg, 0, 0);
}

void RadioCommsPrintStats()
{
	constexpr uint8_t msg = Message::PRINT_STATS;
	osMessageQueuePut(queueHandle, &msg, 0, 0);
}

void SendStartDataStream() {
  constexpr uint8_t msg = Message::SEND_START_DATA_STREAM;
  osMessageQueuePut(queueHandle, &msg, 0, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(__HAL_GPIO_EXTI_GET_IT(RADIO_PACKET_RECEIVED_INTERRUPT_Pin) == RESET)
	{
	  HandleRadioISR();
	}
}

}

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

RadioComms::RadioComms()
{


}

RadioComms::~RadioComms()
{

}

bool RadioComms::Initialize()
{
  radio.init();
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  radio.clearFIFO();
  PRINTLN("Dumping Radio Reg 2 INIT");
  osDelay(500);
  radio.dumpRegisters();
  osDelay(500);
  return radio.setMode(RFM69_MODE_RX);
}

void RadioComms::HandleRadioPacketReceived()
{
	// Read packet
	// Process packet
	PrintStatusRegisters(radio);
}

void RadioComms::HandleRadioSendStatusMsg()
{

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
      PRINTLN("FIFO Not Empty");
    }

    if (irq2 & RF_IRQFLAGS2_FIFOLEVEL) {
      PRINTLN("FIFO Level");
    }

    if (irq2 & RF_IRQFLAGS2_FIFOOVERRUN) {
      PRINTLN("FIFO Overrun");
    }

    if (irq2 & RF_IRQFLAGS2_PACKETSENT) {
      PRINTLN("Packet Sent");
      radio.setMode(RFM69_MODE_STANDBY);
      radio.setMode(RFM69_MODE_RX);
    }

    if (irq2 & RF_IRQFLAGS2_PAYLOADREADY) {
      PRINTLN("Packet Sent");
    }

    if (irq2 & RF_IRQFLAGS2_CRCOK) {
      PRINTLN("CRC OK");
      HandleRadioPacketReceived();
    }

    if (irq2 & RF_IRQFLAGS2_LOWBAT) {
      PRINTLN("CRC OK");
    }
  }
}

void RadioComms::HandleRadioTick()
{

}

void RadioComms::HandleSendStartDataStream() {
  PRINTLN("Sending start data stream");
  radio.setMode(RFM69_MODE_STANDBY);
  radio.setMode(RFM69_MODE_TX);
  uint8_t msg_id = 2;
  radio.send(&msg_id, 1);

  /*
  radio.setMode(RFM69_MODE_TX);
  uint8_t msg_id = 2;
  uint8_t packet[64];
  packet[0] = msg_id;
  radio.send(packet, 64);
  */
  bool sent = false;
  if (radio.packetSent(sent, 2000)) {
    PRINTLN("Packet Sent");
  }

  radio.setMode(RFM69_MODE_RX);
}

void RadioComms::Run()
{
    osStatus_t status;
    uint8_t priority;
    uint8_t msg;
    uint32_t queueWait = osWaitForever;

    radio.powerCycle();
    osDelay(1000);
    PRINTLN("Dumping Radio Reg 1");
    radio.dumpRegisters();
    while (Initialize() == false) ;


    StartPrintStatsTimer();

    while (true)
    {
        // Wait for accelerometer event
        status = osMessageQueueGet(queueHandle, &msg, &priority, queueWait);
        if (status != osOK && status != osErrorTimeout)
        {
        	PRINTLN("RadioComms acquire msg queue not ok Task");
            continue;
        }

        switch (msg)
        {
          case Message::SEND_START_DATA_STREAM:
          HandleSendStartDataStream();
          break;

          case Message::START_RADIO:
        	break;

          case Message::STOP_RADIO:
        	break;

          case Message::RADIO_INTERRUPT:
        	/*
        	 * Three interrupt cases all on the same pin
        	 * 1) Radio entered TX state, write packet to radio and clear the flags usingAccRadioMsg, usingAltRadioMsg or usingGpsRadioMsg
        	 * 2) Packet sent, configure for RX mode
        	 * 3) Packet Ready, read from FIFO and process
        	 */
          HandleRadioInterrupt();
        	break;

          case Message::PRINT_STATS:
        	PrintStatusRegisters(radio);
        	break;

          default:
        	break;
        }
    }
}




