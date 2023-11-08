/*
 * RadioComms.cpp
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */

#include "RadioComms.h"

#include "string.h"
#include <type_traits>

#include "cmsis_os2.h"

#include <Global.h>

extern "C" {

osMessageQueueId_t queueHandle;

static uint8_t received_packet[RadioPacketLength];
uint8_t packet_count = 1;

constexpr uint32_t usb_buffer_length { 64 };
static_assert(usb_buffer_length == RadioPacketLength, "error buffer length mismatch");

extern uint8_t* UserRxBufferFS; // Data received during ISR is stored here during the callback function CDC_Receive_FS
extern uint8_t* UserTxBufferFS; // Data to send is store here and retrieved in the callback function CDC_Transmit_FS

uint16_t rx_write_index{0};
uint8_t rx_next_length{0};
uint8_t rx_packet[usb_buffer_length];
uint8_t tx_packet[usb_buffer_length];

extern void CDC_Transmit_FS(uint8_t*, uint16_t);

void ProcessUsbRxData();
void HandleUsbMessage(uint8_t* data, uint8_t length);
}

class RadioCommsMessage
{
public:
  enum MessageID
  {
    START_RADIO,
    STOP_RADIO,
    RADIO_INTERRUPT,
    PRINT_STATS,
    USB_DATA,
    USB_MSG,
  };
};

extern "C"
{
void StartRadioCommsTask(osMessageQueueId_t handle) {
	queueHandle = handle;
	RadioComms comms;
	comms.Run();
}

void RadioCommsStartRadio() {
	TaskMessage item;
	item.msg.id = static_cast<uint8_t>(RadioCommsMessage::START_RADIO);
	osMessageQueuePut(queueHandle, &item, 0, 0);
}

void RadioCommsStopRadio() {
  TaskMessage item;
  item.msg.id = static_cast<uint8_t>(RadioCommsMessage::STOP_RADIO);
  osMessageQueuePut(queueHandle, &item, 0, 0);
}

void HandleRadioISR() {
  TaskMessage item;
  item.msg.id = static_cast<uint8_t>(RadioCommsMessage::RADIO_INTERRUPT);
  osMessageQueuePut(queueHandle, &item, 0, 0);
}

void RadioCommsPrintStats() {
  TaskMessage item;
  item.msg.id = static_cast<uint8_t>(RadioCommsMessage::PRINT_STATS);
  osMessageQueuePut(queueHandle, &item, 0, 0);
}

void HandleUsbMessage(uint8_t* data, uint8_t length) {
  TaskMessage item;
  item.msg.id = static_cast<uint8_t>(RadioCommsMessage::USB_MSG);
  item.msg.length = length;
  item.msg.buffer = data;
  osMessageQueuePut(queueHandle, &item, 0, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(__HAL_GPIO_EXTI_GET_IT(RADIO_PACKET_RECEIVED_INTERRUPT_Pin) == RESET)
	{
	  HandleRadioISR();
	}
}

} // End extern "C"

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
  CDC_Transmit_FS(received_packet, received_packet[0] + 1);
}

void RadioComms::HandleRadioSendStatusMsg()
{

}

void RadioComms::ProcessRadioMessage(uint8_t* msg, uint16_t length){
  // Pass the data straight up USB just a gateway
  // Need Default Task queue
  //SendBuffer(RocketAppQueueHandle, TaskMsg::RECEIVED_RADIO_MSG, 0, length, buffer);
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

void RadioComms::ForwardMissiongControlMsg(const uint8_t* data, const uint8_t length) {
  PRINTLN("Sending start data stream");
  radio.setMode(RFM69_MODE_STANDBY);
  osDelay(1);
  radio.setMode(RFM69_MODE_TX);
  radio.send(data, length);
  bool sent{false};
  if (radio.packetSent(sent, 2000)) {
    radio.setMode(RFM69_MODE_RX);
    PRINTLN("Packet Sent");
  } else {
    radio.setMode(RFM69_MODE_RX);
  }
}

void RadioComms::Run()
{
    osStatus_t status;
    uint8_t priority;
    TaskMessage queue_data;
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
        status = osMessageQueueGet(queueHandle, &queue_data, &priority, queueWait);
        if (status != osOK && status != osErrorTimeout) {
        	PRINTLN("RadioComms acquire msg queue not ok Task");
            continue;
        }

        switch (static_cast<uint8_t>(queue_data.msg.id)) {
          case RadioCommsMessage::USB_MSG:
          ForwardMissiongControlMsg(static_cast<uint8_t*>(queue_data.msg.buffer), queue_data.msg.length);
          break;

          case RadioCommsMessage::RADIO_INTERRUPT:
        	/*
        	 * Three interrupt cases all on the same pin
        	 * 1) Radio entered TX state, write packet to radio and clear the flags usingAccRadioMsg, usingAltRadioMsg or usingGpsRadioMsg
        	 * 2) Packet sent, configure for RX mode
        	 * 3) Packet Ready, read from FIFO and process
        	 */
          HandleRadioInterrupt();
        	break;

          case RadioCommsMessage::PRINT_STATS:
        	PrintStatusRegisters(radio);
        	break;

          default:
        	break;
        }
    }
}

extern "C" {

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
    HandleUsbMessage(rx_packet, rx_write_index);
    rx_write_index = 0;
    rx_next_length = 0;
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
}

// Callback from USB library during ISR
void UpperLayerRxPacket(uint8_t* packet) {
  for(uint16_t i=0; i<usb_buffer_length; ++i) {
    UsbParsePacket(packet[i]);
  }
}

void UpperLayerTxComplete(uint8_t* packet, uint16_t len) {

}

}



