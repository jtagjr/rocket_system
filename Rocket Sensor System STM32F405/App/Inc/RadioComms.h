/*
 * RadioComms.h
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */

#ifndef RADIOCOMMS_H_
#define RADIOCOMMS_H_

#include <Rfm69.h>
#include "cmsis_os2.h"

extern "C" void StartRadioCommsTask();

class RadioComms {
public:
  RadioComms();
  ~RadioComms();

  void SendSensorData();
  void SendGpsData();
  void Run();

private:
  void Initialize();
  void HandleRadioInterrupt();
  void HandleTxReady();
  void HandlePacketSent();

  void HandleSendOutgoingMessage(uint8_t* buffer, uint16_t length);
  void HandleRadioPacketReceived();

  RFM69 radio;

  friend void StartRadioCommsTask();
};

#endif /* RADIOCOMMS_H_ */
