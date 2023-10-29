/*
 * RocketApp.cpp
 *
 *  Created on: Sep 3, 2023
 *      Author: johnt
 */

#include "RocketApp.h"

#include "main.h"

#include "ExternalMessages.h"
#include "DataSample.h"
#include "FileSystem.h"
#include "Global.h"

extern "C" {
  void StartStatusUpdatesTimer();
  void StopStatusUpdatesTimer();
}

extern osMessageQueueId_t RocketAppQueueHandle;
RocketApp rocket_app;

static uint8_t periodic_update_buffer[RadioPacketLength];

extern "C" void StartRocketAppTask(){
  rocket_app.Initialize();
  rocket_app.Run();
}

void RocketApp::Initialize(){
  mount_file_system();
  SystemConfig fconfig;
  read_config(fconfig);
}

void RocketApp::HandleNewConfiguration(uint8_t* msg, uint16_t length) {

}

void RocketApp::HandleSendConfiguration(uint8_t* msg, uint16_t length) {

}

void RocketApp::HandleSendDeviceStatus(uint8_t* msg, uint16_t length) {

}

void RocketApp::HandleRunCalibrartion(uint8_t* msg, uint16_t length) {

}

void RocketApp::HandleSendSensorData(uint8_t* msg, uint16_t length) {
  // Forward msg buffer for DataCollector to add the message.
  // Offset from 1 so the outgoing command ID can be added.
  GetSensorData(length-1, &msg[1]);
}

void RocketApp::HandleEngageFlightMode(uint8_t* msg, uint16_t length) {
  StartDataWriter();
  StartDataCollector();
  msg[0] = static_cast<uint8_t>(CommandId::FLIGHT_MODE_ENGAGED);
  SendOutgoingRadioPacket(1, msg);
  // Start 1 second timer/interrupt for status updates
  StartStatusUpdatesTimer();
}

void RocketApp::HandleDisengageFlightMode(uint8_t* msg, uint16_t length) {
  StopStatusUpdatesTimer();
}

void RocketApp::HandleEngageLowPower(uint8_t* msg, uint16_t length) {

}

void RocketApp::HandleDisengageLowPower(uint8_t* msg, uint16_t length) {

}

void RocketApp::ProcessSensorData(uint8_t* msg, uint16_t length) {
  // Add the command ID and forward to the RadioComms task for sending out
  msg[0] = static_cast<uint8_t>(CommandId::SENSOR_DATA);
  SendOutgoingRadioPacket(length+1, msg);
}

void RocketApp::HandlePeriodicFlightUpdate() {
  // Forward msg buffer for DataCollector to add the message.
  // Offset from 1 so the outgoing command ID can be added.
  GetSensorData(RadioPacketLength-1, &periodic_update_buffer[1]);
}

void RocketApp::ProcessRadioMessage(uint8_t* msg, uint16_t length) {
  auto id = static_cast<CommandId>(msg[0]);
  switch(id) {
    case CommandId::NEW_CONFIGURATION:
    HandleNewConfiguration(msg, length);
    break;

    case CommandId::SEND_CURRENT_CONFIGURATION:
    HandleSendConfiguration(msg, length);
    break;

    case CommandId::SEND_DEVICE_STATUS:
    HandleSendDeviceStatus(msg, length);
    break;

    case CommandId::RUN_CALIBRATION:
    HandleRunCalibrartion(msg, length);
    break;

    case CommandId::SEND_SENSORS_STATUS:
    HandleSendSensorData(msg, length);
    break;

    case CommandId::ENGAGE_FLIGHT_MODE:
    HandleEngageFlightMode(msg, length);
    break;

    case CommandId::DISENGAGE_FLIGHT_MODE:
    HandleDisengageFlightMode(msg, length);
    break;

    case CommandId::ENGAGE_LOW_POWER:
    HandleEngageLowPower(msg, length);
    break;

    case CommandId::DISENGAGE_LOW_POWER:
    HandleDisengageLowPower(msg, length);
    break;

    default:
    PRINTLN("RocketApp::ProcessRadioMessage invalid ID %u", uint32_t(id));
    break;
  }
}

void RocketApp::Run(){
  osStatus_t status { osStatusReserved };
  uint8_t priority { 0 }; // Not used by FreeRTOS
  TaskMessage queue_item { 0 };
  uint32_t queueWait { 2000 };

  StartStatsDefaultTask();
  StartRadioComms();

  // Call this to auto start collection and writing
  //HandleEngageFlightMode(nullptr, 0);

  while (true) {
    // Wait for accelerometer event
    status = osMessageQueueGet(RocketAppQueueHandle, &queue_item.data, &priority, queueWait);
    if (status == osErrorTimeout) {
      PRINTLN("RocketApp queue timeout");
      continue;
    }
    else if (status != osOK) {
      PRINTLN("RocketApp acquire msg queue not ok task");
      continue;
    }

    switch (queue_item.msg.id) {
      case static_cast<uint8_t>(TaskMsg::RECEIVED_RADIO_MSG):
      ProcessRadioMessage(static_cast<uint8_t*>(queue_item.msg.buffer), queue_item.msg.length);
      break;

      case static_cast<uint8_t>(TaskMsg::SEND_STATS):
      WriteStatsRocketApp(0, queue_item.msg.buffer);
      break;

      case static_cast<uint8_t>(TaskMsg::SENSOR_DATA):
      ProcessSensorData(static_cast<uint8_t*>(queue_item.msg.buffer), queue_item.msg.length);
      break;

      case static_cast<uint8_t>(TaskMsg::SEND_STATUS_UPDATE):
      HandlePeriodicFlightUpdate();
      break;

      default:
      break;
    }
  }
}
