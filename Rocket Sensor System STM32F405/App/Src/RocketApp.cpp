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
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
}

void RocketApp::HandleSendConfiguration(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
}

void RocketApp::HandleSendDeviceStatus(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
}

void RocketApp::HandleRunCalibrartion(uint8_t* msg, uint16_t length) {
  if (is_calibrating == false) {
    is_calibrating = true;
    SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
  }
}

void RocketApp::HandleEngageFlightMode(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
  if (is_writing == false) {
    is_writing = true;
    StartDataWriter();
  }
  if (is_sampling == false) {
    is_sampling = true;
    StartDataCollector();
    StartStatusUpdatesTimer();
  }
}

void RocketApp::HandleDisengageFlightMode(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
  if (is_sampling) {
    is_sampling = false;
    StopDataCollector();
    StopStatusUpdatesTimer();
  }
  if (is_writing) {
    is_writing = false;
    StopDataWriter();
  }
}

void RocketApp::HandleEngageDataStreaming(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
  if (is_sampling == false) {
    is_sampling = true;
    StartDataCollector();
    StartStatusUpdatesTimer();
  }
}

void RocketApp::HandleDisengageDataStreaming(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
  if (is_sampling) {
    is_sampling = false;
    StopDataCollector();
    StopStatusUpdatesTimer();
  }
}

void RocketApp::HandleEngageLowPower(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
}

void RocketApp::HandleDisengageLowPower(uint8_t* msg, uint16_t length) {
  SendOutgoingRadioPacket(length+1, msg); // Length + 1 byte for length field
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
  auto id = static_cast<CommandId>(msg[1]); // Length index 0, Msg ID index 1
  switch(id) {
    case CommandId::NEW_CONFIGURATION:
    HandleNewConfiguration(msg, msg[0]);
    break;

    case CommandId::SEND_CURRENT_CONFIGURATION:
    HandleSendConfiguration(msg, msg[0]);
    break;

    case CommandId::SEND_DEVICE_STATUS:
    HandleSendDeviceStatus(msg, msg[0]);
    break;

    case CommandId::RUN_CALIBRATION:
    HandleRunCalibrartion(msg, msg[0]);
    break;

    case CommandId::ENGAGE_DATA_STREAMING_MODE:
    HandleEngageDataStreaming(msg, msg[0]);
    break;

    case CommandId::DISENGAGE_DATA_STREAMING_MODE:
    HandleDisengageDataStreaming(msg, msg[0]);
    break;

    case CommandId::ENGAGE_FLIGHT_MODE:
    HandleEngageFlightMode(msg, msg[0]);
    break;

    case CommandId::DISENGAGE_FLIGHT_MODE:
    HandleDisengageFlightMode(msg, msg[0]);
    break;

    case CommandId::ENGAGE_LOW_POWER:
    HandleEngageLowPower(msg, msg[0]);
    break;

    case CommandId::DISENGAGE_LOW_POWER:
    HandleDisengageLowPower(msg, msg[0]);
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

      default:
      break;
    }
  }
}
