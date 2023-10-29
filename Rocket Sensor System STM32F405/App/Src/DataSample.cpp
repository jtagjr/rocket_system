/*
 * DataSample.cpp
 *
 *  Created on: Sep 16, 2023
 *      Author: johnt
 */

#include <Global.h>
#include "DataSample.h"
#include "cmsis_os2.h"

extern osMessageQueueId_t DataCollectionQueueHandle;

extern osMessageQueueId_t RadioCommsQueueHandle;

extern osMessageQueueId_t DataWriterQueueHandle;

extern osMessageQueueId_t RocketAppQueueHandle;

extern osMessageQueueId_t DefaultTaskQueueHandle;

static inline void SendBuffer(osMessageQueueId_t queue, TaskMsg id, uint16_t meta_data, uint16_t length, void* buffer){
  TaskMessage info;
  info.msg.id = static_cast<uint8_t>(id);
  info.msg.meta_data = meta_data;
  info.msg.length = length;
  info.msg.buffer = buffer;
  osMessageQueuePut(queue, &info.data, 0, 0);
}

static inline void SendNoBuffer(osMessageQueueId_t queue, TaskMsg id, uint16_t meta_data = 0){
  TaskMessage info;
  info.msg.id = static_cast<uint8_t>(id);
  info.msg.meta_data = meta_data;
  info.msg.length = 0;
  info.msg.buffer = 0;
  osMessageQueuePut(queue, &info.data, 0, 0);
}

// To DefaultTask
extern "C" {

void StartStatsDefaultTask(){
  SendNoBuffer(DefaultTaskQueueHandle, TaskMsg::START);
}

void StopStatsDefaultTask(){
  SendNoBuffer(DefaultTaskQueueHandle, TaskMsg::STOP);
}

void WriteStatsDataCollector(uint16_t length, void* buffer){
  SendBuffer(DefaultTaskQueueHandle, TaskMsg::STATS_BUFFER, static_cast<uint16_t>(TaskIndex::DATA_COLLECTOR), length, buffer);
}

void WriteStatsDataWriter(uint16_t length, void* buffer){
  SendBuffer(DefaultTaskQueueHandle, TaskMsg::STATS_BUFFER, static_cast<uint16_t>(TaskIndex::DATA_WRITER), length, buffer);
}

void WriteStatsRadioComms(uint16_t length, void* buffer){
  SendBuffer(DefaultTaskQueueHandle, TaskMsg::STATS_BUFFER, static_cast<uint16_t>(TaskIndex::RADIO), length, buffer);
}

void WriteStatsRocketApp(uint16_t length, void* buffer){
  SendBuffer(DefaultTaskQueueHandle, TaskMsg::STATS_BUFFER, static_cast<uint16_t>(TaskIndex::ROCKET_APP), length, buffer);
}

// To DataCollector
void StartDataCollector(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::START);
}

void StopDataCollector(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::STOP);
}

void BufferSaved(uint8_t index, uint16_t length, void* buffer){
  SendBuffer(DataCollectionQueueHandle, TaskMsg::SAVED_BUFFER, index, length, buffer);
}

void SendStatsDataCollector(uint16_t length, void* buffer){
  SendBuffer(DataCollectionQueueHandle, TaskMsg::SEND_STATS, 0, length, buffer);
}

void GetSensorData(uint16_t length, void* buffer) {
  SendBuffer(DataCollectionQueueHandle, TaskMsg::GET_SENSOR_DATA, 0, length, buffer);
}

void HandleIrq_ACCEL1_INT1_Pin(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::ACCEL1_INTERRUPT_1);
}

void HandleIrq_ACCEL1_INT2_Pin(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::ACCEL1_INTERRUPT_2);
}

void HandleIrq_ACCEL2_INT1_Pin(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::ACCEL2_INTERRUPT_1);
}

void HandleIrq_BAROMETER_INT_Pin(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::BAROMETER_INTERRUPT);
}

void HandleIrq_GPS_INT_Pin(){
  SendNoBuffer(DataCollectionQueueHandle, TaskMsg::GPS_INTERRUPT_1);
}

// To DataWriter

void StartDataWriter(){
  SendNoBuffer(DataWriterQueueHandle, TaskMsg::START);
}

void StopDataWriter(){
  SendNoBuffer(DataWriterQueueHandle, TaskMsg::STOP);
}

void SaveBuffer(uint8_t index, uint16_t length, void* buffer){
  SendBuffer(DataWriterQueueHandle, TaskMsg::SAVE_BUFFER, index, length, buffer);
}

void SendStatsDataWriter(uint16_t length, void* buffer){
  SendBuffer(DataWriterQueueHandle, TaskMsg::SEND_STATS, 0, length, buffer);
}

// To RadioComms

void StartRadioComms(){
  SendNoBuffer(RadioCommsQueueHandle, TaskMsg::START);
}

void StopRadioComms(){
  SendNoBuffer(RadioCommsQueueHandle, TaskMsg::STOP);
}

void SendOutgoingRadioPacket(uint16_t length, void* buffer){
  SendBuffer(RadioCommsQueueHandle, TaskMsg::SEND_OUTGOING_MSG, 0, length, buffer);
}

void SendStatsRadioComms(uint16_t length, void* buffer){
  SendBuffer(RadioCommsQueueHandle, TaskMsg::SEND_STATS, 0, length, buffer);
}


// The GPIO EXT interrupt callback weak override is in GpioIrqHandler.cpp
// Radio interrupt. Pin is set as external interrupt EXT13 with a pull down  resistor
void HandleIrq_RADIO_INT1_Pin(){
  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(RADIO_INT1_GPIO_Port, RADIO_INT1_Pin)) {
    SendNoBuffer(RadioCommsQueueHandle, TaskMsg::RADIO_INTERRUPT_1, 0);
  }
}

// Unused interrupt. Pin is set as input pull down only
void HandleIrq_RADIO_INT2_Pin(){
  if (GPIO_PIN_SET == HAL_GPIO_ReadPin(RADIO_INT2_GPIO_Port, RADIO_INT2_Pin)) {
    SendNoBuffer(RadioCommsQueueHandle, TaskMsg::RADIO_INTERRUPT_2, 0);
  }
}

// To RocketApp
void ProcessIncomingRadioPacket(uint16_t length, void* buffer){
  SendBuffer(RocketAppQueueHandle, TaskMsg::RECEIVED_RADIO_MSG, 0, length, buffer);
}

void SendStatusUpdate() {
  SendBuffer(RocketAppQueueHandle, TaskMsg::SEND_STATUS_UPDATE, 0, 0, nullptr);
}

void SendStatsRocketApp(uint16_t length, void* buffer){
  SendBuffer(RocketAppQueueHandle, TaskMsg::SEND_STATS, 0, length, buffer);
}

void SensorDataRocketApp(uint16_t length, void* buffer) {
  SendBuffer(RocketAppQueueHandle, TaskMsg::SENSOR_DATA, 0, length, buffer);
}

}
