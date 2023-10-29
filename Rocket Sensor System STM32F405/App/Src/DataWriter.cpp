/*
 * DataWriter.cpp
 *
 *  Created on: Oct 30, 2021
 *      Author: johnt
 */

#include "DataWriter.h"

#include <stdio.h>
#include <fstream>

#include "main.h"
#include "fatfs.h"

#include "DataSample.h"
#include "Global.h"

extern FATFS fs;

extern osMessageQueueId_t DataWriterQueueHandle;

static FIL file;
static bool open = false;
static uint32_t samples_written { 0 };

DataWriter writer;

DataWriter::DataWriter(){

}

DataWriter::~DataWriter(){

}

void DataWriter::WriteBuffer(void* buffer, uint32_t length){
  if (open) {
    ++samples_written;
    UINT bytesWritten = 0;
    uint8_t* byte_buffer = (uint8_t*)buffer;
    while (bytesWritten != length) {
      UINT written = 0;
      f_write(&file, &(byte_buffer[bytesWritten]), length - bytesWritten, &written);
      bytesWritten += written;
    }
  }
}

void DataWriter::StartWriting(){
  int result = f_open(&file, "rocket.bin", FA_CREATE_ALWAYS | FA_WRITE);
  if (FR_OK == result) {
    open = true;
    PRINTLN("Created file rocket.bin");
  }
  else {
    PRINTLN("ERROR DataWriter FAILED TO CREATE FILE rocket_log.bin !!!!!!!!");
  }
}

void DataWriter::StopWriting(){
  if (open) {
    f_close(&file);
    open = false;
    PRINTLN("Closed file rocket.bin");
  }
}

extern "C" void StartDataWriterTask(){
  writer.Run();
}

void DataWriter::Run(){
  osStatus_t status;
  uint8_t priority;
  TaskMessage queue_item;
  uint32_t queueWait { osWaitForever };

  while (true) {
    // Wait for accelerometer event
    status = osMessageQueueGet(DataWriterQueueHandle, &queue_item.data, &priority, queueWait);
    if (status != osOK && status != osErrorTimeout) {
      PRINTLN("DataWriter acquire msg queue not ok Task");
      continue;
    }

    switch (queue_item.msg.id) {
      case static_cast<uint8_t>(TaskMsg::SAVE_BUFFER):
        TP3_On();
        WriteBuffer(queue_item.msg.buffer, queue_item.msg.length);
        BufferSaved(queue_item.msg.meta_data, queue_item.msg.length, queue_item.msg.buffer);
        TP3_Off();
        break;

      case static_cast<uint8_t>(TaskMsg::SEND_STATS):
        WriteStatsDataWriter(0, queue_item.msg.buffer);
        break;

      case static_cast<uint8_t>(TaskMsg::START):
        StartWriting();
        break;

      case static_cast<uint8_t>(TaskMsg::STOP):
        StopWriting();
        break;


      default:
        break;
    }
  }
}

