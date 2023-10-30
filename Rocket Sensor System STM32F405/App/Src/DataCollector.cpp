/*
 * DataCollector.cpp
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */

#include "DataCollector.h"

#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "main.h"

#include "AccelerometerLsm.h"
#include "Bmp5.h"
#include "DataSample.h"
#include "Global.h"
#include "Gps.h"

extern osMessageQueueId_t DataCollectionQueueHandle;

DataCollector collector;

// Shared with DataWriter
constexpr uint8_t MAX_BUFFERS { 2 };

static ImuFileEntry imu_sample_buffer[MAX_BUFFERS] { 0 };
static ImuFileEntry* current_imu_sample_buffer { &imu_sample_buffer[0] };
static ImuFileEntry* next_imu_sample_buffer { &imu_sample_buffer[1] };
static ImuFileEntry* log_imu_buffer{nullptr};

void AddNumImuSamplesRead(uint16_t numRead) {
  current_imu_sample_buffer->header.sample_number += numRead;
}

FifoData* GetNextImuEntry() {
  return &(current_imu_sample_buffer->sample[current_imu_sample_buffer->header.sample_number]);
}

uint16_t GetImuSampleBufferLength() {
  return sizeof(current_imu_sample_buffer->header) +
         sizeof(current_imu_sample_buffer->sample[0]) * current_imu_sample_buffer->header.sample_number;
}

uint16_t RemainingImuSamples() {
  return ImuFileEntry::MaxSamples - current_imu_sample_buffer->header.sample_number;
}

uint16_t MaxImuSamples() {
  return ImuFileEntry::MaxSamples;
}

uint16_t GetImuSampleBufferLength(const ImuFileEntry* entry) {
  if (entry->header.sample_number > 0)
    return sizeof(entry->header) +
           sizeof(entry->sample[0]) * entry->header.sample_number;
  return 0;
}

void SwitchImuBuffers() {
  log_imu_buffer = current_imu_sample_buffer;
  current_imu_sample_buffer = next_imu_sample_buffer;
  next_imu_sample_buffer = nullptr;
  if (current_imu_sample_buffer != nullptr) {
    current_imu_sample_buffer->header.sample_length = 0;
    current_imu_sample_buffer->header.sample_number = 0;
  }
}

static BarometerFileEntry barometer_sample_buffers[MAX_BUFFERS] { 0 };
static BarometerFileEntry* current_barometer_sample_buffer { &barometer_sample_buffers[0] };
static BarometerFileEntry* next_barometer_sample_buffer { &barometer_sample_buffers[1] };
static BarometerFileEntry* log_baro_buffer{nullptr};

void AddNumBarometerSamplesRead(uint16_t numRead) {
  current_barometer_sample_buffer->header.sample_number += numRead;
}

BarometerSample* GetNextBarometerEntry() {
  return &(current_barometer_sample_buffer->sample[current_barometer_sample_buffer->header.sample_number]);
}

uint16_t GetBarometerSampleBufferLength() {
  return sizeof(current_barometer_sample_buffer->header) +
         sizeof(current_barometer_sample_buffer->sample[0]) * current_barometer_sample_buffer->header.sample_number;
}

uint16_t RemainingBarometerSamples() {
  return BarometerFileEntry::MaxSamples - current_barometer_sample_buffer->header.sample_number;
}

uint16_t MaxBarometerSamples() {
  return BarometerFileEntry::MaxSamples;
}

uint16_t GetBarometerSampleBufferLength(const BarometerFileEntry* entry) {
  if (entry->header.sample_number > 0)
    return sizeof(entry->header) +
           sizeof(entry->sample[0]) * entry->header.sample_number;
  return 0;
}

void SwitchBarometerBuffers() {
  log_baro_buffer = current_barometer_sample_buffer;
  current_barometer_sample_buffer = next_barometer_sample_buffer;
  next_barometer_sample_buffer = nullptr;
  if (current_barometer_sample_buffer != nullptr) {
    current_barometer_sample_buffer->header.sample_length = 0;
    current_barometer_sample_buffer->header.sample_number = 0;
  }
}

extern SPI_HandleTypeDef hspi1;

void enable_dso32_high_acc_cs(){
  HAL_GPIO_WritePin(ACCEL2_CS_GPIO_Port, ACCEL2_CS_Pin, GPIO_PIN_RESET);
}

void disable_dso32_high_acc_cs(){
  HAL_GPIO_WritePin(ACCEL2_CS_GPIO_Port, ACCEL2_CS_Pin, GPIO_PIN_SET);
}

void enable_barometer_cs(){
  HAL_GPIO_WritePin(BAROMETER_CS_GPIO_Port, BAROMETER_CS_Pin, GPIO_PIN_RESET);
}

void disable_barometer_acc_cs(){
  HAL_GPIO_WritePin(BAROMETER_CS_GPIO_Port, BAROMETER_CS_Pin, GPIO_PIN_SET);
}

int32_t high_acc_spi_write(void* hspi, uint8_t regAddress, const uint8_t* data, uint16_t len){
  // SPI write handling code
  SPI_HandleTypeDef* p_spi = (SPI_HandleTypeDef*)hspi;
  enable_dso32_high_acc_cs();
  auto status = HAL_SPI_Transmit(p_spi, &regAddress, 1, 10);
  status = HAL_SPI_Transmit(p_spi, (uint8_t*)data, len, 20);
  disable_dso32_high_acc_cs();
  return status;
}

int32_t high_acc_spi_read(void* hspi, uint8_t regAddress, uint8_t* data, uint16_t len){
  // SPI read handling code
  SPI_HandleTypeDef* p_spi = (SPI_HandleTypeDef*)hspi;
  enable_dso32_high_acc_cs();
  regAddress = 0b10000000 | regAddress;
  auto status = HAL_SPI_Transmit(p_spi, &regAddress, 1, 10);
  status = HAL_SPI_Receive(p_spi, data, len, 20);
  disable_dso32_high_acc_cs();
  return status;
}

BMP5_INTF_RET_TYPE barometer_spi_write(uint8_t regAddress, const uint8_t* data, uint32_t len, void* intf_ptr){
  // SPI write handling code
  enable_barometer_cs();
  auto status = HAL_SPI_Transmit(&hspi1, &regAddress, 1, 10);
  status = HAL_SPI_Transmit(&hspi1, (uint8_t*)data, (uint16_t)len, 20);
  disable_barometer_acc_cs();
  return (BMP5_INTF_RET_TYPE)status;
}

BMP5_INTF_RET_TYPE barometer_spi_read(uint8_t regAddress, uint8_t* data, uint32_t len, void* intf_ptr){
  // SPI read handling code
  enable_barometer_cs();
  regAddress = 0b10000000 | regAddress;
  auto status = HAL_SPI_Transmit(&hspi1, &regAddress, 1, 10);
  status = HAL_SPI_Receive(&hspi1, data, (uint16_t)len, 20);
  disable_barometer_acc_cs();
  return (BMP5_INTF_RET_TYPE)status;
}

// Max 65,535us
extern "C" void delay_us(uint32_t period, void* intf_ptr){
  usDelay((uint16_t)period);
}

void LogData(char* buffer, int length);

DataCollector::DataCollector()
    : lowA(true), highA(false), barometer { 0 }, osr_odr_press_cfg { 0 }{
  barometer.intf = BMP5_SPI_INTF;
  barometer.intf_ptr = 0;
  barometer.intf_rslt = 0;
  barometer.read = &barometer_spi_read;
  barometer.write = &barometer_spi_write;
  barometer.delay_us = &delay_us;
}

DataCollector::~DataCollector(){

}

void DataCollector::StartSensorSampling(){
  EnableGPS();
  osDelay(2000);
  ConfigureGpsI2C();
  osDelay(100);
  ConfigureGpsTX();
  SetNavigationMeasurementRate();
  osDelay(20);
  SetNavigationSolutionMessageRate();
  StartNavCollection();

  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  // Accel Int 1 Low Acc
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  // Accel 1 INT 2
  // Shared with Radio interrupt
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // +-32g not +-16g
  lowA.startHighSpeedSampling(FS_XL_16g, ODR_XL_208Hz, FS_G_1000dps, ODR_GYRO_208Hz, FIFO_BDR_ACC_208Hz, FIFO_BDR_GYRO_208Hz, 96);

  StartPressureSensor(&osr_odr_press_cfg, &barometer);
}

void DataCollector::StopSensorSampling(){
  DisableGPS();
  lowA.sleep();
  if (StopPressureSensor(&barometer)) {
    PRINTLN("Pressure sensor failed to enter deep standby");
  }

  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);

  // Accel Int 1 Low Acc
  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  // EXTI15_10_IRQn is shared with Radio interrupt. Don't disable it
}

bool DataCollector::ReadAccelerometerFifo(LSM6DSO& sensor, uint16_t& numberOfSensorDataReady, FifoStatusBits& statusBits){
  // Get the number of samples in the FIFO
  bool result = sensor.getFifoStatus(numberOfSensorDataReady, statusBits);
  if (false == result) {
    PRINTLN("DataCollector::ReadAccelerometerFifo failed to get FIFO status");
  }
  return result;
}

void DataCollector::Initialize(){
  initialize_barometer(&barometer, &osr_odr_press_cfg);
  StopSensorSampling();
}

void DataCollector::HandleImuInterrupt(){
  uint16_t numberOfSensorSamplesReady = 0;
  FifoStatusBits statusBits { 0 };
  bool result = ReadAccelerometerFifo(lowA, numberOfSensorSamplesReady, statusBits);

  if (current_imu_sample_buffer == nullptr) {
    PRINTLN("HandleImuInterrupt no buffer dropping %u samples", numberOfSensorSamplesReady);
    return;
  }

  if (result) {
    // Determine if the number of samples will fit in the current buffer
    //  if notthen send this bufferand switch to the next
    if (RemainingImuSamples() < numberOfSensorSamplesReady) {
      uint16_t length = GetImuSampleBufferLength();
      current_imu_sample_buffer->header.sample_length = length;
      log_imu_buffer = current_imu_sample_buffer;
      SaveBuffer(1, length, current_imu_sample_buffer);
      SwitchImuBuffers();
    }

    if (current_imu_sample_buffer != nullptr) {
      FifoData* buffer = GetNextImuEntry();
      lowA.fifoRead(numberOfSensorSamplesReady, buffer);
    } else {
      PRINTLN("HandleImuInterrupt dropping %u samples", numberOfSensorSamplesReady);
      return;
    }

    current_imu_sample_buffer->header.sample_number += numberOfSensorSamplesReady;
  }
}

void DataCollector::HandleBarometerInterrupt(){
  bmp5_sensor_data data{0};
  if (current_imu_sample_buffer == nullptr) {
    PRINTLN("HandleBarometerInterrupt no buffer dropping barometer sample");
    return;
  }

  if (bmp5_get_sensor_data(&data, &osr_odr_press_cfg, &barometer) == BMP5_OK) {
    auto next = GetNextBarometerEntry();
    next->raw_pressure = data.raw_pressure;
    next->raw_temp = data.raw_temp;
    next->pascals = data.pressure;
    next->celsius = data.temperature;
    AddNumBarometerSamplesRead(1);
    // Determine if the number of samples will fit in the current buffer
    //  if notthen send this bufferand switch to the next
    if (RemainingBarometerSamples() == 0) {
      uint16_t length = GetBarometerSampleBufferLength();
      current_barometer_sample_buffer->header.sample_length = length;
      log_baro_buffer = current_barometer_sample_buffer;
      SaveBuffer(2, length, current_barometer_sample_buffer);
      SwitchBarometerBuffers();
    }
  }
}

void DataCollector::GetSensorData(uint8_t* buffer, const uint16_t length) {
  // buffer payload cannot exceed 66 bytes Radio FIFO length
  uint8_t i = 0;

  if (log_imu_buffer != nullptr) {
    if (log_imu_buffer->header.sample_number > 0) {
      const auto* data = &(log_imu_buffer->sample[log_imu_buffer->header.sample_number-1]);
      memcpy(buffer, data, sizeof(FifoData));
    } else {
      memset(buffer, 0xFF, sizeof(FifoData));
    }
    i += sizeof(FifoData);
  }
  // payload is now 8 bytes

  if (log_baro_buffer != nullptr) {
    if (log_baro_buffer->header.sample_number > 0) {
      const auto* data = &(log_baro_buffer->sample[log_imu_buffer->header.sample_number-1]);
      memcpy(&buffer[i], data, sizeof(BarometerSample));
    } else {
      memset(&buffer[i], 0xFF, sizeof(BarometerSample));
    }
    i += sizeof(BarometerSample);
  }
  // payload is now 24 bytes

  const auto* gps = GpsSample();
  if (gps != nullptr) {
    memcpy(&buffer[i], (void*)&(gps->sample.week), sizeof(gps->sample.week));
    i += sizeof(gps->sample.week);

    memcpy(&buffer[i], (void*)&(gps->sample.gpsFix), sizeof(gps->sample.gpsFix));
    i += sizeof(gps->sample.gpsFix);

    memcpy(&buffer[i], (void*)&(gps->sample.ecefX), sizeof(gps->sample.ecefX));
    i += sizeof(gps->sample.ecefX);

    memcpy(&buffer[i], (void*)&(gps->sample.ecefY), sizeof(gps->sample.ecefY));
    i += sizeof(gps->sample.ecefY);

    memcpy(&buffer[i], (void*)&(gps->sample.ecefZ), sizeof(gps->sample.ecefZ));
    i += sizeof(gps->sample.ecefZ);

    memcpy(&buffer[i], (void*)&(gps->sample.pAcc), sizeof(gps->sample.pAcc));
    i += sizeof(gps->sample.pAcc);

    memcpy(&buffer[i], (void*)&(gps->sample.eceVX), sizeof(gps->sample.eceVX));
    i += sizeof(gps->sample.eceVX);

    memcpy(&buffer[i], (void*)&(gps->sample.eceVY), sizeof(gps->sample.eceVY));
    i += sizeof(gps->sample.eceVY);

    memcpy(&buffer[i], (void*)&(gps->sample.eceVZ), sizeof(gps->sample.eceVZ));
    i += sizeof(gps->sample.eceVZ);

    memcpy(&buffer[i], (void*)&(gps->sample.sAcc), sizeof(gps->sample.sAcc));
    i += sizeof(gps->sample.sAcc);

    // payload is now 59 bytes
    SensorDataRocketApp(i, buffer);
  }
}

extern "C" void StartDataCollectionTask(){
  collector.Initialize();
  collector.Run();
}

void DataCollector::Run(){
  osStatus_t status;
  uint8_t priority;
  TaskMessage queue_item;
  uint32_t queueWait { osWaitForever };

  while (true) {
    // Wait for accelerometer event
    status = osMessageQueueGet(DataCollectionQueueHandle, &queue_item.data, &priority, queueWait);
    if (status != osOK && status != osErrorTimeout) {
      PRINTLN("DataCollector acquire msg queue not ok Task");
      continue;
    }

    switch (queue_item.msg.id) {
    case static_cast<uint8_t>(TaskMsg::ACCEL1_INTERRUPT_1):
      OnLedOne();
      TP0_On();
      HandleImuInterrupt();
      TP0_Off();
      OffLedOne();
      break;

    case static_cast<uint8_t>(TaskMsg::ACCEL1_INTERRUPT_2):
      OnLedTwo();
      HandleImuInterrupt();
      OffLedTwo();
      break;

    case static_cast<uint8_t>(TaskMsg::BAROMETER_INTERRUPT):
      TP1_On();
      HandleBarometerInterrupt();
      TP1_Off();
      break;

    case static_cast<uint8_t>(TaskMsg::SAVED_BUFFER):
      if (queue_item.msg.meta_data == 1) {
        ImuFileEntry** buffer = current_imu_sample_buffer == nullptr ? &current_imu_sample_buffer : &next_imu_sample_buffer;
        *buffer = static_cast<ImuFileEntry*>(queue_item.msg.buffer);
        if (log_imu_buffer != *buffer){
          (*buffer)->header.sample_length = 0;
          (*buffer)->header.sample_number = 0;
        }
      } else if (queue_item.msg.meta_data == 2) {
        BarometerFileEntry** buffer = current_barometer_sample_buffer == nullptr ? &current_barometer_sample_buffer : &next_barometer_sample_buffer;
        *buffer = static_cast<BarometerFileEntry*>(queue_item.msg.buffer);
        if (log_baro_buffer != *buffer){
          (*buffer)->header.sample_length = 0;
          (*buffer)->header.sample_number = 0;
        }
      } else if (queue_item.msg.meta_data == 3) {
        GpsBufferSaved(queue_item.msg.buffer);
      }
      break;

    case static_cast<uint8_t>(TaskMsg::SEND_STATS):
      LogData(static_cast<char*>(queue_item.msg.buffer), static_cast<int>(queue_item.msg.length));
      break;

    case static_cast<uint8_t>(TaskMsg::GET_SENSOR_DATA):
      GetSensorData(static_cast<uint8_t*>(queue_item.msg.buffer), queue_item.msg.length);
      break;

    case static_cast<uint8_t>(TaskMsg::START):
      StartSensorSampling();
      break;

    case static_cast<uint8_t>(TaskMsg::STOP):
      StopSensorSampling();
      break;

    default:
      PRINTLN("DataCollector ignoring Msg ID %u", uint16_t(queue_item.msg.id));
      break;
    }
  }
}

void LogData(char* buffer, int length) {
  size_t current_length{0};
  if (log_imu_buffer != nullptr && GetImuSampleBufferLength(log_imu_buffer) > 0) {
    current_length += snprintf(&buffer[current_length], length, "\nIMU DATA START\n");
    auto& header = log_imu_buffer->header;
    current_length += snprintf(&buffer[current_length], length, "IMU sample_type %u\n", header.sample_type);
    current_length += snprintf(&buffer[current_length], length, "IMU sample_length %u\n", header.sample_length);
    current_length += snprintf(&buffer[current_length], length, "IMU sample_crc %lu\n", header.header_crc);
    current_length += snprintf(&buffer[current_length], length, "IMU sample_time %llu\n", header.sample_time);
    current_length += snprintf(&buffer[current_length], length, "IMU sample_number %lu\n", header.sample_number);
    bool a_sample = false;
    bool g_sample = false;
    for(uint16_t i=0; i<header.sample_number; ++i) {
      current_length += snprintf(&buffer[current_length], length, "Sample number %u\n", i);
      auto sample = &log_imu_buffer->sample[i];
      if (sample->tag == 1) {
        g_sample = true;
        current_length += snprintf(&buffer[current_length], length, "Tag %u\n", uint16_t(sample->tag));
        // Multiplier is 2000 because +-1000 is 2000 range
        float data = (sample->x * 2000.0F)/uint32_t(0xFFFF);
        current_length += snprintf(&buffer[current_length], length, "Gyro X %f reg %i\n", data, sample->x);
        data = (sample->y * 2000.0F)/uint32_t(0xFFFF);
        current_length += snprintf(&buffer[current_length], length, "Gyro Y %f reg %i\n", data, sample->y);
        data = (sample->z * 2000.0F)/uint32_t(0xFFFF);
        current_length += snprintf(&buffer[current_length], length, "Gyro Z %f reg %i\n", data, sample->z);
      } else if (sample->tag == 2) {
        a_sample = true;
        current_length += snprintf(&buffer[current_length], length, "Tag %u\n", uint16_t(sample->tag));
        // Multiplier is 64 because +-32 is 64 range
        float data = (sample->x * 64.0F)/uint32_t(0xFFFF);
        current_length += snprintf(&buffer[current_length], length, "Accel X %f reg %i\n", data, sample->x);
        data = (sample->y * 64.0F)/uint32_t(0xFFFF);
        current_length += snprintf(&buffer[current_length], length, "Accel Y %f reg %i\n", data, sample->y);
        data = (sample->z * 64.0F)/uint32_t(0xFFFF);
        current_length += snprintf(&buffer[current_length], length, "Accel Z %f reg %i\n", data, sample->z);
      }
      if (a_sample && g_sample)
        break; // Only need to print one for console
    }
    current_length += snprintf(&buffer[current_length], length, "IMU DATA END\n");
  } else {
    current_length += snprintf(&buffer[current_length], length, "\nIMU buffer has 0 samples\n");
  }

  if (log_baro_buffer != nullptr && GetBarometerSampleBufferLength(log_baro_buffer) > 0) {
    current_length += snprintf(&buffer[current_length], length, "\nBarometer DATA START\n");
    auto& header = log_baro_buffer->header;
    current_length += snprintf(&buffer[current_length], length, "nBarometer sample_type %u\n", header.sample_type);
    current_length += snprintf(&buffer[current_length], length, "nBarometer sample_length %u\n", header.sample_length);
    current_length += snprintf(&buffer[current_length], length, "nBarometer sample_crc %lu\n", header.header_crc);
    current_length += snprintf(&buffer[current_length], length, "nBarometer sample_time %llu\n", header.sample_time);
    current_length += snprintf(&buffer[current_length], length, "nBarometer sample_number %lu\n", header.sample_number);
    for(uint16_t i=0; i<header.sample_number; ++i) {
      current_length += snprintf(&buffer[current_length], length, "Sample number %u\n", i);
      auto sample = &log_baro_buffer->sample[i];
      current_length += snprintf(&buffer[current_length], length, "celsius %f\n", sample->celsius);
      current_length += snprintf(&buffer[current_length], length, "pressure %f\n", sample->pascals);
      // Only need to print one for console
      break;
    }
    current_length += snprintf(&buffer[current_length], length, "nBarometer DATA END\n");
  } else {
    current_length += snprintf(&buffer[current_length], length, "\nBarometer buffer has 0 samples\n");
  }

  const NavigationFileEntry* entry = GpsSample();
  if (entry != nullptr) {
    current_length += snprintf(&buffer[current_length], length, "\nGPS DATA START\n");
    current_length += snprintf(&buffer[current_length], length, "GPS sample_type %u\n", entry->header.sample_type);
    current_length += snprintf(&buffer[current_length], length, "GPS sample_length %u\n", entry->header.sample_length);
    current_length += snprintf(&buffer[current_length], length, "GPS sample_crc %lu\n", entry->header.header_crc);
    current_length += snprintf(&buffer[current_length], length, "GPS sample_time %llu\n", entry->header.sample_time);
    current_length += snprintf(&buffer[current_length], length, "GPS sample_number %lu\n", entry->header.sample_number);
    current_length += snprintf(&buffer[current_length], length, "GPS msg class_type %u\n", entry->sample.class_type);
    current_length += snprintf(&buffer[current_length], length, "GPS msg id_type %u\n", entry->sample.id_type);
    current_length += snprintf(&buffer[current_length], length, "GPS payload_length %u\n", entry->sample.payload_length);
    current_length += snprintf(&buffer[current_length], length, "GPS Millisecond Time of Week %lu\n", entry->sample.iTOW);
    current_length += snprintf(&buffer[current_length], length, "Nanosecond Remainder %li\n", entry->sample.fTOW);
    current_length += snprintf(&buffer[current_length], length, "Week %i\n", entry->sample.week);
    current_length += snprintf(&buffer[current_length], length, "GPS Fix 0x%01X\n", entry->sample.gpsFix);
    current_length += snprintf(&buffer[current_length], length, "Flags 0x%01X\n", (entry->sample.flags & 0x0F));
    current_length += snprintf(&buffer[current_length], length, "ECEF X %li cm\n", entry->sample.ecefX);
    current_length += snprintf(&buffer[current_length], length, "ECEF Y %li cm\n", entry->sample.ecefY);
    current_length += snprintf(&buffer[current_length], length, "ECEF Z %li cm\n", entry->sample.ecefZ);
    current_length += snprintf(&buffer[current_length], length, "3D position accuracy estimate %lu cm\n", entry->sample.pAcc);
    current_length += snprintf(&buffer[current_length], length, "ECEV X %li cm/s\n", entry->sample.eceVX);
    current_length += snprintf(&buffer[current_length], length, "ECEV Y %li cm/s\n", entry->sample.eceVY);
    current_length += snprintf(&buffer[current_length], length, "ECEV Z %li cm/s\n", entry->sample.eceVZ);
    current_length += snprintf(&buffer[current_length], length, "3D speed accuracy estimate %lu  cm/s\n", entry->sample.sAcc);
    current_length += snprintf(&buffer[current_length], length, "pDOP %u\n", entry->sample.pDOP);
    current_length += snprintf(&buffer[current_length], length, "res1 %u\n", entry->sample.res1);
    current_length += snprintf(&buffer[current_length], length, "numSV %u\n", entry->sample.numSV);
    current_length += snprintf(&buffer[current_length], length, "res2 %lu\n", entry->sample.res2);
    current_length += snprintf(&buffer[current_length], length, "ck_a %u\n", entry->sample.ck_a);
    current_length += snprintf(&buffer[current_length], length, "ck_b %u\n", entry->sample.ck_b);
    current_length += snprintf(&buffer[current_length], length, "Number of SVs used %u\n", entry->sample.numSV);
    current_length += snprintf(&buffer[current_length], length, "Message Count %lu\n", GpsMsgCount());
    current_length += snprintf(&buffer[current_length], length, "Dropped Bytes %lu\n", GpsDroppedBytes());
    current_length += snprintf(&buffer[current_length], length, "Frame Errors %lu\n", GpsFrameErrors());
    current_length += snprintf(&buffer[current_length], length, "Overrun Errors %lu\n", GpsOverrunErrors());
    current_length += snprintf(&buffer[current_length], length, "Noise Errors %lu\n", GpsNoiseErrors());
    current_length += snprintf(&buffer[current_length], length, "GPS DATA END\n");
  } else {
    current_length += snprintf(&buffer[current_length], length, "\nGPS data sample is null\n");
  }

  WriteStatsDataCollector(current_length, buffer);
}
