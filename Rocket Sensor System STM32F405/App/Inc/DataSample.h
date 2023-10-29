/*
 * DataSample.h
 *
 *  Created on: Oct 30, 2021
 *      Author: johnt
 */

#ifndef INC_DATASAMPLE_H_
#define INC_DATASAMPLE_H_

#include <stdint.h>
#include "cmsis_os2.h"
#include "AccelerometerLsm.h"

struct AccConfig {
  uint16_t g;
  uint16_t hz;
};

struct SystemConfig {
  AccConfig acc_one;
  AccConfig acc_two;
};

struct SystemCalibration {
  int16_t avgX;
  int16_t avgY;
  int16_t avgZ;
  float avgAltitude;
};

enum class DeviceState : uint8_t {
  LOW_POWER = 1,
  PINGING,
  FLIGHT_CALIBRATING,
  FLIGHT_STANDBY,
  FLIGHT_ASCENDING,
  FLIGHT_DECENDING,
};

enum class SampleType : uint8_t {
  IMU_SAMPLE = 1,
  BAROMETER_SAMPLE = 2,
  GPS_SAMPLE = 3,
};

enum class GpsFix : uint8_t {
  NO_FIX = 0,
  DEAD_RECKONING,
  TWO_D_FIX,
  THREE_D_FIX,
  GPS_DEAD_RECKONING_COMBINED,
  TIME_ONLY_FIX,
};

struct SamplesHeader {
  uint64_t sample_time; // microsecond timer CNT register that runs during data collection
  uint32_t sample_number;
  uint16_t sample_length;
  uint8_t sample_type;
  uint8_t res1; //pad
  uint32_t header_crc;
  uint32_t pad;
};

struct ImuFileEntry {
  static const uint16_t MaxSamples = 1000;

  SamplesHeader header;
  FifoData sample[MaxSamples];
} __attribute__ ((aligned(8)));

struct BarometerSample {
  int32_t raw_temp;
  uint32_t raw_pressure;
  float celsius;
  float pascals;
};

struct BarometerFileEntry {
  static const uint16_t MaxSamples = 100;

  SamplesHeader header;
  BarometerSample sample[MaxSamples];
} __attribute__ ((aligned(8)));

struct NavigationSolution {
  uint8_t class_type;
  uint8_t id_type;
  uint16_t payload_length;

  uint32_t iTOW;
  int32_t fTOW;
  int16_t week;
  uint8_t gpsFix;
  uint8_t flags;
  int32_t ecefX;
  int32_t ecefY;
  int32_t ecefZ;
  uint32_t pAcc;
  int32_t eceVX;
  int32_t eceVY;
  int32_t eceVZ;
  uint32_t sAcc;
  uint16_t pDOP;
  uint8_t res1;
  uint8_t numSV;
  uint32_t res2;

  uint8_t ck_a;
  uint8_t ck_b;
  uint16_t padding; // added to be a multiple of 4 bytes;
};

struct NavigationFileEntry {
  SamplesHeader header;
  NavigationSolution sample;
} __attribute__ ((aligned(8)));

enum class RadioMessageIDs : uint8_t {
  DEVICE_INFO,
  SENSOR_INFO,
  GPS_INFO,
  SET_FLIGHT_NAME,
  PREFLIGHT_CALIBRATE,
  START_RADIO,
  STOP_RADIO,
  START_FLIGHT,
  STOP_FLIGHT,
  GET_MESSAGE_RATES,
  SET_MESSAGE_RATES,
  SET_TX_POWER_LEVEL,
};

struct DeviceInfoMsg {
  uint8_t length;
  uint8_t receiverAddress;
  uint8_t msgId;
  uint8_t deviceState;
};

struct SensorInfoMsg {
  uint8_t msgId;
  uint8_t pad;
  int16_t low_temperature;
  int16_t low_gyroX;
  int16_t low_gyroY;
  int16_t low_gyroZ;
  int16_t low_accX;
  int16_t low_accY;
  int16_t low_accZ;
  int16_t high_temperature;
  int16_t high;
  int16_t high_gyroY;
  int16_t high_gyroZ;
  int16_t high_accX;
  int16_t high_accY;
  int16_t high_accZ;
  uint32_t calculated_altitude_feet;
};

struct GpsInfoMsg {
  uint8_t length;
  uint8_t receiverAddress;
  uint8_t msgId;
  uint8_t pad;

  uint32_t iTOW;
  int32_t fTOW;
  int16_t week;
  uint8_t gpsFix;
  uint8_t flags;
  int32_t ecefX;
  int32_t ecefY;
  int32_t ecefZ;
  uint32_t pAcc;
  int32_t eceVX;
  int32_t eceVY;
  int32_t eceVZ;
  uint32_t sAcc;
  uint16_t pDOP;
  uint8_t res1;
  uint8_t numSV;
};

// Ordered by priority
enum class TaskIndex : uint8_t {
  DATA_COLLECTOR,
  DATA_WRITER,
  RADIO,
  ROCKET_APP,
  DEFAULT_APP,
  TASK_COUNT,
};

enum class TaskMsg : uint8_t {
  START, // From RocketApp to all others
  STARTED, // From other tasks to RocketApp

  STOP,  // From RocketApp to all others
  STOPPED, // From other tasks to RocketApp

  SEND_STATS, // From Default Task to all other tasks

  STATS_BUFFER, // To Default Task

  SAVE_BUFFER, // To DataWriter

  ACCEL1_INTERRUPT_1, // DataCollector
  ACCEL1_INTERRUPT_2, // DataCollector
  ACCEL2_INTERRUPT_1, // DataCollector
  BAROMETER_INTERRUPT, // DataCollector
  GPS_INTERRUPT_1,
  GET_SENSOR_DATA, // To DataCollector from RadioApp
  SAVED_BUFFER, // To DataWriter

  SEND_OUTGOING_MSG, //From RocketApp to RadioComms
  RADIO_INTERRUPT_1, // To RadioComms
  RADIO_INTERRUPT_2, // To RadioComms

  RECEIVED_RADIO_MSG, // From RadioComms to RocketApp
  SENSOR_DATA, // From DataCollector to RocketApp
  SEND_STATUS_UPDATE // From 1 second ISR to RocketApp for periodic updates in flight mode
};

#define RocketPacketLength 256

struct Message {
  uint8_t id;
  uint8_t meta_data;
  uint16_t length;
  void* buffer;
};

struct TaskMessage {
  union {
    uint64_t data;
    Message msg;
  };
};

extern "C" {
// To DefaultTask
  void StartStatsDefaultTask();
  void StopStatsDefaultTask();
  void WriteStatsDataCollector(uint16_t length, void* buffer);
  void WriteStatsDataWriter(uint16_t length, void* buffer);
  void WriteStatsRadioComms(uint16_t length, void* buffer);
  void WriteStatsRocketApp(uint16_t length, void* buffer);

// To DataCollector
  void StartDataCollector();
  void StopDataCollector();
  void BufferSaved(uint8_t index, uint16_t length, void* buffer);
  void SendStatsDataCollector(uint16_t length, void* buffer);
  void GetSensorData(uint16_t length, void* buffer);
  void HandleIrq_ACCEL1_INT1_Pin();
  void HandleIrq_ACCEL1_INT2_Pin();
  void HandleIrq_ACCEL2_INT1_Pin();
  void HandleIrq_BAROMETER_INT_Pin();
  void HandleIrq_GPS_INT_Pin();

// To DataWriter
  void StartDataWriter();
  void StopDataWriter();
  void SaveBuffer(uint8_t index, uint16_t length, void* buffer);
  void SendStatsDataWriter(uint16_t length, void* buffer);

// To RadioComms
  void StartRadioComms();
  void StopRadioComms();
  void SendOutgoingRadioPacket(uint16_t length, void* buffer);
  void SendStatsRadioComms(uint16_t length, void* buffer);
  void HandleIrq_RADIO_INT1_Pin();
  void HandleIrq_RADIO_INT2_Pin();

// To RocketApp
  void ProcessIncomingRadioPacket(uint16_t length, void* buffer);
  void SendStatusUpdate();
  void SendStatsRocketApp(uint16_t length, void* buffer);
  void SensorDataRocketApp(uint16_t length, void* buffer);
}

#endif /* INC_DATASAMPLE_H_ */
