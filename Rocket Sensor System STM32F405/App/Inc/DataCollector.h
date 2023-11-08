/*
 * DataCollector.h
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */

#ifndef DATACOLLECTOR_H_
#define DATACOLLECTOR_H_

#include <stdint.h>

#include "cmsis_os2.h"

#include "main.h"

#include "AccelerometerLsm.h"
#include "Bmp5.h"
#include "DataSample.h"

extern "C" void StartDataCollectionTask();

class DataCollector {
public:
  DataCollector();
  ~DataCollector();

  void Run();

private:
  void Initialize();
  void StartSensorSampling();
  void StopSensorSampling();

  bool ReadAccelerometerFifo(LSM6DSO& sensor, uint16_t& numberOfSensorDataReady, FifoStatusBits& statusBits);
  void HandleImuInterrupt();
  void HandleBarometerInterrupt();

  void SendSensorData();
  void SaveBuffer(TaskMessage& queue_item);

  LSM6DSO lowA;
  LSM6DSO highA;
  bmp5_dev barometer;
  bmp5_osr_odr_press_config osr_odr_press_cfg;
  bool is_sampling{false};

  friend void StartDataCollectionTask();
};

#endif /* DATACOLLECTOR_H_ */
