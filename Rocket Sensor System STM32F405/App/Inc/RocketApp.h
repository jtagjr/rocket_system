/*
 * RocketApp.h
 *
 *  Created on: Sep 3, 2023
 *      Author: johnt
 */

#ifndef INC_ROCKETAPP_H_
#define INC_ROCKETAPP_H_

#include <stdint.h>
#include "cmsis_os.h"

extern "C" void StartRocketAppTask();

class RocketApp {
public:
  RocketApp(){
  }
  ~RocketApp(){
  }

private:
  enum class AppState : uint8_t {
    INIT,
    READ_CONFIG,
    STANDBY,
    LAUNCHPAD_CALIBRATION_RUNNING,
    READY_FOR_LAUNCH,
    FLYING,
    LANDED,
  };

  void Initialize();
  void Run();
  void ProcessRadioMessage(uint8_t* msg, uint16_t length);
  void HandleNewConfiguration(uint8_t* msg, uint16_t length);
  void HandleSendConfiguration(uint8_t* msg, uint16_t length);
  void HandleSendDeviceStatus(uint8_t* msg, uint16_t length);
  void HandleRunCalibrartion(uint8_t* msg, uint16_t length);
  void HandleSendSensorData(uint8_t* msg, uint16_t length);
  void HandleEngageFlightMode(uint8_t* msg, uint16_t length);
  void HandleDisengageFlightMode(uint8_t* msg, uint16_t length);
  void HandleEngageLowPower(uint8_t* msg, uint16_t length);
  void HandleDisengageLowPower(uint8_t* msg, uint16_t length);
  void ProcessSensorData(uint8_t* msg, uint16_t length);
  void HandlePeriodicFlightUpdate();

  friend void StartRocketAppTask();
};

#endif /* INC_ROCKETAPP_H_ */
