/*
 * ExternalMessages.h
 *
 *  Created on: Jan 16, 2023
 *      Author: johnt
 */

#ifndef INC_EXTERNALMESSAGES_H_
#define INC_EXTERNALMESSAGES_H_

enum class CommandId : uint8_t{
  // To RadioApp from External Device
  NEW_CONFIGURATION = 1,
  SEND_CURRENT_CONFIGURATION,
  SEND_DEVICE_STATUS,
  RUN_CALIBRATION,
  SEND_SENSORS_STATUS,
  ENGAGE_DATA_STREAMING_MODE,
  DISENGAGE_DATA_STREAMING_MODE,
  ENGAGE_FLIGHT_MODE,
  DISENGAGE_FLIGHT_MODE,
  ENGAGE_LOW_POWER,
  DISENGAGE_LOW_POWER,


  // To External Device from RadioApp
  SENSOR_DATA = 20,
};

static constexpr uint8_t RadioPacketLength{64};

struct CommandMessage {
  uint8_t cmd_id;
};

struct NewConfiguration {
  uint8_t cmd_id;
  uint8_t number_of_chute_deployments;
  uint16_t accelerometer_samples_per_sec;
  uint16_t first_deployment;
};

#endif /* INC_EXTERNALMESSAGES_H_ */
