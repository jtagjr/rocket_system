#pragma once
enum class MissionMsgId : uint8_t {
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
  STREAMING_MODE_ENGAGED,
  STREAMING_MODE_DISENGAGED,
  FLIGHT_MODE_ENGAGED,
  FLIGHT_MODE_DISENGAGED,
  SENSOR_DATA,
};

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

static constexpr uint8_t RadioPacketLength{64};
