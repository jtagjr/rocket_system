/**
 * @file rfm69.hpp
 *
 * @brief RFM69 and RFM69HW library for sending and receiving packets in connection with a STM32 controller.
 * @date January, February 2015
 * @author André Heßling
 *
 * This is a protocol agnostic driver library for handling HopeRF's RFM69 433/868/915 MHz RF modules.
 * Support is also available for the +20 dBm high power modules called RFM69HW/RFM69HCW.
 *
 * A CSMA/CA (carrier sense multiple access) algorithm can be enabled to avoid collisions.
 * If you want to enable CSMA, you should initialize the random number generator before.
 *
 * This library is written for the STM32 family of controllers, but can easily be ported to other devices.
 *
 * You have to provide your own functions for delay_ms and mstimer_get.
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

#ifndef RFM69_HPP_
#define RFM69_HPP_

#include <Global.h>
#include <Rfm69Regs.h>
#include "main.h"

/** @addtogroup RFM69
 * @{
 */
#define RFM69_MAX_PAYLOAD		64 ///< Maximum bytes payload

/**
 * Valid RFM69 operation modes.
 */
typedef enum {
  RFM69_MODE_SLEEP = 0, //!< Sleep mode (lowest power consumption)
  RFM69_MODE_STANDBY,  //!< Standby mode
  RFM69_MODE_FS,       //!< Frequency synthesizer enabled
  RFM69_MODE_TX,       //!< TX mode (carrier active)
  RFM69_MODE_RX,       //!< RX mode
  BOOT = 0xFF,
} RFM69Mode;

/**
 * Valid RFM69 data modes.
 */
typedef enum {
  RFM69_DATA_MODE_PACKET = 0,                 //!< Packet engine active
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,   //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,   //!< Continuous mode without clock recovery
} RFM69DataMode;

/** RFM69 driver library for STM32 controllers. */
class RFM69 {
  /** @addtogroup RFM69
   * @{
   */
public:
  RFM69();

  ~RFM69();

  bool init();

  bool enable();

  bool powerCycle();

  bool setFrequency(const uint32_t frequency);

  bool setFrequencyDeviation(const uint32_t frequency);

  bool setBitrate(const uint32_t bitrate);

  bool getMode(RFM69Mode& mode);

  bool setMode(const RFM69Mode mode);

  void setPowerLevel(const uint8_t power);

  bool setPowerDBm(const int8_t dBm);

  bool setHighPowerSettings(const bool enable);

  bool setCustomConfig(const uint8_t config[][2], const uint32_t length);

  bool send(const uint8_t* data, const uint16_t dataLength);

  bool packetSent(bool& sent, uint32_t timeoutInMs);

  bool readRegIrqFlags(uint8_t& flags1, uint8_t& flags2);

  bool receive(uint8_t* data, const uint16_t dataLength);

  bool receivedPacket(bool& hasPacket);

  bool sleep();

  void setOOKMode(const bool enable);

  void setDataMode(const RFM69DataMode dataMode = RFM69_DATA_MODE_PACKET);

  void continuousBit(const bool bit);

  void dumpRegisters();

  bool setAESEncryption(const void* aesKey, const unsigned int keyLength);

  bool readRegisters(const uint8_t regAddress, uint8_t* data, const uint8_t length);

  bool readRegister(const uint8_t regAddress, uint8_t& value);

  bool writeRegister(const uint8_t reg, const uint8_t value);

  void clearFIFO();

  bool readRSSI(int& rssi);

  bool channelFree();

  bool setStandbyMode();

private:

  bool pollForModeChange(const RFM69Mode mode, const uint32_t timeoutInMs);

  RFM69Mode _mode;
  int _rssi;
};

#endif /* RFM69_HPP_ */

/** @}
 *
 */
