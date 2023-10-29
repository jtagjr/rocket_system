/**
 * @file rfm69.cpp
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
 * You have to provide your own functions for delay_ms() and mstimer_get().
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

/** @addtogroup RFM69
 * @{
 */

#include <Rfm69.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal_spi.h"
#include "cmsis_os2.h"

extern SPI_HandleTypeDef hspi2;

#define TIMEOUT_MODE_READY    100 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   100 ///< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_CSMA_READY    500 ///< Maximum CSMA wait time for channel free detection [ms]
#define CSMA_RSSI_THRESHOLD   -85 ///< If RSSI value is smaller than this, consider channel as free [dBm]

/** RFM69 base configuration after init().
 *
 * Change these to your needs or call setCustomConfig() after module init.
 */
static const uint8_t rfm69_base_config[][2] = { { 0x01, 0x04 }, // RegOpMode: 0x04 Standby needed to init these registers,  0x09 is RX Mode, 0x10 is TX mode
    { 0x02, 0x00 }, // RegDataModul: Packet mode, FSK, no shaping
    { 0x03, 0x0C }, // RegBitrateMsb: 10 kbps
    { 0x04, 0x80 }, // RegBitrateLsb
    { 0x05, 0x01 }, // RegFdevMsb: 20 kHz
    { 0x06, 0x52 }, // RegFdevLsb
    { 0x07, 0xe4 }, // RegFrfMsb: 915 MHz
    { 0x08, 0xc0 }, // RegFrfMid
    { 0x09, 0x9A }, // RegFrfLsb
    { 0x18, 0x08 }, // RegLNA: 50 Ohm impedance, gain set by AGC loop
    { 0x19, 0x4C }, // RegRxBw: 25 kHz
    { 0x25, 0x40 }, // Generate GPIO Pin 0, Interrupt PayloadReady during RX mode and TxReady interrupt during TX Mode
    { 0x29, 0x8F }, // RegFifoThresh: TxStart on FifoNotEmpty(at least one byte in FIFO), 15 bytes FifoLevel
    { 0x2C, 0x00 }, // RegPreambleMsb:5 bytes preamble
    { 0x2D, 0x05 }, // RegPreambleLsb
    { 0x2E, 0x88 }, // RegSyncConfig: Enable sync word, 2 bytes sync word
    { 0x2F, 0x41 }, // RegSyncValue1: 0x4148
    { 0x30, 0x48 }, // RegSyncValue2
    { 0x37, 0x10 }, // RegPacketConfig1: Fixed length, CRC on, whitening
    { 0x38, 0x40 }, // RegPayloadLength: 64 byte packet fixed length
    { 0x3C, 0x80 }, // TxStartCondition FifoNotEmpty
    { 0x58, 0x1B }, // RegTestLna: Normal sensitivity mode
    { 0x6F, 0x30 }, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)
    };

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]

/**
 * RFM69 default constructor. Use init() to start working with the RFM69 module.
 *
 * @param spi Pointer to a SPI device
 * @param csGPIO GPIO of /CS line (ie. GPIOA, GPIOB, ...)
 * @param csPin Pin of /CS line (eg. GPIO_Pin_1)
 * @param highPowerDevice Set to true, if this is a RFM69Hxx device (default: false)
 */
RFM69::RFM69() {
  _mode = BOOT;
  _rssi = -127;
}

RFM69::~RFM69() {

}

bool RFM69::powerCycle() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = RADIO_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_RESET_GPIO_Port, &GPIO_InitStruct);
  osDelay(1);
  // Let reset float per datasheet
  HAL_GPIO_DeInit(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin);
  osDelay(20);// 5ms or greater
  return true;
}

/**
 * Initialize the RFM69 module.
 * A base configuration is set and the module is put in standby mode.
 *
 * @return Always true
 */
bool RFM69::init() {
  // set base configuration
  setMode(RFM69_MODE_STANDBY);
  bool result = setCustomConfig(rfm69_base_config, sizeof(rfm69_base_config) / 2);
  return result;
}

bool RFM69::setStandbyMode() {
  bool result = false;
  if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode) {
    if (false == setMode(RFM69_MODE_STANDBY)) {
      PRINTLN("setFrequency failed to set mode to %u from mode %u", RFM69_MODE_STANDBY, _mode);
      _mode = RFM69_MODE_STANDBY;
      result = true;
    }
  } else {
    result = true;
  }
  return result;
}

/**
 * Set the carrier frequency in Hz.
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Carrier frequency in Hz
 */
bool RFM69::setFrequency(const uint32_t frequency) {
  bool result = false;
  // switch to standby if TX/RX was active
  if (setStandbyMode()) {
    // calculate register value
    const uint32_t freq = frequency / RFM69_FSTEP;

    // set new frequency
    if (writeRegister(0x07, freq >> 16)) {
      if (writeRegister(0x08, freq >> 8)) {
        if (writeRegister(0x09, freq)) {
          result = true;
        } else {
          PRINTLN("setFrequency failed third write.");
        }
      } else {
        PRINTLN("setFrequency failed second write.");
      }
    } else {
      PRINTLN("setFrequency failed first write.");
    }
  } else {
    PRINTLN("setFrequency failed to set mode to %u from mode %u", RFM69_MODE_STANDBY, _mode);
  }
  return result;
}

/**
 * Set the bitrate in bits per second.
 * After calling this function, the module is in standby mode.
 *
 * @param bitrate Bitrate in bits per second
 */
bool RFM69::setBitrate(const uint32_t bitrate) {
  // switch to standby if TX/RX was active
  bool result = false;
  if (setStandbyMode()) {
    // calculate register value
    const uint16_t rate = RFM69_XO / bitrate;

    // set new bitrate
    if (writeRegister(0x03, rate >> 8)) {
      if (writeRegister(0x04, rate)) {
        result = true;
      } else {
        PRINTLN("setBitrate failed second write.");
      }
    } else {
      PRINTLN("setBitrate failed first write.");
    }
  } else {
    PRINTLN("setFrequency failed to set mode to %u from mode %u", RFM69_MODE_STANDBY, _mode);
  }
  return result;
}

/**
 * Read a RFM69 register value.
 *
 * @param reg The register to be read
 * @return The value of the register
 */
bool RFM69::readRegister(const uint8_t regAddress, uint8_t& value) {
  bool result = false;
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
    osDelay(1); // spin
  }

  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);
  if (HAL_OK != HAL_SPI_Transmit(&hspi2, (uint8_t*)&regAddress, 1, 10)) {
    PRINTLN("readRegister address transmit failed [0x%X]: 0x%X", regAddress);
  } else if (HAL_OK != HAL_SPI_Receive(&hspi2, &value, 1, 10)) {
    PRINTLN("readRegister data receive failed [0x%X]", regAddress);
  } else {
    result = true;
  }
  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);
  return result;
}

bool RFM69::readRegisters(const uint8_t regAddress, uint8_t *data, const uint8_t length) {
  bool result = false;
  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
    osDelay(1); // spin
  }

  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);
  if (HAL_OK != HAL_SPI_Transmit(&hspi2, (uint8_t*)&regAddress, 1, 10)) {
    PRINTLN("readRegisters address transmit failed [0x%X]: 0x%X", regAddress);
  } else if (HAL_OK != HAL_SPI_Receive(&hspi2, data, length, 10)) {
    PRINTLN("readRegisters data receive failed [0x%X]: bytes requested %u", regAddress, length);
  } else {
    result = true;
  }
  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);
  return result;
}

/**
 * Write a RFM69 register value.
 *
 * @param reg The register to be written
 * @param value The value of the register to be set
 */
bool RFM69::writeRegister(const uint8_t regAddress, const uint8_t value) {
  bool result = false;

  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
    osDelay(1); // spin
  }

  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);
  const uint8_t writeAddress = 0b10000000 | regAddress;
  if (HAL_OK != HAL_SPI_Transmit(&hspi2, (uint8_t*)&writeAddress, 1, 10)) {
    PRINTLN("writeRegister address transmit failed [0x%X]: 0x%X", regAddress);
  } else if (HAL_OK != HAL_SPI_Transmit(&hspi2, (uint8_t*)&value, 1, 10)) {
    PRINTLN("writeRegister value transmit failed [0x%X]: 0x%X", regAddress, value);
  } else {
    result = true;
  }
  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);
  return result;
}

bool RFM69::readRegIrqFlags(uint8_t& flags1, uint8_t& flags2) {
  uint8_t data[2];
  bool result = readRegisters(0x27, data, 2);
  if (result) {
    flags1 = data[0];
    flags2 = data[1];
  }
  return result;
}

bool RFM69::getMode(RFM69Mode& mode) {
  uint8_t value = 0;
  bool result = false;
  if ((result = readRegister(0x01, value))) {
    mode = (RFM69Mode)((value >> 2) & 0x07);
  }
  return result;
}

bool RFM69::pollForModeChange(const RFM69Mode mode, const uint32_t timeoutInMs) {
  RFM69Mode currentMode = BOOT;
  uint32_t now = 0;
  while (now < timeoutInMs) {
    if (getMode(currentMode) && mode == currentMode) {
      return true;
    } else {
      osDelay(1);
      ++now;
    }
  }
  return false;
}

bool RFM69::packetSent(bool& sent, uint32_t timeoutInMs) {
  uint32_t now = 0;
  uint8_t value = 0;
  while (now < timeoutInMs) {
    if (readRegister(0x28, value) && (value & 0x08) == 0x08) {
      sent = true;
      return true;
    } else {
      osDelay(50);
      now += 50;
    }
  }
  return false;
}

/**
 * Switch the mode of the RFM69 module.
 * Using this function you can manually select the RFM69 mode (sleep for example).
 *
 * This function also takes care of the special registers that need to be set when
 * the RFM69 module is a high power device (RFM69Hxx).
 *
 * This function is usually not needed because the library handles mode changes automatically.
 *
 * @param mode RFM69_MODE_SLEEP, RFM69_MODE_STANDBY, RFM69_MODE_FS, RFM69_MODE_TX, RFM69_MODE_RX
 * @return The new mode
 */
bool RFM69::setMode(const RFM69Mode mode) {
  bool result = false;
  RFM69Mode currentMode = BOOT;
  if (getMode(currentMode)) {
    if (mode == currentMode) {
      _mode = mode;
      return true;
    }

    if (mode == RFM69_MODE_RX) {
      if (false == setHighPowerSettings(false)) {
        PRINTLN("RFM69::setMode failed to disable RFM69 to high power!");
      }
    }

    if (mode == RFM69_MODE_TX) {
      if (setHighPowerSettings(true)) {
        //PRINTLN("RFM69::setMode enabled RFM69 TX high power!");
      } else {
        PRINTLN("RFM69::setMode failed to enable RFM69 TX high power!");
      }
    }

    if (writeRegister(0x01, mode << 2)) {
      if (pollForModeChange(mode, 100)) {
        _mode = mode;
        result = true;
      } else {
        PRINTLN("RFM69::setMode failed to set RFM69 mode to %u from %u polling timeout!", mode, currentMode);
      }
    } else {
      PRINTLN("RFM69::setMode failed to set RFM69 mode to %u from %u", mode, currentMode);
    }
  } else {
    PRINTLN("RFM69::setMode failed to read RFM69 mode while in mode %u!", currentMode);
  }
  return result;
}

/**
 * Reconfigure the RFM69 module by writing multiple registers at once.
 *
 * @param config Array of register/value tuples
 * @param length Number of elements in config array
 */
bool RFM69::setCustomConfig(const uint8_t config[][2], const uint32_t length) {
  bool result = true;
  for (unsigned int i = 0; i < length; i++) {
    if (writeRegister(config[i][0], config[i][1])) {
      uint8_t value = 0;
      if (readRegister(config[i][0], value)) {
        if (config[i][1] != value) {
          result = false;
          PRINTLN("setCustomConfig failed to set register 0x%X to 0x%X received 0x%X", config[i][0], config[i][1], value);
        }
      } else {
        result = false;
        PRINTLN("setCustomConfig failed to read register 0x%X after writing value 0x%X", config[i][0], config[i][1]);
      }
    }
  }
  return result;
}

/**
 * Send a packet over the air.
 *
 * After sending the packet, the module goes to standby mode.
 * CSMA/CA is used before sending if enabled by function setCSMA() (default: off).
 *
 * @note A maximum amount of RFM69_MAX_PAYLOAD bytes can be sent.
 * @note This function blocks until packet has been sent.
 *
 * @param data Pointer to buffer with data
 * @param dataLength Size of buffer
 *
 * @return Number of bytes that have been sent
 */
bool RFM69::send(const uint8_t *data, const uint16_t dataLength) {
  // transfer packet to FIFO
  bool result = false;

  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);

  // Set DIO0 interrupt to 0 for packet sent
  uint8_t regAddress = 0b10000000; // FIFO register is address 0;
  if (HAL_OK == HAL_SPI_Transmit(&hspi2, &regAddress, 1, 10)) {
    if (HAL_OK == HAL_SPI_Transmit(&hspi2, (uint8_t*) data, dataLength, 10)) {
      result = true;
    } else {
      PRINTLN("Failed to transmit SPI radio packet");
    }
  } else {
    PRINTLN("Failed to transmit SPI radio fifo reg address");
  }

  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);
  return result;
}

/**
 * Clear FIFO and flags of RFM69 module.
 */
void RFM69::clearFIFO() {
  // clear flags and FIFO
  if (false == writeRegister(0x28, 0x10)) {
    PRINTLN("Failed writeRegister to clear radio FIFO!");
  }
}

bool RFM69::setHighPowerSettings(const bool enable) {
  writeRegister(0x13, enable ? 0x0F : 0x10);
  if (enable) { // Max power
    writeRegister(REG_PALEVEL, RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | 31);
  } // bitwise & don't short circuit
  return writeRegister(0x5A, enable ? 0x5D : 0x55) &
         writeRegister(0x5C, enable ? 0x7C : 0x70);
}

/**
 * Put the RFM69 module to sleep (lowest power consumption).
 */
bool RFM69::sleep() {
  return setMode(RFM69_MODE_SLEEP);
}

/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
bool RFM69::receive(uint8_t *data, const uint16_t dataLength) {
  // check if there is a packet in the internal buffer and copy it
  uint8_t regAddress = 0;
  bool result = false;
  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_RESET);
  if (HAL_OK == HAL_SPI_Transmit(&hspi2, &regAddress, 1, 10)) {
    if (HAL_OK == HAL_SPI_Receive(&hspi2, data, dataLength, 10)) {
      result = true;
    } else {
      PRINTLN("Failed to receive header RFM69");
    }
  } else {
    PRINTLN("Failed to write reg address for receive RFM69");
  }
  HAL_GPIO_WritePin(SPI2_CS_RADIO_GPIO_Port, SPI2_CS_RADIO_Pin, GPIO_PIN_SET);
  return result;
}


bool RFM69::receivedPacket(bool& hasPacket) {
  bool result = false;
  uint8_t value = 0;
  if ((result = readRegister(0x28, value))) {
    hasPacket = 0x04 == (value & 0x04);
  }
  return result;
}

/**
 * Read the last RSSI value.
 *
 * @note Only if the last RSSI value was above the RSSI threshold, a sample can be read.
 *       Otherwise, you always get -127 dBm. Be also careful if you just switched to RX mode.
 *       You may have to wait until a RSSI sample is available.
 *
 * @return RSSI value in dBm.
 */
bool RFM69::readRSSI(int& rssi) {
  uint8_t value = 0;
  bool result = false;
  if ((result = readRegister(0x24, value))) {
    rssi = (-1*value)/2;
    _rssi = rssi;
  }
  return result;
}

/**
 * Debug function to dump all RFM69 registers.
 *
 * Symbol 'DEBUG' has to be defined.
 */
void RFM69::dumpRegisters(void) {
#ifdef DEBUG

  PRINTLN("Dumping radio registers");
  uint8_t data[0x71] = { 0 };
  readRegisters(1, data, 0x71);
  for (unsigned int i = 0; i <= 0x70; i++) {
    PRINTLN("[0x%X]: 0x%X\n", i + 1, data[i]);
  }
#endif
}

/**
 * Check if the channel is free using RSSI measurements.
 *
 * This function is part of the CSMA/CA algorithm.
 *
 * @return true = channel free; otherwise false.
 */
bool RFM69::channelFree() {
  int rssi = 0;
  if (readRSSI(rssi) && rssi < CSMA_RSSI_THRESHOLD) {
    _rssi = rssi;
    return true;
  } else {
    return false;
  }
}

/** @}
 *
 */
