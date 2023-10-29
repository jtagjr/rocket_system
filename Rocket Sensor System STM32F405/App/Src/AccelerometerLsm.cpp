/******************************************************************************
 SparkFunLSM6DSO.cpp
 LSM6DSO Arduino and Teensy Driver

 Marshall Taylor @ SparkFun Electronics
 May 20, 2015
 https://github.com/sparkfun/LSM6DSO_Breakout
 https://github.com/sparkfun/SparkFun_LSM6DSO_Arduino_Library

 Resources:
 Uses Wire.h for i2c operation
 Uses SPI.h for SPI operation
 Either can be omitted if not used

 Development environment specifics:
 Arduino IDE 1.6.4
 Teensy loader 1.23
 0
 This code is released under the [MIT License](http://opensource.org/licenses/MIT).

 Please review the LICENSE.md file included with this example. If you have any questions
 or concerns with licensing, please contact techsupport@sparkfun.com.

 Distributed as-is; no warranty is given.
 ******************************************************************************/
#include <AccelerometerLsm.h>
#include <Global.h>
#include "main.h"
#include "stm32f4xx_hal_spi.h"
#include "cmsis_os2.h"

extern SPI_HandleTypeDef hspi1;

void enable_low_acc_cs(){
  HAL_GPIO_WritePin(ACCEL1_CS_GPIO_Port, ACCEL1_CS_Pin, GPIO_PIN_RESET);
}

void disable_low_acc_cs(){
  HAL_GPIO_WritePin(ACCEL1_CS_GPIO_Port, ACCEL1_CS_Pin, GPIO_PIN_SET);
}

void enable_high_acc_cs(){
  HAL_GPIO_WritePin(ACCEL2_CS_GPIO_Port, ACCEL2_CS_Pin, GPIO_PIN_RESET);
}

void disable_high_acc_cs(){
  HAL_GPIO_WritePin(ACCEL2_CS_GPIO_Port, ACCEL2_CS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef LSM6DSOCore::spi_write(uint8_t regAddress, uint8_t* data, uint16_t len){
  // SPI write handling code
  (*enable_cs)();
  auto status = HAL_SPI_Transmit(&hspi1, &regAddress, 1, 1);
  status = HAL_SPI_Transmit(&hspi1, data, len, 10);
  (*disable_cs)();
  return status;
}

HAL_StatusTypeDef LSM6DSOCore::spi_read(uint8_t regAddress, uint8_t* data, uint16_t len){
  // SPI read handling code
  (*enable_cs)();
  regAddress = 0b10000000 | regAddress;
  auto status = HAL_SPI_Transmit(&hspi1, &regAddress, 1, 1);
  status = HAL_SPI_Receive(&hspi1, data, len, 10);
  (*disable_cs)();
  return status;
}

LSM6DSOCore::LSM6DSOCore(bool lowAcc){
  if (lowAcc) {
    enable_cs = enable_low_acc_cs;
    disable_cs = disable_low_acc_cs;
  }
  else {
    enable_cs = enable_high_acc_cs;
    disable_cs = disable_high_acc_cs;
  }
}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    address -- register to read
//    numBytes -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LSM6DSOCore::readMultipleRegisters(uint8_t outputPointer[], uint8_t address, uint8_t numBytes){
  return spi_read(address, outputPointer, numBytes) == HAL_OK ? IMU_SUCCESS : IMU_HW_ERROR;
}

//****************************************************************************//
//  readRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    address -- register to read
//****************************************************************************//
status_t LSM6DSOCore::readRegister(uint8_t* outputPointer, uint8_t address){
  return spi_read(address, outputPointer, 1) == HAL_OK ? IMU_SUCCESS : IMU_HW_ERROR;
}

//****************************************************************************//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    address -- register to read
//****************************************************************************//
status_t LSM6DSOCore::readRegisterInt16(int16_t* outputPointer, uint8_t address){
  status_t status = IMU_HW_ERROR;
  uint8_t myBuffer[2];
  if (spi_read(address, myBuffer, 2) == HAL_OK) {
    *outputPointer = myBuffer[0] | static_cast<uint16_t>(myBuffer[1] << 8);
    status = IMU_SUCCESS;
  }
  else {
    *outputPointer = 0xFFFF;
  }
  return status;
}

//****************************************************************************//
//  writeRegister
//
//  Parameters:
//    address -- register to write
//    dataToWrite -- 8 bit data to write to register
//****************************************************************************//
status_t LSM6DSOCore::writeRegister(uint8_t address, uint8_t dataToWrite){
  return spi_write(address, &dataToWrite, 1) == HAL_OK ? IMU_SUCCESS : IMU_HW_ERROR;
}

//****************************************************************************//
//  writeMultipleRegisters
//
//  Parameters:
//    inputPointer -- array to be written to device
//    address -- register to write
//    numBytes -- number of bytes contained in the array
//****************************************************************************//
status_t LSM6DSOCore::writeMultipleRegisters(uint8_t inputPointer[], uint8_t address, uint8_t numBytes){
  return spi_write(address, inputPointer, numBytes) == HAL_OK ? IMU_SUCCESS : IMU_HW_ERROR;
}

status_t LSM6DSOCore::enableEmbeddedFunctions(bool enable){
  uint8_t tempVal;
  readRegister(&tempVal, FUNC_CFG_ACCESS);

  tempVal &= 0x7F;

  if (enable)
    tempVal |= 0x80;
  else
    tempVal |= 0x7F;

  return writeRegister(FUNC_CFG_ACCESS, tempVal);
}

bool LSM6DSO::GetSensorData(SensorData* data){
  return HAL_OK == spi_read(OUT_TEMP_L, (uint8_t*)data, sizeof(SensorData));
}

//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LSM6DSO::LSM6DSO(bool lowAcc)
    : LSM6DSOCore(lowAcc){
  //Construct with these default imuSettings

  imuSettings.gyroEnabled = true;  //Can be 0 or 1
  imuSettings.gyroRange = 500;   //Max deg/s.  Can be: 125, 250, 500, 1000, 2000
  imuSettings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imuSettings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  imuSettings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  imuSettings.gyroAccelDecimation = 1;  //Set to include gyro in FIFO

  imuSettings.accelEnabled = true;
  imuSettings.accelRange = 8;      //Max G force readable.  Can be: 2, 4, 8, 16
  imuSettings.accelSampleRate = 416;  //Hz.  Can be: 1.6 (16), 12.5 (125), 26, 52, 104, 208, 416, 833, 1660, 3330, 6660
  imuSettings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO

  imuSettings.fifoEnabled = true;
  imuSettings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
  imuSettings.fifoSampleRate = 416;
  imuSettings.fifoModeWord = 0;  //Default off

  allOnesCounter = 0;
  nonSuccessCounter = 0;
}

bool LSM6DSO::startHighSpeedSampling(LSM6DSO_FS_XL accel_range,
                                     LSM6DSO_ODR_XL accel_odr_rate,
                                     LSM6DSO_FS_G gyro_range,
                                     LSM6DSO_ODR_GYRO_G gyro_odr_rate,
                                     LSM6DSO_BDR_XL_FIFO accel_bdr_rate,
                                     LSM6DSO_BDR_GY_FIFO gyro_bdr_rate,
                                     uint8_t watermark){
  bool result = false;

  softwareReset();
  osDelay(1);

  // Enable auto register incrementing on IMU
  if (!setIncrement()) {
    PRINTLN("Failed to set increment on LSM6DSO");
    return false;
  }

  // Disable interrupts on IMU
  uint8_t data[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  if (IMU_SUCCESS != readMultipleRegisters(data, WHO_AM_I_REG, 1)) {
    PRINTLN("Failed to disable interrupts on LSM6DSO");
    return false;
  }

  if (IMU_SUCCESS != writeMultipleRegisters(data, INT1_CTRL, 2)) {
    PRINTLN("Failed to disable interrupts on LSM6DSO");
    return false;
  }

  // Power down accelerometer and gyro
  data[0] = 0;
  data[1] = 0;
  if (IMU_SUCCESS != writeMultipleRegisters(data, CTRL1_XL, 2)) {
    PRINTLN("Failed to disable interrupts on LSM6DSO");
    return false;
  }

  // Set watermark for fifo to interrupt
  // Set fifo to continue/not stop
  // Disable compression
  data[0] = watermark; // FIFO_STOP_ON_WTM_DISABLED
  // 1 is Watermark flag rises when the number of bytes written in the FIFO is greater than or equal to the threshold level.
  data[1] = FIFO_STOP_ON_WTM_DISABLED | FIFO_COMPR_RT_DISABLED | FIFO_ODRCHG_DISABLED | FIFO_UNCOPTR_RATE_8 | 1;
  data[2] = accel_bdr_rate | gyro_bdr_rate;
  data[3] = FIFO_TS_DEC_DISABLED | FIFO_TEMP_ODR_DISABLE | FIFO_MODE_CONTINUOUS;
  data[4] = 0; //COUNTER_BDR_REG1
  data[5] = 0; //COUNTER_BDR_REG2
  data[6] = INT1_FIFO_TH_ENABLED;
  data[7] = INT2_FIFO_OVR_ENABLED | INT2_FIFO_FULL_ENABLED;
  if (IMU_SUCCESS != writeMultipleRegisters(data, FIFO_CTRL1, 8)) {
    PRINTLN("Failed to configure FIFO and interrupts on LSM6DSO");
    return false;
  }

  data[0] = 0;
  readMultipleRegisters(data, FIFO_CTRL1, 1);
  // Setup CTRL Registser 3 to 10
  data[0] = BOOT_NORMAL_MODE | BDU_CONTINUOS | INT_ACTIVE_LOW | PP_OD_PUSH_PULL | SIM_4_WIRE | IF_INC_ENABLED | SW_RESET_NORMAL_MODE; // CTRL3_C
  data[1] = I2C_DISABLE_SPI_ONLY; // CTRL4_C
  data[2] = 0; // CTRL5_C
  data[3] = HIGH_PERF_ACC_ENABLE; // CTRL6_C
  data[4] = 0; // CTRL7_G
  data[5] = 0; // CTRL8_XL
  if (IMU_SUCCESS != writeMultipleRegisters(data, CTRL3_C, 6)) {
    PRINTLN("Failed to configure FIFO and interrupts on LSM6DSO");
    return false;
  }

  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  if (IMU_SUCCESS != writeMultipleRegisters(data, X_OFS_USR, 3)) {
    PRINTLN("Failed to configure FIFO and interrupts on LSM6DSO");
    return false;
  }

  uint16_t numberOfSensorSamplesReady = 0;
  FifoStatusBits statusBits { 0 };
  bool status = getFifoStatus(numberOfSensorSamplesReady, statusBits);
  uint16_t numSamplesRead = fifoRead(numberOfSensorSamplesReady, nullptr);

  // Start sampling
  data[0] = accel_range | accel_odr_rate; // CTRL1_XL
  data[1] = gyro_range | gyro_odr_rate; // CTRL2_G
  if (IMU_SUCCESS != writeMultipleRegisters(data, CTRL1_XL, 2)) {
    PRINTLN("Failed to configure FIFO and interrupts on LSM6DSO");
    return false;
  }

  data[0] = 0;
  data[1] = 0;
  if (IMU_SUCCESS != readMultipleRegisters(data, CTRL1_XL, 2)) {
    PRINTLN("Failed to disable interrupts on LSM6DSO");
    return false;
  }

  return result;
}

bool LSM6DSO::sleep(){
  setIncrement();
  uint8_t data[2] { 0, 0 };
  if (IMU_SUCCESS != writeMultipleRegisters(data, CTRL1_XL, 2)) {
    PRINTLN("Failed to put accelerometer to sleep mode");
    return false;
  }
  return true;
}

bool LSM6DSO::initialize(uint8_t settings){
  setIncrement();

  if (settings == BASIC_SETTINGS) {
    setAccelRange(2);
    setAccelDataRate(416);
    setGyroRange(1000);
    setGyroDataRate(1660);
    setBlockDataUpdate(true);
  }
  else if (settings == SOFT_INT_SETTINGS) {
    setAccelRange(8);
    setAccelDataRate(416);
    setGyroRange(500);
    setGyroDataRate(416);
  }
  else if (settings == HARD_INT_SETTINGS) {
    setInterruptOne(INT1_DRDY_XL_ENABLED);
    setInterruptTwo(INT2_DRDY_G_ENABLED);
    setAccelRange(8);
    setAccelDataRate(416);
    setGyroRange(500);
    setGyroDataRate(416);
  }

  return true;
}

// Address: 0x1E , bit[2:0]: default value is: 0x00
// Checks if there is new accelerometer, gyro, or temperature data.
uint8_t LSM6DSO::listenDataReady(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, STATUS_REG);

  if (returnError != IMU_SUCCESS)
    return IMU_GENERIC_ERROR;
  else
    return regVal;
}

// Address:0x12 CTRL3_C , bit[6] default value is: 0x00
// This function sets the BDU (Block Data Update) bit. Use when not employing
// the FIFO buffer.
bool LSM6DSO::setBlockDataUpdate(bool enable){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL3_C);
  if (returnError != IMU_SUCCESS)
    return false;

  // Clear boot and sw reset, interrupts active low, int pin push pull, 4 wire SPI, auto increment register with read/write
  // Set block data update (no FIFO)
  regVal = 0b01100100;

  returnError = writeRegister(CTRL3_C, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;

}

// Address:0x0D , bit[7:0]: default value is: 0x00
// Sets whether the accelerometer, gyroscope, or FIFO trigger on hardware
// interrupt one. Error checking for the user's argument is tricky (could be a
// long list of "if not this and not this and not this" instead the function relies on the
// user to set the correct value.
bool LSM6DSO::setInterruptOne(uint8_t setting){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, INT1_CTRL);
  if (returnError != IMU_SUCCESS)
    return false;

  regVal &= 0xFE;
  regVal |= setting;

  returnError = writeRegister(INT1_CTRL, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address:0x0D , bit[7:0]: default value is: 0x00
// Gets whether the accelerometer, gyroscope, or FIFO trigger on hardware
// interrupt one.
uint8_t LSM6DSO::getInterruptOne(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, INT1_CTRL);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return regVal;
}

// Address: 0x12, bit[5,4]: default value is: 0x00
// Configures the polarity of the hardware interrupts and whether they are
// push-pull or open-drain.
bool LSM6DSO::configHardOutInt(uint8_t polarity, uint8_t pushOrDrain){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL3_C);
  if (returnError != IMU_SUCCESS)
    return false;

  regVal &= 0xCF;
  regVal |= polarity;
  regVal |= pushOrDrain;

  returnError = writeRegister(CTRL3_C, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}
// Address:0x0E , bit[7:0]: default value is: 0x00
// Sets whether the accelerometer, gyroscope, temperature sensor or FIFO trigger on hardware
// interrupt two. Error checking for the user's argument is tricky (could be a
// long list of "if not this and not this and not this" instead the function relies on the
// user to set the correct value.
bool LSM6DSO::setInterruptTwo(uint8_t setting){

  status_t returnError = writeRegister(INT2_CTRL, setting);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;

}

// Address:0x15 , bit[4]: default value is: 0x00
// Sets whether high performance mode is on for the acclerometer, by default it is ON.
bool LSM6DSO::setHighPerfAccel(bool enable){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL6_C);
  if (returnError != IMU_SUCCESS)
    return false;

  if (enable)
    regVal |= HIGH_PERF_ACC_ENABLE;
  else
    regVal |= HIGH_PERF_ACC_DISABLE;

  returnError = writeRegister(CTRL6_C, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address:0x16 , bit[7]: default value is: 0x00
// Sets whether high performance mode is on for the gyroscope, by default it is ON.
bool LSM6DSO::setHighPerfGyro(bool enable){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL7_G);
  if (returnError != IMU_SUCCESS)
    return false;

  if (enable)
    regVal |= HIGH_PERF_GYRO_ENABLE;
  else
    regVal |= HIGH_PERF_GYRO_DISABLE;

  returnError = writeRegister(CTRL7_G, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//

// Address: 0x10 , bit[4:3]: default value is: 0x00 (2g)
// Sets the acceleration range of the accleromter portion of the IMU.
bool LSM6DSO::setAccelRange(uint8_t range){

  if ((range < 0) | (range > 16))
    return false;

  uint8_t regVal;
  uint8_t fullScale;
  status_t returnError = readRegister(&regVal, CTRL1_XL);
  if (returnError != IMU_SUCCESS)
    return false;

  fullScale = getAccelFullScale();

  // Can't have 16g with XL_FS_MODE == 1
  if (fullScale == 1 && range == 16)
    range = 8;

  regVal &= FS_XL_MASK;

  switch (range) {
    case 2:
      regVal |= FS_XL_2g;
      break;
    case 4:
      regVal |= FS_XL_4g;
      break;
    case 8:
      regVal |= FS_XL_8g;
      break;
    case 16:
      regVal |= FS_XL_16g;
      break;
    default:
      break;
  }

  returnError = writeRegister(CTRL1_XL, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address: 0x10 , bit[4:3]: default value is: 0x00 (2g)
// Gets the acceleration range of the accleromter portion of the IMU.
// The value is dependent on the full scale bit (see getAccelFullScale).
uint8_t LSM6DSO::getAccelRange(){

  uint8_t regVal;
  uint8_t fullScale;

  status_t returnError = readRegister(&regVal, CTRL1_XL);
  if (returnError != IMU_SUCCESS)
    return IMU_GENERIC_ERROR;

  fullScale = getAccelFullScale();
  regVal = (regVal & 0x0C) >> 2;

  if (fullScale == 1) {
    switch (regVal) {
      case 0:
        return 2;
      case 1:
        return 2;
      case 2:
        return 4;
      case 3:
        return 8;
      default:
        return IMU_GENERIC_ERROR;
    }
  }
  else if (fullScale == 0) {
    switch (regVal) {
      case 0:
        return 2;
      case 1:
        return 16;
      case 2:
        return 4;
      case 3:
        return 8;
      default:
        return IMU_GENERIC_ERROR;
    }
  }
  else
    return IMU_GENERIC_ERROR;

}

// Address: 0x10, bit[7:4]: default value is: 0x00 (Power Down)
// Sets the output data rate of the accelerometer there-by enabling it.
bool LSM6DSO::setAccelDataRate(uint16_t rate){

  if ((rate < 16) | (rate > 6660))
    return false;

  uint8_t regVal;
  uint8_t highPerf;
  status_t returnError = readRegister(&regVal, CTRL1_XL);
  if (returnError != IMU_SUCCESS)
    return false;

  highPerf = getAccelHighPerf();

  // Can't have 1.6Hz and have high performance mode enabled.
  if (highPerf == 0 && rate == 16)
    rate = 125;

  regVal &= ODR_XL_MASK;

  switch (rate) {
    case 0:
      regVal |= ODR_XL_DISABLE;
      break;
    case 16:
      regVal |= ODR_XL_1_6Hz;
      break;
    case 125:
      regVal |= ODR_XL_12_5Hz;
      break;
    case 26:
      regVal |= ODR_XL_26Hz;
      break;
    case 52:
      regVal |= ODR_XL_52Hz;
      break;
    case 104:
      regVal |= ODR_XL_104Hz;
      break;
    case 208:
      regVal |= ODR_XL_208Hz;
      break;
    case 416:
      regVal |= ODR_XL_416Hz;
      break;
    case 833:
      regVal |= ODR_XL_833Hz;
      break;
    case 1660:
      regVal |= ODR_XL_1660Hz;
      break;
    case 3330:
      regVal |= ODR_XL_3330Hz;
      break;
    case 6660:
      regVal |= ODR_XL_6660Hz;
      break;
    default:
      break;
  }

  returnError = writeRegister(CTRL1_XL, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address: 0x10, bit[7:4]: default value is: 0x00 (Power Down)
// Gets the output data rate of the accelerometer checking if high performance
// mode is enabled in which case the lowest possible data rate is 12.5Hz.
float LSM6DSO::getAccelDataRate(){

  uint8_t regVal;
  uint8_t highPerf;

  status_t returnError = readRegister(&regVal, CTRL1_XL);
  highPerf = getAccelHighPerf();

  if (returnError != IMU_SUCCESS)
    return static_cast<float>(IMU_GENERIC_ERROR);

  regVal &= ~ODR_XL_MASK;

  switch (regVal) {
    case 0:
      return ODR_XL_DISABLE;
    case ODR_XL_1_6Hz: // Can't have 1.6 and high performance mode
      if (highPerf == 0)
        return 12.5;
      return 1.6;
    case ODR_XL_12_5Hz:
      return 12.5;
    case ODR_XL_26Hz:
      return 26.0;
    case ODR_XL_52Hz:
      return 52.0;
    case ODR_XL_104Hz:
      return 104.0;
    case ODR_XL_208Hz:
      return 208.0;
    case ODR_XL_416Hz:
      return 416.0;
    case ODR_XL_833Hz:
      return 833.0;
    case ODR_XL_1660Hz:
      return 1660.0;
    case ODR_XL_3330Hz:
      return 3330.0;
    case ODR_XL_6660Hz:
      return 6660.0;
    default:
      return static_cast<float>(IMU_GENERIC_ERROR);
  }

}

// Address: 0x15, bit[4]: default value is: 0x00 (Enabled)
// Checks wheter high performance is enabled or disabled.
uint8_t LSM6DSO::getAccelHighPerf(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL6_C);

  if (returnError != IMU_SUCCESS)
    return IMU_GENERIC_ERROR;
  else
    return ((regVal & 0x10) >> 4);

}

// Address: 0x17, bit[2]: default value is: 0x00
// Checks whether the acclerometer is using "old" full scale or "new", see
// datasheet for more information.
uint8_t LSM6DSO::getAccelFullScale(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL8_XL);

  if (returnError != IMU_SUCCESS)
    return IMU_GENERIC_ERROR;
  else
    return ((regVal & 0x02) >> 1);
}

int16_t LSM6DSO::readRawAccelX(){

  int16_t output;
  status_t errorLevel = readRegisterInt16(&output, OUTX_L_A);
  if (errorLevel != IMU_SUCCESS) {
    if (errorLevel == IMU_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }
  return output;
}

float LSM6DSO::readFloatAccelX(){
  float output = calcAccel(readRawAccelX());
  return output;
}

int16_t LSM6DSO::readRawAccelY(){
  int16_t output;
  status_t errorLevel = readRegisterInt16(&output, OUTY_L_A);
  if (errorLevel != IMU_SUCCESS) {
    if (errorLevel == IMU_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }
  return output;
}

float LSM6DSO::readFloatAccelY(){
  float output = calcAccel(readRawAccelY());
  return output;
}

int16_t LSM6DSO::readRawAccelZ(){
  int16_t output;
  status_t errorLevel = readRegisterInt16(&output, OUTZ_L_A);
  if (errorLevel != IMU_SUCCESS) {
    if (errorLevel == IMU_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }
  return output;
}

float LSM6DSO::readFloatAccelZ(){
  float output = calcAccel(readRawAccelZ());
  return output;
}

float LSM6DSO::calcAccel(int16_t input){
  uint8_t accelRange;
  uint8_t scale;
  float output;

  readRegister(&accelRange, CTRL1_XL);
  scale = (accelRange >> 1) & 0x01;
  accelRange = (accelRange >> 2) & (0x03);

  if (scale == 0) {
    switch (accelRange) {
      case 0: // Register value 0: 2g
        output = (static_cast<float>(input) * (.061)) / 1000;
        break;
      case 1: //Register value 1 : 16g
        output = (static_cast<float>(input) * (.488)) / 1000;
        break;
      case 2: //Register value 2 : 4g
        output = (static_cast<float>(input) * (.122)) / 1000;
        break;
      case 3: //Register value 3: 8g
        output = (static_cast<float>(input) * (.244)) / 1000;
        break;
    }
  }

  if (scale == 1) {
    switch (accelRange) {
      case 0: //Register value 0: 2g
        output = (static_cast<float>(input) * (0.061)) / 1000;
        break;
      case 1: //Register value 1: 2g
        output = (static_cast<float>(input) * (0.061)) / 1000;
        break;
      case 2: //Register value 2: 4g
        output = (static_cast<float>(input) * (.122)) / 1000;
        break;
      case 3: //Register value 3: 8g
        output = (static_cast<float>(input) * (.244)) / 1000;
        break;
    }
  }

  return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//

// Address:CTRL2_G , bit[7:4]: default value is: 0x00.
// Sets the gyro's output data rate thereby enabling it.
bool LSM6DSO::setGyroDataRate(uint16_t rate){

  if ((rate < 125) | (rate > 6660))
    return false;

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL2_G);
  if (returnError != IMU_SUCCESS)
    return false;

  regVal &= ODR_GYRO_MASK;

  switch (rate) {
    case 0:
      regVal |= ODR_GYRO_DISABLE;
      break;
    case 125:
      regVal |= ODR_GYRO_12_5Hz;
      break;
    case 26:
      regVal |= ODR_GYRO_26Hz;
      break;
    case 52:
      regVal |= ODR_GYRO_52Hz;
      break;
    case 104:
      regVal |= ODR_GYRO_104Hz;
      break;
    case 208:
      regVal |= ODR_GYRO_208Hz;
      break;
    case 416:
      regVal |= ODR_GYRO_416Hz;
      break;
    case 833:
      regVal |= ODR_GYRO_833Hz;
      break;
    case 1660:
      regVal |= ODR_GYRO_1660Hz;
      break;
    case 3330:
      regVal |= ODR_GYRO_3330Hz;
      break;
    case 6660:
      regVal |= ODR_GYRO_6660Hz;
      break;
    default:
      break;
  }

  returnError = writeRegister(CTRL2_G, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address:CTRL2_G , bit[7:4]: default value is:0x00
// Gets the gyro's data rate. A data rate of 0, implies that the gyro portion
// of the IMU is disabled.
float LSM6DSO::getGyroDataRate(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL2_G);

  if (returnError != IMU_SUCCESS)
    return static_cast<float>(IMU_GENERIC_ERROR);

  regVal &= ~ODR_GYRO_MASK;

  switch (regVal) {
    case ODR_GYRO_DISABLE:
      return 0.0;
    case ODR_GYRO_12_5Hz:
      return 12.5;
    case ODR_GYRO_26Hz:
      return 26.5;
    case ODR_GYRO_52Hz:
      return 52.0;
    case ODR_GYRO_104Hz:
      return 104.0;
    case ODR_GYRO_208Hz:
      return 208.0;
    case ODR_GYRO_416Hz:
      return 416.0;
    case ODR_GYRO_833Hz:
      return 833.0;
    case ODR_GYRO_1660Hz:
      return 1660.0;
    case ODR_GYRO_3330Hz:
      return 3330.0;
    case ODR_GYRO_6660Hz:
      return 6660.0;
    default:
      return static_cast<float>(IMU_GENERIC_ERROR);
  }

}

// Address: 0x11, bit[3:0]: default value is: 0x00
// Sets the gyroscope's range.
bool LSM6DSO::setGyroRange(uint16_t range){

  if ((range < 250) | (range > 2000))
    return false;

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL2_G);
  if (returnError != IMU_SUCCESS)
    return false;

  regVal &= FS_G_MASK;

  switch (range) {
    case 125:
      regVal |= FS_G_125dps;
      break;
    case 250:
      regVal |= FS_G_250dps;
      break;
    case 500:
      regVal |= FS_G_500dps;
      break;
    case 1000:
      regVal |= FS_G_1000dps;
      break;
    case 2000:
      regVal |= FS_G_2000dps;
      break;
  }

  returnError = writeRegister(CTRL2_G, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address: 0x11, bit[3:0]: default value is: 0x00
// Gets the gyroscope's range.
uint16_t LSM6DSO::getGyroRange(){

  uint8_t regVal;

  status_t returnError = readRegister(&regVal, CTRL2_G);
  if (returnError != IMU_SUCCESS)
    return IMU_GENERIC_ERROR;

  regVal &= ~FS_G_MASK;

  switch (regVal) {
    case FS_G_125dps:
      return 125;
    case FS_G_250dps:
      return 250;
    case FS_G_500dps:
      return 500;
    case FS_G_1000dps:
      return 1000;
    case FS_G_2000dps:
      return 2000;
    default:
      return IMU_GENERIC_ERROR;
  }
}

int16_t LSM6DSO::readRawGyroX(){

  int16_t output;
  status_t errorLevel = readRegisterInt16(&output, OUTX_L_G);

  if (errorLevel != IMU_SUCCESS) {
    if (errorLevel == IMU_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }

  return output;
}

float LSM6DSO::readFloatGyroX(){

  float output = calcGyro(readRawGyroX());
  return output;
}

int16_t LSM6DSO::readRawGyroY(){

  int16_t output;
  status_t errorLevel = readRegisterInt16(&output, OUTY_L_G);

  if (errorLevel != IMU_SUCCESS) {
    if (errorLevel == IMU_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }

  return output;
}

float LSM6DSO::readFloatGyroY(){

  float output = calcGyro(readRawGyroY());
  return output;
}

int16_t LSM6DSO::readRawGyroZ(){

  int16_t output;
  status_t errorLevel = readRegisterInt16(&output, OUTZ_L_G);

  if (errorLevel != IMU_SUCCESS) {
    if (errorLevel == IMU_ALL_ONES_WARNING)
      allOnesCounter++;
    else
      nonSuccessCounter++;
  }

  return output;
}

float LSM6DSO::readFloatGyroZ(){

  float output = calcGyro(readRawGyroZ());
  return output;

}

float LSM6DSO::calcGyro(int16_t input){

  uint8_t gyroRange;
  uint8_t fullScale;
  float output;

  readRegister(&gyroRange, CTRL2_G);
  fullScale = (gyroRange >> 1) & 0x01;
  gyroRange = (gyroRange >> 2) & 0x03;

  if (fullScale)
    output = (static_cast<float>(input) * 4.375) / 1000;
  else {
    switch (gyroRange) {
      case 0:
        output = (static_cast<float>(input) * 8.75) / 1000;
        break;
      case 1:
        output = (static_cast<float>(input) * 17.50) / 1000;
        break;
      case 2:
        output = (static_cast<float>(input) * 35) / 1000;
        break;
      case 3:
        output = (static_cast<float>(input) * 70) / 1000;
        break;
    }
  }

  return output;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LSM6DSO::readRawTemp(){
  int16_t output;
  readRegisterInt16(&output, OUT_TEMP_L);
  return output;
}

float LSM6DSO::readTempC(){
  int16_t temp = (readRawTemp());
  int8_t msbTemp = (temp & 0xFF00) >> 8;
  float tempFloat = static_cast<float>(msbTemp);
  float lsbTemp = temp & 0x00FF;

  lsbTemp /= 256;

  tempFloat += lsbTemp;
  tempFloat += 25; //Add 25 degrees to remove offset

  return tempFloat;

}

float LSM6DSO::readTempF(){
  float output = readTempC();
  output = (output * 9) / 5 + 32;

  return output;

}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//

bool LSM6DSO::getFifoStatus(uint16_t& numberOfSensorDataReady, FifoStatusBits& statusBits){
  uint8_t fifoReg[2] = { 0, 0 };

  if (IMU_SUCCESS != readMultipleRegisters(fifoReg, FIFO_STATUS1, 2)) {
    return false;
  }

  numberOfSensorDataReady = (uint16_t)(((fifoReg[1] & 0x03) << 8) | fifoReg[0]);
  statusBits = FifoStatusBits { fifoReg[1] };

  return true;
}

uint16_t LSM6DSO::fifoRead(const uint16_t numberOfSensorDataReady, FifoData* fifoData){
  //Pull the last data from the fifo
  uint16_t numSamplesRead = 0;
  for (uint16_t i = 0; i < numberOfSensorDataReady; ++i) {
    // Do OS blocking spi transmit receive DMA callback handler will set OS event bit or time out
    uint8_t data[7];
    status_t returnError = readMultipleRegisters(data, FIFO_DATA_OUT_TAG, 7);

    if (returnError != IMU_SUCCESS) {
      return numSamplesRead;
    }

    uint8_t tempTagByte = (data[0] >> 3);

    if ((tempTagByte == ACCELEROMETER_DATA) | (tempTagByte == GYROSCOPE_DATA) && fifoData != nullptr) {
      fifoData[i].tag = tempTagByte;
      fifoData[i].x = (data[2] << 8) | data[1];
      fifoData[i].y = (data[4] << 8) | data[3];
      fifoData[i].z = (data[6] << 8) | data[5];
      ++numSamplesRead;
    }
  }

  return numSamplesRead;
}

// Address: 0x12 , bit[4]: default value is: 0x01
// Sets register iteration when making multiple reads.
bool LSM6DSO::setIncrement(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, CTRL3_C);
  if (returnError != IMU_SUCCESS)
    return false;

  regVal &= 0xFD;
  regVal |= IF_INC_ENABLED;

  returnError = writeRegister(CTRL3_C, regVal);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

bool LSM6DSO::softwareReset(){

  status_t returnError = writeRegister(SW_RESET_DEVICE, CTRL3_C);
  if (returnError != IMU_SUCCESS)
    return false;
  else
    return true;
}

// Address:0x1A , bit[7:0]: default value is: 0x00
// This function clears the given interrupt upon reading it from the register.
uint8_t LSM6DSO::clearAllInt(){

  uint8_t regVal;
  status_t returnError = readRegister(&regVal, ALL_INT_SRC);
  if (returnError != IMU_SUCCESS)
    return returnError;
  else
    return regVal;
}
