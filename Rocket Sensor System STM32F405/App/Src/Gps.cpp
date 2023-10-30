#include <Global.h>
#include <Gps.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <atomic>
#include <type_traits>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "cmsis_os2.h"
#include "main.h"

#include "u-blox_structs.h"
#include "DataSample.h"

/*
 * Reading GPS registers
 * Write GPS Address with write bit set
 * Write Register Address (1 byte)
 * Write GPS Address with read bit set
 * Read X number of bytes
 */

/*
 * Reading messages
 * Write GPS_I2C_ADDRESS with write bit
 * Write REG_ADDRESS_MSB_DATA_READY_LENGTH
 * Write GPS_I2C_ADDRESS with read bit set
 * Clock in MSB byte of length
 * Clock in LSB byte of length
 * Set up for loop
 *    Loop clocking in ((MSB << 8) | LSB)
 *    Parse each byte
 */

/*
 * GPS does not have any registers for MCU to write to so register address isn't needed
 *
 * Writing data
 * Write GPS_I2C_ADDRESS with write bit
 * GPS responds with ACK bit
 * Clock in X number of bytes
 */

extern I2C_HandleTypeDef hi2c2;

constexpr uint8_t GPS_I2C_ADDRESS = 0x42 << 1;
constexpr uint8_t REG_ADDRESS_MSB_DATA_READY_LENGTH = 0xFD;
constexpr uint8_t REG_ADDRESS_LSB_DATA_READY_LENGTH = 0xFE;
constexpr uint8_t REG_ADDRESS_MESSAGE_DATA = 0xFF;
constexpr uint8_t FRAME_1 = 0xB5;
constexpr uint8_t FRAME_2 = 0x62;
constexpr uint8_t expected_payload_length = 52;

enum class ParserState : uint8_t {
  SYNC_WORD_1,
  SYNC_WORD_2,
  CLASS_BYTE,
  ID_BYTE,
  LENGTH_LSB,
  LENGTH_MSB,
  PAYLOAD,
  CKA,
  CKB,
};

static std::atomic<uint32_t> parity_errors = 0;
static std::atomic<uint32_t> frame_errors = 0;
static std::atomic<uint32_t> overrun_errors = 0;
static std::atomic<uint32_t> noise_errors = 0;
static std::atomic<uint32_t> dropped_bytes = 0;

static constexpr uint8_t MAX_BUFFERS { 2 };
static constexpr uint8_t MAX_SAMPLES { 2 };
static NavigationFileEntry gps_sample_buffers[MAX_BUFFERS][MAX_SAMPLES];
NavigationFileEntry* last_sample{0};
static NavigationFileEntry* current_buffer{&gps_sample_buffers[0][0]};
static NavigationFileEntry* next_buffer{&gps_sample_buffers[1][0]};

static uint8_t buffer_idx = 0;
static uint16_t sample_idx = 0;
static uint16_t remaining_message_length = 0;
static ParserState parser_state = ParserState::SYNC_WORD_1;
static uint32_t msg_count = 0;
static uint8_t* tmp_ptr = 0;
static uint8_t ck_a = 0;
static uint8_t ck_b = 0;

void SwitchGpsBuffers() {
  last_sample = &current_buffer[1];
  current_buffer = next_buffer;
  next_buffer = nullptr;
  sample_idx = 0;
  if (current_buffer != nullptr) {
    current_buffer[0].header.sample_length = 0;
    current_buffer[0].header.sample_number = 0;
    current_buffer[1].header.sample_length = 0;
    current_buffer[1].header.sample_number = 0;
  }
}

void GpsBufferSaved(void* buffer) {
  NavigationFileEntry** tmp = current_buffer == nullptr ? &current_buffer : &next_buffer;
  *tmp = static_cast<NavigationFileEntry*>(buffer);
  auto ptr = static_cast<NavigationFileEntry*>(buffer);
  if (&(ptr)[1] != last_sample) {
    ptr[0].header.sample_length = 0;
    ptr[0].header.sample_number = 0;
    ptr[1].header.sample_length = 0;
    ptr[1].header.sample_number = 0;
  }
}

uint32_t GpsMsgCount(){
  return msg_count;
}

uint32_t GpsDroppedBytes(){
  return dropped_bytes.load();
}

uint32_t GpsFrameErrors() {
  return frame_errors.load();
}

uint32_t GpsOverrunErrors() {
  return overrun_errors.load();
}

uint32_t GpsNoiseErrors() {
  return noise_errors.load();
}

const NavigationFileEntry* GpsSample() {
  return last_sample;
}

extern UART_HandleTypeDef huart3;

inline
static bool length_correct(uint8_t classByte, uint8_t idByte, uint16_t length){
  return length == expected_payload_length;
}

inline
static bool has_class_and_id(uint8_t classByte, uint8_t idByte){
  return classByte == UBX_CLASS_NAV && idByte == UBX_NAV_SOL;
}

inline
static void syncword_1(uint8_t data){
  if (FRAME_1 == data) {
    parser_state = ParserState::SYNC_WORD_2;
  }
  else {
    ++dropped_bytes;
  }
}

inline
static void syncword_2(uint8_t data){
  if (FRAME_2 == data) {
    parser_state = ParserState::CLASS_BYTE;
    current_buffer[sample_idx].header.sample_type = static_cast<uint8_t>(SampleType::GPS_SAMPLE);
    current_buffer[sample_idx].header.sample_length = sizeof(NavigationFileEntry) - sizeof(SamplesHeader::sample_type);
    current_buffer[sample_idx].header.header_crc = 0;
    current_buffer[sample_idx].header.sample_time = GetTimeInNanoSeconds();
    current_buffer[sample_idx].header.sample_number = msg_count;
  }
  else {
    parser_state = ParserState::SYNC_WORD_1;
    dropped_bytes += 2;
  }
}

inline
static void class_byte(uint8_t data){
  parser_state = ParserState::ID_BYTE;
  current_buffer[sample_idx].sample.class_type = data;
  ck_a = data;
  ck_b = ck_a;
}

inline
static void id_byte(uint8_t data){
  parser_state = ParserState::LENGTH_LSB;
  current_buffer[sample_idx].sample.id_type = data;
  ck_a += data;
  ck_b += ck_a;

  if (!has_class_and_id(current_buffer[sample_idx].sample.class_type,
                        current_buffer[sample_idx].sample.id_type)) {
    parser_state = ParserState::SYNC_WORD_1;
    dropped_bytes += 4;
  }
}

inline
static void length_lsb(uint8_t data){
  current_buffer[sample_idx].sample.payload_length = data;
  parser_state = ParserState::LENGTH_MSB;
  ck_a += data;
  ck_b += ck_a;
}

inline
static void length_msb(uint8_t data){
  current_buffer[sample_idx].sample.payload_length |= (data << 8);
  remaining_message_length = current_buffer[sample_idx].sample.payload_length; // add CK bytes
  tmp_ptr = (uint8_t*)(&(current_buffer[sample_idx].sample.iTOW));
  parser_state = ParserState::PAYLOAD;
  ck_a += data;
  ck_b += ck_a;

  if (!length_correct(current_buffer[sample_idx].sample.class_type,
                      current_buffer[sample_idx].sample.id_type, remaining_message_length)) {
    parser_state = ParserState::SYNC_WORD_1;
    dropped_bytes += 6;
  }
}

inline
static void payload(uint8_t data){
  *tmp_ptr = data;
  ++tmp_ptr;
  ck_a += data;
  ck_b += ck_a;
  --remaining_message_length;

  if (0 == remaining_message_length) {
    parser_state = ParserState::CKA;
  }
}

inline
static void cka(uint8_t data){
  *tmp_ptr = data;
  ++tmp_ptr;
  parser_state = ParserState::CKB;

  if (ck_a != data) {
    parser_state = ParserState::SYNC_WORD_1;
    dropped_bytes += 7 + current_buffer[sample_idx].sample.payload_length;
  }
}

inline
static void ckb(uint8_t data){
  *tmp_ptr = data;
  ++tmp_ptr;

  if (ck_b == data) {
    ++sample_idx;
    ++msg_count;
    if (2 == sample_idx) {
      SaveBuffer(3, sizeof(NavigationFileEntry)*2, &current_buffer[0]);
      SwitchGpsBuffers();
    }
  }
  else {
    dropped_bytes += 8 + current_buffer[sample_idx].sample.payload_length;
  }

  parser_state = ParserState::SYNC_WORD_1;
}

static void calculate_bytes_dropped(){
  switch (parser_state) {
    case ParserState::PAYLOAD:
      dropped_bytes += 6 + current_buffer[sample_idx].sample.payload_length - remaining_message_length;
      break;

    case ParserState::SYNC_WORD_1:
      ++dropped_bytes;
      break;

    case ParserState::SYNC_WORD_2:
      dropped_bytes += 2;
      break;

    case ParserState::CLASS_BYTE:
      dropped_bytes += 3;
      break;

    case ParserState::ID_BYTE:
      dropped_bytes += 4;
      break;

    case ParserState::LENGTH_LSB:
      dropped_bytes += 5;
      break;

    case ParserState::LENGTH_MSB:
      dropped_bytes += 6;
      break;

    case ParserState::CKA:
      dropped_bytes += 7 + current_buffer[sample_idx].sample.payload_length;
      break;

    case ParserState::CKB:
      dropped_bytes += 8 + current_buffer[sample_idx].sample.payload_length;
      break;

    default:
      ++dropped_bytes;
      break;
  }
}

extern "C" {

void HandleGpsReceiveISR(){
  TP2_On();
  uint32_t isrflags = READ_REG(huart3.Instance->SR);
  uint32_t ddr = READ_REG(huart3.Instance->DR);

  if (isrflags & (uint32_t)(USART_SR_PE)) {
    ++parity_errors;
  }

  if (isrflags & (uint32_t)(USART_SR_FE)) {
    ++frame_errors;
  }

  if (isrflags & (uint32_t)(USART_SR_ORE)) {
    ++overrun_errors;
    parser_state = ParserState::SYNC_WORD_1;
    calculate_bytes_dropped();
    return;
  }

  if (isrflags & (uint32_t)(USART_SR_NE)) {
    ++noise_errors;
  }

  if (current_buffer == nullptr) {
    ++dropped_bytes;
    return;
  }

  uint8_t data = (uint8_t)(ddr & 0x000000FF);
  switch (parser_state) {
  case ParserState::PAYLOAD:
    payload(data);
    break;

  case ParserState::SYNC_WORD_1:
    syncword_1(data);
    break;

  case ParserState::SYNC_WORD_2:
    syncword_2(data);
    break;

  case ParserState::CLASS_BYTE:
    class_byte(data);
    break;

  case ParserState::ID_BYTE:
    id_byte(data);
    break;

  case ParserState::LENGTH_LSB:
    length_lsb(data);
    break;

  case ParserState::LENGTH_MSB:
    length_msb(data);
    break;

  case ParserState::CKA:
    cka(data);
    break;

  case ParserState::CKB:
    ckb(data);
    break;

  default:
    ++dropped_bytes;
    break;
  }
  TP2_Off();
}

}

static uint16_t NMEA_Checksum(const char* packet, uint16_t length);
void DisplayNavigationData(NavigationFileEntry& data);
static void AppendCalculateChecksum(uint8_t* packet, uint16_t length);

static void FillInRequestHeader(uint8_t* data, uint8_t msg_class, uint8_t msg_id){
  data[0] = FRAME_1;
  data[1] = FRAME_2;
  data[2] = msg_class;
  data[3] = msg_id;
  data[4] = 0;
  data[5] = 0;
}

void EnableGPS(){
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);
  memset(gps_sample_buffers, 0, sizeof(gps_sample_buffers));
}

void DisableGPS(){
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_RESET);
  memset(gps_sample_buffers, 0, sizeof(gps_sample_buffers));
}

void ConfigureGpsTX(){ //115200
  char base[] = "PUBX,41,1,0003,0001,115200,0"; // Output only UBX on GPS TX pin
  auto checksum = NMEA_Checksum(base, sizeof(base) - 1);
  char config[50];
  sprintf(config, "$%s*%x\r\n", base, checksum);
  HAL_I2C_Master_Transmit(&hi2c2, GPS_I2C_ADDRESS, (uint8_t*)config, strlen(config), 1000);
}

void SetNavigationMeasurementRate(){
  // Rate is in navigation rate cycles
  // The default navigation rate cycle is 1Hz
  // If you want a faster rate, set the navigation measurement rate
  uint8_t data[] = { 0xB5, 0x62, UBX_CLASS_CFG, UBX_CFG_RATE, 6, 0, 50, 0, 1, 0, 0, 0, // Rate is in ms so here it's 50, 1 is required, last to zeros are for alignment to UTC
      0, 0 };
  AppendCalculateChecksum(data, sizeof(data));
  HAL_I2C_Master_Transmit(&hi2c2, GPS_I2C_ADDRESS, data, sizeof(data), 1000);
}

void SetNavigationSolutionMessageRate(){
  // Rate is in navigation rate cycles
  // The default navigation rate cycle is 1Hz
  // If you want a faster rate, set the navigation measurement rate
  uint8_t data[] = { 0xB5, 0x62, UBX_CLASS_CFG, UBX_CFG_MSG, 8, 0, UBX_CLASS_NAV, UBX_NAV_SOL, 0, 1, 0, 0, 0, 0, // logical port UART TX is port 1 so second (zero based) index
      0, 0 };
  AppendCalculateChecksum(data, sizeof(data));
  HAL_I2C_Master_Transmit(&hi2c2, GPS_I2C_ADDRESS, data, sizeof(data), 1000);
}

void ConfigureGpsI2C(){
  char base[] = "PUBX,41,0,0003,0001,400000,0"; // Output only UBX
  auto checksum = NMEA_Checksum(base, 28);
  char config[50];
  sprintf(config, "$%s*%x\r\n", base, checksum);
  HAL_I2C_Master_Transmit(&hi2c2, GPS_I2C_ADDRESS, (uint8_t*)config, strlen(config), 1000);
}

void GetUartConfiguration(){
  char base[] = "PUBX,41"; // Output only UBX
  auto checksum = NMEA_Checksum(base, 7);
  char config[50];
  sprintf(config, "$%s*%x\r\n", base, checksum);
  HAL_I2C_Master_Transmit(&hi2c2, GPS_I2C_ADDRESS, (uint8_t*)config, strlen(config), 1000);
}

void StartNavCollection(){
  HAL_UART_StateTypeDef uartState = HAL_UART_STATE_RESET;

  // check for Spi to be ready
  while (uartState != HAL_UART_STATE_READY) {
    uartState = HAL_UART_GetState(&huart3);
    osDelay(1);
  }

  parser_state = ParserState::SYNC_WORD_1;
  sample_idx = 0;
  buffer_idx = 0;
  msg_count = 0;
  remaining_message_length = 0;
  ck_a = 0;
  ck_b = 0;
  tmp_ptr = 0;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

void StopNavCollection(){
  __HAL_UART_DISABLE_IT(&huart3, UART_IT_ERR);

  __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
}

void DisplayNavigationData(NavigationFileEntry& entry){
  NavigationSolution& data { entry.sample };

  PRINTLN("\nGPS DATA START");
  PRINTLN("GPS sample_type %u", entry.header.sample_type);
  PRINTLN("GPS sample_length %u", entry.header.sample_length);
  PRINTLN("GPS sample_crc %u", entry.header.header_crc);
  PRINTLN("GPS sample_tick %lu", entry.header.sample_time);
  PRINTLN("GPS sample_number %lu", entry.header.sample_number);
  PRINTLN("GPS msg class_type %u", data.class_type);
  PRINTLN("GPS msg id_type %u", data.id_type);
  PRINTLN("GPS payload_length %u", data.payload_length);
  PRINTLN("GPS Millisecond Time of Week %lu", data.iTOW);
  PRINTLN("Nanosecond Remainder %li", data.fTOW);
  PRINTLN("Week %i", data.week);
  PRINTLN("GPS Fix 0x%01X", data.gpsFix);
  PRINTLN("Flags 0x%01X", (data.flags & 0x0F)); // mask off reserved bits
  PRINTLN("ECEF X %li cm", data.ecefX);
  PRINTLN("ECEF Y %li cm", data.ecefY);
  PRINTLN("ECEF Z %li cm", data.ecefZ);
  PRINTLN("3D position accuracy estimate %lu cm", data.pAcc);
  PRINTLN("ECEV X %li cm/s", data.eceVX);
  PRINTLN("ECEV Y %li cm/s", data.eceVY);
  PRINTLN("ECEV Z %li cm/s", data.eceVZ);
  PRINTLN("3D speed accuracy estimate %lu  cm/s", data.sAcc);
  PRINTLN("pDOP %u", data.pDOP);
  PRINTLN("res1 %u", data.res1);
  PRINTLN("numSV %u", data.numSV);
  PRINTLN("res2 %lu", data.res2);
  PRINTLN("ck_a %u", data.ck_a);
  PRINTLN("ck_b %u", data.ck_b);
  PRINTLN("Number of SVs used %u", data.numSV);
  PRINTLN("Message Count %lu", msg_count);
  PRINTLN("Dropped Bytes %lu", dropped_bytes.load());
  PRINTLN("Frame Errors %lu", frame_errors.load());
  PRINTLN("Overrun Errors %lu", overrun_errors.load());
  PRINTLN("Noise Errors %lu", noise_errors.load());
  PRINTLN("GPS DATA END");
}

void ReadFirmwareInformation(){
  uint8_t data[8];
  FillInRequestHeader(data, UBX_CLASS_MON, UBX_MON_VER);
  AppendCalculateChecksum(data, 8);
  auto result = HAL_I2C_Master_Transmit(&hi2c2, GPS_I2C_ADDRESS, data, 8, 1000);
  auto buffer = (uint8_t*)&(gps_sample_buffers[0][0]);

  if (HAL_OK == result) {
    while (true) {
      uint8_t incomingHeader[3] = { 0 };
      auto result = HAL_I2C_Mem_Read(&hi2c2, GPS_I2C_ADDRESS, REG_ADDRESS_MSB_DATA_READY_LENGTH, 1, incomingHeader, 3, 1000);

      if (HAL_OK == result) {
        if (incomingHeader[2] != 0xFF) {
          uint16_t incomingLength = (incomingHeader[0] << 8) | incomingHeader[1];
          if (incomingLength > 0) {
            auto result = HAL_I2C_Master_Receive(&hi2c2, GPS_I2C_ADDRESS, buffer, incomingLength, 1000);
            if (HAL_OK == result) {
              auto receivedLength = incomingLength;
              if (receivedLength >= 44) {
                if (buffer[0] == UBX_CLASS_MON && buffer[1] == UBX_MON_VER) {
                  uint16_t payloadLength = (buffer[2] | (buffer[3] << 8));
                  PRINTLN((char* )&buffer[4]);
                  PRINTLN((char* )&buffer[34]);
                  uint16_t extLength = payloadLength - 40;
                  char* tmp = (char*)&buffer[44];
                  uint16_t index = 0;
                  while (index < extLength) {
                    PRINTLN(&tmp[index]);
                    index += 30;
                  }
                }
                else {
                  PRINTLN("Received unexpected message 0x%x:0x%x total msg length=%u", buffer[0], buffer[1], incomingLength);
                  buffer[incomingLength] = 0;
                  for (int i = 2; i < incomingLength; ++i) {
                    if (buffer[i] != 0) {
                      PRINT("%c", (char )buffer[i]);
                    }
                  }

                }
              }
              else {
                PRINTLN("Received message 0x%x:0x%x length=%u", buffer[0], buffer[1], buffer[2]);
              }
              break;
            }
          }
          else {
            osDelay(20);
            continue;

          }
        }
      }
      else {
        PRINTLN("GPS I2C error 2!!!");
        break;
      }
    }
  }
  else {
    PRINTLN("GPS I2C error 1!!!");
  }
}

static void AppendCalculateChecksum(uint8_t* packet, uint16_t length){
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  ck_a += packet[2];
  ck_b += ck_a;

  ck_a += packet[3];
  ck_b += ck_a;

  uint16_t dataLength = length - 8;

  ck_a += (dataLength & 0xFF);
  ck_b += ck_a;

  ck_a += (dataLength >> 8);
  ck_b += ck_a;

  for (uint16_t i = 6; i < length - 2; i++) {
    ck_a += packet[i];
    ck_b += ck_a;
  }

  packet[length - 2] = ck_a;
  packet[length - 1] = ck_b;
}

static uint16_t NMEA_Checksum(const char* packet, uint16_t length){
  uint16_t checksum = 0;

  for (int i = 0; i < length; i++) {
    checksum ^= packet[i];
  }

  return checksum;
}

