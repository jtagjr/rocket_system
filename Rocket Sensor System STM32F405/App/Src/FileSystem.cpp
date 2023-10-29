/*
 * file_functions.cpp
 *
 *  Created on: Sep 17, 2023
 *      Author: johnt
 */

#include <Global.h>
#include "FileSystem.h"
#include <stdio.h>
#include <stdlib.h>
#include "fatfs.h"
#include <sstream>
#include <string>
#include <fstream>
#include <memory.h>

FATFS fs;

int mount_file_system(){
  memset(&SDFatFS, 0, sizeof(fs));
  return f_mount(&fs, SDPath, 1);
}

void read_acc(FIL* file, uint16_t& g, uint16_t& hz){
  // Skip accelerometer name
  char buffer[32] = { 0 };
  auto line = f_gets(buffer, 32, file);
  buffer[2] = 0;
  PRINT(buffer);

  line = f_gets(buffer, 32, file);
  g = atoi(line);

  line = f_gets(buffer, 32, file);
  hz = atoi(line);

  PRINTLN(" %ug rate %uhz", g, hz);
}

void write_acc(const SystemConfig& config){
  FIL file;
  FRESULT result = f_open(&file, "config.txt", FA_WRITE | FA_CREATE_ALWAYS | FA_OPEN_EXISTING);
  if (FR_OK == result) {
    std::stringstream stream;
    stream << "A1\n" << config.acc_one.g << "g\n" << config.acc_one.hz << "hz\nA2\n" << config.acc_two.g << "g\n" << config.acc_two.hz
        << "hz\n";
    auto str = stream.str();
    UINT bytesWritten = 0;
    result = f_write(&file, str.c_str(), str.length(), &bytesWritten);
    if (FR_OK != result) {
      PRINTLN("Failed to write config.txt");
    }
    else {
      result = f_close(&file);
      if (FR_OK != result) {
        PRINTLN("Failed to close config.txt after write attempt");
      }
    }
  }
}

void read_config(SystemConfig& config){
  FIL flightConfigFile;
  int result = f_open(&flightConfigFile, "config.txt", FA_READ | FA_OPEN_EXISTING);
  if (FR_OK == result) {
    //PRINTLN("Config file found");
    read_acc(&flightConfigFile, config.acc_one.g, config.acc_one.hz);
    read_acc(&flightConfigFile, config.acc_two.g, config.acc_two.hz);
    f_close(&flightConfigFile);
  }
  else {
    PRINTLN("Config file not found, setting defaults");
    config.acc_one.g = 4;
    config.acc_one.hz = 1000;
    config.acc_two.g = 32;
    config.acc_two.hz = 1000;
    write_acc(config);
  }
}
