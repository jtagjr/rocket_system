/*
 * DataWriter.h
 *
 *  Created on: Oct 30, 2021
 *      Author: johnt
 */

#ifndef SRC_DATAWRITER_H_
#define SRC_DATAWRITER_H_

#include <stdint.h>
#include "cmsis_os2.h"

#include "DataSample.h"

class DataWriter {
public:
  DataWriter();
  ~DataWriter();

  void Run();

private:

  void StartWriting();
  void SaveSensorData(const Message& msg);
  void WriteBuffer(void* buffer, uint32_t length);
  void StopWriting();

  bool is_writing{false};
};

#endif /* SRC_DATAWRITER_H_ */
