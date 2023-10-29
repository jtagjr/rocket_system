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

class DataWriter {
public:
  DataWriter();
  ~DataWriter();

  void Run();

private:

  void StartWriting();
  void WriteBuffer(void* buffer, uint32_t length);
  void StopWriting();
};

#endif /* SRC_DATAWRITER_H_ */
