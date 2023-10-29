/*
 * FileSystem.h
 *
 *  Created on: Sep 17, 2023
 *      Author: johnt
 */

#ifndef INC_FILESYSTEM_H_
#define INC_FILESYSTEM_H_

#include "DataSample.h"

int mount_file_system();
void read_config(AccConfig& config);
void read_config(SystemConfig& config);

#endif /* INC_FILESYSTEM_H_ */
