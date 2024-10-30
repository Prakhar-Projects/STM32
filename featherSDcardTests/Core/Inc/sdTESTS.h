/*
 * sdTESTS.h
 *
 *  Created on: 14 Apr 2023
 *      Author: Javier
 */

#ifndef INC_SDTESTS_H_
#define INC_SDTESTS_H_

#include "fatfs.h"
#include "bsp_driver_sd.h" // Include BSP library for SD card functions

// Global SD path variable
extern char SDPath[4];
extern FIL myFILE;
// Function to initialize and mount the SD card
FRESULT SD_Setup(void);
FRESULT AppendToFile(char* path, size_t path_len, char* msg, size_t msg_len);
#endif /* INC_SDTESTS_H_ */
