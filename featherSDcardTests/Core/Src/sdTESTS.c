/*
 * sdTESTS.c
 *
 *  Created on: 14 Apr 2023
 *      Author: Javier
 */

#include "sdTESTS.h"

// Global variables for the filesystem and SD path
FATFS fs;

// Function to initialize and mount the SD card
FRESULT SD_Setup() {
  // Re-initialize SD card
  if (BSP_SD_Init() != MSD_OK) {
    return FR_NOT_READY;
  }

  // Re-link FATFS driver
  if (FATFS_UnLinkDriver(SDPath) != 0) {
    return FR_NOT_READY;
  }
  if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0) {
    return FR_NOT_READY;
  }

  // Mount filesystem
  FRESULT stat = f_mount(&fs, SDPath, 1);
  if (stat != FR_OK) {
    f_mount(0, SDPath, 0); // Unmount if failed
  }
  return stat;
}

// Function to append string to file at given path
FRESULT AppendToFile(char* path, size_t path_len, char* msg, size_t msg_len) {
  FIL myFILE;
  UINT testByte;
  FRESULT stat;

  // Bounds check on strings
  if ((path[path_len] != 0) || (msg[msg_len] != 0)) {
    return FR_INVALID_NAME;
  }

  // Open file for appending
  stat = f_open(&myFILE, path, FA_WRITE | FA_OPEN_APPEND);
  if (stat != FR_OK) {
    return stat;
  }

  // Write message to the end of the file
  stat = f_write(&myFILE, msg, msg_len, &testByte);
  if (stat != FR_OK) {
    f_close(&myFILE); // Close file on error
    return stat;
  }

  // Close file after writing
  stat = f_close(&myFILE);
  return stat;
}
