/*
 * plm_sd.c
 *
 *  Created on: Jan 29, 2023
 *      Author: jonathan
 */

#include "plm_sd.h"
#include "main.h"
#include "fatfs.h"
#include <stdio.h>

#define ZERO_YEAR 1970

extern RTC_HandleTypeDef hrtc;

PLM_RES plm_sd_init(void) {
    // checks that the card is inserted & initializes SD interface
    DSTATUS status = SD_Driver.disk_initialize(0);
    if (status == STA_NOINIT) return PLM_ERR_SD_INIT;

    // empty SD driver message queue
    // this is necessary to get a clean state after disconnecting USB
    SD_ResetMsgQueue();

    // register file system
    FRESULT res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK) return PLM_ERR_SD_INIT;

    // generate filename
    char filename[] = "PLM_YYYY-MM-DD-hh-mm-ss.gdat";
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
    sprintf(filename, "PLM_%04u-%02u-%02u-%02u-%02u-%02u.gdat", date.Year + ZERO_YEAR, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);

    // open data file
    // file is created if it doesn't exist
    res = f_open(&SDFile, filename, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK) return PLM_ERR_SD_INIT;

    // add the metadata to the front of the message (just the filename again)
    res = f_printf(&SDFile, "%s:\n", filename);
    if (res != FR_OK) return PLM_ERR_SD_INIT;

    return PLM_OK;
}

void plm_sd_deinit(void) {
    // close file and unregister file system
    f_close(&SDFile);
    f_mount(NULL, SDPath, 0);
}

PLM_RES plm_sd_write(uint8_t* buffer, uint16_t size) {
    unsigned int bytes_written = 0;
    FRESULT res = f_write(&SDFile, buffer, size, &bytes_written);
    if (res != FR_OK || bytes_written != size) return PLM_ERR_SD_WRITE;

    // flush cached data
    res = f_sync(&SDFile);
    if (res != FR_OK) return PLM_ERR_SD_WRITE;

    return PLM_OK;
}
