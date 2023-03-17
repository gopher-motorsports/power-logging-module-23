/*
 * sd.c
 *
 *  Created on: Jan 29, 2023
 *      Author: jonathan
 */

#include "main.h"
#include "fatfs.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

uint8_t plm_sd_init(void) {
    // checks that the card is inserted & initializes SD interface
    DSTATUS status = SD_Driver.disk_initialize(0);
    if (status == STA_NOINIT) return 1;

//    FRESULT res;

//    // mount
//    res = f_mount(&SDFatFS, SDPath, 1);
//    if (res != FR_OK) return 1;
//
//    // open a file
//    res = f_open(&SDFile, "data.dat", FA_CREATE_ALWAYS | FA_WRITE);
//    if (res != FR_OK) return 1;

    return 0;
}

void plm_sd_deinit(void) {
//    f_close(&SDFile);
//    f_mount(NULL, SDPath, 0);
}

uint8_t plm_sd_write(uint8_t* buffer, uint16_t size) {
    FRESULT res;

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    // write data
    res = f_write(&SDFile, buffer, size, NULL);
    if (res != FR_OK) return 1;

    // flush cached data
    res = f_sync(&SDFile);
    if (res != FR_OK) return 1;

    return 0;
}
