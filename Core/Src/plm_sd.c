/*
 * sd.c
 *
 *  Created on: Jan 29, 2023
 *      Author: jonathan
 */

#include "main.h"
#include "fatfs.h"

uint8_t plm_sd_init(void) {
    FRESULT res;

    // check if the SD card is inserted (0)
    uint8_t sd_in = !HAL_GPIO_ReadPin(SDMMC1_CD_GPIO_Port, SDMMC1_CD_Pin);
    if (!sd_in) return 1;

    // mount
    res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK) return 1;

    // open a file
    res = f_open(&SDFile, "data.dat", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return 1;

    return 0;
}

void plm_sd_deinit(void) {
    f_close(&SDFile);
    f_mount(NULL, SDPath, 0);
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
