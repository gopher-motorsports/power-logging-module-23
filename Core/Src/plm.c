/*
 * plm.c
 *
 *  Created on: Jan 13, 2023
 *      Author: jonathan
 */

#include <stdint.h>
#include "plm.h"
#include "cmsis_os.h"
#include "plm_sd.h"
#include "main.h"

void plm_init(void) {
}

void plm_store_data(void) {
    static uint8_t sd_ready = 0;
    uint8_t err;

    if (!sd_ready) {
        // initialize SD card
        err = sd_init();
        if (err) sd_deinit();
        else sd_ready = 1;
    }

    if (sd_ready) {
        uint8_t data[3] = {1, 2, 3};
        err = sd_write(data, 3);
        if (err) {
            // write failed
            sd_deinit();
            sd_ready = 0;
        }
    }

    osDelay(1000);
}
