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
        err = plm_sd_init();
        if (err) plm_sd_deinit();
        else sd_ready = 1;
    }

    if (sd_ready) {
        uint8_t data1[3] = {1, 2, 3};
        err = plm_sd_write(data1, 3);
        if (err) {
            // write failed
            plm_sd_deinit();
            sd_ready = 0;
        }
    }

    uint8_t data2[3] = {1, 2, 3};
    plm_xb_send(data2, 3);

    osDelay(1000);
}
