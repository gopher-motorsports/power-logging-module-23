/*
 * plm.c
 *
 *  Created on: Jan 13, 2023
 *      Author: jonathan
 */

#include <stdint.h>
#include <stdio.h>
#include "plm.h"
#include "cmsis_os.h"
#include "plm_sd.h"
#include "main.h"
#include "GopherCAN.h"
#include "plm_xb.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

extern SD_HandleTypeDef hsd1;

void plm_init(void) {
//    S8 err = init_can(GCAN0, &hcan1, PLM_ID, BXTYPE_MASTER);
//    err &= init_can(GCAN1, &hcan2, PLM_ID, BXTYPE_MASTER);
//    err &= init_can(GCAN2, &hcan3, PLM_ID, BXTYPE_MASTER);
}

void plm_service_can(void) {
//    service_can_tx(&hcan1);
//    service_can_tx(&hcan2);
//    service_can_tx(&hcan3);
//    service_can_rx_buffer();
    osDelay(1);
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

//    if (sd_ready) {
//        uint8_t data1[3] = {1, 2, 3};
//        err = plm_sd_write(data1, 3);
//        if (err) {
//            // write failed
//            plm_sd_deinit();
//            sd_ready = 0;
//        }
//    }

    osDelay(1000);
}

void plm_transmit_data(void) {
    uint8_t msg[3] = {1, 2, 3};
    plm_xb_send(msg, 3);
    osDelay(5000);
}
