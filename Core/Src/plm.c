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
#include "main.h"
#include "GopherCAN.h"
#include "usb_device.h"
#include "plm_sd.h"
#include "plm_xb.h"
#include "fatfs.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

extern USBD_HandleTypeDef hUsbDeviceFS;

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
    // if SD is ready for FatFs interaction
    static uint8_t fs_ready = 0;

    // check if device is connected and ready to interact via USB
    uint8_t usb_connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;

    // prevent USB access and FatFs interaction at the same time
    if (usb_connected && fs_ready) {
        plm_sd_deinit();
        fs_ready = 0;
    }

    if (!usb_connected) {
        if (!fs_ready) {
            // init FatFs and open the current data file
            uint8_t err = plm_sd_init("data.dat");
            if (err) plm_sd_deinit();
            else fs_ready = 1;
        }

        if (fs_ready) {
            // write data
            uint8_t data[5] = {1, 2, 3, 4, 5};
            uint8_t err = plm_sd_write(data, 5);
            if (err) {
                // write failed
                plm_sd_deinit();
                fs_ready = 0;
            }
        }
    }

    osDelay(100);
}

void plm_transmit_data(void) {
//    uint8_t msg[3] = {1, 2, 3};
//    plm_xb_send(msg, 3);
    osDelay(5000);
}
