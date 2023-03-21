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
#include "fatfs.h"
#include "plm_error.h"
#include "plm_sd.h"
#include "plm_xb.h"
#include "plm_sim.h"
#include "plm_data.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

extern USBD_HandleTypeDef hUsbDeviceFS;

void plm_init(void) {
    plm_err_reset();

    S8 err = init_can(GCAN0, &hcan1, PLM_ID, BXTYPE_MASTER);
    err |= init_can(GCAN1, &hcan2, PLM_ID, BXTYPE_MASTER);
    err |= init_can(GCAN2, &hcan3, PLM_ID, BXTYPE_MASTER);
    if (err) plm_err_set(PLM_ERR_INIT);
}

void plm_heartbeat(void) {
    // heartbeat blink
    static uint32_t last_blink = 0;
    uint32_t tick = osKernelSysTick();
    if (tick - last_blink >= PLM_DELAY_HEARTBEAT_BLINK) {
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        last_blink = tick;
    }

    // error blink
    plm_err_blink();

    osDelay(PLM_DELAY_HEARTBEAT);
}

void plm_service_can(void) {
    service_can_tx(&hcan1);
    service_can_tx(&hcan2);
    service_can_tx(&hcan3);
    service_can_rx_buffer();

    osDelay(PLM_DELAY_CAN);
}

void plm_collect_data(void) {
    static uint32_t last_log[NUM_OF_PARAMETERS] = {0};

    // check all parameters
    for (uint8_t i = 1; i < NUM_OF_PARAMETERS; i++) {
        CAN_INFO_STRUCT* param = (CAN_INFO_STRUCT*)(PARAMETERS[i]);

        if (param->last_rx > last_log[i]) {
            // parameter has been updated
            PLM_RES res = plm_data_record_param(param->ID);
            if (res != PLM_OK) plm_err_set(res);

            last_log[i] = param->last_rx;
        }
    }

    osDelay(PLM_DELAY_DATA);
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
            PLM_RES res = plm_sd_init("data.dat");
            if (res != PLM_OK) {
                plm_sd_deinit();
                plm_err_set(res);
            }
            else fs_ready = 1;
        }

        if (fs_ready) {
            // write data
            uint8_t data[5] = {1, 2, 3, 4, 5};
            PLM_RES res = plm_sd_write(data, 5);
            if (res != PLM_OK) {
                // write failed
                fs_ready = 0;
                plm_sd_deinit();
                plm_err_set(res);
            }
        }
    }

    osDelay(PLM_DELAY_SD);
}

void plm_transmit_data(void) {
    uint8_t msg[3] = {1, 2, 3};
    PLM_RES res = plm_xb_send(msg, 3);
    if (res != PLM_OK) plm_err_set(res);

    osDelay(PLM_DELAY_XB);
}

void plm_simulate_data(void) {
#ifndef PLM_SIMULATE_DATA
    osThreadId thread_id = osThreadGetId();
    osStatus status = osThreadTerminate(thread_id);
    if (status != osOK) plm_err_set(PLM_ERR_SIM);
#endif
    PLM_RES res = plm_sim_generate_data();
    if (res != PLM_OK) plm_err_set(PLM_ERR_SIM);

    osDelay(PLM_DELAY_SIM);
}
