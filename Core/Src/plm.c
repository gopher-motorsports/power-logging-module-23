/*
 * plm.c
 *
 *  Created on: Jan 13, 2023
 *      Author: jonathan
 */

// TODO needs metadata
// TODO needs new checksum scheme

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "plm.h"
#include "cmsis_os.h"
#include "main.h"
#include "GopherCAN.h"
#include "gopher_sense.h"
#include "usb_device.h"
#include "fatfs.h"
#include "plm_error.h"
#include "plm_sd.h"
#include "plm_xb.h"
#include "plm_sim.h"
#include "plm_data.h"
#include "plm_power.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

extern USBD_HandleTypeDef hUsbDeviceFS;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim10;

extern PLM_DBL_BUFFER SD_DB;
extern PLM_DBL_BUFFER XB_DB;

void plm_init(void) {
    plm_err_reset();

    S8 err = init_can(GCAN0, &hcan1, PLM_ID, BXTYPE_MASTER);
    err |= init_can(GCAN1, &hcan2, PLM_ID, BXTYPE_MASTER);
    err |= init_can(GCAN2, &hcan3, PLM_ID, BXTYPE_MASTER);
    if (err) {
        // PLM shouldn't run without CAN
        plm_err_set(PLM_ERR_INIT);
        HAL_Delay(PLM_DELAY_RESTART);
        NVIC_SystemReset();
    }

    gsense_init(&hcan1, &hadc1, NULL, &hadc3, &htim10, LED_USB_GPIO_Port, LED_USB_Pin);

    // enable all power channel switches
    for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
        PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
        HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
    }

    printf("PLM successfully initialized\n");
}

void plm_heartbeat(void) {
    // heartbeat blink
    static uint32_t last_blink = 0;
    uint32_t tick = osKernelSysTick();
    if (tick - last_blink >= PLM_DELAY_HEARTBEAT_BLINK) {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        printf("PLM ~ tick: %lu\n", tick);
        last_blink = tick;
    }

    // blink any active errors
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
    static uint32_t sd_last_log[NUM_OF_PARAMETERS] = {0};
    static uint32_t xb_last_send[NUM_OF_PARAMETERS] = {0};

    // swap buffers after transfers are complete
    // critical section entry/exit is fast and fine for a quick swap
    if (SD_DB.tx_cplt) {
        taskENTER_CRITICAL();
        SD_DB.write_index = !SD_DB.write_index;
        SD_DB.buffers[SD_DB.write_index]->fill = 0;
        SD_DB.tx_cplt = 0;
        taskEXIT_CRITICAL();
    }

    if (XB_DB.tx_cplt) {
        taskENTER_CRITICAL();
        XB_DB.write_index = !XB_DB.write_index;
        XB_DB.buffers[XB_DB.write_index]->fill = 0;
        XB_DB.tx_cplt = 0;
        taskEXIT_CRITICAL();
    }

    // move gcan parameters to buffers only while USB is not connected
    uint8_t usb_connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
    if (!usb_connected) {
        for (uint8_t i = 1; i < NUM_OF_PARAMETERS; i++) {
            CAN_INFO_STRUCT* param = (CAN_INFO_STRUCT*)(PARAMETERS[i]);
            uint32_t tick = osKernelSysTick();

            if (param->last_rx > sd_last_log[i]) {
                // parameter has been updated
                PLM_RES res = plm_data_record_param(SD_DB.buffers[SD_DB.write_index], param);
                if (res != PLM_OK) plm_err_set(res);

                sd_last_log[i] = tick;
            }

            if (param->last_rx > xb_last_send[i] && tick - xb_last_send[i] > PLM_XB_TX_DELAY) {
                // parameter has been updated and hasn't been sent in a while
                PLM_RES res = plm_data_record_param(XB_DB.buffers[XB_DB.write_index], param);
                if (res != PLM_OK) plm_err_set(res);

                xb_last_send[i] = tick;
            }
        }
    }

    //included turning on/off cooling channels with collect data task to conserve memory
    plm_check_cooling_channels();

    osDelay(PLM_DELAY_DATA);
}

void plm_store_data(void) {
    // if SD is ready for FatFs interaction
    static uint8_t fs_ready = 0;

    // check if device is connected and ready to interact via USB
    uint8_t usb_connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;

    // prevent USB access and FatFs interaction at the same time
    // USB callbacks are in USB_DEVICE/App/usbd_storage_if.c
    // uses the FatFs driver in FATFS/Target/sd_diskio.c
    // WARNING: if USB is connected for too long while logging, buffer will eventually fill
    if (usb_connected && fs_ready) {
        plm_sd_deinit();
        fs_ready = 0;
    }

    if (!usb_connected) {
        if (!fs_ready) {
            // init FatFs and open the current data file
            PLM_RES res = plm_sd_init();
            if (res != PLM_OK) {
                plm_sd_deinit();
                plm_err_set(res);
            }
            else fs_ready = 1;
        }

        if (fs_ready) {
            // write data
            if (!SD_DB.tx_cplt) {
                PLM_BUFFER* buffer = SD_DB.buffers[!SD_DB.write_index];
                if (buffer->fill > 0) {
                    PLM_RES res = plm_sd_write(buffer->bytes, buffer->fill);
                    if (res != PLM_OK) {
                        // write failed
                        fs_ready = 0;
                        plm_sd_deinit();
                        plm_err_set(res);
                    } else {
                        // successful write
                        SD_DB.tx_cplt = 1;
                        HAL_GPIO_TogglePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin);
                    }
                } else SD_DB.tx_cplt = 1;
            }
        }
    }

    osDelay(PLM_DELAY_SD);
}

void plm_transmit_data(void) {
    if (!XB_DB.tx_cplt) {
        PLM_BUFFER* buffer = XB_DB.buffers[!XB_DB.write_index];
        if (buffer->fill > 0) {
            PLM_RES res = plm_xb_send(buffer->bytes, buffer->fill);
            if (res != PLM_OK) plm_err_set(res);
        } else XB_DB.tx_cplt = 1;
    }

    osDelay(PLM_DELAY_XB);
}

void plm_simulate_data(void) {
#ifndef PLM_SIMULATE_DATA
    osThreadTerminate(osThreadGetId());
#endif
    PLM_RES res = plm_sim_generate_data();
    if (res != PLM_OK) plm_err_set(PLM_ERR_SIM);

    osDelay(PLM_DELAY_SIM);
}

void plm_monitor_current(void) {
    for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
        PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
        plm_power_update_channel(channel);

        if (channel->ampsec_sum > channel->ampsec_max && channel->enabled) {
            // channel has reached Amp*sec threshold, open switch
            HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_RESET);
            channel->trip_time = osKernelSysTick();
            channel->enabled = 0;
        } else if (!channel->enabled) {
            // check if it's time to re-enable this channel
            uint32_t ms_since_trip = osKernelSysTick() - channel->trip_time;
            if (ms_since_trip >= channel->reset_delay_ms) {
                channel->ampsec_sum = 0;
                HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
                channel->enabled = 1;
            }
        }
    }

    osDelay(PLM_DELAY_POWER);
}

void plm_check_cooling_channels(void) {
	//ch_12v_0 is Radiator Fan channel
	//ch_12v_1 is Accumulator Fan channel


	//steering wheel data
	//last rx stops increasing if steering wheel is stalled
	bool coolingBtnCurrState = swButon3_state.data;
	bool lastRx = swButon3_stat.last_rx;

	//margin is 5 seconds to turn the cooling back on, could be changed
	static uint32_t acceptableRxMargin = 5000;


	//local timer to compare against Rx Timer
	//last_rx should stay in line with this timer
	static uint32_t timeDelta = HAL_getTick() - currentRxState;


	if(coolingBtnCurrState && (timeDelta < acceptableRxMargin)){
		ch_12v_0.enabled = FALSE;
		ch_12v_1.enabled = FALSE;
		HAL_GPIO_WritePin(ch_12v_0.enable_switch_port, ch_12v_0.enable_switch_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch_12v_1.enable_switch_port, ch_12v_0.enable_switch_pin, GPIO_PIN_SET);
	}

	if(timeDelta >= acceptableRxMargin){
		ch_12v_0.enabled = TRUE;
		ch_12v_1.enabled = TRUE;
		HAL_GPIO_WritePin(ch_12v_0.enable_switch_port, ch_12v_0.enable_switch_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch_12v_1.enable_switch_port, ch_12v_1.enable_switch_pin, GPIO_PIN_RESET);
	}

}

