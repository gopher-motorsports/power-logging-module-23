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
#include "gopher_sense.h"
#include "usb_device.h"
#include "fatfs.h"
#include "plm_error.h"
#include "plm_sd.h"
#include "plm_xb.h"
#include "plm_sim.h"
#include "plm_data.h"
#include "plm_power.h"
#include "plm_misc.h"

// we might need to turn this up for launch control
#define CAN_MESSAGE_FORWARD_INTERVAL_ms 50

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

extern USBD_HandleTypeDef hUsbDeviceFS;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim10;

extern PLM_DBL_BUFFER SD_DB;
extern PLM_DBL_BUFFER XB_DB;

U32 hcan1_rx_callbacks = 0;
U32 hcan2_rx_callbacks = 0;
U32 hcan3_rx_callbacks = 0;

void plm_init(void) {
    plm_err_reset();
    S8 err = 0;

#ifdef PLM_DEV_MODE
    // put CAN peripherals into loopback to receive simulated data
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    hcan2.Init.Mode = CAN_MODE_LOOPBACK;
    hcan3.Init.Mode = CAN_MODE_LOOPBACK;
    err |= HAL_CAN_Init(&hcan1);
    err |= HAL_CAN_Init(&hcan2);
    err |= HAL_CAN_Init(&hcan3);
    if (err) {
        // PLM shouldn't run without CAN
        plm_err_set(PLM_ERR_INIT);
        HAL_Delay(PLM_DELAY_RESTART);
        NVIC_SystemReset();
    }
#endif

    // GopherCAN
    err |= init_can(GCAN0, &hcan1, PLM_ID, BXTYPE_MASTER);
    err |= init_can(GCAN1, &hcan2, PLM_ID, BXTYPE_MASTER);
    err |= init_can(GCAN2, &hcan3, PLM_ID, BXTYPE_MASTER);
    if (err) {
        plm_err_set(PLM_ERR_INIT);
        HAL_Delay(PLM_DELAY_RESTART);
        NVIC_SystemReset();
    }

    // GopherSense
    gsense_init(&hcan1, &hadc1, NULL, &hadc3, LED_USB_GPIO_Port, LED_USB_Pin);

    // enable all power channel switches
    for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
        PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
        HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
    }

    // we dont want to send parameters
	set_all_param_sending(FALSE);

#ifdef PLM_DEV_MODE
    printf("PLM successfully initialized\n");
#endif
}

void plm_heartbeat(void) {
    uint32_t tick = HAL_GetTick();

    static uint32_t last_blink = 0;
    if (tick - last_blink >= PLM_DELAY_HEARTBEAT_BLINK) {
#ifdef PLM_DEV_MODE
        printf("PLM (%lu): ⚡\n", tick);
#endif
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        last_blink = tick;
    }

    plm_update_logging_metrics();
    plm_err_blink();

    osDelay(PLM_TASK_DELAY_HEARTBEAT);
}

void plm_service_can(void) {
	// send out messages that need to be forwarded
#ifdef GO4_23c
	static U32 last_message_send = 0;
	if (HAL_GetTick() - last_message_send >= CAN_MESSAGE_FORWARD_INTERVAL_ms)
	{
		// send the brake pressure and front wheel speeds to the ECU
		send_group(0x10);
		send_group(0x600);
		last_message_send = HAL_GetTick();
	}
#endif

    service_can_tx(&hcan1);
    service_can_tx(&hcan2);
    service_can_tx(&hcan3);
    service_can_rx_buffer();

    osDelay(PLM_TASK_DELAY_CAN);
}

void GCAN_RxMsgPendingCallback(CAN_HandleTypeDef* hcan, U32 rx_mailbox) {
    if (hcan->Instance == CAN1) hcan1_rx_callbacks++;
    else if (hcan->Instance == CAN2) hcan2_rx_callbacks++;
    else if (hcan->Instance == CAN3) hcan3_rx_callbacks++;

    service_can_rx_hardware(hcan, rx_mailbox);
}

void plm_collect_data(void) {
    static uint32_t sd_last_log[NUM_OF_PARAMETERS] = {0};
    static uint32_t xb_last_send[NUM_OF_PARAMETERS] = {0};

    uint8_t usb_connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
    uint8_t voltage_ok = plmVbatVoltage_V.data >= MIN_VBAT_VOLTAGE_V && plm5VVoltage_V.data >= MIN_5V_VOLTAGE_V;
#ifdef PLM_DEV_MODE
    voltage_ok = 1;
#endif

    // must have usb disconnected and minimum 5V and Vbat voltages
    if (usb_connected || !voltage_ok) {
    	osDelay(PLM_TASK_DELAY_DATA);
    	return;
    }

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

    for (uint8_t i = 1; i < NUM_OF_PARAMETERS; i++) {
        CAN_INFO_STRUCT* param = (CAN_INFO_STRUCT*)(PARAMETERS[i]);
        uint32_t tick = HAL_GetTick();

        if (param->last_rx > sd_last_log[i]) {
            // parameter has been updated
#ifdef PLM_JANK
        	// dont log the PLM current channels too fast
        	if (param->GROUP_ID <= 0x3 && param->last_rx - sd_last_log[i] < 9)
        	{
        		continue;
        	}
#endif
            PLM_RES res = plm_data_record_param(SD_DB.buffers[SD_DB.write_index], param);
            if (res == PLM_OK) {
                packetsLogged_ul.data += 1;
            }
            else {
                packetsDropped_ul.data += 1;
                plm_err_set(res);
            }
            sd_last_log[i] = tick;
        }

        if (param->last_rx > xb_last_send[i] && tick - xb_last_send[i] > PLM_XB_TX_DELAY) {
            // parameter has been updated and hasn't been sent in a while
            PLM_RES res = plm_data_record_param(XB_DB.buffers[XB_DB.write_index], param);
            if (res != PLM_OK) plm_err_set(res);
            xb_last_send[i] = tick;
        }
    }

    osDelay(PLM_TASK_DELAY_DATA);
}

void plm_store_data(void) {
    // if SD is ready for FatFs interaction
    static uint8_t fs_ready = 0;

    // check if device is connected and ready to interact via USB
    uint8_t usb_connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;

    // prevent USB access and FatFs interaction at the same time
    // USB callbacks are in USB_DEVICE/App/usbd_storage_if.c
    // uses the FatFs driver in FATFS/Target/sd_diskio.c
	if (usb_connected && fs_ready) {
#ifdef PLM_DEV_MODE
	    printf("PLM (%lu): USB connected\n", HAL_GetTick());
#endif
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

    osDelay(PLM_TASK_DELAY_SD);
}

void plm_transmit_data(void) {
    if (!XB_DB.tx_cplt) {
        PLM_BUFFER* buffer = XB_DB.buffers[!XB_DB.write_index];
        if (buffer->fill > 0) {
            PLM_RES res = plm_xb_send(buffer->bytes, buffer->fill);
            if (res != PLM_OK) plm_err_set(res);
        } else XB_DB.tx_cplt = 1;
    }

    osDelay(PLM_TASK_DELAY_XB);
}

void plm_simulate_data(void) {
#ifndef PLM_SIMULATE_DATA
    osThreadTerminate(osThreadGetId());
#endif

    PLM_RES res = plm_sim_generate_data();
    if (res != PLM_OK) plm_err_set(PLM_ERR_SIM);

    osDelay(PLM_TASK_DELAY_SIM);
}

void plm_monitor_current(void) {
#ifdef PLM_DEV_MODE
    osThreadTerminate(osThreadGetId());
#endif

#ifdef GO4_23c
	plm_cooling_control();
#endif

    for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
        PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
        plm_power_update_channel(channel);

        if (channel->ampsec_sum > channel->ampsec_max && channel->enabled) {
            // channel has reached Amp*sec threshold, open switch
            HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_RESET);
            channel->trip_time = HAL_GetTick();
            channel->enabled = 0;
        } else if (!channel->enabled) {
            // check if it's time to re-enable this channel
            uint32_t ms_since_trip = HAL_GetTick() - channel->trip_time;
            if (ms_since_trip >= channel->reset_delay_ms) {
                channel->ampsec_sum = 0;
                HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
                channel->enabled = 1;
            }
        }
    }

    osDelay(PLM_TASK_DELAY_POWER);
}
