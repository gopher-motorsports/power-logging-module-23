/*
 * plm_error.c
 *
 *  Created on: Mar 20, 2023
 *      Author: jonathan
 */

#include <stdio.h>
#include "plm.h"
#include "plm_error.h"
#include "main.h"
#include "cmsis_os.h"

// USB LED is used for GopherSense status
// STATUS LED is used for heartbeat blink
// FAULT LED is controlled in HardFault_Handler

#define NUM_OF_LEDS 6

#define ERR_STATUS 0
#define ERR_FAULT 1
#define ERR_MEMORY 2
#define ERR_STORAGE 3
#define ERR_USB 4
#define ERR_OVERCURRENT 5

static GPIO_TypeDef* led_port[NUM_OF_LEDS] = {
    LED_STATUS_GPIO_Port,
    LED_FAULT_GPIO_Port,
    LED_MEMORY_GPIO_Port,
    LED_STORAGE_GPIO_Port,
    LED_USB_GPIO_Port,
    LED_OVERCURRENT_GPIO_Port
};

static uint16_t led_pin[NUM_OF_LEDS] = {
    LED_STATUS_Pin,
    LED_FAULT_Pin,
    LED_MEMORY_Pin,
    LED_STORAGE_Pin,
    LED_USB_Pin,
    LED_OVERCURRENT_Pin
};

static PLM_RES err_state[NUM_OF_LEDS] = {PLM_OK, PLM_OK, PLM_OK, PLM_OK, PLM_OK, PLM_OK};

void plm_err_set(PLM_RES code) {
#ifdef PLM_DEV_MODE
    printf("ERROR (%lu): code %u set\n", HAL_GetTick(), code);
#endif

    if (code >= PLM_ERR_SD_INIT && code <= PLM_ERR_SD_WRITE) {
        err_state[ERR_STORAGE] = code;
    } else if (code == PLM_ERR_BUFFER_FULL) {
        err_state[ERR_MEMORY] = code;
    }
}

void plm_err_reset(void) {
    for (uint8_t i = 1; i < NUM_OF_LEDS; i++) {
        err_state[i] = PLM_OK;
    }
}

void plm_err_blink(void) {
    static uint32_t last_blink[NUM_OF_LEDS] = {0};
    static uint16_t blinks_remaining[NUM_OF_LEDS] = {0};

    // check state for each LED
    for (uint8_t i = 1; i < NUM_OF_LEDS; i++) {
        if (err_state[i] == PLM_OK && blinks_remaining[i] == 0) {
            // no error to show
            HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_RESET);
        } else if (blinks_remaining == 0) {
            // not ok, start/restart blinking
            uint32_t tick = HAL_GetTick();
            if (tick - last_blink[i] >= ERR_BLINK_DELAY) {
                blinks_remaining[i] = err_state[i] * 2;
                HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_SET);
#ifdef PLM_DEV_MODE
                printf("ERROR (%lu): code %u active on LED %u\n", HAL_GetTick(), err_state[i], i);
#endif
                last_blink[i] = tick;
            }
        } else {
            // continue blinking
            uint32_t tick = HAL_GetTick();
            if (tick - last_blink[i] >= ERR_BLINK_PERIOD) {
                blinks_remaining[i]--;
                HAL_GPIO_TogglePin(led_port[i], led_pin[i]);
                last_blink[i] = tick;
            }
        }
    }
}
