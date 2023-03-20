/*
 * plm_error.c
 *
 *  Created on: Mar 20, 2023
 *      Author: jonathan
 */

#include "plm_error.h"
#include "main.h"

static PLM_RES err_code;
static GPIO_TypeDef* err_led_port;
static uint16_t err_led_pin;

PLM_RES plm_err_status(void) {
    return err_code;
}

void plm_err_set(PLM_RES code) {
    err_code = code;

    // TODO: map codes to LEDs
//    if (code >= PLM_ERR_SD_INIT && code <= PLM_ERR_SD_WRITE)
    err_led_port = DEV_LED_GPIO_Port;
    err_led_pin = DEV_LED_Pin;
}

void plm_err_reset(void) {
    err_code = PLM_OK;
    err_led_port = NULL;
    err_led_pin = 0;
}

void plm_err_blink(void) {
    static uint32_t last_blink = 0;
    static uint16_t blinks_remaining = 0;

    if (err_led_port == NULL) return;

    if (err_code == PLM_OK && blinks_remaining == 0) {
        // no error to show
        HAL_GPIO_WritePin(err_led_port, err_led_pin, GPIO_PIN_RESET);
    } else if (blinks_remaining == 0) {
        // not ok, start blinking
        uint32_t tick = osKernelSysTick();
        if (tick - last_blink >= ERR_BLINK_DELAY) {
            blinks_remaining = err_code * 2;
            HAL_GPIO_WritePin(err_led_port, err_led_pin, GPIO_PIN_SET);
            last_blink = tick;
        }
    } else {
        // continue blinking
        uint32_t tick = osKernelSysTick();
        if (tick - last_blink >= ERR_BLINK_PERIOD) {
            blinks_remaining--;
            HAL_GPIO_TogglePin(err_led_port, err_led_pin);
            last_blink = tick;
        }
    }
}
