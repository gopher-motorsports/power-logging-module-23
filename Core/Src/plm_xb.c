/*
 * plm_xb.c
 *
 *  Created on: Feb 4, 2023
 *      Author: jonathan
 */

#include "stm32f7xx_hal.h"
#include "plm_xb.h"

extern UART_HandleTypeDef huart2;

void plm_xb_send(uint8_t* buffer, uint16_t size) {
    HAL_UART_Transmit_DMA(&huart2, buffer, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    return;
}
