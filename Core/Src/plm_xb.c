/*
 * plm_xb.c
 *
 *  Created on: Feb 4, 2023
 *      Author: jonathan
 */

#include "stm32f7xx_hal.h"
#include "plm_xb.h"

extern UART_HandleTypeDef huart2;

PLM_RES plm_xb_send(uint8_t* buffer, uint16_t size) {
    HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&huart2, buffer, size);
    return res == HAL_OK ? PLM_OK : PLM_ERR_XB_TX;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    return;
}
