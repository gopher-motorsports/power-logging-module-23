/*
 * plm_error.h
 *
 *  Created on: Mar 20, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_ERROR_H_
#define INC_PLM_ERROR_H_

#include <stdint.h>
#include "stm32f767xx.h"

#define ERR_HARDFAULT_BLINKS 10 // # times to blink in hardfault handler
#define ERR_BLINK_PERIOD 200 // ms between individual blinks
#define ERR_BLINK_DELAY 800 // ms between sets of blinks

typedef enum {
  PLM_OK                = 0,
  PLM_ERR_INIT          = 1,
  PLM_ERR_SD_INIT       = 2,
  PLM_ERR_SD_WRITE      = 3,
  PLM_ERR_XB_TX         = 4,
  PLM_ERR_SIM           = 5,
  PLM_ERR_PACKET        = 6,
  PLM_ERR_BUFFER_FULL   = 7
} PLM_RES;

PLM_RES plm_err_status(void);
void plm_err_set(PLM_RES code);
void plm_err_reset(void);
void plm_err_blink(void);

#endif /* INC_PLM_ERROR_H_ */
