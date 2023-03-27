/*
 * plm_sd.h
 *
 *  Created on: Jan 29, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_SD_H_
#define INC_PLM_SD_H_

#include <stdint.h>
#include "plm_error.h"

PLM_RES plm_sd_init(const char* filename);
void plm_sd_deinit(void);
PLM_RES plm_sd_write(uint8_t* buffer, uint16_t size);

#endif /* INC_SD_H_ */
