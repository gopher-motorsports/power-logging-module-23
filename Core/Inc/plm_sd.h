/*
 * sd.h
 *
 *  Created on: Jan 29, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_SD_H_
#define INC_PLM_SD_H_

uint8_t sd_init(void);
void sd_deinit(void);
uint8_t sd_write(uint8_t* buffer, uint16_t size);

#endif /* INC_SD_H_ */
