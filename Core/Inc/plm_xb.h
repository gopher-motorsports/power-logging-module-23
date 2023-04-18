/*
 * plm_xb.h
 *
 *  Created on: Feb 4, 2023
 *      Author: jonol
 */

#ifndef INC_PLM_XB_H_
#define INC_PLM_XB_H_

#include <stdint.h>
#include "plm_error.h"

#define PLM_XB_TX_DELAY 200

PLM_RES plm_xb_send(uint8_t* buffer, uint16_t size);

#endif /* INC_PLM_XB_H_ */
