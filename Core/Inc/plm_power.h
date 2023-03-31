/*
 * plm_power.h
 *
 *  Created on: Mar 31, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_POWER_H_
#define INC_PLM_POWER_H_

typedef struct {
    GPIO_TypeDef* enable_switch_port;
    uint16_t enable_switch_pin;
} PLM_POWER_CHANNEL;

#endif /* INC_PLM_POWER_H_ */
