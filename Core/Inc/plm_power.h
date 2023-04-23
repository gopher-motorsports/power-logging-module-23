/*
 * plm_power.h
 *
 *  Created on: Mar 31, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_POWER_H_
#define INC_PLM_POWER_H_

#include "main.h"
#include "plm_error.h"
#include "GopherCAN.h"

#define NUM_OF_CHANNELS 11
#define MIN_5V_VOLTAGE_V 2.0f
#define MIN_VBAT_VOLTAGE_V 2.0f

typedef struct {
    FLOAT_CAN_STRUCT* parameter;
    GPIO_TypeDef* enable_switch_port;
    uint16_t enable_switch_pin;
    uint8_t enabled;
    float amp_max;
    float ampsec_max;
    float ampsec_sum;
    uint32_t trip_time;
    uint32_t reset_delay_ms;
    uint32_t last_update;
} PLM_POWER_CHANNEL;

extern PLM_POWER_CHANNEL* POWER_CHANNELS[NUM_OF_CHANNELS];

void plm_power_update_channel(PLM_POWER_CHANNEL* channel);

#endif /* INC_PLM_POWER_H_ */
