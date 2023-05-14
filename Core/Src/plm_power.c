/*
 * plm_power.c
 *
 *  Created on: Mar 31, 2023
 *      Author: jonathan
 */

#include "plm_power.h"
#include "main.h"
#include "cmsis_os.h"

// CAN SUPPORT: 20A
// IC CHANNEL:  Fans
// EXPECTED IC: 6A
// EV CHANNEL:  Radiator Fan
// EXPECTED EV: 4A
PLM_POWER_CHANNEL ch_12v_0 = {
    .parameter = &vbatChan0Current_A,
    .enable_switch_port = EN_12V_0_GPIO_Port,
    .enable_switch_pin = EN_12V_0_Pin,
    .enabled = 0,
    .amp_max = 9.0f,
    .ampsec_max = 4.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 20A
// IC CHANNEL:  Water Pump
// EXPECTED IC: 7.5A
// EV CHANNEL:  Accumulator Fans
// EXPECTED EV: 14A
PLM_POWER_CHANNEL ch_12v_1 = {
    .parameter = &vbatChan1Current_A,
    .enable_switch_port = EN_12V_1_GPIO_Port,
    .enable_switch_pin = EN_12V_1_Pin,
    .enabled = 0,
    .amp_max = 15.0f,
    .ampsec_max = 5.0f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 20A
// IC CHANNEL:  IGN and Fuel Pump
// EXPECTED IC: 10A
// EV CHANNEL:  Water Pump
// EXPECTED EV: 14A
PLM_POWER_CHANNEL ch_12v_2 = {
    .parameter = &vbatChan2Current_A,
    .enable_switch_port = EN_12V_2_GPIO_Port,
    .enable_switch_pin = EN_12V_2_Pin,
    .enabled = 0,
    .amp_max = 15.0f,
    .ampsec_max = 5.0f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 10A
// IC CHANNEL:  ECU and Fuel INJ
// EXPECTED IC: 4A
// EV CHANNEL:  Inverter and Accumulator
// EXPECTED EV: 4A
PLM_POWER_CHANNEL ch_12v_3 = {
    .parameter = &vbatChan3Current_A,
    .enable_switch_port = EN_12V_3_GPIO_Port,
    .enable_switch_pin = EN_12V_3_Pin,
    .enabled = 0,
    .amp_max = 6.0f,
    .ampsec_max = 3.0f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 10A
// IC CHANNEL:  Front VBat things
// EXPECTED IC: 1A
// EV CHANNEL:  Front VBat things
// EXPECTED EV: 1A
PLM_POWER_CHANNEL ch_12v_4 = {
    .parameter = &vbatChan4Current_A,
    .enable_switch_port = EN_12V_4_GPIO_Port,
    .enable_switch_pin = EN_12V_4_Pin,
    .enabled = 0,
    .amp_max = 3.0f,
    .ampsec_max = 1.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 10A
// IC CHANNEL:  Rear VBat things
// EXPECTED IC: 1A
// EV CHANNEL:  Rear VBat things
// EXPECTED EV: 1A
PLM_POWER_CHANNEL ch_12v_5 = {
    .parameter = &vbatChan5Current_A,
    .enable_switch_port = EN_12V_5_GPIO_Port,
    .enable_switch_pin = EN_12V_5_Pin,
    .enabled = 0,
    .amp_max = 3.0f,
    .ampsec_max = 1.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 10A
// IC CHANNEL:  Mid VBat things
// EXPECTED IC: 1A
// EV CHANNEL:  Shut Down Circuit -> AIRs
// EXPECTED EV: 1A
PLM_POWER_CHANNEL ch_12v_6 = {
    .parameter = &vbatChan6Current_A,
    .enable_switch_port = EN_12V_6_GPIO_Port,
    .enable_switch_pin = EN_12V_6_Pin,
    .enabled = 0,
    .amp_max = 3.0f,
    .ampsec_max = 1.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 2A
// IC CHANNEL:  TCM
// EXPECTED IC: 0.5A
// EV CHANNEL:  VCU
// EXPECTED EV: 0.5A
PLM_POWER_CHANNEL ch_5v_0 = {
    .parameter = &fiveVChan0Current_A,
    .enable_switch_port = EN_5V_0_GPIO_Port,
    .enable_switch_pin = EN_5V_0_Pin,
    .enabled = 0,
    .amp_max = 1.0f,
    .ampsec_max = 0.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 2A
// IC CHANNEL:  Rear 5V
// EXPECTED IC: 0.5A
// EV CHANNEL:  Rear 5V
// EXPECTED EV: 0.5A
PLM_POWER_CHANNEL ch_5v_1 = {
    .parameter = &fiveVChan1Current_A,
    .enable_switch_port = EN_5V_1_GPIO_Port,
    .enable_switch_pin = EN_5V_1_Pin,
    .enabled = 0,
    .amp_max = 1.0f,
    .ampsec_max = 0.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 2A
// IC CHANNEL:  Front 5V
// EXPECTED IC: 0.5A
// EV CHANNEL:  Front 5V
// EXPECTED EV: 0.5A
PLM_POWER_CHANNEL ch_5v_2 = {
    .parameter = &fiveVChan2Current_A,
    .enable_switch_port = EN_5V_2_GPIO_Port,
    .enable_switch_pin = EN_5V_2_Pin,
    .enabled = 0,
    .amp_max = 1.0f,
    .ampsec_max = 0.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

// CAN SUPPORT: 2A
// IC CHANNEL:  Aux 5V
// EXPECTED IC: 0.5A
// EV CHANNEL:  Aux 5V
// EXPECTED EV: 0.5A
PLM_POWER_CHANNEL ch_5v_3 = {
    .parameter = &fiveVChan3Current_A,
    .enable_switch_port = EN_5V_3_GPIO_Port,
    .enable_switch_pin = EN_5V_3_Pin,
    .enabled = 0,
    .amp_max = 1.0f,
    .ampsec_max = 0.5f,
    .ampsec_sum = 0.0f,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL* POWER_CHANNELS[NUM_OF_CHANNELS] = {
    &ch_12v_0,
    &ch_12v_1,
    &ch_12v_2,
    &ch_12v_3,
    &ch_12v_4,
    &ch_12v_5,
    &ch_12v_6,
    &ch_5v_0,
    &ch_5v_1,
    &ch_5v_2,
    &ch_5v_3,
};

void plm_power_update_channel(PLM_POWER_CHANNEL* channel) {
    uint32_t tick = HAL_GetTick();
    uint32_t elapsed_ms = tick - channel->last_update;
    channel->last_update = tick;
    float delta_max = channel->parameter->data - channel->amp_max;

    // integrate Amps*sec
    channel->ampsec_sum += delta_max * (elapsed_ms / 1000.0);

    // bound current integral above 0
    if (channel->ampsec_sum <= 0) channel->ampsec_sum = 0;
}

void plm_cooling_control(void) {
    // code to turn off the fans if the wheel speed goes above a threshold
    if (HAL_GetTick() - wheelSpeedFrontLeft_mph.info.last_rx <= TRUST_VALUE_TIME_DELTA_ms &&
        HAL_GetTick() - wheelSpeedFrontLeft_mph.info.last_rx <= TRUST_VALUE_TIME_DELTA_ms &&
        wheelSpeedFrontLeft_mph.data >= WHEEL_SPEED_FAN_OFF_THRESH_mph &&
        wheelSpeedFrontRight_mph.data >= WHEEL_SPEED_FAN_OFF_THRESH_mph)
    {
        // we want to turn off this channel. This is done by manually writing to
        // the GPIO pin without changing the channel enabled state, meaning the power
        // logic should still be fine
        HAL_GPIO_WritePin(EN_12V_0_GPIO_Port, EN_12V_0_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // fans can be turned back on as long as the channel is enabled
        if (POWER_CHANNELS[0]->enabled)
        {
            HAL_GPIO_WritePin(EN_12V_0_GPIO_Port, EN_12V_0_Pin, GPIO_PIN_SET);
        }
    }
}
