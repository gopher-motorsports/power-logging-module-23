/*
 * plm_power.c
 *
 *  Created on: Mar 31, 2023
 *      Author: jonathan
 */

#include "plm_power.h"
#include "main.h"
#include "cmsis_os.h"

//PLM_POWER_CHANNEL ch_12v_0 = {
//    .parameter = &vbatChan0Current_A,
//    .enable_switch_port = EN_12V_0_GPIO_Port,
//    .enable_switch_pin = EN_12V_0_Pin,
//    .enabled = 0,
//    .amp_max = 2,
//    .ampsec_max = 1000,
//    .ampsec_sum = 0,
//    .trip_time = 0,
//    .reset_delay_ms = 1000,
//    .last_update = 0
//};
//
//PLM_POWER_CHANNEL ch_12v_1 = {
//    .parameter = &vbatChan1Current_A,
//    .enable_switch_port = EN_12V_1_GPIO_Port,
//    .enable_switch_pin = EN_12V_1_Pin,
//    .enabled = 0,
//    .amp_max = 2,
//    .ampsec_max = 1000,
//    .ampsec_sum = 0,
//    .trip_time = 0,
//    .reset_delay_ms = 1000,
//    .last_update = 0
//};
//
//PLM_POWER_CHANNEL ch_12v_2 = {
//    .parameter = &vbatChan2Current_A,
//    .enable_switch_port = EN_12V_2_GPIO_Port,
//    .enable_switch_pin = EN_12V_2_Pin,
//    .enabled = 0,
//    .amp_max = 2,
//    .ampsec_max = 1000,
//    .ampsec_sum = 0,
//    .trip_time = 0,
//    .reset_delay_ms = 1000,
//    .last_update = 0
//};

PLM_POWER_CHANNEL ch_12v_3 = {
    .parameter = &vbatChan3Current_A,
    .enable_switch_port = EN_12V_3_GPIO_Port,
    .enable_switch_pin = EN_12V_3_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_12v_4 = {
    .parameter = &vbatChan4Current_A,
    .enable_switch_port = EN_12V_4_GPIO_Port,
    .enable_switch_pin = EN_12V_4_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_12v_5 = {
    .parameter = &vbatChan5Current_A,
    .enable_switch_port = EN_12V_5_GPIO_Port,
    .enable_switch_pin = EN_12V_5_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_12v_6 = {
    .parameter = &vbatChan6Current_A,
    .enable_switch_port = EN_12V_6_GPIO_Port,
    .enable_switch_pin = EN_12V_6_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_5v_0 = {
    .parameter = &fiveVChan0Current_A,
    .enable_switch_port = EN_5V_0_GPIO_Port,
    .enable_switch_pin = EN_5V_0_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_5v_1 = {
    .parameter = &fiveVChan1Current_A,
    .enable_switch_port = EN_5V_1_GPIO_Port,
    .enable_switch_pin = EN_5V_1_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_5v_2 = {
    .parameter = &fiveVChan2Current_A,
    .enable_switch_port = EN_5V_2_GPIO_Port,
    .enable_switch_pin = EN_5V_2_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL ch_5v_3 = {
    .parameter = &fiveVChan3Current_A,
    .enable_switch_port = EN_5V_3_GPIO_Port,
    .enable_switch_pin = EN_5V_3_Pin,
    .enabled = 0,
    .amp_max = 2,
    .ampsec_max = 1000,
    .ampsec_sum = 0,
    .trip_time = 0,
    .reset_delay_ms = 1000,
    .last_update = 0
};

PLM_POWER_CHANNEL* POWER_CHANNELS[NUM_OF_CHANNELS] = {
//    &ch_12v_0,
//    &ch_12v_1,
//    &ch_12v_2,
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
    uint32_t tick = osKernelSysTick();
    uint32_t elapsed_ms = tick - channel->last_update;
    channel->last_update = tick;
    float delta_max = channel->parameter->data - channel->amp_max;

    // integrate Amps*sec
    channel->ampsec_sum += delta_max * (elapsed_ms / 1000);

    // bound current integral above 0
    if (channel->ampsec_sum <= 0) channel->ampsec_sum = 0;
}
