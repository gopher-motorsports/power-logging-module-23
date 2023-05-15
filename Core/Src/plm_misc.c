/*
 * plm_misc.c
 *
 *  Created on: May 14, 2023
 *      Author: jonathan
 */

#include "plm_misc.h"
#include "plm.h"
#include "plm_data.h"
#include "GopherCAN.h"

extern PLM_DBL_BUFFER SD_DB;
extern PLM_DBL_BUFFER XB_DB;

extern U32 hcan1_rx_callbacks;
extern U32 hcan2_rx_callbacks;
extern U32 hcan3_rx_callbacks;

void plm_update_logging_metrics(void) {
    static uint32_t last_update = 0;
    uint32_t tick = HAL_GetTick();
    if (tick - last_update >= PLM_DELAY_LOGGING_METRICS) {
        packetsDropped_ul.info.last_rx = tick;
        packetsLogged_ul.info.last_rx = tick;

        // SD and Xbee buffer fill %
        storageBufferFill_percent.data = (float) SD_DB.buffers[SD_DB.write_index]->fill / SD_DB.buffers[SD_DB.write_index]->size * 100.0f;
        telemetryBufferFill_percent.data = (float) XB_DB.buffers[XB_DB.write_index]->fill / XB_DB.buffers[XB_DB.write_index]->size * 100.0f;
        storageBufferFill_percent.info.last_rx = tick;
        telemetryBufferFill_percent.info.last_rx = tick;

        // CAN bus load
        // does not include messages filtered out by hardware
        // does not include messages sent by the PLM
        // assumes all CAN frames are 125 bits (11-bit ID, 8 data bytes, other frame stuff)
        float hcan1_rx_bps = (hcan1_rx_callbacks * 125) / ((float)(tick - last_update) / 1000.0f);
        float hcan2_rx_bps = (hcan2_rx_callbacks * 125) / ((float)(tick - last_update) / 1000.0f);
        float hcan3_rx_bps = (hcan3_rx_callbacks * 125) / ((float)(tick - last_update) / 1000.0f);

        // bus load = sampled bps / 1Mbps
        can0Utilization_percent.data = hcan1_rx_bps / 1000000.0f * 100.0f;
        can1Utilization_percent.data = hcan2_rx_bps / 1000000.0f * 100.0f;
        can2Utilization_percent.data = hcan3_rx_bps / 1000000.0f * 100.0f;

        can0Utilization_percent.info.last_rx = tick;
        can1Utilization_percent.info.last_rx = tick;
        can2Utilization_percent.info.last_rx = tick;

        hcan1_rx_callbacks = 0;
        hcan2_rx_callbacks = 0;
        hcan3_rx_callbacks = 0;

        last_update = tick;
    }
}
