/*
 * plm_sim.c
 *
 *  Created on: Mar 20, 2023
 *      Author: jonathan
 */

#include "plm_sim.h"
#include "GopherCAN.h"

PLM_RES plm_sim_generate_data(void) {
    CAN_INFO_STRUCT* param = (CAN_INFO_STRUCT*)(PARAMETERS[1]);
    S8 err = send_parameter(param);
    if (err) return PLM_ERR_SIM;

    return PLM_OK;
}
