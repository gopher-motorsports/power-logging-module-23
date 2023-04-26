/*
 * plm.h
 *
 *  Created on: Jan 13, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_H_
#define INC_PLM_H_

#define PLM_DELAY_HEARTBEAT 100
#define PLM_DELAY_HEARTBEAT_BLINK 1000
#define PLM_DELAY_CAN 10
#define PLM_DELAY_DATA 10
#define PLM_DELAY_SD 100
#define PLM_DELAY_XB 1000
#define PLM_DELAY_SIM 5000
#define PLM_DELAY_POWER 10
#define PLM_DELAY_RESTART 1000

// define to automatically generate and transmit GCAN data
#define PLM_SIMULATE_DATA

void plm_init(void);
void plm_heartbeat(void);
void plm_service_can(void);
void plm_collect_data(void);
void plm_store_data(void);
void plm_transmit_data(void);
void plm_simulate_data(void);
void plm_monitor_current(void);
void plm_check_cooling_channels(void);


#endif /* INC_PLM_H_ */
