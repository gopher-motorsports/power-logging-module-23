/*
 * plm.h
 *
 *  Created on: Jan 13, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_H_
#define INC_PLM_H_

// task delays
#define PLM_TASK_DELAY_HEARTBEAT 100
#define PLM_TASK_DELAY_CAN 1
#define PLM_TASK_DELAY_DATA 1
#define PLM_TASK_DELAY_SD 250
#define PLM_TASK_DELAY_XB 1000
#define PLM_TASK_DELAY_SIM 5000
#define PLM_TASK_DELAY_POWER 10

// other delays
#define PLM_DELAY_RESTART 1000
#define PLM_DELAY_HEARTBEAT_BLINK 1000
#define PLM_DELAY_LOGGING_METRICS 1000

// ignores voltage check in plm_collect_data
// terminates the plm_monitor_current thread
//#define PLM_DEV_MODE

// automatically generates and transmits GCAN data
// intended for loopback mode
//#define PLM_SIMULATE_DATA

// to define which car we are using
#define GO4_23c
//#define GO4_23e

void plm_init(void);
void plm_heartbeat(void);
void plm_service_can(void);
void plm_collect_data(void);
void plm_store_data(void);
void plm_transmit_data(void);
void plm_simulate_data(void);
void plm_monitor_current(void);

#endif /* INC_PLM_H_ */
