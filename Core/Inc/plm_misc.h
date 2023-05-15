/*
 * plm_misc.h
 *
 *  Created on: May 14, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_MISC_H_
#define INC_PLM_MISC_H_

#define PLM_DELAY_LOGGING_METRICS 1000
#define PLM_DELAY_SYNC_RTC 10000

void plm_update_logging_metrics(void);
void plm_sync_rtc(void);

#endif /* INC_PLM_MISC_H_ */
