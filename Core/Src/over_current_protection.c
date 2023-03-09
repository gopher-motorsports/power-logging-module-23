#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "over_current_protection.h"
//include GopherCan.h cause it has all networks and buffers, also has TRUE and FALSE defined
#include "GopherCAN.h"
#include "cmsis_os.h"

//could add up differences from orignal timer in that period
//room for optomization and readability

void isOverCurrentLimit(plmChannel *inputChan){
    inputChan -> currentTime = HAL_GetTick();
    inputChan -> amperageRightNowAdjusted = inputChan -> amperageRightNow - inputChan -> currentMax;
    uint32_t changeInTime = (inputChan -> currentTime)-(inputChan -> timeFromlastCalculation);

    inputChan -> amperageSecondsSum += inputChan -> amperageRightNowAdjusted * changeInTime;

    inputChan -> timeFromlastCalculation = inputChan -> currentTime;

    //makes sure ampSecondsSum doesn't go below 0
    if(inputChan -> amperageSecondsSum <= 0){
    	inputChan -> amperageSecondsSum = 0;
    }

    else{
		//if integral is greater than threshold and has not yet past threshold
    	if((inputChan -> amperageSecondsSum > inputChan -> amperageSecondsLimit) && !inputChan -> tripped){
			inputChan -> powerSwitchPin = 0;
			inputChan -> tripTimeStart = inputChan -> timeFromlastCalculation;
			inputChan -> tripped = TRUE;
		}
    	//if it has past the amp second threshold, wait reset threshold before setting back to zero
    	else if((inputChan -> tripped == TRUE)){
			uint32_t trippedDuration = (inputChan -> currentTime)-(inputChan -> tripTimeStart);

			if(trippedDuration > inputChan -> trippedTimeResetThreshold){
				inputChan -> amperageSecondsSum = 0;
				inputChan -> powerSwitchPin = 1;
				inputChan -> tripped = FALSE;

			}
		}
    }
}


int over_current_protection()
{
    //fiveV = 5V
    plmChannel vbat_chan0 = {0};
    vbat_chan0.currentMax = 10.0;
    vbat_chan0.powerSwitchPin = 1;
    //test values:
   /* vbat_chan0.trippedTimeResetThreshold = 5000;
	vbat_chan0.amperageSecondsLimit = 3000;*/

    plmChannel vbat_chan1 = {0};
    vbat_chan1.currentMax = 10.0;
    vbat_chan1.powerSwitchPin = 1;

    plmChannel vbat_chan2 = {0};
    vbat_chan2.currentMax = 10.0;
    vbat_chan2.powerSwitchPin = 1;

    plmChannel vbat_chan3 = {0};
    vbat_chan3.currentMax = 10.0;
    vbat_chan3.powerSwitchPin = 1;

    plmChannel vbat_chan4 = {0};
    vbat_chan4.currentMax = 10.0;
    vbat_chan4.powerSwitchPin = 1;

    plmChannel vbat_chan5 = {0};
    vbat_chan5.currentMax = 10.0;
    vbat_chan5.powerSwitchPin = 1;

    plmChannel vbat_chan6 = {0};
    vbat_chan6.currentMax = 10.0;
    vbat_chan6.powerSwitchPin = 1;


    plmChannel fiveV_chan0 = {0};
    fiveV_chan0.currentMax = 5.0;
    fiveV_chan0.powerSwitchPin = 1;

    plmChannel fiveV_chan1 = {0};
    fiveV_chan1.currentMax = 5.0;
    fiveV_chan0.powerSwitchPin = 1;

    plmChannel fiveV_chan2 = {0};
    fiveV_chan2.currentMax = 5.0;
    fiveV_chan0.powerSwitchPin = 1;

    plmChannel fiveV_chan3 = {0};
    fiveV_chan3.currentMax = 5.0;
    fiveV_chan0.powerSwitchPin = 1;


    while(1){
		vbat_chan0.amperageRightNow = vbatChan0Current_A.data;
		/*vbat_chan1.amperageRightNow = vbatChan1Current_A.data;
		vbat_chan2.amperageRightNow = vbatChan2Current_A.data;
		vbat_chan3.amperageRightNow = vbatChan3Current_A.data;
		vbat_chan4.amperageRightNow = vbatChan4Current_A.data;
		vbat_chan5.amperageRightNow = vbatChan5Current_A.data;
		vbat_chan6.amperageRightNow = vbatChan6Current_A.data;

		fiveV_chan0.amperageRightNow = fiveVChan0Current_A.data;
		fiveV_chan1.amperageRightNow = fiveVChan1Current_A.data;
		fiveV_chan2.amperageRightNow = fiveVChan2Current_A.data;
		fiveV_chan3.amperageRightNow = fiveVChan3Current_A.data;*/




		isOverCurrentLimit(&vbat_chan0);
		/*isOverCurrentLimit(&vbat_chan1);
		isOverCurrentLimit(&vbat_chan2);
		isOverCurrentLimit(&vbat_chan3);
		isOverCurrentLimit(&vbat_chan4);
		isOverCurrentLimit(&vbat_chan5);
		isOverCurrentLimit(&vbat_chan6);*/


		/*isOverCurrentLimit(&fiveV_chan0);
		isOverCurrentLimit(&fiveV_chan1);
		isOverCurrentLimit(&fiveV_chan2);
		isOverCurrentLimit(&fiveV_chan3);*/

		osDelay(1);
    }
    //printf("%d",vbat_chan0.powerSwitchPin);
    return 0;
}


