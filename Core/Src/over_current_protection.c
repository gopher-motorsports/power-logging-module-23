#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "GopherCAN.h"



typedef struct {

  uint32_t currentTime;
  uint32_t timeFromlastCalculation;

  uint32_t tripTimeStart;
  uint32_t trippedTimeResetThreshold;
  bool tripped;

  float currentMax;
  float amperageSecondsLimit;
  float amperageRightNow;
  float amperageSecondsSum;

  uint8_t powerSwitchPin;
  uint8_t powerSwitchPort;

} plmChannel;

//could add up differences from orignal timer in that period
//room for opromiation and readability

void isOverCurrentLimit(plmChannel *inputChan){
    inputChan -> currentTime = HAL_GetTick();
    uint32_t changeInTime = (inputChan -> currentTime)-(inputChan -> timeFromlastCalculation);

    inputChan -> amperageSecondsSum += inputChan -> amperageRightNow * changeInTime;

    inputChan -> timeFromlastCalculation = inputChan -> currentTime;
    if((inputChan -> amperageSecondsSum > inputChan -> amperageSecondsLimit) && !inputChan -> tripped){
        inputChan -> powerSwitchPin = 0;
        inputChan -> tripTimeStart = inputChan -> timeFromlastCalculation;
        inputChan -> tripped = TRUE;
    }


    if((inputChan -> tripped == TRUE)){
    	uint32_t trippedDuration = (inputChan -> currentTime)-(inputChan -> tripTimeStart);

    	if(trippedDuration > inputChan -> trippedTimeResetThreshold){
    		inputChan -> amperageSecondsSum = 0;
    		inputChan -> tripped = FALSE;

    	}
    }

}


int over_current_protection()
{
    //Vbat = 10V, fiveV = 5V
    plmChannel vbat_chan0 = {0};
    vbat_chan0.currentMax = 10.0;
    vbat_chan0.powerSwitchPin = 1;

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
		vbat_chan1.amperageRightNow = vbatChan1Current_A.data;
		vbat_chan2.amperageRightNow = vbatChan2Current_A.data;
		vbat_chan3.amperageRightNow = vbatChan3Current_A.data;
		vbat_chan4.amperageRightNow = vbatChan4Current_A.data;
		vbat_chan5.amperageRightNow = vbatChan5Current_A.data;
		vbat_chan6.amperageRightNow = vbatChan6Current_A.data;

		fiveV_chan0.amperageRightNow = fiveVChan0Current_A.data;
		fiveV_chan1.amperageRightNow = fiveVChan1Current_A.data;
		fiveV_chan2.amperageRightNow = fiveVChan2Current_A.data;
		fiveV_chan3.amperageRightNow = fiveVChan3Current_A.data;




		isOverCurrentLimit(&vbat_chan0);
		isOverCurrentLimit(&vbat_chan1);
		isOverCurrentLimit(&vbat_chan2);
		isOverCurrentLimit(&vbat_chan3);
		isOverCurrentLimit(&vbat_chan4);
		isOverCurrentLimit(&vbat_chan5);
		isOverCurrentLimit(&vbat_chan6);


		isOverCurrentLimit(&fiveV_chan0);
		isOverCurrentLimit(&fiveV_chan1);
		isOverCurrentLimit(&fiveV_chan2);
		isOverCurrentLimit(&fiveV_chan3);


    }
    //printf("%d",vbat_chan0.powerSwitchPin);
    return 0;
}


