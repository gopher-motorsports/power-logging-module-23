#ifndef over_current_protection_h
#define over_current_protection_h
#include <stdbool.h>

typedef struct {

  uint32_t currentTime;
  uint32_t timeFromlastCalculation;

  uint32_t tripTimeStart;
  uint32_t trippedTimeResetThreshold;
  bool tripped;

  float currentMax;
  float amperageSecondsLimit;
  float amperageRightNow;
  float amperageRightNowAdjusted;
  float amperageSecondsSum;

  uint8_t powerSwitchPin;
  uint8_t powerSwitchPort;

} plmChannel;

void isOverCurrentLimit(plmChannel *inputChan);
int over_current_protection();
#endif
