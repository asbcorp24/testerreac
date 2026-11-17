#pragma once
#include <Arduino.h>
void imuReceiverBegin();
void imuReceiverPoll();
void imuMarkCue(uint32_t us);
void imuMarkPress(uint32_t us);
String imuStatusJson();
String imuTrialJson();
