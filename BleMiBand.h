#ifndef BLEMIBAND_H
#define BLEMIBAND_H

#include <Arduino.h>

// Mi Band 2 emulation: xDrip+ (Android) connects to us as if we were a band
// and pushes BG as text alerts ("BG: 123 →") to the New Alert characteristic.
void miBandBegin();
void miBandTick();
bool miBandConnected();
bool miBandIsAuthenticated();
void miBandSendSnooze();   // DeviceEvent CALL_REJECT -> snoozes the xDrip alert

#endif
