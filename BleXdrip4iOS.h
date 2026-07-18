#ifndef BLEXDRIP4IOS_H
#define BLEXDRIP4IOS_H

#include <Arduino.h>

// xDrip4iOS / xdripswift "M5Stack" BLE protocol (port of M5_NightscoutMon-xDrip4iOS)
void xdrip4iosBegin();
void xdrip4iosTick();
bool xdrip4iosConnected();
bool xdrip4iosIsAuthenticated();

#endif
