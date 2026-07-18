#ifndef BLECONTROLLER_H
#define BLECONTROLLER_H

#include <Arduino.h>

// starts the BLE server for the configured data source (cfg.source)
void bleBegin();
// call from loop: restarts advertising after disconnects if needed
void bleTick();
bool bleIsConnected();
// connected AND the phone has completed authentication/pairing
bool bleIsAuthenticated();
const char *bleModeName();
// user snoozed locally; forward to the phone when the protocol supports it
void bleNotifySnoozeToPhone();

#endif
