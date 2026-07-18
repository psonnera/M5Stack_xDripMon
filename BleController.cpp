#include "BleController.h"
#include "AppConfig.h"
#include "BleMiBand.h"
#include "BleXdrip4iOS.h"

void bleBegin() {
  if (cfg.source == SRC_XDRIP4IOS)
    xdrip4iosBegin();
  else
    miBandBegin();
}

void bleTick() {
  if (cfg.source == SRC_XDRIP4IOS)
    xdrip4iosTick();
  else
    miBandTick();
}

bool bleIsConnected() {
  return cfg.source == SRC_XDRIP4IOS ? xdrip4iosConnected() : miBandConnected();
}

bool bleIsAuthenticated() {
  return cfg.source == SRC_XDRIP4IOS ? xdrip4iosIsAuthenticated()
                                     : miBandIsAuthenticated();
}

const char *bleModeName() {
  return cfg.source == SRC_XDRIP4IOS ? "xDrip4iOS" : "xDrip";
}

void bleNotifySnoozeToPhone() {
  if (cfg.source == SRC_MIBAND)
    miBandSendSnooze();
}
