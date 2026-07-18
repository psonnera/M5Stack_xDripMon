#ifndef APPCONFIG_H
#define APPCONFIG_H

#include <Arduino.h>

// data sources
#define SRC_MIBAND    0   // Android xDrip+ via Mi Band 2 emulation
#define SRC_XDRIP4IOS 1   // xDrip4iOS / xdripswift via "M5Stack" protocol

// units
#define UNITS_MGDL 0
#define UNITS_MMOL 1

#define MGDL_TO_MMOL(x) ((x) / 18.0f)

struct AppConfig {
  uint8_t  source          = SRC_MIBAND;
  uint8_t  units           = UNITS_MGDL;
  // display thresholds (colors), canonical mg/dL
  uint16_t yellowLow       = 70;
  uint16_t yellowHigh      = 180;
  uint16_t redLow          = 55;
  uint16_t redHigh         = 250;
  // sound thresholds, canonical mg/dL
  uint8_t  alarmsEnabled   = 1;
  uint16_t warnLow         = 70;
  uint16_t alarmLow        = 55;
  uint16_t warnHigh        = 180;
  uint16_t alarmHigh       = 250;
  uint16_t noReadingsMin   = 30;   // warn when data older than this
  uint8_t  warnVolume      = 30;   // 0-100
  uint8_t  alarmVolume     = 80;   // 0-100
  uint8_t  alarmRepeatMin  = 5;
  uint8_t  snoozeMin       = 30;
  // display
  uint8_t  brightness      = 100;  // 0-100
  uint8_t  rotation        = 1;    // 1 = normal, 3 = upside down
  uint8_t  timeFormat24    = 1;    // 1 = 24h, 0 = 12h am/pm
  uint8_t  dateFormatDMY   = 1;    // 1 = d.m., 0 = m/d
  // time
  int32_t  tzOffsetSec     = 0;    // local - UTC, incl. DST
  // xDrip4iOS BLE password (max 10 chars used + NUL)
  char     blePassword[16] = "";
  // Mi Band auth key received from xDrip+
  uint8_t  mibandKey[16]   = {0};
  uint8_t  mibandKeySet    = 0;

  // true after load() when no valid config was found (fresh device / factory
  // reset). Not persisted; the setup code uses it to prompt for a data source.
  bool     firstRun        = false;

  void load();
  void save();
  void factoryReset();

  float yellowLowDisp()  const { return units == UNITS_MGDL ? yellowLow  : MGDL_TO_MMOL(yellowLow); }
  bool  isMgdl()         const { return units == UNITS_MGDL; }
};

extern AppConfig cfg;

#endif
