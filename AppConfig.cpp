#include "AppConfig.h"
#include <Preferences.h>

AppConfig cfg;

static const char *NVS_NS = "xdripmon";
// bump when the stored layout changes incompatibly
static const uint16_t CFG_VERSION = 1;

void AppConfig::load() {
  Preferences p;
  p.begin(NVS_NS, true);
  if (p.getUShort("ver", 0) != CFG_VERSION) {
    firstRun = true;
  } else {
    source         = p.getUChar("src", source);
    units          = p.getUChar("units", units);
    yellowLow      = p.getUShort("ylo", yellowLow);
    yellowHigh     = p.getUShort("yhi", yellowHigh);
    redLow         = p.getUShort("rlo", redLow);
    redHigh        = p.getUShort("rhi", redHigh);
    alarmsEnabled  = p.getUChar("aen", alarmsEnabled);
    warnLow        = p.getUShort("swlo", warnLow);
    alarmLow       = p.getUShort("salo", alarmLow);
    warnHigh       = p.getUShort("swhi", warnHigh);
    alarmHigh      = p.getUShort("sahi", alarmHigh);
    noReadingsMin  = p.getUShort("snor", noReadingsMin);
    warnVolume     = p.getUChar("wvol", warnVolume);
    alarmVolume    = p.getUChar("avol", alarmVolume);
    alarmRepeatMin = p.getUChar("arep", alarmRepeatMin);
    snoozeMin      = p.getUChar("snoz", snoozeMin);
    brightness     = p.getUChar("bri", brightness);
    rotation       = p.getUChar("rot", rotation);
    timeFormat24   = p.getUChar("tfmt", timeFormat24);
    dateFormatDMY  = p.getUChar("dfmt", dateFormatDMY);
    tzOffsetSec    = p.getInt("tzof", tzOffsetSec);
    p.getString("blepwd", blePassword, sizeof(blePassword));
    if (p.getBytesLength("mbkey") == sizeof(mibandKey)) {
      p.getBytes("mbkey", mibandKey, sizeof(mibandKey));
      mibandKeySet = 1;
    }
  }
  p.end();
}

void AppConfig::save() {
  Preferences p;
  p.begin(NVS_NS, false);
  p.putUShort("ver", CFG_VERSION);
  p.putUChar("src", source);
  p.putUChar("units", units);
  p.putUShort("ylo", yellowLow);
  p.putUShort("yhi", yellowHigh);
  p.putUShort("rlo", redLow);
  p.putUShort("rhi", redHigh);
  p.putUChar("aen", alarmsEnabled);
  p.putUShort("swlo", warnLow);
  p.putUShort("salo", alarmLow);
  p.putUShort("swhi", warnHigh);
  p.putUShort("sahi", alarmHigh);
  p.putUShort("snor", noReadingsMin);
  p.putUChar("wvol", warnVolume);
  p.putUChar("avol", alarmVolume);
  p.putUChar("arep", alarmRepeatMin);
  p.putUChar("snoz", snoozeMin);
  p.putUChar("bri", brightness);
  p.putUChar("rot", rotation);
  p.putUChar("tfmt", timeFormat24);
  p.putUChar("dfmt", dateFormatDMY);
  p.putInt("tzof", tzOffsetSec);
  p.putString("blepwd", blePassword);
  if (mibandKeySet)
    p.putBytes("mbkey", mibandKey, sizeof(mibandKey));
  else
    p.remove("mbkey");
  p.end();
}

void AppConfig::factoryReset() {
  Preferences p;
  p.begin(NVS_NS, false);
  p.clear();
  p.end();
}
