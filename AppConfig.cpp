#include "AppConfig.h"
#include "Log.h"
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
    ledMode        = p.getUChar("lmode", ledMode);
    ledPin         = p.getUChar("lpin", ledPin);
    ledCount       = p.getUChar("lcnt", ledCount);
    ledBright      = p.getUChar("lbri", ledBright);
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
  // put* returns 0 on failure (e.g. NVS out of space); a silently dropped
  // write looks like "settings don't save" to the user, so surface it
  int failed = 0;
  auto chk = [&failed](size_t written) { if (written == 0) failed++; };
  chk(p.putUShort("ver", CFG_VERSION));
  chk(p.putUChar("src", source));
  chk(p.putUChar("units", units));
  chk(p.putUShort("ylo", yellowLow));
  chk(p.putUShort("yhi", yellowHigh));
  chk(p.putUShort("rlo", redLow));
  chk(p.putUShort("rhi", redHigh));
  chk(p.putUChar("aen", alarmsEnabled));
  chk(p.putUShort("swlo", warnLow));
  chk(p.putUShort("salo", alarmLow));
  chk(p.putUShort("swhi", warnHigh));
  chk(p.putUShort("sahi", alarmHigh));
  chk(p.putUShort("snor", noReadingsMin));
  chk(p.putUChar("wvol", warnVolume));
  chk(p.putUChar("avol", alarmVolume));
  chk(p.putUChar("arep", alarmRepeatMin));
  chk(p.putUChar("snoz", snoozeMin));
  chk(p.putUChar("bri", brightness));
  chk(p.putUChar("rot", rotation));
  chk(p.putUChar("tfmt", timeFormat24));
  chk(p.putUChar("dfmt", dateFormatDMY));
  chk(p.putUChar("lmode", ledMode));
  chk(p.putUChar("lpin", ledPin));
  chk(p.putUChar("lcnt", ledCount));
  chk(p.putUChar("lbri", ledBright));
  chk(p.putInt("tzof", tzOffsetSec));
  if (blePassword[0])                  // putString returns 0 for "" even on success
    chk(p.putString("blepwd", blePassword));
  else
    p.putString("blepwd", blePassword);
  if (mibandKeySet)
    chk(p.putBytes("mbkey", mibandKey, sizeof(mibandKey)));
  else
    p.remove("mbkey");
  p.end();
  if (failed)
    logAdd("config save FAILED (%d keys)", failed);
}

void AppConfig::factoryReset() {
  Preferences p;
  p.begin(NVS_NS, false);
  p.clear();
  p.end();
  // Reset the in-RAM copy too; otherwise a later save() (e.g. in Menu::close)
  // would re-persist the stale values over the just-cleared NVS. Leaving NVS
  // empty is what makes load() report firstRun on the next boot.
  *this = AppConfig{};
}
