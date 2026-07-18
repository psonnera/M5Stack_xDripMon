#include "Alarms.h"
#include "AppConfig.h"
#include "GlucoseState.h"
#include "Log.h"
#include <M5Unified.h>

Alarms alarms;

static void play_tone(uint16_t frequency, uint32_t duration, uint8_t volume) {
  M5.Speaker.setVolume(map(volume, 0, 100, 0, 255));
  M5.Speaker.tone(frequency, duration);
  uint32_t start = millis();
  while (M5.Speaker.isPlaying() && millis() - start < duration + 200)
    delay(1);
}

void Alarms::sound(bool isAlarm) {
  if (isAlarm) {
    for (int j = 0; j < 6; j++) {           // NightscoutMon sndAlarm()
      play_tone(660, 400, cfg.alarmVolume);
      delay(200);
    }
  } else {
    for (int j = 0; j < 3; j++) {           // NightscoutMon sndWarning()
      play_tone(3000, 100, cfg.warnVolume);
      delay(300);
    }
  }
}

AlarmState Alarms::evaluate() const {
  if (!cfg.alarmsEnabled || !gs.hasData) return ALARM_NONE;
  uint16_t v = gs.mgdl;
  if (v >= 10 && v <= cfg.alarmLow)  return ALARM_ALARM_LOW;
  if (v >= 10 && v <= cfg.warnLow)   return ALARM_WARN_LOW;
  if (v >= cfg.alarmHigh)            return ALARM_ALARM_HIGH;
  if (v >= cfg.warnHigh)             return ALARM_WARN_HIGH;
  if (gs.minutesAgo() >= (int)cfg.noReadingsMin) return ALARM_WARN_NOREAD;
  return ALARM_NONE;
}

void Alarms::tick() {
  uint32_t now = millis();
  if (now - lastEvalMs < 1000) return;
  lastEvalMs = now;

  AlarmState s = evaluate();
  if (s != current) {
    static const char *names[] = {"ok", "warn low", "warn high", "no data",
                                  "ALARM LOW", "ALARM HIGH"};
    logAdd("alarm: %s", names[s]);
    current = s;
    stateChanged = true;
  }
  if (s == ALARM_NONE) return;
  if (isSnoozed()) return;

  bool repeatDue = !everSounded ||
                   (now - lastSoundMs) > (uint32_t)cfg.alarmRepeatMin * 60000UL;
  if (!repeatDue) return;

  lastSoundMs = now;
  everSounded = true;
  sound(s == ALARM_ALARM_LOW || s == ALARM_ALARM_HIGH);
}

void Alarms::snooze() {
  uint32_t now = millis();
  if (now - lastSnoozePressMs < 2000) {   // rapid re-press: extend the snooze
    snoozeMult++;
    if (snoozeMult > 4) snoozeMult = 0;   // ...then cycle back to OFF
  } else {
    snoozeMult = 1;                        // fresh snooze
  }
  lastSnoozePressMs = now;
  if (snoozeMult == 0)
    snoozeUntilMs = 0;                      // OFF
  else
    snoozeUntilMs = now + (uint32_t)snoozeMult * cfg.snoozeMin * 60000UL;
  logAdd("snooze %d min", snoozeMult * cfg.snoozeMin);
  stateChanged = true;
}

void Alarms::clearSnooze() {
  snoozeUntilMs = 0;
  snoozeMult = 0;
  stateChanged = true;
}

int Alarms::snoozeRemainingMin() const {
  uint32_t now = millis();
  if (snoozeUntilMs == 0 || (int32_t)(snoozeUntilMs - now) <= 0) return 0;
  return (int)((snoozeUntilMs - now + 59999) / 60000);
}
