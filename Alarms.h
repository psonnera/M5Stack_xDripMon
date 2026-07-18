#ifndef ALARMS_H
#define ALARMS_H

#include <Arduino.h>

enum AlarmState : uint8_t {
  ALARM_NONE = 0,
  ALARM_WARN_LOW,
  ALARM_WARN_HIGH,
  ALARM_WARN_NOREAD,
  ALARM_ALARM_LOW,
  ALARM_ALARM_HIGH,
};

class Alarms {
public:
  void tick();                 // evaluate + sound; call from loop (rate-limits itself)
  void snooze();               // BtnB
  void clearSnooze();
  AlarmState state() const { return current; }
  int  snoozeRemainingMin() const;
  bool isSnoozed() const { return snoozeRemainingMin() > 0; }
  // true right after the last snooze press cycled the multiplier to OFF
  bool snoozeIsOff() const { return snoozeMult == 0; }
  // set by tick when the bottom bar needs a redraw
  volatile bool stateChanged = false;

private:
  AlarmState evaluate() const;
  void sound(bool isAlarm);
  AlarmState current = ALARM_NONE;
  uint32_t lastEvalMs = 0;
  uint32_t lastSoundMs = 0;
  bool everSounded = false;
  uint32_t snoozeUntilMs = 0;
  // increasing-snooze multiplier (NightscoutMon behaviour): repeated presses
  // within 2 s step 1->2->3->4->0(off)->1...; each step = snoozeMin minutes
  int  snoozeMult = 0;
  uint32_t lastSnoozePressMs = 0;
};

extern Alarms alarms;

#endif
