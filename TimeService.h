#ifndef TIMESERVICE_H
#define TIMESERVICE_H

#include <Arduino.h>
#include <time.h>

class TimeService {
public:
  void begin();                                   // restore from RTC (Core2) + stored tz
  void setFromUtc(time_t utc, int32_t tzSec);     // e.g. xDrip4iOS 0x12/0x14
  void setFromLocal(time_t localEpoch, int32_t tzSec); // local wall time known, tz optional
  void setManual(int year, int month, int day, int hour, int minute); // menu entry
  bool known() const { return timeKnown; }
  bool getLocalTm(struct tm &out);                // false if time unknown

private:
  void applyTz(int32_t tzSec);
  void writeRtc(time_t utc);
  bool timeKnown = false;
};

extern TimeService timeService;

#endif
