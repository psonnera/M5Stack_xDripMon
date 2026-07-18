#include "TimeService.h"
#include "AppConfig.h"
#include "Log.h"
#include <M5Unified.h>
#include <sys/time.h>

TimeService timeService;

// interpret tm fields as UTC and return the epoch, independent of the TZ env
static time_t epochFromUtcTm(struct tm &t) {
  char *oldTz = getenv("TZ") ? strdup(getenv("TZ")) : nullptr;
  setenv("TZ", "UTC0", 1); tzset();
  time_t epoch = mktime(&t);
  if (oldTz) { setenv("TZ", oldTz, 1); free(oldTz); } else unsetenv("TZ");
  tzset();
  return epoch;
}

void TimeService::applyTz(int32_t tzSec) {
  // POSIX TZ sign is inverted: UTC+2 -> "LT-2"
  char tz[24];
  int32_t off = -tzSec;
  int h = off / 3600;
  int m = abs((int)(off % 3600)) / 60;
  if (m)
    snprintf(tz, sizeof(tz), "LT%+d:%02d", h, m);
  else
    snprintf(tz, sizeof(tz), "LT%+d", h);
  setenv("TZ", tz, 1);
  tzset();
  cfg.tzOffsetSec = tzSec;
}

void TimeService::writeRtc(time_t utc) {
  if (!M5.Rtc.isEnabled()) return;
  struct tm t;
  gmtime_r(&utc, &t);
  m5::rtc_datetime_t dt;
  dt.date.year    = t.tm_year + 1900;
  dt.date.month   = t.tm_mon + 1;
  dt.date.date    = t.tm_mday;
  dt.date.weekDay = t.tm_wday;
  dt.time.hours   = t.tm_hour;
  dt.time.minutes = t.tm_min;
  dt.time.seconds = t.tm_sec;
  M5.Rtc.setDateTime(dt);
}

void TimeService::begin() {
  applyTz(cfg.tzOffsetSec);
  if (M5.Rtc.isEnabled()) {
    m5::rtc_datetime_t dt = M5.Rtc.getDateTime();
    if (dt.date.year >= 2024) {                 // RTC holds a plausible date (kept in UTC)
      struct tm t = {};
      t.tm_year = dt.date.year - 1900;
      t.tm_mon  = dt.date.month - 1;
      t.tm_mday = dt.date.date;
      t.tm_hour = dt.time.hours;
      t.tm_min  = dt.time.minutes;
      t.tm_sec  = dt.time.seconds;
      time_t utc = epochFromUtcTm(t);
      struct timeval tv = { .tv_sec = utc, .tv_usec = 0 };
      settimeofday(&tv, nullptr);
      timeKnown = true;
      Serial.println("[time] restored from RTC");
    }
  }
}

void TimeService::setFromUtc(time_t utc, int32_t tzSec) {
  applyTz(tzSec);
  struct timeval tv = { .tv_sec = utc, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  writeRtc(utc);
  cfg.save();
  timeKnown = true;
  logAdd("time set (tz %+ldh)", (long)tzSec / 3600);
}

void TimeService::setFromLocal(time_t localEpoch, int32_t tzSec) {
  setFromUtc(localEpoch - tzSec, tzSec);
}

void TimeService::setManual(int year, int month, int day, int hour, int minute) {
  // Entered value is local wall time; convert to UTC using the stored offset.
  struct tm t = {};
  t.tm_year = year - 1900;
  t.tm_mon  = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min  = minute;
  time_t asIfUtc = epochFromUtcTm(t);
  setFromUtc(asIfUtc - cfg.tzOffsetSec, cfg.tzOffsetSec);
}

bool TimeService::getLocalTm(struct tm &out) {
  if (!timeKnown) return false;
  time_t now = time(nullptr);
  localtime_r(&now, &out);
  return true;
}
