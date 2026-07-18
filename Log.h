#ifndef LOG_H
#define LOG_H

#include <Arduino.h>

#define LOG_ENTRIES 14
#define LOG_LINE_LEN 44

// add a line to the on-device log page (also mirrored to Serial)
void logAdd(const char *fmt, ...);

struct LogEntry {
  uint32_t ms;                 // millis() when logged
  time_t   utc;                // wall clock if known, else 0
  char     text[LOG_LINE_LEN];
};

// newest-first access for the log page; idx 0 = latest. Returns nullptr past end.
const LogEntry *logGet(int idx);
// set when a new entry arrives; consumed by the UI
extern volatile bool logDirty;

#endif
