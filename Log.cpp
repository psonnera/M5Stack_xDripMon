#include "Log.h"
#include <stdarg.h>
#include <time.h>

static LogEntry entries[LOG_ENTRIES];
static int count = 0;
static int head = 0;            // next write slot
volatile bool logDirty = false;

void logAdd(const char *fmt, ...) {
  LogEntry &e = entries[head];
  va_list args;
  va_start(args, fmt);
  vsnprintf(e.text, sizeof(e.text), fmt, args);
  va_end(args);
  e.ms = millis();
  time_t now = time(nullptr);
  e.utc = now > 1600000000 ? now : 0;

  head = (head + 1) % LOG_ENTRIES;
  if (count < LOG_ENTRIES) count++;
  logDirty = true;

  Serial.print("[log] ");
  Serial.println(e.text);
}

const LogEntry *logGet(int idx) {
  if (idx < 0 || idx >= count) return nullptr;
  int pos = (head - 1 - idx + 2 * LOG_ENTRIES) % LOG_ENTRIES;
  return &entries[pos];
}
