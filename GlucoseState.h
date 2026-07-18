#ifndef GLUCOSESTATE_H
#define GLUCOSESTATE_H

#include <Arduino.h>
#include <time.h>

#define HIST_SIZE 48        // 4 hours at 5-minute cadence
#define ARROW_HIDDEN 180    // NightscoutMon convention

struct GlucoseState {
  uint16_t mgdl = 0;             // 0 = no data yet
  int      arrowAngle = ARROW_HIDDEN;
  time_t   readingUtc = 0;       // 0 = unknown wall-clock timestamp
  uint32_t readingMillis = 0;    // millis() when reading arrived (staleness fallback)
  bool     hasData = false;

  // history for the mini graph (sequence of mg/dL values, newest last)
  uint16_t hist[HIST_SIZE] = {0};
  uint8_t  histCount = 0;

  int16_t  deltaMgdl = 0;        // vs previous distinct reading
  bool     deltaValid = false;

  // set by BLE layer, consumed (cleared) by UI
  volatile bool dataChanged = false;

  void onReading(uint16_t newMgdl, time_t utc, int arrowAngleIn);
  void onDirectionString(const char *nsDir);   // xDrip4iOS slope names

  float sgvMmol() const { return mgdl / 18.0f; }
  // minutes since last reading; prefers wall clock, falls back to millis
  int   minutesAgo() const;
  // last 10 history values in mmol for drawMiniGraph (0 = empty slot)
  void  last10Mmol(float out[10]) const;
  void  deltaString(char *out, size_t outLen, bool asMgdl) const;

  void persist();
  void restore();
};

// map a Nightscout direction name ("Flat", "SingleUp", ...) to an arrow angle
int nsDirectionToAngle(const char *dir);
// map an xDrip slope-arrow UTF-8 char to an angle
int slopeArrowToAngle(const char *utf8);

extern GlucoseState gs;

#endif
