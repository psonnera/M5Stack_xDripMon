#include "GlucoseState.h"
#include <Preferences.h>
#include <string.h>

GlucoseState gs;

static const char *NVS_NS = "xdriphist";

void GlucoseState::onReading(uint16_t newMgdl, time_t utc, int arrowAngleIn) {
  if (newMgdl < 10 || newMgdl > 600) return;   // implausible, ignore

  uint32_t nowMs = millis();
  // de-duplicate: same value arriving again within 2 minutes (reconnect replays)
  if (hasData && newMgdl == mgdl && (nowMs - readingMillis) < 120000UL)
    return;

  if (hasData) {
    deltaMgdl = (int16_t)newMgdl - (int16_t)mgdl;
    // a delta only makes sense between consecutive readings (~5 min apart)
    uint32_t gapMs = nowMs - readingMillis;
    deltaValid = gapMs > 30000UL && gapMs < 450000UL;
    if (utc != 0 && readingUtc != 0) {
      long gap = (long)difftime(utc, readingUtc);
      deltaValid = gap > 30 && gap < 450;
    }
  }

  mgdl = newMgdl;
  readingUtc = utc;
  readingMillis = nowMs;
  if (arrowAngleIn != INT32_MIN)
    arrowAngle = arrowAngleIn;
  hasData = true;

  if (histCount < HIST_SIZE) {
    hist[histCount++] = newMgdl;
  } else {
    memmove(hist, hist + 1, (HIST_SIZE - 1) * sizeof(hist[0]));
    hist[HIST_SIZE - 1] = newMgdl;
  }

  persist();
  dataChanged = true;
}

void GlucoseState::onDirectionString(const char *nsDir) {
  arrowAngle = nsDirectionToAngle(nsDir);
  dataChanged = true;
}

int GlucoseState::minutesAgo() const {
  if (!hasData) return 9999;
  time_t now = time(nullptr);
  if (readingUtc != 0 && now > 1600000000) {          // wall clock known
    long dif = (long)difftime(now, readingUtc);
    if (dif < 0) dif = 0;
    return (int)((dif + 30) / 60);
  }
  return (int)(((millis() - readingMillis) / 1000 + 30) / 60);
}

void GlucoseState::last10Mmol(float out[10]) const {
  for (int i = 0; i < 10; i++) out[i] = 0;
  int n = histCount < 10 ? histCount : 10;
  for (int i = 0; i < n; i++)
    out[10 - n + i] = hist[histCount - n + i] / 18.0f;
}

void GlucoseState::deltaString(char *out, size_t outLen, bool asMgdl) const {
  if (!deltaValid) { strlcpy(out, "---", outLen); return; }
  if (asMgdl)
    snprintf(out, outLen, "%+d", deltaMgdl);
  else
    snprintf(out, outLen, "%+.1f", deltaMgdl / 18.0f);
}

void GlucoseState::persist() {
  Preferences p;
  p.begin(NVS_NS, false);
  p.putBytes("hist", hist, sizeof(hist));
  p.putUChar("cnt", histCount);
  p.putUShort("mgdl", mgdl);
  p.putLong64("utc", (int64_t)readingUtc);
  p.end();
}

void GlucoseState::restore() {
  Preferences p;
  p.begin(NVS_NS, true);
  if (p.getBytesLength("hist") == sizeof(hist)) {
    p.getBytes("hist", hist, sizeof(hist));
    histCount = p.getUChar("cnt", 0);
    uint16_t v = p.getUShort("mgdl", 0);
    time_t utc = (time_t)p.getLong64("utc", 0);
    if (v >= 10 && v <= 600) {
      mgdl = v;
      readingUtc = utc;
      readingMillis = millis();
      hasData = true;
      arrowAngle = ARROW_HIDDEN;   // trend unknown after reboot
    }
  }
  p.end();
}

int nsDirectionToAngle(const char *dir) {
  if (!dir) return ARROW_HIDDEN;
  if (strcmp(dir, "DoubleDown") == 0)    return 90;
  if (strcmp(dir, "SingleDown") == 0)    return 75;
  if (strcmp(dir, "FortyFiveDown") == 0) return 45;
  if (strcmp(dir, "Flat") == 0)          return 0;
  if (strcmp(dir, "FortyFiveUp") == 0)   return -45;
  if (strcmp(dir, "SingleUp") == 0)      return -75;
  if (strcmp(dir, "DoubleUp") == 0)      return -90;
  return ARROW_HIDDEN;                   // NONE / NOT COMPUTABLE / unknown
}

int slopeArrowToAngle(const char *s) {
  if (!s) return ARROW_HIDDEN;
  // xDrip BgReading.slopeToArrowSymbol() characters, UTF-8 encoded
  if (memcmp(s, "\xE2\x87\x88", 3) == 0) return -90;  // ⇈ DoubleUp
  if (memcmp(s, "\xE2\x86\x91", 3) == 0) return -75;  // ↑ SingleUp
  if (memcmp(s, "\xE2\x86\x97", 3) == 0) return -45;  // ↗ FortyFiveUp
  if (memcmp(s, "\xE2\x86\x92", 3) == 0) return 0;    // → Flat
  if (memcmp(s, "\xE2\x86\x98", 3) == 0) return 45;   // ↘ FortyFiveDown
  if (memcmp(s, "\xE2\x86\x93", 3) == 0) return 75;   // ↓ SingleDown
  if (memcmp(s, "\xE2\x87\x8A", 3) == 0) return 90;   // ⇊ DoubleDown
  return ARROW_HIDDEN;
}
