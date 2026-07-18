#include "DebugInject.h"
#include "GlucoseState.h"
#include "TimeService.h"
#include "AppConfig.h"
#include <Arduino.h>

void debugInjectPoll() {
  static char line[64];
  static size_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (n < sizeof(line) - 1) line[n++] = c;
      continue;
    }
    line[n] = 0;
    n = 0;

    if (strncmp(line, "bg ", 3) == 0) {
      int mgdl = 0, angle = ARROW_HIDDEN;
      sscanf(line + 3, "%d %d", &mgdl, &angle);
      time_t now = time(nullptr);
      gs.onReading((uint16_t)mgdl, now > 1600000000 ? now : 0, angle);
      Serial.printf("[dbg] injected %d mg/dL angle %d\n", mgdl, angle);
    } else if (strcmp(line, "demo") == 0) {
      static const uint16_t curve[] =
        {110, 118, 130, 145, 160, 172, 165, 150, 132, 120};
      for (unsigned i = 0; i < sizeof(curve) / sizeof(curve[0]); i++) {
        gs.hist[gs.histCount < HIST_SIZE ? gs.histCount++ : HIST_SIZE - 1] = curve[i];
      }
      gs.onReading(114, 0, -45);
      Serial.println("[dbg] demo curve injected");
    } else if (strncmp(line, "time ", 5) == 0) {
      int h = 0, m = 0;
      sscanf(line + 5, "%d %d", &h, &m);
      timeService.setManual(2026, 1, 15, h, m);
      Serial.printf("[dbg] time set to %02d:%02d\n", h, m);
    } else if (strcmp(line, "status") == 0) {
      Serial.printf("[dbg] mgdl=%u angle=%d minAgo=%d hist=%u units=%s src=%u\n",
                    gs.mgdl, gs.arrowAngle, gs.minutesAgo(), gs.histCount,
                    cfg.isMgdl() ? "mgdl" : "mmol", cfg.source);
    }
  }
}
