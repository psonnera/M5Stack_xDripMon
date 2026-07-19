#include "Ui.h"
#include "AppConfig.h"
#include "GlucoseState.h"
#include "TimeService.h"
#include "Alarms.h"
#include "BleController.h"
#include <M5Unified.h>
#include "Free_Fonts.h"

Ui ui;

// icons from iot_iconset_16x16.c
extern "C" {
  extern unsigned char bat0_icon16x16[];
  extern unsigned char bat1_icon16x16[];
  extern unsigned char bat2_icon16x16[];
  extern unsigned char bat3_icon16x16[];
  extern unsigned char plug_icon16x16[];
  extern unsigned char clock_icon16x16[];
  extern unsigned char bluetooth_icon16x16[];
  extern unsigned char arrow_left_icon16x16[];
  extern unsigned char arrow_right_icon16x16[];
}

#include "Log.h"

static int icon_xpos[3] = {144, 144 + 18, 144 + 2 * 18};
static int icon_ypos[3] = {0, 0, 0};

// 16x16 monochrome icon blitter (NightscoutMon drawIcon)
static void drawIcon(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t w = 16, h = 16;
  int32_t i, j, byteWidth = (w + 7) / 8;
  M5.Lcd.fillRect(x, y, w, h, TFT_BLACK);
  for (j = 0; j < h; j++)
    for (i = 0; i < w; i++)
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7)))
        M5.Lcd.drawPixel(x + i, y + j, color);
}

// NightscoutMon trend arrow
static void drawArrow(int x, int y, int asize, int aangle, int pwidth, int plength, uint16_t color) {
  float dx = (asize - 10) * cos(aangle - 90) * PI / 180 + x;
  float dy = (asize - 10) * sin(aangle - 90) * PI / 180 + y;
  float x1 = 0;           float y1 = plength;
  float x2 = pwidth / 2;  float y2 = pwidth / 2;
  float x3 = -pwidth / 2; float y3 = pwidth / 2;
  float angle = aangle * PI / 180 - 135;
  float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
  float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
  float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
  float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
  float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
  float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
  M5.Lcd.fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, color);
  for (int o = -2; o <= 2; o++) {
    M5.Lcd.drawLine(x + o, y, xx1 + o, yy1, color);
    M5.Lcd.drawLine(x, y + o, xx1, yy1 + o, color);
  }
}

void Ui::begin() {
  M5.Lcd.fillScreen(TFT_BLACK);
  fullRedraw = true;
}

void Ui::setStatusMessage(const char *msg) {
  strlcpy(statusMsg, msg, sizeof(statusMsg));
  statusMsgMs = millis();
  alarms.stateChanged = true;    // force info line redraw
}

void Ui::nextPage() {
  dispPage = (dispPage >= MAX_PAGE) ? 0 : dispPage + 1;
  fullRedraw = true;
}

void Ui::prevPage() {
  dispPage = (dispPage <= 0) ? MAX_PAGE : dispPage - 1;
  fullRedraw = true;
}

uint16_t Ui::glucoseColor() const {
  uint16_t c = TFT_GREEN;
  if (gs.mgdl < cfg.yellowLow || gs.mgdl > cfg.yellowHigh) c = TFT_YELLOW;
  if (gs.mgdl < cfg.redLow || gs.mgdl > cfg.redHigh) c = TFT_RED;
  if (gs.minutesAgo() > 15) c = TFT_DARKGREY;      // stale data
  return c;
}

void Ui::bgString(char *out, size_t len, bool &smallerFont) const {
  smallerFont = false;
  if (!gs.hasData) { strlcpy(out, "---", len); return; }
  if (cfg.isMgdl()) {
    snprintf(out, len, "%3d", gs.mgdl);
  } else {
    snprintf(out, len, "%4.1f", gs.sgvMmol());
    if (out[0] != ' ') smallerFont = true;         // >= 10.0 mmol
  }
}

void Ui::headerDateTime(char *out, size_t len) {
  struct tm t;
  if (!timeService.getLocalTm(t)) { strlcpy(out, "--:--", len); return; }
  char timeStr[16], dateStr[16];
  if (cfg.timeFormat24) {
    strftime(timeStr, sizeof(timeStr), "%H:%M ", &t);
  } else {
    strftime(timeStr, sizeof(timeStr), "%I:%M%p", &t);
    timeStr[strlen(timeStr) - 1] = 0;              // "AM" -> "A" like reference
    strlcat(timeStr, " ", sizeof(timeStr));
  }
  if (cfg.dateFormatDMY)
    strftime(dateStr, sizeof(dateStr), "%d.%m.  ", &t);
  else
    strftime(dateStr, sizeof(dateStr), "%m/%d  ", &t);
  snprintf(out, len, "%s%s", timeStr, dateStr);
}

void Ui::drawBleIcon() {
  // steady blue = connected + authenticated
  // blinking blue = connected, not yet authenticated
  // blinking red = not connected
  if (bleIsAuthenticated()) {
    drawIcon(icon_xpos[0], icon_ypos[0], bluetooth_icon16x16, COLOR_APP_BLUE);
  } else if (blinkState) {
    uint16_t c = bleIsConnected() ? COLOR_APP_BLUE : TFT_RED;
    drawIcon(icon_xpos[0], icon_ypos[0], bluetooth_icon16x16, c);
  } else {
    M5.Lcd.fillRect(icon_xpos[0], icon_ypos[0], 16, 16, TFT_BLACK);
  }
}

void Ui::drawHeaderIcons() {
  // [0] BLE state, [1] snooze, [2] battery
  drawBleIcon();

  if (alarms.isSnoozed())
    drawIcon(icon_xpos[1], icon_ypos[1], clock_icon16x16, TFT_RED);
  else
    M5.Lcd.fillRect(icon_xpos[1], icon_ypos[1], 16, 16, TFT_BLACK);

  int x = icon_xpos[2], y = icon_ypos[2];
  int32_t bl = M5.Power.getBatteryLevel();
  bool charging = M5.Power.isCharging() == m5::Power_Class::is_charging_t::is_charging;
  M5.Lcd.fillRect(x, y, 16, 17, TFT_BLACK);
  if (charging)          drawIcon(x, y, plug_icon16x16, TFT_LIGHTGREY);
  else if (bl >= 0 && bl <= 12) drawIcon(x, y + 1, bat0_icon16x16, TFT_RED);
  else if (bl <= 40)     drawIcon(x, y + 1, bat1_icon16x16, TFT_YELLOW);
  else if (bl <= 65)     drawIcon(x, y + 1, bat2_icon16x16, TFT_WHITE);
  else if (bl <= 100)    drawIcon(x, y + 1, bat3_icon16x16, TFT_LIGHTGREY);
}

void Ui::drawMiniGraph() {
  float last10[10];
  gs.last10Mmol(last10);
  M5.Lcd.drawLine(231, 113, 319, 113, TFT_LIGHTGREY);
  M5.Lcd.drawLine(231, 203, 319, 203, TFT_LIGHTGREY);
  M5.Lcd.drawLine(231, 193, 319, 193, TFT_LIGHTGREY);   // 4 mmol guide
  M5.Lcd.drawLine(231, 143, 319, 143, TFT_LIGHTGREY);   // 9 mmol guide
  float yl = MGDL_TO_MMOL(cfg.yellowLow), yh = MGDL_TO_MMOL(cfg.yellowHigh);
  float rl = MGDL_TO_MMOL(cfg.redLow),   rh = MGDL_TO_MMOL(cfg.redHigh);
  for (int i = 9; i >= 0; i--) {
    float glk = last10[i];
    if (glk == 0) continue;
    uint16_t sgvColor = TFT_GREEN;
    if (glk < yl || glk > yh) sgvColor = TFT_YELLOW;
    if (glk < rl || glk > rh) sgvColor = TFT_RED;
    float g = glk;
    if (g > 12) g = 12;
    if (g < 3)  g = 3;
    M5.Lcd.fillCircle(234 + i * 9, 203 - (g - 3.0f) * 10.0f, 3, sgvColor);
  }
}

void uiDrawButtonHints(const unsigned char *iconA, const unsigned char *iconB,
                       const char *textB, const unsigned char *iconC) {
  // physical button x centers: A=57, B=160, C=263
  M5.Lcd.fillRect(0, 220, 320, 20, TFT_BLACK);
  if (iconA) drawIcon(57 - 8, 222, iconA, TFT_DARKGREY);
  if (iconB) drawIcon(160 - 8, 222, iconB, TFT_DARKGREY);
  if (textB) {
    M5.Lcd.setTextSize(1);
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.setTextDatum(BC_DATUM);
    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.drawString(textB, 160, 239);
    M5.Lcd.setTextDatum(TL_DATUM);
  }
  if (iconC) drawIcon(263 - 8, 222, iconC, TFT_DARKGREY);
}

// xDrip+ "load settings from QR" payload (marker xdpref: + base64(zlib(json))).
// JSON: {"miband_enabled":"true","miband_data_mac":"",
//        "miband_send_readings_as_notification":"true"}
// Enables Mi Band, clears any stored device MAC, and forces readings-as-
// notification so xDrip+ pairs cleanly with this device.
static const char *XDRIP_SETUP_QR =
  "xdpref:eNqrVsrNTErMS4lPzUtMyklNUbJSKikqTVXSgYmnJJYkxucmJgMlEILFqUCiKDUxJTMvvTg+sTg+L78kMy0zObEkMz8PZkQtAPdMIVk=";

// Full-screen QR the user scans in xDrip+ (Settings -> Scan QR to load
// settings). Stays up until button C is pressed or xDrip connects over BLE.
void showXdripSetupQr() {
  // dim to 10% while the QR is shown (reduces glare so it scans reliably),
  // restored to the user's brightness on exit
  M5.Lcd.setBrightness(map(10, 0, 100, 0, 255));

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.drawString("xDrip setup", 160, 12);

  // white quiet zone behind the QR so scanners lock on reliably
  const int qrW = 150;
  const int qrX = (320 - qrW) / 2;
  const int qrY = 28;
  M5.Lcd.fillRect(qrX - 8, qrY - 8, qrW + 16, qrW + 16, TFT_WHITE);
  M5.Lcd.qrcode(XDRIP_SETUP_QR, qrX, qrY, qrW, 6);

  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString("In xDrip+: Settings", 160, qrY + qrW + 14);
  M5.Lcd.drawString("Scan QR to load settings", 160, qrY + qrW + 32);
  M5.Lcd.setTextDatum(TL_DATUM);

  uiDrawButtonHints(nullptr, nullptr, nullptr, arrow_right_icon16x16);

  for (;;) {
    M5.update();
    bleTick();                     // keep advertising alive while shown
    if (M5.BtnC.wasClicked()) break;
    if (bleIsConnected()) break;   // xDrip scanned the QR and connected
    delay(10);
  }

  M5.Lcd.setBrightness(map(cfg.brightness, 0, 100, 0, 255));  // restore
}

void Ui::drawAlarmInfoLine() {
  char tmpStr[12];
  int snoozeRem = alarms.snoozeRemainingMin();
  if (snoozeRem > 0)
    snprintf(tmpStr, sizeof(tmpStr), "%d", snoozeRem);
  else
    strlcpy(tmpStr, "Snooze", sizeof(tmpStr));

  M5.Lcd.setTextDatum(BL_DATUM);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB12);

  AlarmState s = alarms.state();
  bool isAlarm = (s == ALARM_ALARM_LOW || s == ALARM_ALARM_HIGH);
  bool isWarn  = (s == ALARM_WARN_LOW || s == ALARM_WARN_HIGH || s == ALARM_WARN_NOREAD);

  if (isAlarm || isWarn) {
    uint16_t bg = isAlarm ? TFT_RED : TFT_YELLOW;
    M5.Lcd.fillRect(0, 220, 320, 20, bg);
    M5.Lcd.setTextColor(TFT_BLACK, bg);
    int stw = M5.Lcd.textWidth(tmpStr);
    M5.Lcd.drawString(tmpStr, 159 - stw / 2, 240);
    if (s == ALARM_WARN_NOREAD)
      M5.Lcd.drawString("NO DATA", 2, 240);
    M5.Lcd.setTextDatum(TL_DATUM);
  } else if (statusMsg[0] && millis() - statusMsgMs < 30000UL) {
    // transient status/alert message, centered
    M5.Lcd.fillRect(0, 220, 320, 20, TFT_BLACK);
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.setTextDatum(BC_DATUM);
    M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Lcd.drawString(statusMsg, 160, 239);
    M5.Lcd.setTextDatum(TL_DATUM);
  } else {
    // middle-button hint: on the log page it opens the menu; on the display
    // pages it snoozes, and while a snooze is active it shows the remaining
    // minutes (grows as the button is re-pressed).
    char midBuf[16];
    const char *midHint;
    int snz = alarms.snoozeRemainingMin();
    if (dispPage == PAGE_LOG) {
      midHint = "Menu";
    } else if (snz > 0) {
      snprintf(midBuf, sizeof(midBuf), "Snooze %d", snz);
      midHint = midBuf;
    } else {
      midHint = "Snooze";
    }
    uiDrawButtonHints(arrow_left_icon16x16, nullptr, midHint,
                      arrow_right_icon16x16);
  }
}

void Ui::drawPage0() {
  char tmpstr[64];

  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  char datetimeStr[40];
  headerDateTime(datetimeStr, sizeof(datetimeStr));
  M5.Lcd.fillRect(0, 0, 140, 24, TFT_BLACK);
  M5.Lcd.drawString(datetimeStr, 0, 0);

  drawHeaderIcons();

  // data source label (userName slot in NightscoutMon)
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString(bleModeName(), 0, 24);

  // big delta
  char deltaStr[16];
  gs.deltaString(deltaStr, sizeof(deltaStr), cfg.isMgdl());
  M5.Lcd.setFreeFont(FSSB24);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.fillRect(0, 58, 199, 47, TFT_BLACK);
  M5.Lcd.drawString(deltaStr, 0, 58);
  M5.Lcd.setFreeFont(FSSB12);

  // minutes-ago rounded box
  int sensorDifMin = gs.minutesAgo();
  uint16_t tdColor = TFT_LIGHTGREY;
  if (sensorDifMin > 5)  tdColor = TFT_WHITE;
  if (sensorDifMin > 15) tdColor = TFT_RED;
  M5.Lcd.fillRoundRect(200, 0, 120, 90, 15, tdColor);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB24);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextColor(TFT_BLACK, tdColor);
  if (!gs.hasData)
    M5.Lcd.drawString("---", 260, 32);
  else if (sensorDifMin > 99)
    M5.Lcd.drawString("Err", 260, 32);
  else
    M5.Lcd.drawNumber(sensorDifMin, 260, 32);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.drawString("min", 260, 70);

  // big glucose + arrow
  uint16_t glColor = glucoseColor();
  M5.Lcd.fillRect(0, 110, 320, 110, TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextColor(glColor, TFT_BLACK);
  char sensSgvStr[16];
  bool smallerFont;
  bgString(sensSgvStr, sizeof(sensSgvStr), smallerFont);
  if (smallerFont) {
    M5.Lcd.setFreeFont(FSSB18);
    M5.Lcd.drawString(sensSgvStr, 0, 130);
  } else {
    M5.Lcd.setFreeFont(FSSB24);
    M5.Lcd.drawString(sensSgvStr, 0, 120);
  }
  int tw = M5.Lcd.textWidth(sensSgvStr);
  M5.Lcd.setTextSize(1);
  if (gs.hasData && gs.arrowAngle != ARROW_HIDDEN)
    drawArrow(tw + 25, 160, 10, gs.arrowAngle + 85, 40, 40, glColor);

  drawMiniGraph();
  drawAlarmInfoLine();
}

void Ui::drawPage1() {
  uint16_t glColor = glucoseColor();

  M5.Lcd.fillRect(0, 40, 320, 180, TFT_BLACK);
  M5.Lcd.setTextSize(4);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextColor(glColor, TFT_BLACK);
  char sensSgvStr[16];
  // MC_DATUM centres the full font cell; digits have no descender, shift down
  int numYpos = 144;
  if (!gs.hasData) {
    strlcpy(sensSgvStr, "---", sizeof(sensSgvStr));
    M5.Lcd.setFreeFont(FSSB24);
  } else if (cfg.isMgdl()) {
    snprintf(sensSgvStr, sizeof(sensSgvStr), "%d", gs.mgdl);
    M5.Lcd.setFreeFont(FSSB24);
  } else {
    if (gs.sgvMmol() < 10) {
      snprintf(sensSgvStr, sizeof(sensSgvStr), "%3.1f", gs.sgvMmol());
      M5.Lcd.setFreeFont(FSSB24);
    } else {
      snprintf(sensSgvStr, sizeof(sensSgvStr), "%4.1f", gs.sgvMmol());
      M5.Lcd.setFreeFont(FSSB18);
      numYpos = 136;
    }
  }
  M5.Lcd.drawString(sensSgvStr, 160, numYpos);

  M5.Lcd.fillRect(0, 0, 320, 40, TFT_BLACK);
  M5.Lcd.setFreeFont(FSSB24);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  char datetimeStr[16];
  struct tm t;
  if (timeService.getLocalTm(t))
    snprintf(datetimeStr, sizeof(datetimeStr), "%02d:%02d", t.tm_hour, t.tm_min);
  else
    strlcpy(datetimeStr, "--:--", sizeof(datetimeStr));
  M5.Lcd.drawString(datetimeStr, 0, 0);

  char deltaStr[16];
  gs.deltaString(deltaStr, sizeof(deltaStr), cfg.isMgdl());
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString(deltaStr, 180, 0);

  if (gs.hasData && gs.arrowAngle != ARROW_HIDDEN) {
    int ay;
    if (gs.arrowAngle >= 45)      ay = 4;
    else if (gs.arrowAngle > -45) ay = 18;
    else                          ay = 30;
    drawArrow(280, ay, 10, gs.arrowAngle + 85, 28, 28, glColor);
  }

  drawAlarmInfoLine();
  drawHeaderIcons();
}

void Ui::drawClockFace() {
  uint16_t glColor = glucoseColor();

  // top row: BG left, delta right, min-ago box below
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextColor(glColor, TFT_BLACK);
  char sensSgvStr[16];
  bool smallerFont;
  bgString(sensSgvStr, sizeof(sensSgvStr), smallerFont);
  M5.Lcd.setFreeFont(FSSB24);
  M5.Lcd.fillRect(0, 0, 100, 40, TFT_BLACK);
  M5.Lcd.drawString(sensSgvStr, 0, 0);

  M5.Lcd.setTextDatum(TR_DATUM);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.fillRect(220, 0, 100, 40, TFT_BLACK);
  char deltaStr[16];
  gs.deltaString(deltaStr, sizeof(deltaStr), cfg.isMgdl());
  M5.Lcd.drawString(deltaStr, 319, 0);

  int sensorDifMin = gs.minutesAgo();
  uint16_t tdColor = TFT_LIGHTGREY;
  if (sensorDifMin > 5)  tdColor = TFT_WHITE;
  if (sensorDifMin > 15) tdColor = TFT_RED;
  M5.Lcd.fillRoundRect(0, 44, 68, 22, 7, tdColor);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextColor(TFT_BLACK, tdColor);
  if (sensorDifMin > 99)
    M5.Lcd.drawString("Err min", 34, 53);
  else
    M5.Lcd.drawString(String(sensorDifMin) + " min", 34, 53);

  struct tm t;
  if (!timeService.getLocalTm(t)) {
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setFreeFont(FSSB12);
    M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Lcd.drawString("No time available", 160, 110);
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.drawString("set in menu (hold middle button)", 160, 140);
    M5.Lcd.setTextDatum(TL_DATUM);
    drawAlarmInfoLine();
    return;
  }

  // clock face
  M5.Lcd.fillCircle(160, 110, 98, glColor);
  M5.Lcd.fillCircle(160, 110, 92, TFT_BLACK);
  for (int i = 0; i < 360; i += 30) {
    float sx = cos((i - 90) * 0.0174532925f);
    float sy = sin((i - 90) * 0.0174532925f);
    M5.Lcd.drawLine(sx * 94 + 160, sy * 94 + 110, sx * 80 + 160, sy * 80 + 110, glColor);
  }
  for (int i = 0; i < 360; i += 6) {
    float sx = cos((i - 90) * 0.0174532925f);
    float sy = sin((i - 90) * 0.0174532925f);
    uint16_t x0 = sx * 82 + 160, yy0 = sy * 82 + 110;
    M5.Lcd.drawPixel(x0, yy0, TFT_WHITE);
    if (i % 90 == 0) M5.Lcd.fillCircle(x0, yy0, 2, TFT_WHITE);
  }
  M5.Lcd.fillCircle(160, 110, 3, TFT_WHITE);
  clockInitial = true;
  updateClockHands(true);
  drawAlarmInfoLine();
}

void Ui::updateClockHands(bool force) {
  struct tm t;
  if (!timeService.getLocalTm(t)) return;
  if (!force && t.tm_sec == lastSec) return;
  lastSec = t.tm_sec;

  uint8_t hh = t.tm_hour, mm = t.tm_min, ss = t.tm_sec;
  float sdeg = ss * 6;
  float mdeg = mm * 6 + sdeg * 0.01666667f;
  float hdeg = hh * 30 + mdeg * 0.0833333f;
  float hx = cos((hdeg - 90) * 0.0174532925f);
  float hy = sin((hdeg - 90) * 0.0174532925f);
  float mx = cos((mdeg - 90) * 0.0174532925f);
  float my = sin((mdeg - 90) * 0.0174532925f);
  float sx = cos((sdeg - 90) * 0.0174532925f);
  float sy = sin((sdeg - 90) * 0.0174532925f);

  if (ss == 0 || clockInitial) {
    clockInitial = false;
    // erase hour and minute hands once a minute
    for (int o = -1; o <= 1; o++) {
      M5.Lcd.drawLine(ohx + o, ohy, 160 + o, 110, TFT_BLACK);
      M5.Lcd.drawLine(ohx, ohy + o, 160, 110 + o, TFT_BLACK);
    }
    ohx = hx * 52 + 160;
    ohy = hy * 52 + 110;
    M5.Lcd.drawLine(omx, omy, 160, 110, TFT_BLACK);
    omx = mx * 74 + 160;
    omy = my * 74 + 110;
  }

  // erase old second hand
  M5.Lcd.drawLine(osx, osy, 160, 110, TFT_BLACK);

  // day-of-month box
  M5.Lcd.drawRoundRect(182, 97, 36, 26, 7, TFT_LIGHTGREY);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setFreeFont(FSSB9);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.drawString(String(t.tm_mday), 200, 108);

  osx = sx * 78 + 160;
  osy = sy * 78 + 110;
  for (int o = -1; o <= 1; o++) {
    M5.Lcd.drawLine(ohx + o, ohy, 160 + o, 110, TFT_WHITE);
    M5.Lcd.drawLine(ohx, ohy + o, 160, 110 + o, TFT_WHITE);
  }
  M5.Lcd.drawLine(omx, omy, 160, 110, TFT_WHITE);
  M5.Lcd.drawLine(osx, osy, 160, 110, TFT_RED);
  M5.Lcd.fillCircle(160, 110, 3, TFT_RED);
}

void Ui::drawLogPage() {
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.fillRect(0, 0, 320, 28, TFT_BLACK);
  M5.Lcd.drawString("Log", 4, 2);
  M5.Lcd.drawLine(0, 26, 320, 26, TFT_DARKGREY);

  M5.Lcd.fillRect(0, 28, 320, 192, TFT_BLACK);
  M5.Lcd.setFreeFont(FSS9);
  for (int i = 0; i < 12; i++) {
    const LogEntry *e = logGet(i);
    if (!e) break;
    char stamp[10];
    if (e->utc) {
      struct tm t;
      localtime_r(&e->utc, &t);
      snprintf(stamp, sizeof(stamp), "%02d:%02d", t.tm_hour, t.tm_min);
    } else {
      uint32_t ageMin = (millis() - e->ms) / 60000UL;
      if (ageMin > 999) ageMin = 999;
      snprintf(stamp, sizeof(stamp), "%4lum", (unsigned long)ageMin);
    }
    int y = 30 + i * 16;
    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.drawString(stamp, 2, y);
    M5.Lcd.setTextColor(i == 0 ? TFT_WHITE : TFT_LIGHTGREY, TFT_BLACK);
    M5.Lcd.drawString(e->text, 52, y);
  }
  logDirty = false;
  drawAlarmInfoLine();
}

void Ui::drawPage() {
  switch (dispPage) {
    case PAGE_MAIN:  drawPage0(); break;
    case PAGE_BIGBG: drawPage1(); break;
    case PAGE_CLOCK: drawClockFace(); break;
    case PAGE_LOG:   drawLogPage(); break;
  }
}

void Ui::tick() {
  if (suspended) return;
  uint32_t now = millis();
  if (now - lastTickMs < 100) return;
  lastTickMs = now;

  bool redraw = fullRedraw;
  if (gs.dataChanged) { gs.dataChanged = false; redraw = true; }
  if (alarms.stateChanged) { alarms.stateChanged = false; redraw = true; }
  if (dispPage == PAGE_LOG && logDirty) redraw = true;

  int ma = gs.minutesAgo();
  if (ma != lastMinAgo) { lastMinAgo = ma; redraw = true; }

  struct tm t;
  if (timeService.getLocalTm(t) && t.tm_min != lastMinute) {
    lastMinute = t.tm_min;
    redraw = true;
  }

  if (redraw) {
    if (fullRedraw) {
      M5.Lcd.fillScreen(TFT_BLACK);
      fullRedraw = false;
    }
    drawPage();
  } else if (dispPage == PAGE_CLOCK) {
    updateClockHands(false);
  }

  // Bluetooth icon blinks until authenticated (red = not connected,
  // blue = connected but not yet authenticated); steady once authenticated.
  if ((dispPage == PAGE_MAIN || dispPage == PAGE_BIGBG) &&
      !bleIsAuthenticated() && now - lastBlinkMs >= 700) {
    lastBlinkMs = now;
    blinkState = !blinkState;
    drawBleIcon();
  }
}
