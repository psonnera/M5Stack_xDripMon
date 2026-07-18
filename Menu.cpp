#include "Menu.h"
#include "AppConfig.h"
#include "GlucoseState.h"
#include "TimeService.h"
#include "Alarms.h"
#include "Ui.h"
#include <M5Unified.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include "Free_Fonts.h"

Menu menu;

extern "C" {
  extern unsigned char arrow_up_icon16x16[];
  extern unsigned char arrow_down_icon16x16[];
  extern unsigned char check_icon16x16[];
  extern unsigned char plus_icon16x16[];
  extern unsigned char minus_icon16x16[];
}

// item ids per screen
enum : uint8_t {
  R_SOURCE, R_UNITS, R_THRESH, R_ALARMS, R_DISPLAY, R_TIME, R_BLUETOOTH, R_FACTORY, R_EXIT, R_COUNT,
  T_YLOW = 0, T_YHIGH, T_RLOW, T_RHIGH, T_BACK, T_COUNT,
  A_ENABLED = 0, A_WLOW, A_ALOW, A_WHIGH, A_AHIGH, A_NOREAD, A_WVOL, A_AVOL,
  A_REPEAT, A_SNOOZE, A_TESTW, A_TESTA, A_BACK, A_COUNT,
  D_BRIGHT = 0, D_ROT, D_TFMT, D_DFMT, D_BACK, D_COUNT,
  B_RESETPWD = 0, B_FORGETKEY, B_MAC, B_BACK, B_COUNT,
};

static bool rebootNeeded = false;

static void fmtGlucose(char *out, size_t len, uint16_t mgdl) {
  if (cfg.isMgdl())
    snprintf(out, len, "%u mg/dL", mgdl);
  else
    snprintf(out, len, "%.1f mmol/L", MGDL_TO_MMOL(mgdl));
}

void Menu::open() {
  active = true;
  screen = ROOT;
  cursor = 0;
  editing = false;
  ui.suspended = true;
  draw();
}

void Menu::close() {
  active = false;
  ui.suspended = false;
  cfg.save();
  if (rebootNeeded) {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setFreeFont(FSSB12);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.drawString("Restarting...", 160, 120);
    delay(800);
    ESP.restart();
  }
  ui.requestFullRedraw();
}

int Menu::itemCount() const {
  switch (screen) {
    case ROOT:        return R_COUNT;
    case THRESHOLDS:  return T_COUNT;
    case ALARMS_S:    return A_COUNT;
    case DISPLAY_S:   return D_COUNT;
    case BLUETOOTH_S: return B_COUNT;
    default:          return 0;
  }
}

const char *Menu::itemLabel(int idx) const {
  switch (screen) {
    case ROOT:
      switch (idx) {
        case R_SOURCE:    return "Data source";
        case R_UNITS:     return "Units";
        case R_THRESH:    return "Color thresholds";
        case R_ALARMS:    return "Alarms";
        case R_DISPLAY:   return "Display";
        case R_TIME:      return "Set time";
        case R_BLUETOOTH: return "Bluetooth";
        case R_FACTORY:   return "Factory reset";
        case R_EXIT:      return "< Exit";
      }
      break;
    case THRESHOLDS:
      switch (idx) {
        case T_YLOW:  return "Yellow low";
        case T_YHIGH: return "Yellow high";
        case T_RLOW:  return "Red low";
        case T_RHIGH: return "Red high";
        case T_BACK:  return "< Back";
      }
      break;
    case ALARMS_S:
      switch (idx) {
        case A_ENABLED: return "Alarms enabled";
        case A_WLOW:    return "Warn low";
        case A_ALOW:    return "Alarm low";
        case A_WHIGH:   return "Warn high";
        case A_AHIGH:   return "Alarm high";
        case A_NOREAD:  return "No-data warn (min)";
        case A_WVOL:    return "Warn volume";
        case A_AVOL:    return "Alarm volume";
        case A_REPEAT:  return "Repeat (min)";
        case A_SNOOZE:  return "Snooze (min)";
        case A_TESTW:   return "Test warning";
        case A_TESTA:   return "Test alarm";
        case A_BACK:    return "< Back";
      }
      break;
    case DISPLAY_S:
      switch (idx) {
        case D_BRIGHT: return "Brightness";
        case D_ROT:    return "Rotation";
        case D_TFMT:   return "Time format";
        case D_DFMT:   return "Date format";
        case D_BACK:   return "< Back";
      }
      break;
    case BLUETOOTH_S:
      switch (idx) {
        case B_RESETPWD:  return "Reset iOS password";
        case B_FORGETKEY: return "Forget xDrip pairing";
        case B_MAC:       return "MAC";
        case B_BACK:      return "< Back";
      }
      break;
    default: break;
  }
  return "";
}

bool Menu::itemEditable(int idx) const {
  switch (screen) {
    case ROOT:       return idx == R_SOURCE || idx == R_UNITS;
    case THRESHOLDS: return idx != T_BACK;
    case ALARMS_S:   return idx < A_TESTW;
    case DISPLAY_S:  return idx != D_BACK;
    default:         return false;
  }
}

void Menu::valueString(int idx, char *out, size_t len) const {
  out[0] = 0;
  switch (screen) {
    case ROOT:
      if (idx == R_SOURCE)
        strlcpy(out, cfg.source == SRC_MIBAND ? "xDrip (Android)" : "xDrip4iOS", len);
      else if (idx == R_UNITS)
        strlcpy(out, cfg.isMgdl() ? "mg/dL" : "mmol/L", len);
      break;
    case THRESHOLDS:
      switch (idx) {
        case T_YLOW:  fmtGlucose(out, len, cfg.yellowLow); break;
        case T_YHIGH: fmtGlucose(out, len, cfg.yellowHigh); break;
        case T_RLOW:  fmtGlucose(out, len, cfg.redLow); break;
        case T_RHIGH: fmtGlucose(out, len, cfg.redHigh); break;
      }
      break;
    case ALARMS_S:
      switch (idx) {
        case A_ENABLED: strlcpy(out, cfg.alarmsEnabled ? "on" : "off", len); break;
        case A_WLOW:    fmtGlucose(out, len, cfg.warnLow); break;
        case A_ALOW:    fmtGlucose(out, len, cfg.alarmLow); break;
        case A_WHIGH:   fmtGlucose(out, len, cfg.warnHigh); break;
        case A_AHIGH:   fmtGlucose(out, len, cfg.alarmHigh); break;
        case A_NOREAD:  snprintf(out, len, "%u", cfg.noReadingsMin); break;
        case A_WVOL:    snprintf(out, len, "%u", cfg.warnVolume); break;
        case A_AVOL:    snprintf(out, len, "%u", cfg.alarmVolume); break;
        case A_REPEAT:  snprintf(out, len, "%u", cfg.alarmRepeatMin); break;
        case A_SNOOZE:  snprintf(out, len, "%u", cfg.snoozeMin); break;
      }
      break;
    case DISPLAY_S:
      switch (idx) {
        case D_BRIGHT: snprintf(out, len, "%u%%", cfg.brightness); break;
        case D_ROT:    strlcpy(out, cfg.rotation == 3 ? "180" : "0", len); break;
        case D_TFMT:   strlcpy(out, cfg.timeFormat24 ? "24h" : "12h", len); break;
        case D_DFMT:   strlcpy(out, cfg.dateFormatDMY ? "d.m." : "m/d", len); break;
      }
      break;
    case BLUETOOTH_S:
      if (idx == B_MAC)
        strlcpy(out, NimBLEDevice::getAddress().toString().c_str(), len);
      else if (idx == B_RESETPWD)
        strlcpy(out, cfg.blePassword[0] ? "set" : "none", len);
      else if (idx == B_FORGETKEY)
        strlcpy(out, cfg.mibandKeySet ? "paired" : "none", len);
      break;
    default: break;
  }
}

void Menu::draw() {
  if (screen == TIME_S) { drawTimeEditor(); return; }
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  const char *title = "Settings";
  switch (screen) {
    case THRESHOLDS:  title = "Color thresholds"; break;
    case ALARMS_S:    title = "Alarms"; break;
    case DISPLAY_S:   title = "Display"; break;
    case BLUETOOTH_S: title = "Bluetooth"; break;
    default: break;
  }
  M5.Lcd.drawString(title, 4, 2);
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextDatum(TR_DATUM);
  M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Lcd.drawString("hold:back", 316, 6);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.drawLine(0, 26, 320, 26, TFT_DARKGREY);

  int count = itemCount();
  int visible = 8;
  int first = 0;
  if (cursor >= visible) first = cursor - visible + 1;

  for (int row = 0; row < visible && first + row < count; row++) {
    int idx = first + row;
    int y = 30 + row * 22;
    bool sel = (idx == cursor);
    uint16_t fgLabel = sel ? TFT_BLACK : TFT_WHITE;
    uint16_t bg = sel ? (editing ? TFT_YELLOW : TFT_WHITE) : TFT_BLACK;
    M5.Lcd.fillRect(0, y, 320, 22, bg);
    M5.Lcd.setTextColor(fgLabel, bg);
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.drawString(itemLabel(idx), 6, y + 2);
    char val[32];
    valueString(idx, val, sizeof(val));
    if (val[0]) {
      M5.Lcd.setTextDatum(TR_DATUM);
      M5.Lcd.drawString(val, 314, y + 2);
      M5.Lcd.setTextDatum(TL_DATUM);
    }
  }

  if (editing)
    uiDrawButtonHints(minus_icon16x16, check_icon16x16, nullptr, plus_icon16x16);
  else
    uiDrawButtonHints(arrow_up_icon16x16, check_icon16x16, nullptr, arrow_down_icon16x16);
}

void Menu::drawTimeEditor() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.drawString("Set time", 4, 2);
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextDatum(TR_DATUM);
  M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Lcd.drawString("hold:cancel", 316, 6);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.drawLine(0, 26, 320, 26, TFT_DARKGREY);

  static const char *fields[5] = {"Year", "Month", "Day", "Hour", "Minute"};
  for (int i = 0; i < 5; i++) {
    int y = 40 + i * 30;
    bool sel = (i == teField);
    uint16_t bg = sel ? TFT_YELLOW : TFT_BLACK;
    uint16_t fg = sel ? TFT_BLACK : TFT_WHITE;
    M5.Lcd.fillRect(0, y, 320, 26, bg);
    M5.Lcd.setTextColor(fg, bg);
    M5.Lcd.setFreeFont(FSS9);
    M5.Lcd.drawString(fields[i], 6, y + 4);
    char v[8];
    snprintf(v, sizeof(v), "%02d", teVals[i]);
    M5.Lcd.setTextDatum(TR_DATUM);
    M5.Lcd.drawString(v, 314, y + 4);
    M5.Lcd.setTextDatum(TL_DATUM);
  }
  uiDrawButtonHints(minus_icon16x16, check_icon16x16, nullptr, plus_icon16x16);
}

void Menu::applyEdit(int dir) {
  // step values; thresholds step 1 mg/dL
  switch (screen) {
    case ROOT:
      if (cursor == R_SOURCE) {
        cfg.source = (cfg.source == SRC_MIBAND) ? SRC_XDRIP4IOS : SRC_MIBAND;
        rebootNeeded = true;
      } else if (cursor == R_UNITS) {
        cfg.units = cfg.isMgdl() ? UNITS_MMOL : UNITS_MGDL;
        gs.dataChanged = true;
      }
      break;
    case THRESHOLDS: {
      uint16_t *v = nullptr;
      switch (cursor) {
        case T_YLOW:  v = &cfg.yellowLow; break;
        case T_YHIGH: v = &cfg.yellowHigh; break;
        case T_RLOW:  v = &cfg.redLow; break;
        case T_RHIGH: v = &cfg.redHigh; break;
      }
      if (v) {
        int nv = (int)*v + dir;
        if (nv >= 30 && nv <= 450) *v = (uint16_t)nv;
      }
    } break;
    case ALARMS_S: {
      switch (cursor) {
        case A_ENABLED: cfg.alarmsEnabled = !cfg.alarmsEnabled; break;
        case A_WLOW:
        case A_ALOW:
        case A_WHIGH:
        case A_AHIGH: {
          uint16_t *v = cursor == A_WLOW ? &cfg.warnLow :
                        cursor == A_ALOW ? &cfg.alarmLow :
                        cursor == A_WHIGH ? &cfg.warnHigh : &cfg.alarmHigh;
          int nv = (int)*v + dir;
          if (nv >= 30 && nv <= 450) *v = (uint16_t)nv;
        } break;
        case A_NOREAD: {
          int nv = (int)cfg.noReadingsMin + dir * 5;
          if (nv >= 10 && nv <= 240) cfg.noReadingsMin = (uint16_t)nv;
        } break;
        case A_WVOL:
        case A_AVOL: {
          uint8_t *v = cursor == A_WVOL ? &cfg.warnVolume : &cfg.alarmVolume;
          int nv = (int)*v + dir * 5;
          if (nv >= 0 && nv <= 100) *v = (uint8_t)nv;
        } break;
        case A_REPEAT: {
          int nv = (int)cfg.alarmRepeatMin + dir;
          if (nv >= 1 && nv <= 60) cfg.alarmRepeatMin = (uint8_t)nv;
        } break;
        case A_SNOOZE: {
          int nv = (int)cfg.snoozeMin + dir * 5;
          if (nv >= 5 && nv <= 120) cfg.snoozeMin = (uint8_t)nv;
        } break;
      }
    } break;
    case DISPLAY_S:
      switch (cursor) {
        case D_BRIGHT: {
          int nv = (int)cfg.brightness + dir * 10;
          if (nv >= 10 && nv <= 100) {
            cfg.brightness = (uint8_t)nv;
            M5.Lcd.setBrightness(map(cfg.brightness, 0, 100, 0, 255));
          }
        } break;
        case D_ROT:
          cfg.rotation = (cfg.rotation == 3) ? 1 : 3;
          M5.Lcd.setRotation(cfg.rotation);
          break;
        case D_TFMT: cfg.timeFormat24 = !cfg.timeFormat24; break;
        case D_DFMT: cfg.dateFormatDMY = !cfg.dateFormatDMY; break;
      }
      break;
    default: break;
  }
}

void Menu::select() {
  if (screen == TIME_S) {
    if (teField < 4) {
      teField++;
    } else {
      timeService.setManual(teVals[0], teVals[1], teVals[2], teVals[3], teVals[4]);
      screen = ROOT;
      cursor = R_TIME;
    }
    draw();
    return;
  }

  if (editing) { editing = false; draw(); return; }
  if (itemEditable(cursor)) {
    if (screen == ROOT && (cursor == R_SOURCE || cursor == R_UNITS)) {
      applyEdit(1);            // simple toggles - no edit mode needed
    } else if (screen == ALARMS_S && cursor == A_ENABLED) {
      applyEdit(1);
    } else if (screen == DISPLAY_S && (cursor == D_ROT || cursor == D_TFMT || cursor == D_DFMT)) {
      applyEdit(1);
    } else {
      editing = true;
    }
    draw();
    return;
  }

  switch (screen) {
    case ROOT:
      switch (cursor) {
        case R_THRESH:    screen = THRESHOLDS; cursor = 0; break;
        case R_ALARMS:    screen = ALARMS_S; cursor = 0; break;
        case R_DISPLAY:   screen = DISPLAY_S; cursor = 0; break;
        case R_BLUETOOTH: screen = BLUETOOTH_S; cursor = 0; break;
        case R_TIME: {
          struct tm t;
          if (timeService.getLocalTm(t)) {
            teVals[0] = t.tm_year + 1900; teVals[1] = t.tm_mon + 1;
            teVals[2] = t.tm_mday; teVals[3] = t.tm_hour; teVals[4] = t.tm_min;
          } else {
            teVals[0] = 2026; teVals[1] = 1; teVals[2] = 1; teVals[3] = 12; teVals[4] = 0;
          }
          teField = 0;
          screen = TIME_S;
        } break;
        case R_FACTORY:
          cfg.factoryReset();
          {
            Preferences p;
            p.begin("xdriphist", false);
            p.clear();
            p.end();
          }
          rebootNeeded = true;
          close();
          return;
        case R_EXIT:
          close();
          return;
      }
      break;
    case THRESHOLDS:
      if (cursor == T_BACK) { screen = ROOT; cursor = R_THRESH; }
      break;
    case ALARMS_S:
      if (cursor == A_TESTW) { alarms.clearSnooze(); /* audible check */
        M5.Speaker.setVolume(map(cfg.warnVolume, 0, 100, 0, 255));
        for (int j = 0; j < 3; j++) { M5.Speaker.tone(3000, 100); delay(400); }
      } else if (cursor == A_TESTA) {
        M5.Speaker.setVolume(map(cfg.alarmVolume, 0, 100, 0, 255));
        for (int j = 0; j < 2; j++) { M5.Speaker.tone(660, 400); delay(600); }
      } else if (cursor == A_BACK) { screen = ROOT; cursor = R_ALARMS; }
      break;
    case DISPLAY_S:
      if (cursor == D_BACK) { screen = ROOT; cursor = R_DISPLAY; }
      break;
    case BLUETOOTH_S:
      if (cursor == B_RESETPWD) {
        cfg.blePassword[0] = 0;
        cfg.save();
      } else if (cursor == B_FORGETKEY) {
        cfg.mibandKeySet = 0;
        memset(cfg.mibandKey, 0, sizeof(cfg.mibandKey));
        cfg.save();
      } else if (cursor == B_BACK) { screen = ROOT; cursor = R_BLUETOOTH; }
      break;
    default: break;
  }
  draw();
}

void Menu::back() {
  if (screen == TIME_S) { screen = ROOT; cursor = R_TIME; draw(); return; }
  if (editing) { editing = false; draw(); return; }
  if (screen != ROOT) { screen = ROOT; cursor = 0; draw(); return; }
  close();
}

void Menu::adjust(int dir) {
  if (screen == TIME_S) {
    static const int lo[5] = {2024, 1, 1, 0, 0};
    static const int hi[5] = {2099, 12, 31, 23, 59};
    teVals[teField] += dir;
    if (teVals[teField] < lo[teField]) teVals[teField] = hi[teField];
    if (teVals[teField] > hi[teField]) teVals[teField] = lo[teField];
    draw();
    return;
  }
  if (editing) {
    applyEdit(dir);
    draw();
    return;
  }
  cursor += dir;
  int count = itemCount();
  if (cursor < 0) cursor = count - 1;
  if (cursor >= count) cursor = 0;
  draw();
}

void Menu::handleButtons() {
  bool editMode = editing || screen == TIME_S;

  if (M5.BtnB.wasHold()) { back(); return; }
  if (M5.BtnB.wasClicked()) { select(); return; }

  int dirA = editMode ? -1 : -1;    // A = up in lists, minus in edit
  int dirC = 1;
  if (M5.BtnA.wasClicked()) { adjust(dirA); return; }
  if (M5.BtnC.wasClicked()) { adjust(dirC); return; }

  // auto-repeat while holding in edit mode
  if (editMode && millis() - lastRepeatMs > 150) {
    if (M5.BtnA.pressedFor(500)) { adjust(-1); lastRepeatMs = millis(); }
    else if (M5.BtnC.pressedFor(500)) { adjust(1); lastRepeatMs = millis(); }
  }
}
