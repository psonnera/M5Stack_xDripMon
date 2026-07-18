/*
  M5Stack xDripMon - Bluetooth-only glucose monitor
  --------------------------------------------------
  UI modeled on M5_NightscoutMon (Martin Lukasek), without any Wi-Fi.

  Data sources (selectable in the on-device menu, hold middle button):
   - xDrip (Android): the M5Stack emulates a "MI Band 2" internally; xDrip
     pushes glucose as text alerts. Enable in xDrip: Settings > Smart Watch
     Features > Mi Band > Use Mi Band, and send readings as notification.
   - xDrip4iOS / xdripswift: the M5Stack advertises the "M5Stack" protocol;
     add it in the app under Bluetooth devices (type M5Stack).

  Hardware: M5Stack Basic 2.6/2.7, Fire, Core2 (16 MB flash).
  Libraries: M5Unified, NimBLE-Arduino 2.x.

  GPL v3, based on GPL v3 sources.
*/

#include <M5Unified.h>
#include "AppConfig.h"
#include "GlucoseState.h"
#include "TimeService.h"
#include "Alarms.h"
#include "Ui.h"
#include "Menu.h"
#include "BleController.h"
#include "DebugInject.h"
#include "Log.h"
#include "Free_Fonts.h"

#define XDRIPMON_VERSION "2.0.3"

static void startupLogo() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB24);
  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
  M5.Lcd.drawString("xDrip", 160, 60);
  M5.Lcd.setTextColor(COLOR_APP_BLUE, TFT_BLACK);
  M5.Lcd.drawString("xDrip4iOS", 160, 105);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString("M5Stack monitor " XDRIPMON_VERSION, 160, 155);
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.drawString(bleModeName(), 160, 185);
  M5.Lcd.setTextDatum(TL_DATUM);
}

// First-boot / post-factory-reset data-source picker. Blocks until the user
// presses A (xDrip Android) or C (xDrip4iOS), then persists the choice.
static void chooseSourceInteractive() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString("Choose data source", 160, 30);

  M5.Lcd.setFreeFont(FSSB24);
  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
  M5.Lcd.drawString("xDrip", 160, 95);
  M5.Lcd.setTextColor(COLOR_APP_BLUE, TFT_BLACK);
  M5.Lcd.drawString("xDrip4iOS", 160, 140);

  // labels above the physical buttons (A left, C right)
  uiDrawButtonHints(nullptr, nullptr, nullptr, nullptr);  // clear the row
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextDatum(BC_DATUM);
  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
  M5.Lcd.drawString("xDrip", 57, 239);
  M5.Lcd.setTextColor(COLOR_APP_BLUE, TFT_BLACK);
  M5.Lcd.drawString("xDrip4iOS", 263, 239);
  M5.Lcd.setTextDatum(TL_DATUM);

  for (;;) {
    M5.update();
    if (M5.BtnA.wasClicked()) { cfg.source = SRC_MIBAND; break; }
    if (M5.BtnC.wasClicked()) { cfg.source = SRC_XDRIP4IOS; break; }
    delay(10);
  }
  cfg.save();
  logAdd("source: %s", bleModeName());
}

void setup() {
  auto mcfg = M5.config();
  mcfg.internal_spk = true;
  M5.begin(mcfg);
  Serial.begin(115200);
  Serial.println("\n[main] M5Stack xDripMon " XDRIPMON_VERSION);

  cfg.load();
  M5.Lcd.setRotation(cfg.rotation);
  M5.Lcd.setBrightness(map(cfg.brightness, 0, 100, 0, 255));

  gs.restore();
  timeService.begin();

  if (cfg.firstRun)
    chooseSourceInteractive();

  logAdd("boot v" XDRIPMON_VERSION " (%s)", bleModeName());

  startupLogo();
  delay(1500);

  bleBegin();
  ui.begin();
  ui.requestFullRedraw();
}

void loop() {
  M5.update();

  if (menu.active) {
    menu.handleButtons();
  } else if (ui.page() == PAGE_LOG) {
    // log page: middle button opens the menu with a short press
    if (M5.BtnB.wasClicked()) {
      menu.open();
    } else if (M5.BtnA.wasClicked()) {
      ui.prevPage();
    } else if (M5.BtnC.wasClicked()) {
      ui.nextPage();
    }
  } else {
    // display pages: middle button snoozes (short press; re-press to extend)
    if (M5.BtnB.wasClicked()) {
      alarms.snooze();
      bleNotifySnoozeToPhone();
    } else if (M5.BtnA.wasClicked()) {
      ui.prevPage();
    } else if (M5.BtnC.wasClicked()) {
      ui.nextPage();
    }
  }

  debugInjectPoll();
  alarms.tick();
  ui.tick();
  bleTick();
  delay(10);
}
