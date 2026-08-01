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
  Libraries: M5Unified/M5GFX (MIT), NimBLE-Arduino (Apache-2.0),
             Adafruit NeoPixel (LGPL-3.0), mbedTLS via ESP-IDF (Apache-2.0).

  Copyright (C) 2023-2026 Patrick Sonnerat
  License: GNU General Public License v3 (see LICENSE).

  Derived from and referencing other GPL v3 works:
   - M5_NightscoutMon, Copyright (C) Martin Lukasek <martin@lukasek.cz>:
     on-screen UI (main/big/clock/log pages, Ui.cpp), LED strip handling
     (LedStrip.cpp), alarm tone patterns (Alarms.cpp), and its xDrip4iOS
     fork by Johan Degraeve for the "M5Stack" BLE protocol
     (BleXdrip4iOS.cpp). The analog clock face originates from Bodmer's
     TFT_eSPI TFT_Clock example (via NightscoutMon).
   - xDrip+ (Nightscout Foundation): Mi Band 2 protocol UUIDs, auth flow and
     time message layout (BleMiBand.cpp, from watch/miband/*.java, itself
     based on Gadgetbridge), slope-arrow characters (GlucoseState.cpp) and
     the settings-QR payload format (Ui.cpp).
   - xdripswift / xDrip4iOS, Copyright (C) Johan Degraeve: counterpart of
     the M5Stack protocol opcodes (BleXdrip4iOS.cpp).
   - iot_iconset_16x16.c, Copyright (C) Artur Funk, GPL v3 (unmodified
     header kept in the file).
   - Free_Fonts.h from Bodmer's TFT_eSPI examples (via M5_NightscoutMon);
     the GFX FreeFonts derive from the GNU FreeFont project.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details: this is not a medical device;
  never rely on it for treatment decisions.
*/

#include <M5Unified.h>
#include "AppConfig.h"
#include "GlucoseState.h"
#include "TimeService.h"
#include "Alarms.h"
#include "LedStrip.h"
#include "Ui.h"
#include "Menu.h"
#include "BleController.h"
#include "DebugInject.h"
#include "Log.h"
#include "Free_Fonts.h"

#define XDRIPMON_VERSION "2.0.10"

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
  leds.begin();

  gs.restore();
  timeService.begin();

  bool firstRun = cfg.firstRun;
  if (cfg.firstRun)
    chooseSourceInteractive();

  logAdd("boot v" XDRIPMON_VERSION " (%s)", bleModeName());

  startupLogo();
  if (cfg.ledMode != LED_MODE_OFF)
    leds.bootAnimation();   // strip self-test (~1.8 s) replaces the logo pause
  else
    delay(1500);

  bleBegin();
  // first-time xDrip (Android) setup: show the pairing QR so xDrip+ can enable
  // Mi Band and clear any stale device MAC (BLE is up so it can auto-dismiss)
  if (firstRun && cfg.source == SRC_MIBAND)
    showXdripSetupQr();
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
  leds.tick();
  ui.tick();
  bleTick();
  delay(10);
}
