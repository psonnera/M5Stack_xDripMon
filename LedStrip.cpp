#include "LedStrip.h"
#include "AppConfig.h"
#include "GlucoseState.h"
#include "Alarms.h"
#include <M5Unified.h>
#include <Adafruit_NeoPixel.h>

LedStrip leds;

// module-private: no other file touches the pixel object
static Adafruit_NeoPixel pixels(10, LED_PIN_INTERNAL, NEO_GRB + NEO_KHZ800);

// On Basic/Fire Port A is G21/G22; on Core2 it is G32/G33 (G21/G22 drive the
// AXP192 there). G21 is shared with the internal I2C bus (IP5306 battery
// gauge) on Basic/Fire, but strip writes and I2C polls both run sequentially
// from loop(), so they never overlap - same trade-off UiFlow makes for RGB
// units on Port A.
uint8_t ledPortAPin() {
  return M5.getBoard() == m5::board_t::board_M5StackCore2 ? LED_PIN_PORT_A_C2
                                                          : LED_PIN_PORT_A;
}

// GPIO15 is the Fire's LED bar, but the LCD D/C line on Core2 - never drive
// it there. External port pins (26/17) are safe on every core.
bool LedStrip::hwOk() const {
  return cfg.ledPin != LED_PIN_INTERNAL ||
         M5.getBoard() == m5::board_t::board_M5Stack;
}

uint32_t LedStrip::scaled(uint8_t r, uint8_t g, uint8_t b) const {
  return Adafruit_NeoPixel::Color((uint16_t)r * cfg.ledBright / 100,
                                  (uint16_t)g * cfg.ledBright / 100,
                                  (uint16_t)b * cfg.ledBright / 100);
}

void LedStrip::fillShow(uint32_t c) {
  if (c == 0)
    pixels.clear();
  else
    pixels.fill(c);
  pixels.show();
}

void LedStrip::begin() {
  // Blank the Fire's internal bar even with the strip off, so power-up noise
  // never leaves stray LEDs lit (NightscoutMon does the same).
  if (M5.getBoard() == m5::board_t::board_M5Stack) {
    pixels.begin();
    pixels.clear();
    pixels.show();
  }
  applyConfig();
}

void LedStrip::applyConfig() {
  if (active) {          // blank at the old pin/length so moved or shortened
    pixels.clear();      // strips don't keep stale lit LEDs
    pixels.show();
  }
  active = (cfg.ledMode != LED_MODE_OFF) && hwOk();
  if (active) {
    pixels.setPin(cfg.ledPin);
    pixels.updateLength(cfg.ledCount);
    pixels.begin();
  }
  lastColor = 0xFFFFFFFF;
}

// steady-state colour: alert overrides, then (mode 3) the display colour
uint32_t LedStrip::targetColor() const {
  if (cfg.ledMode < LED_MODE_ALERTS)
    return 0;
  AlarmState s = alarms.state();
  if (s != ALARM_NONE && !alarms.isSnoozed())
    return (s == ALARM_ALARM_LOW || s == ALARM_ALARM_HIGH)
               ? scaled(255, 0, 0) : scaled(255, 192, 0);
  if (cfg.ledMode == LED_MODE_ALWAYS) {
    // same semantics as Ui::glucoseColor()
    if (!gs.hasData) return 0;
    if (gs.minutesAgo() > 15) return scaled(64, 64, 64);     // stale
    if (gs.mgdl < cfg.redLow || gs.mgdl > cfg.redHigh) return scaled(255, 0, 0);
    if (gs.mgdl < cfg.yellowLow || gs.mgdl > cfg.yellowHigh) return scaled(255, 192, 0);
    return scaled(0, 255, 0);
  }
  return 0;
}

void LedStrip::tick() {
  if (!active) return;
  uint32_t now = millis();
  if (now - lastTickMs < 250) return;
  lastTickMs = now;
  uint32_t c = targetColor();
  if (c != lastColor) {
    fillShow(c);
    lastColor = c;
  }
}

void LedStrip::flashFill(bool isAlarm) {
  if (!active) return;
  fillShow(isAlarm ? scaled(255, 0, 0) : scaled(255, 192, 0));
  lastColor = 0xFFFFFFFF;   // let tick() restore the steady colour afterwards
}

void LedStrip::flashClear() {
  if (!active) return;
  fillShow(0);
  lastColor = 0xFFFFFFFF;
}

// 4-pixel red/amber/green/blue comet over ~1.5 s, then a short amber flash
void LedStrip::bootAnimation() {
  if (!active) return;
  if (cfg.ledCount > 3) {
    for (int i = 0; i < cfg.ledCount - 3; i++) {
      pixels.setPixelColor(i, scaled(255, 0, 0));
      pixels.setPixelColor(i + 1, scaled(255, 192, 0));
      pixels.setPixelColor(i + 2, scaled(0, 255, 0));
      pixels.setPixelColor(i + 3, scaled(0, 0, 255));
      pixels.show();
      delay(1500 / (cfg.ledCount - 3));
      pixels.setPixelColor(i, 0);
    }
  }
  fillShow(scaled(255, 192, 0));
  delay(300);
  fillShow(0);
  lastColor = 0;
}
