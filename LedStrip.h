#ifndef LEDSTRIP_H
#define LEDSTRIP_H

#include <Arduino.h>

// cfg.ledMode values
#define LED_MODE_OFF    0   // strip disabled
#define LED_MODE_SOUND  1   // flash only while alarm/warning tones play
#define LED_MODE_ALERTS 2   // steady red/amber during warnings and alarms
#define LED_MODE_ALWAYS 3   // alerts, plus the glucose colour when in range

// selectable data pins
#define LED_PIN_INTERNAL 15   // M5Stack Fire built-in 10-LED side bars
#define LED_PIN_PORT_B   26
#define LED_PIN_PORT_C   17

// External RGB LED strip (WS2812/NeoPixel) or the Fire's built-in bars,
// ported from M5_NightscoutMon. Colours follow the alarm state (red/amber)
// and, in "always on" mode, the display colour thresholds.
class LedStrip {
public:
  void begin();          // blank the Fire bar, then target the configured pin/count
  void bootAnimation();  // blocking ~1.8 s comet self-test; no-op when disabled
  void tick();           // call from loop(); rate-limited, shows only on change
  void applyConfig();    // re-target pin/count after a menu edit
  void flashFill(bool isAlarm);  // around alarm/warning tones (any mode but off)
  void flashClear();

private:
  bool hwOk() const;     // internal-bar pin is only valid on the Core1 family
  uint32_t scaled(uint8_t r, uint8_t g, uint8_t b) const;  // * ledBright / 100
  uint32_t targetColor() const;
  void fillShow(uint32_t c);

  bool active = false;
  uint32_t lastColor = 0xFFFFFFFF;   // sentinel forces the first show
  uint32_t lastTickMs = 0;
};

extern LedStrip leds;

#endif
