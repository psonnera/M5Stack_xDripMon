# M5Stack xDripMon

A Bluetooth-only glucose monitor for M5Stack — no Wi-Fi, no web server, no
cloud. The phone connects directly to the device over BLE and pushes each
reading.

<img width="300" src="https://raw.githubusercontent.com/psonnera/M5Stack_xDripMon/refs/heads/main/images/M5Stack_xDrip.jpg">

## Features

- Interface modeled on [M5_NightscoutMon](https://github.com/mlukasek/M5_NightscoutMon):
  main page with big value, trend arrow and mini-graph; big-glucose page;
  analog clock page; log page.
- High/low alarms with yellow/red thresholds, warning and alarm sounds,
  snooze on the middle button.
- Fully configurable on the device (hold the middle button for the menu) —
  no web configuration needed.
- Bluetooth icon shows the link state: blinking red = not connected,
  blinking blue = connecting/authenticating, steady blue = authenticated.
- Optional RGB LED strip (or the Fire's built-in LED bars): flash with the
  alarm sounds, light up on warnings/alarms, or follow the glucose colour
  permanently (see *LED strip* below).

## Data sources

Chosen on first boot (button **A** = xDrip Android, **C** = xDrip4iOS) and
changeable any time in the menu:

- **xDrip (Android)** — in xDrip enable:
  `Settings > Smart Watch Features > Mi Band > Use Mi Band` (leave the auth
  key empty) and enable sending the reading as a notification. xDrip finds
  the device, authenticates and pushes each reading. This mode has **no time
  sync** — set the clock in the device menu (see *Time & clock* below).
- **xDrip4iOS / xdripswift (iOS)** — add a Bluetooth device of type
  **M5Stack** in the app; pairing, readings, trend and time are set
  automatically (the app pushes the current time on connect).

## Time & clock

Whether the clock survives a reset depends on the core and the data source:

- **Core2** — has a battery-backed RTC, so the clock is kept across resets and
  power cycles in either mode.
- **Basic / Fire** — no RTC, so the clock is lost on every reset. Recovery
  depends on the data source:
  - **xDrip4iOS** — the time re-syncs automatically once the phone reconnects.
  - **xDrip (Android)** — no time sync at all; re-enter the clock in the device
    menu after each reset.

## Hardware

16 MB-flash ESP32 M5Stack cores: **Basic 2.6/2.7, Fire, Core2** — one binary
for all (`Binaries/ESP32_16MB`, layout compatible with M5_NightscoutMon
flashing tools). See `TESTING.md` for the esptool command and test guide.

## LED strip (optional)

An external WS2812/NeoPixel strip — or the Fire's built-in 10-LED side
bars — can mirror the glucose state. Everything is configured on the device
in *Menu → LED strip*:

- **Mode** — `off` (default), `sound` (flash red/amber in sync with the
  alarm and warning tones), `alerts` (steady red/amber while a warning or
  alarm is active, dark otherwise; snooze turns it off), `always on`
  (alerts, plus the glucose colour when in range: green / amber / red
  following the colour thresholds, dim grey when the data is stale).
- **Pin** — `15 internal` (Fire's built-in bars; Fire only), `26 Port B`
  or `17 Port C` for an external strip. Port A is not offered — it is the
  I2C bus (GPIO 21/22).
- **LED count** — 10 for the Fire bars; typical strips: 15 = 10 cm,
  29 = 20 cm, 72 = 50 cm, 144 = 100 cm.
- **Brightness** — 5–100 %. Keep 5–10 % for more than 10 LEDs or power the
  strip externally: at high brightness the load current can brown out the
  M5Stack.
- **Test** — plays the boot animation (red/amber/green/blue sweep, also
  shown at power-on whenever the mode is not off).

Wiring an external strip: GND and 5 V from the Grove port, data to the
port's signal pin (Port B = GPIO 26, Port C = GPIO 17).

## Building

Arduino IDE or arduino-cli with:
- board package **M5Stack 2.1.1** (targets M5Core / M5Fire / M5Core2)
- libraries **M5Unified** (with M5GFX), **NimBLE-Arduino 2.x** and
  **Adafruit NeoPixel**

## History

v1 emulated a LeFun band and was abandoned due to Bluedroid Bluetooth-stack
issues (readings only every 10 minutes). v2 is a complete rewrite on NimBLE
with a different xDrip transport. (The v1 sources remain in git history.)

## License

GPL v3. UI ported from M5_NightscoutMon (Martin Lukasek, GPL v3); protocol
reference from xDrip and xdripswift (GPL v3).
