# M5Stack xDripMon

A Bluetooth-only glucose monitor for M5Stack — no Wi-Fi, no web server, no
cloud. The phone connects directly to the device over BLE and pushes each
reading.

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

## Data sources

Chosen on first boot (button **A** = xDrip Android, **C** = xDrip4iOS) and
changeable any time in the menu:

- **xDrip (Android)** — in xDrip enable:
  `Settings > Smart Watch Features > Mi Band > Use Mi Band` (leave the auth
  key empty) and enable sending the reading as a notification. xDrip finds
  the device, authenticates and pushes each reading. *(No time sync in this
  mode: set the clock once in the device menu; Core2 keeps it via RTC.)*
- **xDrip4iOS / xdripswift (iOS)** — add a Bluetooth device of type
  **M5Stack** in the app; pairing, readings, trend and time sync are
  automatic.

## Hardware

16 MB-flash ESP32 M5Stack cores: **Basic 2.6/2.7, Fire, Core2** — one binary
for all (`Binaries/ESP32_16MB`, layout compatible with M5_NightscoutMon
flashing tools). See `TESTING.md` for the esptool command and test guide.

## Building

Arduino IDE or arduino-cli with:
- board package **M5Stack 2.1.1** (targets M5Core / M5Fire / M5Core2)
- libraries **M5Unified** (with M5GFX) and **NimBLE-Arduino 2.x**

## History

v1 (in `legacy/`) emulated a LeFun band and was abandoned due to Bluedroid
Bluetooth-stack issues (readings only every 10 minutes). v2 is a complete
rewrite on NimBLE with a different xDrip transport.

## License

GPL v3. UI ported from M5_NightscoutMon (Martin Lukasek, GPL v3); protocol
reference from xDrip and xdripswift (GPL v3).
