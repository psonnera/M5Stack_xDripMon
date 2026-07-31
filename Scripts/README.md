# Firmware build script

One M5Unified source (`M5Stack_xDripMon.ino`) covers the whole 16 MB ESP32 M5Stack
lineup — Basic 16 MB/v2.7, Fire and all Core2 — so there is a **single firmware**:

| Firmware (folder)     | Build (FQBN)                              | Runs on                               |
|-----------------------|-------------------------------------------|---------------------------------------|
| `Binaries\ESP32_16MB` | `m5stack:esp32:m5stack_fire` (defaults)   | Basic 16 MB/v2.7, Fire, **all** Core2 |

The FQBN's default options are already correct: 16 MB flash, `default_16MB` partition
scheme (2× 6400K OTA app slots + 3.6 MB spiffs), flash mode dio @ 80 MHz. Board
sub-variants (PMU, IMU, RTC, touch) are auto-detected at runtime by M5Unified.

## Prerequisites

* **arduino-cli** — standalone, or the one bundled with Arduino IDE 2.x (found automatically).
* Board package **m5stack:esp32** (tested with 2.1.1) — Boards Manager URL
  `https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json`,
  or `arduino-cli core install m5stack:esp32 --additional-urls <that URL>`.
* Libraries: **M5Unified**, **NimBLE-Arduino** (2.x), **Adafruit NeoPixel**.

## Usage

```
Scripts\build.bat              # test build (update.inf untouched)
Scripts\build.bat -Release     # release build (bumps update.inf)
Scripts\build.ps1 -Clean       # guaranteed-fresh build (PowerShell directly)
```

`build.bat` is just a double-click launcher for `build.ps1` (bypasses the PowerShell
execution policy). The three flashable binaries land in `Binaries\ESP32_16MB\`.

The script finds `arduino-cli` automatically, in this order: `-ArduinoCli <path>` parameter →
`$env:ARDUINO_CLI` → `arduino-cli` on PATH → the known Arduino IDE 2.x install locations
(per-user `%LOCALAPPDATA%\Programs\Arduino IDE` and all-users `%ProgramFiles%\Arduino IDE`).
The board cores and libraries are looked up in the standard per-user Arduino folders
(`%LOCALAPPDATA%\Arduino15` and `Documents\Arduino`); if yours live elsewhere, set
`$env:ARDUINO_DIRECTORIES_DATA` / `$env:ARDUINO_DIRECTORIES_USER` before running.

## Versioning

Two version identifiers, deliberately separate:

* **`XDRIPMON_VERSION`** (in `M5Stack_xDripMon.ino`) — the semver shown on screen and in
  logs. Hand-maintained; the script never touches it.
* **`Binaries\ESP32_16MB\update.inf`** — the OTA build number, a fixed-width `YYYYMMDDnn`
  token compared lexicographically by the updater. `-Release` bumps it automatically
  (same-day sequence +1, or a new day starts at `01`) after a successful build; test
  builds leave it alone.

`Binaries\firmware.json` (M5Burner metadata) and `whatsnew.txt` are hand-maintained.

## How to flash a built firmware

`Binaries\ESP32_16MB\` holds three files: `*.ino.bin` (app), `*.ino.bootloader.bin`,
`*.ino.partitions.bin`. Replace `<port>` with your serial port (e.g. `COM5` — check
Device Manager or `arduino-cli board list`).

**Easiest — let arduino-cli place them at the right offsets:**

```
arduino-cli upload -p <port> `
  --fqbn m5stack:esp32:m5stack_fire `
  --input-dir Binaries\ESP32_16MB
```

**Manual — esptool with explicit offsets** (ESP32: bootloader at 0x1000):

```
esptool --chip esp32 -p <port> write_flash `
  0x1000  M5Stack_xDripMon.ino.bootloader.bin `
  0x8000  M5Stack_xDripMon.ino.partitions.bin `
  0x10000 M5Stack_xDripMon.ino.bin
```
