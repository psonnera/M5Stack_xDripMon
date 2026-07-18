# Testing & flashing

## Flashing the pre-built binary (16MB M5Stack: Basic 16MB/v2.7, Fire, Core2)

Files are in `Binaries/ESP32_16MB/`. Flash with esptool (adjust COM port):

```
esptool --chip esp32 --port COM5 --baud 921600 --before default_reset --after hard_reset \
  write_flash -z --flash_mode dio --flash_freq 80m \
  0x1000  Binaries/ESP32_16MB/M5Stack_xDripMon.ino.bootloader.bin \
  0x8000  Binaries/ESP32_16MB/M5Stack_xDripMon.ino.partitions.bin \
  0x10000 Binaries/ESP32_16MB/M5Stack_xDripMon.ino.bin
```

`firmware.json` / `update.inf` follow the M5_NightscoutMon layout, so the same
web flasher / updater tooling works.

## Buttons

- **A (left)** / **C (right)**: previous / next page (main → big glucose → clock → log).
- **B (middle) short press**:
  - on the display pages: snooze (in Android mode the phone alert snoozes too).
    Re-press within 2 s to extend the snooze (×1, ×2, ×3, ×4 of the snooze
    minutes, then off) — the remaining minutes show in the bottom bar.
  - on the **log page**: open the configuration menu.
- **B (middle) long press**: nothing (reserved).
- The menu is reached from the **log page** (last page): press A/C to page to it,
  then press B.
- In the menu the icons above the buttons show the actions: up / select / down
  (minus / ok / plus while editing a value); hold B = back.
- Bluetooth icon: blinking red = not connected, blinking blue = connected but
  not yet authenticated, steady blue = connected and authenticated.

## First boot / after a factory reset

The device asks for the data source before starting Bluetooth: press **A**
for xDrip (Android) or **C** for xDrip4iOS. The choice is stored in NVS and
is not asked again until the next factory reset. It can still be changed later
in the menu (Data source).

## Phone-free smoke test (USB serial, 115200)

Type commands in the serial monitor:

- `bg 123 -45`  inject a reading (angle -90..90; 180 hides the arrow)
- `demo`        fill the mini-graph with a demo curve
- `time 14 30`  set the clock to 14:30
- `status`      print current state

Check: value/colour/arrow on all three pages, graph dots, min-ago box turning
white >5 min and red >15 min, alarm bar + sound when a reading crosses a
threshold (set thresholds in the menu to force it), snooze.

## xDrip4iOS (iOS)

1. Menu → Data source → xDrip4iOS (device reboots).
2. In xDrip4iOS: Settings → add a Bluetooth device of type **M5Stack**; it
   should find "M5Stack", pair, and exchange a password automatically.
3. Verify: reading + trend arrow appear, clock sets itself, changing the app's
   unit / brightness / rotation is reflected. Reboot the M5Stack → it should
   reconnect on its own.

## xDrip (Android)

1. Choose "xDrip (Android)" at first boot (button A), or later via
   Menu → Data source → xDrip (Android) (device reboots).
2. In xDrip: Settings → Smart Watch Features → **Mi Band** → enable "Use Mi Band",
   and enable sending the reading as a notification (BG on band). Leave the auth
   key blank so xDrip generates one.
3. xDrip scans and finds **MI Band 2** → connects → authenticates (watch the
   serial log: `auth OK`). The BG text alert then arrives; value + arrow show
   within one reading cycle (≤5 min).
4. Short-press B to snooze → the xDrip alert should also snooze.

Notes for Android mode:
- There is no time sync from Android in this mode. Set the clock via the menu
  ("Set time"); on Core2 it is kept by the RTC across reboots, on Basic it is
  lost on power-off (the "min ago" counter still works via the internal timer).
- The value arrives as text in the phone's configured unit; the device converts.

## nRF Connect (Android-mode protocol, no phone app)

Advertised name: `MI Band 2`. Auth characteristic `00000009-...af100700`:
- write `01 00 <16-byte key>` → expect notify `10 01 01`
- write `02 00`               → expect notify `10 02 01 <16-byte challenge>`
- write `03 00 <AES-ECB(key, challenge)>` → expect `10 03 01` (or `10 03 04` on mismatch)
Then write to New Alert `2A46`: `05 01 42 47 3A 20 31 32 33 20 E2 86 97` ("BG: 123 ↗")
→ the device shows 123 with an up-right arrow.
```
