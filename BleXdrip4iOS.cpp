#include "BleXdrip4iOS.h"
#include "AppConfig.h"
#include "GlucoseState.h"
#include "TimeService.h"
#include "Ui.h"
#include "Log.h"
#include <NimBLEDevice.h>
#include <M5Unified.h>

#define BLE_DEVICE_NAME  "M5Stack"
#define SERVICE_UUID     "AF6E5F78-706A-43FB-B1F4-C27D7D5C762F"
#define CHAR_UUID        "6D810E9F-0983-4030-BDA7-C7C9A6A19C1C"
#define MAX_PKT          20

static NimBLEServer *server = nullptr;
static NimBLECharacteristic *chr = nullptr;
static volatile bool connected = false;
static volatile bool advertisePending = false;
static bool authenticated = false;
static bool initialStartUpOngoing = true;
static uint32_t lastTimeRequestMs = 0;
// glucose values arrive on 0x10, the slope name separately on 0x13
static char pendingDir[32] = "";

static const char *pwLetters = "abcdefghijklmnopqrstuvwxyz0123456789";

// frame: [opcode][packetNo 1-based][totalPackets][ascii payload], <= 20 bytes
static void sendToClient(const char *text, uint8_t opCode) {
  if (!chr) return;
  int size = text ? strlen(text) : 0;
  if (size == 0) {
    uint8_t pkt[3] = {opCode, 0x01, 0x01};
    chr->setValue(pkt, 3);
    chr->notify();
    return;
  }
  int total = size / (MAX_PKT - 3);
  if (size > total * (MAX_PKT - 3)) total++;
  int sent = 0, num = 1;
  while (sent < size) {
    int chunk = size - sent;
    if (chunk > MAX_PKT - 3) chunk = MAX_PKT - 3;
    uint8_t pkt[MAX_PKT];
    pkt[0] = opCode;
    pkt[1] = (uint8_t)num;
    pkt[2] = (uint8_t)total;
    memcpy(pkt + 3, text + sent, chunk);
    chr->setValue(pkt, chunk + 3);
    chr->notify();
    sent += chunk;
    num++;
  }
}

static void generatePassword() {
  for (int i = 0; i < 10; i++)
    cfg.blePassword[i] = pwLetters[random(0, 36)];
  cfg.blePassword[10] = 0;
  cfg.save();
}

class CharCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t *d = v.data();
    size_t len = v.length();
    if (len == 0) return;
    uint8_t opCode = d[0];
    // NUL-terminated copy of the payload (starts at byte 3 for framed opcodes)
    char payload[64] = "";
    if (len > 3) {
      size_t n = len - 3;
      if (n > sizeof(payload) - 1) n = sizeof(payload) - 1;
      memcpy(payload, d + 3, n);
      payload[n] = 0;
    }
    Serial.printf("[ios] opcode %02X len %u\n", opCode, (unsigned)len);

    switch (opCode) {
      case 0x01:   // Nightscout URL - no Wi-Fi, consume and ignore
      case 0x02:   // API token
      case 0x07:   // WLAN SSID
      case 0x08:   // WLAN password
        break;

      case 0x03: { // unit: "true" = mg/dL
        uint8_t units = (strcmp(payload, "true") == 0) ? UNITS_MGDL : UNITS_MMOL;
        if (units != cfg.units) {
          cfg.units = units;
          cfg.save();
          gs.dataChanged = true;
        }
      } break;

      case 0x09: { // client requests BLE password
        if (cfg.blePassword[0] == 0) {
          generatePassword();
          sendToClient(cfg.blePassword, 0x0E);
          authenticated = true;
          gs.dataChanged = true;   // refresh BLE icon to steady blue
          logAdd("paired");
        } else {
          // password already exists: app must authenticate with 0x0A,
          // or the user resets it in our menu
          sendToClient("", 0x0F);
        }
      } break;

      case 0x0A: { // authenticate: [0x0A][password chars]
        if (cfg.blePassword[0] == 0) {
          generatePassword();
          sendToClient(cfg.blePassword, 0x0E);
          authenticated = true;
          gs.dataChanged = true;   // refresh BLE icon to steady blue
        } else {
          size_t pwLen = strlen(cfg.blePassword);
          bool match = (len == pwLen + 1) && memcmp(d + 1, cfg.blePassword, pwLen) == 0;
          sendToClient("", match ? 0x0B : 0x0C);
          authenticated = match;
          if (match) gs.dataChanged = true;
          logAdd("auth %s", match ? "OK" : "FAILED");
        }
      } break;

      case 0x10: { // "mgdl epochSecondsUTC"
        if (!authenticated) break;
        char *sp = strchr(payload, ' ');
        if (!sp) break;
        *sp = 0;
        int mgdl = atoi(payload);
        time_t utc = (time_t)strtoul(sp + 1, nullptr, 10);
        // readings are intentionally not logged (they would flood the log page)
        gs.onReading((uint16_t)mgdl, utc, nsDirectionToAngle(pendingDir));
      } break;

      case 0x12: { // local time epoch seconds
        if (!authenticated) break;
        unsigned long localEpoch = strtoul(payload, nullptr, 10);
        if (localEpoch > 1600000000UL)
          timeService.setFromLocal((time_t)localEpoch, cfg.tzOffsetSec);
        if (initialStartUpOngoing) {
          sendToClient("", 0x16);   // request all parameters once
          initialStartUpOngoing = false;
        }
      } break;

      case 0x13: { // slope as Nightscout direction name
        if (!authenticated) break;
        strlcpy(pendingDir, payload, sizeof(pendingDir));
        gs.onDirectionString(pendingDir);
      } break;

      case 0x14: { // timezone offset seconds (local - UTC)
        if (!authenticated) break;
        long off = strtol(payload, nullptr, 10);
        if (timeService.known())
          timeService.setFromUtc(time(nullptr), (int32_t)off);
        else {
          cfg.tzOffsetSec = (int32_t)off;
          cfg.save();
        }
      } break;

      case 0x15:   // text color - fixed palette here, ignore
      case 0x17:   // background color
        break;

      case 0x18: { // rotation 0-3
        if (!authenticated || len < 2) break;
        cfg.rotation = d[1];
        cfg.save();
        M5.Lcd.setRotation(cfg.rotation);
        ui.requestFullRedraw();
      } break;

      case 0x19: { // brightness 0-100
        if (!authenticated || len < 2) break;
        cfg.brightness = d[1];
        cfg.save();
        M5.Lcd.setBrightness(map(cfg.brightness, 0, 100, 0, 255));
      } break;

      case 0x21: { // read battery level -> 0x20
        if (!authenticated) break;
        int32_t lvl = M5.Power.getBatteryLevel();
        uint8_t pkt[2] = {0x20, (uint8_t)(lvl < 0 ? 100 : lvl)};
        chr->setValue(pkt, 2);
        chr->notify();
      } break;

      case 0x22:   // power off
        if (authenticated) M5.Power.powerOff();
        break;

      default:
        break;
    }
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
    connected = true;
    logAdd("connected %s", info.getAddress().toString().c_str());
    gs.dataChanged = true;
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    connected = false;
    authenticated = false;         // another client could connect next
    initialStartUpOngoing = true;
    logAdd("disconnected (%d)", reason);
    advertisePending = true;
    gs.dataChanged = true;
  }
};

void xdrip4iosBegin() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService *svc = server->createService(SERVICE_UUID);
  chr = svc->createCharacteristic(CHAR_UUID,
          NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  chr->setCallbacks(new CharCallbacks());
  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setName(BLE_DEVICE_NAME);
  adv->addServiceUUID(NimBLEUUID(SERVICE_UUID));
  adv->enableScanResponse(true);
  adv->start();
  Serial.println("[ios] advertising");
}

void xdrip4iosTick() {
  if (advertisePending) {
    advertisePending = false;
    delay(100);
    NimBLEDevice::getAdvertising()->start();
    Serial.println("[ios] advertising");
  }
  // ask the app for the time while we don't know it
  if (connected && authenticated && !timeService.known() &&
      millis() - lastTimeRequestMs > 30000UL) {
    lastTimeRequestMs = millis();
    sendToClient("", 0x11);
  }
}

bool xdrip4iosConnected() { return connected; }
bool xdrip4iosIsAuthenticated() { return connected && authenticated; }
