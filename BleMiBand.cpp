#include "BleMiBand.h"
#include "AppConfig.h"
#include "GlucoseState.h"
#include "Ui.h"
#include "Log.h"
#include <NimBLEDevice.h>
#include <M5Unified.h>
#include <mbedtls/aes.h>
#include <esp_random.h>

// Huami / Mi Band UUIDs (from xDrip watch/miband/Const.java)
#define UUID_SVC_MIBAND1     "FEE0"
#define UUID_SVC_MIBAND2     "FEE1"
#define UUID_SVC_ANS         "1811"
#define UUID_SVC_IMM_ALERT   "1802"
#define UUID_SVC_HEARTRATE   "180D"
#define UUID_SVC_DEVINFO     "180A"

#define UUID_CHR_AUTH        "00000009-0000-3512-2118-0009af100700"
#define UUID_CHR_BATTERY     "00000006-0000-3512-2118-0009af100700"
#define UUID_CHR_CONFIG      "00000003-0000-3512-2118-0009af100700"
#define UUID_CHR_DEVICEEVENT "00000010-0000-3512-2118-0009af100700"
#define UUID_CHR_CHUNKED     "00000020-0000-3512-2118-0009af100700"
#define UUID_CHR_USERSETT    "00000008-0000-3512-2118-0009af100700"
#define UUID_CHR_NEW_ALERT   "2A46"
#define UUID_CHR_ALERT_LEVEL "2A06"
#define UUID_CHR_ALERT_CTRL  "2A44"
#define UUID_CHR_HR_MEASURE  "2A37"
#define UUID_CHR_HR_CONTROL  "2A39"
#define UUID_CHR_SOFT_REV    "2A28"
#define UUID_CHR_SERIAL      "2A25"
#define UUID_CHR_HW_REV      "2A27"
#define UUID_CHR_CURR_TIME   "2A2B"

// auth opcodes (OperationCodes.java)
#define AUTH_SEND_KEY        0x01
#define AUTH_REQ_RANDOM      0x02
#define AUTH_SEND_ENCRYPTED  0x03
#define AUTH_RESPONSE        0x10
#define AUTH_SUCCESS         0x01
#define AUTH_FAIL            0x04

#define DEVICEEVENT_CALL_REJECT 0x07

static NimBLEServer *server = nullptr;
static NimBLECharacteristic *chrAuth = nullptr;
static NimBLECharacteristic *chrBattery = nullptr;
static NimBLECharacteristic *chrDeviceEvent = nullptr;
static volatile bool connected = false;
static volatile bool authOk = false;
static volatile bool advertisePending = false;
static uint8_t challenge[16];
static bool challengeValid = false;

static void startAdvertising() {
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->start();
  Serial.println("[miband] advertising");
}

// ---------------------------------------------------------------- battery

static void updateBatteryValue() {
  if (!chrBattery) return;
  uint8_t blob[20] = {0};
  int32_t lvl = M5.Power.getBatteryLevel();
  if (lvl < 0) lvl = 100;
  blob[0] = 0x0F;
  blob[1] = (uint8_t)lvl;
  blob[2] = (M5.Power.isCharging() == m5::Power_Class::is_charging_t::is_charging) ? 1 : 0;
  // plausible "last charge" record (xDrip only logs it)
  blob[10] = 25; blob[11] = 0; blob[12] = 1;
  blob[19] = 100;
  chrBattery->setValue(blob, sizeof(blob));
}

// ---------------------------------------------------------------- auth

class AuthCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    NimBLEAttValue v = chr->getValue();
    const uint8_t *d = v.data();
    size_t len = v.length();
    if (len < 2) return;
    Serial.printf("[miband] auth write op=%02X len=%u\n", d[0], (unsigned)len);

    if (d[0] == AUTH_SEND_KEY && len >= 18) {
      // xDrip hands us the 16-byte AES key
      memcpy(cfg.mibandKey, d + 2, 16);
      cfg.mibandKeySet = 1;
      cfg.save();
      uint8_t resp[3] = {AUTH_RESPONSE, AUTH_SEND_KEY, AUTH_SUCCESS};
      chr->setValue(resp, sizeof(resp));
      chr->notify();
      logAdd("pairing key received");
    } else if (d[0] == AUTH_REQ_RANDOM) {
      esp_fill_random(challenge, sizeof(challenge));
      challengeValid = true;
      uint8_t resp[19];
      resp[0] = AUTH_RESPONSE;
      resp[1] = AUTH_REQ_RANDOM;
      resp[2] = AUTH_SUCCESS;
      memcpy(resp + 3, challenge, 16);
      chr->setValue(resp, sizeof(resp));
      chr->notify();
    } else if (d[0] == AUTH_SEND_ENCRYPTED && len >= 18) {
      uint8_t expected[16];
      bool ok = false;
      if (challengeValid && cfg.mibandKeySet) {
        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        mbedtls_aes_setkey_enc(&aes, cfg.mibandKey, 128);
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, challenge, expected);
        mbedtls_aes_free(&aes);
        ok = memcmp(expected, d + 2, 16) == 0;
      }
      uint8_t resp[3] = {AUTH_RESPONSE, AUTH_SEND_ENCRYPTED,
                         ok ? (uint8_t)AUTH_SUCCESS : (uint8_t)AUTH_FAIL};
      chr->setValue(resp, sizeof(resp));
      chr->notify();
      challengeValid = false;
      authOk = ok;
      logAdd("auth %s", ok ? "OK" : "FAILED");
      if (ok) gs.dataChanged = true;   // refresh BLE icon to steady blue
    }
  }
};

// ---------------------------------------------------------------- alerts

// parse "BG: 5,6 ↗" / "BG: 123 →" (uppercased by xDrip, any locale separator)
static bool parseBgText(const char *text, size_t len) {
  const char *p = nullptr;
  for (size_t i = 0; i + 3 <= len; i++) {
    if ((text[i] == 'B') && (text[i + 1] == 'G') && (text[i + 2] == ':')) {
      p = text + i + 3;
      break;
    }
  }
  if (!p) return false;
  while (*p == ' ') p++;

  char numBuf[12];
  size_t n = 0;
  while (n < sizeof(numBuf) - 1 && ((*p >= '0' && *p <= '9') || *p == '.' || *p == ',')) {
    numBuf[n++] = (*p == ',') ? '.' : *p;
    p++;
  }
  numBuf[n] = 0;
  if (n == 0) return false;
  float val = atof(numBuf);
  uint16_t mgdl;
  if (strchr(numBuf, '.') || val < 30.0f)
    mgdl = (uint16_t)lroundf(val * 18.0f);      // mmol/L
  else
    mgdl = (uint16_t)lroundf(val);              // mg/dL

  while (*p == ' ') p++;
  int angle = slopeArrowToAngle(p);

  // readings are intentionally not logged (they would flood the log page)
  time_t now = time(nullptr);
  gs.onReading(mgdl, now > 1600000000 ? now : 0, angle);
  return true;
}

class NewAlertCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    NimBLEAttValue v = chr->getValue();
    const uint8_t *d = v.data();
    size_t len = v.length();
    if (len < 3) return;
    // band2:      [category][count][text...]
    // band3/4:    [0xFA][count][icon][0x00 text 0x00 title 0x00]
    // Extract printable/UTF-8 bytes; NULs and control bytes become spaces so
    // the "BG:" search below works for every packet layout.
    static char text[192];
    size_t n = 0;
    for (size_t i = 2; i < len && n < sizeof(text) - 1; i++) {
      char c = (char)d[i];
      text[n++] = (d[i] >= 32) ? c : ' ';
    }
    while (n && text[n - 1] == ' ') n--;
    text[n] = 0;
    Serial.printf("[miband] alert text: '%s'\n", text);
    if (!parseBgText(text, n) && n > 0) {
      ui.setStatusMessage(text);   // non-BG notification from xDrip
      logAdd("msg: %.30s", text);
    }
  }
};

class AlertLevelCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    // vibration requests from xDrip alerts; local alarms already cover sound.
    NimBLEAttValue v = chr->getValue();
    if (v.length() >= 1)
      Serial.printf("[miband] alert level %02X\n", v.data()[0]);
  }
};

class IgnoreCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    Serial.printf("[miband] write to %s ignored (%u bytes)\n",
                  chr->getUUID().toString().c_str(), (unsigned)chr->getValue().length());
  }
};

class BatteryCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *chr, NimBLEConnInfo &info) override {
    updateBatteryValue();
  }
};

// ---------------------------------------------------------------- server

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
    connected = true;
    authOk = false;
    challengeValid = false;
    logAdd("connected %s", info.getAddress().toString().c_str());
    gs.dataChanged = true;         // refresh status icon
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    connected = false;
    authOk = false;
    challengeValid = false;
    logAdd("disconnected (%d)", reason);
    advertisePending = true;
    gs.dataChanged = true;
  }
};

void miBandBegin() {
  authOk = false;
  NimBLEDevice::init("MI Band 2");
  NimBLEDevice::setMTU(247);       // xDrip requests MTU 247
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  // device information
  NimBLEService *devInfo = server->createService(UUID_SVC_DEVINFO);
  devInfo->createCharacteristic(UUID_CHR_SOFT_REV, NIMBLE_PROPERTY::READ)->setValue("1.0.1.81");
  devInfo->createCharacteristic(UUID_CHR_HW_REV, NIMBLE_PROPERTY::READ)->setValue("V0.8.3.4");
  devInfo->createCharacteristic(UUID_CHR_SERIAL, NIMBLE_PROPERTY::READ)->setValue("M5xDripMon");

  // main Huami service
  NimBLEService *fee0 = server->createService(UUID_SVC_MIBAND1);
  chrBattery = fee0->createCharacteristic(UUID_CHR_BATTERY,
                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chrBattery->setCallbacks(new BatteryCallbacks());
  updateBatteryValue();
  fee0->createCharacteristic(UUID_CHR_CONFIG,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY)
      ->setCallbacks(new IgnoreCallbacks());
  chrDeviceEvent = fee0->createCharacteristic(UUID_CHR_DEVICEEVENT, NIMBLE_PROPERTY::NOTIFY);
  fee0->createCharacteristic(UUID_CHR_CHUNKED, NIMBLE_PROPERTY::WRITE_NR)
      ->setCallbacks(new IgnoreCallbacks());
  fee0->createCharacteristic(UUID_CHR_USERSETT,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)
      ->setCallbacks(new IgnoreCallbacks());
  fee0->createCharacteristic(UUID_CHR_CURR_TIME,
                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE)
      ->setCallbacks(new IgnoreCallbacks());

  // auth service
  NimBLEService *fee1 = server->createService(UUID_SVC_MIBAND2);
  chrAuth = fee1->createCharacteristic(UUID_CHR_AUTH,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
  chrAuth->setCallbacks(new AuthCallbacks());

  // alert notification service (BG arrives here)
  NimBLEService *ans = server->createService(UUID_SVC_ANS);
  ans->createCharacteristic(UUID_CHR_NEW_ALERT,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)
      ->setCallbacks(new NewAlertCallbacks());
  ans->createCharacteristic(UUID_CHR_ALERT_CTRL,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)
      ->setCallbacks(new IgnoreCallbacks());

  // immediate alert (vibration)
  NimBLEService *imm = server->createService(UUID_SVC_IMM_ALERT);
  imm->createCharacteristic(UUID_CHR_ALERT_LEVEL,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)
      ->setCallbacks(new AlertLevelCallbacks());

  // heart rate (xDrip may subscribe / write control point)
  NimBLEService *hr = server->createService(UUID_SVC_HEARTRATE);
  hr->createCharacteristic(UUID_CHR_HR_MEASURE, NIMBLE_PROPERTY::NOTIFY);
  hr->createCharacteristic(UUID_CHR_HR_CONTROL,
                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::READ)
      ->setCallbacks(new IgnoreCallbacks());

  devInfo->start();
  fee0->start();
  fee1->start();
  ans->start();
  imm->start();
  hr->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setName("MI Band 2");                    // xDrip matches this name exactly
  adv->addServiceUUID(NimBLEUUID(UUID_SVC_MIBAND1));
  adv->enableScanResponse(true);
  startAdvertising();
}

void miBandTick() {
  if (advertisePending) {
    advertisePending = false;
    delay(100);
    startAdvertising();
  }
  // keep the battery characteristic fresh once a minute
  static uint32_t lastBatt = 0;
  if (millis() - lastBatt > 60000) {
    lastBatt = millis();
    updateBatteryValue();
  }
}

bool miBandConnected() { return connected; }
bool miBandIsAuthenticated() { return connected && authOk; }

void miBandSendSnooze() {
  if (!connected || !chrDeviceEvent) return;
  uint8_t ev = DEVICEEVENT_CALL_REJECT;
  chrDeviceEvent->setValue(&ev, 1);
  chrDeviceEvent->notify();
  Serial.println("[miband] sent CALL_REJECT (snooze) to phone");
}
