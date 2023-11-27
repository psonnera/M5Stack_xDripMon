/*  M5Stack xDrip+ monitor
    Copyright (C) 2021 Patrick Sonnerat

    Connects to xDrip+ using LeFun Band protocol via Bluetooth.
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>. 

    Original M5Stack_NightscoutMon code by Martin Lukasek https://github.com/mlukasek/M5_NightscoutMon
    LeFun xDrip+ interface code from Fabian Schmidt https://github.com/Fabian-Schmidt/PlaygroundArduino/tree/master/ESP32/xDrip-LeFun
    Preferences from Martin Sloup (Arcao) https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences/examples/StartCounter
    IoT Icon Set by Artur Funk (GPL v3)
    
*/

#include <Arduino.h>
#include <M5Stack.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Free_Fonts.h"
#include <Preferences.h>
#include "icons.c"
#include "time.h"

#define EEPROM_SIZE 1
#define BGU_MMOL 'A'
#define BGU_MGDL 'B'

time_t xdriptime, time_offset, last_refreshBG, last_refreshDelta, last_reading, prev_reading;
int iBGLast, iBGPrev;
char sensSgvStr[8], sensPrvStr[8];
bool bFirstValue;

#define INH_NONE 0
#define INH_BGUPDATE 1
#define INH_LASTREAD 2
#define INH_BGDELTA  4
#define INH_ALL 0xFF

int inhibit;

#define BT_CONNECTED 2
#define BT_CONNECTING 1
#define BT_DISCONNECTED 0

int iBTState, iBTOldState;
char cAgo[16];

/**********************************************************************

Utilities here

**********************************************************************/

void SetupScreen(void)
{
  M5.Lcd.fillScreen(BLACK);
  drawIcon(120, 0, 32, 40, (uint8_t*)xDrip_icon32x40, TFT_RED);
  M5.Lcd.setCursor(160 , 28);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("xDrip+");
  iBTOldState = 0;
  inhibit = INH_NONE;
}

char WaitButtonPressed(void)
{
  bool bBTN = false;
  char cBTN;
  while(!bBTN)
  {
    if(M5.BtnA.read())
    {
      cBTN = 'A';
      bBTN = true;
    }
    if(M5.BtnB.read())
    {
      cBTN = 'B';
      bBTN = true;
    }
    if(M5.BtnC.read())
    {
      cBTN = 'C';
      bBTN = true;
    }
  }
  Serial.print(F("Button "));
  Serial.print(cBTN);
  Serial.print(F(" was pressed\n"));
  
  return cBTN; 
}

// This is from Martin Lukasek in M5Stack_NightscoutMon

void drawArrow(int x, int y, int asize, int aangle, int pwidth, int plength, uint16_t color)
{
  float dx = (asize-10)*cos(aangle-90)*PI/180+x; // calculate X position  
  float dy = (asize-10)*sin(aangle-90)*PI/180+y; // calculate Y position  
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth/2;  float y2 = pwidth/2;
  float x3 = -pwidth/2; float y3 = pwidth/2;
  float angle = aangle*PI/180-135;
  float xx1 = x1*cos(angle)-y1*sin(angle)+dx;
  float yy1 = y1*cos(angle)+x1*sin(angle)+dy;
  float xx2 = x2*cos(angle)-y2*sin(angle)+dx;
  float yy2 = y2*cos(angle)+x2*sin(angle)+dy;
  float xx3 = x3*cos(angle)-y3*sin(angle)+dx;
  float yy3 = y3*cos(angle)+x3*sin(angle)+dy;
  M5.Lcd.fillTriangle(xx1,yy1,xx3,yy3,xx2,yy2, color);
  M5.Lcd.drawLine(x, y, xx1, yy1, color);
  M5.Lcd.drawLine(x+1, y, xx1+1, yy1, color);
  M5.Lcd.drawLine(x, y+1, xx1, yy1+1, color);
  M5.Lcd.drawLine(x-1, y, xx1-1, yy1, color);
  M5.Lcd.drawLine(x, y-1, xx1, yy1-1, color);
  M5.Lcd.drawLine(x+2, y, xx1+2, yy1, color);
  M5.Lcd.drawLine(x, y+2, xx1, yy1+2, color);
  M5.Lcd.drawLine(x-2, y, xx1-2, yy1, color);
  M5.Lcd.drawLine(x, y-2, xx1, yy1-2, color);
}

void drawIcon(int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t *bitmap, uint16_t color) {
  int16_t w = width;
  int16_t h = height; 
  int32_t i, j, byteWidth = (w + 7) / 8;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
        M5.Lcd.drawPixel(x + i, y + j, color);
      }
    }
  }
}

/**********************************************************************

BLE characteristics for a LeFun Band emulation

**********************************************************************/

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID "000018d0-0000-1000-8000-00805f9b34fb"           // LeFun
#define CHARACTERISTIC_UUID_RX "00002d01-0000-1000-8000-00805f9b34fb" // REPLY_CHARACTERISTIC
#define CHARACTERISTIC_UUID_TX "00002d00-0000-1000-8000-00805f9b34fb" // WRITE_CHARACTERISTIC

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

uint8_t calculateCRC(const uint8_t input[], byte length)
{
  uint8_t result = 0;
  int index = 0;
  int bit = 0;

  for (index = 0; index < length; index++)
  {
    for (bit = 0; bit < 8; bit++)
    {
      result = ((input[index] >> bit ^ result) & 1) == 0 ? result >> 1 : (result ^ 0x18) >> 1 | 0x80;
    }
  }
  return result;
}

void Pong()
{
  uint8_t rxPing[] = {
    /*RxStartByte*/ 0x5A,
    /*Length*/ 0x14,
    /*OpCode*/ 0x00,
    /*ff1*/ 0x00,
    /*othree*/ 0x00,
    /*zero2*/ 0x00,
    /*zero3*/ 0x00,
    /*3xmodel*/ 0x4D, 0x35, 0x53,
    /*vers1*/ 0x00,
    /*2x hwVersion*/ 0x00, 0x01,
    /*2x swVersion*/ 0x00, 0x01,
    /*4x manufacturer*/ 0x45, 0x53, 0x50, 0x20,
    /*CRC*/ 0x00};
  rxPing[sizeof(rxPing) - 1] = calculateCRC(rxPing, sizeof(rxPing) - 1);

  pTxCharacteristic->setValue(rxPing, sizeof(rxPing));
  pTxCharacteristic->notify();
  Serial.print(F("Pong ******************\n"));
}

void Shake()
{
  uint8_t rxShake[] = {
    /*RxStartByte*/ 0x5A,
    /*Length*/ 0x04,
    /*OpCode*/ 0x0E,
    /*CRC*/ 0x00};
  rxShake[sizeof(rxShake) - 1] = calculateCRC(rxShake, sizeof(rxShake) - 1);

  pTxCharacteristic->setValue(rxShake, sizeof(rxShake));
  pTxCharacteristic->notify();
  Serial.print("Shake ******************\n");
}

void Find()
{
  uint8_t rxShake[] = {
    /*RxStartByte*/ 0x5A,
    /*Length*/ 0x04,
    /*OpCode*/ 0x0A,
    /*CRC*/ 0x00};
  rxShake[sizeof(rxShake) - 1] = calculateCRC(rxShake, sizeof(rxShake) - 1);

  pTxCharacteristic->setValue(rxShake, sizeof(rxShake));
  pTxCharacteristic->notify();
  Serial.print("Find ******************\n");
}

/**********************************************************************

This is the main BLE callback routine

**********************************************************************/

int BGUnit;
bool bOnce, bWaitBG;

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      Serial.print(F("***********************************************************\n"));
    }


    if (rxValue.length() > 3 && rxValue[0] == 0xAB && rxValue[1] == rxValue.length() && rxValue[rxValue.length() - 1] == calculateCRC(reinterpret_cast<const uint8_t *>(rxValue.data()), rxValue.length() - 1))
    {
      Serial.print(F("OpCode: 0x"));
      Serial.print(rxValue[2], HEX);
      Serial.print(F("\n"));
      
    if (rxValue[2] == 0x00 && rxValue[1] == (1 + 3)) //******************************** 0x00
      { //TxPing
        Serial.print(F("Ping\n"));
        if(bWaitBG)M5.Lcd.print(":");
        Pong();
      }
      else if (rxValue[2] == 0x02 && rxValue[1] == (5 + 3)) //******************************** 0x02
      { //TxSetLocaleFeature
        Serial.print(F("SetLocaleFeature\n"));
        if(bWaitBG)M5.Lcd.print(".");
     }
      else if (rxValue[2] == 0x04 && rxValue[1] == (8 + 3))  //******************************** 0x04
      { //TxSetTime
        Serial.print(F("SetTime\n"));
        if(bWaitBG)M5.Lcd.print("@");
        
        if (rxValue[3] == 0x01 /*WRITE*/)
        {
          bWaitBG = false;
         
          //rxValue[4]  Year ignore
          uint8_t decimal1 = 0x00;
          uint8_t decimal2 = 0x00;
          if (rxValue[5] == 0x00 && rxValue[6] == 0x00)
          {
            //no value
            return;
          }
          else if (rxValue[5] == 0x00)
          {
            //value is rxValue[6] only
            decimal1 = rxValue[6];
          }
          else
          {
            //value is rxValue[5].rxValue[6]
            decimal1 = rxValue[5];
            decimal2 = rxValue[6];
          }
          uint8_t hours = rxValue[7];
          uint8_t minutes = rxValue[8];
          uint8_t seconds = rxValue[9];

          Serial.printf("Current time is %02d:%02d:%02d\n", hours, minutes, seconds);

          time(&last_reading);
          xdriptime = 3600*hours + 60*minutes + seconds;
          time_offset = difftime(xdriptime, last_reading);
          Serial.printf("M5Time %ld\n", last_reading);
          Serial.printf("xDripTime %ld\n", xdriptime);
          Serial.printf("DeltaTime %ld\n", time_offset);
          
          Serial.printf("BG codes received %d - %2d\n", decimal1, decimal2);

// Display xDrip+ logo and text at first BG --------------------
          
          if(!strcmp(sensPrvStr, " ")) SetupScreen();

          iBGPrev = iBGLast;
          int iBGv, iBGd;
          if(decimal1==0) 
          {
            iBGv = decimal2;
            iBGd = 0; 
          }
          else
          {
            iBGv = decimal1;
            iBGd = decimal2;
          }

          if(BGUnit == BGU_MMOL)
          {
            if(iBGv < 10) 
            {
              sprintf(sensSgvStr, "%d.%d", iBGv, iBGd);
              iBGLast = iBGv*10 + iBGd;
            }
            else
            {
              sprintf(sensSgvStr, " %d", iBGv);
              iBGLast = iBGv*10;
            }
          }
          else
          {
            uint iBG = decimal1*10 + decimal2;
            Serial.printf("BG calc1 mg/dl %d\n", iBG);
            iBG *= 18;
            Serial.printf("BG calc2 mg/dl %d\n", iBG);
            if(iBG % 10>5) iBG += 10; // fixing mmol to mgdl round up errors
            Serial.printf("BG calc3 mg/dl %d\n", iBG);
            iBG /= 10;
            Serial.printf("BG calc4 mg/dl %d\n", iBG);
            if(iBG < 100) sprintf(sensSgvStr, " %d", iBG);
            else  sprintf(sensSgvStr, "%d", iBG);
            iBGLast = iBG;
          }
          bFirstValue=true;
        }
      }
      else if (rxValue[2] == 0x07 && rxValue[1] == (4 + 3))
      { //TxSetScreens
        Serial.print(F("SetScreens\n"));
        if(bWaitBG)M5.Lcd.print("*");
      }
      else if (rxValue[2] == 0x08 && rxValue[1] == (4 + 3))
      { //TxSetFeatures
        Serial.print(F("SetFeatures\n"));
        if(bWaitBG)M5.Lcd.print("+");
      }
    }
    Serial.print(F("*********\n"));

    // Show BT activity
    drawIcon(303, 10, 16, 16, (uint8_t*)arrow_updown_icon16x16, TFT_WHITE);
    delay(200);
    drawIcon(303, 10, 16, 16, (uint8_t*)arrow_updown_icon16x16, TFT_BLACK);
  }
};

/**********************************************************************

This is the main M5Stack Setup routine

**********************************************************************/

Preferences preferences;

void setup()
{

  bWaitBG = false;
  bFirstValue = false;
  inhibit = INH_ALL;

  esp_bt_controller_enable(ESP_BT_MODE_BLE);

  // Create the BLE Device
  BLEDevice::init("Lefun");
  Serial.print("Creating LeFun device\n");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.print("Creating BLE server\n");

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.print("Creating BLE service\n");

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);
  Serial.print("Creating BLE characteristic\n");

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.print("Start advertising. Waiting a client connection to notify...\n");

  // initialize the M5Stack object
  M5.begin();
  M5.Power.begin();

  // That supposed to silent the device?
  ledcAttachPin(SPEAKER_PIN, TONE_PIN_CHANNEL);
  ledcWriteTone(TONE_PIN_CHANNEL, 0);
  CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);


  // prevent button A "ghost" random presses
  Wire.begin();
  SD.begin();
  // M5.Speaker.mute();

  // Lcd display
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(0, 16);
  M5.Lcd.setFreeFont(FSS9);
  M5.Lcd.setTextSize(1);
  yield();

  # ifdef ARDUINO_M5STACK_Core2
    Serial.print("M5Stack CORE2 code starting\n");
  # else
    Serial.print("M5Stack BASIC code starting\n");
  # endif

  Serial.print("Free Heap: "); Serial.print(ESP.getFreeHeap());

  inhibit = INH_ALL;
  M5.Lcd.print("Hello. I'm your xDrip+ Monitor\n \n");

  preferences.begin("xDripMon", false);
  
  BGUnit = preferences.getUInt("units", 0);
  Serial.printf("\nMemorized units (A for mmol B for mgdl: %u\n", BGUnit);
 
  while((BGUnit!=BGU_MMOL)&&(BGUnit!=BGU_MGDL))
  {
    Serial.print(F("\nUnits undefined in EEPROM\n"));
    M5.Lcd.print("\nWhich unit do you want?\n");
    M5.Lcd.print("Right button mg/dl\n");
    M5.Lcd.print("Left button mmol/l\n");
    char cBTN = WaitButtonPressed();
    switch(cBTN)
    {
      case 'A': BGUnit = BGU_MMOL;
                M5.Lcd.print("You selected mmol/l\n");
                break;
      case 'C': BGUnit = BGU_MGDL;
                M5.Lcd.print("You selected mg/dl\n");
                break;
    }
    delay(500);
    Serial.print(F("Saving units EEPROM\n"));
    preferences.putUInt("units", BGUnit);
    preferences.end();
    Serial.print("Restarting in 5 seconds...");
    M5.Lcd.print("Let me save that and restart...");
    delay(5000);
    ESP.restart();
  }
  preferences.end();
  M5.Lcd.print("Enable phone bluetooth\n");
  M5.Lcd.print("In xDrip+:\n - Smart Watch Features\n - Enable Use LeFun Band\n");
  M5.Lcd.print("\nWaiting for pairing...");
  inhibit = INH_ALL;
  bOnce = true;

  last_refreshBG = 0;
  last_refreshDelta = 0;
  strcpy(sensSgvStr, " ");
  strcpy(sensPrvStr, " ");

  last_refreshBG = -60;
  last_reading = -300;
  prev_reading = -300;
  iBTOldState = BT_DISCONNECTED;
  iBGLast = 0;
  iBGPrev = 0;
}

/**********************************************************************

This is the main M5Stack Loop

**********************************************************************/

void loop() 
{
  // Bluetooth connection handling ********************************
  
  if (deviceConnected)
  {
    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    iBTState = BT_CONNECTED;
    if(iBTState != iBTOldState)
    {
      Serial.print(F("BT: State change to connected\n"));
      iBTOldState = iBTState;
      drawIcon(275, 0, 24, 32, (uint8_t*)bluetooth_icon24x32, TFT_BLUE);
    }
    if(bOnce)
    {
      bOnce = false;
      Serial.print(F("Device connected\n"));
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setFreeFont(FSS9);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(0 , 0);
      M5.Lcd.print("\nI'm connected!\nWait for the next measurement\n(It can take up to 5 minutes)\n\n");
      M5.Lcd.print("If nothing shows up, in xDrip+ Settings:\n-Switch Off Send Readings\n-Switch off Use Lefun Band\nThen switch On Both\n\nWait ");
      inhibit = INH_ALL;
      bWaitBG = true;
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    iBTState = BT_DISCONNECTED;
    if(iBTState != iBTOldState)
    {
      Serial.print(F("BT: State change to disconnected\n"));
      iBTOldState = iBTState;
      drawIcon(275, 0, 24, 32, (uint8_t*)bluetooth_icon24x32, TFT_RED);
    }
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.print("Disconnecting.\nStart advertising\n");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    iBTState = BT_CONNECTING;
    if(iBTState != iBTOldState)
    {
      Serial.print(F("BT: State change to connecting\n"));
      iBTOldState = iBTState;
      drawIcon(275, 0, 24, 32, (uint8_t*)bluetooth_icon24x32, TFT_GREEN);
    }
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.print(F("Connecting\n"));
  }

// Periodic information refresh ***********************************

  time_t timenow, time_delta;
  
  time(&timenow);

// Every minute, or first reading, update BG

  if(strcmp(sensSgvStr, sensPrvStr)||!strcmp(sensPrvStr, " ")||bFirstValue)
  {
    if((inhibit!=INH_BGUPDATE) && (inhibit!=INH_ALL))
    {
      Serial.printf("Now Time %ld\n", timenow);
      Serial.printf("Last refresh %ld\n", last_refreshBG);
      Serial.printf("Old and new values %s %s\n", sensSgvStr, sensPrvStr);
      Serial.printf("Inhibit %d\n", inhibit);
    
      bFirstValue=false;
      inhibit = INH_LASTREAD;

      Serial.printf("Updating BG value %s\n", sensSgvStr);
      M5.Lcd.setTextSize(2);  //BIG
      M5.Lcd.setTextDatum(MC_DATUM);
      M5.Lcd.setFreeFont(FSSB24);

      M5.Lcd.fillRect(90, 90, 175, 10, TFT_BLACK);
      M5.Lcd.setTextColor(BLACK, TFT_BLACK);
      M5.Lcd.drawString(sensPrvStr, 170, 90, GFXFF);
      M5.Lcd.setTextColor(WHITE, TFT_BLACK);
      M5.Lcd.drawString(sensSgvStr, 170, 90, GFXFF);
      last_refreshBG = timenow;
      strcpy(sensPrvStr, sensSgvStr);
      M5.Lcd.setFreeFont(FSS9);

      inhibit = INH_NONE;
    }
  }

// Every minute update time and "ago"

  if(strcmp(sensSgvStr, " ") && difftime(timenow, last_refreshDelta) > 60)
  {
    if((inhibit!=INH_LASTREAD) && (inhibit!=INH_ALL))
    {
      inhibit = INH_BGUPDATE;
      M5.Lcd.fillRect(0, 60, 90, 55, TFT_BLACK);
      M5.Lcd.setFreeFont(FSS9);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setCursor(0, 75);
      time_delta = difftime(timenow, last_reading);
      sprintf(cAgo, "%d min ago", time_delta / 60);
      M5.Lcd.print(cAgo);
      M5.Lcd.setCursor(0, 100);
      int iDelta = iBGLast - iBGPrev;
      char cDelta[20];
      if(iBGLast < 39) sprintf(cDelta, "%d.%d mmol/l", iDelta/10, iDelta%10);
      else sprintf(cDelta, "%d mg/dl", iDelta/10);
      M5.Lcd.print(cDelta);

      time_delta = difftime(timenow, last_reading);
      Serial.printf("Time delta cross value %ul\n", time_delta);
      if(time_delta > 600) M5.Lcd.fillRect(90, 90, 175, 10, TFT_WHITE);
      
 /*     if(time_delta > 60) //Workaround for xDrip+ losing Bluetooth connectivity after few minutes...
      {
        Serial.printf("+*+*+*+*+* Restarting advertising +*+*+*+*+*\n");
        drawIcon(275, 0, 24, 32, (uint8_t*)bluetooth_icon24x32, TFT_CYAN);
        pServer->getAdvertising()->start();
        delay(200);
        pServer->startAdvertising(); 
        drawIcon(275, 0, 24, 32, (uint8_t*)bluetooth_icon24x32, TFT_BLUE);
      }
      Pong();*/

      last_refreshDelta = timenow;

      time_t displaytime = timenow + time_offset;
      struct tm *tmp = gmtime(&displaytime);
      char cTime[8];
      sprintf(cTime, "%02d:%02d", tmp->tm_hour, tmp->tm_min);
      M5.Lcd.fillRect(0, 1, 100, 40, TFT_BLACK);
      M5.Lcd.setFreeFont(FSS9);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0, 30);
      M5.Lcd.print(cTime);
      M5.Lcd.setFreeFont(FSS9);

      M5.Lcd.fillRect(253, 5, 13, 26, TFT_BLACK);
      if(M5.Power.isCharging())
      {
        Serial.print("M5Stack is plugged\n");
        drawIcon(250, 0, 16, 32, (uint8_t*)batt0_icon16x32, TFT_GREEN);
        drawIcon(254, 9, 8, 16, (uint8_t*)battcharging_icon8x16, TFT_YELLOW);
      }
      else
      {
        int8_t iBatt = M5.Power.getBatteryLevel();
        Serial.printf("M5Stack battery %d\n", iBatt);

        switch(iBatt)
        {
          case 0:   drawIcon(250, 0, 16, 32, (uint8_t*)batt20_icon16x32, TFT_RED); break; 
          case 25:  drawIcon(250, 0, 16, 32, (uint8_t*)batt40_icon16x32, TFT_YELLOW); break;
          case 50:  drawIcon(250, 0, 16, 32, (uint8_t*)batt60_icon16x32, TFT_GREEN); break;
          case 75:  drawIcon(250, 0, 16, 32, (uint8_t*)batt80_icon16x32, TFT_GREEN); break;
          case 100: drawIcon(250, 0, 16, 32, (uint8_t*)batt100_icon16x32, TFT_GREEN); break;
        }
      }    
      inhibit = INH_NONE;
    }
  }
}
