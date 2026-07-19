#ifndef UI_H
#define UI_H

#include <Arduino.h>

// lighter blue than TFT_BLUE (which is near-black on the panel), RGB565
#define COLOR_APP_BLUE 0x33DF

#define PAGE_MAIN  0
#define PAGE_BIGBG 1
#define PAGE_CLOCK 2
#define PAGE_LOG   3
#define MAX_PAGE   3

class Ui {
public:
  void begin();
  void tick();                       // call from loop; rate-limits itself
  void requestFullRedraw() { fullRedraw = true; }
  void nextPage();
  void prevPage();
  int  page() const { return dispPage; }
  bool suspended = false;            // true while the config menu owns the screen
  // transient text shown in the bottom info line (e.g. xDrip alert messages)
  void setStatusMessage(const char *msg);

private:
  void drawPage();
  void drawPage0();
  void drawPage1();
  void drawClockFace();
  void drawLogPage();
  void updateClockHands(bool force);
  void drawHeaderIcons();
  void drawBleIcon();
  void drawAlarmInfoLine();
  void drawMiniGraph();
  void headerDateTime(char *out, size_t len);
  uint16_t glucoseColor() const;
  void bgString(char *out, size_t len, bool &smallerFont) const;

  int  dispPage = PAGE_MAIN;
  bool fullRedraw = true;
  uint32_t lastTickMs = 0;
  int  lastMinAgo = -1;
  int  lastMinute = -1;
  int  lastSec = -1;
  bool clockInitial = true;
  bool blinkState = false;
  uint32_t lastBlinkMs = 0;
  float ohx = 160, ohy = 110, omx = 160, omy = 110, osx = 160, osy = 110;
  char statusMsg[64] = "";
  uint32_t statusMsgMs = 0;
};

// bottom hint row above the physical buttons (y=222). Any slot may be nullptr.
// Middle slot: icon, text, or both side by side.
void uiDrawButtonHints(const unsigned char *iconA, const unsigned char *iconB,
                       const char *textB, const unsigned char *iconC);

// Full-screen xDrip+ pairing QR; blocks until button C or an xDrip BLE connect.
void showXdripSetupQr();

extern Ui ui;

#endif
