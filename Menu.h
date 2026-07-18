#ifndef MENU_H
#define MENU_H

#include <Arduino.h>

// 3-button config menu:
//   A = up / decrease, C = down / increase, B = select / confirm
//   hold B = back / exit
class Menu {
public:
  bool active = false;
  void open();
  void handleButtons();   // call from loop while active

private:
  enum Screen : uint8_t { ROOT, THRESHOLDS, ALARMS_S, DISPLAY_S, TIME_S, BLUETOOTH_S };
  void draw();
  void drawTimeEditor();
  void select();
  void back();
  void adjust(int dir);
  void applyEdit(int dir);
  void close();
  int  itemCount() const;
  void valueString(int idx, char *out, size_t len) const;
  const char *itemLabel(int idx) const;
  bool itemEditable(int idx) const;

  Screen screen = ROOT;
  int cursor = 0;
  bool editing = false;
  uint32_t lastRepeatMs = 0;
  // time editor state
  int teField = 0;
  int teVals[5];   // year, month, day, hour, minute
};

extern Menu menu;

#endif
