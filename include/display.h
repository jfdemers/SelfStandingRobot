#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "Arduino.h"
#include <Wire.h>

class Display {
public:
enum DisplayType { off, no_cursor, no_cursor_blink, cursor_no_blink, cursor_on_blink };

  static void clear();
  static void setPos(uint8_t x = 0, uint8_t y = 0);

  static void setDisplay(DisplayType display = cursor_no_blink);
};

#endif
