#include "display.h"

void Display::clear() {
  Serial.write(uint8_t(12));
  delay(5);
}

void Display::setPos(uint8_t x, uint8_t y) {
  if (y == 0) {
    Serial.write(uint8_t(128 + x));
  }
  else if (y == 1) {
    Serial.write(uint8_t(148 + x));
  }
}

void Display::setDisplay(DisplayType display) {
  Serial.write(uint8_t(21 + display));
}
