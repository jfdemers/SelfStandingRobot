#include <Arduino.h>

#include "robot.h"

Robot robot;

void setup() {
  robot.setup();
}

void loop() {
  robot.run();
}


/*#include <Arduino.h>
#include <stdlib.h>
#include <U8x8lib.h>
#include <Wire.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;

long int startTime;

void setup(void) {
  Serial.begin(1000000);
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_artosserif8_r);

  u8x8.setCursor(0,0);

  startTime = millis();
}

long int frames = 0;

void loop() {
  long int currentTime = millis();
  long int elapsed = currentTime - startTime;
  frames ++;

  char line1[16];
  sprintf(line1, "ms/f:%ld", (elapsed / frames));

  u8x8.setCursor(0, 0);
  u8x8.print(line1);

  if (frames % 100 == 0) {
    char line2[16];
    sprintf(line2, "F:%ld", frames);

    char line3[16];
    sprintf(line3, "T:%ld", elapsed);

    u8x8.setCursor(0, 1);
    u8x8.print(line2);

    u8x8.setCursor(0, 2);
    u8x8.print(line3);

    Serial.println(line2);
    Serial.println(line3);
  }
}*/