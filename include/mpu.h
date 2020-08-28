#ifndef _MPU_H
#define _MPU_H

#include <Wire.h>

class MPU {
public:
  void initialize();

  void readIMUData();
  inline double getAccAngle() const { return accAngle; }
  inline float getGyroRate() const { return gyroRate; }

  bool calibrateYGyro();

private:
  float accAngle;
  float gyroRate;
  float gyroYZero;
  static uint8_t i2cBuffer[14];

  uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
  uint8_t i2cWriteBuffer(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
  uint8_t i2cRead(uint8_t registerAddress, uint8_t nbytes);
  bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);
};

#endif
