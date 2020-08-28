#include <Arduino.h>

#include "mpu.h"

#define IMU           uint8_t(0x68)
#define PITCH_CORRECT 3.2f        // This is specific to my chip.

uint8_t MPU::i2cBuffer[14];

void MPU::initialize() {
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  while (i2cWrite(0x6B, 0x80, true)); // Reset device, this resets all internal registers to their default values
  do {
    while (i2cRead(0x6B, 1));
  } while (i2cBuffer[0] & 0x80); // Wait for the bit to clear
  delay(5);
  while (i2cWrite(0x6B, 0x09, true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode

  i2cBuffer[0] = 4; // Set the sample rate to 200Hz = 1kHz/(1+4)
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWriteBuffer(0x19, i2cBuffer, 4, true)); // Write to all four registers at once
  delay(100); // Wait for the sensor to get ready
}

void MPU::readIMUData() {
  while (i2cRead(0x3B, 14));
  int16_t AcX = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t AcZ = ((i2cBuffer[4] << 8) | i2cBuffer[5]);
  int16_t GyY = ((i2cBuffer[10] << 8) | i2cBuffer[11]);

  accAngle = -atan2((float)AcX, (float)AcZ) * RAD_TO_DEG + PITCH_CORRECT;
  gyroRate = (float)GyY / 131.0f - gyroYZero; // Convert to deg/s
}

uint8_t MPU::i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWriteBuffer(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t MPU::i2cWriteBuffer(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMU);
  Wire.write(registerAddress);
  Wire.write(data, length);
  return(Wire.endTransmission(sendStop)); // Returns 0 on success
}

uint8_t MPU::i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMU);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    return rcode;
  }
  Wire.requestFrom(IMU, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      i2cBuffer[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 100) && !Wire.available());
      if (Wire.available())
        i2cBuffer[i] = Wire.read();
      else {
        return 5; // Timeout
      }
    }
  }
  return 0; // Success
}

bool MPU::calibrateYGyro() {
  int16_t gyroYbuffer[50];

  gyroYZero = 0;

  for (uint8_t i = 0; i < 50; i++) {
    readIMUData();
    gyroYbuffer[i] = gyroRate;
    gyroYZero += gyroYbuffer[i];
    delay(10);
  }
  if (!checkMinMax(gyroYbuffer, 50, 2000)) {  // Check that min and max differ by no more than 2000 counts (15 deg/sec)
                                              // Note: Zero-rate output spec value is +/- 20 deg/sec, so this value may
                                              // need to be increased for gyros with large drift
    return true;
  }

  gyroYZero /= (50.0f * 131.0f);

  return false;
}

bool MPU::checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference) { // Used to check that the robot is laying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++) {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return max - min < maxDifference;
}
