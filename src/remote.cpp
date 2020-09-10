#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "remote.h"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "4b541423-23b7-48e7-8a4c-a00ef3c23c7b"
#define AXIS1_CHARACTERISTIC_UUID "e9047fc4-b82f-490c-9732-9089231ad245"
#define AXIS2_CHARACTERISTIC_UUID "31e7f1fc-ac15-4004-8e2f-6de012658ff9"
#define BATTERY_CHARACTERISTIC_UUID "206027d7-f0b5-4645-94f9-91e8f40324c4"
#define LOG_CHARACTERISTIC_UUID "0ea15bc2-78c0-4812-9eea-73378fd09455"

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    char *data = (char *)pCharacteristic->getData();

    signed char x = data[0];
    signed char y = data[1];

    char buffer[64];

    sprintf(buffer, "x: %i; y: %i\n", (int)x, (int)y);

    if (x < 0 || y < 0)
    {
      Serial.println("x or y is negative");
    }
    else
    {
      Serial.println("Both values are positive.");
    }

    Serial.println(buffer);
  }
};

void Remote::setup()
{
  Serial.println("Starting BLE for remote control.");

  BLEDevice::init("Self Standing Robot");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pAxis1Characteristic = pService->createCharacteristic(
      AXIS1_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pAxis1Characteristic->setCallbacks(new MyCallbacks());

  BLECharacteristic *pAxis2Characteristic = pService->createCharacteristic(
      AXIS2_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pAxis2Characteristic->setCallbacks(new MyCallbacks());

  BLECharacteristic *pBatteryCharacteristic = pService->createCharacteristic(
      BATTERY_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Characteristics defined. You can now control from phone.");
}
