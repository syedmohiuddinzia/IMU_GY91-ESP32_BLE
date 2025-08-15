#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE server and characteristic pointers
extern BLEServer* pServer;
extern BLECharacteristic* pSensorCharacteristic;
extern BLECharacteristic* pLedCharacteristic;

extern bool deviceConnected;
extern bool oldDeviceConnected;
extern uint32_t value;

extern const int ledPin;

// UUIDs
#define SERVICE_UUID                 "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID   "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID      "19b10002-e8f2-537e-4f6c-d104768a1214"

// Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
};

// Characteristic callbacks
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic);
};

// BLE setup
void initBLE();
void handleBLE();

#endif
