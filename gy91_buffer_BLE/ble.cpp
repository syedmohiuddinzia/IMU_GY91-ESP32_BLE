#include "ble.h"
#include "gy91functions.h"

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
const int ledPin = 2;

// Server callbacks
void MyServerCallbacks::onConnect(BLEServer* pServer) {
  deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
}

// Characteristic callbacks
void MyCharacteristicCallbacks::onWrite(BLECharacteristic* pLedCharacteristic) {
  String val = pLedCharacteristic->getValue();
  if (val.length() > 0) {
    Serial.print("Characteristic event, written: ");
    Serial.println(static_cast<int>(val[0])); // Integer value

    int receivedValue = static_cast<int>(val[0]);
    digitalWrite(ledPin, (receivedValue == 1) ? HIGH : LOW);
  }
}

// BLE initialization
void initBLE() {
  pinMode(ledPin, OUTPUT);

  BLEDevice::init("ESP32-BLE-Device");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );

  pLedCharacteristic = pService->createCharacteristic(
    LED_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );

  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  Serial.println("Waiting for a client connection to notify...");
}

// BLE handling in loop
void handleBLE() {
  if (deviceConnected) {
    static uint8_t buffer[108]; size_t len = 0; // enough for 27 floats * 4 bytes
    sendDataBLE(buffer, len);
    pSensorCharacteristic->setValue(buffer, len);
    pSensorCharacteristic->notify();
    delay(100);
  }

  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}
