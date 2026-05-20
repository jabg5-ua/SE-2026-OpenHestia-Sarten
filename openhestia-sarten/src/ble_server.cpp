// ESP32-C3 Super Mini
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
//#include <BLEUtils.h>
#include <BLE2902.h>

#ifndef LED_PIN
  #define LED_PIN LED_BUILTIN
#endif
#define PIN_BLUETOOTH 6 // Cuando el pin GPIO6 esté activo, intenta escuchar por BLE al fogón
//#define PIN_BLUETOOTH 32 // Si queréis probar en ESP32 normal, uso el pin GPIO32
#define LED_ON LOW // El LED está invertido y se enciende cuando vale 0
#define LED_OFF HIGH
#define DEVICE_NAME "HESTIA_C3"
#define SERVICE_UUID        "00000000-0000-0000-0000-000000000000" // UUIDs inventadas
#define CHARACTERISTIC_UUID "00000000-0000-0000-0000-0000000b0000"

bool adv = false;
unsigned long milisegundos;
BLEServer *bleServer;
BLEService *service;
BLECharacteristic *characteristic;
BLEAdvertising *advertising;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BLUETOOTH, INPUT_PULLDOWN);
  digitalWrite(LED_PIN, LED_OFF);

  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  service = bleServer->createService(SERVICE_UUID);
  characteristic = service->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  characteristic->setValue("Initializing...");
  service->start();
  advertising = BLEDevice::getAdvertising();

  //xTaskCreate(conexionBLE, "Tarea_BLE", 2048, NULL, 1, NULL);
}

void loop() {
  if (!bleServer->getConnectedCount()) {
    if (!adv && digitalRead(PIN_BLUETOOTH)) {
      advertising->start();
      adv = true;
      Serial.println("Buscando fogón...");
    }
    else if (adv && !digitalRead(PIN_BLUETOOTH)) {
      advertising->stop();
      adv = false;
      Serial.println("Detenido");
    }
  }
  else {
    if (adv) {
      advertising->stop();
      adv = false;
      Serial.println("Conectado!");
    }

    // Enviar millis() al fogón
    milisegundos = millis();
    Serial.print("Enviando millis(): ");
    Serial.println(milisegundos);
    characteristic->setValue(String(milisegundos).c_str());
    characteristic->notify();
    digitalWrite(LED_PIN, LED_ON);
    delay(900);
    digitalWrite(LED_PIN, LED_OFF);
    delay(100);
  }
}

void conexionBLE(void *pvParameters) {

}