#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <math.h>

namespace {
constexpr uint8_t kAccelerometerAddress = 0x68;
constexpr uint8_t kMuxSignalPin = 6;
constexpr uint8_t kMuxSelectPins[4] = {7, 8, 10, 18};
constexpr uint8_t kThermistorMuxChannels[4] = {0, 1, 2, 3};
constexpr float kReferenceResistorOhms = 100000.0f;
constexpr float kThermistorNominalOhms = 100000.0f;
constexpr float kThermistorBeta = 3950.0f;
constexpr float kNominalTemperatureC = 25.0f;
constexpr int kAdcMax = 4095;

constexpr char kServiceUuid[] = "f0f2f4f6-0000-4000-8000-000000000001";
constexpr char kDataUuid[] = "f0f2f4f6-0000-4000-8000-000000000002";

BLECharacteristic *dataCharacteristic = nullptr;

void tinyMlPredict(float ax, float ay, float az, float *fallProbability, float *activityScore) {
  const float weights1[3] = {1.7f, 1.7f, 2.3f};
  const float bias1 = -0.8f;
  const float weights2[3] = {0.5f, 0.5f, 0.7f};

  const float linear = weights1[0] * fabsf(ax) + weights1[1] * fabsf(ay) + weights1[2] * fabsf(az) + bias1;
  *fallProbability = 1.0f / (1.0f + expf(-linear));
  *activityScore = weights2[0] * fabsf(ax) + weights2[1] * fabsf(ay) + weights2[2] * fabsf(az);
}

bool initAccelerometer() {
  Wire.beginTransmission(kAccelerometerAddress);
  Wire.write(0x6B);
  Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

bool readAccelerometer(float *ax, float *ay, float *az) {
  Wire.beginTransmission(kAccelerometerAddress);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(kAccelerometerAddress, static_cast<uint8_t>(6)) != 6) {
    return false;
  }

  const int16_t rawAx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  const int16_t rawAy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  const int16_t rawAz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

  *ax = static_cast<float>(rawAx) / 16384.0f;
  *ay = static_cast<float>(rawAy) / 16384.0f;
  *az = static_cast<float>(rawAz) / 16384.0f;
  return true;
}

void selectMuxChannel(uint8_t channel) {
  for (size_t i = 0; i < 4; ++i) {
    const bool bitValue = ((channel >> i) & 0x01U) != 0U;
    digitalWrite(kMuxSelectPins[i], bitValue ? HIGH : LOW);
  }
  delayMicroseconds(10);
}

float readThermistorCelsiusFromMux(uint8_t channel) {
  if (channel > 15) {
    return NAN;
  }
  selectMuxChannel(channel);
  const int adc = analogRead(kMuxSignalPin);
  if (adc == 0 || adc >= kAdcMax) {
    return NAN;
  }

  const float resistance = (kReferenceResistorOhms * static_cast<float>(adc)) / static_cast<float>(kAdcMax - adc);
  const float nominalKelvin = kNominalTemperatureC + 273.15f;
  const float steinhart = (1.0f / nominalKelvin) + (1.0f / kThermistorBeta) * logf(resistance / kThermistorNominalOhms);
  return (1.0f / steinhart) - 273.15f;
}

float mergeThermistorValues(const float *temps, size_t count) {
  float sum = 0.0f;
  size_t validCount = 0;
  for (size_t i = 0; i < count; ++i) {
    if (!isnan(temps[i])) {
      sum += temps[i];
      ++validCount;
    }
  }
  if (validCount == 0) {
    return NAN;
  }
  return sum / static_cast<float>(validCount);
}

String buildPayload(bool accelerometerOk, float ax, float ay, float az, float fallProbability, float activityScore,
                    const float *temps, float mergedTempC) {
  String payload = "{";
  if (accelerometerOk) {
    payload += "\"accelerometer\":{";
    payload += "\"ax\":" + String(ax, 3) + ",";
    payload += "\"ay\":" + String(ay, 3) + ",";
    payload += "\"az\":" + String(az, 3) + "},";
    payload += "\"tinyml\":{";
    payload += "\"fall_probability\":" + String(fallProbability, 3) + ",";
    payload += "\"activity_score\":" + String(activityScore, 3) + "},";
  } else {
    payload += "\"accelerometer\":null,";
    payload += "\"tinyml\":null,";
  }
  payload += "\"thermistors_c\":[";
  for (size_t i = 0; i < 4; ++i) {
    payload += isnan(temps[i]) ? "null" : String(temps[i], 2);
    if (i < 3) {
      payload += ",";
    }
  }
  payload += "],";
  payload += "\"thermistors_merged_c\":";
  payload += isnan(mergedTempC) ? "null" : String(mergedTempC, 2);
  payload += "}";
  return payload;
}
} // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  const bool accelerometerOk = initAccelerometer();
  Serial.println(accelerometerOk ? "Accelerometer initialized" : "Accelerometer init failed");

  analogReadResolution(12);
  pinMode(kMuxSignalPin, INPUT);
  for (uint8_t pin : kMuxSelectPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  BLEDevice::init("OpenHestia-ESP32C3");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(kServiceUuid);
  dataCharacteristic = service->createCharacteristic(
      kDataUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  dataCharacteristic->setValue("{}");
  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  advertising->start();

  Serial.println("Bluetooth advertising started");
}

void loop() {
  static uint32_t lastUpdateMs = 0;
  const uint32_t nowMs = millis();
  if ((nowMs - lastUpdateMs) < 1000U) {
    delay(20);
    return;
  }
  lastUpdateMs = nowMs;

  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  const bool accelerometerOk = readAccelerometer(&ax, &ay, &az);

  float fallProbability = 0.0f;
  float activityScore = 0.0f;
  if (accelerometerOk) {
    tinyMlPredict(ax, ay, az, &fallProbability, &activityScore);
  } else {
    Serial.println("Accelerometer read failed");
  }

  float temps[4];
  for (size_t i = 0; i < 4; ++i) {
    temps[i] = readThermistorCelsiusFromMux(kThermistorMuxChannels[i]);
  }
  const float mergedTempC = mergeThermistorValues(temps, 4);

  const String payload = buildPayload(accelerometerOk, ax, ay, az, fallProbability, activityScore, temps, mergedTempC);
  Serial.println(payload);
  if (dataCharacteristic != nullptr) {
    dataCharacteristic->setValue(payload.c_str());
    dataCharacteristic->notify();
  }
}
