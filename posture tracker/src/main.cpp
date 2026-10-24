#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

Adafruit_MPU6050 mpu;

const int BUZZER_PIN = 23;           // Connected directly to Active Buzzer I/O (SIG) pin
float baselinePitch = 0;             // Calibrated upright posture angle
const float SLOUCH_THRESHOLD = 15.0; // Degrees forward tilt to trigger warning

// BLE UUIDs for the service and characteristic
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aae"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool sensorInitialized = false;

// Callback class to monitor BLE connection state
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnected(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Mobile device connected via BLE.");
    }
    void onDisconnected(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Mobile device disconnected. Restarting advertising...");
        pServer->startAdvertising(); // Resume advertising so phones can reconnect
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer starts silent

  // Explicitly initialize I2C pins for ESP32 (SDA = GPIO 21, SCL = GPIO 22)
  Wire.begin(21, 22);

  // 1. START BLE FIRST so the board is always discoverable on your phone
  BLEDevice::init("PostureTracker_ESP32");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // iPhone connection stability helper
  BLEDevice::startAdvertising();
  Serial.println("BLE Server Active and Advertising!");

  // 2. Initialize MPU6050 safely without locking up the board if wiring is loose
  if (!mpu.begin()) {
    Serial.println("WARNING: MPU6050 not found! Check wiring (SDA=21, SCL=22, 3V3, GND).");
    sensorInitialized = false;
  } else {
    Serial.println("MPU6050 Sensor Found Successfully!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensorInitialized = true;
  }

  Serial.println("Calibrating baseline in 3 seconds... Sit up straight!");
  delay(3000);

  // Take initial calibration reading if sensor is active
  if (sensorInitialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    baselinePitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    Serial.print("Baseline Pitch Set: ");
    Serial.println(baselinePitch);
  } else {
    baselinePitch = 0.0;
    Serial.println("Using default baseline (0.0) because sensor is offline.");
  }
}

void loop() {
  float pitchDelta = 0.0;
  bool isSlouching = false;

  if (sensorInitialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate pitch angle from accelerometer data
    float currentPitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    pitchDelta = currentPitch - baselinePitch;

    // Check if user is slouching past the threshold angle
    isSlouching = (pitchDelta > SLOUCH_THRESHOLD);

    if (isSlouching) {
      digitalWrite(BUZZER_PIN, HIGH); // Sound active buzzer alarm
    } else {
      digitalWrite(BUZZER_PIN, LOW);  // Silence buzzer
    }

    Serial.print("Pitch Delta: ");
    Serial.println(pitchDelta);
  } else {
    Serial.println("Sensor offline - check I2C connections.");
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Broadcast data over BLE if a phone/tablet is connected
  if (deviceConnected) {
    String payload = sensorInitialized ? (String(pitchDelta, 1) + (isSlouching ? ",SLOUCH" : ",OK")) : "ERROR,SENSOR_OFFLINE";
    pCharacteristic->setValue(payload.c_str());
    pCharacteristic->notify();
  }

  delay(500); // Update twice per second
}