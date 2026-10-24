
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Pin configuration
const int BUZZER = 18;       // GPIO18
const int SDA_PIN = 21;
const int SCL_PIN = 22;

int MPU = 0x68;

// Posture detection settings
const float TILT_THRESHOLD = 20.0;
const float RESET_THRESHOLD = 12.0;
const unsigned long ALERT_DELAY = 2000;

// Buzzer settings
const int BEEP_FREQUENCY = 1000;  // Hz
const int BEEP_DUTY = 10;        // Lower = potentially softer
const int BEEP_DURATION = 50;    // ms
const int BEEP_INTERVAL = 250;   // ms

struct Vec3 {
    float x, y, z;
};

Vec3 baseline;
Vec3 filtered;

bool tracking = false;
bool counting = false;
bool alertActive = false;

unsigned long tiltStart = 0;
unsigned long lastBeep = 0;
unsigned long lastPrint = 0;

// Read MPU-6050 acceleration
bool readAccel(Vec3 &a) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(MPU, 6, true) != 6)
        return false;

    int16_t x = (Wire.read() << 8) | Wire.read();
    int16_t y = (Wire.read() << 8) | Wire.read();
    int16_t z = (Wire.read() << 8) | Wire.read();

    a = {
        x / 16384.0f,
        y / 16384.0f,
        z / 16384.0f
    };

    return true;
}

// Normalize acceleration vector
Vec3 normalize(Vec3 v) {
    float m = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

    if (m < 0.01f)
        return {0, 0, 0};

    return {v.x/m, v.y/m, v.z/m};
}

// Generate short, low-duty-cycle beep
void beep() {
    int period = 1000000 / BEEP_FREQUENCY;
    int onTime = period * BEEP_DUTY / 100;
    int offTime = period - onTime;
    int cycles = BEEP_DURATION * 1000 / period;

    for (int i = 0; i < cycles; i++) {
        digitalWrite(BUZZER, HIGH);
        delayMicroseconds(onTime);

        digitalWrite(BUZZER, LOW);
        delayMicroseconds(offTime);
    }

    digitalWrite(BUZZER, LOW);
}

// Calibrate current position to 0 degrees
void calibrate() {
    tracking = false;
    counting = false;
    alertActive = false;
    digitalWrite(BUZZER, LOW);

    Serial.println("Calibrating... Keep sensor upright!");

    Vec3 sum = {0, 0, 0};

    for (int i = 0; i < 100; i++) {
        Vec3 a;

        if (!readAccel(a)) {
            Serial.println("Calibration failed!");
            return;
        }

        sum.x += a.x;
        sum.y += a.y;
        sum.z += a.z;

        delay(10);
    }

    baseline = normalize(sum);

    if (baseline.x == 0 &&
        baseline.y == 0 &&
        baseline.z == 0) {
        Serial.println("Invalid calibration!");
        return;
    }

    filtered = baseline;

    tiltStart = 0;
    lastBeep = 0;
    tracking = true;

    Serial.println("CALIBRATED: 0 degrees");
    Serial.println("Posture tracking started!");
    Serial.println("Type R to recalibrate.");
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    delay(1000);

    // Detect MPU-6050 address
    Wire.beginTransmission(0x68);

    if (Wire.endTransmission() != 0) {
        MPU = 0x69;
        Wire.beginTransmission(MPU);

        if (Wire.endTransmission() != 0) {
            Serial.println("MPU6050 NOT FOUND!");
            while (true) delay(1000);
        }
    }

    // Wake accelerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);

    if (Wire.endTransmission() != 0) {
        Serial.println("MPU wake failed!");
        while (true) delay(1000);
    }

    Serial.println("-------------------------");
    Serial.println("POSTURE DETECTOR READY");
    Serial.println("Sit upright.");
    Serial.println("Type S + Enter to START");
    Serial.println("-------------------------");
}

void loop() {
    // Handle start and recalibration commands
    while (Serial.available()) {
        char command = Serial.read();

        if (command == 'S' || command == 's' ||
            command == 'R' || command == 'r') {
            calibrate();
        }
    }

    if (!tracking) {
        delay(20);
        return;
    }

    Vec3 a;

    if (!readAccel(a)) {
        counting = false;
        alertActive = false;
        digitalWrite(BUZZER, LOW);
        Serial.println("Sensor read failed");
        delay(100);
        return;
    }

    a = normalize(a);

    float magnitude = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);

    if (magnitude < 0.01f) {
        counting = false;
        alertActive = false;
        digitalWrite(BUZZER, LOW);
        delay(100);
        return;
    }

    // Smooth acceleration readings
    filtered.x = 0.8f*filtered.x + 0.2f*a.x;
    filtered.y = 0.8f*filtered.y + 0.2f*a.y;
    filtered.z = 0.8f*filtered.z + 0.2f*a.z;
    filtered = normalize(filtered);

    // Calculate angle relative to calibrated position
    float dot =
        baseline.x * filtered.x +
        baseline.y * filtered.y +
        baseline.z * filtered.z;

    dot = constrain(dot, -1.0f, 1.0f);

    float angle = acos(dot) * 180.0f / PI;

    unsigned long now = millis();
    bool newAlert = false;

    // Detect sustained tilt
    if (angle > TILT_THRESHOLD) {
        if (!counting) {
            counting = true;
            tiltStart = now;
        }

        if (!alertActive &&
            now - tiltStart >= ALERT_DELAY) {
            alertActive = true;
            newAlert = true;
        }
    }
    else if (angle < RESET_THRESHOLD) {
        counting = false;
        alertActive = false;
    }
    else if (!alertActive) {
        counting = false;
    }

    // Repeat beep every 250 ms while tilted
    if (alertActive &&
        (newAlert || now - lastBeep >= BEEP_INTERVAL)) {
        beep();
        lastBeep = millis();
    }

    // Print angle and alert status
    if (now - lastPrint >= 200) {
        Serial.printf(
            "Angle: %.1f | Alert: %s\n",
            angle,
            alertActive ? "ON" : "OFF"
        );

        lastPrint = now;
    }

    delay(20);
}
