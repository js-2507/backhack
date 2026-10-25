
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ==========================================
// CHANGE THIS IF YOU WANT A NEW WIFI PASSWORD
// ==========================================
const char* AP_SSID = "Posture-ESP32";
const char* AP_PASSWORD = "Posture2026";

// Hardware pins
const int BUZZER = 23;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

int MPU = 0x68;
bool sensorReady = false;

// Posture settings
const float TILT_THRESHOLD = 20.0;
const float RESET_THRESHOLD = 12.0;
const unsigned long ALERT_DELAY = 2000;

// Buzzer settings
const int BEEP_FREQUENCY = 1000;
const int BEEP_DUTY = 5;
const int BEEP_DURATION = 15;
const int BEEP_INTERVAL = 150;

// Automatically calibrate when powered on
const bool AUTO_START = true;

WebServer server(80);

struct Vec3 {
    float x, y, z;
};

Vec3 baseline = {0, 0, 0};
Vec3 filtered = {0, 0, 0};

bool tracking = false;
bool counting = false;
bool alertActive = false;

float currentAngle = 0;

unsigned long tiltStart = 0;
unsigned long lastBeep = 0;
unsigned long lastPrint = 0;

// ==========================================
// ACCELEROMETER
// ==========================================
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

float magnitude(Vec3 v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

Vec3 normalize(Vec3 v) {
    float m = magnitude(v);

    if (m < 0.01f)
        return {0, 0, 0};

    return {v.x/m, v.y/m, v.z/m};
}

// ==========================================
// BUZZER
// ==========================================
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

void stopTracking() {
    tracking = false;
    counting = false;
    alertActive = false;
    digitalWrite(BUZZER, LOW);
}

// ==========================================
// CALIBRATION
// ==========================================
bool calibrate() {
    stopTracking();

    if (!sensorReady)
        return false;

    Serial.println("Calibrating... Hold upright!");

    Vec3 sum = {0, 0, 0};

    for (int i = 0; i < 100; i++) {
        Vec3 a;

        if (!readAccel(a)) {
            Serial.println("Calibration failed");
            return false;
        }

        sum.x += a.x;
        sum.y += a.y;
        sum.z += a.z;

        delay(10);
    }

    baseline = normalize(sum);

    if (magnitude(baseline) < 0.01f)
        return false;

    filtered = baseline;
    currentAngle = 0;

    tiltStart = 0;
    lastBeep = 0;
    tracking = true;

    Serial.println("CALIBRATED: 0 degrees");
    Serial.println("Tracking started");

    return true;
}

// ==========================================
// PHONE WEBPAGE
// ==========================================
void handleHome() {
    server.send(200, "text/html", R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport"
      content="width=device-width, initial-scale=1">
<title>Posture Monitor</title>
<style>
body {
    font-family: Arial, sans-serif;
    text-align: center;
    background: #101827;
    color: white;
    padding: 25px;
}
h1 { margin-top: 35px; }
.angle {
    font-size: 65px;
    font-weight: bold;
    margin: 30px 0 5px;
}
#status { margin: 20px; font-size: 20px; }
button {
    background: #38bdf8;
    border: none;
    padding: 15px 22px;
    margin: 8px;
    border-radius: 12px;
    font-size: 16px;
}
</style>
</head>
<body>

<h1>Posture Monitor</h1>

<div class="angle" id="angle">--°</div>
<div id="status">Connecting...</div>

<button onclick="send('/start')">
Start / Calibrate
</button>

<button onclick="send('/stop')">
Pause
</button>

<p id="sensor"></p>

<script>
async function send(path) {
    try {
        await fetch(path, {method: 'POST'});
        update();
    } catch(e) {
        document.getElementById('status').innerText =
            'Connection error';
    }
}

async function update() {
    try {
        const r = await fetch('/status');
        const d = await r.json();

        document.getElementById('angle').innerText =
            d.angle.toFixed(1) + '°';

        document.getElementById('status').innerText =
            !d.tracking ? 'PAUSED' :
            d.alert ? 'POSTURE ALERT' : 'MONITORING';

        document.getElementById('sensor').innerText =
            d.sensor ? 'Sensor connected' :
                       'Sensor unavailable';
    } catch(e) {
        document.getElementById('status').innerText =
            'Connection lost';
    }
}

setInterval(update, 500);
update();
</script>
</body>
</html>
)HTML");
}

void handleStatus() {
    String json = "{";

    json += "\"angle\":";
    json += String(currentAngle, 2);

    json += ",\"tracking\":";
    json += tracking ? "true" : "false";

    json += ",\"alert\":";
    json += alertActive ? "true" : "false";

    json += ",\"sensor\":";
    json += sensorReady ? "true" : "false";

    json += "}";

    server.send(200, "application/json", json);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    delay(500);

    // Detect MPU6050
    Wire.beginTransmission(0x68);

    if (Wire.endTransmission() == 0) {
        MPU = 0x68;
        sensorReady = true;
    } else {
        Wire.beginTransmission(0x69);

        if (Wire.endTransmission() == 0) {
            MPU = 0x69;
            sensorReady = true;
        }
    }

    if (sensorReady) {
        Wire.beginTransmission(MPU);
        Wire.write(0x6B);
        Wire.write(0);

        if (Wire.endTransmission() != 0)
            sensorReady = false;
    }

    if (!sensorReady)
        Serial.println("MPU6050 NOT FOUND!");

    // Create ESP32's own Wi-Fi network
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),
        IPAddress(192, 168, 4, 1),
        IPAddress(255, 255, 255, 0)
    );

    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.println("Wi-Fi network started");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("Wi-Fi startup failed");
    }

    // Webpage routes
    server.on("/", HTTP_GET, handleHome);
    server.on("/status", HTTP_GET, handleStatus);

    server.on("/start", HTTP_POST, []() {
        bool success = calibrate();
        server.send(
            success ? 200 : 500,
            "text/plain",
            success ? "Calibrated" : "Calibration failed"
        );
    });

    server.on("/stop", HTTP_POST, []() {
        stopTracking();
        server.send(200, "text/plain", "Paused");
    });

    server.begin();

    // Automatic calibration after startup
    if (AUTO_START && sensorReady) {
        Serial.println("Starting in 3 seconds...");
        Serial.println("Hold sensor upright.");
        delay(3000);
        calibrate();
    }

    Serial.println("POSTURE DETECTOR READY");
    Serial.println("Wi-Fi: Posture-ESP32");
    Serial.println("Open: http://192.168.4.1");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    // Phone requests are handled continuously
    server.handleClient();

    if (!tracking) {
        delay(20);
        return;
    }

    Vec3 a;

    if (!readAccel(a)) {
        stopTracking();
        Serial.println("Sensor disconnected!");
        return;
    }

    a = normalize(a);

    if (magnitude(a) < 0.01f) {
        stopTracking();
        return;
    }

    // Smooth acceleration readings
    filtered.x = 0.8f * filtered.x + 0.2f * a.x;
    filtered.y = 0.8f * filtered.y + 0.2f * a.y;
    filtered.z = 0.8f * filtered.z + 0.2f * a.z;

    filtered = normalize(filtered);

    // Calculate tilt angle from baseline
    float dot =
        baseline.x * filtered.x +
        baseline.y * filtered.y +
        baseline.z * filtered.z;

    dot = constrain(dot, -1.0f, 1.0f);

    currentAngle = acos(dot) * 180.0f / PI;

    unsigned long now = millis();
    bool newAlert = false;

    // Detect sustained tilt
    if (currentAngle > TILT_THRESHOLD) {
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
    else if (currentAngle < RESET_THRESHOLD) {
        counting = false;
        alertActive = false;
    }
    else if (!alertActive) {
        counting = false;
    }

    // Repeated short beeps
    if (alertActive &&
        (newAlert ||
         now - lastBeep >= BEEP_INTERVAL)) {
        beep();
        lastBeep = millis();
    }

    if (now - lastPrint >= 500) {
        Serial.printf(
            "Angle: %.1f | Alert: %s\n",
            currentAngle,
            alertActive ? "ON" : "OFF"
        );

        lastPrint = now;
    }

    delay(20);
}

// #include <Arduino.h>
// #include <Wire.h>
// #include <math.h>

// // Pin configuration
// const int BUZZER = 23;       // GPIO18
// const int SDA_PIN = 21;
// const int SCL_PIN = 22;

// int MPU = 0x68;

// // Posture detection settings
// const float TILT_THRESHOLD = 20.0;
// const float RESET_THRESHOLD = 12.0;
// const unsigned long ALERT_DELAY = 2000;

// // Buzzer settings
// const int BEEP_FREQUENCY = 1000;  // Hz
// const int BEEP_DUTY = 5;        // Lower = potentially softer
// const int BEEP_DURATION = 15;    // ms
// const int BEEP_INTERVAL = 150;   // ms

// struct Vec3 {
//     float x, y, z;
// };

// Vec3 baseline;
// Vec3 filtered;

// bool tracking = false;
// bool counting = false;
// bool alertActive = false;

// unsigned long tiltStart = 0;
// unsigned long lastBeep = 0;
// unsigned long lastPrint = 0;

// // Read MPU-6050 acceleration
// bool readAccel(Vec3 &a) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x3B);

//     if (Wire.endTransmission(false) != 0)
//         return false;

//     if (Wire.requestFrom(MPU, 6, true) != 6)
//         return false;

//     int16_t x = (Wire.read() << 8) | Wire.read();
//     int16_t y = (Wire.read() << 8) | Wire.read();
//     int16_t z = (Wire.read() << 8) | Wire.read();

//     a = {
//         x / 16384.0f,
//         y / 16384.0f,
//         z / 16384.0f
//     };

//     return true;
// }

// // Normalize acceleration vector
// Vec3 normalize(Vec3 v) {
//     float m = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

//     if (m < 0.01f)
//         return {0, 0, 0};

//     return {v.x/m, v.y/m, v.z/m};
// }

// // Generate short, low-duty-cycle beep
// void beep() {
//     int period = 1000000 / BEEP_FREQUENCY;
//     int onTime = period * BEEP_DUTY / 100;
//     int offTime = period - onTime;
//     int cycles = BEEP_DURATION * 1000 / period;

//     for (int i = 0; i < cycles; i++) {
//         digitalWrite(BUZZER, HIGH);
//         delayMicroseconds(onTime);

//         digitalWrite(BUZZER, LOW);
//         delayMicroseconds(offTime);
//     }

//     digitalWrite(BUZZER, LOW);
// }

// // Calibrate current position to 0 degrees
// void calibrate() {
//     tracking = false;
//     counting = false;
//     alertActive = false;
//     digitalWrite(BUZZER, LOW);

//     Serial.println("Calibrating... Keep sensor upright!");

//     Vec3 sum = {0, 0, 0};

//     for (int i = 0; i < 100; i++) {
//         Vec3 a;

//         if (!readAccel(a)) {
//             Serial.println("Calibration failed!");
//             return;
//         }

//         sum.x += a.x;
//         sum.y += a.y;
//         sum.z += a.z;

//         delay(10);
//     }

//     baseline = normalize(sum);

//     if (baseline.x == 0 &&
//         baseline.y == 0 &&
//         baseline.z == 0) {
//         Serial.println("Invalid calibration!");
//         return;
//     }

//     filtered = baseline;

//     tiltStart = 0;
//     lastBeep = 0;
//     tracking = true;

//     Serial.println("CALIBRATED: 0 degrees");
//     Serial.println("Posture tracking started!");
//     Serial.println("Type R to recalibrate.");
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin(SDA_PIN, SCL_PIN);

//     pinMode(BUZZER, OUTPUT);
//     digitalWrite(BUZZER, LOW);

//     delay(1000);

//     // Detect MPU-6050 address
//     Wire.beginTransmission(0x68);

//     if (Wire.endTransmission() != 0) {
//         MPU = 0x69;
//         Wire.beginTransmission(MPU);

//         if (Wire.endTransmission() != 0) {
//             Serial.println("MPU6050 NOT FOUND!");
//             while (true) delay(1000);
//         }
//     }

//     // Wake accelerometer
//     Wire.beginTransmission(MPU);
//     Wire.write(0x6B);
//     Wire.write(0x00);

//     if (Wire.endTransmission() != 0) {
//         Serial.println("MPU wake failed!");
//         while (true) delay(1000);
//     }

//     Serial.println("-------------------------");
//     Serial.println("POSTURE DETECTOR READY");
//     Serial.println("Sit upright.");
//     Serial.println("Type S + Enter to START");
//     Serial.println("-------------------------");
// }

// void loop() {
//     // Handle start and recalibration commands
//     while (Serial.available()) {
//         char command = Serial.read();

//         if (command == 'S' || command == 's' ||
//             command == 'R' || command == 'r') {
//             calibrate();
//         }
//     }

//     if (!tracking) {
//         delay(20);
//         return;
//     }

//     Vec3 a;

//     if (!readAccel(a)) {
//         counting = false;
//         alertActive = false;
//         digitalWrite(BUZZER, LOW);
//         Serial.println("Sensor read failed");
//         delay(100);
//         return;
//     }

//     a = normalize(a);

//     float magnitude = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);

//     if (magnitude < 0.01f) {
//         counting = false;
//         alertActive = false;
//         digitalWrite(BUZZER, LOW);
//         delay(100);
//         return;
//     }

//     // Smooth acceleration readings
//     filtered.x = 0.8f*filtered.x + 0.2f*a.x;
//     filtered.y = 0.8f*filtered.y + 0.2f*a.y;
//     filtered.z = 0.8f*filtered.z + 0.2f*a.z;
//     filtered = normalize(filtered);

//     // Calculate angle relative to calibrated position
//     float dot =
//         baseline.x * filtered.x +
//         baseline.y * filtered.y +
//         baseline.z * filtered.z;

//     dot = constrain(dot, -1.0f, 1.0f);

//     float angle = acos(dot) * 180.0f / PI;

//     unsigned long now = millis();
//     bool newAlert = false;

//     // Detect sustained tilt
//     if (angle > TILT_THRESHOLD) {
//         if (!counting) {
//             counting = true;
//             tiltStart = now;
//         }

//         if (!alertActive &&
//             now - tiltStart >= ALERT_DELAY) {
//             alertActive = true;
//             newAlert = true;
//         }
//     }
//     else if (angle < RESET_THRESHOLD) {
//         counting = false;
//         alertActive = false;
//     }
//     else if (!alertActive) {
//         counting = false;
//     }

//     // Repeat beep every 250 ms while tilted
//     if (alertActive &&
//         (newAlert || now - lastBeep >= BEEP_INTERVAL)) {
//         beep();
//         lastBeep = millis();
//     }

//     // Print angle and alert status
//     if (now - lastPrint >= 200) {
//         Serial.printf(
//             "Angle: %.1f | Alert: %s\n",
//             angle,
//             alertActive ? "ON" : "OFF"
//         );

//         lastPrint = now;
//     }

//     delay(20);
// }
