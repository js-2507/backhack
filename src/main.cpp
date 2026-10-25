#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ==========================================
// WIFI SETTINGS
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
const float TILT_THRESHOLD = 10.0;
const float RESET_THRESHOLD = 7.0;
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

// Latest camera results received from the laptop
String cameraState = "OFFLINE";
float cameraHead = 0;
float cameraShoulder = 0;
bool cameraSeen = false;
unsigned long cameraUpdated = 0;

bool cameraFresh() {
    return cameraSeen && millis() - cameraUpdated < 1500;
}

String getCameraJson() {
    bool online = cameraFresh();
    bool hasAngles = online &&
        (cameraState == "WITHIN_RANGE" ||
         cameraState == "CHANGED" ||
         cameraState == "ALERT");

    String json = "{\"online\":";
    json += online ? "true" : "false";

    json += ",\"state\":\"";
    json += online ? cameraState : "OFFLINE";
    json += "\"";

    json += ",\"head\":";
    json += hasAngles ? String(cameraHead, 1) : "null";

    json += ",\"shoulder\":";
    json += hasAngles ? String(cameraShoulder, 1) : "null";

    json += "}";
    return json;
}

String getAgreementState() {
    bool cameraTracking = cameraFresh() &&
        (cameraState == "WITHIN_RANGE" ||
         cameraState == "CHANGED" ||
         cameraState == "ALERT");

    bool wearableTracking = tracking && sensorReady;

    if (!cameraTracking || !wearableTracking) {
        return "UNAVAILABLE";
    }

    if (alertActive && cameraState == "ALERT") {
        return "CONFIRMED";
    }

    bool wearableChanged =
        alertActive || counting || currentAngle > TILT_THRESHOLD;

    bool cameraChanged =
        cameraState == "CHANGED" || cameraState == "ALERT";

    if (!wearableChanged && !cameraChanged) {
        return "WITHIN_RANGE";
    }

    if (wearableChanged && cameraChanged) {
        return "WAITING";
    }

    return "DISAGREEMENT";
}

bool wearableMayBeep() {
    bool cameraTracking = cameraFresh() &&
        (cameraState == "WITHIN_RANGE" ||
         cameraState == "CHANGED" ||
         cameraState == "ALERT");

    return !cameraTracking || cameraState == "ALERT";
}

bool cameraMayBeep() {
    bool wearableTracking = tracking && sensorReady;
    return !wearableTracking || alertActive;
}

bool cameraTrackingReady() {
    return cameraFresh() &&
        (cameraState == "WITHIN_RANGE" ||
         cameraState == "CHANGED" ||
         cameraState == "ALERT");
}

bool soundAlertActive() {
    bool wearableTracking = tracking && sensorReady;
    bool cameraTracking = cameraTrackingReady();
    if (wearableTracking && cameraTracking) {
        return alertActive && cameraState == "ALERT";
    }
    if (wearableTracking) return alertActive;
    if (cameraTracking) return cameraState == "ALERT";
    return false;
}

String getSoundMode() {
    bool wearableTracking = tracking && sensorReady;
    bool cameraTracking = cameraTrackingReady();
    if (wearableTracking && cameraTracking) return "AGREEMENT";
    if (wearableTracking) return "WEARABLE_ONLY";
    if (cameraTracking) return "CAMERA_ONLY";
    return "NO_TRACKING";
}

// Forward declarations
bool calibrate();
void stopTracking();

// ==========================================
// ACCELEROMETER
// ==========================================
bool readAccel(Vec3 &a) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom((uint8_t)MPU, (size_t)6, true) != 6)
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

// ==========================================
// STATUS JSON HELPER
// ==========================================
String getStatusJson() {
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
    return json;
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
// WI-FI WEBPAGE
// ==========================================
const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Posture Monitor</title>
<style>
:root {
  --bg-color: #0b1120;
  --card-bg: #1e293b;
  --text-primary: #f8fafc;
  --text-secondary: #94a3b8;
  --accent-cyan: #38bdf8;
  --accent-blue: #3b82f6;
  --status-ok: #10b981;
  --status-alert: #ef4444;
  --status-paused: #64748b;
  --border-color: #334155;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
  background-color: var(--bg-color);
  color: var(--text-primary);
  display: flex;
  flex-direction: column;
  align-items: center;
  min-height: 100vh;
  padding: 16px;
}
.app-container {
  width: 100%;
  max-width: 440px;
  display: flex;
  flex-direction: column;
  gap: 16px;
}
header {
  text-align: center;
  padding: 12px 0 4px;
}
h1 {
  font-size: 26px;
  font-weight: 700;
  letter-spacing: -0.5px;
}
.subtitle {
  color: var(--text-secondary);
  font-size: 13px;
  margin-top: 4px;
}
.card {
  background: var(--card-bg);
  border: 1px solid var(--border-color);
  border-radius: 16px;
  padding: 18px;
  box-shadow: 0 4px 16px rgba(0,0,0,0.3);
}
.conn-status-bar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: rgba(15, 23, 42, 0.6);
  border-radius: 10px;
  padding: 10px 14px;
  font-size: 13px;
}
.dot {
  width: 9px;
  height: 9px;
  border-radius: 50%;
  display: inline-block;
  margin-right: 6px;
}
.dot-green { background: var(--status-ok); box-shadow: 0 0 8px var(--status-ok); }
.dot-blue { background: var(--accent-blue); box-shadow: 0 0 8px var(--accent-blue); }
.dot-gray { background: var(--status-paused); }
.dot-red { background: var(--status-alert); box-shadow: 0 0 8px var(--status-alert); }

.gauge-card {
  text-align: center;
  padding: 28px 18px;
  display: flex;
  flex-direction: column;
  align-items: center;
}
.gauge-ring {
  width: 210px;
  height: 210px;
  border-radius: 50%;
  border: 8px solid var(--status-paused);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  margin-bottom: 20px;
  transition: border-color 0.3s ease;
  position: relative;
}
.angle-num {
  font-size: 64px;
  font-weight: 800;
  line-height: 1;
}
.angle-label {
  font-size: 13px;
  text-transform: uppercase;
  color: var(--text-secondary);
  letter-spacing: 1.5px;
  margin-top: 4px;
}
.status-badge {
  display: inline-block;
  padding: 6px 18px;
  border-radius: 20px;
  font-size: 14px;
  font-weight: 700;
  letter-spacing: 1px;
}
.status-ok { background: rgba(16, 185, 129, 0.15); color: var(--status-ok); border: 1px solid rgba(16, 185, 129, 0.3); }
.status-alert {
  background: rgba(239, 68, 68, 0.2);
  color: var(--status-alert);
  border: 1px solid var(--status-alert);
  animation: pulse 1s infinite alternate;
}
@keyframes pulse {
  0% { transform: scale(1); }
  100% { transform: scale(1.05); }
}
.status-paused { background: rgba(100, 116, 139, 0.15); color: var(--status-paused); border: 1px solid var(--status-paused); }

.btn-group {
  display: flex;
  gap: 12px;
  width: 100%;
}
button.action-btn {
  flex: 1;
  padding: 15px;
  border-radius: 12px;
  border: none;
  font-size: 15px;
  font-weight: 700;
  cursor: pointer;
  transition: transform 0.1s, opacity 0.2s;
}
button.action-btn:active { transform: scale(0.98); }
.btn-primary {
  background: linear-gradient(135deg, #38bdf8 0%, #2563eb 100%);
  color: white;
}
.btn-secondary {
  background: #334155;
  color: #f1f5f9;
}
.info-row {
  display: flex;
  justify-content: space-between;
  font-size: 12px;
  color: var(--text-secondary);
  padding: 6px 0;
}
.sensor-ok { color: var(--status-ok); font-weight: 600; }
.sensor-err { color: var(--status-alert); font-weight: 600; }

.help-box {
  background: rgba(15, 23, 42, 0.7);
  border-radius: 12px;
  padding: 12px;
  font-size: 12px;
  color: var(--text-secondary);
  line-height: 1.5;
}
.help-box summary {
  cursor: pointer;
  font-weight: 600;
  color: var(--accent-cyan);
}
.help-box p, .help-box ul { margin-top: 8px; }
.help-box ul { padding-left: 18px; }
</style>
</head>
<body>

<div class="app-container">
  <header>
    <h1>Posture Monitor</h1>
    <div class="subtitle">Wi-Fi Posture Monitoring</div>
  </header>

  <!-- Wi-Fi connection -->
  <div class="card">
    <div class="conn-status-bar">
      <span id="conn-label"><span class="dot dot-gray" id="conn-dot"></span>Connecting...</span>
      <span style="color:var(--text-secondary); font-size:11px;">Wi-Fi</span>
    </div>
  </div>

  <!-- Posture Readout -->
  <div class="card gauge-card">
    <div class="gauge-ring" id="gauge-ring">
      <div class="angle-num" id="angle">--°</div>
      <div class="angle-label">Tilt Angle</div>
    </div>
    <div class="status-badge status-paused" id="status">CONNECTING</div>
  </div>

  <!-- Control Buttons -->
  <div class="btn-group">
    <button class="action-btn btn-primary" onclick="handleStart()">Start / Calibrate</button>
    <button class="action-btn btn-secondary" onclick="handleStop()">Pause</button>
  </div>

  <!-- Camera readings -->
  <div class="card">
    <h2 style="font-size:18px; margin-bottom:12px;">
      Laptop Camera
    </h2>

    <div id="camera-status"
        class="status-badge status-paused">
      OFFLINE
    </div>

    <div class="info-row" style="margin-top:12px;">
      <span>Head-line change</span>
      <span id="camera-head">--</span>
    </div>

    <div class="info-row">
      <span>Shoulder-line change</span>
      <span id="camera-shoulder">--</span>
    </div>

    <p style="font-size:12px; color:var(--text-secondary);
              margin-top:10px;">
      Run the camera Python program on your laptop.
      Press C in its preview to recalibrate.
    </p>
  </div>

  <!-- Combined assessment -->
  <div class="card">
    <h2 style="font-size:18px; margin-bottom:12px;">
      Camera + Wearable Agreement
    </h2>
    <div id="agreement-status" class="status-badge status-paused"
         role="status" aria-live="polite">CHECKING</div>
    <p id="agreement-detail"
       style="font-size:12px; color:var(--text-secondary);
              margin-top:12px; line-height:1.5;">
      Waiting for both systems.
    </p>
    <p style="font-size:12px; color:var(--text-secondary);
              margin-top:10px; line-height:1.5;">
      Both tracking: sound requires agreement. One available: independent alerts.
    </p>
  </div>

  <!-- Diagnostics -->
  <div class="card">
    <div class="info-row">
      <span>Hardware Sensor</span>
      <span id="sensor" class="sensor-ok">Checking...</span>
    </div>
    <div class="info-row">
      <span>Active Connection</span>
      <span id="active-link">Wi-Fi (AP)</span>
    </div>
    <div class="info-row">
      <span>Alert Threshold</span>
      <span>10.0° (Reset: 7.0°)</span>
    </div>
  </div>

  <!-- Help & Tips -->
  <details class="help-box">
    <summary>Wi-Fi Connection Guide</summary>
    <p>Connect your laptop to <code>Posture-ESP32</code>
       (Password: <code>Posture2026</code>) and open
       <code>http://192.168.4.1</code>.</p>
    <p>Run the camera Python program on that laptop for camera readings
       and laptop sound. Keep it connected to the ESP32's Wi-Fi.</p>
  </details>
</div>

<script>
function getWifiBaseUrl() {
  if (window.location.protocol.startsWith('http') && window.location.hostname !== '') {
    return '';
  }
  return 'http://192.168.4.1';
}

function updateConnStatus(text, color) {
  const dot = document.getElementById('conn-dot');
  if (dot) dot.className = 'dot dot-' + color;
  const label = document.getElementById('conn-label');
  if (label) label.innerHTML = '<span class="dot dot-' + color + '"></span>' + text;
}

function parseAndApplyData(data) {
  let d = data;
  if (typeof d === 'string') {
    try {
      d = JSON.parse(d.trim());
    } catch(e) {
      return;
    }
  }
  const angle = (typeof d.angle === 'number') ? d.angle : 0;
  const isTracking = Boolean(d.tracking);
  const isAlert = Boolean(d.alert);
  const hasSensor = Boolean(d.sensor);

  document.getElementById('angle').innerText = angle.toFixed(1) + '°';

  const statusEl = document.getElementById('status');
  const ringEl = document.getElementById('gauge-ring');

  if (!isTracking) {
    statusEl.innerText = 'PAUSED';
    statusEl.className = 'status-badge status-paused';
    ringEl.style.borderColor = 'var(--status-paused)';
  } else if (isAlert) {
    statusEl.innerText = 'POSTURE ALERT';
    statusEl.className = 'status-badge status-alert';
    ringEl.style.borderColor = 'var(--status-alert)';
  } else {
    statusEl.innerText = 'MONITORING';
    statusEl.className = 'status-badge status-ok';
    ringEl.style.borderColor = 'var(--status-ok)';
  }

  const sensorEl = document.getElementById('sensor');
  sensorEl.innerText = hasSensor ? '● Sensor Connected (MPU-6050)' : '▲ Sensor Unavailable';
  sensorEl.className = hasSensor ? 'sensor-ok' : 'sensor-err';
}

// Wi-Fi polling
async function updateWifi() {
  try {
    const res = await fetch(getWifiBaseUrl() + '/status', { cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const data = await res.json();
    parseAndApplyData(data);
    updateConnStatus('Wi-Fi Connected', 'green');
    document.getElementById('active-link').innerText = 'Wi-Fi (AP 192.168.4.1)';
  } catch(e) {
    updateConnStatus('Wi-Fi Disconnected', 'red');
    document.getElementById('active-link').innerText = 'Wi-Fi (Unavailable)';
  }
}

async function handleStart() {
  await sendCommand('start');
}

async function handleStop() {
  await sendCommand('stop');
}

async function sendCommand(cmd) {
  try {
    const response = await fetch(getWifiBaseUrl() + '/' + cmd, {
      method: 'POST'
    });
    if (!response.ok) throw new Error('HTTP ' + response.status);
    await updateWifi();
  } catch (error) {
    console.error('Wi-Fi command failed:', error);
    updateConnStatus('Command failed: check Wi-Fi connection', 'red');
  }
}

window.addEventListener('DOMContentLoaded', () => {
  updateWifi();
  setInterval(updateWifi, 500);
});

let cameraPollBusy = false;

async function updateCamera() {
  if (cameraPollBusy) return;
  cameraPollBusy = true;

  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 1500);

  try {
    const response = await fetch(
      getWifiBaseUrl() + '/camera/status',
      {cache: 'no-store', signal: controller.signal}
    );

    if (!response.ok) throw new Error('Camera request failed');

    const data = await response.json();
    const status = document.getElementById('camera-status');

    status.textContent = data.online
      ? data.state.replaceAll('_', ' ')
      : 'OFFLINE';

    status.className = 'status-badge ' + (
      !data.online ||
      data.state === 'NO_TRACKING' ||
      data.state === 'CALIBRATING'
        ? 'status-paused'
        : data.state === 'ALERT'
          ? 'status-alert'
          : 'status-ok'
    );

    document.getElementById('camera-head').textContent =
      typeof data.head === 'number'
        ? data.head.toFixed(1) + '°'
        : '--';

    document.getElementById('camera-shoulder').textContent =
      typeof data.shoulder === 'number'
        ? data.shoulder.toFixed(1) + '°'
        : '--';

  } catch (error) {
    const status = document.getElementById('camera-status');
    status.textContent = 'CONNECTION LOST';
    status.className = 'status-badge status-paused';

    document.getElementById('camera-head').textContent = '--';
    document.getElementById('camera-shoulder').textContent = '--';

  } finally {
    clearTimeout(timeout);
    cameraPollBusy = false;
  }
}

window.addEventListener('DOMContentLoaded', () => {
  updateCamera();
  setInterval(updateCamera, 500);
});


let agreementPollBusy = false;

function renderAgreement(state) {
  const states = {
    CONFIRMED: ['CONFIRMED POSTURE CHANGE', 'status-alert',
      'Both systems detected a sustained posture change.'],
    WITHIN_RANGE: ['BOTH WITHIN RANGE', 'status-ok',
      'Both systems report posture within their calibrated ranges.'],
    WAITING: ['CONFIRMING CHANGE', 'status-paused',
      'Both detect a change. Waiting for their alert delays.'],
    DISAGREEMENT: ['WAITING FOR AGREEMENT', 'status-paused',
      'Only one system detects a change. Combined sound is withheld.'],
    UNAVAILABLE: ['INSUFFICIENT DATA', 'status-paused',
      'Both must be tracking for agreement. An available system can alert independently.'],
    DISCONNECTED: ['CONNECTION LOST', 'status-paused',
      'Unable to retrieve the combined assessment from the ESP32.']
  };
  const result = states[state] || states.UNAVAILABLE;
  const badge = document.getElementById('agreement-status');
  badge.textContent = result[0];
  badge.className = 'status-badge ' + result[1];
  document.getElementById('agreement-detail').textContent = result[2];
}

async function updateAgreement() {
  if (agreementPollBusy) return;
  agreementPollBusy = true;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 1500);
  try {
    const response = await fetch(getWifiBaseUrl() + '/agreement/status', {
      cache: 'no-store', signal: controller.signal
    });
    if (!response.ok) throw new Error('Agreement request failed');
    const data = await response.json();
    renderAgreement(data.state);
    if (typeof data.mode === 'string') {
      document.getElementById('agreement-detail').textContent +=
        ' Sound mode: ' + data.mode.replaceAll('_', ' ') + '.';
    }
  } catch (error) {
    renderAgreement('DISCONNECTED');
  } finally {
    clearTimeout(timeout);
    agreementPollBusy = false;
  }
}

window.addEventListener('DOMContentLoaded', () => {
  updateAgreement();
  setInterval(updateAgreement, 500);
});

</script>
</body>
</html>
)HTML";

void handleHome() {
    server.send(200, "text/html", PAGE_HTML);
}

void handleStatus() {
    server.send(200, "application/json", getStatusJson());
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

    // 1. Create ESP32's own Wi-Fi network
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),
        IPAddress(192, 168, 4, 1),
        IPAddress(255, 255, 255, 0)
    );

    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.println("Wi-Fi AP started");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("Wi-Fi AP startup failed");
    }

    // Webpage HTTP routes
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

    server.enableCORS(true);
    server.on("/camera/status", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", getCameraJson());
});

server.on("/camera/update", HTTP_POST, []() {
    String state = server.arg("state");

    if (state != "CALIBRATING" &&
        state != "WITHIN_RANGE" &&
        state != "CHANGED" &&
        state != "ALERT" &&
        state != "NO_TRACKING" &&
        state != "STOPPED") {
        server.send(400, "text/plain", "Invalid camera state");
        return;
    }

    bool needsAngles =
        state == "WITHIN_RANGE" ||
        state == "CHANGED" ||
        state == "ALERT";

    float head = server.arg("head").toFloat();
    float shoulder = server.arg("shoulder").toFloat();

    if (needsAngles &&
        (!server.hasArg("head") ||
         !server.hasArg("shoulder") ||
         !isfinite(head) ||
         !isfinite(shoulder) ||
         head < 0 || head > 180 ||
         shoulder < 0 || shoulder > 180)) {
        server.send(400, "text/plain", "Invalid angles");
        return;
    }

    cameraState = state;
    cameraHead = head;
    cameraShoulder = shoulder;
    cameraSeen = state != "STOPPED";
    cameraUpdated = millis();

    server.send(200, "text/plain", "OK");
    });
    server.on("/agreement/status", HTTP_GET, []() {
        String json = "{\"state\":\"";
        json += getAgreementState();
        json += "\",\"mode\":\"";
        json += getSoundMode();
        json += "\",\"wearable_tracking\":";
        json += (tracking && sensorReady) ? "true" : "false";
        json += ",\"wearable_alert\":";
        json += (tracking && sensorReady && alertActive) ? "true" : "false";
        json += ",\"camera_tracking\":";
        json += cameraTrackingReady() ? "true" : "false";
        json += ",\"sound_alert_active\":";
        json += soundAlertActive() ? "true" : "false";
        json += ",\"camera_beep_allowed\":";
        json += cameraMayBeep() ? "true" : "false";
        json += "}";

        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", json);
    });

    server.begin();

    // Automatic calibration after startup
    if (AUTO_START && sensorReady) {
        Serial.println("Starting in 3 seconds...");
        Serial.println("Hold sensor upright.");
        delay(3000);
        calibrate();
    }

    Serial.println("=========================================");
    Serial.println("POSTURE DETECTOR READY");
    Serial.println("Wi-Fi 'Posture-ESP32' -> http://192.168.4.1");
    Serial.println("=========================================");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    // Phone Wi-Fi requests are handled continuously
    server.handleClient();

    unsigned long now = millis();

    // Allow serial commands (S/R = start/calibrate, P = pause)
    while (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'S' || cmd == 's' || cmd == 'R' || cmd == 'r') {
            calibrate();
        } else if (cmd == 'P' || cmd == 'p') {
            stopTracking();
        }
    }

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

    // When both systems track, require agreement before sounding.
    if (alertActive && wearableMayBeep() &&
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