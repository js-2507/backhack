#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ==========================================
// WIFI SETTINGS
// ==========================================
const char* AP_SSID = "Posture-ESP32";
const char* AP_PASSWORD = "Posture2026";

// ==========================================
// BLE SETTINGS (Nordic UART Service standard)
// ==========================================
#define BLE_SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHARACTERISTIC_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify & Read
#define BLE_CHARACTERISTIC_RX_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write

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

// BLE Global objects
BLEServer* pBleServer = nullptr;
BLECharacteristic* pStatusCharacteristic = nullptr;
BLECharacteristic* pCmdCharacteristic = nullptr;
bool bleClientConnected = false;
bool oldBleClientConnected = false;
unsigned long lastBleNotify = 0;

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

// Forward declarations
bool calibrate();
void stopTracking();
void notifyBleStatus();

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

void notifyBleStatus() {
    if (bleClientConnected && pStatusCharacteristic != nullptr) {
        String json = getStatusJson();
        pStatusCharacteristic->setValue(json);
        pStatusCharacteristic->notify();
    }
}

void stopTracking() {
    tracking = false;
    counting = false;
    alertActive = false;
    digitalWrite(BUZZER, LOW);
    notifyBleStatus();
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

    notifyBleStatus();
    return true;
}

// ==========================================
// BLE CALLBACKS
// ==========================================
class BleServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        bleClientConnected = true;
        Serial.println("BLE client connected!");
    }

    void onDisconnect(BLEServer* pServer) {
        bleClientConnected = false;
        Serial.println("BLE client disconnected");
    }
};

class BleCmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            rxValue.trim();
            Serial.print("BLE received command: ");
            Serial.println(rxValue);

            if (rxValue.equalsIgnoreCase("start") || 
                rxValue.equalsIgnoreCase("calibrate") || 
                rxValue.equalsIgnoreCase("/start")) {
                calibrate();
            } else if (rxValue.equalsIgnoreCase("stop") || 
                       rxValue.equalsIgnoreCase("pause") || 
                       rxValue.equalsIgnoreCase("/stop")) {
                stopTracking();
            }
        }
    }
};

void setupBle() {
    BLEDevice::init("Posture-ESP32");
    BLEDevice::setMTU(517);

    pBleServer = BLEDevice::createServer();
    pBleServer->setCallbacks(new BleServerCallbacks());

    BLEService* pService = pBleServer->createService(BLE_SERVICE_UUID);

    pStatusCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    pStatusCharacteristic->addDescriptor(new BLE2902());

    pCmdCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCmdCharacteristic->setCallbacks(new BleCmdCallbacks());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE ready: Advertising as 'Posture-ESP32'");
}

// ==========================================
// PHONE WEBPAGE (DUAL WI-FI & BLE)
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
.mode-tabs {
  display: flex;
  background: #0f172a;
  border-radius: 10px;
  padding: 4px;
  margin-bottom: 14px;
}
.mode-tab {
  flex: 1;
  padding: 10px;
  text-align: center;
  font-size: 14px;
  font-weight: 600;
  border-radius: 8px;
  cursor: pointer;
  border: none;
  background: transparent;
  color: var(--text-secondary);
  transition: all 0.2s ease;
}
.mode-tab.active {
  background: var(--card-bg);
  color: var(--accent-cyan);
  box-shadow: 0 2px 8px rgba(0,0,0,0.25);
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
.btn-ble {
  background: #2563eb;
  color: white;
  width: 100%;
  margin-top: 10px;
  padding: 12px;
  border-radius: 10px;
  border: none;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
}
.btn-ble:hover { background: #1d4ed8; }
.btn-ble.disconnect { background: #475569; }

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
    <div class="subtitle">Dual Wi-Fi & Bluetooth (BLE) Interface</div>
  </header>

  <!-- Connection Controls -->
  <div class="card">
    <div class="mode-tabs">
      <button class="mode-tab active" id="tab-wifi" onclick="selectMode('wifi')">Wi-Fi Mode</button>
      <button class="mode-tab" id="tab-ble" onclick="selectMode('ble')">Bluetooth (BLE)</button>
    </div>

    <div class="conn-status-bar">
      <span id="conn-label"><span class="dot dot-gray" id="conn-dot"></span>Connecting...</span>
      <span id="transport-badge" style="color: var(--text-secondary); font-size: 11px;">HTTP</span>
    </div>

    <div id="ble-panel" style="display: none; margin-top: 10px;">
      <button class="btn-ble" id="btn-ble-toggle" onclick="toggleBle()">Connect via Bluetooth (BLE)</button>
      <div id="ble-help" style="font-size: 11px; color: var(--text-secondary); margin-top: 8px; text-align: center;">
        Connect directly without changing your phone's Wi-Fi.
      </div>
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
      <span>20.0° (Reset: 12.0°)</span>
    </div>
  </div>

  <!-- Help & Tips -->
  <details class="help-box">
    <summary>Connection Guide & Compatibility</summary>
    <p><strong>Wi-Fi Option:</strong> Connect phone to Wi-Fi network <code>Posture-ESP32</code> (Password: <code>Posture2026</code>) and open <code>http://192.168.4.1</code>.</p>
    <p style="margin-top: 6px;"><strong>BLE Option:</strong> Select the Bluetooth tab and tap 'Connect'. You can keep normal phone internet active!</p>
    <ul style="margin-top: 6px;">
      <li><strong>Android:</strong> Works in Chrome, Edge, and Samsung Internet.</li>
      <li><strong>iPhone / iPad:</strong> Safari does not support Web Bluetooth. Please open this page in the free <em>Bluefy</em> or <em>WebBLE</em> browser app from the App Store.</li>
      <li><strong>BLE Apps:</strong> You can also connect via <em>nRF Connect</em> or <em>Serial Bluetooth Terminal</em> with Nordic UART UUIDs.</li>
    </ul>
  </details>
</div>

<script>
const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_TX_UUID      = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // Notify from ESP32
const NUS_RX_UUID      = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // Commands to ESP32

let currentMode = 'wifi';
let isBleConnected = false;
let bleDevice = null;
let bleServer = null;
let bleTxChar = null;
let bleRxChar = null;
let wifiPollTimer = null;

function getWifiBaseUrl() {
  if (window.location.protocol.startsWith('http') && window.location.hostname !== '') {
    return '';
  }
  return 'http://192.168.4.1';
}

function selectMode(mode) {
  currentMode = mode;
  document.getElementById('tab-wifi').className = 'mode-tab' + (mode === 'wifi' ? ' active' : '');
  document.getElementById('tab-ble').className = 'mode-tab' + (mode === 'ble' ? ' active' : '');
  document.getElementById('ble-panel').style.display = (mode === 'ble') ? 'block' : 'none';

  if (mode === 'wifi') {
    document.getElementById('transport-badge').innerText = 'HTTP (Wi-Fi)';
    startWifiPolling();
  } else {
    stopWifiPolling();
    document.getElementById('transport-badge').innerText = 'BLE (Nordic UART)';
    if (!isBleConnected) {
      updateConnStatus('Disconnected from BLE', 'gray');
      document.getElementById('active-link').innerText = 'Bluetooth (Disconnected)';
    } else {
      updateConnStatus('Connected to Posture-ESP32', 'blue');
      document.getElementById('active-link').innerText = 'Bluetooth (Connected)';
    }
  }
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
function startWifiPolling() {
  stopWifiPolling();
  updateWifi();
  wifiPollTimer = setInterval(updateWifi, 500);
}

function stopWifiPolling() {
  if (wifiPollTimer) {
    clearInterval(wifiPollTimer);
    wifiPollTimer = null;
  }
}

async function updateWifi() {
  if (currentMode !== 'wifi') return;
  try {
    const res = await fetch(getWifiBaseUrl() + '/status', { cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const data = await res.json();
    parseAndApplyData(data);
    updateConnStatus('Wi-Fi Connected', 'green');
    document.getElementById('active-link').innerText = 'Wi-Fi (AP 192.168.4.1)';
  } catch(e) {
    if (currentMode === 'wifi') {
      updateConnStatus('Wi-Fi Disconnected', 'red');
      document.getElementById('active-link').innerText = 'Wi-Fi (Unavailable)';
    }
  }
}

// BLE handling
async function toggleBle() {
  if (isBleConnected) {
    disconnectBle();
  } else {
    await connectBle();
  }
}

async function connectBle() {
  if (!navigator.bluetooth) {
    alert('Web Bluetooth is not supported on this browser.\n\n• Android: Use Chrome or Edge.\n• iPhone/iPad: Open this page in the Bluefy or WebBLE app.\n• Desktop: Use Chrome or Edge.\n• Or use Wi-Fi Mode at http://192.168.4.1');
    return;
  }

  const btn = document.getElementById('btn-ble-toggle');
  btn.innerText = 'Connecting...';

  try {
    bleDevice = await navigator.bluetooth.requestDevice({
      filters: [
        { name: 'Posture-ESP32' },
        { namePrefix: 'Posture' }
      ],
      optionalServices: [NUS_SERVICE_UUID]
    });

    bleDevice.addEventListener('gattserverdisconnected', onBleDisconnected);

    bleServer = await bleDevice.gatt.connect();
    const service = await bleServer.getPrimaryService(NUS_SERVICE_UUID);
    bleTxChar = await service.getCharacteristic(NUS_TX_UUID);
    bleRxChar = await service.getCharacteristic(NUS_RX_UUID);

    await bleTxChar.startNotifications();
    bleTxChar.addEventListener('characteristicvaluechanged', (ev) => {
      const text = new TextDecoder().decode(ev.target.value);
      parseAndApplyData(text);
    });

    try {
      const val = await bleTxChar.readValue();
      parseAndApplyData(new TextDecoder().decode(val));
    } catch(e) {}

    isBleConnected = true;
    btn.innerText = 'Disconnect BLE';
    btn.className = 'btn-ble disconnect';
    updateConnStatus('Connected to Posture-ESP32', 'blue');
    document.getElementById('active-link').innerText = 'Bluetooth (Posture-ESP32)';
  } catch(err) {
    console.warn('BLE Connect failed or cancelled:', err);
    btn.innerText = 'Connect via Bluetooth (BLE)';
    btn.className = 'btn-ble';
    if (!isBleConnected) {
      updateConnStatus('BLE Connection Failed', 'gray');
    }
  }
}

function disconnectBle() {
  if (bleDevice && bleDevice.gatt.connected) {
    bleDevice.gatt.disconnect();
  }
  onBleDisconnected();
}

function onBleDisconnected() {
  isBleConnected = false;
  bleServer = null;
  bleTxChar = null;
  bleRxChar = null;
  const btn = document.getElementById('btn-ble-toggle');
  if (btn) {
    btn.innerText = 'Connect via Bluetooth (BLE)';
    btn.className = 'btn-ble';
  }
  if (currentMode === 'ble') {
    updateConnStatus('BLE Disconnected', 'gray');
    document.getElementById('active-link').innerText = 'Bluetooth (Disconnected)';
  }
}

// Commands
async function handleStart() {
  await sendCommand('start');
}

async function handleStop() {
  await sendCommand('stop');
}

async function sendCommand(cmd) {
  if (currentMode === 'ble') {
    if (!bleRxChar) {
      alert('Bluetooth not connected! Please tap "Connect via Bluetooth (BLE)" first, or switch to Wi-Fi mode.');
      return;
    }
    try {
      const enc = new TextEncoder();
      await bleRxChar.writeValue(enc.encode(cmd + '\n'));
    } catch(e) {
      console.error('BLE write failed:', e);
    }
  } else {
    try {
      await fetch(getWifiBaseUrl() + '/' + cmd, { method: 'POST' });
      updateWifi();
    } catch(e) {
      console.error('Wi-Fi command failed:', e);
    }
  }
}

// Auto detect mode on page load
window.addEventListener('DOMContentLoaded', () => {
  const isHttpLocal = window.location.protocol.startsWith('http') && 
                      (window.location.hostname === '192.168.4.1' || window.location.hostname === 'localhost');
  if (isHttpLocal) {
    selectMode('wifi');
  } else if (!window.location.protocol.startsWith('http')) {
    selectMode('ble');
  } else {
    selectMode('wifi');
  }
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
    server.begin();

    // 2. Start Bluetooth Low Energy (BLE)
    setupBle();

    // Automatic calibration after startup
    if (AUTO_START && sensorReady) {
        Serial.println("Starting in 3 seconds...");
        Serial.println("Hold sensor upright.");
        delay(3000);
        calibrate();
    }

    Serial.println("=========================================");
    Serial.println("POSTURE DETECTOR READY");
    Serial.println("Option 1: Wi-Fi 'Posture-ESP32' -> http://192.168.4.1");
    Serial.println("Option 2: BLE 'Posture-ESP32' via Web Bluetooth / App");
    Serial.println("=========================================");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    // Phone Wi-Fi requests are handled continuously
    server.handleClient();

    // Handle BLE advertising restart on client disconnect
    if (!bleClientConnected && oldBleClientConnected) {
        delay(500); // give bluetooth stack a moment
        pBleServer->startAdvertising();
        Serial.println("Restarted BLE advertising");
        oldBleClientConnected = bleClientConnected;
    }
    if (bleClientConnected && !oldBleClientConnected) {
        oldBleClientConnected = bleClientConnected;
        notifyBleStatus();
    }

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
        if (bleClientConnected && (now - lastBleNotify >= 500)) {
            notifyBleStatus();
            lastBleNotify = now;
        }
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

    // Repeated short beeps
    if (alertActive &&
        (newAlert ||
         now - lastBeep >= BEEP_INTERVAL)) {
        beep();
        lastBeep = millis();
    }

    // Send BLE status update (every 250ms or immediately on new alert)
    if (bleClientConnected && (now - lastBleNotify >= 250 || newAlert)) {
        notifyBleStatus();
        lastBleNotify = now;
    }

    if (now - lastPrint >= 500) {
        Serial.printf(
            "Angle: %.1f | Alert: %s | BLE: %s\n",
            currentAngle,
            alertActive ? "ON" : "OFF",
            bleClientConnected ? "CONNECTED" : "OFF"
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

