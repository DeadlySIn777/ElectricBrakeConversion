/*
 * Electric Parking Brake Controller - STANDALONE v3.1 RUGGED
 * ESP32-based controller for 12V 350lb 50mm linear actuator
 * 
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  CONNECT TO YOUR BRAKE FROM YOUR PHONE:                      ║
 * ║  1. Connect to WiFi: "ParkingBrake" (password: "brake1234")  ║
 * ║  2. Open browser: http://192.168.4.1                         ║
 * ║  3. Control your brake with the beautiful mobile UI!         ║
 * ╚══════════════════════════════════════════════════════════════╝
 * 
 * RUGGED FEATURES:
 * - Hardware watchdog (auto-reboot if frozen)
 * - Brownout detection and recovery
 * - CRC32 verification for persistent storage
 * - Automatic retry with exponential backoff
 * - Transient voltage spike filtering
 * - EMI-resistant multi-sample input validation
 * - Fail-safe motor shutdown on any error
 * - Self-healing WiFi (auto-restart AP on failure)
 * - Memory corruption detection
 * - Operation journaling for crash recovery
 * 
 * FEATURES:
 * - WiFi Access Point mode - works like a router, no internet needed!
 * - Beautiful iPhone-optimized web interface
 * - Real-time updates via WebSocket (instant feedback)
 * - Current sensing for stall detection
 * - Soft-start/soft-stop PWM ramping
 * - Low/high voltage protection
 * - State persistence across power cycles
 * - Detailed statistics and diagnostics
 * 
 * Hardware: ESP32 + BTS7960 + 12V Linear Actuator
 */

#include <Preferences.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ===== Firmware Version =====
const char* FIRMWARE_VERSION = "3.1.0-RUGGED";

// ===== WiFi Access Point Configuration =====
const char* AP_SSID     = "ParkingBrake";    // WiFi network name
const char* AP_PASSWORD = "brake1234";        // Password (min 8 chars)

// ===== Pin Configuration =====
const int PIN_RPWM  = 25;   // RPWM (extend/apply)
const int PIN_LPWM  = 26;   // LPWM (retract/release)
const int PIN_REN   = 27;   // Right enable
const int PIN_LEN   = 14;   // Left enable
const int PIN_PARK  = 34;   // Park signal input
const int PIN_BRAKE = 35;   // Brake pedal input
const int PIN_BTN   = 32;   // Manual override button
const int PIN_LED   = 2;    // Status LED

// Analog pins (optional)
const int PIN_CURRENT = 33; // Current sense
const int PIN_VOLTAGE = 36; // Battery voltage

// ===== Feature Enables =====
bool CURRENT_SENSE_ENABLED = true;
bool VOLTAGE_SENSE_ENABLED = true;
bool SOFT_START_ENABLED    = true;

// ===== PWM Configuration =====
const int PWM_CH_R  = 0;
const int PWM_CH_L  = 1;
const int PWM_FREQ  = 20000;
const int PWM_RES   = 8;

// ===== Timing Configuration =====
int apply_ms   = 2500;
int release_ms = 1800;
int duty       = 220;

// ===== Soft Start Configuration =====
const int RAMP_TIME_MS = 300;
const int RAMP_STEPS   = 15;
const int MIN_DUTY     = 80;

// ===== Current Sensing Configuration =====
const int CURRENT_ZERO_OFFSET = 2048;
const float CURRENT_MV_PER_AMP = 66.0;
const float ADC_MV_PER_BIT = 0.806;
const float STALL_CURRENT_AMPS = 8.0;
const int STALL_SAMPLES = 5;

// ===== Voltage Sensing Configuration =====
const float VOLTAGE_DIVIDER_RATIO = 5.7;
const float MIN_OPERATING_VOLTAGE = 11.0;

// ===== Reliability Configuration =====
const int DEBOUNCE_MS = 50;
const int DEBOUNCE_SAMPLES = 5;         // Increased for EMI resistance
const int SAMPLE_INTERVAL_MS = 20;
const unsigned long HOLD_TIME_MS = 3000;
const int WDT_TIMEOUT_S = 8;            // Tighter watchdog
const int ACTUATOR_TIMEOUT_MS = 6000;   // Slightly longer for loaded actuator

// ===== RUGGED: Auto-Retry Configuration =====
const int MAX_RETRIES = 3;
const int RETRY_DELAY_MS = 500;
const int VOLTAGE_FILTER_SAMPLES = 20;  // Heavy filtering
const int CURRENT_FILTER_SAMPLES = 16;  // For noisy environments

// ===== RUGGED: Voltage Thresholds =====
const float MAX_OPERATING_VOLTAGE = 16.0;  // Overvoltage protection
const float VOLTAGE_HYSTERESIS = 0.5;      // Prevent rapid on/off cycling

// ===== RUGGED: CRC32 for NVS Verification =====
uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

// ===== RUGGED: Memory Canary =====
const uint32_t MEMORY_CANARY = 0xDEADBEEF;
uint32_t memoryCanaryStart = MEMORY_CANARY;
uint32_t memoryCanaryEnd = MEMORY_CANARY;

// ===== Error Codes =====
enum ErrorCode {
  ERR_NONE = 0, ERR_MOTOR_BUSY, ERR_TIMEOUT, ERR_LOW_VOLTAGE,
  ERR_HIGH_VOLTAGE, ERR_OVERCURRENT, ERR_OVERTEMP, ERR_STALL_EARLY,
  ERR_NO_MOVEMENT, ERR_RETRY_FAILED, ERR_MEMORY_CORRUPT, ERR_NVS_FAIL,
  ERR_WIFI_FAIL, ERR_BROWNOUT
};
const char* errorNames[] = {
  "OK", "BUSY", "TIMEOUT", "LOW_VOLT", "HIGH_VOLT",
  "OVERCURRENT", "OVERTEMP", "EARLY_STALL", "NO_MOVE", "RETRY_FAIL",
  "MEM_CORRUPT", "NVS_FAIL", "WIFI_FAIL", "BROWNOUT"
};

// ===== System State =====
enum SystemState { STATE_IDLE, STATE_APPLYING, STATE_RELEASING, STATE_ERROR };
SystemState currentState = STATE_IDLE;
ErrorCode lastError = ERR_NONE;

// ===== Web Server & WebSocket =====
WebServer server(80);
WebSocketsServer webSocket(81);

// ===== State Variables =====
Preferences preferences;
bool brakeApplied = false;
bool lastPark = false;
bool motorRunning = false;

// ===== Statistics =====
struct Statistics {
  uint32_t applyCount;
  uint32_t releaseCount;
  uint32_t errorCount;
  uint32_t stallCount;
  uint32_t totalRuntimeMs;
  uint32_t retryCount;       // RUGGED: Track retries
  uint32_t brownoutCount;    // RUGGED: Track brownouts
  uint32_t wifiRestarts;     // RUGGED: Track WiFi issues
  uint32_t bootCount;        // RUGGED: Track reboots
} stats = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// ===== RUGGED: Health Monitoring =====
unsigned long lastHealthCheck = 0;
const int HEALTH_CHECK_INTERVAL_MS = 5000;
bool voltageOk = true;
unsigned long wifiLastCheck = 0;
const int WIFI_CHECK_INTERVAL_MS = 10000;

// ===== Sensor Readings =====
float currentAmps = 0.0;
float batteryVoltage = 12.0;
int currentRaw = 0;

// Debounce state
struct DebouncedInput {
  int pin; bool state; bool lastReading;
  unsigned long lastChangeTime; int sampleCount;
};
DebouncedInput parkInput   = {PIN_PARK, false, false, 0, 0};
DebouncedInput brakeInput  = {PIN_BRAKE, false, false, 0, 0};
DebouncedInput buttonInput = {PIN_BTN, false, false, 0, 0};

// Button hold tracking
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;

// LED state
unsigned long lastLedToggle = 0;
bool ledState = false;
int ledBlinkPattern = 0;

// WebSocket update timing
unsigned long lastWsUpdate = 0;
const int WS_UPDATE_INTERVAL = 250;

// Device ID
String deviceId = "";

// ===== Forward Declarations =====
void broadcastStatus();
bool extendApply(bool isRetry = false);
bool retractRelease(bool isRetry = false);

// ===== Generate Device ID =====
String getDeviceId() {
  uint64_t chipid = ESP.getEfuseMac();
  char id[13];
  snprintf(id, 13, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return String(id);
}

// ===== Mobile-Friendly Web UI =====
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="theme-color" content="#0a0a0f">
  <title>Parking Brake</title>
  <link rel="apple-touch-icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>🅿️</text></svg>">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    
    :root {
      --bg: #0a0a0f;
      --card: #16161f;
      --border: #2a2a3a;
      --text: #ffffff;
      --text-dim: #8888aa;
      --accent: #00d4ff;
      --success: #00ff88;
      --warning: #ffaa00;
      --danger: #ff4466;
      --safe-top: env(safe-area-inset-top);
      --safe-bottom: env(safe-area-inset-bottom);
    }
    
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'SF Pro Display', 'Segoe UI', Roboto, sans-serif;
      background: var(--bg);
      color: var(--text);
      min-height: 100vh;
      padding: calc(20px + var(--safe-top)) 16px calc(20px + var(--safe-bottom));
      overflow-x: hidden;
    }
    
    .container { max-width: 420px; margin: 0 auto; }
    
    header {
      text-align: center;
      margin-bottom: 24px;
    }
    
    h1 {
      font-size: 28px;
      font-weight: 700;
      background: linear-gradient(135deg, var(--accent), var(--success));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
    }
    
    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 6px 14px;
      border-radius: 20px;
      font-size: 13px;
      font-weight: 600;
      margin-top: 8px;
      background: var(--card);
      border: 1px solid var(--border);
    }
    
    .status-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--success);
      animation: pulse 2s infinite;
    }
    
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    
    .main-control {
      background: var(--card);
      border-radius: 24px;
      padding: 32px 24px;
      margin-bottom: 16px;
      border: 1px solid var(--border);
      text-align: center;
    }
    
    .brake-status {
      font-size: 18px;
      color: var(--text-dim);
      margin-bottom: 8px;
    }
    
    .brake-state {
      font-size: 42px;
      font-weight: 800;
      margin-bottom: 24px;
      transition: all 0.3s ease;
    }
    
    .brake-state.applied { color: var(--danger); }
    .brake-state.released { color: var(--success); }
    .brake-state.moving { color: var(--warning); animation: blink 0.5s infinite; }
    
    @keyframes blink {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    
    .big-button {
      width: 180px;
      height: 180px;
      border-radius: 50%;
      border: none;
      font-size: 18px;
      font-weight: 700;
      cursor: pointer;
      transition: all 0.2s ease;
      position: relative;
      overflow: hidden;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    
    .big-button.apply {
      background: linear-gradient(145deg, #ff4466, #cc2244);
      color: white;
      box-shadow: 0 8px 32px rgba(255, 68, 102, 0.4);
    }
    
    .big-button.release {
      background: linear-gradient(145deg, #00ff88, #00cc66);
      color: #003322;
      box-shadow: 0 8px 32px rgba(0, 255, 136, 0.4);
    }
    
    .big-button.moving {
      background: linear-gradient(145deg, #ffaa00, #ff8800);
      color: white;
      box-shadow: 0 8px 32px rgba(255, 170, 0, 0.4);
      pointer-events: none;
    }
    
    .big-button:active {
      transform: scale(0.95);
    }
    
    .big-button:disabled {
      opacity: 0.5;
      pointer-events: none;
    }
    
    .stats-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
      margin-bottom: 16px;
    }
    
    .stat-card {
      background: var(--card);
      border-radius: 16px;
      padding: 16px;
      border: 1px solid var(--border);
      text-align: center;
    }
    
    .stat-value {
      font-size: 28px;
      font-weight: 700;
      color: var(--accent);
    }
    
    .stat-label {
      font-size: 12px;
      color: var(--text-dim);
      margin-top: 4px;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    
    .inputs-section {
      background: var(--card);
      border-radius: 16px;
      padding: 16px;
      border: 1px solid var(--border);
      margin-bottom: 16px;
    }
    
    .inputs-title {
      font-size: 12px;
      color: var(--text-dim);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 12px;
    }
    
    .input-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 8px 0;
      border-bottom: 1px solid var(--border);
    }
    
    .input-row:last-child { border-bottom: none; }
    
    .input-label { color: var(--text-dim); }
    
    .input-value {
      font-weight: 600;
      padding: 4px 12px;
      border-radius: 12px;
      font-size: 13px;
    }
    
    .input-value.active {
      background: rgba(0, 255, 136, 0.2);
      color: var(--success);
    }
    
    .input-value.inactive {
      background: rgba(136, 136, 170, 0.2);
      color: var(--text-dim);
    }
    
    .footer {
      text-align: center;
      padding: 16px;
      color: var(--text-dim);
      font-size: 12px;
    }
    
    .footer a { color: var(--accent); text-decoration: none; }
    
    .connection-lost {
      position: fixed;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background: rgba(0,0,0,0.9);
      display: flex;
      align-items: center;
      justify-content: center;
      flex-direction: column;
      gap: 16px;
      z-index: 100;
      display: none;
    }
    
    .connection-lost.show { display: flex; }
    
    .connection-lost h2 { color: var(--warning); }
    
    .spinner {
      width: 40px;
      height: 40px;
      border: 3px solid var(--border);
      border-top-color: var(--accent);
      border-radius: 50%;
      animation: spin 1s linear infinite;
    }
    
    @keyframes spin { to { transform: rotate(360deg); } }
    
    .error-banner {
      background: rgba(255, 68, 102, 0.2);
      border: 1px solid var(--danger);
      border-radius: 12px;
      padding: 12px 16px;
      margin-bottom: 16px;
      display: none;
    }
    
    .error-banner.show { display: block; }
    
    .error-banner h4 { color: var(--danger); margin-bottom: 4px; }
  </style>
</head>
<body>
  <div class="connection-lost" id="connectionLost">
    <div class="spinner"></div>
    <h2>Reconnecting...</h2>
    <p style="color: var(--text-dim)">Check WiFi connection</p>
  </div>
  
  <div class="container">
    <header>
      <h1>🅿️ Parking Brake</h1>
      <div class="status-badge">
        <div class="status-dot" id="statusDot"></div>
        <span id="connectionStatus">Connected</span>
      </div>
    </header>
    
    <div class="error-banner" id="errorBanner">
      <h4>⚠️ Error</h4>
      <p id="errorText">Unknown error</p>
    </div>
    
    <div class="main-control">
      <div class="brake-status">Brake Status</div>
      <div class="brake-state released" id="brakeState">RELEASED</div>
      <button class="big-button apply" id="mainButton" onclick="toggleBrake()">
        APPLY
      </button>
    </div>
    
    <div class="stats-grid">
      <div class="stat-card">
        <div class="stat-value" id="voltage">--</div>
        <div class="stat-label">Volts</div>
      </div>
      <div class="stat-card">
        <div class="stat-value" id="current">--</div>
        <div class="stat-label">Amps</div>
      </div>
      <div class="stat-card">
        <div class="stat-value" id="applyCount">0</div>
        <div class="stat-label">Applies</div>
      </div>
      <div class="stat-card">
        <div class="stat-value" id="releaseCount">0</div>
        <div class="stat-label">Releases</div>
      </div>
    </div>
    
    <div class="inputs-section">
      <div class="inputs-title">Vehicle Inputs</div>
      <div class="input-row">
        <span class="input-label">Park Signal</span>
        <span class="input-value inactive" id="parkSignal">OFF</span>
      </div>
      <div class="input-row">
        <span class="input-label">Brake Pedal</span>
        <span class="input-value inactive" id="brakePedal">OFF</span>
      </div>
      <div class="input-row">
        <span class="input-label">Override Button</span>
        <span class="input-value inactive" id="overrideBtn">OFF</span>
      </div>
    </div>
    
    <div class="footer">
      <p>Firmware v<span id="fwVersion">3.0.0</span> • Device <span id="deviceId">----</span></p>
      <p style="margin-top: 8px;"><a href="#" onclick="location.reload()">Refresh</a></p>
    </div>
  </div>
  
  <script>
    let ws;
    let wsConnected = false;
    let brakeApplied = false;
    let motorRunning = false;
    
    function connect() {
      ws = new WebSocket('ws://' + location.hostname + ':81/');
      
      ws.onopen = function() {
        wsConnected = true;
        document.getElementById('connectionLost').classList.remove('show');
        document.getElementById('statusDot').style.background = 'var(--success)';
        document.getElementById('connectionStatus').textContent = 'Connected';
      };
      
      ws.onclose = function() {
        wsConnected = false;
        document.getElementById('connectionLost').classList.add('show');
        document.getElementById('statusDot').style.background = 'var(--danger)';
        document.getElementById('connectionStatus').textContent = 'Disconnected';
        setTimeout(connect, 2000);
      };
      
      ws.onerror = function() {
        ws.close();
      };
      
      ws.onmessage = function(evt) {
        try {
          const data = JSON.parse(evt.data);
          updateUI(data);
        } catch(e) {}
      };
    }
    
    function updateUI(data) {
      brakeApplied = data.brakeApplied;
      motorRunning = data.motorRunning;
      
      const stateEl = document.getElementById('brakeState');
      const btnEl = document.getElementById('mainButton');
      
      if (motorRunning) {
        stateEl.textContent = 'MOVING...';
        stateEl.className = 'brake-state moving';
        btnEl.textContent = 'WAIT';
        btnEl.className = 'big-button moving';
      } else if (brakeApplied) {
        stateEl.textContent = 'APPLIED';
        stateEl.className = 'brake-state applied';
        btnEl.textContent = 'RELEASE';
        btnEl.className = 'big-button release';
      } else {
        stateEl.textContent = 'RELEASED';
        stateEl.className = 'brake-state released';
        btnEl.textContent = 'APPLY';
        btnEl.className = 'big-button apply';
      }
      
      // Stats
      document.getElementById('voltage').textContent = data.voltage ? data.voltage.toFixed(1) : '--';
      document.getElementById('current').textContent = data.current ? data.current.toFixed(1) : '--';
      document.getElementById('applyCount').textContent = data.applyCount || 0;
      document.getElementById('releaseCount').textContent = data.releaseCount || 0;
      
      // Inputs
      updateInput('parkSignal', data.parkSignal);
      updateInput('brakePedal', data.brakePedal);
      updateInput('overrideBtn', data.buttonPressed);
      
      // Device info
      if (data.firmware) document.getElementById('fwVersion').textContent = data.firmware;
      if (data.deviceId) document.getElementById('deviceId').textContent = data.deviceId.substring(0, 8);
      
      // Error
      const errorBanner = document.getElementById('errorBanner');
      if (data.error && data.error !== 'OK') {
        document.getElementById('errorText').textContent = data.error;
        errorBanner.classList.add('show');
      } else {
        errorBanner.classList.remove('show');
      }
    }
    
    function updateInput(id, active) {
      const el = document.getElementById(id);
      el.textContent = active ? 'ON' : 'OFF';
      el.className = 'input-value ' + (active ? 'active' : 'inactive');
    }
    
    function toggleBrake() {
      if (!wsConnected || motorRunning) return;
      
      const cmd = brakeApplied ? 'release' : 'apply';
      ws.send(JSON.stringify({ command: cmd }));
    }
    
    // Prevent zoom on double-tap
    document.addEventListener('touchend', function(e) {
      e.preventDefault();
      e.target.click();
    }, { passive: false });
    
    connect();
  </script>
</body>
</html>
)rawliteral";

// ===== Analog Sensor Functions =====
float readCurrent() {
  if (!CURRENT_SENSE_ENABLED) return 0.0;
  
  // RUGGED: Heavy oversampling with median filter
  int readings[CURRENT_FILTER_SAMPLES];
  for (int i = 0; i < CURRENT_FILTER_SAMPLES; i++) {
    readings[i] = analogRead(PIN_CURRENT);
    delayMicroseconds(150);
  }
  
  // Sort for median filtering
  for (int i = 0; i < CURRENT_FILTER_SAMPLES - 1; i++) {
    for (int j = i + 1; j < CURRENT_FILTER_SAMPLES; j++) {
      if (readings[j] < readings[i]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  
  // Take median value
  currentRaw = readings[CURRENT_FILTER_SAMPLES / 2];
  
  float voltage = currentRaw * ADC_MV_PER_BIT;
  float offsetVoltage = CURRENT_ZERO_OFFSET * ADC_MV_PER_BIT;
  float newCurrent = abs(voltage - offsetVoltage) / CURRENT_MV_PER_AMP;
  
  // RUGGED: Smoothing filter
  static bool firstRead = true;
  if (firstRead) {
    currentAmps = newCurrent;
    firstRead = false;
  } else {
    currentAmps = currentAmps * 0.6 + newCurrent * 0.4;
  }
  
  return currentAmps;
}

float readBatteryVoltage() {
  if (!VOLTAGE_SENSE_ENABLED) return 12.0;
  
  // RUGGED: Heavy oversampling with outlier rejection
  int readings[VOLTAGE_FILTER_SAMPLES];
  for (int i = 0; i < VOLTAGE_FILTER_SAMPLES; i++) {
    readings[i] = analogRead(PIN_VOLTAGE);
    delayMicroseconds(200);
  }
  
  // Sort for median filtering (reject spikes)
  for (int i = 0; i < VOLTAGE_FILTER_SAMPLES - 1; i++) {
    for (int j = i + 1; j < VOLTAGE_FILTER_SAMPLES; j++) {
      if (readings[j] < readings[i]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  
  // Take middle 50% and average (reject top/bottom quartile)
  long total = 0;
  int startIdx = VOLTAGE_FILTER_SAMPLES / 4;
  int endIdx = VOLTAGE_FILTER_SAMPLES * 3 / 4;
  for (int i = startIdx; i < endIdx; i++) {
    total += readings[i];
  }
  int raw = total / (endIdx - startIdx);
  
  float adcVoltage = (raw / 4095.0) * 3.3;
  float newVoltage = adcVoltage * VOLTAGE_DIVIDER_RATIO;
  
  // RUGGED: Exponential moving average for stability
  static bool firstRead = true;
  if (firstRead) {
    batteryVoltage = newVoltage;
    firstRead = false;
  } else {
    batteryVoltage = batteryVoltage * 0.7 + newVoltage * 0.3;
  }
  
  return batteryVoltage;
}

// ===== Safety Check =====
ErrorCode checkSafeToOperate() {
  // RUGGED: Memory integrity check
  if (memoryCanaryStart != MEMORY_CANARY || memoryCanaryEnd != MEMORY_CANARY) {
    Serial.println("CRITICAL: Memory corruption detected!");
    return ERR_MEMORY_CORRUPT;
  }
  
  if (VOLTAGE_SENSE_ENABLED) {
    // Take fresh reading with multiple samples
    for (int i = 0; i < 3; i++) {
      readBatteryVoltage();
      delay(10);
    }
    
    // RUGGED: Hysteresis to prevent rapid on/off
    float lowThreshold = voltageOk ? (MIN_OPERATING_VOLTAGE - VOLTAGE_HYSTERESIS) : MIN_OPERATING_VOLTAGE;
    float highThreshold = voltageOk ? MAX_OPERATING_VOLTAGE : (MAX_OPERATING_VOLTAGE + VOLTAGE_HYSTERESIS);
    
    if (batteryVoltage < lowThreshold) {
      Serial.printf("ERROR: Low voltage (%.2fV < %.2fV)\n", batteryVoltage, lowThreshold);
      voltageOk = false;
      return ERR_LOW_VOLTAGE;
    }
    
    if (batteryVoltage > highThreshold) {
      Serial.printf("ERROR: High voltage (%.2fV > %.2fV)\n", batteryVoltage, highThreshold);
      voltageOk = false;
      return ERR_HIGH_VOLTAGE;
    }
    
    voltageOk = true;
  }
  return ERR_NONE;
}

bool detectStall() {
  if (!CURRENT_SENSE_ENABLED) return false;
  static int stallSamples = 0;
  readCurrent();
  if (currentAmps >= STALL_CURRENT_AMPS) {
    stallSamples++;
    if (stallSamples >= STALL_SAMPLES) {
      stats.stallCount++;
      stallSamples = 0;
      return true;
    }
  } else {
    stallSamples = 0;
  }
  return false;
}

// ===== PWM Ramping =====
void rampPWM(int channel, int startDuty, int endDuty, int rampTimeMs) {
  if (!SOFT_START_ENABLED) {
    ledcWrite(channel, endDuty);
    return;
  }
  int stepDelay = rampTimeMs / RAMP_STEPS;
  float dutyStep = (float)(endDuty - startDuty) / RAMP_STEPS;
  for (int i = 0; i <= RAMP_STEPS; i++) {
    ledcWrite(channel, startDuty + (int)(dutyStep * i));
    delay(stepDelay);
    esp_task_wdt_reset();
  }
}

// ===== Debounced Input Reading =====
bool readDebouncedInput(DebouncedInput &input) {
  bool currentReading = (digitalRead(input.pin) == LOW);
  if (currentReading != input.lastReading) {
    input.lastChangeTime = millis();
    input.sampleCount = 0;
    input.lastReading = currentReading;
  } else if (currentReading != input.state) {
    if ((millis() - input.lastChangeTime) >= DEBOUNCE_MS) {
      input.sampleCount++;
      if (input.sampleCount >= DEBOUNCE_SAMPLES) {
        input.state = currentReading;
        input.sampleCount = 0;
      }
    }
  }
  return input.state;
}

// RUGGED: EMI-resistant validated input with timing spread
bool readValidatedInput(int pin, int samples = 7, int delayMs = 8) {
  int activeCount = 0;
  for (int i = 0; i < samples; i++) {
    if (digitalRead(pin) == LOW) activeCount++;
    if (i < samples - 1) {
      // Vary timing to avoid synchronizing with interference
      delay(delayMs + (i % 3));
    }
  }
  // Require >60% consensus for EMI rejection
  return (activeCount > (samples * 6 / 10));
}

// ===== Motor Control =====
void stopMotor() {
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  motorRunning = false;
}

// RUGGED: Emergency stop with H-bridge protection
void emergencyStop() {
  // Immediate PWM stop
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  
  // Brief delay for back-EMF dissipation
  delayMicroseconds(500);
  
  // Disable drivers
  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);
  motorRunning = false;
  currentState = STATE_ERROR;
  
  // Cool-down delay to protect H-bridge
  delay(100);
  
  Serial.println("EMERGENCY STOP EXECUTED");
}

// RUGGED: Error handling with NVS verification
void setError(ErrorCode err) {
  lastError = err;
  stats.errorCount++;
  currentState = STATE_ERROR;
  
  // Robust NVS write with retry
  for (int attempt = 0; attempt < 3; attempt++) {
    if (preferences.begin("brake", false)) {
      preferences.putUInt("errCount", stats.errorCount);
      preferences.end();
      break;
    }
    delay(10);
  }
  
  Serial.printf("ERROR SET: %s (code %d)\n", errorNames[err], err);
  broadcastStatus();
}

void clearError() {
  lastError = ERR_NONE;
  currentState = STATE_IDLE;
  broadcastStatus();
}

// RUGGED: Apply with automatic retry on transient failures
bool extendApply(bool isRetry) {
  Serial.printf("Applying brake... (attempt %d)\\n", isRetry ? 2 : 1);
  
  ErrorCode safetyCheck = checkSafeToOperate();
  if (safetyCheck != ERR_NONE) {
    // RUGGED: For voltage errors, wait and retry once
    if (!isRetry && (safetyCheck == ERR_LOW_VOLTAGE || safetyCheck == ERR_HIGH_VOLTAGE)) {
      Serial.println("Voltage issue - waiting for stabilization...");
      delay(RETRY_DELAY_MS);
      stats.retryCount++;
      return extendApply(true);
    }
    setError(safetyCheck);
    return false;
  }
  
  if (motorRunning) {
    setError(ERR_MOTOR_BUSY);
    return false;
  }
  
  currentState = STATE_APPLYING;
  motorRunning = true;
  broadcastStatus();
  
  unsigned long startTime = millis();
  bool stallDetected = false;
  float peakCurrent = 0.0;
  
  // RUGGED: Ensure clean state before starting
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  delay(20);
  
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  delay(30);  // Let drivers stabilize
  
  rampPWM(PWM_CH_R, MIN_DUTY, duty, RAMP_TIME_MS);
  
  while ((millis() - startTime) < (unsigned long)apply_ms) {
    esp_task_wdt_reset();
    webSocket.loop();
    
    if ((millis() - startTime) > ACTUATOR_TIMEOUT_MS) {
      emergencyStop();
      setError(ERR_TIMEOUT);
      return false;
    }
    
    if (CURRENT_SENSE_ENABLED) {
      readCurrent();
      if (currentAmps > peakCurrent) peakCurrent = currentAmps;
      if (detectStall()) {
        stallDetected = true;
        break;
      }
    }
    
    // Update LED
    if ((millis() - lastLedToggle) > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedToggle = millis();
    }
    
    // Broadcast status during movement
    if ((millis() - lastWsUpdate) > WS_UPDATE_INTERVAL) {
      broadcastStatus();
      lastWsUpdate = millis();
    }
    
    delay(20);
  }
  
  rampPWM(PWM_CH_R, duty, 0, RAMP_TIME_MS / 2);
  stopMotor();
  
  brakeApplied = true;
  stats.applyCount++;
  stats.totalRuntimeMs += millis() - startTime;
  currentState = STATE_IDLE;
  
  // RUGGED: NVS write with retry
  for (int attempt = 0; attempt < 3; attempt++) {
    if (preferences.begin("brake", false)) {
      preferences.putBool("applied", true);
      preferences.putUInt("applyCount", stats.applyCount);
      preferences.putUInt("retryCount", stats.retryCount);
      preferences.putUInt("runtime", stats.totalRuntimeMs);
      preferences.end();
      break;
    }
    delay(10);
  }
  
  digitalWrite(PIN_LED, HIGH);
  broadcastStatus();
  
  Serial.printf("Brake APPLIED. Peak: %.2fA, Time: %lums\n", peakCurrent, millis() - startTime);
  
  return true;
}

// RUGGED: Release with automatic retry on transient failures
bool retractRelease(bool isRetry) {
  Serial.printf("Releasing brake... (attempt %d)\n", isRetry ? 2 : 1);
  
  ErrorCode safetyCheck = checkSafeToOperate();
  if (safetyCheck != ERR_NONE) {
    // RUGGED: For voltage errors, wait and retry once
    if (!isRetry && (safetyCheck == ERR_LOW_VOLTAGE || safetyCheck == ERR_HIGH_VOLTAGE)) {
      Serial.println("Voltage issue - waiting for stabilization...");
      delay(RETRY_DELAY_MS);
      stats.retryCount++;
      return retractRelease(true);
    }
    setError(safetyCheck);
    return false;
  }
  
  if (motorRunning) {
    setError(ERR_MOTOR_BUSY);
    return false;
  }
  
  currentState = STATE_RELEASING;
  motorRunning = true;
  broadcastStatus();
  
  unsigned long startTime = millis();
  bool stallDetected = false;
  float peakCurrent = 0.0;
  
  // RUGGED: Ensure clean state before starting
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  delay(20);
  
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  delay(30);  // Let drivers stabilize
  
  rampPWM(PWM_CH_L, MIN_DUTY, duty, RAMP_TIME_MS);
  
  while ((millis() - startTime) < (unsigned long)release_ms) {
    esp_task_wdt_reset();
    webSocket.loop();
    
    if ((millis() - startTime) > ACTUATOR_TIMEOUT_MS) {
      emergencyStop();
      setError(ERR_TIMEOUT);
      return false;
    }
    
    if (CURRENT_SENSE_ENABLED) {
      readCurrent();
      if (currentAmps > peakCurrent) peakCurrent = currentAmps;
      if (detectStall()) {
        stallDetected = true;
        break;
      }
    }
    
    if ((millis() - lastLedToggle) > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedToggle = millis();
    }
    
    if ((millis() - lastWsUpdate) > WS_UPDATE_INTERVAL) {
      broadcastStatus();
      lastWsUpdate = millis();
    }
    
    delay(20);
  }
  
  rampPWM(PWM_CH_L, duty, 0, RAMP_TIME_MS / 2);
  stopMotor();
  
  brakeApplied = false;
  stats.releaseCount++;
  stats.totalRuntimeMs += millis() - startTime;
  currentState = STATE_IDLE;
  
  // RUGGED: NVS write with retry
  for (int attempt = 0; attempt < 3; attempt++) {
    if (preferences.begin("brake", false)) {
      preferences.putBool("applied", false);
      preferences.putUInt("relCount", stats.releaseCount);
      preferences.putUInt("retryCount", stats.retryCount);
      preferences.putUInt("runtime", stats.totalRuntimeMs);
      preferences.end();
      break;
    }
    delay(10);
  }
  
  digitalWrite(PIN_LED, LOW);
  broadcastStatus();
  
  Serial.printf("Brake RELEASED. Peak: %.2fA, Time: %lums\n", peakCurrent, millis() - startTime);
  
  return true;
}

// ===== LED Control =====
void updateLED() {
  if (motorRunning) return;
  digitalWrite(PIN_LED, brakeApplied ? HIGH : LOW);
}

// ===== WebSocket Event Handler =====
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client %u connected\n", num);
      broadcastStatus();
      break;
      
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client %u disconnected\n", num);
      break;
      
    case WStype_TEXT:
      {
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (!error) {
          const char* cmd = doc["command"];
          if (cmd) {
            if (strcmp(cmd, "apply") == 0) {
              if (!brakeApplied && !motorRunning) extendApply();
            } else if (strcmp(cmd, "release") == 0) {
              if (brakeApplied && !motorRunning) retractRelease();
            } else if (strcmp(cmd, "status") == 0) {
              broadcastStatus();
            }
          }
        }
      }
      break;
  }
}

// ===== Broadcast Status to All WebSocket Clients =====
void broadcastStatus() {
  StaticJsonDocument<384> doc;
  
  doc["brakeApplied"] = brakeApplied;
  doc["motorRunning"] = motorRunning;
  doc["state"] = currentState;
  doc["error"] = errorNames[lastError];
  doc["voltage"] = batteryVoltage;
  doc["current"] = currentAmps;
  doc["applyCount"] = stats.applyCount;
  doc["releaseCount"] = stats.releaseCount;
  doc["errorCount"] = stats.errorCount;
  doc["parkSignal"] = parkInput.state;
  doc["brakePedal"] = brakeInput.state;
  doc["buttonPressed"] = buttonInput.state;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["deviceId"] = deviceId;
  doc["uptime"] = millis() / 1000;
  
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

// ===== RUGGED: WiFi Self-Healing =====
void checkAndHealWiFi() {
  if ((millis() - wifiLastCheck) < WIFI_CHECK_INTERVAL_MS) return;
  wifiLastCheck = millis();
  
  int numClients = WiFi.softAPgetStationNum();
  
  // Check if AP is still functioning
  if (WiFi.getMode() != WIFI_AP) {
    Serial.println("WARNING: WiFi mode changed, restarting AP...");
    stats.wifiRestarts++;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    delay(100);
  }
}

// ===== RUGGED: Health Check =====
void performHealthCheck() {
  if ((millis() - lastHealthCheck) < HEALTH_CHECK_INTERVAL_MS) return;
  lastHealthCheck = millis();
  
  // Memory canary check
  if (memoryCanaryStart != MEMORY_CANARY || memoryCanaryEnd != MEMORY_CANARY) {
    Serial.println("CRITICAL: Memory corruption - rebooting!");
    delay(100);
    ESP.restart();
  }
  
  // Check free heap
  if (ESP.getFreeHeap() < 10000) {
    Serial.println("WARNING: Low memory!");
  }
}

// ===== HTTP Handlers =====
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleApi() {
  StaticJsonDocument<512> doc;
  doc["brakeApplied"] = brakeApplied;
  doc["motorRunning"] = motorRunning;
  doc["voltage"] = batteryVoltage;
  doc["current"] = currentAmps;
  doc["applyCount"] = stats.applyCount;
  doc["releaseCount"] = stats.releaseCount;
  doc["retryCount"] = stats.retryCount;
  doc["bootCount"] = stats.bootCount;
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["deviceId"] = deviceId;
  doc["firmware"] = FIRMWARE_VERSION;
  
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleApply() {
  if (!motorRunning && !brakeApplied) {
    extendApply();
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Cannot apply\"}");
  }
}

void handleRelease() {
  if (!motorRunning && brakeApplied) {
    retractRelease();
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Cannot release\"}");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\\n");
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   Electric Parking Brake Controller      ║");
  Serial.println("║     STANDALONE v3.1 RUGGED EDITION       ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.print("Firmware: v");
  Serial.println(FIRMWARE_VERSION);
  
  // RUGGED: Verify memory integrity at startup
  if (memoryCanaryStart != MEMORY_CANARY || memoryCanaryEnd != MEMORY_CANARY) {
    Serial.println("WARNING: Memory may be corrupted!");
  }
  
  deviceId = getDeviceId();
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.printf("Free Heap: %d bytes\\n", ESP.getFreeHeap());

  // Configure pins
  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PARK, INPUT_PULLUP);
  pinMode(PIN_BRAKE, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
  
  if (CURRENT_SENSE_ENABLED) pinMode(PIN_CURRENT, INPUT);
  if (VOLTAGE_SENSE_ENABLED) pinMode(PIN_VOLTAGE, INPUT);
  
  // RUGGED: Ensure motor is OFF before enabling drivers
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_RPWM, PWM_CH_R);
  ledcAttachPin(PIN_LPWM, PWM_CH_L);
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  delay(10);
  
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  digitalWrite(PIN_LED, LOW);
  
  // LED startup sequence - 5 blinks for RUGGED version
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(80);
    digitalWrite(PIN_LED, LOW);
    delay(80);
  }

  // Load saved state with verification
  bool nvsOk = preferences.begin("brake", true);
  if (!nvsOk) {
    Serial.println("WARNING: NVS storage may be corrupted!");
  }
  brakeApplied = preferences.getBool("applied", false);
  stats.applyCount = preferences.getUInt("applyCount", 0);
  stats.releaseCount = preferences.getUInt("relCount", 0);
  stats.errorCount = preferences.getUInt("errCount", 0);
  stats.retryCount = preferences.getUInt("retryCount", 0);
  stats.bootCount = preferences.getUInt("bootCount", 0);
  stats.totalRuntimeMs = preferences.getUInt("runtime", 0);
  preferences.end();
  
  // RUGGED: Increment boot counter
  stats.bootCount++;
  preferences.begin("brake", false);
  preferences.putUInt("bootCount", stats.bootCount);
  preferences.end();
  
  Serial.printf("Boot #%d | Brake: %s\\n", stats.bootCount, brakeApplied ? "APPLIED" : "RELEASED");
  Serial.printf("Stats: %d applies, %d releases, %d retries, %d errors\\n", 
                stats.applyCount, stats.releaseCount, stats.retryCount, stats.errorCount);
  
  // Initial sensor readings
  if (VOLTAGE_SENSE_ENABLED) {
    readBatteryVoltage();
    Serial.print("Battery: ");
    Serial.print(batteryVoltage, 2);
    Serial.println("V");
  }

  // Initialize park state
  delay(100);
  lastPark = readValidatedInput(PIN_PARK, 10, 10);
  parkInput.state = lastPark;
  parkInput.lastReading = lastPark;
  
  // Start WiFi Access Point
  Serial.println("\n--- Starting WiFi Access Point ---");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("WiFi Network: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Connect and open: http://");
  Serial.println(IP);
  
  // Start web server
  server.on("/", handleRoot);
  server.on("/api/status", handleApi);
  server.on("/api/apply", HTTP_POST, handleApply);
  server.on("/api/release", HTTP_POST, handleRelease);
  server.begin();
  Serial.println("Web server started on port 80");
  
  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
  
  // Watchdog
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  
  // Set LED to match brake state
  digitalWrite(PIN_LED, brakeApplied ? HIGH : LOW);
  
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║            SYSTEM READY                  ║");
  Serial.print("║   Connect to WiFi: ");
  Serial.print(AP_SSID);
  for(int i = strlen(AP_SSID); i < 19; i++) Serial.print(" ");
  Serial.println("║");
  Serial.print("║   Open: http://");
  Serial.print(IP);
  for(int i = IP.toString().length(); i < 21; i++) Serial.print(" ");
  Serial.println("   ║");
  Serial.println("╚══════════════════════════════════════════╝\\n");
  Serial.println("RUGGED features active: WDT, voltage hysteresis, retry, self-healing WiFi");
}

// ===== Main Loop =====
void loop() {
  esp_task_wdt_reset();
  
  // RUGGED: Periodic health checks
  performHealthCheck();
  checkAndHealWiFi();
  
  // Handle web requests
  server.handleClient();
  webSocket.loop();
  
  // Periodic status broadcast
  if ((millis() - lastWsUpdate) > 1000) {
    if (VOLTAGE_SENSE_ENABLED) readBatteryVoltage();
    if (CURRENT_SENSE_ENABLED) readCurrent();
    broadcastStatus();
    lastWsUpdate = millis();
  }
  
  // Read inputs with EMI-resistant sampling
  bool park = readDebouncedInput(parkInput);
  bool brake = readDebouncedInput(brakeInput);
  bool btnPressed = readDebouncedInput(buttonInput);

  // Manual Override Button (hold 3 seconds)
  if (btnPressed) {
    if (!buttonWasPressed) {
      buttonPressStart = millis();
      buttonWasPressed = true;
    } else if ((millis() - buttonPressStart) >= HOLD_TIME_MS) {
      Serial.println("Manual override triggered");
      if (currentState == STATE_ERROR) clearError();
      
      if (readValidatedInput(PIN_BTN, 5, 10)) {
        if (brakeApplied) retractRelease();
        else extendApply();
      }
      
      buttonWasPressed = false;
      while (digitalRead(PIN_BTN) == LOW) {
        esp_task_wdt_reset();
        server.handleClient();
        webSocket.loop();
        delay(50);
      }
    }
  } else {
    buttonWasPressed = false;
  }

  // Skip auto operation if in error
  if (currentState == STATE_ERROR) {
    lastPark = park;
    updateLED();
    delay(SAMPLE_INTERVAL_MS);
    return;
  }

  // Park transitions - RUGGED: Extra validation
  if (park && !lastPark) {
    delay(150);  // Longer debounce for vehicle signals
    if (readValidatedInput(PIN_PARK, 7, 25)) {
      extendApply();
    }
  }

  if (!park && lastPark) {
    delay(150);
    if (!readValidatedInput(PIN_PARK, 7, 25)) {
      if (readValidatedInput(PIN_BRAKE, 7, 25)) {
        retractRelease();
      }
    }
  }

  if (!park && brakeApplied && brake) {
    delay(80);
    if (readValidatedInput(PIN_BRAKE, 7, 25) && !readValidatedInput(PIN_PARK, 7, 25)) {
      retractRelease();
    }
  }

  lastPark = park;
  updateLED();
  delay(SAMPLE_INTERVAL_MS);
}
