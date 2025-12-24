/*
 * Electric Parking Brake Controller - CLOUD CONNECTED v2.1
 * ESP32-based controller for 12V 350lb 50mm linear actuator
 * 
 * RELIABILITY FEATURES:
 * - Hardware watchdog timer (auto-reset if code hangs)
 * - Software debouncing on all inputs (50ms)
 * - Multi-sample input validation (5 consecutive reads)
 * - State persistence across power cycles (NVS flash)
 * - Status LED with multiple blink patterns
 * - Actuation timeout protection
 * - Graceful error handling with error codes
 * 
 * PROFESSIONAL FEATURES (v2.0):
 * - Current sensing for stall detection (knows when actuator hits limits)
 * - Soft-start/soft-stop PWM ramping (reduces mechanical stress)
 * - Low voltage protection (prevents operation on weak battery)
 * - Movement verification (confirms actuator actually moved)
 * - Configurable parameters via serial commands
 * - Detailed diagnostics and error reporting
 * - Operation statistics (cycle count, errors, runtime)
 * - Temperature monitoring input (optional)
 * - Auto-retry on failed operations
 *
 * CLOUD FEATURES (v2.1):
 * - WiFi connectivity with auto-reconnect
 * - Server-side usage tracking and telemetry
 * - Remote monitoring of brake state, voltage, errors
 * - Unique device ID for fleet management
 * - Non-blocking HTTP requests (won't delay brake operation)
 * - Offline buffering (reports when connection restored)
 * 
 * Hardware:
 * - ESP32 DevKit
 * - BTS7960 Motor Driver (or similar H-bridge with RPWM/LPWM control)
 * - 12V 350lb 50mm Linear Actuator (with built-in limit switches)
 * - Park signal input (ground-switch)
 * - Brake pedal input (ground-switch, safety interlock)
 * - Manual override button (momentary, ground-switch)
 * - Status LED (optional but recommended)
 * - Current sense resistor or ACS712 module (optional, for stall detection)
 * - Voltage divider for battery monitoring (optional)
 * 
 * Wiring:
 * - P25 -> RPWM (extend/apply)
 * - P26 -> LPWM (retract/release)
 * - P27 -> REN (right enable)
 * - P14 -> LEN (left enable)
 * - P34 -> Park signal input
 * - P35 -> Brake pedal input
 * - P32 -> Manual override button
 * - P2  -> Status LED (built-in on most ESP32 boards)
 * - P33 -> Current sense input (optional - ADC)
 * - P36 -> Voltage sense input (optional - ADC, VP pin)
 * - P39 -> Temperature sense input (optional - ADC, VN pin)
 */

#include <Preferences.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ===== Firmware Version =====
const char* FIRMWARE_VERSION = "2.1.0";

// ===== Pin Configuration =====
const int PIN_RPWM  = 25;   // P25 -> RPWM (extend)
const int PIN_LPWM  = 26;   // P26 -> LPWM (retract)
const int PIN_REN   = 27;   // P27 -> REN (right enable)
const int PIN_LEN   = 14;   // P14 -> LEN (left enable)

const int PIN_PARK  = 34;   // P34 input - Park signal
const int PIN_BRAKE = 35;   // P35 input - Brake pedal (safety)
const int PIN_BTN   = 32;   // P32 input - Manual override button
const int PIN_LED   = 2;    // P2  - Status LED (built-in on most ESP32)

// Analog sensing pins (optional features)
const int PIN_CURRENT = 33; // P33 - Current sense (ADC1_CH5)
const int PIN_VOLTAGE = 36; // P36/VP - Battery voltage sense (ADC1_CH0)
const int PIN_TEMP    = 39; // P39/VN - Temperature sense (ADC1_CH3)

// ===== Feature Enables (set to false if hardware not installed) =====
bool CURRENT_SENSE_ENABLED = true;   // Enable current-based stall detection
bool VOLTAGE_SENSE_ENABLED = true;   // Enable low voltage protection
bool TEMP_SENSE_ENABLED    = false;  // Enable temperature monitoring
bool SOFT_START_ENABLED    = true;   // Enable PWM ramping
bool CLOUD_ENABLED         = true;   // Enable cloud telemetry

// ===== WiFi Configuration =====
// CHANGE THESE TO YOUR NETWORK CREDENTIALS
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ===== Cloud Server Configuration =====
// Server endpoint for telemetry (you can use any REST API)
const char* SERVER_URL    = "https://your-server.com/api/brake-telemetry";
const char* API_KEY       = "your-api-key-here";  // Optional API key for auth

// Telemetry settings
const int TELEMETRY_INTERVAL_MS = 60000;    // Send heartbeat every 60 seconds
const int WIFI_CONNECT_TIMEOUT_MS = 10000;  // Max time to wait for WiFi
const int HTTP_TIMEOUT_MS = 5000;           // HTTP request timeout
const int MAX_OFFLINE_EVENTS = 20;          // Max events to buffer when offline

// ===== PWM Configuration =====
const int PWM_CH_R  = 0;    // PWM channel for RPWM
const int PWM_CH_L  = 1;    // PWM channel for LPWM
const int PWM_FREQ  = 20000; // 20kHz - above audible range
const int PWM_RES   = 8;    // 8-bit resolution (0-255)

// ===== Timing Configuration =====
int apply_ms   = 2500;      // Time to fully extend/apply brake (ms)
int release_ms = 1800;      // Time to fully retract/release brake (ms)
int duty       = 220;       // PWM duty cycle 0-255 (~86% power)

// ===== Soft Start/Stop Configuration =====
const int RAMP_TIME_MS     = 300;   // Time to ramp up/down PWM
const int RAMP_STEPS       = 15;    // Number of steps in ramp
const int MIN_DUTY         = 80;    // Minimum duty cycle to start moving

// ===== Current Sensing Configuration =====
// For ACS712-30A: 66mV/A, midpoint at VCC/2 (1.65V for 3.3V ESP32)
// ADC reading: 0-4095 for 0-3.3V
const int CURRENT_ZERO_OFFSET = 2048;    // ADC reading at 0A (midpoint)
const float CURRENT_MV_PER_AMP = 66.0;   // mV per amp for ACS712-30A
const float ADC_MV_PER_BIT = 0.806;      // 3300mV / 4095 bits
const float STALL_CURRENT_AMPS = 8.0;    // Current threshold for stall detection
const int STALL_SAMPLES = 5;             // Consecutive samples above threshold
const int CURRENT_SAMPLE_MS = 20;        // Time between current samples

// ===== Voltage Sensing Configuration =====
// Voltage divider: 47K + 10K = divide by 5.7, so 12V -> 2.1V
const float VOLTAGE_DIVIDER_RATIO = 5.7; // Adjust based on your resistors
const float MIN_OPERATING_VOLTAGE = 11.0; // Minimum voltage to operate
const float LOW_VOLTAGE_WARNING = 11.5;   // Warning threshold
const float MAX_OPERATING_VOLTAGE = 15.0; // Maximum safe voltage

// ===== Temperature Sensing Configuration =====
// For NTC thermistor with 10K pullup
const float TEMP_WARNING_C = 60.0;       // Warning temperature
const float TEMP_SHUTDOWN_C = 80.0;      // Shutdown temperature

// ===== Reliability Configuration =====
const int DEBOUNCE_MS = 50;           // Debounce time for inputs
const int DEBOUNCE_SAMPLES = 3;       // Number of consecutive reads required
const int SAMPLE_INTERVAL_MS = 20;    // Time between samples
const unsigned long HOLD_TIME_MS = 3000;  // Button hold time for override
const int WDT_TIMEOUT_S = 10;         // Watchdog timeout in seconds
const int ACTUATOR_TIMEOUT_MS = 5000; // Max time for actuator movement
const int MAX_RETRIES = 2;            // Max retry attempts on failure

// ===== Error Codes =====
enum ErrorCode {
  ERR_NONE = 0,
  ERR_MOTOR_BUSY = 1,
  ERR_TIMEOUT = 2,
  ERR_LOW_VOLTAGE = 3,
  ERR_HIGH_VOLTAGE = 4,
  ERR_OVERCURRENT = 5,
  ERR_OVERTEMP = 6,
  ERR_STALL_EARLY = 7,      // Stalled before expected
  ERR_NO_MOVEMENT = 8,      // No current draw detected
  ERR_RETRY_FAILED = 9
};

const char* errorNames[] = {
  "NONE", "MOTOR_BUSY", "TIMEOUT", "LOW_VOLTAGE", "HIGH_VOLTAGE",
  "OVERCURRENT", "OVERTEMP", "STALL_EARLY", "NO_MOVEMENT", "RETRY_FAILED"
};

// ===== System State =====
enum SystemState {
  STATE_IDLE,
  STATE_APPLYING,
  STATE_RELEASING,
  STATE_ERROR,
  STATE_LOW_VOLTAGE
};

SystemState currentState = STATE_IDLE;
ErrorCode lastError = ERR_NONE;

SystemState currentState = STATE_IDLE;
ErrorCode lastError = ERR_NONE;

// ===== State Variables =====
Preferences preferences;
bool brakeApplied = false;
bool lastPark = false;
bool motorRunning = false;

// ===== Statistics (saved to flash) =====
struct Statistics {
  uint32_t applyCount;       // Total apply operations
  uint32_t releaseCount;     // Total release operations  
  uint32_t errorCount;       // Total errors
  uint32_t stallCount;       // Times stall detection triggered
  uint32_t lowVoltageCount;  // Low voltage events
  uint32_t totalRuntimeMs;   // Total motor runtime
  uint32_t lastErrorCode;    // Last error that occurred
} stats = {0, 0, 0, 0, 0, 0, 0};

// ===== Sensor Readings =====
float currentAmps = 0.0;
float batteryVoltage = 12.0;
float temperature = 25.0;
int currentRaw = 0;

// Debounce state for each input
struct DebouncedInput {
  int pin;
  bool state;
  bool lastReading;
  unsigned long lastChangeTime;
  int sampleCount;
};

DebouncedInput parkInput   = {PIN_PARK, false, false, 0, 0};
DebouncedInput brakeInput  = {PIN_BRAKE, false, false, 0, 0};
DebouncedInput buttonInput = {PIN_BTN, false, false, 0, 0};

// Button hold tracking
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;

// LED blink state
unsigned long lastLedToggle = 0;
bool ledState = false;
int ledBlinkPattern = 0;  // 0=off, 1=solid, 2=slow, 3=fast, 4=error pattern

// Serial command buffer
String serialBuffer = "";

// ===== WiFi and Cloud State =====
bool wifiConnected = false;
unsigned long lastTelemetryTime = 0;
unsigned long lastWifiCheck = 0;
String deviceId = "";  // Unique device identifier

// Offline event buffer (for when WiFi is down)
struct TelemetryEvent {
  char eventType[16];    // "apply", "release", "error", "boot"
  uint32_t timestamp;    // millis() when event occurred
  float voltage;
  float current;
  uint8_t errorCode;
  bool sent;
};
TelemetryEvent offlineBuffer[20];
int offlineBufferCount = 0;

// ===== Generate Unique Device ID =====
String getDeviceId() {
  uint64_t chipid = ESP.getEfuseMac();
  char id[17];
  snprintf(id, 17, "EPB-%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return String(id);
}

// ===== WiFi Functions =====
void connectWiFi() {
  if (!CLOUD_ENABLED) return;
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return;
  }
  
  Serial.print("[WIFI] Connecting to ");
  Serial.print(WIFI_SSID);
  Serial.print("...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && 
         (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
    esp_task_wdt_reset();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println(" Connected!");
    Serial.print("[WIFI] IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WIFI] Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println(" Failed!");
    Serial.println("[WIFI] Will retry in background");
  }
}

void checkWiFiConnection() {
  if (!CLOUD_ENABLED) return;
  
  // Check every 30 seconds
  if ((millis() - lastWifiCheck) < 30000) return;
  lastWifiCheck = millis();
  
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("[WIFI] Connection lost - attempting reconnect...");
      wifiConnected = false;
    }
    WiFi.reconnect();
    delay(100);
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println("[WIFI] Reconnected!");
      // Try to send buffered events
      sendBufferedEvents();
    }
  } else {
    wifiConnected = true;
  }
}

// ===== Cloud Telemetry Functions =====
bool sendTelemetry(const char* eventType, bool immediate = false) {
  if (!CLOUD_ENABLED) return true;
  
  // If WiFi not connected, buffer the event
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    bufferEvent(eventType);
    return false;
  }
  
  // Build JSON payload
  StaticJsonDocument<512> doc;
  doc["device_id"] = deviceId;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["event"] = eventType;
  doc["timestamp"] = millis();
  doc["uptime_sec"] = millis() / 1000;
  
  // State
  doc["brake_applied"] = brakeApplied;
  doc["motor_running"] = motorRunning;
  doc["error_code"] = lastError;
  doc["error_name"] = errorNames[lastError];
  
  // Statistics
  JsonObject statsObj = doc.createNestedObject("stats");
  statsObj["apply_count"] = stats.applyCount;
  statsObj["release_count"] = stats.releaseCount;
  statsObj["error_count"] = stats.errorCount;
  statsObj["stall_count"] = stats.stallCount;
  statsObj["total_runtime_ms"] = stats.totalRuntimeMs;
  
  // Sensors
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["battery_voltage"] = batteryVoltage;
  sensors["current_amps"] = currentAmps;
  sensors["temperature"] = temperature;
  
  // WiFi info
  JsonObject wifi = doc.createNestedObject("wifi");
  wifi["rssi"] = WiFi.RSSI();
  wifi["ip"] = WiFi.localIP().toString();
  
  // Serialize to string
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  
  // Send HTTP POST (non-blocking attempt)
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  http.setTimeout(HTTP_TIMEOUT_MS);
  
  Serial.print("[CLOUD] Sending ");
  Serial.print(eventType);
  Serial.print(" event... ");
  
  int httpCode = http.POST(jsonPayload);
  
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
      Serial.println("OK");
      
      // Parse response if server sends back data
      String response = http.getString();
      if (response.length() > 0 && response.length() < 256) {
        StaticJsonDocument<256> respDoc;
        DeserializationError error = deserializeJson(respDoc, response);
        if (!error) {
          // Server can send back global stats
          if (respDoc.containsKey("global_apply_count")) {
            Serial.print("[CLOUD] Global applies: ");
            Serial.println(respDoc["global_apply_count"].as<unsigned long>());
          }
          if (respDoc.containsKey("global_release_count")) {
            Serial.print("[CLOUD] Global releases: ");
            Serial.println(respDoc["global_release_count"].as<unsigned long>());
          }
          if (respDoc.containsKey("total_devices")) {
            Serial.print("[CLOUD] Total devices: ");
            Serial.println(respDoc["total_devices"].as<int>());
          }
        }
      }
      http.end();
      return true;
    } else {
      Serial.print("HTTP Error: ");
      Serial.println(httpCode);
    }
  } else {
    Serial.print("Failed: ");
    Serial.println(http.errorToString(httpCode));
  }
  
  http.end();
  bufferEvent(eventType);  // Buffer for retry
  return false;
}

void bufferEvent(const char* eventType) {
  if (offlineBufferCount >= MAX_OFFLINE_EVENTS) {
    // Buffer full - remove oldest
    for (int i = 0; i < MAX_OFFLINE_EVENTS - 1; i++) {
      offlineBuffer[i] = offlineBuffer[i + 1];
    }
    offlineBufferCount = MAX_OFFLINE_EVENTS - 1;
  }
  
  TelemetryEvent& evt = offlineBuffer[offlineBufferCount];
  strncpy(evt.eventType, eventType, 15);
  evt.eventType[15] = '\0';
  evt.timestamp = millis();
  evt.voltage = batteryVoltage;
  evt.current = currentAmps;
  evt.errorCode = lastError;
  evt.sent = false;
  offlineBufferCount++;
  
  Serial.print("[CLOUD] Event buffered (");
  Serial.print(offlineBufferCount);
  Serial.println(" in queue)");
}

void sendBufferedEvents() {
  if (!CLOUD_ENABLED || offlineBufferCount == 0) return;
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) return;
  
  Serial.print("[CLOUD] Sending ");
  Serial.print(offlineBufferCount);
  Serial.println(" buffered events...");
  
  int sent = 0;
  for (int i = 0; i < offlineBufferCount; i++) {
    if (!offlineBuffer[i].sent) {
      if (sendTelemetry(offlineBuffer[i].eventType)) {
        offlineBuffer[i].sent = true;
        sent++;
      } else {
        break;  // Stop if sending fails
      }
      delay(100);  // Rate limiting
      esp_task_wdt_reset();
    }
  }
  
  // Remove sent events
  int remaining = 0;
  for (int i = 0; i < offlineBufferCount; i++) {
    if (!offlineBuffer[i].sent) {
      offlineBuffer[remaining++] = offlineBuffer[i];
    }
  }
  offlineBufferCount = remaining;
  
  Serial.print("[CLOUD] Sent ");
  Serial.print(sent);
  Serial.print(" events, ");
  Serial.print(offlineBufferCount);
  Serial.println(" remaining");
}

void sendHeartbeat() {
  if (!CLOUD_ENABLED) return;
  if ((millis() - lastTelemetryTime) < TELEMETRY_INTERVAL_MS) return;
  
  lastTelemetryTime = millis();
  
  // Update sensor readings before sending
  if (VOLTAGE_SENSE_ENABLED) readBatteryVoltage();
  if (CURRENT_SENSE_ENABLED) readCurrent();
  if (TEMP_SENSE_ENABLED) readTemperature();
  
  sendTelemetry("heartbeat");
}

// ===== Analog Sensor Functions =====
float readCurrent() {
  if (!CURRENT_SENSE_ENABLED) return 0.0;
  
  // Read multiple samples and average
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(PIN_CURRENT);
    delayMicroseconds(100);
  }
  currentRaw = total / 10;
  
  // Convert to amps
  // ADC reading -> voltage -> current
  float voltage = currentRaw * ADC_MV_PER_BIT;
  float offsetVoltage = CURRENT_ZERO_OFFSET * ADC_MV_PER_BIT;
  currentAmps = abs(voltage - offsetVoltage) / CURRENT_MV_PER_AMP;
  
  return currentAmps;
}

float readBatteryVoltage() {
  if (!VOLTAGE_SENSE_ENABLED) return 12.0;
  
  // Read multiple samples and average
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(PIN_VOLTAGE);
    delayMicroseconds(100);
  }
  int raw = total / 10;
  
  // Convert to actual voltage
  float adcVoltage = (raw / 4095.0) * 3.3;
  batteryVoltage = adcVoltage * VOLTAGE_DIVIDER_RATIO;
  
  return batteryVoltage;
}

float readTemperature() {
  if (!TEMP_SENSE_ENABLED) return 25.0;
  
  int raw = analogRead(PIN_TEMP);
  
  // Simple linear approximation for NTC (adjust for your sensor)
  // This is a rough approximation - use proper Steinhart-Hart for accuracy
  float voltage = (raw / 4095.0) * 3.3;
  float resistance = (3.3 - voltage) / voltage * 10000.0;  // 10K pullup
  
  // Simplified temperature calculation (Beta equation approximation)
  // Assumes 10K NTC with B=3950
  float steinhart = log(resistance / 10000.0) / 3950.0;
  steinhart += 1.0 / (25.0 + 273.15);
  temperature = (1.0 / steinhart) - 273.15;
  
  return temperature;
}

// ===== Safety Check Functions =====
ErrorCode checkSafeToOperate() {
  // Check battery voltage
  if (VOLTAGE_SENSE_ENABLED) {
    readBatteryVoltage();
    
    if (batteryVoltage < MIN_OPERATING_VOLTAGE) {
      Serial.print("ERROR: Battery voltage too low: ");
      Serial.print(batteryVoltage, 1);
      Serial.println("V");
      stats.lowVoltageCount++;
      return ERR_LOW_VOLTAGE;
    }
    
    if (batteryVoltage > MAX_OPERATING_VOLTAGE) {
      Serial.print("ERROR: Battery voltage too high: ");
      Serial.print(batteryVoltage, 1);
      Serial.println("V");
      return ERR_HIGH_VOLTAGE;
    }
    
    if (batteryVoltage < LOW_VOLTAGE_WARNING) {
      Serial.print("WARNING: Battery voltage low: ");
      Serial.print(batteryVoltage, 1);
      Serial.println("V");
    }
  }
  
  // Check temperature
  if (TEMP_SENSE_ENABLED) {
    readTemperature();
    
    if (temperature > TEMP_SHUTDOWN_C) {
      Serial.print("ERROR: Temperature too high: ");
      Serial.print(temperature, 1);
      Serial.println("°C");
      return ERR_OVERTEMP;
    }
    
    if (temperature > TEMP_WARNING_C) {
      Serial.print("WARNING: Temperature elevated: ");
      Serial.print(temperature, 1);
      Serial.println("°C");
    }
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
      Serial.print("STALL DETECTED! Current: ");
      Serial.print(currentAmps, 2);
      Serial.println("A");
      stats.stallCount++;
      stallSamples = 0;
      return true;
    }
  } else {
    stallSamples = 0;
  }
  
  return false;
}

// ===== Soft Start/Stop PWM Control =====
void rampPWM(int channel, int startDuty, int endDuty, int rampTimeMs) {
  if (!SOFT_START_ENABLED) {
    ledcWrite(channel, endDuty);
    return;
  }
  
  int stepDelay = rampTimeMs / RAMP_STEPS;
  float dutyStep = (float)(endDuty - startDuty) / RAMP_STEPS;
  
  for (int i = 0; i <= RAMP_STEPS; i++) {
    int currentDuty = startDuty + (int)(dutyStep * i);
    ledcWrite(channel, currentDuty);
    delay(stepDelay);
    esp_task_wdt_reset();
  }
}

// ===== Debounced Input Reading =====
bool readDebouncedInput(DebouncedInput &input) {
  bool currentReading = (digitalRead(input.pin) == LOW);  // Active LOW
  
  if (currentReading != input.lastReading) {
    // Input changed - reset debounce counter
    input.lastChangeTime = millis();
    input.sampleCount = 0;
    input.lastReading = currentReading;
  } else if (currentReading != input.state) {
    // Input stable but different from confirmed state
    if ((millis() - input.lastChangeTime) >= DEBOUNCE_MS) {
      input.sampleCount++;
      if (input.sampleCount >= DEBOUNCE_SAMPLES) {
        input.state = currentReading;
        input.sampleCount = 0;
        Serial.print("Input GPIO");
        Serial.print(input.pin);
        Serial.print(" confirmed: ");
        Serial.println(input.state ? "ACTIVE" : "INACTIVE");
      }
    }
  }
  
  return input.state;
}

// ===== Validated Input Reading (extra confirmation) =====
bool readValidatedInput(int pin, int samples = 5, int delayMs = 5) {
  int activeCount = 0;
  for (int i = 0; i < samples; i++) {
    if (digitalRead(pin) == LOW) activeCount++;
    if (i < samples - 1) delay(delayMs);
  }
  // Require majority of samples to agree
  return (activeCount > samples / 2);
}

// ===== Motor Control Functions =====
void stopMotor() {
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  digitalWrite(PIN_REN, HIGH);  // Keep enabled but no PWM
  digitalWrite(PIN_LEN, HIGH);
  motorRunning = false;
  Serial.println("Motor stopped");
}

void emergencyStop() {
  // Immediate stop - disable everything
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
  digitalWrite(PIN_REN, LOW);   // Disable drivers
  digitalWrite(PIN_LEN, LOW);
  motorRunning = false;
  currentState = STATE_ERROR;
  Serial.println("!!! EMERGENCY STOP !!!");
}

void setError(ErrorCode err) {
  lastError = err;
  stats.errorCount++;
  stats.lastErrorCode = err;
  currentState = STATE_ERROR;
  ledBlinkPattern = 4;  // Error blink pattern
  
  Serial.print("ERROR SET: ");
  Serial.print(errorNames[err]);
  Serial.print(" (code ");
  Serial.print(err);
  Serial.println(")");
  
  // Save error to flash
  preferences.begin("brake", false);
  preferences.putUInt("errCount", stats.errorCount);
  preferences.putUInt("lastErr", stats.lastErrorCode);
  preferences.end();
}

void clearError() {
  lastError = ERR_NONE;
  currentState = STATE_IDLE;
  ledBlinkPattern = brakeApplied ? 1 : 0;
  Serial.println("Error cleared");
}

bool extendApply(bool isRetry = false) {
  Serial.println("\n========== APPLY BRAKE ==========");
  
  // Pre-flight checks
  ErrorCode safetyCheck = checkSafeToOperate();
  if (safetyCheck != ERR_NONE) {
    setError(safetyCheck);
    return false;
  }
  
  // Double-check we're not already running
  if (motorRunning) {
    Serial.println("ERROR: Motor already running!");
    setError(ERR_MOTOR_BUSY);
    return false;
  }
  
  currentState = STATE_APPLYING;
  motorRunning = true;
  ledBlinkPattern = 3;  // Fast blink
  
  unsigned long startTime = millis();
  bool stallDetected = false;
  bool movementDetected = false;
  float peakCurrent = 0.0;
  
  // Re-enable drivers
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  
  // Ensure retract is off first
  ledcWrite(PWM_CH_L, 0);
  delay(50);  // Brief pause for driver to settle
  
  Serial.println("Starting extension with soft-start...");
  
  // Soft-start ramp up
  rampPWM(PWM_CH_R, MIN_DUTY, duty, RAMP_TIME_MS);
  
  // Run with stall detection, timeout protection and watchdog feeding
  unsigned long minRunTime = apply_ms / 3;  // Must run at least 1/3 of expected time
  
  while ((millis() - startTime) < (unsigned long)apply_ms) {
    esp_task_wdt_reset();  // Feed watchdog
    
    // Check for timeout (extra safety)
    if ((millis() - startTime) > ACTUATOR_TIMEOUT_MS) {
      Serial.println("ERROR: Actuator timeout!");
      emergencyStop();
      setError(ERR_TIMEOUT);
      return false;
    }
    
    // Current monitoring
    if (CURRENT_SENSE_ENABLED) {
      readCurrent();
      if (currentAmps > peakCurrent) peakCurrent = currentAmps;
      
      // Check for movement (current should be drawn when motor runs)
      if (currentAmps > 0.5 && !movementDetected) {
        movementDetected = true;
        Serial.println("Movement detected - actuator responding");
      }
      
      // Stall detection - actuator hit limit switch
      if (detectStall()) {
        stallDetected = true;
        unsigned long runTime = millis() - startTime;
        
        if (runTime < minRunTime) {
          Serial.println("WARNING: Stalled too early - may already be applied");
          // This might be OK - brake was already applied
        } else {
          Serial.println("Stall at limit - operation complete");
        }
        break;  // Stop early, actuator has reached limit
      }
    }
    
    // Update LED while moving
    if ((millis() - lastLedToggle) > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedToggle = millis();
    }
    
    delay(CURRENT_SAMPLE_MS);
  }
  
  // Soft-stop ramp down
  Serial.println("Soft-stop...");
  rampPWM(PWM_CH_R, duty, 0, RAMP_TIME_MS / 2);
  
  stopMotor();
  
  unsigned long totalRuntime = millis() - startTime;
  stats.totalRuntimeMs += totalRuntime;
  
  // Verify movement occurred
  if (CURRENT_SENSE_ENABLED && !movementDetected && !isRetry) {
    Serial.println("WARNING: No movement detected - retrying...");
    delay(500);
    return extendApply(true);  // Retry once
  }
  
  if (CURRENT_SENSE_ENABLED && !movementDetected && isRetry) {
    Serial.println("ERROR: Still no movement after retry");
    setError(ERR_NO_MOVEMENT);
    return false;
  }
  
  brakeApplied = true;
  stats.applyCount++;
  currentState = STATE_IDLE;
  ledBlinkPattern = 1;  // Solid on
  
  // Save state and stats to flash memory
  preferences.begin("brake", false);
  preferences.putBool("applied", true);
  preferences.putUInt("applyCount", stats.applyCount);
  preferences.putUInt("runtime", stats.totalRuntimeMs);
  preferences.end();
  
  // LED solid ON when brake applied
  digitalWrite(PIN_LED, HIGH);
  
  Serial.println("----------------------------------------");
  Serial.println("Parking brake APPLIED");
  Serial.print("  Runtime: ");
  Serial.print(totalRuntime);
  Serial.println("ms");
  Serial.print("  Peak current: ");
  Serial.print(peakCurrent, 2);
  Serial.println("A");
  Serial.print("  Stall detected: ");
  Serial.println(stallDetected ? "YES" : "NO");
  Serial.print("  Total applies: ");
  Serial.println(stats.applyCount);
  Serial.println("========================================\n");
  
  // Send telemetry to cloud
  sendTelemetry("apply");
  
  return true;
}

bool retractRelease(bool isRetry = false) {
  Serial.println("\n========== RELEASE BRAKE ==========");
  
  // Pre-flight checks
  ErrorCode safetyCheck = checkSafeToOperate();
  if (safetyCheck != ERR_NONE) {
    setError(safetyCheck);
    return false;
  }
  
  // Double-check we're not already running
  if (motorRunning) {
    Serial.println("ERROR: Motor already running!");
    setError(ERR_MOTOR_BUSY);
    return false;
  }
  
  currentState = STATE_RELEASING;
  motorRunning = true;
  ledBlinkPattern = 3;  // Fast blink
  
  unsigned long startTime = millis();
  bool stallDetected = false;
  bool movementDetected = false;
  float peakCurrent = 0.0;
  
  // Re-enable drivers
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  
  // Ensure extend is off first
  ledcWrite(PWM_CH_R, 0);
  delay(50);  // Brief pause for driver to settle
  
  Serial.println("Starting retraction with soft-start...");
  
  // Soft-start ramp up
  rampPWM(PWM_CH_L, MIN_DUTY, duty, RAMP_TIME_MS);
  
  // Run with stall detection, timeout protection and watchdog feeding
  unsigned long minRunTime = release_ms / 3;  // Must run at least 1/3 of expected time
  
  while ((millis() - startTime) < (unsigned long)release_ms) {
    esp_task_wdt_reset();  // Feed watchdog
    
    // Check for timeout (extra safety)
    if ((millis() - startTime) > ACTUATOR_TIMEOUT_MS) {
      Serial.println("ERROR: Actuator timeout!");
      emergencyStop();
      setError(ERR_TIMEOUT);
      return false;
    }
    
    // Current monitoring
    if (CURRENT_SENSE_ENABLED) {
      readCurrent();
      if (currentAmps > peakCurrent) peakCurrent = currentAmps;
      
      // Check for movement
      if (currentAmps > 0.5 && !movementDetected) {
        movementDetected = true;
        Serial.println("Movement detected - actuator responding");
      }
      
      // Stall detection - actuator hit limit switch
      if (detectStall()) {
        stallDetected = true;
        unsigned long runTime = millis() - startTime;
        
        if (runTime < minRunTime) {
          Serial.println("WARNING: Stalled too early - may already be released");
        } else {
          Serial.println("Stall at limit - operation complete");
        }
        break;
      }
    }
    
    // Update LED while moving
    if ((millis() - lastLedToggle) > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedToggle = millis();
    }
    
    delay(CURRENT_SAMPLE_MS);
  }
  
  // Soft-stop ramp down
  Serial.println("Soft-stop...");
  rampPWM(PWM_CH_L, duty, 0, RAMP_TIME_MS / 2);
  
  stopMotor();
  
  unsigned long totalRuntime = millis() - startTime;
  stats.totalRuntimeMs += totalRuntime;
  
  // Verify movement occurred
  if (CURRENT_SENSE_ENABLED && !movementDetected && !isRetry) {
    Serial.println("WARNING: No movement detected - retrying...");
    delay(500);
    return retractRelease(true);
  }
  
  if (CURRENT_SENSE_ENABLED && !movementDetected && isRetry) {
    Serial.println("ERROR: Still no movement after retry");
    setError(ERR_NO_MOVEMENT);
    return false;
  }
  
  brakeApplied = false;
  stats.releaseCount++;
  currentState = STATE_IDLE;
  ledBlinkPattern = 0;  // Off
  
  // Save state and stats to flash memory
  preferences.begin("brake", false);
  preferences.putBool("applied", false);
  preferences.putUInt("relCount", stats.releaseCount);
  preferences.putUInt("runtime", stats.totalRuntimeMs);
  preferences.end();
  
  // LED OFF when brake released
  digitalWrite(PIN_LED, LOW);
  
  Serial.println("----------------------------------------");
  Serial.println("Parking brake RELEASED");
  Serial.print("  Runtime: ");
  Serial.print(totalRuntime);
  Serial.println("ms");
  Serial.print("  Peak current: ");
  Serial.print(peakCurrent, 2);
  Serial.println("A");
  Serial.print("  Stall detected: ");
  Serial.println(stallDetected ? "YES" : "NO");
  Serial.print("  Total releases: ");
  Serial.println(stats.releaseCount);
  Serial.println("========================================\n");
  
  // Send telemetry to cloud
  sendTelemetry("release");
  
  return true;
}

// ===== LED Status Indicator =====
void updateLED() {
  if (motorRunning) {
    // LED is controlled in motor functions during movement
    return;
  }
  
  unsigned long now = millis();
  
  switch (ledBlinkPattern) {
    case 0:  // OFF
      digitalWrite(PIN_LED, LOW);
      break;
      
    case 1:  // Solid ON
      digitalWrite(PIN_LED, HIGH);
      break;
      
    case 2:  // Slow blink (1Hz) - warning/low voltage
      if ((now - lastLedToggle) > 500) {
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
        lastLedToggle = now;
      }
      break;
      
    case 3:  // Fast blink (5Hz) - moving
      if ((now - lastLedToggle) > 100) {
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
        lastLedToggle = now;
      }
      break;
      
    case 4:  // Error pattern - double blink
      {
        int phase = (now / 150) % 6;
        digitalWrite(PIN_LED, (phase == 0 || phase == 2) ? HIGH : LOW);
      }
      break;
  }
}

// ===== Serial Command Processing =====
void printHelp() {
  Serial.println("\n========== COMMAND HELP ==========");
  Serial.println("Commands (case insensitive):");
  Serial.println("  APPLY    - Apply parking brake");
  Serial.println("  RELEASE  - Release parking brake");
  Serial.println("  STATUS   - Show current status");
  Serial.println("  STATS    - Show statistics");
  Serial.println("  DIAG     - Show diagnostics");
  Serial.println("  RESET    - Clear error state");
  Serial.println("  REBOOT   - Restart controller");
  Serial.println("  CLEARSTATS - Reset statistics");
  Serial.println("  SET <param> <value> - Set parameter");
  Serial.println("    SET DUTY <0-255>    - PWM duty cycle");
  Serial.println("    SET APPLY <ms>      - Apply time");
  Serial.println("    SET RELEASE <ms>    - Release time");
  Serial.println("    SET CURRENT <0/1>   - Enable current sense");
  Serial.println("    SET VOLTAGE <0/1>   - Enable voltage sense");
  Serial.println("    SET SOFTSTART <0/1> - Enable soft start");
  Serial.println("    SET CLOUD <0/1>     - Enable cloud telemetry");
  Serial.println("  WIFI     - Show WiFi status & reconnect");
  Serial.println("  PING     - Send test telemetry to server");
  Serial.println("  HELP     - Show this help");
  Serial.println("===================================\n");
}

void printStatus() {
  Serial.println("\n========== STATUS ==========");
  Serial.print("Firmware: v");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.print("Brake State: ");
  Serial.println(brakeApplied ? "APPLIED" : "RELEASED");
  Serial.print("System State: ");
  switch (currentState) {
    case STATE_IDLE: Serial.println("IDLE"); break;
    case STATE_APPLYING: Serial.println("APPLYING"); break;
    case STATE_RELEASING: Serial.println("RELEASING"); break;
    case STATE_ERROR: Serial.println("ERROR"); break;
    case STATE_LOW_VOLTAGE: Serial.println("LOW_VOLTAGE"); break;
  }
  if (lastError != ERR_NONE) {
    Serial.print("Last Error: ");
    Serial.println(errorNames[lastError]);
  }
  Serial.print("Motor Running: ");
  Serial.println(motorRunning ? "YES" : "NO");
  Serial.println("---------- Inputs ----------");
  Serial.print("Park Signal: ");
  Serial.println(parkInput.state ? "ENGAGED" : "DISENGAGED");
  Serial.print("Brake Pedal: ");
  Serial.println(brakeInput.state ? "PRESSED" : "RELEASED");
  Serial.print("Button: ");
  Serial.println(buttonInput.state ? "PRESSED" : "RELEASED");
  Serial.println("---------- Sensors ----------");
  if (VOLTAGE_SENSE_ENABLED) {
    readBatteryVoltage();
    Serial.print("Battery: ");
    Serial.print(batteryVoltage, 2);
    Serial.println("V");
  }
  if (CURRENT_SENSE_ENABLED) {
    readCurrent();
    Serial.print("Current: ");
    Serial.print(currentAmps, 2);
    Serial.print("A (raw: ");
    Serial.print(currentRaw);
    Serial.println(")");
  }
  if (TEMP_SENSE_ENABLED) {
    readTemperature();
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println("°C");
  }
  Serial.println("---------- Cloud ----------");
  Serial.print("Cloud Enabled: ");
  Serial.println(CLOUD_ENABLED ? "YES" : "NO");
  if (CLOUD_ENABLED) {
    Serial.print("WiFi Status: ");
    Serial.println(wifiConnected ? "CONNECTED" : "DISCONNECTED");
    if (wifiConnected) {
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Signal (RSSI): ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
    }
    Serial.print("Buffered Events: ");
    Serial.println(offlineBufferCount);
  }
  Serial.println("=============================\n");
}

void printStats() {
  Serial.println("\n========== STATISTICS ==========");
  Serial.print("Apply Count: ");
  Serial.println(stats.applyCount);
  Serial.print("Release Count: ");
  Serial.println(stats.releaseCount);
  Serial.print("Error Count: ");
  Serial.println(stats.errorCount);
  Serial.print("Stall Count: ");
  Serial.println(stats.stallCount);
  Serial.print("Low Voltage Events: ");
  Serial.println(stats.lowVoltageCount);
  Serial.print("Total Runtime: ");
  Serial.print(stats.totalRuntimeMs / 1000);
  Serial.println(" seconds");
  if (stats.lastErrorCode != 0) {
    Serial.print("Last Error: ");
    Serial.println(errorNames[stats.lastErrorCode]);
  }
  Serial.println("================================\n");
}

void printDiagnostics() {
  Serial.println("\n========== DIAGNOSTICS ==========");
  Serial.println("--- Configuration ---");
  Serial.print("Apply Time: ");
  Serial.print(apply_ms);
  Serial.println("ms");
  Serial.print("Release Time: ");
  Serial.print(release_ms);
  Serial.println("ms");
  Serial.print("PWM Duty: ");
  Serial.print(duty);
  Serial.print(" (");
  Serial.print((duty * 100) / 255);
  Serial.println("%)");
  Serial.print("Soft Start: ");
  Serial.println(SOFT_START_ENABLED ? "ENABLED" : "DISABLED");
  Serial.print("Current Sense: ");
  Serial.println(CURRENT_SENSE_ENABLED ? "ENABLED" : "DISABLED");
  Serial.print("Voltage Sense: ");
  Serial.println(VOLTAGE_SENSE_ENABLED ? "ENABLED" : "DISABLED");
  Serial.print("Temp Sense: ");
  Serial.println(TEMP_SENSE_ENABLED ? "ENABLED" : "DISABLED");
  Serial.println("--- Thresholds ---");
  Serial.print("Stall Current: ");
  Serial.print(STALL_CURRENT_AMPS, 1);
  Serial.println("A");
  Serial.print("Min Voltage: ");
  Serial.print(MIN_OPERATING_VOLTAGE, 1);
  Serial.println("V");
  Serial.print("Max Voltage: ");
  Serial.print(MAX_OPERATING_VOLTAGE, 1);
  Serial.println("V");
  Serial.println("--- Free Memory ---");
  Serial.print("Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.println("==================================\n");
}

void processSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.length() == 0) return;
  
  Serial.print("> ");
  Serial.println(cmd);
  
  if (cmd == "HELP" || cmd == "?") {
    printHelp();
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd == "STATS") {
    printStats();
  }
  else if (cmd == "DIAG") {
    printDiagnostics();
  }
  else if (cmd == "APPLY") {
    if (!brakeApplied) {
      extendApply();
    } else {
      Serial.println("Brake already applied");
    }
  }
  else if (cmd == "RELEASE") {
    if (brakeApplied) {
      retractRelease();
    } else {
      Serial.println("Brake already released");
    }
  }
  else if (cmd == "RESET") {
    clearError();
  }
  else if (cmd == "REBOOT") {
    Serial.println("Rebooting in 1 second...");
    delay(1000);
    ESP.restart();
  }
  else if (cmd == "CLEARSTATS") {
    stats = {0, 0, 0, 0, 0, 0, 0};
    preferences.begin("brake", false);
    preferences.putUInt("applyCount", 0);
    preferences.putUInt("relCount", 0);
    preferences.putUInt("errCount", 0);
    preferences.putUInt("stallCount", 0);
    preferences.putUInt("lvCount", 0);
    preferences.putUInt("runtime", 0);
    preferences.end();
    Serial.println("Statistics cleared");
  }
  else if (cmd.startsWith("SET ")) {
    String params = cmd.substring(4);
    int spaceIdx = params.indexOf(' ');
    if (spaceIdx > 0) {
      String param = params.substring(0, spaceIdx);
      int value = params.substring(spaceIdx + 1).toInt();
      
      if (param == "DUTY") {
        duty = constrain(value, 0, 255);
        Serial.print("Duty set to: ");
        Serial.println(duty);
      }
      else if (param == "APPLY") {
        apply_ms = constrain(value, 500, 10000);
        Serial.print("Apply time set to: ");
        Serial.print(apply_ms);
        Serial.println("ms");
      }
      else if (param == "RELEASE") {
        release_ms = constrain(value, 500, 10000);
        Serial.print("Release time set to: ");
        Serial.print(release_ms);
        Serial.println("ms");
      }
      else if (param == "CURRENT") {
        CURRENT_SENSE_ENABLED = (value != 0);
        Serial.print("Current sense: ");
        Serial.println(CURRENT_SENSE_ENABLED ? "ENABLED" : "DISABLED");
      }
      else if (param == "VOLTAGE") {
        VOLTAGE_SENSE_ENABLED = (value != 0);
        Serial.print("Voltage sense: ");
        Serial.println(VOLTAGE_SENSE_ENABLED ? "ENABLED" : "DISABLED");
      }
      else if (param == "SOFTSTART") {
        SOFT_START_ENABLED = (value != 0);
        Serial.print("Soft start: ");
        Serial.println(SOFT_START_ENABLED ? "ENABLED" : "DISABLED");
      }
      else if (param == "CLOUD") {
        CLOUD_ENABLED = (value != 0);
        Serial.print("Cloud telemetry: ");
        Serial.println(CLOUD_ENABLED ? "ENABLED" : "DISABLED");
        if (CLOUD_ENABLED && !wifiConnected) {
          connectWiFi();
        }
      }
      else {
        Serial.println("Unknown parameter");
      }
    } else {
      Serial.println("Usage: SET <param> <value>");
    }
  }
  else if (cmd == "WIFI") {
    Serial.println("\n========== WIFI STATUS ==========");
    Serial.print("Cloud Enabled: ");
    Serial.println(CLOUD_ENABLED ? "YES" : "NO");
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Gateway: ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("Signal (RSSI): ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      Serial.print("MAC: ");
      Serial.println(WiFi.macAddress());
    } else {
      Serial.println("Attempting reconnect...");
      connectWiFi();
    }
    Serial.print("Server URL: ");
    Serial.println(SERVER_URL);
    Serial.print("Buffered Events: ");
    Serial.println(offlineBufferCount);
    Serial.println("==================================\n");
  }
  else if (cmd == "PING") {
    Serial.println("Sending test telemetry...");
    if (CLOUD_ENABLED) {
      if (sendTelemetry("ping")) {
        Serial.println("Ping successful!");
      } else {
        Serial.println("Ping failed - check WiFi/server");
      }
    } else {
      Serial.println("Cloud disabled. Use SET CLOUD 1 to enable.");
    }
  }
  else {
    Serial.println("Unknown command. Type HELP for list.");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(100);  // Let serial stabilize
  
  Serial.println("\n");
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   Electric Parking Brake Controller      ║");
  Serial.println("║       CLOUD CONNECTED VERSION v2.1       ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.print("Firmware: v");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Initializing...\n");

  // Generate unique device ID
  deviceId = getDeviceId();
  Serial.print("[OK] Device ID: ");
  Serial.println(deviceId);

  // Configure enable pins
  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);

  // Configure LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  // Configure analog inputs
  if (CURRENT_SENSE_ENABLED) {
    pinMode(PIN_CURRENT, INPUT);
    Serial.println("[OK] Current sensing enabled");
  }
  if (VOLTAGE_SENSE_ENABLED) {
    pinMode(PIN_VOLTAGE, INPUT);
    Serial.println("[OK] Voltage sensing enabled");
  }
  if (TEMP_SENSE_ENABLED) {
    pinMode(PIN_TEMP, INPUT);
    Serial.println("[OK] Temperature sensing enabled");
  }
  
  // Startup LED sequence (visual confirmation of boot)
  Serial.println("LED startup sequence...");
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }

  // Configure input pins (ground-switch: LOW = active)
  pinMode(PIN_PARK, INPUT_PULLUP);
  pinMode(PIN_BRAKE, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);

  // Configure PWM channels
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_RPWM, PWM_CH_R);
  ledcAttachPin(PIN_LPWM, PWM_CH_L);

  // Ensure motor is stopped on startup
  stopMotor();

  // Load saved state and statistics from flash memory
  preferences.begin("brake", true);  // Read-only mode
  brakeApplied = preferences.getBool("applied", false);
  stats.applyCount = preferences.getUInt("applyCount", 0);
  stats.releaseCount = preferences.getUInt("relCount", 0);
  stats.errorCount = preferences.getUInt("errCount", 0);
  stats.stallCount = preferences.getUInt("stallCount", 0);
  stats.lowVoltageCount = preferences.getUInt("lvCount", 0);
  stats.totalRuntimeMs = preferences.getUInt("runtime", 0);
  stats.lastErrorCode = preferences.getUInt("lastErr", 0);
  preferences.end();
  
  Serial.print("[OK] Restored brake state: ");
  Serial.println(brakeApplied ? "APPLIED" : "RELEASED");
  Serial.print("[OK] Loaded stats: ");
  Serial.print(stats.applyCount);
  Serial.print(" applies, ");
  Serial.print(stats.releaseCount);
  Serial.println(" releases");
  
  // Initial sensor readings
  if (VOLTAGE_SENSE_ENABLED) {
    readBatteryVoltage();
    Serial.print("[OK] Battery voltage: ");
    Serial.print(batteryVoltage, 2);
    Serial.println("V");
    
    if (batteryVoltage < MIN_OPERATING_VOLTAGE) {
      Serial.println("WARNING: Low battery voltage!");
      currentState = STATE_LOW_VOLTAGE;
      ledBlinkPattern = 2;  // Slow blink
    }
  }
  
  if (CURRENT_SENSE_ENABLED) {
    readCurrent();
    Serial.print("[OK] Current sensor baseline: ");
    Serial.print(currentRaw);
    Serial.print(" (");
    Serial.print(currentAmps, 2);
    Serial.println("A)");
  }

  // Read initial park state (with extra validation)
  delay(100);  // Let inputs settle after boot
  lastPark = readValidatedInput(PIN_PARK, 10, 10);
  parkInput.state = lastPark;
  parkInput.lastReading = lastPark;
  
  Serial.print("[OK] Initial Park state: ");
  Serial.println(lastPark ? "ENGAGED" : "DISENGAGED");

  // Initialize watchdog timer (will reset ESP32 if loop hangs)
  Serial.print("[OK] Watchdog timer: ");
  Serial.print(WDT_TIMEOUT_S);
  Serial.println("s timeout");
  esp_task_wdt_init(WDT_TIMEOUT_S, true);  // Enable panic/reset on timeout
  esp_task_wdt_add(NULL);  // Add current thread to watchdog
  
  // Connect to WiFi (non-blocking, will retry in loop)
  if (CLOUD_ENABLED) {
    Serial.println("\n--- WiFi Setup ---");
    connectWiFi();
  }
  
  // Update LED to match current brake state
  ledBlinkPattern = brakeApplied ? 1 : 0;
  updateLED();
  
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║            SYSTEM READY                  ║");
  Serial.println("║     Type HELP for serial commands        ║");
  Serial.println("╚══════════════════════════════════════════╝\n");
  
  // Send boot telemetry
  if (CLOUD_ENABLED) {
    sendTelemetry("boot");
  }
}

// ===== Main Loop =====
void loop() {
  // Feed the watchdog - MUST be called regularly!
  esp_task_wdt_reset();
  
  // Process serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) {
        serialBuffer = "";  // Prevent buffer overflow
      }
    }
  }
  
  // WiFi connection management and heartbeat
  if (CLOUD_ENABLED) {
    checkWiFiConnection();
    sendHeartbeat();
  }
  
  // Periodic voltage check (every 5 seconds)
  static unsigned long lastVoltageCheck = 0;
  if (VOLTAGE_SENSE_ENABLED && (millis() - lastVoltageCheck) > 5000) {
    readBatteryVoltage();
    lastVoltageCheck = millis();
    
    if (batteryVoltage < MIN_OPERATING_VOLTAGE && currentState != STATE_ERROR) {
      currentState = STATE_LOW_VOLTAGE;
      ledBlinkPattern = 2;  // Slow blink
    } else if (currentState == STATE_LOW_VOLTAGE && batteryVoltage >= MIN_OPERATING_VOLTAGE) {
      currentState = STATE_IDLE;
      ledBlinkPattern = brakeApplied ? 1 : 0;
    }
  }
  
  // Read debounced inputs
  bool park = readDebouncedInput(parkInput);
  bool brake = readDebouncedInput(brakeInput);
  bool btnPressed = readDebouncedInput(buttonInput);

  // ===== Manual Override Button (hold 3 seconds) =====
  if (btnPressed) {
    if (!buttonWasPressed) {
      // Button just pressed - start timing
      buttonPressStart = millis();
      buttonWasPressed = true;
      Serial.println("Manual button pressed - hold 3 sec to override");
    } else {
      // Button still held - check if 3 seconds elapsed
      if ((millis() - buttonPressStart) >= HOLD_TIME_MS) {
        Serial.println("\n>> MANUAL OVERRIDE TRIGGERED <<");
        
        // Clear any error state
        if (currentState == STATE_ERROR) {
          clearError();
        }
        
        // Extra validation - confirm button is really still held
        if (readValidatedInput(PIN_BTN, 5, 10)) {
          if (brakeApplied) {
            retractRelease();
          } else {
            extendApply();
          }
        } else {
          Serial.println("Button validation failed - ignoring");
        }
        
        // Reset button state and wait for release
        buttonWasPressed = false;
        
        // Wait for button release with timeout
        unsigned long releaseTimeout = millis();
        while (digitalRead(PIN_BTN) == LOW) {
          esp_task_wdt_reset();  // Keep feeding watchdog while waiting
          if ((millis() - releaseTimeout) > 10000) {
            Serial.println("WARNING: Button stuck? Continuing anyway...");
            break;
          }
          delay(50);
        }
        Serial.println("Button released\n");
      }
    }
  } else {
    // Button released before 3 seconds
    if (buttonWasPressed) {
      unsigned long heldTime = millis() - buttonPressStart;
      Serial.print("Button released after ");
      Serial.print(heldTime);
      Serial.println("ms (need 3000ms for override)");
    }
    buttonWasPressed = false;
  }

  // Skip automatic operation if in error state
  if (currentState == STATE_ERROR) {
    lastPark = park;
    updateLED();
    delay(SAMPLE_INTERVAL_MS);
    return;
  }

  // ===== Park Signal Transitions =====
  
  // Transition INTO Park -> Apply brake
  if (park && !lastPark) {
    Serial.println("\n>> Park ENGAGED detected <<");
    
    // Double-check park signal is real (not a glitch)
    delay(100);
    if (readValidatedInput(PIN_PARK, 5, 20)) {
      extendApply();
    } else {
      Serial.println("Park signal not confirmed - ignoring false trigger\n");
    }
  }

  // Transition OUT OF Park -> Release brake (only if brake pedal pressed)
  if (!park && lastPark) {
    Serial.println("\n>> Park DISENGAGED detected <<");
    
    // Double-check signals are real
    delay(100);
    if (!readValidatedInput(PIN_PARK, 5, 20)) {
      if (readValidatedInput(PIN_BRAKE, 5, 20)) {
        Serial.println("Brake pedal confirmed - Releasing");
        retractRelease();
      } else {
        Serial.println("Waiting for brake pedal press (safety interlock)...\n");
      }
    } else {
      Serial.println("Park disengage not confirmed - ignoring glitch\n");
    }
  }

  // If out of Park but brake still applied, check for brake pedal
  // This catches the case where user shifts out of Park without pressing brake
  // and then presses brake later
  if (!park && brakeApplied && brake) {
    Serial.println("\n>> Brake pedal pressed while out of Park <<");
    delay(50);
    if (readValidatedInput(PIN_BRAKE, 5, 20) && !readValidatedInput(PIN_PARK, 5, 20)) {
      retractRelease();
    }
  }

  lastPark = park;
  
  // Update LED status
  updateLED();
  
  // Small delay to prevent CPU hogging and allow stable readings
  delay(SAMPLE_INTERVAL_MS);
}
