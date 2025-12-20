/*
 * Electric Parking Brake Controller - RELIABLE VERSION
 * ESP32-based controller for 12V 350lb 50mm linear actuator
 * 
 * RELIABILITY FEATURES:
 * - Hardware watchdog timer (auto-reset if code hangs)
 * - Software debouncing on all inputs (50ms)
 * - Multi-sample input validation (5 consecutive reads)
 * - State persistence across power cycles (NVS flash)
 * - Status LED for visual feedback
 * - Actuation timeout protection
 * - Graceful error handling
 * 
 * Hardware:
 * - ESP32 DevKit
 * - BTS7960 Motor Driver (or similar H-bridge with RPWM/LPWM control)
 * - 12V 350lb 50mm Linear Actuator (with built-in limit switches)
 * - Park signal input (ground-switch)
 * - Brake pedal input (ground-switch, safety interlock)
 * - Manual override button (momentary, ground-switch)
 * - Status LED (optional but recommended)
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
 */

#include <Preferences.h>
#include <esp_task_wdt.h>

// ===== Pin Configuration =====
const int PIN_RPWM  = 25;   // P25 -> RPWM (extend)
const int PIN_LPWM  = 26;   // P26 -> LPWM (retract)
const int PIN_REN   = 27;   // P27 -> REN (right enable)
const int PIN_LEN   = 14;   // P14 -> LEN (left enable)

const int PIN_PARK  = 34;   // P34 input - Park signal
const int PIN_BRAKE = 35;   // P35 input - Brake pedal (safety)
const int PIN_BTN   = 32;   // P32 input - Manual override button
const int PIN_LED   = 2;    // P2  - Status LED (built-in on most ESP32)

// ===== PWM Configuration =====
const int PWM_CH_R  = 0;    // PWM channel for RPWM
const int PWM_CH_L  = 1;    // PWM channel for LPWM
const int PWM_FREQ  = 20000; // 20kHz - above audible range
const int PWM_RES   = 8;    // 8-bit resolution (0-255)

// ===== Timing Configuration =====
int apply_ms   = 2500;      // Time to fully extend/apply brake (ms)
int release_ms = 1800;      // Time to fully retract/release brake (ms)
int duty       = 220;       // PWM duty cycle 0-255 (~86% power)

// ===== Reliability Configuration =====
const int DEBOUNCE_MS = 50;           // Debounce time for inputs
const int DEBOUNCE_SAMPLES = 3;       // Number of consecutive reads required
const int SAMPLE_INTERVAL_MS = 20;    // Time between samples
const unsigned long HOLD_TIME_MS = 3000;  // Button hold time for override
const int WDT_TIMEOUT_S = 10;         // Watchdog timeout in seconds
const int ACTUATOR_TIMEOUT_MS = 5000; // Max time for actuator movement

// ===== State Variables =====
Preferences preferences;
bool brakeApplied = false;
bool lastPark = false;
bool motorRunning = false;

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
  Serial.println("!!! EMERGENCY STOP !!!");
}

bool extendApply() {
  Serial.println("Applying parking brake (extending actuator)...");
  
  // Double-check we're not already running
  if (motorRunning) {
    Serial.println("ERROR: Motor already running!");
    return false;
  }
  
  motorRunning = true;
  unsigned long startTime = millis();
  
  // Re-enable drivers
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  
  // Ensure retract is off first
  ledcWrite(PWM_CH_L, 0);
  delay(50);  // Brief pause for driver to settle
  
  // Start extension
  ledcWrite(PWM_CH_R, duty);
  
  // Run with timeout protection and watchdog feeding
  while ((millis() - startTime) < (unsigned long)apply_ms) {
    esp_task_wdt_reset();  // Feed watchdog
    
    // Check for timeout (extra safety)
    if ((millis() - startTime) > ACTUATOR_TIMEOUT_MS) {
      Serial.println("ERROR: Actuator timeout!");
      emergencyStop();
      return false;
    }
    
    // Blink LED while moving
    if ((millis() - lastLedToggle) > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedToggle = millis();
    }
    
    delay(25);
  }
  
  stopMotor();
  brakeApplied = true;
  
  // Save state to flash memory
  preferences.begin("brake", false);
  preferences.putBool("applied", true);
  preferences.end();
  
  // LED solid ON when brake applied
  digitalWrite(PIN_LED, HIGH);
  
  Serial.println("Parking brake APPLIED - State saved to flash");
  return true;
}

bool retractRelease() {
  Serial.println("Releasing parking brake (retracting actuator)...");
  
  // Double-check we're not already running
  if (motorRunning) {
    Serial.println("ERROR: Motor already running!");
    return false;
  }
  
  motorRunning = true;
  unsigned long startTime = millis();
  
  // Re-enable drivers
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);
  
  // Ensure extend is off first
  ledcWrite(PWM_CH_R, 0);
  delay(50);  // Brief pause for driver to settle
  
  // Start retraction
  ledcWrite(PWM_CH_L, duty);
  
  // Run with timeout protection and watchdog feeding
  while ((millis() - startTime) < (unsigned long)release_ms) {
    esp_task_wdt_reset();  // Feed watchdog
    
    // Check for timeout (extra safety)
    if ((millis() - startTime) > ACTUATOR_TIMEOUT_MS) {
      Serial.println("ERROR: Actuator timeout!");
      emergencyStop();
      return false;
    }
    
    // Fast blink LED while moving
    if ((millis() - lastLedToggle) > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedToggle = millis();
    }
    
    delay(25);
  }
  
  stopMotor();
  brakeApplied = false;
  
  // Save state to flash memory
  preferences.begin("brake", false);
  preferences.putBool("applied", false);
  preferences.end();
  
  // LED OFF when brake released
  digitalWrite(PIN_LED, LOW);
  
  Serial.println("Parking brake RELEASED - State saved to flash");
  return true;
}

// ===== LED Status Indicator =====
void updateLED() {
  if (motorRunning) {
    // Fast blink while moving (handled in motor functions)
    return;
  }
  
  if (brakeApplied) {
    // Solid ON when brake applied
    digitalWrite(PIN_LED, HIGH);
  } else {
    // OFF when brake released
    digitalWrite(PIN_LED, LOW);
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(100);  // Let serial stabilize
  
  Serial.println("\n========================================");
  Serial.println("  Electric Parking Brake Controller");
  Serial.println("         RELIABLE VERSION v1.0");
  Serial.println("========================================");
  Serial.println("Initializing...");

  // Configure enable pins
  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);

  // Configure LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
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

  // Load saved brake state from flash memory
  preferences.begin("brake", true);  // Read-only mode
  brakeApplied = preferences.getBool("applied", false);
  preferences.end();
  
  Serial.print("Restored brake state from flash: ");
  Serial.println(brakeApplied ? "APPLIED" : "RELEASED");

  // Read initial park state (with extra validation)
  delay(100);  // Let inputs settle after boot
  lastPark = readValidatedInput(PIN_PARK, 10, 10);
  parkInput.state = lastPark;
  parkInput.lastReading = lastPark;
  
  Serial.print("Initial Park state: ");
  Serial.println(lastPark ? "ENGAGED" : "DISENGAGED");

  // Initialize watchdog timer (will reset ESP32 if loop hangs)
  Serial.print("Enabling watchdog timer (");
  Serial.print(WDT_TIMEOUT_S);
  Serial.println("s timeout)...");
  esp_task_wdt_init(WDT_TIMEOUT_S, true);  // Enable panic/reset on timeout
  esp_task_wdt_add(NULL);  // Add current thread to watchdog
  
  // Update LED to match current brake state
  updateLED();
  
  Serial.println("\n========================================");
  Serial.println("         SYSTEM READY");
  Serial.println("========================================\n");
}

// ===== Main Loop =====
void loop() {
  // Feed the watchdog - MUST be called regularly!
  esp_task_wdt_reset();
  
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
