<p align="center">
  <img src="https://img.shields.io/badge/Version-3.1%20RUGGED-ff6b35?style=for-the-badge" alt="Version 3.1 RUGGED">
  <img src="https://img.shields.io/badge/Platform-ESP32-blue?style=for-the-badge&logo=espressif" alt="ESP32">
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="MIT License">
</p>

<h1 align="center">Electric Parking Brake Controller</h1>

<p align="center">
  <b>Automotive-grade controller for 12V linear actuators with built-in mobile web UI</b>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/WiFi-Access%20Point-blueviolet?style=flat-square" alt="WiFi AP">
  <img src="https://img.shields.io/badge/UI-Mobile%20Optimized-orange?style=flat-square" alt="Mobile UI">
  <img src="https://img.shields.io/badge/WebSocket-Real--Time-brightgreen?style=flat-square" alt="WebSocket">
  <img src="https://img.shields.io/badge/Works-Offline-success?style=flat-square" alt="Works Offline">
</p>

---

## Connect to Your Brake

```
1. Connect to WiFi: ParkingBrake (password: brake1234)
2. Open browser: http://192.168.4.1
3. Control your brake!
```

**No internet required!** The ESP32 creates its own WiFi network.

---

## Features

### RUGGED Edition (v3.1)

| Feature | Description |
|---------|-------------|
| Hardware Watchdog | Auto-reboot if firmware freezes (8s timeout) |
| Voltage Spike Filtering | 20-sample median filter rejects transients |
| EMI-Resistant Inputs | Multi-sample validation with timing spread |
| Auto-Retry | Transient failures retry automatically |
| Self-Healing WiFi | AP restarts if it fails |
| Memory Protection | Canary values detect RAM corruption |
| Voltage Protection | Blocks operation outside 11-16V range |
| Boot Counter | Track reboots for reliability analysis |

### Standalone Features (v3.0)

- **WiFi Access Point** - Connect directly like a router
- **iPhone-Optimized UI** - Beautiful dark theme, touch-friendly
- **Real-Time Updates** - WebSocket for instant feedback
- **Works Offline** - Perfect for vehicle installation

### Core Features

- **Auto Apply** - When Park input becomes active
- **Safe Release** - Requires brake pedal to release
- **Manual Override** - Hold button ~3 seconds to toggle
- **State Persistence** - Remembers position across power cycles

### Professional Features

- **Current Sensing** - Stall detection via ACS712
- **Soft Start/Stop** - PWM ramping reduces stress
- **Detailed Diagnostics** - Error codes, statistics

---

## Hardware Requirements

### Essential

| Component | Specification |
|-----------|---------------|
| ESP32 DevKit | Arduino-compatible |
| BTS7960 | Motor driver (43A H-Bridge) |
| Linear Actuator | 12V, 350lb, 50mm stroke, with limit switches |
| Buck Converter | 12V to 5V, 3A+ |
| Fuse | 10A inline |
| Button | Momentary pushbutton |

### Optional (Recommended)

| Component | Purpose |
|-----------|---------|
| ACS712-30A | Current sensing on GPIO33 |
| 47K + 10K Resistors | Voltage divider on GPIO36 |

---

## Wiring

### ESP32 to BTS7960

| ESP32 | BTS7960 | Function |
|:-----:|:-------:|----------|
| 25 | RPWM | Extend/Apply |
| 26 | LPWM | Retract/Release |
| 27 | R_EN | Enable |
| 14 | L_EN | Enable |
| GND | GND | Ground |

### Inputs (Active LOW)

| GPIO | Signal |
|:----:|--------|
| 34 | Park signal |
| 35 | Brake pedal |
| 32 | Manual button |
| 2 | Status LED |

### Optional Sensors

| GPIO | Sensor |
|:----:|--------|
| 33 | ACS712 current sensor |
| 36 | Voltage divider (47K+10K) |

---

## Installation

### 1. Install Libraries

In Arduino IDE: **Sketch > Include Library > Manage Libraries**

| Library | Author |
|---------|--------|
| WebSockets | Markus Sattler |
| ArduinoJson | Benoit Blanchon |

### 2. Upload Firmware

1. Install ESP32 board support via Boards Manager
2. Open `HandbrakeController_v3.ino`
3. Select **ESP32 Dev Module**
4. Upload!

### 3. Connect & Control

1. Connect phone to **ParkingBrake** WiFi
2. Password: `brake1234`
3. Open **http://192.168.4.1**

---

## LED Status

| Pattern | Meaning |
|---------|---------|
| 3 blinks | System ready |
| Fast blink | Actuator moving |
| Solid ON | Brake applied |
| OFF | Brake released |

---

## Error Codes

| Code | Description | Action |
|------|-------------|--------|
| E01 | Low voltage (<11V) | Check battery |
| E02 | High voltage (>16V) | Check charging |
| E03 | Overcurrent (>25A) | Check for binding |
| E04 | Stall detected | Check actuator |
| E05 | Timeout | Adjust timing |
| E06 | Memory corruption | Auto-reboots |
| E07 | WiFi failed | Auto-restarts |
| E08 | NVS error | Settings reset |

---

## Configuration

### Customize WiFi

```cpp
const char* AP_SSID     = "ParkingBrake";  // Your name here
const char* AP_PASSWORD = "brake1234";      // Min 8 characters
```

### Timing Parameters

```cpp
int apply_ms   = 2500;   // Apply time (ms)
int release_ms = 1800;   // Release time (ms)
int duty       = 220;    // PWM power (0-255)
```

---

## Project Structure

```
HandbreakDelete/
  HandbrakeController/
    HandbrakeController.ino      # Cloud-connected version
    HandbrakeController_v3.ino   # Standalone with mobile UI
  server/
    server.js                     # Optional telemetry server
    package.json
  ElectricParkingBrake_Manual.html  # Wiring manual
  README.md
  LICENSE
```

---

## Safety

> **This is a safety-critical system.** Modifying brake systems can be dangerous.

- Always bench test before vehicle installation
- Fuse the 12V feed close to the battery
- Use 14-16 AWG wire for motor power
- Keep motor wires away from signal wires
- Test with vehicle secured
- Keep a mechanical backup release method

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| ESP32 resets during operation | Better power supply, add capacitors |
| Actuator wrong direction | Swap M+ and M- wires |
| Won't release | Check brake pedal input (active LOW) |
| Can't connect to WiFi | Check Serial Monitor for AP status |
| Random behavior | Route signal wires away from motor wires |

---

## Documentation

For detailed wiring diagrams and instructions, see:

[ElectricParkingBrake_Manual.html](ElectricParkingBrake_Manual.html) - Printable wiring manual

---

## License

MIT License - See [LICENSE](LICENSE) file

---

<p align="center">
  Made with for DIY automotive projects
</p>
