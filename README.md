# ElectricBrakeConversion

Electric parking brake controller for a 12V linear actuator (e.g., 350lb, 50mm stroke) using an **ESP32** and a **BTS7960** motor driver.

## Connect to Your Brake

1. Connect your phone to WiFi: **ParkingBrake**
   Password: `brake1234`

2. Open browser: **http://192.168.4.1**

3. Control your brake with the beautiful mobile UI!

**No internet required!** The ESP32 creates its own WiFi network - works completely standalone.

---

## Features

### RUGGED Edition (v3.1) - Automotive Grade!
- **Hardware watchdog:** Auto-reboot if firmware freezes
- **Voltage spike filtering:** Median filter rejects transients
- **EMI-resistant inputs:** Multi-sample validation with timing spread
- **Auto-retry:** Transient failures retry automatically
- **Self-healing WiFi:** AP restarts if it fails
- **Memory corruption detection:** Canary values monitored
- **Overvoltage protection:** Won't operate above 16V
- **Boot counter:** Track reboots for reliability analysis

### Standalone Features (v3.0)
- **WiFi Access Point mode:** Connect directly like a router - no internet needed!
- **Beautiful iPhone-optimized UI:** Control from your phone's browser
- **Real-time WebSocket updates:** Instant feedback, no page refresh
- **Works offline:** Perfect for vehicle installation
- **Touch-optimized controls:** Big button, easy to use

### Core Features
- **Auto apply:** when Park input becomes active
- **Safe release:** requires brake pedal input to release when leaving Park
- **Manual override:** hold a momentary button ~3 seconds to toggle
- **State persistence:** remembers brake position across power cycles

### Professional Features
- **Current sensing:** Detects when actuator hits limit switches (stall detection)
- **Soft start/stop:** PWM ramping reduces mechanical stress
- **Low/high voltage protection:** Won't operate outside 11-16V range
- **Detailed diagnostics:** Error codes, retry counts, statistics

> **Safety-critical project:** bench test first, fuse the 12V feed, and test with the vehicle secured.

---

## Project Files

| File | Description |
|------|-------------|
| `HandbrakeController_v3.ino` | **Recommended!** Standalone with mobile UI |
| `HandbrakeController.ino` | Cloud-connected version |
| `server/server.js` | Optional telemetry server |
| `ElectricParkingBrake_Manual.html` | Wiring manual |

---

## Hardware

### Required
- ESP32 DevKit (Arduino-compatible)
- BTS7960 motor driver
- 12V linear actuator with built-in limit switches
- 12V to 5V buck converter (3A+ recommended)
- Inline fuse + holder (10A)
- Momentary pushbutton (manual override)

### Optional
- **ACS712-30A module** on GPIO33 (current sensing)
- **Voltage divider** (47K + 10K) on GPIO36 (battery monitoring)

---

## Wiring

### ESP32 to BTS7960

| ESP32 | BTS7960 | Function |
|-------|---------|----------|
| 25 | RPWM | Extend/Apply |
| 26 | LPWM | Retract/Release |
| 27 | R_EN | Enable |
| 14 | L_EN | Enable |
| GND | GND | Ground |

### Inputs (active LOW - connect to ground when active)

| ESP32 | Signal |
|-------|--------|
| 34 | Park signal |
| 35 | Brake pedal |
| 32 | Manual button |
| 2 | Status LED |

---

## Upload to ESP32

### 1. Install Libraries

In Arduino IDE: **Sketch > Include Library > Manage Libraries**

| Library | Author |
|---------|--------|
| WebSockets | Markus Sattler |
| ArduinoJson | Benoit Blanchon |

### 2. Upload

1. Install **ESP32 board support** (Boards Manager)
2. Open `HandbrakeController_v3.ino`
3. Board: **ESP32 Dev Module**
4. Select COM port
5. Upload!

### 3. Connect

1. Open Serial Monitor (115200 baud)
2. Connect phone to WiFi: **ParkingBrake** (password: `brake1234`)
3. Open **http://192.168.4.1**

### Customize WiFi Name

Edit these lines in the code:
```cpp
const char* AP_SSID     = "ParkingBrake";    // Change this
const char* AP_PASSWORD = "brake1234";        // Min 8 chars
```

---

## LED Patterns

| Pattern | Meaning |
|---------|---------|
| OFF | Brake released |
| Solid ON | Brake applied |
| Fast blink | Actuator moving |
| 3 blinks at boot | System ready |

---

## Tuning

Adjust these values in the code:
```cpp
int apply_ms   = 2500;   // Apply travel time (ms)
int release_ms = 1800;   // Release travel time (ms)
int duty       = 220;    // PWM power (0-255)
```

---

## Safety Notes

- Fuse the 12V feed close to the battery
- Use 14-16 AWG wire for motor power
- Keep motor wires away from signal wires
- Test on bench before vehicle installation

---

## License

MIT - See [LICENSE](LICENSE) file
