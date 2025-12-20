# ElectricBreakConversion

Electric parking brake controller for a 12V linear actuator (e.g., 350lb, 50mm stroke) using an **ESP32** and a **BTS7960** motor driver.

- **Auto apply:** when Park input becomes active.
- **Safe release:** requires brake pedal input to release when leaving Park.
- **Manual override:** hold a momentary button ~3 seconds to toggle.
- **Reliability-focused:** watchdog, debounced inputs, multi-sample validation, state persistence, LED status, actuation timeout.

> Safety-critical project: bench test first, fuse the 12V feed, and test with the vehicle secured.

## Project files

- Sketch: [HandbrakeController/HandbrakeController.ino](HandbrakeController/HandbrakeController.ino)
- Wiring/installation manual (print to PDF): [ElectricParkingBrake_Manual.html](ElectricParkingBrake_Manual.html)

## Hardware

- ESP32 DevKit (Arduino-compatible)
- BTS7960 motor driver (RPWM/LPWM + R_EN/L_EN)
- 12V linear actuator with built-in limit switches
- 12V → 5V buck converter (3A+ recommended)
- Inline fuse + holder (start at 10A; adjust for your actuator)
- Momentary pushbutton (manual override)

## Wiring (pin map)

All inputs are configured as `INPUT_PULLUP`:
- **Inactive = HIGH** (internal pull-up)
- **Active = LOW** (switch to ground)

### ESP32 → BTS7960

| ESP32 GPIO | BTS7960 Pin | Meaning |
|---:|---|---|
| 25 | RPWM | Extend / Apply |
| 26 | LPWM | Retract / Release |
| 27 | R_EN | Enable |
| 14 | L_EN | Enable |
| 5V/Vin | VCC | Logic power (verify your module) |
| GND | GND | Common ground |

### Inputs (active LOW)

| ESP32 GPIO | Signal | How |
|---:|---|---|
| 34 | Park | Ground when in Park |
| 35 | Brake pedal | Ground when brake pressed |
| 32 | Manual override button | Momentary to GND (hold ~3s) |

### Status LED

| ESP32 GPIO | Purpose |
|---:|---|
| 2 | Status LED (built-in on many ESP32 boards) |

LED behavior:
- 3 quick blinks at boot = firmware running
- blinking = actuator moving
- solid ON = brake applied
- OFF = brake released

## Firmware behavior

- **Shift into Park:** brake applies.
- **Shift out of Park:** brake releases **only** if brake pedal input is active.
- **Manual override:** hold the button ~3 seconds to toggle apply/release.

## Tuning

Open [HandbrakeController/HandbrakeController.ino](HandbrakeController/HandbrakeController.ino) and adjust:

- `apply_ms` — apply travel time
- `release_ms` — release travel time
- `duty` — PWM power (0–255)
- `HOLD_TIME_MS` — override hold time

Tip: because your actuator has built-in limit switches, it won’t run past end-of-travel, but keeping timings close reduces heat.

## Upload (Arduino IDE)

1. Install ESP32 board support in Arduino IDE (Boards Manager).
2. Open [HandbrakeController/HandbrakeController.ino](HandbrakeController/HandbrakeController.ino)
3. Board: **ESP32 Dev Module** (or your exact ESP32 model)
4. Select the correct COM port
5. Upload
6. Serial Monitor: **115200**

## Print the manual to PDF

Open [ElectricParkingBrake_Manual.html](ElectricParkingBrake_Manual.html) in a browser → `Ctrl+P` → **Save as PDF**.

## Safety notes

- Fuse the 12V feed close to the source.
- Use appropriate wire gauge for motor power (often 14–16AWG).
- Keep motor leads away from signal wires.
- If the ESP32 resets when the actuator starts: upgrade the buck converter, improve grounding, and add capacitance on 5V.

## Contributing

PRs welcome (especially around noise hardening, enclosure guidance, and vehicle-safe wiring practices).
