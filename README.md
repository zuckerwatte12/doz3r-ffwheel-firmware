# Steering PAD 900-F — Custom Firmware Notes

## Video Demo

[![Drifting in Assetto Corsa with DIY Force Feedback Wheel](https://img.youtube.com/vi/a-UPCigjC2w/maxresdefault.jpg)](https://www.youtube.com/shorts/a-UPCigjC2w)

**Watch the wheel in action:** This video demonstrates the force feedback system working in real-time during drift maneuvers in Assetto Corsa. Notice how the steering wheel returns by itself as the car countersteers through the drift — this is not spring-centering, but **actual force feedback controlled by Assetto Corsa's physics engine**. The game calculates tire forces and sends them directly to the motor, creating realistic self-aligning torque just like a real car.

---

## Overview

This is a modified version of the Steering PAD 900-F v3.2 firmware by HomeGameCoder, adapted for a specific hardware unit with non-standard wiring. The original project is available at:
- Printables: https://www.printables.com/model/1223875-steering-pad-900-f
- YouTube: @HomeGameCoderBuilds

### Assetto Corsa Direct Drive Force Feedback

**This firmware is specifically designed to work with Assetto Corsa (and other racing simulators) with direct force feedback from the game to the motor.** The device operates in **direct drive mode** where the motor receives force feedback commands directly from the game via USB HID protocol.

**How it works:**
- The Arduino Pro Micro identifies as a **Logitech G29** (or optionally Thrustmaster T300RS in alt mode) over USB
- Games like Assetto Corsa send FFB commands through the standard USB HID interface
- The firmware reads these force commands using `Joystick.getForce()` from the ArduinoJoystickWithFFBLibrary
- Forces are applied to the motor in **real-time** with configurable gain, deadzone, and damping
- The motor responds to every bump, kerb, oversteer, and road surface change sent by the game
- **900° rotation range** with hard software limits to prevent over-rotation
- Low latency response ensures realistic feel for racing simulation

**Key FFB features:**
- Pure direct drive mode (no artificial centering spring)
- Adjustable gain (0.1–20.0x multiplier)
- Configurable deadzone (0–50) to prevent motor drift from small forces
- Optional damping (0–30) to reduce free-spinning
- Force inversion toggle for correct directional feel
- Max torque limiter (50–250) to protect motor and prevent excessive forces

This makes the wheel suitable for competitive sim racing in Assetto Corsa, providing direct feedback from the physics engine to your hands.

---

## Hardware Configuration

### Microcontroller
- **Board:** Arduino Pro Micro / Arduino Micro (ATmega32U4, 5V, 16MHz)
- **USB:** Identifies as Logitech device (VID `0x046d`) for broad game compatibility

### Steering Sensor
- **Type:** Hall effect sensors (49E) read via ADS1115 ADC over I2C (AceWire fast interface)
- **I2C address:** `0x48`
- **ADS1115 channel mapping (this unit):**
  - Channel 0 → Accelerator pedal
  - Channel 1 → Steering wheel
  - Channel 2 → Brake pedal
- **Note:** This differs from the reference firmware which maps steering to channel 0. The wiring on this unit routes sensors differently.

### Button Matrix
Buttons are read via analog multiplexing on pins A0–A3. The pin order for this unit is:

```
sensorValues[3] = analogRead(A0);
sensorValues[1] = analogRead(A1);
sensorValues[2] = analogRead(A2);
sensorValues[0] = analogRead(A3);
```

### Button Thresholds (hardware-specific)
These values were measured directly from the hardware and must match the resistor ladder on your specific PCB:

```cpp
238, // button T (single)
503, // button N (single)
380, // button Wiper (single)
195, // T + N
175, // T + Wiper
278, // N + Wiper
152  // T + N + Wiper
```

### Motor
- **Driver:** DRV8838
- **Pins:** Direction=8, PWM=9, Enable=10
- **Power source:** USB only (5V)

---

## Firmware Changes vs Original V3.2

### 1. ADS1115 uses AceWire instead of Wire
The original firmware uses the standard Wire library. This build uses `AceWire` with `SimpleWireFastInterface` for faster I2C communication, which resolved steering sensor reading issues on this hardware.

### 2. Channel remapping
```cpp
steeringSensor    = ADS.readADC(1);  // was channel 0 in original
brakeSensor       = ADS.readADC(2);  // was channel 1 in original
acceleratorSensor = ADS.readADC(0);  // was channel 2 in original
```

### 3. Analog pin order remapped
A0 reads as sensorValues[3] instead of sensorValues[0] due to different PCB wiring.

### 4. Menu controls remapped
The T button (threshold 238) does not register reliably on this unit due to a hardware/firmware incompatibility. Menu is controlled via third row buttons instead:

| Physical button | Function |
|---|---|
| Third row LEFT (hold 1s) | Open menu |
| Third row LEFT (short press) | Next option |
| Third row MIDDLE | Exit / No |
| Third row RIGHT | Set / Yes / Change |

---

## Menu Navigation

1. Hold the **third row left button** for 1 second to open the menu
2. Short press the **same button** to cycle through options
3. Press **third row right** to change/set a value
4. Press **third row middle** to exit or discard

### Menu Options
1. Force Feedback — Active (Yes/No)
2. Force Feedback — Gain (0–200, use steering wheel to set value)
3. Brake pedal — Linearity curve (Ease In / Linear / Ease Out)
4. Accelerator pedal — Linearity curve
5. Steering wheel — Calibration (11-point, 0°–900°)
6. Brake pedal — Min/Max calibration
7. Accelerator pedal — Min/Max calibration
8. Show sensor data on screen (Yes/No)

---

## Calibration Procedure

### Steering Wheel (required after first flash)
1. Open menu → navigate to option 5 (Steering wheel)
2. Press SET → rotate to full left → press SET
3. Follow prompts for each 90° increment up to 900°
4. Save when complete

### Pedals
1. Open menu → option 6 (Brake) or 7 (Accelerator)
2. Press slightly (1mm) → press SET for minimum
3. Press fully then release 1mm → press SET for maximum
4. Save

---

## First Flash Instructions

1. Set `isFirstTimeUploading` to `true`
2. Flash the firmware (double-tap reset on Pro Micro to enter bootloader)
3. Set `isFirstTimeUploading` back to `false`
4. Flash again
5. Calibrate steering and pedals via menu

**Important:** The firmware folder name must match the `.ino` filename exactly.

---

## Libraries Required

- `AceWire` — https://github.com/bxparks/AceWire
- `DigitalWriteFast`
- `ADS1X15` (AceWire-compatible version)
- `SSD1306Ascii` (AceWire-compatible version)
- `ArduinoJoystickWithFFBLibrary`

---

## Known Issues

- T button (threshold 238) does not trigger menu on this unit — remapped to third row left button as workaround
- Firmware is at ~99% program storage — no additional debug code can be added without removing existing features
- FFB shows 0 until steering calibration is completed

---

## Detailed Build Information

### 🔧 Hardware

Based on the Steering PAD 900-F (https://www.printables.com/model/1223875-steering-pad-900-f) with the following setup:
- Arduino Pro Micro (ATmega32U4, 5V 16MHz)
- RF-300CA motor with DRV8838 motor driver for force feedback
- ADS1115 16-bit ADC for steering sensor reading via Hall effect sensors (49E)
- SSD1306 OLED display for on-device menu and calibration
- USB powered — no external power supply needed

### 💾 Firmware Improvements

Custom firmware fork with the following improvements over the original V3.2:
- **AceWire fast I2C library** replaces the standard Wire library for more reliable ADS1115 steering sensor communication
- **Remapped ADS1115 channels** to match the actual PCB wiring of this unit
- **Corrected analog pin order** for the button matrix
- **Alternative menu controls** for compatibility with this hardware variant
- **USB device spoofing** — registers as a Logitech steering wheel (VID 0x046d) for maximum game compatibility, including games that only detect known brands
- **Full DirectInput force feedback support** via ArduinoJoystickWithFFBLibrary
- **Spring, damper, inertia and friction effects** with tunable parameters
- **11-point steering linearity correction LUT** for accurate 900° range
- **Pedal linearity curves:** Linear, Ease In, Ease Out per pedal independently
- **On-device calibration menu** for steering wheel and both pedals — no PC software needed

### 🎮 Game Compatibility

Tested in **Assetto Corsa** running on Parallels (Windows 11 ARM) on Apple MacBook M4. Force feedback works natively through DirectInput. The wheel spoofs as a Logitech device which improves detection in games that whitelist specific controllers.

---

## Credits

- Original firmware: Rui Caldas (HomeGameCoder) — GNU GPL v3.0
- AceWire integration and hardware adaptation: this fork
