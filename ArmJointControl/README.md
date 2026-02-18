# Arm Joint Control System (ESP32-S3)

This project provides high-precision PID control for a 3-joint robotic arm (M3, Pitch, and Roll) using an ESP32-S3. It features multi-turn tracking for the M3 joint, a differential wrist mixer for Pitch/Roll, and a robust ROS binary-packet UART interface with a communication watchdog.

## Project Structure (Refactored)

The codebase follows **SOLID principles**, separating hardware drivers, control logic, and communication layers:

*   **`ArmJointControl.ino`**: Main entry point; orchestrates the 50Hz control loop.
*   **`Config.h` / `Globals.cpp`**: Centralized configuration for pins, PID gains, and calibration.
*   **`Joint.h` / `Joint.cpp`**: Encapsulates state, PID, trajectory, and safety logic for each joint.
*   **`Hardware.h` / `Hardware.cpp`**: Low-level drivers for I2C MUX (TCA9548A), Encoders (AS5600), Motors, and Status LED.
*   **`Comms.h` / `Comms.cpp`**: ROS UART protocol handling and USB Serial Command Interface.
*   **`Types.h`**: Shared data structures for ROS communication.
*   **`Utils.h` / `Utils.cpp`**: Math utilities (wrapping, deadzones) and coordinate conversion logic.

## Hardware Configuration

### Pinout (ESP32-S3)
| Component | Function | Pin |
| :--- | :--- | :--- |
| **I2C** | SDA | GPIO 2 |
| **I2C** | SCL | GPIO 1 |
| **ROS UART** | RX | GPIO 8 |
| **ROS UART** | TX | GPIO 9 |
| **Status LED** | WS2812B | GPIO 21 |
| **Motor 0 (M3)** | PWM / DIR | GPIO 13 / 12 |
| **Motor 1 (Wrist A)**| PWM / DIR | GPIO 4 / 3 |
| **Motor 2 (Wrist B)**| PWM / DIR | GPIO 6 / 7 |

### Sensors
*   **Encoders**: AS5600 magnetic encoders via TCA9548A I2C Multiplexer.
*   **Mux Channels**: M3 (Ch 5), Pitch (Ch 0), Roll (Ch 1).

---

## Operating Instructions

### 1. Startup Behavior
1.  **Booting**: Status LED turns **MAGENTA**.
2.  **Safety**: Motors remain **OFF** until either a valid ROS packet is received or the `R` (Reset/Enable) command is sent via USB Serial.
3.  **M3 Assumption**: On startup, M3 assumes it is within its home half-turn (-90° to +90° shaft). If it starts outside this, the continuous position tracking will be offset by 360°.

### 2. Status LED Indicators
*   **MAGENTA**: Booting / No ROS Connection / Watchdog Expired.
*   **GREEN**: ROS Connected / All motors holding position.
*   **BLUE**: ROS Connected / Actively driving at least one motor.

### 3. USB Serial Interface (115200 Baud)
Send characters or numbers to the ESP32 via the Serial Monitor:

| Command | Description |
| :--- | :--- |
| **`R`** | **Reset/Enable**: Re-enables all joints and sets targets to current positions. |
| **`S`** | **E-Stop**: Immediately stops all motors and disables PID. |
| **`0 [deg]`** | Move M3 shaft to absolute degree (-180 to 180). |
| **`1 [deg]`** | Move Pitch to absolute degree. |
| **`2 [deg]`** | Move Roll to absolute degree. |
| **`W [p] [r]`** | Move Wrist (Pitch/Roll) simultaneously. |
| **`?`** | Print current status, errors, and motor states. |
| **`P/I/D [id] [val]`**| Tune PID gains (id: 0=M3, 1=Pitch, 2=Roll). |
| **`E`** | Print raw encoder values for debugging. |

### 4. ROS Integration
The system listens for binary packets on UART1.
*   **Watchdog**: If no valid packet is received for **500ms**, the motors stop and the LED turns magenta.
*   **Feedback**: For every valid command received, the ESP32 immediately returns a `FeedbackPacketB` containing current joint positions in encoder steps.

---

## Calibration & Tuning

1.  **Home Offsets**: Adjust `HOME_OFFSETS` in `Config.h` to align your physical zero position with the encoder's zero.
2.  **Direction Signs**: Use the `F` command (via original code logic, or modify `Globals.cpp`) to flip motor or PID signs if a joint moves away from its target.
3.  **PID Gains**: Initial gains are set in `Globals.cpp`. Use the USB interface to live-tune for your specific mechanical load.

## Safety Features
*   **I2C Watchdog**: Automatically resets the I2C bus if the multiplexer or encoders hang.
*   **Sensor Fail Protection**: If an encoder fails to read for 10 consecutive cycles, all motors are disabled to prevent runaway.
*   **Soft Limits**: M3 features a "Soft Zone" that reduces motor power as it approaches its continuous travel limits (±4096 steps).
*   **Pitch Hard Stop**: Prevents the pitch joint from rotating into mechanically forbidden zones.
