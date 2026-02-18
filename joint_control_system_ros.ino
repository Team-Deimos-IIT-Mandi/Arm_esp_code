#include <Wire.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// =================================================================
//  JOINT CONTROL SYSTEM + ROS UART INTEGRATION
//
//  Core logic, limits, PID, safety, and serial UI from
//  joint_control_system.ino — fully preserved.
//
//  ROS binary-packet UART layer, watchdog, and status LED from
//  ros_motor_control.ino — layered on top.
//
//  Behaviour:
//    • When ROS packets arrive on UART1, they set joint targets and
//      the watchdog stays satisfied.  Feedback packets are returned.
//    • When ROS is silent (watchdog expired), motors stop and the
//      LED turns magenta.  Local USB-serial commands still work for
//      diagnostics / manual override.
//    • USB serial commands (H, T, E, R, S, …) are ALWAYS available.
//      Sending a target via USB while ROS is active will be
//      overwritten on the next ROS packet — this is intentional.
// =================================================================

// ===================== ROS COMMUNICATION STRUCTS =================
// MUST match esp_coms.hpp on the RPi exactly — same size, same order.
struct __attribute__((packed)) CommandPacketB {
    char    header[2];            // 'S', 'T'
    int32_t motor_cmd[3];         // encoder steps: [M3, wrist, gripper]
    char    footer;               // '\n'
};                                // Total: 15 bytes

struct __attribute__((packed)) FeedbackPacketB {
    char    header[2]      = {'F', 'B'};
    int32_t motor_pos[3]   = {0, 0, 0}; // encoder steps
    char    footer         = '\n';
};                                // Total: 15 bytes

// Explicit prototype for the Arduino pre-processor
bool tryReadPacket(CommandPacketB& cmd);

// ===================== ROS UART CONFIG ===========================
const int ROS_RX_PIN = 8;
const int ROS_TX_PIN = 9;
HardwareSerial ROSSerial(1);

// ===================== STATUS LED ================================
// Waveshare ESP32-S3 Zero onboard WS2812B on GPIO 21
//
//   MAGENTA — booting / no ROS connection / watchdog expired
//   GREEN   — ROS connected, all motors holding position
//   BLUE    — ROS connected, actively driving at least one motor
//
#define LED_PIN   21
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_MS = 100;

void setLed(uint32_t colour) {
    led.setPixelColor(0, colour);
    led.show();
}

// ===================== WATCHDOG ==================================
const unsigned long COMMS_TIMEOUT_MS = 500;
unsigned long last_valid_cmd_time    = 0;
bool comms_ok                        = false;

// ================= USER CALIBRATION SECTION ======================

const bool JOINT_ALLOWED[3] = {true, true, true};

int HOME_OFFSETS[3] = {430, 1883, 0};

const int POS_MIN[3] = {0, 0, 0};
const int POS_MAX[3] = {4096, 1837, 4096};

const int PITCH_LIMIT_MIN = 0;
const int PITCH_LIMIT_MAX = 1837;

// ================= MOTOR 3 CONFIG ================================
const float M3_GEAR_RATIO  = 2.0f;
const float M3_MOTOR_LIMIT = 360.0f;
const float M3_OUTPUT_LIMIT = 180.0f;

// ================= ANGLE CONVERSION ==============================
const float PITCH_RANGE_DEG = (1837.0f / 4096.0f) * 360.0f;
const float ROLL_RANGE_DEG  = 360.0f;

float motorDegToSteps(float motor_deg) {
    return (motor_deg / 360.0f) * 4096.0f;
}
float motorStepsToDeg(float steps) {
    return (steps / 4096.0f) * 360.0f;
}
float outputToMotorDeg(float output_deg) {
    return output_deg * M3_GEAR_RATIO;
}
float motorToOutputDeg(float motor_deg) {
    return motor_deg / M3_GEAR_RATIO;
}
float pitchStepsToDeg(float steps) {
    return (steps / (float)PITCH_LIMIT_MAX) * PITCH_RANGE_DEG;
}
float pitchDegToSteps(float deg) {
    return (deg / PITCH_RANGE_DEG) * (float)PITCH_LIMIT_MAX;
}
float rollStepsToDeg(float steps) {
    return (steps / 4096.0f) * ROLL_RANGE_DEG;
}
float rollDegToSteps(float deg) {
    return (deg / ROLL_RANGE_DEG) * 4096.0f;
}

// ================= HARDWARE CONFIGURATION ========================
#define I2C_MUX_ADDR   0x70
#define AS5600_ADDR    0x36
#define ENCODER_COUNTS 4096
#define ENCODER_HALF   2048

const int SDA_PIN = 2;
const int SCL_PIN = 1;
const int PWM_PINS[3]     = {13, 4, 6};
const int DIR_PINS[3]     = {12, 3, 7};
const int MUX_CHANNELS[3] = {5, 0, 1};

const int LEDC_FREQ       = 10000;
const int LEDC_RES        = 8;
const uint32_t I2C_FREQ   = 400000;

bool USE_ROLL_ENCODER = true;

// PID Gains
float Kp[3] = {4.0f,  4.0f,  4.0f};
float Ki[3] = {0.05f, 0.02f, 0.02f};
float Kd[3] = {0.0f,  0.15f, 0.15f};

const float PID_OUTPUT_MAX = 255.0f;
const float INTEGRAL_MAX   = 100.0f;
const float ERROR_DEADZONE = 5.0f;
const int   MAX_SENSOR_FAILS = 10;
const float MAX_SLEW_RATE  = 40.0f;

// ================= DIRECTION SIGNS ===============================
int       M3_PID_SIGN         = 1;
const int MOTOR_DIR_SIGN[3]   = {1, 1, 1};
int       MOTOR_A_ROLL_SIGN   = 1;
int       ROLL_PID_SIGN       = 1;

// ================= STATE VARIABLES ===============================
bool  motor_enabled[3]     = {false};
float target_pos[3]        = {0};
float commanded_pos[3]     = {0};
float current_pos[3]       = {0};
float prev_pos[3]          = {0};
float integral[3]          = {0};
float prev_error[3]        = {0};
float pid_output[3]        = {0};
int   sensor_fail_count[3] = {0};
bool  sensor_read_ok[3]    = {false};

// ===== M3 MULTI-TURN TRACKING ===================================
int   m3_wrap_count        = 0;
int   m3_prev_raw          = 0;
float m3_continuous_pos    = 0;
float m3_continuous_target = 0;
float m3_continuous_cmd    = 0;
float m3_continuous_prev   = 0;

const float M3_CONT_LIMIT     = 4096.0f;
const float M3_CONT_SOFT_ZONE = 400.0f;

unsigned long last_pid_time   = 0;
unsigned long last_debug_time = 0;
bool manual_test_mode         = false;

bool debug_show[3] = {true, true, true};

// =================================================================
//                       CORE FUNCTIONS
//  (all preserved verbatim from joint_control_system.ino)
// =================================================================

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(I2C_MUX_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delayMicroseconds(5);
}

float wrapValue(float val) {
    while (val < 0) val += ENCODER_COUNTS;
    while (val >= ENCODER_COUNTS) val -= ENCODER_COUNTS;
    return val;
}

float wrapError(float error) {
    if (error > ENCODER_HALF) error -= ENCODER_COUNTS;
    if (error < -ENCODER_HALF) error += ENCODER_COUNTS;
    return error;
}

float applyDeadzone(float error, float deadzone) {
    if (fabsf(error) < deadzone) return 0.0f;
    return (error > 0) ? (error - deadzone) : (error + deadzone);
}

int readEncoderRaw(int joint) {
    tcaSelect(MUX_CHANNELS[joint]);
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte  = Wire.read();
        return (highByte << 8) | lowByte;
    }
    return -1;
}

int readEncoder(int joint) {
    int raw = readEncoderRaw(joint);
    if (raw == -1) return -1;
    int corrected = raw - HOME_OFFSETS[joint];
    while (corrected < 0)    corrected += 4096;
    while (corrected >= 4096) corrected -= 4096;
    return corrected;
}

bool m3UpdatePosition() {
    int raw = readEncoder(0);
    if (raw == -1) return false;

    int delta = raw - m3_prev_raw;
    if (delta >  ENCODER_HALF) m3_wrap_count--;
    else if (delta < -ENCODER_HALF) m3_wrap_count++;

    m3_prev_raw       = raw;
    m3_continuous_pos  = (float)(m3_wrap_count * ENCODER_COUNTS) + (float)raw;
    current_pos[0]    = (float)raw;
    return true;
}

float m3OutputDegToContSteps(float output_deg) {
    output_deg = constrain(output_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
    float motor_deg = outputToMotorDeg(output_deg);
    return motorDegToSteps(motor_deg);
}

float m3ContStepsToOutputDeg(float cont_steps) {
    float motor_deg = motorStepsToDeg(cont_steps);
    return motorToOutputDeg(motor_deg);
}

float jointStepsToDeg(int joint, float steps) {
    if (joint == 0) return m3ContStepsToOutputDeg(m3_continuous_pos);
    if (joint == 1) return pitchStepsToDeg(steps);
    return rollStepsToDeg(steps);
}

const char* jointName(int joint) {
    if (joint == 0) return "M3";
    if (joint == 1) return "Pitch";
    return "Roll";
}

void i2cReset() {
    Wire.end();
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    }
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);
    Serial.println("I2C RESET");
}

bool i2cHealthCheck() {
    Wire.beginTransmission(I2C_MUX_ADDR);
    return (Wire.endTransmission() == 0);
}

void setMotor(int joint, int pwm_val) {
    if (!motor_enabled[joint]) {
        ledcWrite(PWM_PINS[joint], 0);
        return;
    }
    pwm_val *= MOTOR_DIR_SIGN[joint];
    int speed = abs(pwm_val);
    speed = constrain(speed, 0, 255);
    if (speed < 15) speed = 0;
    digitalWrite(DIR_PINS[joint], (pwm_val >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[joint], speed);
}

void setMotorM3(int pwm_val) {
    if (!motor_enabled[0]) {
        ledcWrite(PWM_PINS[0], 0);
        return;
    }
    pwm_val *= M3_PID_SIGN * MOTOR_DIR_SIGN[0];
    int speed = abs(pwm_val);
    speed = constrain(speed, 0, 255);
    if (speed < 15) speed = 0;
    digitalWrite(DIR_PINS[0], (pwm_val >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[0], speed);
}

void driveMotorRaw(int pin_index, int pwm_signed) {
    int speed = abs(pwm_signed);
    speed = constrain(speed, 0, 255);
    digitalWrite(DIR_PINS[pin_index], (pwm_signed >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[pin_index], speed);
}

void stopAllMotorsRaw() {
    for (int i = 0; i < 3; i++) {
        ledcWrite(PWM_PINS[i], 0);
    }
}

void emergencyStopAll() {
    manual_test_mode = false;
    for (int i = 0; i < 3; i++) {
        motor_enabled[i] = false;
        ledcWrite(PWM_PINS[i], 0);
        integral[i] = 0;
    }
    Serial.println("!!! E-STOP !!!");
}

// =================================================================
//                      SENSOR READING
// =================================================================

void readAllSensors() {
    static int i2c_fail_count = 0;
    if (!i2cHealthCheck()) {
        if (++i2c_fail_count > 3) { i2cReset(); i2c_fail_count = 0; }
        return;
    }
    i2c_fail_count = 0;

    // M3: use wrap-tracking reader
    sensor_read_ok[0] = false;
    if (motor_enabled[0]) {
        if (m3UpdatePosition()) {
            sensor_fail_count[0] = 0;
            sensor_read_ok[0]    = true;
        } else {
            if (++sensor_fail_count[0] > MAX_SENSOR_FAILS) emergencyStopAll();
        }
    }

    // Pitch & Roll: normal reading
    for (int i = 1; i < 3; i++) {
        sensor_read_ok[i] = false;
        if (!motor_enabled[i]) continue;
        if (i == 2 && !USE_ROLL_ENCODER) {
            sensor_read_ok[i] = true;
            continue;
        }
        int raw = readEncoder(i);
        if (raw != -1) {
            current_pos[i]       = raw;
            sensor_fail_count[i] = 0;
            sensor_read_ok[i]    = true;
        } else {
            if (++sensor_fail_count[i] > MAX_SENSOR_FAILS) emergencyStopAll();
        }
    }
}

// =================================================================
//                    TRAJECTORY & PID
//  (full original logic with deadzone, soft limits, pitch safety)
// =================================================================

void updateTrajectory() {
    // === M3: linear trajectory in continuous space ===
    if (motor_enabled[0]) {
        float error_to_target = m3_continuous_target - m3_continuous_cmd;
        if (fabsf(error_to_target) > MAX_SLEW_RATE) {
            m3_continuous_cmd += (error_to_target > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
        } else {
            m3_continuous_cmd = m3_continuous_target;
        }
        m3_continuous_cmd = constrain(m3_continuous_cmd, -M3_CONT_LIMIT, M3_CONT_LIMIT);
    } else {
        m3_continuous_cmd = m3_continuous_pos;
    }

    // === Pitch & Roll: circular ===
    for (int i = 1; i < 3; i++) {
        if (!motor_enabled[i]) {
            commanded_pos[i] = current_pos[i];
            continue;
        }
        float error_to_target = target_pos[i] - commanded_pos[i];
        error_to_target = wrapError(error_to_target);
        if (fabsf(error_to_target) > MAX_SLEW_RATE) {
            commanded_pos[i] += (error_to_target > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
            commanded_pos[i] = wrapValue(commanded_pos[i]);
        } else {
            commanded_pos[i] = target_pos[i];
        }
    }
}

void computePID(float dt) {
    // === M3: linear PID in continuous space ===
    if (motor_enabled[0]) {
        float error    = m3_continuous_cmd - m3_continuous_pos;
        float error_dz = applyDeadzone(error, ERROR_DEADZONE);

        float p_term = Kp[0] * error_dz;

        if (fabsf(pid_output[0]) < PID_OUTPUT_MAX) {
            integral[0] += (error_dz * dt);
            integral[0]  = constrain(integral[0], -INTEGRAL_MAX, INTEGRAL_MAX);
        }
        if (fabsf(error_dz) < 0.1f) integral[0] = 0;
        float i_term = Ki[0] * integral[0];

        float d_term = 0.0f;
        if (sensor_read_ok[0]) {
            float dInput = m3_continuous_pos - m3_continuous_prev;
            d_term = Kd[0] * (dInput / dt);
            m3_continuous_prev = m3_continuous_pos;
        }

        float output = p_term + i_term - d_term;
        output = constrain(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        pid_output[0] = output;

        // --- M3 SAFETY: hard limits ---
        float pos = m3_continuous_pos;
        if (pos >= M3_CONT_LIMIT && pid_output[0] > 0) {
            pid_output[0] = 0; integral[0] = 0;
        }
        if (pos <= -M3_CONT_LIMIT && pid_output[0] < 0) {
            pid_output[0] = 0; integral[0] = 0;
        }
        // --- M3 SAFETY: soft slowdown approaching +limit ---
        if (pos > (M3_CONT_LIMIT - M3_CONT_SOFT_ZONE) && pos < M3_CONT_LIMIT && pid_output[0] > 0) {
            float scale = (M3_CONT_LIMIT - pos) / M3_CONT_SOFT_ZONE;
            pid_output[0] *= constrain(scale, 0.0f, 1.0f);
        }
        // --- M3 SAFETY: soft slowdown approaching -limit ---
        if (pos < (-M3_CONT_LIMIT + M3_CONT_SOFT_ZONE) && pos > -M3_CONT_LIMIT && pid_output[0] < 0) {
            float scale = (pos + M3_CONT_LIMIT) / M3_CONT_SOFT_ZONE;
            pid_output[0] *= constrain(scale, 0.0f, 1.0f);
        }
    } else {
        pid_output[0] = 0; integral[0] = 0;
    }

    // === Pitch & Roll: circular PID ===
    for (int i = 1; i < 3; i++) {
        if (!motor_enabled[i]) { pid_output[i] = 0; integral[i] = 0; continue; }
        float error    = commanded_pos[i] - current_pos[i];
        error          = wrapError(error);
        float error_dz = applyDeadzone(error, ERROR_DEADZONE);

        float p_term = Kp[i] * error_dz;

        if (fabsf(pid_output[i]) < PID_OUTPUT_MAX) {
            integral[i] += (error_dz * dt);
            integral[i]  = constrain(integral[i], -INTEGRAL_MAX, INTEGRAL_MAX);
        }
        if (fabsf(error_dz) < 0.1f) integral[i] = 0;
        float i_term = Ki[i] * integral[i];

        float d_term = 0.0f;
        if (sensor_read_ok[i]) {
            float dInput = current_pos[i] - prev_pos[i];
            dInput       = wrapError(dInput);
            d_term       = Kd[i] * (dInput / dt);
            prev_pos[i]  = current_pos[i];
        }

        float output = p_term + i_term - d_term;
        output       = constrain(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        pid_output[i] = output;
    }

    // ===== PITCH SAFETY =====
    if (motor_enabled[1]) {
        float pos    = current_pos[1];
        float output = pid_output[1];
        const float WRAP_BUFFER = 200.0f;
        bool in_valid_zone      = (pos <= (float)PITCH_LIMIT_MAX);
        bool in_near_zero_wrap  = (pos >= (4096.0f - WRAP_BUFFER));
        bool in_forbidden_zone  = (!in_valid_zone && !in_near_zero_wrap);

        if (in_forbidden_zone || (pos >= (float)PITCH_LIMIT_MAX && pos <= (float)ENCODER_HALF)) {
            if (output > 0) { pid_output[1] = 0; integral[1] = 0; }
        }
        if (pos <= (float)PITCH_LIMIT_MIN && pos < WRAP_BUFFER) {
            if (output < 0) { pid_output[1] = 0; integral[1] = 0; }
        }
    }
}

// =================================================================
//                    MOTOR OUTPUT (differential wrist)
// =================================================================

void applyMotorOutputs() {
    if (manual_test_mode) return;

    if (motor_enabled[0]) setMotorM3((int)pid_output[0]);
    else ledcWrite(PWM_PINS[0], 0);

    bool pitch_active = motor_enabled[1] && sensor_read_ok[1];
    bool roll_active  = motor_enabled[2] && sensor_read_ok[2] && USE_ROLL_ENCODER;

    if (pitch_active || roll_active) {
        float pitch_cmd = pitch_active ? pid_output[1] : 0.0f;
        float roll_cmd  = roll_active  ? (pid_output[2] * ROLL_PID_SIGN) : 0.0f;

        float mA = pitch_cmd + roll_cmd;
        float mB = -pitch_cmd + roll_cmd;

        float max_val = max(fabsf(mA), fabsf(mB));
        if (max_val > 255.0f) {
            float scale = 255.0f / max_val;
            mA *= scale;
            mB *= scale;
        }

        int pwmA   = (int)mA * MOTOR_DIR_SIGN[1];
        int speedA = constrain(abs(pwmA), 0, 255);
        digitalWrite(DIR_PINS[1], (pwmA >= 0) ? HIGH : LOW);
        ledcWrite(PWM_PINS[1], (speedA < 15) ? 0 : speedA);

        int pwmB   = (int)mB * MOTOR_DIR_SIGN[2];
        int speedB = constrain(abs(pwmB), 0, 255);
        digitalWrite(DIR_PINS[2], (pwmB >= 0) ? HIGH : LOW);
        ledcWrite(PWM_PINS[2], (speedB < 15) ? 0 : speedB);
    } else {
        ledcWrite(PWM_PINS[1], 0);
        ledcWrite(PWM_PINS[2], 0);
    }
}

// =================================================================
//                        M3 INIT
// =================================================================

void m3Init(int encoder_val) {
    m3_prev_raw          = encoder_val;
    m3_wrap_count        = 0;
    m3_continuous_pos    = (float)encoder_val;
    m3_continuous_target = m3_continuous_pos;
    m3_continuous_cmd    = m3_continuous_pos;
    m3_continuous_prev   = m3_continuous_pos;
}

// =================================================================
//            ROS UART — PACKET READER & COMMAND PROCESSING
// =================================================================

bool tryReadPacket(CommandPacketB& cmd) {
    // Drain up to 32 leading garbage bytes to find the 'S' header
    for (int i = 0; i < 32 && ROSSerial.available() > 0; i++) {
        if (ROSSerial.peek() == 'S') break;
        ROSSerial.read();
    }

    // Full packet not yet buffered — return and try again next loop
    if (ROSSerial.available() < (int)sizeof(CommandPacketB)) return false;

    // All bytes confirmed present — readBytes() will not block
    ROSSerial.readBytes((char*)&cmd, sizeof(CommandPacketB));

    if (cmd.header[0] == 'S' && cmd.header[1] == 'T' && cmd.footer == '\n') {
        return true;
    }

    // Footer mismatch — flush entire buffer and re-sync next cycle
    while (ROSSerial.available()) ROSSerial.read();
    Serial.println("ROS: bad packet footer — buffer flushed.");
    return false;
}

void processROSCommand() {
    CommandPacketB cmd;
    if (!tryReadPacket(cmd)) return;

    // Valid packet — reset watchdog
    last_valid_cmd_time = millis();

    // If we were disconnected, re-enable motors on reconnection
    if (!comms_ok) {
        comms_ok = true;
        Serial.println("ROS: connected — motors re-enabled.");
        for (int i = 0; i < 3; i++) {
            if (!JOINT_ALLOWED[i]) continue;
            int val = readEncoder(i);
            if (val != -1) {
                motor_enabled[i]  = true;
                current_pos[i]    = val;
                commanded_pos[i]  = val;
                prev_pos[i]       = val;
                integral[i]       = 0;
                if (i == 0) {
                    m3UpdatePosition();
                    m3_continuous_cmd  = m3_continuous_pos;
                    m3_continuous_prev = m3_continuous_pos;
                }
            }
        }
    }

    // Apply incoming targets — these are in ENCODER STEPS (continuous for M3)
    // The ROS side is responsible for respecting the same coordinate frame.
    // Safety limits in computePID() still clamp everything on this side.
    if (JOINT_ALLOWED[0]) {
        m3_continuous_target = constrain((float)cmd.motor_cmd[0],
                                         -M3_CONT_LIMIT, M3_CONT_LIMIT);
    }
    if (JOINT_ALLOWED[1]) {
        target_pos[1] = constrain((float)cmd.motor_cmd[1],
                                   (float)POS_MIN[1], (float)POS_MAX[1]);
    }
    if (JOINT_ALLOWED[2]) {
        target_pos[2] = wrapValue((float)cmd.motor_cmd[2]);
    }

    // Send feedback packet back to RPi
    FeedbackPacketB fb;
    fb.motor_pos[0] = (int32_t)m3_continuous_pos;
    fb.motor_pos[1] = (int32_t)current_pos[1];
    fb.motor_pos[2] = (int32_t)current_pos[2];
    ROSSerial.write((uint8_t*)&fb, sizeof(FeedbackPacketB));
}

// =================================================================
//                      WATCHDOG CHECK
// =================================================================

void checkCommsWatchdog() {
    if (!comms_ok) return;
    if ((millis() - last_valid_cmd_time) > COMMS_TIMEOUT_MS) {
        comms_ok = false;
        // Stop motors but don't disable the serial command interface
        for (int i = 0; i < 3; i++) {
            motor_enabled[i] = false;
            ledcWrite(PWM_PINS[i], 0);
            integral[i] = 0;
        }
        Serial.println("ROS WATCHDOG: comms lost — motors stopped. USB serial still active.");
    }
}

// =================================================================
//                      STATUS LED UPDATE
// =================================================================

void updateLed() {
    if (millis() - lastLedUpdate < LED_UPDATE_MS) return;
    lastLedUpdate = millis();

    if (!comms_ok) {
        setLed(led.Color(180, 0, 180));   // MAGENTA — no ROS connection
        return;
    }

    bool moving = false;
    for (int i = 0; i < 3; i++) {
        if (motor_enabled[i] && fabsf(pid_output[i]) > 15.0f) {
            moving = true;
            break;
        }
    }
    setLed(moving ? led.Color(0, 0, 255)    // BLUE  — motors driving
                  : led.Color(0, 180, 0));   // GREEN — holding position
}

// =================================================================
//          USB SERIAL COMMAND INTERFACE (fully preserved)
// =================================================================

void processSerialCommand() {
    if (!Serial.available()) return;
    char type = Serial.peek();

    if (type >= '0' && type <= '9') {
        int id = Serial.parseInt();
        float deg = Serial.parseFloat();
        if (id >= 0 && id < 3 && JOINT_ALLOWED[id]) {
            if (id == 0) {
                deg = constrain(deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                m3_continuous_target = m3OutputDegToContSteps(deg);
                m3_continuous_target = constrain(m3_continuous_target, -M3_CONT_LIMIT, M3_CONT_LIMIT);
                Serial.printf("CMD M3: -> %.1f shaft deg (cont step %.0f)\n", deg, m3_continuous_target);
            }
            else if (id == 1) {
                deg = constrain(deg, 0.0f, PITCH_RANGE_DEG);
                target_pos[1] = pitchDegToSteps(deg);
                Serial.printf("CMD Pitch: -> %.1f deg\n", deg);
            }
            else {
                target_pos[2] = wrapValue(rollDegToSteps(deg));
                Serial.printf("CMD Roll: -> %.1f deg\n", deg);
            }
        } else {
            Serial.printf("J%d not allowed\n", id);
        }
    } else {
        char cmd = Serial.read();
        switch (cmd) {
            case 'A': case 'a': {
                float m3_deg = Serial.parseFloat();
                float p_deg  = Serial.parseFloat();
                float r_deg  = Serial.parseFloat();

                if (JOINT_ALLOWED[0]) {
                    m3_deg = constrain(m3_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                    m3_continuous_target = m3OutputDegToContSteps(m3_deg);
                    m3_continuous_target = constrain(m3_continuous_target, -M3_CONT_LIMIT, M3_CONT_LIMIT);
                }
                if (JOINT_ALLOWED[1]) {
                    p_deg = constrain(p_deg, 0.0f, PITCH_RANGE_DEG);
                    target_pos[1] = pitchDegToSteps(p_deg);
                }
                if (JOINT_ALLOWED[2]) {
                    target_pos[2] = wrapValue(rollDegToSteps(r_deg));
                }
                Serial.printf("CMD ALL -> M3:%.1f Pitch:%.1f Roll:%.1f\n", m3_deg, p_deg, r_deg);
                break;
            }

            case 'M': case 'm': {
                float shaft_deg = Serial.parseFloat();
                if (JOINT_ALLOWED[0]) {
                    shaft_deg = constrain(shaft_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                    m3_continuous_target = m3OutputDegToContSteps(shaft_deg);
                    m3_continuous_target = constrain(m3_continuous_target, -M3_CONT_LIMIT, M3_CONT_LIMIT);
                    Serial.printf("M3 -> %.1f shaft deg (cont step %.0f, wraps=%d)\n",
                        shaft_deg, m3_continuous_target, m3_wrap_count);
                }
                break;
            }
            case 'W': case 'w': {
                float p_deg = Serial.parseFloat();
                float r_deg = Serial.parseFloat();
                if (JOINT_ALLOWED[1]) {
                    p_deg = constrain(p_deg, 0.0f, PITCH_RANGE_DEG);
                    target_pos[1] = pitchDegToSteps(p_deg);
                }
                if (JOINT_ALLOWED[2]) {
                    target_pos[2] = wrapValue(rollDegToSteps(r_deg));
                }
                Serial.printf("Wrist -> P:%.1f R:%.1f deg\n", p_deg, r_deg);
                break;
            }

            case 'P': case 'p': {
                int id = Serial.parseInt(); float val = Serial.parseFloat();
                if (id >= 0 && id < 3) { Kp[id] = val; Serial.printf("Kp[%s] = %.2f\n", jointName(id), val); }
                break;
            }
            case 'I': case 'i': {
                int id = Serial.parseInt(); float val = Serial.parseFloat();
                if (id >= 0 && id < 3) { Ki[id] = val; Serial.printf("Ki[%s] = %.3f\n", jointName(id), val); }
                break;
            }
            case 'D': case 'd': {
                int id = Serial.parseInt(); float val = Serial.parseFloat();
                if (id >= 0 && id < 3) { Kd[id] = val; Serial.printf("Kd[%s] = %.3f\n", jointName(id), val); }
                break;
            }

            case 'E': case 'e': {
                Serial.println("--- ENCODERS ---");
                for (int i = 0; i < 3; i++) {
                    int val = readEncoder(i);
                    if (val != -1) {
                        if (i == 0) {
                            float out_deg = m3ContStepsToOutputDeg(m3_continuous_pos);
                            Serial.printf("  M3: enc=%d cont=%.0f wraps=%d shaft=%.1f deg\n",
                                val, m3_continuous_pos, m3_wrap_count, out_deg);
                        } else {
                            float deg = (i == 1) ? pitchStepsToDeg((float)val) : rollStepsToDeg((float)val);
                            Serial.printf("  %s: step=%d deg=%.1f %s\n",
                                jointName(i), val, deg, JOINT_ALLOWED[i] ? "ON" : "off");
                        }
                    } else {
                        Serial.printf("  %s: READ FAIL\n", jointName(i));
                    }
                }
                break;
            }

            case 'T': case 't': {
                char sub = Serial.read();
                while (sub == ' ' || sub == '\n' || sub == '\r') sub = Serial.read();

                if (sub == 'A' || sub == 'a' || sub == 'B' || sub == 'b' || sub == 'C' || sub == 'c') {
                    int pwm = Serial.parseInt();
                    int pin_idx = (sub == 'A' || sub == 'a') ? 1 :
                                  (sub == 'B' || sub == 'b') ? 2 : 0;
                    const char* name = (pin_idx == 0) ? "M3" :
                                       (pin_idx == 1) ? "Wrist A" : "Wrist B";
                    manual_test_mode = true;
                    stopAllMotorsRaw();
                    for (int j = 0; j < 3; j++) { motor_enabled[j] = false; integral[j] = 0; }

                    int before[3];
                    float m3_cont_before = m3_continuous_pos;
                    Serial.printf("TEST: %s (pin %d) PWM=%d for 1s\n", name, PWM_PINS[pin_idx], pwm);
                    Serial.println("BEFORE:");
                    for (int j = 0; j < 3; j++) {
                        before[j] = readEncoder(j);
                        if (before[j] != -1) {
                            if (j == 0) {
                                Serial.printf("  M3: enc=%d cont=%.0f shaft=%.1f deg\n",
                                    before[j], m3_continuous_pos, m3ContStepsToOutputDeg(m3_continuous_pos));
                            } else {
                                Serial.printf("  %s: %.1f deg (step %d)\n",
                                    jointName(j), jointStepsToDeg(j, (float)before[j]), before[j]);
                            }
                        }
                    }

                    driveMotorRaw(pin_idx, pwm);
                    unsigned long test_start = millis();
                    while (millis() - test_start < 1000) {
                        if (pin_idx == 0) m3UpdatePosition();
                        delay(10);
                    }
                    stopAllMotorsRaw();

                    Serial.println("AFTER:");
                    for (int j = 0; j < 3; j++) {
                        int after = readEncoder(j);
                        if (after != -1) {
                            if (j == 0) {
                                m3UpdatePosition();
                                float cont_delta = m3_continuous_pos - m3_cont_before;
                                Serial.printf("  M3: enc=%d cont=%.0f shaft=%.1f deg (delta=%+.0f cont steps)\n",
                                    after, m3_continuous_pos, m3ContStepsToOutputDeg(m3_continuous_pos), cont_delta);
                            } else {
                                float delta = wrapError((float)after - (float)before[j]);
                                Serial.printf("  %s: %.1f deg (step %d) delta=%+.0f\n",
                                    jointName(j), jointStepsToDeg(j, (float)after), after, delta);
                            }
                        }
                    }
                    if (pin_idx == 0) {
                        float cont_delta = m3_continuous_pos - m3_cont_before;
                        Serial.println("--- M3 DIRECTION CHECK ---");
                        if (pwm > 0 && cont_delta > 0) Serial.println("  +PWM = +cont: M3_PID_SIGN should be +1");
                        if (pwm > 0 && cont_delta < 0) Serial.println("  +PWM = -cont: M3_PID_SIGN should be -1 >>> F 3");
                        if (pwm < 0 && cont_delta < 0) Serial.println("  -PWM = -cont: M3_PID_SIGN should be +1");
                        if (pwm < 0 && cont_delta > 0) Serial.println("  -PWM = +cont: M3_PID_SIGN should be -1 >>> F 3");
                        Serial.printf("  Current M3_PID_SIGN = %d  wraps = %d\n", M3_PID_SIGN, m3_wrap_count);
                    }
                }
                else if (sub == '0') {
                    stopAllMotorsRaw();
                    Serial.println("Motors stopped");
                    for (int j = 0; j < 3; j++) {
                        int val = readEncoder(j);
                        if (val != -1) Serial.printf("  %s: %.1f deg\n", jointName(j),
                            (j == 0) ? m3ContStepsToOutputDeg(m3_continuous_pos) : jointStepsToDeg(j, (float)val));
                    }
                }
                else if (sub == 'X' || sub == 'x') {
                    manual_test_mode = false;
                    stopAllMotorsRaw();
                    Serial.println("TEST MODE OFF — send R to re-enable PID");
                }
                else {
                    manual_test_mode = true;
                    stopAllMotorsRaw();
                    for (int j = 0; j < 3; j++) { motor_enabled[j] = false; integral[j] = 0; }
                    Serial.println("=== MANUAL TEST MODE ===");
                    Serial.println("  T C [pwm] - Motor 3 for 1s (tracks wraps!)");
                    Serial.println("  T A [pwm] - Wrist motor A for 1s");
                    Serial.println("  T B [pwm] - Wrist motor B for 1s");
                    Serial.println("  T 0       - Stop all, read encoders");
                    Serial.println("  T X       - Exit test mode");
                    Serial.printf("  M3: cont=%.0f wraps=%d shaft=%.1f deg\n",
                        m3_continuous_pos, m3_wrap_count, m3ContStepsToOutputDeg(m3_continuous_pos));
                }
                break;
            }

            case 'F': case 'f': {
                char which = Serial.read();
                while (which == ' ') which = Serial.read();
                if (which == '3') {
                    M3_PID_SIGN *= -1;
                    Serial.printf("M3_PID_SIGN -> %d\n", M3_PID_SIGN);
                }
                else if (which == 'R' || which == 'r') {
                    ROLL_PID_SIGN *= -1;
                    Serial.printf("ROLL_PID_SIGN -> %d\n", ROLL_PID_SIGN);
                }
                else if (which == 'M' || which == 'm') {
                    MOTOR_A_ROLL_SIGN *= -1;
                    Serial.printf("MOTOR_A_ROLL_SIGN -> %d\n", MOTOR_A_ROLL_SIGN);
                }
                else {
                    Serial.printf("Signs: M3_PID=%d ROLL_PID=%d ROLL_MIX=%d\n",
                        M3_PID_SIGN, ROLL_PID_SIGN, MOTOR_A_ROLL_SIGN);
                }
                break;
            }

            case 'V': case 'v': {
                int id = Serial.parseInt();
                if (id >= 0 && id < 3) {
                    debug_show[id] = !debug_show[id];
                    Serial.printf("%s debug: %s\n", jointName(id), debug_show[id] ? "ON" : "OFF");
                } else {
                    Serial.printf("Visible: M3=%d Pitch=%d Roll=%d\n", debug_show[0], debug_show[1], debug_show[2]);
                }
                break;
            }

            case 'S': case 's': emergencyStopAll(); break;

            case 'R': case 'r':
                manual_test_mode = false;
                Serial.println("Re-enabling all joints...");
                for (int j = 0; j < 3; j++) {
                    if (!JOINT_ALLOWED[j]) {
                        motor_enabled[j] = false;
                        Serial.printf("  %s: DISABLED\n", jointName(j));
                        continue;
                    }
                    int val = readEncoder(j);
                    if (val != -1) {
                        motor_enabled[j]  = true;
                        current_pos[j]    = val;
                        target_pos[j]     = val;
                        commanded_pos[j]  = val;
                        prev_pos[j]       = val;
                        integral[j]       = 0;

                        if (j == 0) {
                            m3UpdatePosition();
                            m3_continuous_target = m3_continuous_pos;
                            m3_continuous_cmd    = m3_continuous_pos;
                            m3_continuous_prev   = m3_continuous_pos;
                            Serial.printf("  M3: ON at shaft %.1f deg (cont=%.0f wraps=%d)\n",
                                m3ContStepsToOutputDeg(m3_continuous_pos), m3_continuous_pos, m3_wrap_count);
                        } else {
                            float deg = (j == 1) ? pitchStepsToDeg((float)val) : rollStepsToDeg((float)val);
                            Serial.printf("  %s: ON at %.1f deg (step %d)\n", jointName(j), deg, val);
                        }
                    } else {
                        Serial.printf("  %s: ENCODER FAIL\n", jointName(j));
                    }
                }
                break;

            case '?': {
                Serial.println("========== STATUS ==========");
                float m3_out = m3ContStepsToOutputDeg(m3_continuous_pos);
                float m3_tgt = m3ContStepsToOutputDeg(m3_continuous_target);
                float m3_err = m3_continuous_cmd - m3_continuous_pos;
                Serial.printf("  M3:    cur=%+.1f tgt=%+.1f pid=%.0f en=%d\n",
                    m3_out, m3_tgt, pid_output[0], motor_enabled[0]);
                Serial.printf("         cont: pos=%.0f tgt=%.0f cmd=%.0f err=%.0f\n",
                    m3_continuous_pos, m3_continuous_target, m3_continuous_cmd, m3_err);
                Serial.printf("         enc=%d wraps=%d limit=±%.0f\n",
                    m3_prev_raw, m3_wrap_count, M3_CONT_LIMIT);
                Serial.printf("         Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp[0], Ki[0], Kd[0]);
                for (int j = 1; j < 3; j++) {
                    float cur = jointStepsToDeg(j, current_pos[j]);
                    float tgt = jointStepsToDeg(j, target_pos[j]);
                    float err = wrapError(commanded_pos[j] - current_pos[j]);
                    Serial.printf("  %s: cur=%+.1f tgt=%+.1f pid=%.0f en=%d err=%.0f\n",
                        jointName(j), cur, tgt, pid_output[j], motor_enabled[j], err);
                    Serial.printf("         Kp=%.2f Ki=%.3f Kd=%.3f\n", Kp[j], Ki[j], Kd[j]);
                }
                Serial.printf("  Signs: M3_PID=%d ROLL_PID=%d ROLL_MIX=%d DIR=[%d,%d,%d]\n",
                    M3_PID_SIGN, ROLL_PID_SIGN, MOTOR_A_ROLL_SIGN,
                    MOTOR_DIR_SIGN[0], MOTOR_DIR_SIGN[1], MOTOR_DIR_SIGN[2]);
                Serial.printf("  Mode: %s | ROS: %s\n",
                    manual_test_mode ? "MANUAL" : "PID",
                    comms_ok ? "CONNECTED" : "DISCONNECTED");
                Serial.println("============================");
                break;
            }

            case 'H': case 'h':
                Serial.println("========= COMMANDS =========");
                Serial.println("  A [m3] [pitch] [roll]- Move ALL (Absolute)");
                Serial.println("  0 [deg]          - Move M3 (-180 to +180 shaft)");
                Serial.println("  1 [deg]          - Move Pitch");
                Serial.println("  2 [deg]          - Move Roll");
                Serial.println("  M [deg]          - Move M3 shaft");
                Serial.println("  W [pitch] [roll] - Move wrist");
                Serial.println("  P/I/D [id] [val] - Tune PID (0=M3 1=Pitch 2=Roll)");
                Serial.println("  T                - Raw motor test mode");
                Serial.println("  T C/A/B [pwm]    - Raw drive M3/WristA/WristB");
                Serial.println("  F 3 / F R / F M  - Flip M3/roll/mixer signs");
                Serial.println("  V [id]           - Toggle debug for joint");
                Serial.println("  E - Encoders | S - Stop | R - Reset");
                Serial.println("  ? - Status   | H - This help");
                Serial.println("  --- ROS UART on pins 8(RX)/9(TX) ---");
                Serial.println("  ROS packets auto-enable motors;");
                Serial.println("  watchdog stops them if ROS goes silent.");
                Serial.println("============================");
                break;
        }
    }
    while (Serial.available()) Serial.read();
}

// =================================================================
//                           SETUP
// =================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- Status LED (magenta = booting) ---
    led.begin();
    led.setBrightness(40);
    setLed(led.Color(180, 0, 180));

    // --- ROS UART ---
    ROSSerial.begin(115200, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN);

    Serial.println("===== 3-JOINT CONTROL SYSTEM + ROS UART =====");
    Serial.println("M3: wrap-tracking enabled (encoder on motor, 2:1 gearbox)");
    Serial.printf("M3: shaft -180 to +180 deg (motor ±360, PID_SIGN=%d)\n", M3_PID_SIGN);
    Serial.printf("Pitch: 0-%.1f deg | Roll: 0-360 deg\n", PITCH_RANGE_DEG);
    Serial.printf("PID: Kp=[%.2f,%.2f,%.2f] Ki=[%.3f,%.3f,%.3f] Kd=[%.3f,%.3f,%.3f]\n",
        Kp[0], Kp[1], Kp[2], Ki[0], Ki[1], Ki[2], Kd[0], Kd[1], Kd[2]);
    Serial.printf("ROS UART: RX=%d TX=%d @ 115200  Watchdog=%lums\n",
        ROS_RX_PIN, ROS_TX_PIN, COMMS_TIMEOUT_MS);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);

    for (int i = 0; i < 3; i++) {
        pinMode(DIR_PINS[i], OUTPUT);
        ledcAttach(PWM_PINS[i], LEDC_FREQ, LEDC_RES);
        ledcWrite(PWM_PINS[i], 0);
    }

    if (!i2cHealthCheck()) i2cReset();

    for (int i = 0; i < 3; i++) {
        if (!JOINT_ALLOWED[i]) {
            Serial.printf("  %s: DISABLED\n", jointName(i));
            continue;
        }
        int val = readEncoder(i);
        if (val != -1) {
            motor_enabled[i]  = false;   // Motors stay off until ROS connects or 'R' is sent
            current_pos[i]    = val;
            target_pos[i]     = val;
            commanded_pos[i]  = val;
            prev_pos[i]       = val;

            if (i == 0) {
                m3Init(val);
                Serial.printf("  M3: OK at shaft %.1f deg (enc=%d, cont=%.0f)\n",
                    m3ContStepsToOutputDeg(m3_continuous_pos), val, m3_continuous_pos);
                Serial.println("  >>> M3 assumes startup = home half (-90 to +90 shaft)");
                Serial.println("  >>> If M3 starts beyond ±90, position will be WRONG");
            } else {
                float deg = (i == 1) ? pitchStepsToDeg((float)val) : rollStepsToDeg((float)val);
                Serial.printf("  %s: OK at %.1f deg (step %d)\n", jointName(i), deg, val);
            }
        } else {
            Serial.printf("  %s: ENCODER FAIL\n", jointName(i));
        }
    }

    last_valid_cmd_time = millis();

    Serial.println("");
    Serial.println(">>> Motors OFF until ROS connects or you send 'R' via USB.");
    Serial.println(">>> Run T C 100 to check direction, F 3 to flip");
    Serial.println("");
    Serial.println("Send H for help, S for stop");
    Serial.println("==============================================");
}

// =================================================================
//                           LOOP
// =================================================================

void loop() {
    unsigned long now = millis();

    // 1. Watchdog — stops motors if RPi goes silent
    checkCommsWatchdog();

    // 2. ROS UART — receive command packets, send feedback
    processROSCommand();

    // 3. USB serial — local commands always available
    processSerialCommand();

    // 4. PID control loop at 50 Hz
    if (!manual_test_mode && (now - last_pid_time >= 20)) {
        float dt = (now - last_pid_time) / 1000.0f;
        if (dt > 0.1f) dt = 0.02f;  // Cap to prevent integral windup after pause
        last_pid_time = now;

        readAllSensors();
        updateTrajectory();
        computePID(dt);
        applyMotorOutputs();
    }

    // 5. Status LED — non-blocking, updates every 100ms
    updateLed();

    // 6. Debug output every 200ms (USB only, does not affect ROS UART)
    if (now - last_debug_time > 200) {
        last_debug_time = now;
        if (!manual_test_mode) {
            if (motor_enabled[0] && debug_show[0]) {
                float cur = m3ContStepsToOutputDeg(m3_continuous_pos);
                float tgt = m3ContStepsToOutputDeg(m3_continuous_target);
                float err = m3_continuous_cmd - m3_continuous_pos;
                float err_dz = applyDeadzone(err, ERROR_DEADZONE);
                Serial.printf("M3 | tgt:%+.1f cur:%+.1f | pid:%.0f | P:%.1f I:%.1f | cont:%.0f w:%d\n",
                    tgt, cur, pid_output[0],
                    Kp[0] * err_dz, Ki[0] * integral[0],
                    m3_continuous_pos, m3_wrap_count);
            }
            for (int i = 1; i < 3; i++) {
                if (!motor_enabled[i] || !debug_show[i]) continue;
                float cur = jointStepsToDeg(i, current_pos[i]);
                float tgt = jointStepsToDeg(i, target_pos[i]);
                float err = wrapError(commanded_pos[i] - current_pos[i]);
                float err_dz = applyDeadzone(err, ERROR_DEADZONE);
                Serial.printf("%s | tgt:%+.1f cur:%+.1f | pid:%.0f | P:%.1f I:%.1f\n",
                    jointName(i), tgt, cur, pid_output[i],
                    Kp[i] * err_dz, Ki[i] * integral[i]);
            }
        }
    }
}