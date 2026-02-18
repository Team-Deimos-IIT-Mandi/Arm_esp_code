#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== ROS UART CONFIG ===========================
const int ROS_RX_PIN = 8;
const int ROS_TX_PIN = 9;
const long ROS_BAUD  = 115200;

// ===================== STATUS LED ================================
#define LED_PIN   21
#define LED_COUNT 1

// ===================== WATCHDOG ==================================
const unsigned long COMMS_TIMEOUT_MS = 500;
const unsigned long LED_UPDATE_MS    = 100;

// ================= USER CALIBRATION SECTION ======================
const bool JOINT_ALLOWED[3] = {true, true, true};
const int HOME_OFFSETS[3]   = {430, 1883, 0};
const int POS_MIN[3]        = {0, 0, 0};
const int POS_MAX[3]        = {4096, 1837, 4096};
const int PITCH_LIMIT_MIN   = 0;
const int PITCH_LIMIT_MAX   = 1837;

// ================= MOTOR 3 CONFIG ================================
const float M3_GEAR_RATIO   = 2.0f;
const float M3_MOTOR_LIMIT  = 360.0f;
const float M3_OUTPUT_LIMIT = 180.0f;
const float M3_CONT_LIMIT   = 4096.0f;
const float M3_CONT_SOFT_ZONE = 400.0f;

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

// ================= PID & TRAJECTORY GAINS ========================
const float PID_OUTPUT_MAX = 255.0f;
const float INTEGRAL_MAX   = 100.0f;
const float ERROR_DEADZONE = 5.0f;
const int   MAX_SENSOR_FAILS = 10;
const float MAX_SLEW_RATE  = 40.0f;

// ================= PID INITIAL GAINS =============================
extern float Kp[3];
extern float Ki[3];
extern float Kd[3];

// ================= DIRECTION SIGNS ===============================
extern int M3_PID_SIGN;
extern const int MOTOR_DIR_SIGN[3];
extern int MOTOR_A_ROLL_SIGN;
extern int ROLL_PID_SIGN;

#endif
