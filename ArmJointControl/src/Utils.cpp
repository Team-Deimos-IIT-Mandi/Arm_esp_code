#include "../include/Utils.h"
#include <math.h>

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

float m3OutputDegToContSteps(float output_deg) {
    output_deg = constrain(output_deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
    float motor_deg = outputToMotorDeg(output_deg);
    return motorDegToSteps(motor_deg);
}

float m3ContStepsToOutputDeg(float cont_steps) {
    float motor_deg = motorStepsToDeg(cont_steps);
    return motorToOutputDeg(motor_deg);
}

float jointStepsToDeg(int joint, float steps, float m3_cont_pos) {
    if (joint == 0) return m3ContStepsToOutputDeg(m3_cont_pos);
    if (joint == 1) return pitchStepsToDeg(steps);
    return rollStepsToDeg(steps);
}

const char* jointName(int joint) {
    if (joint == 0) return "M3";
    if (joint == 1) return "Pitch";
    return "Roll";
}
