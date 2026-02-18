#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "Config.h"

float wrapValue(float val);
float wrapError(float error);
float applyDeadzone(float error, float deadzone);

// Angle Conversion Logic
float motorDegToSteps(float motor_deg);
float motorStepsToDeg(float steps);
float outputToMotorDeg(float output_deg);
float motorToOutputDeg(float motor_deg);
float pitchStepsToDeg(float steps);
float pitchDegToSteps(float deg);
float rollStepsToDeg(float steps);
float rollDegToSteps(float deg);

// Joint Conversions
float m3OutputDegToContSteps(float output_deg);
float m3ContStepsToOutputDeg(float cont_steps);
float jointStepsToDeg(int joint, float steps, float m3_cont_pos);

const char* jointName(int joint);

#endif
