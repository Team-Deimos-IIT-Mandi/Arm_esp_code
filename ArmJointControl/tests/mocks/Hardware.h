#ifndef HARDWARE_H
#define HARDWARE_H

#include "Arduino.h"
#include "../include/Config.h"

int readEncoderRaw(int joint);
int readEncoder(int joint);

void setMotor(int joint, int pwm_val, bool enabled);
void setMotorM3(int pwm_val, bool enabled);
void driveMotorRaw(int pin_index, int pwm_signed);
void stopAllMotorsRaw();

#endif
