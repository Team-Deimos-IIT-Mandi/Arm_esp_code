#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "Config.h"

extern Adafruit_NeoPixel led;

void hardwareSetup();
void tcaSelect(uint8_t channel);
void i2cReset();
bool i2cHealthCheck();

int readEncoderRaw(int joint);
int readEncoder(int joint);

void setMotor(int joint, int pwm_val, bool enabled);
void setMotorM3(int pwm_val, bool enabled);
void driveMotorRaw(int pin_index, int pwm_signed);
void stopAllMotorsRaw();

void setLed(uint32_t colour);

#endif
