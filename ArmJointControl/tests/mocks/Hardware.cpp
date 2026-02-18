#include "Hardware.h"

int current_encoder_val[3] = {0, 0, 0};

int readEncoderRaw(int joint) { return current_encoder_val[joint]; }
int readEncoder(int joint) { return current_encoder_val[joint]; }

void setMotor(int joint, int pwm_val, bool enabled) {}
void setMotorM3(int pwm_val, bool enabled) {}
void driveMotorRaw(int pin_index, int pwm_signed) {}
void stopAllMotorsRaw() {}
