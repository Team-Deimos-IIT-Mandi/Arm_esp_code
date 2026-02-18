#include "../include/Config.h"

// PID Gains
float Kp[3] = {4.0f,  4.0f,  4.0f};
float Ki[3] = {0.05f, 0.02f, 0.02f};
float Kd[3] = {0.0f,  0.15f, 0.15f};

// Direction Signs
int       M3_PID_SIGN         = 1;
const int MOTOR_DIR_SIGN[3]   = {1, 1, 1};
int       MOTOR_A_ROLL_SIGN   = 1;
int       ROLL_PID_SIGN       = 1;
