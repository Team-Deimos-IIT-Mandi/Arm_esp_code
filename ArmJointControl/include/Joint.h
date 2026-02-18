#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "Config.h"
#include "Hardware.h"
#include "Utils.h"

class Joint {
public:
    int id;
    bool enabled = false;
    float current_pos = 0;
    float target_pos = 0;
    float commanded_pos = 0;
    float prev_pos = 0;
    float integral = 0;
    float pid_output = 0;
    int sensor_fail_count = 0;
    bool sensor_read_ok = false;
    bool debug_show = true;

    // Multi-turn tracking for M3
    int m3_wrap_count = 0;
    int m3_prev_raw = 0;
    float continuous_pos = 0;
    float continuous_target = 0;
    float continuous_cmd = 0;
    float continuous_prev = 0;

    Joint(int _id);
    void reset();
    void updateSensor();
    void updateTrajectory();
    void computePID(float dt);
    
    // M3 specific
    bool m3UpdatePosition();
    void m3Init(int encoder_val);

private:
    void applyPitchSafety();
    void applyM3Safety();
};

#endif
