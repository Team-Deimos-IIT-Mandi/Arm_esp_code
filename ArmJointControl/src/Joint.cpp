#include "../include/Joint.h"
#include <math.h>

Joint::Joint(int _id) : id(_id) {}

void Joint::reset() {
    int val = readEncoder(id);
    if (val != -1) {
        enabled      = true;
        current_pos  = val;
        target_pos   = val;
        commanded_pos = val;
        prev_pos     = val;
        integral     = 0;
        sensor_fail_count = 0;
        sensor_read_ok = true;

        if (id == 0) {
            m3Init(val);
        }
    }
}

void Joint::updateSensor() {
    sensor_read_ok = false;
    if (!enabled) return;

    if (id == 0) {
        if (m3UpdatePosition()) {
            sensor_fail_count = 0;
            sensor_read_ok = true;
        } else {
            sensor_fail_count++;
        }
    } else {
        int raw = readEncoder(id);
        if (raw != -1) {
            current_pos = raw;
            sensor_fail_count = 0;
            sensor_read_ok = true;
        } else {
            sensor_fail_count++;
        }
    }
}

bool Joint::m3UpdatePosition() {
    int raw = readEncoder(0);
    if (raw == -1) return false;

    int delta = raw - m3_prev_raw;
    if (delta >  ENCODER_HALF) m3_wrap_count--;
    else if (delta < -ENCODER_HALF) m3_wrap_count++;

    m3_prev_raw    = raw;
    continuous_pos = (float)(m3_wrap_count * ENCODER_COUNTS) + (float)raw;
    current_pos    = (float)raw;
    return true;
}

void Joint::m3Init(int encoder_val) {
    m3_prev_raw       = encoder_val;
    m3_wrap_count     = 0;
    continuous_pos    = (float)encoder_val;
    continuous_target = continuous_pos;
    continuous_cmd    = continuous_pos;
    continuous_prev   = continuous_pos;
}

void Joint::updateTrajectory() {
    if (!enabled) {
        if (id == 0) continuous_cmd = continuous_pos;
        else commanded_pos = current_pos;
        return;
    }

    if (id == 0) {
        float error_to_target = continuous_target - continuous_cmd;
        if (fabsf(error_to_target) > MAX_SLEW_RATE) {
            continuous_cmd += (error_to_target > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
        } else {
            continuous_cmd = continuous_target;
        }
        continuous_cmd = constrain(continuous_cmd, -M3_CONT_LIMIT, M3_CONT_LIMIT);
    } else {
        float error_to_target = target_pos - commanded_pos;
        error_to_target = wrapError(error_to_target);
        if (fabsf(error_to_target) > MAX_SLEW_RATE) {
            commanded_pos += (error_to_target > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
            commanded_pos = wrapValue(commanded_pos);
        } else {
            commanded_pos = target_pos;
        }
    }
}

void Joint::computePID(float dt) {
    if (!enabled) { pid_output = 0; integral = 0; return; }

    float error, error_dz, dInput;

    if (id == 0) {
        error    = continuous_cmd - continuous_pos;
        error_dz = applyDeadzone(error, ERROR_DEADZONE);
        dInput   = sensor_read_ok ? (continuous_pos - continuous_prev) : 0;
        continuous_prev = continuous_pos;
    } else {
        error    = commanded_pos - current_pos;
        error    = wrapError(error);
        error_dz = applyDeadzone(error, ERROR_DEADZONE);
        dInput   = sensor_read_ok ? wrapError(current_pos - prev_pos) : 0;
        prev_pos = current_pos;
    }

    float p_term = Kp[id] * error_dz;

    if (fabsf(pid_output) < PID_OUTPUT_MAX) {
        integral += (error_dz * dt);
        integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    }
    if (fabsf(error_dz) < 0.1f) integral = 0;
    float i_term = Ki[id] * integral;

    float d_term = Kd[id] * (dInput / dt);

    float output = p_term + i_term - d_term;
    output = constrain(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    pid_output = output;

    if (id == 0) applyM3Safety();
    if (id == 1) applyPitchSafety();
}

void Joint::applyM3Safety() {
    if (continuous_pos >= M3_CONT_LIMIT && pid_output > 0) {
        pid_output = 0; integral = 0;
    }
    if (continuous_pos <= -M3_CONT_LIMIT && pid_output < 0) {
        pid_output = 0; integral = 0;
    }
    // soft slowdown approaching +limit
    if (continuous_pos > (M3_CONT_LIMIT - M3_CONT_SOFT_ZONE) && continuous_pos < M3_CONT_LIMIT && pid_output > 0) {
        float scale = (M3_CONT_LIMIT - continuous_pos) / M3_CONT_SOFT_ZONE;
        pid_output *= constrain(scale, 0.0f, 1.0f);
    }
    // soft slowdown approaching -limit
    if (continuous_pos < (-M3_CONT_LIMIT + M3_CONT_SOFT_ZONE) && continuous_pos > -M3_CONT_LIMIT && pid_output < 0) {
        float scale = (continuous_pos + M3_CONT_LIMIT) / M3_CONT_SOFT_ZONE;
        pid_output *= constrain(scale, 0.0f, 1.0f);
    }
}

void Joint::applyPitchSafety() {
    float pos = current_pos;
    const float WRAP_BUFFER = 200.0f;
    bool in_valid_zone      = (pos <= (float)PITCH_LIMIT_MAX);
    bool in_near_zero_wrap  = (pos >= (4096.0f - WRAP_BUFFER));
    bool in_forbidden_zone  = (!in_valid_zone && !in_near_zero_wrap);

    if (in_forbidden_zone || (pos >= (float)PITCH_LIMIT_MAX && pos <= (float)ENCODER_HALF)) {
        if (pid_output > 0) { pid_output = 0; integral = 0; }
    }
    if (pos <= (float)PITCH_LIMIT_MIN && pos < WRAP_BUFFER) {
        if (pid_output < 0) { pid_output = 0; integral = 0; }
    }
}
