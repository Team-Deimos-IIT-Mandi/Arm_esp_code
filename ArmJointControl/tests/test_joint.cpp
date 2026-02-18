#include <iostream>
#include <cassert>
#include <cmath>
#include "../include/Joint.h"
#include "../include/Config.h"
#include "mocks/Hardware.h"

// External globals from Globals.cpp
float Kp[3] = {4.0f,  4.0f,  4.0f};
float Ki[3] = {0.05f, 0.02f, 0.02f};
float Kd[3] = {0.0f,  0.15f, 0.15f};
int M3_PID_SIGN = 1;
const int MOTOR_DIR_SIGN[3] = {1, 1, 1};
int MOTOR_A_ROLL_SIGN = 1;
int ROLL_PID_SIGN = 1;

extern int current_encoder_val[3];

void test_joint_reset() {
    std::cout << "Testing Joint reset..." << std::endl;
    Joint j(1);
    current_encoder_val[1] = 1000;
    j.reset();
    assert(j.enabled == true);
    assert(j.current_pos == 1000);
    assert(j.target_pos == 1000);
    assert(j.integral == 0);
    std::cout << "  Passed!" << std::endl;
}

void test_joint_trajectory() {
    std::cout << "Testing Joint trajectory..." << std::endl;
    Joint j(1);
    current_encoder_val[1] = 1000;
    j.reset();
    
    // Set target far away
    j.target_pos = 2000;
    
    // MAX_SLEW_RATE is 40.0
    j.updateTrajectory();
    // 1000 + 40 = 1040
    assert(j.commanded_pos == 1040);
    
    j.updateTrajectory();
    assert(j.commanded_pos == 1080);
    
    std::cout << "  Passed!" << std::endl;
}

void test_joint_pid() {
    std::cout << "Testing Joint PID..." << std::endl;
    Joint j(1); // Joint 1: Pitch
    current_encoder_val[1] = 1000;
    j.reset();
    
    // commanded_pos is 1000. Set target and commanded to 1100 to simulate a jump or after slew.
    j.commanded_pos = 1100;
    j.current_pos = 1000;
    
    // error = 100. Kp[1] = 4.0. error_dz = applyDeadzone(100, 5) = 95.
    // p_term = 4.0 * 95 = 380. 
    // Constrained to PID_OUTPUT_MAX (255).
    
    j.computePID(0.02); // 20ms = 50Hz
    assert(j.pid_output == 255.0f);
    
    // Close to target
    j.current_pos = 1098; // error = 2. error_dz = 0.
    j.computePID(0.02);
    assert(j.pid_output == 0.0f);
    
    std::cout << "  Passed!" << std::endl;
}

void test_m3_wrapping() {
    std::cout << "Testing M3 wrapping..." << std::endl;
    Joint j(0); // M3
    current_encoder_val[0] = 4000;
    j.reset(); // m3_prev_raw = 4000, continuous_pos = 4000
    
    // Move forward past wrap
    current_encoder_val[0] = 100; // raw jumped from 4000 to 100
    // delta = 100 - 4000 = -3900. delta < -2048, so wrap_count++
    j.updateSensor();
    assert(j.m3_wrap_count == 1);
    // continuous_pos = 1 * 4096 + 100 = 4196
    assert(j.continuous_pos == 4196.0f);
    
    // Move backward past wrap
    current_encoder_val[0] = 4000; // raw jumped from 100 to 4000
    // delta = 4000 - 100 = 3900. delta > 2048, so wrap_count--
    j.updateSensor();
    assert(j.m3_wrap_count == 0);
    assert(j.continuous_pos == 4000.0f);
    
    std::cout << "  Passed!" << std::endl;
}

int main() {
    test_joint_reset();
    test_joint_trajectory();
    test_joint_pid();
    test_m3_wrapping();
    std::cout << "All Joint tests passed!" << std::endl;
    return 0;
}
