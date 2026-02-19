#include "include/Config.h"
#include "include/Hardware.h"
#include "include/Joint.h"
#include "include/Comms.h"
#include "include/Utils.h"

Joint joints[3] = {Joint(0), Joint(1), Joint(2)};
unsigned long last_pid_time = 0;
unsigned long last_debug_time = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    hardwareSetup();
    commsSetup();
    
    // Initial readout
    for (int i = 0; i < 3; i++) {
        if (JOINT_ALLOWED[i]) {
            int val = readEncoder(i);
            if (val != -1) {
                joints[i].current_pos = val;
                if (i == 0) joints[i].m3Init(val);
                Serial.print("  J%d: OK at %.1f", i, jointStepsToDeg(i, (float)val, joints[i].continuous_pos));
            } else {
                Serial.print("  J%d: ENCODER FAIL", i);
            }
        }
    }
    
    Serial.println("System Initialized. Awaiting ROS or USB 'R' to start.");
}

void loop() {
    unsigned long now = millis();

    // 1. ROS Watchdog
    checkCommsWatchdog(joints);

    // 2. ROS UART Processing
    processROSCommand(joints);

    // 3. USB Serial Command Interface
    processSerialCommand(joints);

    // 4. PID control loop at 50 Hz
    if (!manual_test_mode && (now - last_pid_time >= 20)) {
        float dt = (now - last_pid_time) / 1000.0f;
        if (dt > 0.1f) dt = 0.02f;
        last_pid_time = now;

        // Sensor Reading
        if (!i2cHealthCheck()) i2cReset();
        for (int i = 0; i < 3; i++) {
            joints[i].updateSensor();
            if (joints[i].sensor_fail_count > MAX_SENSOR_FAILS) {
                for(int j=0; j<3; j++) joints[j].enabled = false;
                stopAllMotorsRaw();
                Serial.println("!!! SENSOR FAIL E-STOP !!!");
            }
        }

        // Trajectory & PID
        for (int i = 0; i < 3; i++) {
            joints[i].updateTrajectory();
            joints[i].computePID(dt);
        }

        // Apply Motor Outputs
        applyMotorOutputs();
    }

    // 5. LED Status
    updateLed(joints);

    // 6. Debugging (USB)
    if (now - last_debug_time > 200) {
        last_debug_time = now;
        // Optional debug logging can go here
    }
}

void applyMotorOutputs() {
    if (manual_test_mode) return;

    // Joint 0 (M3)
    setMotorM3((int)joints[0].pid_output, joints[0].enabled);

    // Joint 1 (Pitch) and 2 (Roll) - Differential Wrist
    bool p_active = joints[1].enabled && joints[1].sensor_read_ok;
    bool r_active = joints[2].enabled && joints[2].sensor_read_ok;

    if (p_active || r_active) {
        float p_cmd = p_active ? joints[1].pid_output : 0.0f;
        float r_cmd = r_active ? (joints[2].pid_output * ROLL_PID_SIGN) : 0.0f;

        float mA = p_cmd + r_cmd;
        float mB = -p_cmd + r_cmd;

        float max_val = max(fabsf(mA), fabsf(mB));
        if (max_val > 255.0f) {
            float scale = 255.0f / max_val;
            mA *= scale;
            mB *= scale;
        }

        int pwmA   = (int)mA * MOTOR_DIR_SIGN[1];
        int pwmB   = (int)mB * MOTOR_DIR_SIGN[2];

        driveMotorRaw(1, pwmA);
        driveMotorRaw(2, pwmB);
    } else {
        ledcWrite(PWM_PINS[1], 0);
        ledcWrite(PWM_PINS[2], 0);
    }
}
