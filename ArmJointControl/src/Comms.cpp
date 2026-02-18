#include "../include/Comms.h"

HardwareSerial ROSSerial(1);
unsigned long last_valid_cmd_time = 0;
unsigned long lastLedUpdate = 0;
bool comms_ok = false;
bool manual_test_mode = false;

void commsSetup() {
    ROSSerial.begin(ROS_BAUD, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN);
    last_valid_cmd_time = millis();
}

bool tryReadPacket(CommandPacketB& cmd) {
    for (int i = 0; i < 32 && ROSSerial.available() > 0; i++) {
        if (ROSSerial.peek() == 'S') break;
        ROSSerial.read();
    }
    if (ROSSerial.available() < (int)sizeof(CommandPacketB)) return false;
    ROSSerial.readBytes((char*)&cmd, sizeof(CommandPacketB));
    if (cmd.header[0] == 'S' && cmd.header[1] == 'T' && cmd.footer == '
') return true;
    while (ROSSerial.available()) ROSSerial.read();
    return false;
}

void processROSCommand(Joint* joints) {
    CommandPacketB cmd;
    if (!tryReadPacket(cmd)) return;

    last_valid_cmd_time = millis();

    if (!comms_ok) {
        comms_ok = true;
        Serial.println("ROS: connected — motors re-enabled.");
        for (int i = 0; i < 3; i++) {
            if (JOINT_ALLOWED[i]) joints[i].reset();
        }
    }

    if (JOINT_ALLOWED[0]) {
        joints[0].continuous_target = constrain((float)cmd.motor_cmd[0], -M3_CONT_LIMIT, M3_CONT_LIMIT);
    }
    if (JOINT_ALLOWED[1]) {
        joints[1].target_pos = constrain((float)cmd.motor_cmd[1], (float)POS_MIN[1], (float)POS_MAX[1]);
    }
    if (JOINT_ALLOWED[2]) {
        joints[2].target_pos = wrapValue((float)cmd.motor_cmd[2]);
    }

    FeedbackPacketB fb;
    fb.motor_pos[0] = (int32_t)joints[0].continuous_pos;
    fb.motor_pos[1] = (int32_t)joints[1].current_pos;
    fb.motor_pos[2] = (int32_t)joints[2].current_pos;
    ROSSerial.write((uint8_t*)&fb, sizeof(FeedbackPacketB));
}

void checkCommsWatchdog(Joint* joints) {
    if (!comms_ok) return;
    if ((millis() - last_valid_cmd_time) > COMMS_TIMEOUT_MS) {
        comms_ok = false;
        for (int i = 0; i < 3; i++) {
            joints[i].enabled = false;
            joints[i].integral = 0;
            ledcWrite(PWM_PINS[i], 0);
        }
        Serial.println("ROS WATCHDOG: comms lost — motors stopped.");
    }
}

void updateLed(Joint* joints) {
    if (millis() - lastLedUpdate < LED_UPDATE_MS) return;
    lastLedUpdate = millis();

    if (!comms_ok) {
        setLed(led.Color(180, 0, 180));   // MAGENTA
        return;
    }

    bool moving = false;
    for (int i = 0; i < 3; i++) {
        if (joints[i].enabled && fabsf(joints[i].pid_output) > 15.0f) {
            moving = true;
            break;
        }
    }
    setLed(moving ? led.Color(0, 0, 255) : led.Color(0, 180, 0)); // BLUE : GREEN
}

void processSerialCommand(Joint* joints) {
    if (!Serial.available()) return;
    char type = Serial.peek();

    if (type >= '0' && type <= '9') {
        int id = Serial.parseInt();
        float deg = Serial.parseFloat();
        if (id >= 0 && id < 3 && JOINT_ALLOWED[id]) {
            if (id == 0) {
                deg = constrain(deg, -M3_OUTPUT_LIMIT, M3_OUTPUT_LIMIT);
                joints[0].continuous_target = m3OutputDegToContSteps(deg);
                Serial.printf("CMD M3: -> %.1f deg
", deg);
            } else if (id == 1) {
                deg = constrain(deg, 0.0f, pitchStepsToDeg(POS_MAX[1]));
                joints[1].target_pos = pitchDegToSteps(deg);
                Serial.printf("CMD Pitch: -> %.1f deg
", deg);
            } else {
                joints[2].target_pos = wrapValue(rollDegToSteps(deg));
                Serial.printf("CMD Roll: -> %.1f deg
", deg);
            }
        }
    } else {
        char cmd = Serial.read();
        switch (cmd) {
            case 'S': case 's': 
                for(int i=0; i<3; i++) joints[i].enabled = false; 
                manual_test_mode = false; 
                stopAllMotorsRaw();
                Serial.println("!!! E-STOP !!!"); 
                break;
            case 'R': case 'r': 
                manual_test_mode = false;
                for(int i=0; i<3; i++) { if(JOINT_ALLOWED[i]) joints[i].reset(); }
                Serial.println("Re-enabling all joints...");
                break;
            case '?':
                Serial.println("--- STATUS ---");
                for(int i=0; i<3; i++) {
                    Serial.printf("  J%d: cur=%+.1f tgt=%+.1f pid=%.0f en=%d
", 
                        i, jointStepsToDeg(i, joints[i].current_pos, joints[i].continuous_pos),
                        (i==0) ? m3ContStepsToOutputDeg(joints[i].continuous_target) : jointStepsToDeg(i, joints[i].target_pos, 0),
                        joints[i].pid_output, joints[i].enabled);
                }
                break;
            // ... (Other cases like T, F, V could be added back, simplified here for length)
        }
    }
    while (Serial.available()) Serial.read();
}
