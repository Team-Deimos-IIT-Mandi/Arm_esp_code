#include <ESP32Servo.h>
#include <AccelStepper.h>

// --- Pin Definitions ---
#define M1_PIN1 1
#define M1_PIN3 3 
#define M1_PIN2 2 
#define M1_PIN4 4

#define M2_PIN1 7
#define M2_PIN3 9
#define M2_PIN2 8 
#define M2_PIN4 10 

#define REAL_SERVO1_PIN 11
#define REAL_SERVO2_PIN 12

// --- Object Instantiation ---
AccelStepper stepper1(AccelStepper::HALF4WIRE, M1_PIN1, M1_PIN3, M1_PIN2, M1_PIN4);
AccelStepper stepper2(AccelStepper::HALF4WIRE, M2_PIN1, M2_PIN3, M2_PIN2, M2_PIN4);

Servo servo1;
Servo servo2;

// --- Settings ---
const int MAX_SPEED = 1000;
const int ACCELERATION = 500;
const int STEP_SCALE = 256; 

// --- Binary Communication Structs ---
// RPi sends this (19 bytes: 2 header + 16 data + 1 footer)
struct __attribute__((packed)) EspCommand {
  char     header[2];
  int32_t  servo_cmd[2];   // 4 bytes each = 8 bytes
  int32_t  stepper_cmd[2]; // 4 bytes each = 8 bytes
  char     footer;
};

// ESP32 sends this back (11 bytes: 2 header + 8 data + 1 footer)
struct __attribute__((packed)) EspFeedback {
  char     header[2]    = {'F', 'B'};
  int32_t  stepper_pos[2] = {0, 0};
  char     footer       = '\n';
};

EspCommand incomingCmd;
EspFeedback outgoingFeedback;

unsigned long lastFeedbackTime = 0;
const int FEEDBACK_INTERVAL = 50; // Send position data every 50ms

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Servo Setup 
  servo1.setPeriodHertz(50); 
  servo1.attach(REAL_SERVO1_PIN, 500, 2400);
  servo2.setPeriodHertz(50);
  servo2.attach(REAL_SERVO2_PIN, 500, 2400);

  // Stepper Setup
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(ACCELERATION);


}

void loop() {
  // --- 1. Read Incoming Binary Commands ---
  // Only read if a full packet (19 bytes) is waiting in the buffer
  if (Serial.available() >= sizeof(EspCommand)) {
    
    Serial.readBytes((char*)&incomingCmd, sizeof(EspCommand));

    // Validate header and footer
    if (incomingCmd.header[0] == 'S' && incomingCmd.header[1] == 'T' && incomingCmd.footer == '\n') {
      
      // Apply Constraints to Servos (0-61 degrees from your original code)
      int s1_final = constrain(incomingCmd.servo_cmd[0], 0, 61);
      int s2_final = constrain(incomingCmd.servo_cmd[1], 0, 61);
      
      // Apply Scaling to Steppers
      int m1_final = incomingCmd.stepper_cmd[0] * STEP_SCALE;
      int m2_final = incomingCmd.stepper_cmd[1] * STEP_SCALE;

      // Execute Moves
      servo1.write(s1_final);
      servo2.write(s2_final);
      stepper1.moveTo(m1_final);
      stepper2.moveTo(m2_final);

    } else {
      // If the packet is misaligned or corrupted, clear the buffer to resynchronize
      while(Serial.available() > 0) {
        Serial.read();
      }
    }
  }

  // --- 2. Send Periodic Binary Feedback ---
  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    lastFeedbackTime = millis();
    
    // Populate the struct with current stepper positions
    outgoingFeedback.stepper_pos[0] = stepper1.currentPosition();
    outgoingFeedback.stepper_pos[1] = stepper2.currentPosition();

    // Send the struct as raw bytes over USB
    Serial.write((uint8_t*)&outgoingFeedback, sizeof(outgoingFeedback));
  }

  // --- 3. Keep Motors Running ---
  stepper1.run();
  stepper2.run();
}