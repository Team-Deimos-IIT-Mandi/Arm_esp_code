#include "../include/Hardware.h"

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void hardwareSetup() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);

    for (int i = 0; i < 3; i++) {
        pinMode(DIR_PINS[i], OUTPUT);
        ledcAttach(PWM_PINS[i], LEDC_FREQ, LEDC_RES);
        ledcWrite(PWM_PINS[i], 0);
    }

    led.begin();
    led.setBrightness(40);

    if (!i2cHealthCheck()) i2cReset();
}

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(I2C_MUX_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delayMicroseconds(5);
}

void i2cReset() {
    Wire.end();
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    }
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);
    Serial.println("I2C RESET");
}

bool i2cHealthCheck() {
    Wire.beginTransmission(I2C_MUX_ADDR);
    return (Wire.endTransmission() == 0);
}

int readEncoderRaw(int joint) {
    tcaSelect(MUX_CHANNELS[joint]);
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte  = Wire.read();
        return (highByte << 8) | lowByte;
    }
    return -1;
}

int readEncoder(int joint) {
    int raw = readEncoderRaw(joint);
    if (raw == -1) return -1;
    int corrected = raw - HOME_OFFSETS[joint];
    while (corrected < 0)    corrected += 4096;
    while (corrected >= 4096) corrected -= 4096;
    return corrected;
}

void setMotor(int joint, int pwm_val, bool enabled) {
    if (!enabled) {
        ledcWrite(PWM_PINS[joint], 0);
        return;
    }
    pwm_val *= MOTOR_DIR_SIGN[joint];
    int speed = abs(pwm_val);
    speed = constrain(speed, 0, 255);
    if (speed < 15) speed = 0;
    digitalWrite(DIR_PINS[joint], (pwm_val >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[joint], speed);
}

void setMotorM3(int pwm_val, bool enabled) {
    if (!enabled) {
        ledcWrite(PWM_PINS[0], 0);
        return;
    }
    pwm_val *= M3_PID_SIGN * MOTOR_DIR_SIGN[0];
    int speed = abs(pwm_val);
    speed = constrain(speed, 0, 255);
    if (speed < 15) speed = 0;
    digitalWrite(DIR_PINS[0], (pwm_val >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[0], speed);
}

void driveMotorRaw(int pin_index, int pwm_signed) {
    int speed = abs(pwm_signed);
    speed = constrain(speed, 0, 255);
    digitalWrite(DIR_PINS[pin_index], (pwm_signed >= 0) ? HIGH : LOW);
    ledcWrite(PWM_PINS[pin_index], speed);
}

void stopAllMotorsRaw() {
    for (int i = 0; i < 3; i++) {
        ledcWrite(PWM_PINS[i], 0);
    }
}

void setLed(uint32_t colour) {
    led.setPixelColor(0, colour);
    led.show();
}
