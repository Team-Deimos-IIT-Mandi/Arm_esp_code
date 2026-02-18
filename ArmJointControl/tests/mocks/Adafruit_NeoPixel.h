#ifndef ADAFRUIT_NEOPIXEL_H
#define ADAFRUIT_NEOPIXEL_H
#include <stdint.h>

#define NEO_GRB 0
#define NEO_KHZ800 0

class Adafruit_NeoPixel {
public:
    Adafruit_NeoPixel(int count, int pin, int type) {}
    void begin() {}
    void setBrightness(int b) {}
    void setPixelColor(int i, uint32_t c) {}
    void show() {}
    uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return 0; }
};

#endif
