#ifndef WIRE_H
#define WIRE_H
#include <stdint.h>
class WireMock {
public:
    void begin(int sda, int scl) {}
    void setClock(uint32_t freq) {}
    void beginTransmission(uint8_t addr) {}
    void endTransmission() {}
    void write(uint8_t val) {}
    int available() { return 0; }
    uint8_t read() { return 0; }
    void requestFrom(uint8_t addr, int len) {}
};
extern WireMock Wire;
#endif
