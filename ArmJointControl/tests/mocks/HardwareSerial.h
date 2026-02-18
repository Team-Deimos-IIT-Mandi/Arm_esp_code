#include <vector>
#include <stdint.h>
#include <cstring>

class HardwareSerial {
    std::vector<uint8_t> buffer;
    size_t head = 0;
public:
    HardwareSerial(int i) {}
    void begin(long baud, uint32_t config, int rx, int tx) {}
    int available() { return buffer.size() - head; }
    uint8_t read() { return available() > 0 ? buffer[head++] : 0; }
    uint8_t peek() { return available() > 0 ? buffer[head] : 0; }
    void readBytes(char* buf, size_t len) {
        for (size_t i = 0; i < len; i++) {
            buf[i] = read();
        }
    }
    void write(const uint8_t* buf, size_t len) {}
    
    // Test helper
    void push(const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; i++) {
            buffer.push_back(buf[i]);
        }
    }
    void clear() { buffer.clear(); head = 0; }
};
