#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// MUST match esp_coms.hpp on the RPi exactly — same size, same order.
struct __attribute__((packed)) CommandPacketB {
    char    header[2];            // 'S', 'T'
    int32_t motor_cmd[3];         // encoder steps: [M3, wrist, gripper]
    char    footer;               // '
};                                // Total: 15 bytes

struct __attribute__((packed)) FeedbackPacketB {
    char    header[2]      = {'F', 'B'};
    int32_t motor_pos[3]   = {0, 0, 0}; // encoder steps
    char    footer         = '\n';
};                                // Total: 15 bytes

#endif
