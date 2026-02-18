#include <iostream>
#include <cassert>
#include <cstring>
#include "mocks/HardwareSerial.h"
#include "../include/Types.h"

HardwareSerial ROSSerial(1);

bool tryReadPacket_test(CommandPacketB& cmd) {
    for (int i = 0; i < 32 && ROSSerial.available() > 0; i++) {
        if (ROSSerial.peek() == 'S') break;
        ROSSerial.read();
    }
    if (ROSSerial.available() < (int)sizeof(CommandPacketB)) return false;
    ROSSerial.readBytes((char*)&cmd, sizeof(CommandPacketB));
    if (cmd.header[0] == 'S' && cmd.header[1] == 'T' && cmd.footer == 0x0A) return true;
    while (ROSSerial.available()) ROSSerial.read();
    return false;
}

void test_packet_reading() {
    std::cout << "Testing packet reading..." << std::endl;
    
    CommandPacketB cmd_send;
    cmd_send.header[0] = 'S';
    cmd_send.header[1] = 'T';
    cmd_send.motor_cmd[0] = 100;
    cmd_send.motor_cmd[1] = 200;
    cmd_send.motor_cmd[2] = 300;
    cmd_send.footer = 0x0A;
    
    ROSSerial.clear();
    ROSSerial.push((uint8_t*)&cmd_send, sizeof(CommandPacketB));
    
    CommandPacketB cmd_recv;
    bool ok = tryReadPacket_test(cmd_recv);
    assert(ok == true);
    assert(cmd_recv.motor_cmd[0] == 100);
    assert(cmd_recv.motor_cmd[1] == 200);
    assert(cmd_recv.motor_cmd[2] == 300);
    
    ROSSerial.clear();
    uint8_t garbage[] = {0x00, 0xFF, 'A', 'B'};
    ROSSerial.push(garbage, 4);
    ROSSerial.push((uint8_t*)&cmd_send, sizeof(CommandPacketB));
    
    ok = tryReadPacket_test(cmd_recv);
    assert(ok == true);
    assert(cmd_recv.motor_cmd[0] == 100);
    
    ROSSerial.clear();
    cmd_send.footer = 'X';
    ROSSerial.push((uint8_t*)&cmd_send, sizeof(CommandPacketB));
    ok = tryReadPacket_test(cmd_recv);
    assert(ok == false);
    assert(ROSSerial.available() == 0);
    
    std::cout << "  Passed!" << std::endl;
}

int main() {
    test_packet_reading();
    std::cout << "All Comms tests passed!" << std::endl;
    return 0;
}
