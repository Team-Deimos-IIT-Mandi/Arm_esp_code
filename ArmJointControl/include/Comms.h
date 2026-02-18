#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "Config.h"
#include "Types.h"
#include "Joint.h"
#include "Hardware.h"
#include "Utils.h"

extern HardwareSerial ROSSerial;
extern unsigned long last_valid_cmd_time;
extern bool comms_ok;
extern bool manual_test_mode;

void commsSetup();
void processROSCommand(Joint* joints);
void checkCommsWatchdog(Joint* joints);
void processSerialCommand(Joint* joints);
void updateLed(Joint* joints);

bool tryReadPacket(CommandPacketB& cmd);

#endif
