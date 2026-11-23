#pragma once

#include <Arduino.h>

namespace protocol {
void begin();
void stopSession(bool centerSteering);
void printStatus(Print &out);
bool executeCommand(char *command, Print &out);
} // namespace protocol
