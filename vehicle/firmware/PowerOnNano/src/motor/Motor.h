#pragma once

#include <Arduino.h>

namespace motor {
void begin();
void stop();
void commandPercent(float requestedPercent);
void commandVoltage(float requestedVoltage);
float commandReliableVoltage(float requestedVoltage);
void service();
void printStatus(Print &out);
} // namespace motor
