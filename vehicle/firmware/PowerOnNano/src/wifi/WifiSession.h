#pragma once

#include <Arduino.h>

namespace wifi {
using CommandHandler = bool (*)(char *, Print &);
using SessionHandler = void (*)(bool);
using StatusHandler = void (*)(Print &);

void begin(const char *ssid, const char *password, SessionHandler stopSession);
bool service(CommandHandler executeCommand, SessionHandler stopSession, StatusHandler printStatus);
Print *controllerOutput();
void printNetwork(Print &out);
} // namespace wifi
