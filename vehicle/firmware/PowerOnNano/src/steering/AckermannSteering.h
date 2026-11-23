#pragma once

#include <Arduino.h>

namespace steering {
void begin();
void apply(float centerDeg);
float center();
void printStatus(Print &out);
} // namespace steering
