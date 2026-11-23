#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stdint.h>
class Print;
constexpr uint8_t A0 = 14, A1 = 15, A2 = 16;
constexpr uint8_t INPUT = 0, AR_DEFAULT = 0;
uint32_t millis();
void analogReference(uint8_t);
void analogReadResolution(int);
void pinMode(uint8_t, uint8_t);
int analogRead(uint8_t);
#endif
