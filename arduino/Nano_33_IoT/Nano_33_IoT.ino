#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef ARDUINO_SAMD_NANO_33_IOT
#error "Select Arduino Nano 33 IoT in the Arduino IDE."
#endif

#include "wifi_secrets.h" // Copy wifi_secrets.example.h and set your own password.

namespace {

// These are the PRINTED Nano pin labels, not physical header positions.
constexpr uint8_t kMotorPwmPin = 9;       // D9 -> buffer -> driver pin 5 L_PWM
constexpr uint8_t kMotorIn1Pin = 2;       // D2 -> buffer -> driver pin 1 L_IN1
constexpr uint8_t kMotorIn2Pin = 4;       // D4 -> buffer -> driver pin 2 L_IN2
constexpr uint8_t kEnableSwitchPin = 3;  // D3 -> NPN -> driver pin 3 L_ENABLE
constexpr uint8_t kLeftServoPin = 5;      // D5 -> former harness channel L
constexpr uint8_t kRightServoPin = 6;     // D6 -> former harness channel R

// These values came from the Uno vehicle and must be checked on the actual car.
constexpr float kMotorSupplyVoltage = 12.0f;
constexpr float kMinimumReliableMotorVoltage = 7.0f;
constexpr bool kForwardIn1High = true;
constexpr bool kServoChannelsCrossed = true;
constexpr float kWheelbaseM = 0.215f;
constexpr float kFrontTrackM = 0.188f;
constexpr float kMinWheelSteerDeg = -45.0f;
constexpr float kMaxWheelSteerDeg = 55.0f;
constexpr float kServoUsPerDegree = 10.0f;
constexpr float kLeftServoDirection = 1.0f;
constexpr float kRightServoDirection = 1.0f;
constexpr uint16_t kLeftServoMinUs = 948;
constexpr uint16_t kLeftServoNeutralUs = 1398;
constexpr uint16_t kLeftServoMaxUs = 1948;
constexpr uint16_t kRightServoMinUs = 952;
constexpr uint16_t kRightServoNeutralUs = 1402;
constexpr uint16_t kRightServoMaxUs = 1952;

constexpr uint16_t kTcpPort = 5000;
constexpr uint32_t kDirectionPauseMs = 50;
constexpr uint32_t kDefaultTimeoutMs = 1500;
constexpr uint32_t kMinimumTimeoutMs = 200;
constexpr uint32_t kMaximumTimeoutMs = 5000;
constexpr uint8_t kLineCapacity = 64;
constexpr uint8_t kMaxBytesPerLoop = 64;
constexpr float kEpsilon = 0.001f;
constexpr float kDegToRad = 0.0174532925f;
constexpr float kRadToDeg = 57.2957795f;

WiFiServer server(kTcpPort);
WiFiClient controller;
Servo leftServo;
Servo rightServo;
bool hasController = false;
bool motorEnabled = false;
bool motorForward = true;
bool directionPauseActive = false;
float targetDutyPercent = 0.0f;
float appliedDutyPercent = 0.0f;
float steeringCenterDeg = 0.0f;
float frontLeftSteerDeg = 0.0f;
float frontRightSteerDeg = 0.0f;
uint16_t leftServoUs = kLeftServoNeutralUs;
uint16_t rightServoUs = kRightServoNeutralUs;
uint32_t directionPauseStartedMs = 0;
uint32_t lastMotorCommandMs = 0;
uint32_t commandTimeoutMs = kDefaultTimeoutMs;
uint32_t timeoutStopCount = 0;
char line[kLineCapacity] = {};
uint8_t lineLength = 0;
bool discardUntilEol = false;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float absoluteFloat(float value) { return value < 0.0f ? -value : value; }

void disableMotor() {
  // The NPN releases the driver's externally pulled-up, active-LOW ENABLE.
  digitalWrite(kEnableSwitchPin, LOW);
  motorEnabled = false;
  analogWrite(kMotorPwmPin, 0);
  digitalWrite(kMotorIn1Pin, LOW);
  digitalWrite(kMotorIn2Pin, LOW);
  appliedDutyPercent = 0.0f;
}

void stopMotor() {
  targetDutyPercent = 0.0f;
  directionPauseActive = false;
  disableMotor();
}

void setMotorDirection(bool forward) {
  const bool in1High = forward ? kForwardIn1High : !kForwardIn1High;
  digitalWrite(kMotorIn1Pin, in1High ? HIGH : LOW);
  digitalWrite(kMotorIn2Pin, in1High ? LOW : HIGH);
}

void commandMotorPercent(float requestedPercent) {
  const float limited = clampFloat(requestedPercent, -100.0f, 100.0f);
  lastMotorCommandMs = millis();
  if (absoluteFloat(limited) <= kEpsilon) {
    stopMotor();
    return;
  }
  targetDutyPercent = limited;
  const bool requestedForward = limited > 0.0f;
  if (requestedForward != motorForward) {
    disableMotor();
    motorForward = requestedForward;
    directionPauseStartedMs = millis();
    directionPauseActive = true;
  }
}

float commandReliableVoltage(float requestedVoltage) {
  float voltage = clampFloat(requestedVoltage, -kMotorSupplyVoltage,
                             kMotorSupplyVoltage);
  if (absoluteFloat(voltage) > kEpsilon &&
      absoluteFloat(voltage) < kMinimumReliableMotorVoltage) {
    voltage = voltage > 0.0f ? kMinimumReliableMotorVoltage
                             : -kMinimumReliableMotorVoltage;
  }
  commandMotorPercent(voltage * 100.0f / kMotorSupplyVoltage);
  return voltage;
}

void serviceMotor() {
  if (absoluteFloat(targetDutyPercent) <= kEpsilon) return;
  if (directionPauseActive) {
    if (millis() - directionPauseStartedMs < kDirectionPauseMs) return;
    directionPauseActive = false;
  }
  const float duty = absoluteFloat(targetDutyPercent);
  if (motorEnabled && absoluteFloat(appliedDutyPercent - duty) <= kEpsilon) return;
  digitalWrite(kEnableSwitchPin, LOW);
  setMotorDirection(motorForward);
  analogWrite(kMotorPwmPin, static_cast<int>(duty * 255.0f / 100.0f + 0.5f));
  digitalWrite(kEnableSwitchPin, HIGH);
  motorEnabled = true;
  appliedDutyPercent = duty;
}

float centerSteeringForInnerWheel(float innerMagnitudeDeg) {
  const float inner = innerMagnitudeDeg * kDegToRad;
  const float radius = kFrontTrackM * 0.5f + kWheelbaseM / tanf(inner);
  return atanf(kWheelbaseM / radius) * kRadToDeg;
}

void setServosFromWheelAngles(float leftDeg, float rightDeg) {
  leftDeg = clampFloat(leftDeg, kMinWheelSteerDeg, kMaxWheelSteerDeg);
  rightDeg = clampFloat(rightDeg, kMinWheelSteerDeg, kMaxWheelSteerDeg);
  const float channelL = kServoChannelsCrossed ? rightDeg : leftDeg;
  const float channelR = kServoChannelsCrossed ? leftDeg : rightDeg;
  leftServoUs = static_cast<uint16_t>(
      clampFloat(kLeftServoNeutralUs + channelL * kLeftServoDirection *
                     kServoUsPerDegree,
                 kLeftServoMinUs, kLeftServoMaxUs) + 0.5f);
  rightServoUs = static_cast<uint16_t>(
      clampFloat(kRightServoNeutralUs + channelR * kRightServoDirection *
                     kServoUsPerDegree,
                 kRightServoMinUs, kRightServoMaxUs) + 0.5f);
  leftServo.writeMicroseconds(leftServoUs);
  rightServo.writeMicroseconds(rightServoUs);
}

void applySteering(float centerDeg) {
  const float minimum =
      -centerSteeringForInnerWheel(absoluteFloat(kMinWheelSteerDeg));
  const float maximum = centerSteeringForInnerWheel(kMaxWheelSteerDeg);
  steeringCenterDeg = clampFloat(centerDeg, minimum, maximum);
  if (absoluteFloat(steeringCenterDeg) <= kEpsilon) {
    steeringCenterDeg = frontLeftSteerDeg = frontRightSteerDeg = 0.0f;
  } else {
    const float signedRadius = kWheelbaseM /
                               tanf(steeringCenterDeg * kDegToRad);
    const float radius = absoluteFloat(signedRadius);
    const float halfTrack = kFrontTrackM * 0.5f;
    const float inner = atanf(kWheelbaseM / (radius - halfTrack)) * kRadToDeg;
    const float outer = atanf(kWheelbaseM / (radius + halfTrack)) * kRadToDeg;
    frontLeftSteerDeg = signedRadius > 0.0f ? inner : -outer;
    frontRightSteerDeg = signedRadius > 0.0f ? outer : -inner;
  }
  setServosFromWheelAngles(frontLeftSteerDeg, frontRightSteerDeg);
}

void printStatus(Print &out) {
  out.print(F("STATUS duty_cmd=")); out.print(targetDutyPercent, 1);
  out.print(F("% duty_applied=")); out.print(appliedDutyPercent, 1);
  out.print(F("% enabled=")); out.print(motorEnabled ? 1 : 0);
  out.print(F(" steer_center=")); out.print(steeringCenterDeg, 1);
  out.print(F("deg servo_us_l=")); out.print(leftServoUs);
  out.print(F(" servo_us_r=")); out.print(rightServoUs);
  out.print(F(" timeout_ms=")); out.print(commandTimeoutMs);
  out.print(F(" timeout_stops=")); out.println(timeoutStopCount);
}

void printHelp(Print &out) {
  out.println(F("Nano 33 IoT car; TCP line commands, ASCII + newline"));
  out.println(F("DRIVE <nominal V> <center deg> | VOLTAGE <nominal V>"));
  out.println(F("MOTOR <-100..100> | STEER <deg> | CENTER | STOP | IDLE"));
  out.println(F("STATUS | HELP | TIMEOUT <200..5000> | KEEPALIVE"));
  out.println(F("DRIVE applies the inherited +/-7 V starting floor."));
  out.println(F("VOLTAGE is open-loop PWM, not measured motor voltage."));
}

bool parseFloat(const char *text, float &result) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const double value = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(value)) return false;
  result = static_cast<float>(value);
  return isfinite(result);
}

bool parseTimeout(const char *text, uint32_t &result) {
  if (text == nullptr || *text == '\0' || *text == '-') return false;
  char *end = nullptr;
  const unsigned long value = strtoul(text, &end, 10);
  if (end == text || *end != '\0' || value < kMinimumTimeoutMs ||
      value > kMaximumTimeoutMs) return false;
  result = static_cast<uint32_t>(value);
  return true;
}

bool executeCommand(char *command, Print &out) {
  char *save = nullptr;
  char *verb = strtok_r(command, " \t", &save);
  if (verb == nullptr) return true;
  for (char *p = verb; *p; ++p)
    *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  char *a = strtok_r(nullptr, " \t", &save);
  char *b = strtok_r(nullptr, " \t", &save);
  char *extra = strtok_r(nullptr, " \t", &save);
  float value = 0.0f, steer = 0.0f;
  if (strcmp(verb, "DRIVE") == 0) {
    if (!parseFloat(a, value) || !parseFloat(b, steer) || extra) return false;
    applySteering(steer);
    out.print(F("OK DRIVE applied_nominal_V="));
    out.println(commandReliableVoltage(value), 2);
    return true;
  }
  if (strcmp(verb, "VOLTAGE") == 0 || strcmp(verb, "MOTOR") == 0 ||
      strcmp(verb, "STEER") == 0) {
    if (!parseFloat(a, value) || b) return false;
    if (strcmp(verb, "STEER") == 0) {
      applySteering(value);
    } else if (strcmp(verb, "VOLTAGE") == 0) {
      commandMotorPercent(clampFloat(value, -kMotorSupplyVoltage,
                                     kMotorSupplyVoltage) *
                          100.0f / kMotorSupplyVoltage);
    } else {
      commandMotorPercent(value);
    }
    out.println(F("OK"));
    return true;
  }
  if (strcmp(verb, "TIMEOUT") == 0) {
    uint32_t requested = 0;
    if (b || !parseTimeout(a, requested)) return false;
    commandTimeoutMs = requested;
    out.println(F("OK TIMEOUT"));
    return true;
  }
  if (a || b || extra) return false;
  if (strcmp(verb, "STOP") == 0) { stopMotor(); out.println(F("OK STOP")); return true; }
  if (strcmp(verb, "CENTER") == 0) { applySteering(0.0f); out.println(F("OK CENTER")); return true; }
  if (strcmp(verb, "IDLE") == 0) { stopMotor(); applySteering(0.0f); out.println(F("OK IDLE")); return true; }
  if (strcmp(verb, "STATUS") == 0) { printStatus(out); return true; }
  if (strcmp(verb, "HELP") == 0 || strcmp(verb, "?") == 0) { printHelp(out); return true; }
  if (strcmp(verb, "KEEPALIVE") == 0) {
    if (absoluteFloat(targetDutyPercent) > kEpsilon) lastMotorCommandMs = millis();
    return true;
  }
  return false;
}

void resetLineParser() { lineLength = 0; discardUntilEol = false; }

void pollController() {
  uint8_t consumed = 0;
  while (controller.available() > 0 && consumed++ < kMaxBytesPerLoop) {
    const int raw = controller.read();
    if (raw < 0) break;
    const char ch = static_cast<char>(raw);
    if (discardUntilEol) {
      if (ch == '\r' || ch == '\n') discardUntilEol = false;
      continue;
    }
    if (ch == '\r' || ch == '\n') {
      if (lineLength == 0) continue;
      line[lineLength] = '\0';
      if (!executeCommand(line, controller))
        controller.println(F("ERR bad command; send HELP"));
      lineLength = 0;
    } else if (ch < 0x20 || ch > 0x7e || lineLength >= kLineCapacity - 1) {
      lineLength = 0;
      discardUntilEol = true;
      controller.println(F("ERR invalid or long line discarded"));
    } else {
      line[lineLength++] = ch;
    }
  }
}

void closeController() {
  stopMotor();
  applySteering(0.0f);
  controller.stop();
  hasController = false;
  resetLineParser();
}

void configureOutputs() {
  // Passive pull-up and pull-down components in WIRING.ko.md cover reset time.
  analogWriteResolution(8);
  digitalWrite(kEnableSwitchPin, LOW);
  pinMode(kEnableSwitchPin, OUTPUT);
  digitalWrite(kMotorIn1Pin, LOW); pinMode(kMotorIn1Pin, OUTPUT);
  digitalWrite(kMotorIn2Pin, LOW); pinMode(kMotorIn2Pin, OUTPUT);
  digitalWrite(kMotorPwmPin, LOW); pinMode(kMotorPwmPin, OUTPUT);
  stopMotor();
  leftServo.attach(kLeftServoPin);
  rightServo.attach(kRightServoPin);
  applySteering(0.0f);
}

} // namespace

void setup() {
  configureOutputs();
  Serial.begin(115200); // USB diagnostics; do not wait for a PC.
  if (strcmp(WIFI_PASSWORD, "REPLACE_WITH_PRIVATE_PASSWORD") == 0 ||
      strlen(WIFI_PASSWORD) < 8 || strlen(WIFI_PASSWORD) > 63) {
    Serial.println(F("ERR set a private 8-63 character Wi-Fi password"));
    return;
  }
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("ERR NINA module missing; motor stays disabled"));
    return;
  }
  WiFi.config(IPAddress(192, 168, 4, 1));
  if (WiFi.beginAP(WIFI_SSID, WIFI_PASSWORD) != WL_AP_LISTENING) {
    Serial.println(F("ERR access point failed; motor stays disabled"));
    return;
  }
  server.begin();
  Serial.print(F("READY AP=")); Serial.print(WIFI_SSID);
  Serial.print(F(" IP=")); Serial.print(WiFi.localIP());
  Serial.print(F(" PORT=")); Serial.println(kTcpPort);
}

void loop() {
  if (absoluteFloat(targetDutyPercent) > kEpsilon &&
      millis() - lastMotorCommandMs >= commandTimeoutMs) {
    stopMotor();
    ++timeoutStopCount;
    if (hasController && controller.connected()) controller.println(F("TIMEOUT STOP"));
  }
  const int wifiStatus = WiFi.status();
  if (wifiStatus != WL_AP_LISTENING && wifiStatus != WL_AP_CONNECTED) {
    if (hasController) closeController();
    else stopMotor();
    return;
  }
  if (hasController && !controller.connected()) closeController();
  WiFiClient candidate = server.accept();
  if (candidate) {
    if (hasController) {
      candidate.println(F("BUSY one controller allowed"));
      candidate.stop();
    } else {
      controller = candidate;
      hasController = true;
      resetLineParser();
      stopMotor();
      applySteering(0.0f);
      controller.println(F("READY PowerOn Nano 33 IoT"));
      printStatus(controller);
    }
  }
  if (hasController) pollController();
  serviceMotor();
}
