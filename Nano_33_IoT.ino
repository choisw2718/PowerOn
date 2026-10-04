#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
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
// Direct 3.3 V drive must be verified against the driver's 3.2 V HIGH minimum.
// Rewire the four signals to the driver's R inputs before using this sketch.
constexpr char kMotorChannel = 'R';
constexpr uint8_t kMotorPwmPin = 9;       // D9 -> driver pin 6 R_PWM
constexpr uint8_t kMotorIn1Pin = 2;       // D2 -> driver pin 4 R_IN1
constexpr uint8_t kMotorIn2Pin = 4;       // D4 -> driver pin 7 R_IN2
constexpr uint8_t kMotorEnablePin = 3;   // D3 -> driver pin 8 R_ENABLE (LOW active)
constexpr uint8_t kLeftServoPin = 5;      // D5 -> servo signal, former channel L
constexpr uint8_t kRightServoPin = 6;     // D6 -> servo signal, former channel R

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
constexpr uint16_t kDiscoveryPort = 5001;
constexpr uint32_t kWiFiRetryMs = 5000;
constexpr uint32_t kSocketRetryMs = 5000;
// WiFiNINA utility/wifi_spi.h wl_tcp_state values.
constexpr uint8_t kTcpListenState = 1;
constexpr uint8_t kTcpEstablishedState = 4;
constexpr uint32_t kDirectionPauseMs = 50;
constexpr uint8_t kLineCapacity = 64;
constexpr uint8_t kMaxBytesPerLoop = 64;
constexpr float kEpsilon = 0.001f;
constexpr float kDegToRad = 0.0174532925f;
constexpr float kRadToDeg = 57.2957795f;

WiFiServer server(kTcpPort);
WiFiUDP discovery;
WiFiClient controller;
Servo leftServo;
Servo rightServo;
bool hasController = false;
bool wifiModuleAvailable = false;
bool wifiWasConnected = false;
bool wifiServerStarted = false;
bool discoveryStarted = false;
uint32_t lastWiFiAttemptMs = 0;
uint32_t lastSocketAttemptMs = 0;
uint32_t discoveryPackets = 0;
uint32_t discoveryRequests = 0;
uint32_t discoveryReplies = 0;
uint32_t discoverySendErrors = 0;
bool pingLedOn = false;
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
char line[kLineCapacity] = {};
uint8_t lineLength = 0;
bool discardUntilEol = false;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float absoluteFloat(float value) { return value < 0.0f ? -value : value; }

void disableMotor() {
  // Direct connection: HIGH disables the active-LOW driver ENABLE.
  digitalWrite(kMotorEnablePin, HIGH);
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
  digitalWrite(kMotorEnablePin, HIGH);
  setMotorDirection(motorForward);
  analogWrite(kMotorPwmPin, static_cast<int>(duty * 255.0f / 100.0f + 0.5f));
  digitalWrite(kMotorEnablePin, LOW);
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
  out.print(F("STATUS channel=")); out.print(kMotorChannel);
  out.print(F(" duty_cmd=")); out.print(targetDutyPercent, 1);
  out.print(F("% duty_applied=")); out.print(appliedDutyPercent, 1);
  out.print(F("% enabled=")); out.print(motorEnabled ? 1 : 0);
  out.print(F(" steer_center=")); out.print(steeringCenterDeg, 1);
  out.print(F("deg servo_us_l=")); out.print(leftServoUs);
  out.print(F(" servo_us_r=")); out.println(rightServoUs);
}

void printNetwork(Print &out) {
  out.print(F("NETWORK wifi="));
  out.print(WiFi.status() == WL_CONNECTED ? F("connected") : F("disconnected"));
  out.print(F(" ip=")); out.print(WiFi.localIP());
  out.print(F(" rssi_dbm="));
  if (WiFi.status() == WL_CONNECTED) out.print(WiFi.RSSI());
  else out.print(F("NA"));
  out.print(F(" tcp_ready=")); out.print(wifiServerStarted ? 1 : 0);
  out.print(F(" tcp_state=")); out.print(server.status());
  out.print(F(" udp_ready=")); out.print(discoveryStarted ? 1 : 0);
  out.print(F(" udp_packets=")); out.print(discoveryPackets);
  out.print(F(" udp_requests=")); out.print(discoveryRequests);
  out.print(F(" udp_replies=")); out.print(discoveryReplies);
  out.print(F(" udp_errors=")); out.println(discoverySendErrors);
}

void printHelp(Print &out) {
  out.println(F("Nano 33 IoT car; TCP line commands, ASCII + newline"));
  out.println(F("DRIVE <nominal V> <center deg> | VOLTAGE <nominal V>"));
  out.println(F("MOTOR <-100..100> | STEER <deg> | CENTER | STOP | IDLE"));
  out.println(F("STATUS | NETWORK | PING | HELP"));
  out.println(F("DRIVE applies the inherited +/-7 V starting floor."));
  out.println(F("VOLTAGE is open-loop PWM, not measured motor voltage."));
  out.println(F("Motor holds the last command until STOP, IDLE, or disconnect."));
}

bool parseFloat(const char *text, float &result) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const double value = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(value)) return false;
  result = static_cast<float>(value);
  return isfinite(result);
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
  if (a || b || extra) return false;
  if (strcmp(verb, "PING") == 0) {
    pingLedOn = !pingLedOn;
    digitalWrite(LED_BUILTIN, pingLedOn ? HIGH : LOW);
    out.print(F("PONG LED="));
    out.println(pingLedOn ? F("ON") : F("OFF"));
    return true;
  }
  if (strcmp(verb, "STOP") == 0) { stopMotor(); out.println(F("OK STOP")); return true; }
  if (strcmp(verb, "CENTER") == 0) { applySteering(0.0f); out.println(F("OK CENTER")); return true; }
  if (strcmp(verb, "IDLE") == 0) { stopMotor(); applySteering(0.0f); out.println(F("OK IDLE")); return true; }
  if (strcmp(verb, "STATUS") == 0) { printStatus(out); return true; }
  if (strcmp(verb, "NETWORK") == 0) { printNetwork(out); return true; }
  if (strcmp(verb, "HELP") == 0 || strcmp(verb, "?") == 0) { printHelp(out); return true; }
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

bool serviceWiFi() {
  if (!wifiModuleAvailable) {
    stopMotor();
    return false;
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      lastSocketAttemptMs = millis() - kSocketRetryMs;
    }
    if ((!wifiServerStarted || !discoveryStarted) &&
        millis() - lastSocketAttemptMs >= kSocketRetryMs) {
      lastSocketAttemptMs = millis();
      if (!wifiServerStarted) {
        server.begin();
        const uint8_t state = server.status();
        Serial.print(F("TCP start state=")); Serial.println(state);
        // WiFiServer::operator bool() only confirms socket allocation, while
        // startServer() does not expose its result. Read the NINA TCP state.
        wifiServerStarted = static_cast<bool>(server) &&
                            (state == kTcpListenState ||
                             state == kTcpEstablishedState);
        if (wifiServerStarted) {
          Serial.print(F("READY WIFI=")); Serial.print(WIFI_SSID);
          Serial.print(F(" IP=")); Serial.print(WiFi.localIP());
          Serial.print(F(" PORT=")); Serial.println(kTcpPort);
        } else {
          server.end();
          Serial.println(F("ERR TCP server not listening; retrying in 5 s"));
        }
      }
      if (!discoveryStarted) {
        discoveryStarted = discovery.begin(kDiscoveryPort) != 0;
        if (discoveryStarted) Serial.println(F("READY UDP discovery PORT=5001"));
        else Serial.println(F("ERR UDP discovery socket unavailable; retrying"));
      }
    }
    return wifiServerStarted;
  }
  if (hasController) closeController();
  else stopMotor();
  if (discoveryStarted) {
    discovery.stop();
    discoveryStarted = false;
  }
  if (wifiServerStarted) {
    server.end();
    wifiServerStarted = false;
  }
  if (wifiWasConnected) {
    Serial.println(F("WIFI DISCONNECTED; motor stopped"));
    wifiWasConnected = false;
  }
  if (millis() - lastWiFiAttemptMs >= kWiFiRetryMs) {
    lastWiFiAttemptMs = millis();
    Serial.print(F("Connecting to Wi-Fi ")); Serial.println(WIFI_SSID);
    const int result = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (result != WL_CONNECTED) {
      Serial.print(F("WIFI connect result=")); Serial.println(result);
    }
  }
  return false;
}

void pollDiscovery() {
  const int packetSize = discovery.parsePacket();
  if (packetSize <= 0) return;
  ++discoveryPackets;
  char request[32];
  const int length = discovery.read(request, sizeof(request) - 1);
  if (length < 0) return;
  request[length] = '\0';
  while (discovery.available() > 0) discovery.read();
  if (strcmp(request, "POWERON_DISCOVER") != 0) return;
  ++discoveryRequests;
  if (discovery.beginPacket(discovery.remoteIP(), discovery.remotePort())) {
    discovery.print(F("POWERON_NANO "));
    discovery.print(kTcpPort);
    if (discovery.endPacket()) { ++discoveryReplies; return; }
  }
  ++discoverySendErrors;
}

void configureOutputs() {
  // Software cannot hold ENABLE HIGH while the Nano is reset or unpowered.
  analogWriteResolution(8);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(kMotorEnablePin, HIGH);
  pinMode(kMotorEnablePin, OUTPUT);
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
  wifiModuleAvailable = true;
  lastWiFiAttemptMs = millis() - kWiFiRetryMs;
  serviceWiFi();
}

void loop() {
  if (!serviceWiFi()) return;
  if (discoveryStarted) pollDiscovery();
  if (hasController && !controller.connected()) closeController();
  // WiFiNINA may return the active socket again from accept(). Do not accept
  // while a controller owns it, or the BUSY path would close that controller.
  if (!hasController) {
    WiFiClient candidate = server.accept();
    if (candidate) {
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
