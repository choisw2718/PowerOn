#include <Arduino.h>
#include "src/ir_sensors/DistanceSensors.h"
#include "src/ir_wall_following/IrAutonomy.h"
#include "src/lidar_drive/DriveLease.h"
#include "src/motor/Motor.h"
#include "src/protocol/CommandRouter.h"
#include "src/steering/AckermannSteering.h"
#include "src/wifi/WifiSession.h"

#ifndef ARDUINO_SAMD_NANO_33_IOT
#error "Select Arduino Nano 33 IoT in the Arduino IDE."
#endif

#include "wifi_secrets.h" // Copy wifi_secrets.example.h and set your own password.

void setup() {
  // Software cannot hold ENA LOW while the Nano is reset or unpowered.
  analogWriteResolution(8);
  protocol::begin();
  motor::begin();
  steering::begin();
  beginDistanceSensors();
  Serial.begin(115200); // USB diagnostics; do not wait for a PC.
  wifi::begin(WIFI_SSID, WIFI_PASSWORD, protocol::stopSession);
}

void loop() {
  lidar_drive::service(millis());
  motor::service();
  if (!wifi::service(protocol::executeCommand, protocol::stopSession, protocol::printStatus)) return;
  ir_autonomy::service(wifi::controllerOutput());
  lidar_drive::service(millis());
  motor::service();
}
