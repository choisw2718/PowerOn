#include "WifiSession.h"

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <string.h>

namespace wifi {
namespace {

constexpr uint16_t kTcpPort = 5000;
constexpr uint16_t kDiscoveryPort = 5001;
constexpr uint32_t kWiFiRetryMs = 5000;
constexpr uint32_t kSocketRetryMs = 5000;
// WiFiNINA utility/wifi_spi.h wl_tcp_state values.
constexpr uint8_t kTcpListenState = 1;
constexpr uint8_t kTcpEstablishedState = 4;
constexpr uint8_t kLineCapacity = 64;
constexpr uint8_t kMaxBytesPerLoop = 64;

WiFiServer server(kTcpPort);
WiFiUDP discovery;
WiFiClient controller;
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
char line[kLineCapacity] = {};
uint8_t lineLength = 0;
bool discardUntilEol = false;
const char *wifiSsid = nullptr;
const char *wifiPassword = nullptr;

void resetLineParser() { lineLength = 0; discardUntilEol = false; }

void pollController(CommandHandler executeCommand) {
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

void closeController(SessionHandler stopSession) {
  stopSession(true);
  controller.stop();
  hasController = false;
  resetLineParser();
}

bool serviceWiFi(SessionHandler stopSession) {
  if (!wifiModuleAvailable) {
    stopSession(false);
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
          Serial.print(F("READY WIFI=")); Serial.print(wifiSsid);
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
  if (hasController) closeController(stopSession);
  else stopSession(false);
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
    Serial.print(F("Connecting to Wi-Fi ")); Serial.println(wifiSsid);
    const int result = WiFi.begin(wifiSsid, wifiPassword);
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

} // namespace

void begin(const char *ssid, const char *password, SessionHandler stopSession) {
  wifiSsid = ssid;
  wifiPassword = password;
  if (strcmp(wifiPassword, "REPLACE_WITH_PRIVATE_PASSWORD") == 0 ||
      strlen(wifiPassword) < 8 || strlen(wifiPassword) > 63) {
    Serial.println(F("ERR set a private 8-63 character Wi-Fi password"));
    return;
  }
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("ERR NINA module missing; motor stays disabled"));
    return;
  }
  wifiModuleAvailable = true;
  lastWiFiAttemptMs = millis() - kWiFiRetryMs;
  serviceWiFi(stopSession);
}

bool service(CommandHandler executeCommand, SessionHandler stopSession, StatusHandler printStatus) {
  if (!serviceWiFi(stopSession)) return false;
  if (discoveryStarted) pollDiscovery();
  if (hasController && !controller.connected()) closeController(stopSession);
  // WiFiNINA may return the active socket again from accept(). Do not accept
  // while a controller owns it, or the BUSY path would close that controller.
  if (!hasController) {
    WiFiClient candidate = server.accept();
    if (candidate) {
      controller = candidate;
      hasController = true;
      resetLineParser();
      stopSession(true);
      controller.println(F("READY PowerOn Nano 33 IoT"));
      printStatus(controller);
    }
  }
  if (hasController) pollController(executeCommand);
  return true;
}

Print *controllerOutput() { return hasController ? &controller : nullptr; }

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

} // namespace wifi
