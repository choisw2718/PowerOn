// PowerOn Nano 33 IoT Wi-Fi and HTTP port 8000 diagnostic server.
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>

// The sketch works without credentials: it scans nearby access points.
// Add a local arduino_secrets.h to also test association and DHCP.
#if defined(__has_include)
#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#define POWERON_HAS_WIFI_SECRETS 1
#endif
#endif

#ifndef POWERON_HAS_WIFI_SECRETS
#define POWERON_HAS_WIFI_SECRETS 0
#endif

namespace
{
constexpr unsigned long kSerialWaitMs = 5000UL;
constexpr unsigned long kScanIntervalMs = 15000UL;
constexpr unsigned long kConnectTimeoutMs = 20000UL;
constexpr unsigned long kHttpRequestTimeoutMs = 1000UL;
constexpr unsigned long kReconnectIntervalMs = 10000UL;
constexpr uint16_t kServerPort = 8000;

unsigned long lastScanMs = 0;

#if POWERON_HAS_WIFI_SECRETS
WiFiServer server(kServerPort);
bool serverStarted = false;
bool ledState = false;
unsigned long lastConnectAttemptMs = 0;
#endif

const char *wifiStatusName(int status)
{
  switch (status)
  {
  case WL_IDLE_STATUS:
    return "idle";
  case WL_NO_SSID_AVAIL:
    return "SSID not found";
  case WL_SCAN_COMPLETED:
    return "scan completed";
  case WL_CONNECTED:
    return "connected";
  case WL_CONNECT_FAILED:
    return "authentication/connect failed";
  case WL_CONNECTION_LOST:
    return "connection lost";
  case WL_DISCONNECTED:
    return "disconnected";
  case WL_NO_MODULE:
    return "NINA module missing";
  default:
    return "unknown";
  }
}

const char *encryptionName(uint8_t encryption)
{
  switch (encryption)
  {
  case ENC_TYPE_WEP:
    return "WEP";
  case ENC_TYPE_TKIP:
    return "WPA/TKIP";
  case ENC_TYPE_CCMP:
    return "WPA2/CCMP";
  case ENC_TYPE_NONE:
    return "open";
  case ENC_TYPE_AUTO:
    return "auto";
  default:
    return "unknown";
  }
}

void printMacAddress()
{
  byte mac[6];
  WiFi.macAddress(mac);

  Serial.print("MAC: ");
  for (int index = 5; index >= 0; --index)
  {
    if (mac[index] < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(mac[index], HEX);
    if (index > 0)
    {
      Serial.print(':');
    }
  }
  Serial.println();
}

void printCurrentConnection()
{
  Serial.println("\n[connected]");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  digitalWrite(LED_BUILTIN, HIGH);
#if POWERON_HAS_WIFI_SECRETS
  ledState = true;
#endif
}

void scanNetworks()
{
  Serial.println("\n[scan] Looking for nearby Wi-Fi networks...");
  const int networkCount = WiFi.scanNetworks();

  if (networkCount < 0)
  {
    Serial.println("Scan failed.");
    return;
  }

  if (networkCount == 0)
  {
    Serial.println("No networks found. Nano 33 IoT supports 2.4 GHz Wi-Fi only.");
    return;
  }

  Serial.print(networkCount);
  Serial.println(" network(s) found:");
  for (int index = 0; index < networkCount; ++index)
  {
    Serial.print(index + 1);
    Serial.print(". ");
    Serial.print(WiFi.SSID(index));
    Serial.print(" | ");
    Serial.print(WiFi.RSSI(index));
    Serial.print(" dBm | ");
    Serial.println(encryptionName(WiFi.encryptionType(index)));
  }

  lastScanMs = millis();
}

#if POWERON_HAS_WIFI_SECRETS
void startHttpServer()
{
  server.begin();
  serverStarted = true;

  Serial.print("HTTP test server: http://");
  Serial.print(WiFi.localIP());
  Serial.print(':');
  Serial.println(kServerPort);
  Serial.println("Endpoints: GET /ping, /led/on, /led/off");
}

void sendJsonResponse(WiFiClient &client, int statusCode,
                      const char *statusText, const String &body)
{
  client.print("HTTP/1.1 ");
  client.print(statusCode);
  client.print(' ');
  client.println(statusText);
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(body.length());
  client.println();
  client.print(body);
}

void handleHttpClient()
{
  WiFiClient client = server.available();
  if (!client)
  {
    return;
  }

  char requestLine[96] = {0};
  size_t length = 0;
  const unsigned long startedMs = millis();

  while (client.connected() &&
         millis() - startedMs < kHttpRequestTimeoutMs)
  {
    if (!client.available())
    {
      delay(1);
      continue;
    }

    const char character = static_cast<char>(client.read());
    if (character == '\n')
    {
      break;
    }
    if (character != '\r' && length < sizeof(requestLine) - 1U)
    {
      requestLine[length++] = character;
    }
  }

  requestLine[length] = '\0';
  Serial.print("[http] ");
  Serial.println(requestLine);

  bool knownRoute = true;
  if (strcmp(requestLine, "GET /led/on HTTP/1.1") == 0)
  {
    ledState = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (strcmp(requestLine, "GET /led/off HTTP/1.1") == 0)
  {
    ledState = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (strcmp(requestLine, "GET / HTTP/1.1") != 0 &&
           strcmp(requestLine, "GET /ping HTTP/1.1") != 0)
  {
    knownRoute = false;
  }

  if (!knownRoute)
  {
    sendJsonResponse(client, 404, "Not Found",
                     "{\"ok\":false,\"error\":\"not found\"}\n");
  }
  else
  {
    String body;
    body.reserve(112);
    body += "{\"ok\":true,\"device\":\"Arduino Nano 33 IoT\",\"port\":";
    body += kServerPort;
    body += ",\"rssi\":";
    body += WiFi.RSSI();
    body += ",\"led\":";
    body += ledState ? "true" : "false";
    body += "}\n";
    sendJsonResponse(client, 200, "OK", body);
  }

  delay(1);
  client.stop();
}

bool connectToConfiguredNetwork()
{
  lastConnectAttemptMs = millis();
  Serial.print("\n[connect] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.setTimeout(kConnectTimeoutMs);
  int status;
  if (WIFI_PASSWORD[0] == '\0')
  {
    status = WiFi.begin(WIFI_SSID);
  }
  else
  {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  Serial.print("Wi-Fi status: ");
  Serial.print(wifiStatusName(status));
  Serial.print(" (");
  Serial.print(status);
  Serial.println(')');

  if (status != WL_CONNECTED)
  {
    Serial.println("Connection failed. Check exact credentials and use 2.4 GHz WPA2-Personal/AES.");
    Serial.println("WPA3-only and enterprise authentication are not supported by this test.");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }

  printCurrentConnection();
  startHttpServer();
  return true;
}
#endif
} // namespace

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  const unsigned long serialStartedMs = millis();
  while (!Serial && millis() - serialStartedMs < kSerialWaitMs)
  {
  }

  Serial.println("\nPowerOn Nano 33 IoT Wi-Fi test");

  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("FAIL: WiFiNINA module was not detected.");
    while (true)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  const String ninaFirmware = WiFi.firmwareVersion();
  Serial.print("NINA firmware: ");
  Serial.println(ninaFirmware);
  Serial.print("WiFiNINA recommended firmware: ");
  Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  if (ninaFirmware < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println("WARNING: Update the NINA firmware before deeper troubleshooting.");
  }
  printMacAddress();
  scanNetworks();

#if POWERON_HAS_WIFI_SECRETS
  connectToConfiguredNetwork();
#else
  Serial.println("\nScan-only mode. Add arduino_secrets.h to test connection and DHCP.");
#endif
}

void loop()
{
#if POWERON_HAS_WIFI_SECRETS
  if (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, LOW);
    ledState = false;
    serverStarted = false;
    if (lastConnectAttemptMs == 0 ||
        millis() - lastConnectAttemptMs >= kReconnectIntervalMs)
    {
      connectToConfiguredNetwork();
    }
  }
  else if (!serverStarted)
  {
    startHttpServer();
  }

  handleHttpClient();
  delay(5);
#else
  if (millis() - lastScanMs >= kScanIntervalMs)
  {
    scanNetworks();
  }
  delay(50);
#endif
}
