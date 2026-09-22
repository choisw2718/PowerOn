#include <Arduino.h>
#include <ESP8266WiFi.h>

/*
 * ESP8266 ESP-01 Wi-Fi bridge for the PowerOn single-rear-motor vehicle.
 *
 * Laptop TCP client -> ESP-01 access point -> UART -> Arduino Uno.
 * The Uno remains responsible for motor safety, command validation, and its
 * 1.5 second watchdog. This bridge additionally sends IDLE whenever the TCP
 * controller disconnects.
 */

namespace
{

/* Change the password before operating near other people. Minimum 8 chars. */
const char kAccessPointSsid[] = "PowerOn-Car";
const char kAccessPointPassword[] = "poweron-car";

const IPAddress kAccessPointIp(192, 168, 4, 1);
const IPAddress kAccessPointGateway(192, 168, 4, 1);
const IPAddress kAccessPointSubnet(255, 255, 255, 0);
const uint16_t kTcpPort = 5000;
const uint32_t kUnoUartBaud = 38400UL;
const size_t kCommandCapacity = 64;

WiFiServer commandServer(kTcpPort);
WiFiClient commandClient;

char commandLine[kCommandCapacity];
size_t commandLength = 0U;
bool discardUntilEol = false;
bool clientWasConnected = false;

void sendUnoIdle()
{
  Serial.print(F("IDLE\n"));
  Serial.flush();
}

void resetNetworkParser()
{
  commandLength = 0U;
  discardUntilEol = false;
}

void closeControllerConnection()
{
  if (clientWasConnected)
  {
    sendUnoIdle();
  }

  if (commandClient)
  {
    commandClient.stop();
  }

  clientWasConnected = false;
  resetNetworkParser();
}

void acceptController()
{
  if (commandClient && commandClient.connected())
  {
    return;
  }

  if (clientWasConnected)
  {
    closeControllerConnection();
  }

  WiFiClient candidate = commandServer.accept();
  if (!candidate)
  {
    return;
  }

  commandClient = candidate;
  commandClient.setNoDelay(true);
  clientWasConnected = true;
  resetNetworkParser();

  /* A new controller always starts from a stopped, centered state. */
  sendUnoIdle();
  commandClient.println(F("READY PowerOn ESP-01 TCP bridge"));
  commandClient.print(F("AP="));
  commandClient.print(kAccessPointSsid);
  commandClient.print(F(" IP="));
  commandClient.print(kAccessPointIp);
  commandClient.print(F(" PORT="));
  commandClient.println(kTcpPort);
}

void forwardCommandToUno()
{
  if (!commandClient || !commandClient.connected())
  {
    return;
  }

  while (commandClient.available() > 0)
  {
    const int raw = commandClient.read();
    if (raw < 0)
    {
      break;
    }
    const char incoming = static_cast<char>(raw);

    if (discardUntilEol)
    {
      if (incoming == '\r' || incoming == '\n')
      {
        discardUntilEol = false;
      }
      continue;
    }

    if (incoming == '\r' || incoming == '\n')
    {
      if (commandLength == 0U)
      {
        continue;
      }

      Serial.write(reinterpret_cast<const uint8_t *>(commandLine), commandLength);
      Serial.write('\n');
      commandLength = 0U;
      continue;
    }

    if (incoming < 0x20 || incoming > 0x7e)
    {
      commandLength = 0U;
      discardUntilEol = true;
      commandClient.println(F("ERR non-ASCII command discarded"));
      continue;
    }

    if (commandLength < (kCommandCapacity - 1U))
    {
      commandLine[commandLength++] = incoming;
    }
    else
    {
      commandLength = 0U;
      discardUntilEol = true;
      commandClient.println(F("ERR line too long; discarded"));
    }
  }
}

void forwardUnoResponse()
{
  while (Serial.available() > 0)
  {
    const int incoming = Serial.read();
    if (incoming < 0)
    {
      break;
    }

    if (commandClient && commandClient.connected())
    {
      commandClient.write(static_cast<uint8_t>(incoming));
    }
  }
}

} // namespace

void setup()
{
  Serial.begin(kUnoUartBaud);
  delay(50);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPdisconnect(true);
  WiFi.softAPConfig(kAccessPointIp,
                    kAccessPointGateway,
                    kAccessPointSubnet);
  WiFi.softAP(kAccessPointSsid, kAccessPointPassword);

  commandServer.begin();
  commandServer.setNoDelay(true);

  /* Never inherit an earlier motor command after either board reboots. */
  sendUnoIdle();
}

void loop()
{
  acceptController();

  if (clientWasConnected &&
      (!commandClient || !commandClient.connected()))
  {
    closeControllerConnection();
  }

  forwardCommandToUno();
  forwardUnoResponse();
  delay(1);
}
