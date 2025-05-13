#include <SPI.h>
#include <WiFi.h>       // GIGA R1 Wi-Fi lib
#include <WiFiUdp.h>

// Wi-Fi credentials
const char ssid[]     = "test";
const char password[] = "password";

// Static IP settings (match your hotspot’s network)
const IPAddress deviceIP(192, 168, 155, 43);
const IPAddress gateway( 192, 168, 155, 1);
const IPAddress subnet(  255, 255, 255, 0);

const unsigned int listenPort = 3000;

WiFiUDP Udp;
char incomingPacket[50];

void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // 1) tear down any old connection
  WiFi.disconnect();
  delay(500);

  // 2) configure static IP (no return value to check)
  WiFi.config(deviceIP, gateway, subnet);

  // 3) begin joining
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  const unsigned long timeout = 1000; // 1 second is too short
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeout) {
    Serial.print('.');
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connected to Wi-Fi");
    Serial.print("  IP:      "); Serial.println(WiFi.localIP());
    Serial.print("  Gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.print("  Subnet:  "); Serial.println(WiFi.subnetMask());

    // 4) restart UDP on a clean socket
    Udp.stop();
    Udp.begin(listenPort);
    Serial.print("Listening on UDP port ");
    Serial.println(listenPort);
  }
  else {
    Serial.println();
    Serial.println("Failed to connect to Wi-Fi");
  }
}

bool handleWiFi() {
  static unsigned long lastCheck = 0;
  const unsigned long interval = 200;
  unsigned long now = millis();
  if (now - lastCheck < interval) return false;
  lastCheck = now;

  // if we ever drop out, reconnect
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost, reconnecting...");
    setupWiFi();
    return false;
  }

  // check for incoming UDP packets
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) incomingPacket[len] = '\0';

    Serial.print("Received: ");
    Serial.println(incomingPacket);
    if (strstr(incomingPacket, "msg=stop")) {
      Serial.println("Parsed: msg=stop");
      return true;
    }
    else {
      Serial.println("Could not parse message");
    }
  }

  return false;
}
