#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_logic.h"

// WiFi credentials
const char ssid[] = "test";
const char password[] = "password";

// Assign unique IPs manually (match with your network settings)
const IPAddress deviceIP(192, 168, 155, 43); // Change this for the second Arduino
// const IPAddress peerIP(192, 168, 243, 22); // The other Arduino's IP

// const unsigned int sendPort = 2000;       // Port to send data
const unsigned int listenPort = 3000;         // Port to listen for incoming data

WiFiUDP Udp;
char incomingPacket[50]; // Buffer for received data

void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  delay(2000); // Give time for serial and WiFi chip to initialise
  WiFi.disconnect(); // Reset the WiFi module
  delay(1000);

  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 15000; // 15 seconds max

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("My IP is: ");
    Serial.println(WiFi.localIP());

    // Start UDP
    Udp.begin(listenPort);
    Serial.print("Listening on port ");
    Serial.println(listenPort);
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

bool handleWiFi() {
  static unsigned long lastCheckTime = 0;
  const unsigned long interval = 200; // 200 ms

  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= interval) {
    lastCheckTime = currentTime;

    // --- Receiving Data ---
    int packetSize = Udp.parsePacket();

    if (packetSize) {
      int len = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);
      if (len > 0) incomingPacket[len] = '\0';

      Serial.print("Received message: ");
      Serial.println(incomingPacket);

      if (strstr(incomingPacket, "msg=stop") != nullptr) {
        Serial.println("Parsed: msg=stop");
        return true;
      }
      else {
        Serial.println("Failed to parse message.");
      }
    }
  }
  return false;
}