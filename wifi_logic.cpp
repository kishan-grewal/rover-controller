#include <Arduino.h>
#include "wifi_logic.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
static const char ssid[] = "test";
static const char password[] = "password";
// static const char ssid[] = "PhaseSpaceNetwork_2.4G";
// static const char password[] = "8igMacNet";
// 192 168 155 43
// 192 168 0 46
static IPAddress deviceIP(192, 168, 155, 43); // Replace with your assigned static IP
//static IPAddress deviceIP(192, 168, 0, 46); // Replace with your assigned static IP
static IPAddress gateway(192, 168, 155, 1);   // May need to be updated
static IPAddress subnet(255, 255, 255, 0);

// UDP configuration
static const unsigned int listenPort = 55500; // Port for all robots
static WiFiUDP Udp;

// Buffer size for UDP packets
static const int UDP_PACKET_SIZE = 64; // More than enough for "Stop" + headers
static char packetBuffer[UDP_PACKET_SIZE + 1]; // +1 for null termination

// Connection status tracking
static bool wasConnected = false;
static unsigned long lastReconnectAttempt = 0;
static const unsigned long reconnectInterval = 30000; // 30 seconds between reconnection attempts

// Command detection
static const char* STOP_COMMAND = "Stop"; // Exact stop command (capital 'S')

// Forward declarations for internal use
static void connectToWiFi();
static void checkWiFiConnection();
static bool processUdpPackets();

// Initialize WiFi and UDP
void setupWiFi() {
  Serial.println("\nInitializing WiFi for UDP communication...");
  
  Serial.println("REMINDER: Ensure WiFi antenna is properly attached!");
  delay(1000); // Give a moment to read the reminder
  
  // Configure static IP - on Arduino Giga, this returns void, not bool
  WiFi.config(deviceIP, gateway, subnet);
  
  connectToWiFi();
  
  // Start listening for UDP packets
  Udp.begin(listenPort);
  Serial.print("UDP server started on port ");
  Serial.println(listenPort);
  Serial.println("Listening for 'Stop' kill switch command...");
}

// Main function to handle WiFi and UDP - returns true if stop command received
bool handleWiFi() {
  // Check WiFi connection status and reconnect if necessary
  checkWiFiConnection();
  
  // Only process UDP packets if connected
  if (WiFi.status() == WL_CONNECTED) {
    return processUdpPackets(); // Return true if stop command received
  }
  
  return false; // No stop command if not connected
}

// Connect to WiFi with timeout
static void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < 20000) { // 20 second timeout
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP().toString());
    wasConnected = true;
  } else {
    Serial.println("\nFailed to connect to WiFi network.");
    wasConnected = false;
  }
}

// Check and maintain WiFi connection
static void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wasConnected) {
      Serial.println("WiFi connection lost.");
      wasConnected = false;
    }
    
    // Try to reconnect at intervals
    unsigned long currentMillis = millis();
    if (currentMillis - lastReconnectAttempt > reconnectInterval) {
      lastReconnectAttempt = currentMillis;
      Serial.println("Attempting to reconnect to WiFi...");
      connectToWiFi();
    }
  } else if (!wasConnected) {
    Serial.println("WiFi reconnected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP().toString());
    wasConnected = true;
  }
}

// Process UDP packets, return true if stop command received
static bool processUdpPackets() {
  pid_updated = false;  // Default to false every loop

  int packetSize = Udp.parsePacket();
  if (packetSize) {
    memset(packetBuffer, 0, UDP_PACKET_SIZE + 1);
    int len = Udp.read(packetBuffer, UDP_PACKET_SIZE);
    if (len > 0) {
      packetBuffer[len] = '\0';
    }

    Serial.print("Received UDP packet: ");
    Serial.println(packetBuffer);

    // Check for "Stop"
    if (strcmp(packetBuffer, STOP_COMMAND) == 0) {
      Serial.println("STOP command received!");
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("Stop command received");
      Udp.endPacket();
      return true;
    }

    // Try to parse PID values
    float values[3];
    int count = 0;
    char* token = strtok(packetBuffer, ",");

    while (token != NULL && count < 3) {
      values[count] = atof(token);
      token = strtok(NULL, ",");
      count++;
    }

    if (count == 3 && token == NULL) {
      pid_values[0] = values[0]; // kp
      pid_values[1] = values[1]; // ki
      pid_values[2] = values[2]; // kd
      pid_updated = true;

      Serial.print("Updated PID values: ");
      Serial.print(pid_values[0]); Serial.print(", ");
      Serial.print(pid_values[1]); Serial.print(", ");
      Serial.println(pid_values[2]);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("PID values updated");
      Udp.endPacket();
    } else {
      Serial.println("Invalid PID format. Expected: kp,ki,kd");
    }
  }

  return false; // not a stop command
}