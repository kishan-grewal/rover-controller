#include <Arduino.h>
#include "line_sensor.h"
#include "motor_driver.h"
#include "wifi_logic.h"
#include "distance_sensor.h"
#include <Average.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// one motor (IN1, IN2, ENA)
//MotorDriver motor(22, 23, 12);

DistanceSensor sensor(A0);

const int BASE_SPEED = 100; // change if needed

void setup() {
  Serial.begin(115200);
  while (!Serial);

  setupLineSensor(); // QTR init + calibrate
  //motor.begin();     // pins out
  setupWiFi();       // UDP
}

void loop() {
  uint16_t values[9];
  int position = qtr.readLine(values); // 0–8000

  // Debugging line sensor:
  Serial.print("line: ");
  Serial.println(position);

  //motor.setSpeed(BASE_SPEED); // constant forward

  Serial.print("distance: ");
  Serial.println(sensor.getMean());

  handleWiFi(); // UDP logic
  delay(200);
}
