#include <Arduino.h>
#include "line_sensor.h"
#include "motor_driver.h"
#include "wifi_logic.h"
// #include "distance_sensor.h" // optional IR

// one motor (IN1, IN2, ENA)
MotorDriver motor(22, 23, 6);

// DistanceSensor distSensor; // if using Sharp IR

const int BASE_SPEED = 100; // change if needed

void setup() {
  Serial.begin(115200);
  while (!Serial);

  setupLineSensor(); // qtr init + calibrate
  motor.begin();     // pins out
  setupWiFi();       // udp
  // distSensor.begin(); // if using IR
}

void loop() {
  uint16_t values[9];
  int position = qtr.readLine(values); // 0–8000
  Serial.print("line: ");
  Serial.println(position);

  motor.setSpeed(BASE_SPEED); // constant forward

  // float d = distSensor.readDistance(); // mm
  // Serial.print("dist: ");
  // Serial.println(d);

  handleWiFi(); // udp
  delay(20);
}
