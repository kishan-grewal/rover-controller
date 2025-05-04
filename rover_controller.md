#include <Arduino.h>
#include "line_sensor.h"
#include "motor_driver.h"
#include "wifi_logic.h"
#include "pid_controller.h"

// Motor connected to IN1 = 22, IN2 = 23, ENA = 6
MotorDriver motor(22, 23, 6);

// PID controller: tune these values as needed
PIDController pid(0.3, 0.0, 1.5);

// Target is center of line sensor range
const int LINE_CENTER = 4000;
const int BASE_SPEED = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  setupLineSensor();     // Sets up and calibrates QTR sensor
  motor.begin();         // Set motor pins as output
  setupWiFi();           // Optional WiFi debug
}

void loop() {
  uint16_t values[9];
  int position = qtr.readLine(values);  // 0–8000
  int error = LINE_CENTER - position;
  float correction = pid.compute(error);

  // Print position for debug
  Serial.print("Line position: ");
  Serial.println(position);

  // Apply correction to motor speed (adjust to fit your platform)
  int motorSpeed = BASE_SPEED + correction;
  motorSpeed = constrain(motorSpeed, -255, 255);
  motor.setSpeed(motorSpeed);

  handleWiFi();  // optional, keep if you're using UDP

  delay(20);  // small delay for stability
}
