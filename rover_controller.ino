#include <Arduino.h>
#include "line_sensor.h"
#include "distance_sensor.h"
#include "wifi_logic.h"
#include "motor_driver.h"

// uint16_t sensorValues[9];

// Pin mapping for GIGA R1

// MotorDriver motor1(22, 23, 6);  // IN1, IN2, ENA

// PWM -255 to 255

void setup() {
  Serial.begin(115200);
  while (!Serial);

  setupLineSensor();       
  //initDistanceSensor();    
  setupWiFi();           
  // motor1.begin(); 
}

void loop() {
  loopLineSensor(); 
  //unsigned int distance = readDistance();                

  //Serial.print("Line: ");
  //Serial.print(linePosition);
  //Serial.print(" | Distance: ");
  //Serial.println(distance);

  handleWiFi();  

  // delay(1000);
  // motor1.setSpeed(-150); // reverse
  // delay(1000);
  // motor1.setSpeed(0); // stop
  // delay(1000);
  // motor1.setSpeed(150); // stop
}

// comment to show I pushed from pc and pulled on laptop


