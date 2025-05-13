#include <Arduino.h>
#include "line_sensor.h"
#include "wifi_logic.h"
#include "distance_sensor.h"
#include <Average.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#define Wire Wire1  // Tell all libraries to use the secondary I2C bus
#include <Motoron.h>

MotoronI2C mc(0x10);

#define BUTTON_PIN 50
#define DEBOUNCE_TIME 25

int last_steady_state = LOW;       // the previous steady state from the input pin
int last_flickerable_state = LOW;  // the previous flickerable state from the input pin
int current_state;                 // the current reading from the input pin
bool robot_enabled = true;

unsigned long last_debounce_time = 0;  // the last time the output pin was toggled

DistanceSensor sensor(A0);

const int BASE_SPEED = 150; // change if needed

void setup() {
  Serial.begin(115200);
  while (!Serial);

  setupWiFi();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Wire.begin();
  mc.reinitialize();    // Bytes: 0x96 0x74
  mc.disableCrc();      // Bytes: 0x8B 0x04 0x7B 0x43
  mc.clearResetFlag();  // Bytes: 0xA9 0x00 0x04
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);
  mc.setMaxAcceleration(2, 140);
  mc.setMaxDeceleration(2, 300);

  // give the line a moment to settle
  delay(50);

  // read the real button state once, and use that
  int initState = digitalRead(BUTTON_PIN);
  last_steady_state = initState;
  last_flickerable_state = initState;
  last_debounce_time = millis();
}

void loop() {
  if (robot_enabled == true) {
    uint16_t values[9];
    //int position = qtr.readLine(values); // 0–8000

    // Debugging line sensor:
    //Serial.print("line: ");
    //Serial.println(position);

    sensor.update();
    Serial.println(sensor.getMean());

    //float convexity = qtr.getLineConvexity(sensorValues);
    //Serial.println(convexity);

    if (millis() & 2048)
    {
      mc.setSpeed(1, 800);
      mc.setSpeed(2, 800);
    }
    else
    {
      mc.setSpeed(1, -800);
      mc.setSpeed(2, 800);
    }
  }

  bool stop = handleWiFi(); // UDP logic
  if (stop == true) {
    robot_enabled = false;
  }

  current_state = digitalRead(BUTTON_PIN);

  if (current_state != last_flickerable_state) {
    last_debounce_time = millis();
    last_flickerable_state = current_state;
  }

  if ((millis() - last_debounce_time) > DEBOUNCE_TIME) {
    if (last_steady_state == HIGH && current_state == LOW) {
      int foo = 1;
    } else if (last_steady_state == LOW && current_state == HIGH) {
        robot_enabled = !robot_enabled;
    }
    last_steady_state = current_state;
  }
}
