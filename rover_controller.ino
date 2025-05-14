#include <Arduino.h>
#include "wifi_logic.h"
#include "distance_sensor.h"
#include "QTRSensorArray.h"
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

const uint8_t SENSOR_PINS[9] = {24, 25, 26, 27, 28, 29, 30, 31, 32};
const uint8_t LED_PIN = 22;
QTRSensorArray qtr(SENSOR_PINS, LED_PIN);

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

  Serial.println("start calibration");
  qtr.begin();
  delay(700);

  qtr.calibrate();
  qtr.printCalibration();

  delay(12000);
  Serial.println("\nr0,r1,r2,r3,r4,r5,r6,r7,r8,pos");
  delay(5000);

  // read the real button state once, and use that
  int initState = digitalRead(BUTTON_PIN);
  last_steady_state = initState;
  last_flickerable_state = initState;
  last_debounce_time = millis();
}

void loop() {
  if (robot_enabled == true) {
    sensor.update();

    static unsigned long lastCheck = 0;
    const unsigned long interval = 500;
    unsigned long now = millis();

    static uint16_t pos = 4000; // keep previous position if not updated
    static uint16_t raw[9];
    pos = qtr.readLineBlack(raw);

    if (now - lastCheck > interval) {
        lastCheck = now;
        
        Serial.print("Dist: ");
        Serial.println(sensor.getMean());

        // for (uint8_t i = 0; i < 9; i++) {
        //     Serial.print(raw[i]);
        //     Serial.write(',');
        // }

        Serial.print("Pos: ");
        Serial.println(pos);
    }

    now = millis();
    unsigned long t = now % 20000;  // Loop every 20 seconds

    if (t < 5000) {
      // 0–5s: Forward
      mc.setSpeed(1, 600);
      mc.setSpeed(2, 600);
    } else if (t < 10000) {
      // 5–10s: Backward
      mc.setSpeed(1, -600);
      mc.setSpeed(2, -600);
    } else if (t < 15000) {
      // 10–15s: Forward-Left
      mc.setSpeed(1, 300); // Slow left
      mc.setSpeed(2, 600); // Normal right
    } else {
      // 15–20s: Forward-Right
      mc.setSpeed(1, 600); // Normal left
      mc.setSpeed(2, 300); // Slow right
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
