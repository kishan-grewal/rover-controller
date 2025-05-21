#include <Arduino.h>
#include "wifi_logic.h"
#include "distance_sensor.h"
#include "QTRSensorArray.h"
#include <Average.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
//#define Wire Wire1  // Tell all libraries to use the secondary I2C bus
#include <Motoron.h>

Average<float> ave_ldr(100);
Average<float> ave_pos(100);

MotoronI2C mc1(0x10);
MotoronI2C mc2(0x0B);

#define BUTTON_PIN 50
#define DEBOUNCE_TIME 25

int last_steady_state = LOW;       // the previous steady state from the input pin
int last_flickerable_state = LOW;  // the previous flickerable state from the input pin
int current_state;                 // the current reading from the input pin
bool robot_enabled = false;

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
  
  // --- Controller 1 on secondary bus (Wire1) ---
  Serial.println("Initializing controller 1 on Wire1 (I2C1)...");
  mc1.setBus(&Wire1);      // point Motoron at Wire1
  Wire1.begin();           // start secondary I2C
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc1.setMaxAcceleration(3, 280);
  mc1.setMaxDeceleration(3, 600);
  mc1.setMaxAcceleration(2, 280);
  mc1.setMaxDeceleration(2, 600);

  // --- Controller 2 on primary bus (Wire) ---
  Serial.println("Initializing controller 2 on Wire (I2C0)...");
  mc2.setBus(&Wire);       // point Motoron at Wire
  Wire.begin();            // start primary I2C
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.setMaxAcceleration(3, 280);
  mc2.setMaxDeceleration(3, 600);
  mc2.setMaxAcceleration(2, 280);
  mc2.setMaxDeceleration(2, 600);

  Serial.println("Initialization complete");

  // give the line a moment to settle
  delay(50);

  Serial.println("Calibration Started");
  qtr.begin();
  delay(700);

  qtr.calibrate();
  qtr.printCalibration();

  //delay(12000);

  delay(6000);

  // read the real button state once, and use that
  int initState = digitalRead(BUTTON_PIN);
  last_steady_state = initState;
  last_flickerable_state = initState;
  last_debounce_time = millis();
}

void loop() {
  sensor.update();
  long ldr = analogRead(A1);
  float ldr_alpha = 0.1;
  float filtered_ldr = ldr_alpha * ldr + (1 - ldr_alpha) * ldr;
  ave_ldr.push(filtered_ldr);

  static unsigned long lastCheck = 0;
  const unsigned long interval = 500;
  unsigned long now = millis();

  static uint16_t pos = 4000; // keep previous position if not updated
  static uint16_t raw[9];
  pos = qtr.readLineBlack(raw);
  ave_pos.push(pos);

  if (now - lastCheck > interval) {
      lastCheck = now;
      
      // Serial.print("Dist: ");
      // Serial.println(sensor.getMean());

      for (uint8_t i = 0; i < 9; i++) {
          Serial.print(raw[i]);
          Serial.write(',');
      }

      float average_pos = ave_pos.mean();
      Serial.print("M: ");
      Serial.print(average_pos);
      Serial.print(" - P: ");
      Serial.print(pos);
      Serial.print(" = ");
      Serial.println(ave_pos.mean() - pos);

      // // 0.1 or 0.01
      // if (qtr.isLineDetected(0.10)) {
      //   Serial.println("line detected");
      // }
      // else {
      //   Serial.println("no line detected");
      // }

      // Serial.print("LDR: ");
      // Serial.println(ave_ldr.mean());
  }

  now = millis();
  unsigned long t = now % 20000;  // Loop every 20 seconds

  if (t < 5000) {
    // 0–5s: Forward
    mc2.setSpeed(2, 1200);
    mc2.setSpeed(3, 1200 * -1);
  } else if (t < 10000) {
    // 5–10s: Backward
    mc2.setSpeed(2, -1200);
    mc2.setSpeed(3, -1200 * -1);
  } else if (t < 15000) {
    // 10–15s: Forward-Left
    mc2.setSpeed(2, 600); 
    mc2.setSpeed(3, 1200 * -1); 
  } else {
    // 15–20s: Forward-Right
    mc2.setSpeed(2, 1200);
    mc2.setSpeed(3, 600 * -1);
  }

  mc1.setSpeed(2, 600);
  mc1.setSpeed(3, 600);

  if (robot_enabled == false) {
    mc1.setSpeed(2, 0);
    mc1.setSpeed(3, 0);
    mc2.setSpeed(2, 0);
    mc2.setSpeed(3, 0);
  }

  bool stop = handleWiFi(); // UDP logic
  if (stop == true) {
    robot_enabled = false;
  }

  if (abs(ave_pos.mean() - pos) > 1500.0) {
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
