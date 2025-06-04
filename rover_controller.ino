#include <Arduino.h>
#include "wifi_logic.h"
#include "distance_sensor.h"
#include "QTRSensorArray.h"
#include <Average.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "pid_controller.h"

#include <Wire.h>
#include <Motoron.h>
MotoronI2C mc1(0x10); // on top
MotoronI2C mc2(0x0B); // at the bottom (since I removed the top to change its address)

#define BUTTON_PIN 23
#define DEBOUNCE_TIME 25
int last_steady_state = LOW;       // the previous steady state from the input pin
int last_flickerable_state = LOW;  // the previous flickerable state from the input pin
int current_state;                 // the current reading from the input pin
bool robot_enabled = true;
unsigned long last_debounce_time = 0;  // the last time the output pin was toggled

const uint8_t SENSOR_PINS_L[9] = {40, 41, 42, 43, 44, 45, 46, 47, 48};
const uint8_t LED_PIN_L = 38;
QTRSensorArray qtrL(SENSOR_PINS_L, LED_PIN_L);
const uint8_t SENSOR_PINS_R[9] = {24, 25, 26, 27, 28, 29, 30, 31, 32};
const uint8_t LED_PIN_R = 22;
QTRSensorArray qtrR(SENSOR_PINS_R, LED_PIN_R);

float pid_values[3] = {0.0, 0.0, 0.0};  // kp, ki, kd
bool pid_updated = false;              // set true when new values are received

// Motor speed constants from doubleqtr
const int16_t MOTOR_SPEED_MIN = 200;
const int16_t MOTOR_SPEED_MAX = 800;

// PID Controller with doubleqtr parameters
PIDController pid(1.10, 0.10, 0.05);
float last_pos = 0.0;

// Turn detection from doubleqtr
Average<uint16_t> ave_L(10);
Average<uint16_t> ave_R(10);
bool turning_left = false;
bool turning_right = false;

Average<float> ave_ldr(100);

Average<uint16_t> ave_posL(10);
Average<uint16_t> ave_posR(10);
bool junc_mode = false;
const bool junc_right = true;  // or false if junction is always left

// Helper functions from doubleqtr
uint16_t sum(bool* arr, size_t size) {
    uint16_t count = 0;
    for (size_t i = 0; i < size; i++) {
        if (arr[i]) count++;
    }
    return count;
}

void reverseArray(uint16_t* arr, size_t size) {
  for (size_t i = 0; i < size / 2; i++) {
    uint16_t temp = arr[i];
    arr[i] = arr[size - 1 - i];
    arr[size - 1 - i] = temp;
  }
}

float calculatePos(const uint16_t* norm1, const uint16_t* norm2) {
  float x = 0.4, y = 0.75;
  float S_m = 0.0, S_n = 0.0, weighted_sum = 0.0, M = 0.0;
  for (int i = 0; i < 9; i++) {
    S_m += norm1[i];
    weighted_sum += norm1[i] * (-(y/2.0) - (8-i)*x);
  }
  for (int i = 0; i < 9; i++) {
    S_n += norm2[i];
    weighted_sum += norm2[i] * ((y/2.0) + i*x);
  }
  M = S_m + S_n;
  return 1000 * (weighted_sum / M) / x;
}

float halfPos(const uint16_t* norm) {
    float sum = 0, total = 0;
    for (int i = 0; i < 9; i++) {
        sum += norm[i] * i;
        total += norm[i];
    }
    return sum / total;
}

float lineAverage(const uint16_t* norm)
{
    // Compute average of the 9 values (each in range 0–1000)
    uint32_t total = 0;
    for (uint8_t i = 0; i < 9; i++) {
        total += norm[i];
    }
    float average = total / (1000.0f * 9); // Normalised to 0–1
    return average;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  //setupWiFi();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  {
    Serial.println("Initializing controller 1 on Wire1 (I2C1)...");
    mc1.setBus(&Wire1);      // point Motoron at Wire1
    Wire1.begin();           // start secondary I2C
    mc1.reinitialize();
    mc1.disableCrc();
    mc1.clearResetFlag();
    // Use doubleqtr acceleration values
    mc1.setMaxAcceleration(3, 1000);
    mc1.setMaxDeceleration(3, 1000);
    mc1.setMaxAcceleration(2, 1000);
    mc1.setMaxDeceleration(2, 1000);

    Serial.println("Initializing controller 2 on Wire (I2C0)...");
    mc2.setBus(&Wire);       // point Motoron at Wire
    Wire.begin();            // start primary I2C
    mc2.reinitialize();
    mc2.disableCrc();
    mc2.clearResetFlag();
    // Use doubleqtr acceleration values
    mc2.setMaxAcceleration(3, 1000);
    mc2.setMaxDeceleration(3, 1000);
    mc2.setMaxAcceleration(2, 1000);
    mc2.setMaxDeceleration(2, 1000);

    Serial.println("Initialization complete");
  }

  {
    Serial.print("Calibrating L");
    delay(50);
    qtrL.begin();
    delay(700);
    qtrL.calibrate_time(3000);
    qtrL.printCalibration();

    Serial.print("Calibrating R");
    delay(50);
    qtrR.begin();
    delay(700);
    qtrR.calibrate_time(3000);
    qtrR.printCalibration();
  }

  {
    // read the real button state once, and use that
    int initState = digitalRead(BUTTON_PIN);
    last_steady_state = initState;
    last_flickerable_state = initState;
    last_debounce_time = millis();
  }

  delay(5000);
}

void setDrive(float left_speed, float right_speed) {
  // Constrain final motor speeds to MOTOR_SPEED_MIN/MAX
  left_speed = constrain(left_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  right_speed = constrain(right_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  int16_t left = (int16_t)round(left_speed * 5 / 11.9);
  int16_t right = (int16_t)round(right_speed * 5 / 11.9);

  mc1.setSpeed(2, -left);
  mc1.setSpeed(3, right);
}

void setDriveUnc(float left_speed, float right_speed) {
  int16_t left = (int16_t)round(left_speed);
  int16_t right = (int16_t)round(right_speed);

  mc1.setSpeed(2, -left);
  mc1.setSpeed(3, right);
}

void loop() {
  // --- State Tracking from doubleqtr ---
  static unsigned long lastLoopTime = 0;  // Loop timing
  static unsigned long lostLineStartTime = 0;  // When line loss started
  static bool lost_line = false;  // Lost line flag (line lost too long)
  unsigned long lostLineDuration = 0;  // Duration of line loss (updated each loop)

  const unsigned long LOST_LINE_TIMEOUT = 3000;  // Timeout for lost line
  const unsigned long LOOP_INTERVAL = 10;  // Run every 10ms (non-blocking)

  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = currentTime;

    // long ldr = analogRead(A1);
    // ave_ldr.push(ldr);
    // float mldr = ave_ldr.mean();
    // bool dark = (mldr < 100.0);
    // if (dark) {
    //   Serial.println("DARK DARK DARK DARK DARK DARK DARK");
    //   qtrL.calibrateDark();
    //   qtrR.calibrateDark();
    //   delay(10000);
    // }
    // else {
    //   qtrL.calibrateFast();
    //   qtrR.calibrateFast();
    // }

    // *******************
    // QTR sensor reading with doubleqtr logic
    qtrL.updateSensors();
    const uint16_t* backnormL = qtrL.getNormalised();
    qtrR.updateSensors();
    const uint16_t* backnormR = qtrR.getNormalised();

    uint16_t normL[9], normR[9];
    memcpy(normL, backnormL, 9 * sizeof(uint16_t));
    memcpy(normR, backnormR, 9 * sizeof(uint16_t));
    reverseArray(normL, 9);
    reverseArray(normR, 9);

    bool lineL[9], lineR[9];
    uint16_t line_threshold = 200;
    for (uint8_t i = 0; i < 9; i++) {
      lineL[i] = (normL[i] > line_threshold);
      lineR[i] = (normR[i] > line_threshold);
    }

    float pos;
    float threshold = 0.1;
    if (qtrL.isLineDetected(threshold) || qtrR.isLineDetected(threshold)) {
      pos = calculatePos(normL, normR);
      last_pos = pos;  // Update last known good pos
    } else {
      pos = last_pos;  // Use last valid pos if line is lost
    }

    // Turn detection logic from doubleqtr
    uint16_t sL = sum(lineL, 9);
    uint16_t sR = sum(lineR, 9);
    ave_L.push(sL);
    ave_R.push(sR);
    if (ave_R.mean() - sR > 4.0) {
      turning_right = true;
      turning_left = false;
      Serial.println("RIGHT HAND TURN");
      setDrive(0.0, 0.0);
    }
    else if (ave_L.mean() - sL > 4.0) {
      turning_right = false;
      turning_left = true;      
      Serial.println("LEFT HAND TURN");
      setDrive(0.0, 0.0);
    }

    // float posL = halfPos(normL);
    // float posR = halfPos(normR);
    // static unsigned long lastDiffTime = 0;  // Static to retain across loop calls
    // if (millis() - lastDiffTime >= 100) {
    //   ave_posL.push(posL);
    //   ave_posR.push(posR);
    //   // Check Y-junction activation (replace with your divergence logic)
    //   float diffL = posL - ave_posL.mean();
    //   float diffR = posR - ave_posR.mean();
    //   Serial.println(diffL);
    //   lastDiffTime = millis();
    // }
    //Serial.println(diffR);
    // if ((diffL < -1000.0) && (diffR > 1000.0)) {
    //   junc_mode = true;
    // }

    // // --- PID CONTROLLER ---
    // if (junc_mode) {
    //   pos += junc_right ? 1000.0 : -1000.0;
    // }

    // --- PID CONTROLLER from doubleqtr ---
    float correction = pid.compute(pos);
    const float baseSpeed = (MOTOR_SPEED_MIN + MOTOR_SPEED_MAX) / 2.0;  // 500
    float left_speed = baseSpeed + correction;
    float right_speed = baseSpeed - correction;

    // if (junc_mode) {
    //   setDrive(left_speed, right_speed);

    //   if ((junc_right && qtrR.isLineDetected(0.15)) ||
    //       (!junc_right && qtrL.isLineDetected(0.15))) {
    //       junc_mode = false;
    //       pid.reset();
    //   }
    // }
    if (turning_right) {
      if (qtrR.isLineDetected(0.15) && pos < 6500.0) {
        turning_right = false;
        pid.reset();
        // delay(0);
        // delay(100);
        delay(200);
      } else {
        setDriveUnc(800, -800);
      }
    }
    else if (turning_left) {
        if (qtrL.isLineDetected(0.15) && pos > -6500.0) {
          turning_left = false;
          pid.reset();
          // delay(0);
          // delay(100);
          delay(200);
        } else {
          setDriveUnc(-800, 800);
        }
    }
    else 
    {
      if (lost_line) {
        Serial.println("LOST LINE");
        setDriveUnc(800, -800);
      } 
      else {
        setDrive(left_speed, right_speed);
      }

      // // Lost line detection (only if not turning)
      // if (lost_line && qtrR.isLineDetected(0.15) && pos < 6500.0){
      //     // Reset lost line tracking when line detected
      //     lostLineStartTime = 0;
      //     lostLineDuration = 0;
      //     lost_line = false;
      // }
      // else if (!(qtrL.isLineDetected(threshold) || qtrR.isLineDetected(threshold))) {
      //     if (lostLineStartTime == 0) {
      //       lostLineStartTime = currentTime;  // Start timing
      //     }
      //     lostLineDuration = currentTime - lostLineStartTime;
      //     if (lostLineDuration >= LOST_LINE_TIMEOUT) {
      //       lost_line = true;  // Set the lost flag
      //       setDrive(0.0, 0.0);
      //       pid.reset();
      //       delay(200);
      //     }
      // } 
    }
  }

  {
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
    // bool stop = handleWiFi(); // UDP logic
    // if (stop == true) {
    //   robot_enabled = false;
    // }
    if (robot_enabled == false) {
      setDrive(0.0, 0.0);
      mc2.setSpeed(2, 0.0);
      mc2.setSpeed(3, 0.0);
      pid.reset();
    }
  }
}