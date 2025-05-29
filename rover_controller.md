#include <Arduino.h>
#include "QTRSensorArray.h"
#include "pid_controller.h"
#include <Wire.h>
#include <Motoron.h>
#include <Average.h>

// QTR Sensor Setup
const uint8_t SENSOR_PINS_L[9] = {40, 41, 42, 43, 44, 45, 46, 47, 48};
const uint8_t LED_PIN_L = 38;
QTRSensorArray qtrL(SENSOR_PINS_L, LED_PIN_L);

const uint8_t SENSOR_PINS_R[9] = {24, 25, 26, 27, 28, 29, 30, 31, 32};
const uint8_t LED_PIN_R = 22;
QTRSensorArray qtrR(SENSOR_PINS_R, LED_PIN_R);

// Motor Setup
MotoronI2C mc1(0x10); // top controller
MotoronI2C mc2(0x0B); // bottom controller

// Motor speed constants
const int16_t MOTOR_SPEED_MIN = 200;
const int16_t MOTOR_SPEED_MAX = 800;

PIDController pid(1.10, 0.10, 0.05);
float last_pos = 0.0;

Average<uint16_t> ave_R(10);
bool turning = false;

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

void setDrive(float left_speed, float right_speed) {
  // Constrain final motor speeds to MOTOR_SPEED_MIN/MAX
  left_speed = constrain(left_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  right_speed = constrain(right_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  int16_t left = (int16_t)round(left_speed);
  int16_t right = (int16_t)round(right_speed);

  mc1.setSpeed(2, -left);
  mc1.setSpeed(3, right);
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

  // Motoron Initialization
  mc1.setBus(&Wire1);
  Wire1.begin();
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc1.setMaxAcceleration(3, 1000);
  mc1.setMaxDeceleration(3, 1000);
  mc1.setMaxAcceleration(2, 1000);
  mc1.setMaxDeceleration(2, 1000);

  mc2.setBus(&Wire);
  Wire.begin();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.setMaxAcceleration(3, 1000);
  mc2.setMaxDeceleration(3, 1000);
  mc2.setMaxAcceleration(2, 1000);
  mc2.setMaxDeceleration(2, 1000);

  // QTR Calibration
  Serial.println("Calibrating sensors...");
  qtrL.begin();
  delay(700);
  qtrL.calibrate();
  qtrL.printCalibration();

  qtrR.begin();
  delay(700);
  qtrR.calibrate();
  qtrR.printCalibration();

  delay(5000);
}

void loop() {
    // --- State Tracking ---
  static unsigned long lastLoopTime = 0;  // Loop timing
  static unsigned long lostLineStartTime = 0;  // When line loss started
  static bool lost_line = false;  // Lost line flag (line lost too long)
  unsigned long lostLineDuration = 0;  // Duration of line loss (updated each loop)
  static bool turning = false;  // Are we in a turn mode?

  const unsigned long LOST_LINE_TIMEOUT = 1000;  // Timeout for lost line
  const unsigned long LOOP_INTERVAL = 10;  // Run every 20ms (non-blocking)

  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = currentTime;

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

    // --- PID CONTROLLER ---
    // Use PID controller instead of power coefficient mapping

    float correction = pid.compute(pos);

    const float baseSpeed = (MOTOR_SPEED_MIN + MOTOR_SPEED_MAX) / 2.0;  // 500

    float left_speed = baseSpeed + correction;
    float right_speed = baseSpeed - correction;

    uint16_t s = sum(lineR, 9);
    ave_R.push(s);
    if (ave_R.mean() - s > 4.0) {
      turning = true;
      //Serial.println("RIGHT HAND TURN");
      setDrive(0.0, 0.0);
      delay(200);
    }

    if (turning) {
        if (qtrR.isLineDetected(0.15) && pos < 6500.0) {
            turning = false;  // Exit turn mode
            pid.reset();
            Serial.print("pos: ");
            Serial.print(pos);
            Serial.print(" avL: ");
            Serial.print(lineAverage(normL));
            Serial.print(" avR: ");
            Serial.print(lineAverage(normR));
            delay(200);
        } else {
            // Serial.println("TURNING TURNING TURNING TURNING TURNING TURNING TURNING TURNING TURNING");
            // Serial.print("pos: ");
            // Serial.println(pos);
            // Serial.print(" avL: ");
            // Serial.print(lineAverage(normL));
            // Serial.print(" avR: ");
            Serial.println(lineAverage(normR));
            mc1.setSpeed(2, -800);  // Turn in place
            mc1.setSpeed(3, -800);
        }
    } 
    else 
    {
      if (lost_line) {
          setDrive(0.0, 0.0);  // Stop the robot if truly lost
          Serial.println("!! ROBOT STOPPED: LINE LOST TOO LONG !!");
      } 
      else {
          setDrive(left_speed, right_speed);  // Normal line-following
      }


      // Lost line detection (only if not turning)
      if (!(qtrL.isLineDetected(threshold) || qtrR.isLineDetected(threshold))) {
          if (lostLineStartTime == 0) {
              lostLineStartTime = currentTime;  // Start timing
          }
          lostLineDuration = currentTime - lostLineStartTime;
          if (lostLineDuration >= LOST_LINE_TIMEOUT) {
              lost_line = true;  // Set the lost flag
          }
      } 
      else {
          // Reset lost line tracking when line detected
          lostLineStartTime = 0;
          lostLineDuration = 0;
          lost_line = false;
      }
    }

    // Serial.print("ave_R: ");
    // Serial.print(ave_R.mean());
    // Serial.print(" s: ");
    // Serial.print(s);
    // Serial.println();  // End the line

    // Serial.print("normL: ");
    // for (uint8_t i = 0; i < 9; i++) {
    //   Serial.print(normL[i]);
    //   Serial.print(" ");
    // }
    // Serial.print(" normR: ");
    // for (uint8_t i = 0; i < 9; i++) {
    //   Serial.print(normR[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();  // End the line
    // // Print lineL and lineR on the next line
    // Serial.print("lineL: ");
    // for (uint8_t i = 0; i < 9; i++) {
    //   Serial.print(lineL[i]);
    //   Serial.print(" ");
    // }
    // Serial.print(" lineR: ");
    // for (uint8_t i = 0; i < 9; i++) {
    //   Serial.print(lineR[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();  // End the line
  }
}