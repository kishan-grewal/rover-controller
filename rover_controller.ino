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

#define BUTTON_PIN 52
#define DEBOUNCE_TIME 25
int last_steady_state = LOW;       // the previous steady state from the input pin
int last_flickerable_state = LOW;  // the previous flickerable state from the input pin
int current_state;                 // the current reading from the input pin
bool robot_enabled = false;
unsigned long last_debounce_time = 0;  // the last time the output pin was toggled

Average<float> ave_ldr(100);
DistanceSensor sensor(A0);

const uint8_t SENSOR_PINS_L[9] = {40, 41, 42, 43, 44, 45, 46, 47, 48};
const uint8_t LED_PIN_L = 38;
QTRSensorArray qtrL(SENSOR_PINS_L, LED_PIN_L);
const uint8_t SENSOR_PINS_R[9] = {24, 25, 26, 27, 28, 29, 30, 31, 32};
const uint8_t LED_PIN_R = 22;
QTRSensorArray qtrR(SENSOR_PINS_R, LED_PIN_R);
uint16_t whiteAvg[9];
uint16_t blackAvg[9];

float pid_values[3] = {0.0, 0.0, 0.0};  // kp, ki, kd
bool pid_updated = false;              // set true when new values are received

enum State { FOLLOW, TURN_LEFT, TURN_RIGHT, TURN_AROUND, END };
enum Direction { NONE, LEFT, RIGHT };
Direction lastPath = NONE; // Start with NONE
State currentState = FOLLOW;
float turnBias = 0.0;

const float LINE_SPEED = 100.0;
const float TURN_ADJUST = 200.0;
PIDController pid_line(0.1, 0.0, 0.0);
const float TURN_SPEED = 400.0;

// Global variables
unsigned long turnStartTime = 0;
bool turnActive = false;
State activeTurnState = FOLLOW;

// Define durations for each turn (in milliseconds)
const unsigned long TURN_LEFT_DURATION = 4000;   
const unsigned long TURN_RIGHT_DURATION = 4000;   
const unsigned long TURN_AROUND_DURATION = 8000; 

const int16_t sMAX = 800;
const int16_t sMIN = 400;


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
    mc1.setMaxAcceleration(3, 280);
    mc1.setMaxDeceleration(3, 600);
    mc1.setMaxAcceleration(2, 280);
    mc1.setMaxDeceleration(2, 600);

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
  }

  {
    Serial.print("Calibrating L");
    delay(50);
    qtrL.begin();
    delay(700);
    qtrL.calibrate();
    qtrL.printCalibration();

    Serial.print("Calibrating R");
    delay(50);
    qtrR.begin();
    delay(700);
    qtrR.calibrate();
    qtrR.printCalibration();
  }

  {
    // read the real button state once, and use that
    int initState = digitalRead(BUTTON_PIN);
    last_steady_state = initState;
    last_flickerable_state = initState;
    last_debounce_time = millis();
  }
}

void setDrive(float left_speed, float right_speed) {
  int16_t left = (int16_t)round(left_speed);
  int16_t right = (int16_t)round(right_speed);
  
  // Print left and right every 500ms
  const unsigned long PRINT_INTERVAL = 500; // 500 ms
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    Serial.print("left: ");
    Serial.print(left);
    Serial.print("  right: ");
    Serial.println(right);
  }
  
  if (left > 0) {
    left = constrain(left, sMIN, sMAX);
  } else if (left == 0) {
    left = 0;
  } else {
    left = constrain(left, -sMAX, -sMIN);
  }

  if (right > 0) {
    right = constrain(right, sMIN, sMAX);
  } else if (right == 0) {
    right = 0;
  } else {
    right = constrain(right, -sMAX, -sMIN);
  }
  
  // Set speeds, remembering motor 2 runs opposite
  mc1.setSpeed(2, -left);
  mc1.setSpeed(3, right);
}

void applyDrive(float baseSpeed, float pidBias, float turnBias) {
  float left_speed = baseSpeed + pidBias + turnBias;
  float right_speed = baseSpeed - pidBias - turnBias;
  setDrive(left_speed, right_speed);
}

float calculatePos(const uint16_t* norm1, const uint16_t* norm2) {
      float x = 0.4;
      float y = 0.75;

      float S_m = 0.0;
      float S_n = 0.0;
      float weighted_sum = 0.0;
      float M = 0.0;
    // Left side (norm1)
    for (int i = 0; i < 9; i++) {
        S_m += norm1[i];
        float pos = -(y / 2.0) - (8 - i) * x;  // Position for norm1[i]
        weighted_sum += norm1[i] * pos;
    }
    // Right side (norm2)
    for (int i = 0; i < 9; i++) {
        S_n += norm2[i];
        float pos = (y / 2.0) + i * x;  // Position for norm2[i]
        weighted_sum += norm2[i] * pos;
    }
    M = S_m + S_n;
    float center_of_mass = weighted_sum / M;
    float pos_result = 1000 * center_of_mass / x;
    return pos_result;
}

void reverseArray(uint16_t* arr, size_t size) {
  for (size_t i = 0; i < size / 2; i++) {
    uint16_t temp = arr[i];
    arr[i] = arr[size - 1 - i];
    arr[size - 1 - i] = temp;
  }
}

float filtered_ldr = 0.0;

void loop() {
  if (pid_updated == true) {
    for (int i = 0; i < 3; i++) {
      Serial.print(pid_values[i]);
      Serial.print(",");
    }
    Serial.println();
  }

  sensor.update();

  long ldr = analogRead(A1);
  float ldr_alpha = 0.1;
  filtered_ldr = ldr_alpha * ldr + (1 - ldr_alpha) * filtered_ldr;
  ave_ldr.push(filtered_ldr);

  qtrL.updateSensors();
  const uint16_t* rawL = qtrL.getRaw();
  const uint16_t* backnormL = qtrL.getNormalised();
  qtrR.updateSensors();
  const uint16_t* rawR = qtrR.getRaw();
  const uint16_t* backnormR = qtrR.getNormalised();
  uint16_t normL[9];
  memcpy(normL, backnormL, 9 * sizeof(uint16_t)); // Copy into mutable buffer
  reverseArray(normL, 9); // Reverse in place
  uint16_t normR[9];
  memcpy(normR, backnormR, 9 * sizeof(uint16_t)); // Copy into mutable buffer
  reverseArray(normR, 9); // Reverse in place

  switch (currentState)
  {
    case FOLLOW:
      turnActive = false;
      activeTurnState = FOLLOW;
      
      // Only run PID and applyDrive every 20ms
      const unsigned long FOLLOW_PID_INTERVAL = 20;
      static unsigned long lastFollowPIDTime = 0;
      unsigned long currentTime = millis();
      if (currentTime - lastFollowPIDTime >= FOLLOW_PID_INTERVAL) {
          lastFollowPIDTime = currentTime;

          float pos = calculatePos(normL, normR);
          float pidBias = pid_line.compute(pos);
          applyDrive(LINE_SPEED, pidBias, turnBias);
      }

      // if (detectFinish()) {
      //     currentState = END;
      // } else if (detectJunction()) {
      //     if (lastPath == NONE) {
      //         lastPath = RIGHT;
      //         turnBias = TURN_ADJUST;
      //     } else if (lastPath == RIGHT) {
      //         lastPath = NONE;
      //         turnBias = 0.0;
      //     }
      // } else if (detectLeftTurn()) {
      //     currentState = TURN_LEFT;
      // } else if (detectRightTurn()) {
      //     currentState = TURN_RIGHT;
      // } else if (lineEnded()) {
      //     currentState = TURN_AROUND;
      // }
      break;

  //   case TURN_LEFT:
  //     if (!turnActive || activeTurnState != TURN_LEFT) {
  //       turnStartTime = millis();
  //       turnActive = true;
  //       activeTurnState = TURN_LEFT;
  //     }
  //     if (millis() - turnStartTime < TURN_LEFT_DURATION) {
  //       setDrive(-TURN_SPEED, TURN_SPEED);
  //     } else {
  //       setDrive(0.0, 0.0);
  //       turnActive = false;
  //       currentState = FOLLOW;
  //       lastPath = LEFT;
  //     }
  //     break;

  // case TURN_RIGHT:
  //   if (!turnActive || activeTurnState != TURN_RIGHT) {
  //     turnStartTime = millis();
  //     turnActive = true;
  //     activeTurnState = TURN_RIGHT;
  //   }
  //   if (millis() - turnStartTime < TURN_RIGHT_DURATION) {
  //     setDrive(TURN_SPEED, -TURN_SPEED);
  //   } else {
  //     setDrive(0.0, 0.0);
  //     turnActive = false;
  //     currentState = FOLLOW;
  //     lastPath = RIGHT;
  //   }
  //   break;

  // case TURN_AROUND:
  //   if (!turnActive || activeTurnState != TURN_AROUND) {
  //     turnStartTime = millis();
  //     turnActive = true;
  //     activeTurnState = TURN_AROUND;
  //   }
  //   if (millis() - turnStartTime < TURN_AROUND_DURATION) {
  //     setDrive(TURN_SPEED, -TURN_SPEED);
  //   } else {
  //     setDrive(0.0, 0.0);
  //     turnActive = false;
  //     currentState = FOLLOW;
  //   }
  //   break;

  //   case END:
  //     robot_enabled = false;
  //     break;
  }

  const unsigned long interval = 800;
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck > interval) {
      lastCheck = now;
      for (uint8_t i = 0; i < 9; i++) {
          Serial.print(normL[i] > 200.0);
          Serial.print(",");
      }
      for (uint8_t i = 0; i < 9; i++) {
          Serial.print(normR[i] > 200.0);
          Serial.print(",");
      }
      Serial.println();

      float pos = calculatePos(normL, normR);
      Serial.println(pos);
  }

  if (robot_enabled == false) {
    setDrive(0.0, 0.0);
    mc2.setSpeed(2, 0);
    mc2.setSpeed(3, 0);
  }

  {
    // bool stop = handleWiFi(); // UDP logic
    // if (stop == true) {
    //   robot_enabled = false;
    // }
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
}

