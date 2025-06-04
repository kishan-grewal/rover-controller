#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc1(0x10);
MotoronI2C mc2(0x0B);

const int16_t MOTOR_SPEED_MIN = 200;
const int16_t MOTOR_SPEED_MAX = 800;
const unsigned long LOOP_INTERVAL = 10;

#define BUTTON_PIN 23
#define DEBOUNCE_TIME 25
int last_steady_state = LOW;
int last_flickerable_state = LOW;
int current_state;
bool robot_enabled = true;
unsigned long last_debounce_time = 0;

void setDrive(float left_speed, float right_speed) {
    left_speed = constrain(left_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    right_speed = constrain(right_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    int16_t left = (int16_t)round(left_speed * 5 / 10.905);
    int16_t right = (int16_t)round(right_speed * 5 / 10.905);
    mc1.setSpeed(2, -left);
    mc1.setSpeed(3, right);
}

void setDriveUnc(float left_speed, float right_speed) {
    int16_t left = (int16_t)round(left_speed * 5 / 11.9);
    int16_t right = (int16_t)round(right_speed * 5 / 11.9);
    mc1.setSpeed(2, -left);
    mc1.setSpeed(3, right);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    //setupWiFi();
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Setup button pin

    // Initialize Motoron controllers
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

    // Read initial button state
    int initState = digitalRead(BUTTON_PIN);
    last_steady_state = initState;
    last_flickerable_state = initState;
    last_debounce_time = millis();
}

void loop() {
    static unsigned long lastLoopTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
      lastLoopTime = currentTime;

      setDrive(0.0, 0.0);
      mc2.setSpeed(1, 800.0);
      mc2.setSpeed(2, 800.0);
      mc2.setSpeed(3, 800.0);

      // --- BUTTON HANDLING ---
      current_state = digitalRead(BUTTON_PIN);
      if (current_state != last_flickerable_state) {
        last_debounce_time = millis();
        last_flickerable_state = current_state;
      }
      if ((millis() - last_debounce_time) > DEBOUNCE_TIME) {
        if (last_steady_state == HIGH && current_state == LOW) {
            // Button press detected
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
        // pid.reset();
      }
    }
}
