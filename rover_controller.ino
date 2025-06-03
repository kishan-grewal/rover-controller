#include <Arduino.h>
#include "distance_sensor.h"
#include "pid_controller.h"
#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc1(0x10);
MotoronI2C mc2(0x0B);

DistanceSensor sensorLB(A6);
DistanceSensor sensorLF(A5);
DistanceSensor sensorC(A4);
DistanceSensor sensorRF(A3);
DistanceSensor sensorRB(A2);
PIDController pid_distance(10.0, 0.0, 0.0);
PIDController pid_angle(0.0, 0.0, 0.0);  // Start with a small P gain

const int16_t MOTOR_SPEED_MIN = 300;
const int16_t MOTOR_SPEED_MAX = 500;
const float TARGET_DISTANCE = 8.0;
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
  int16_t left = (int16_t)round(left_speed * 5 / 10.905);
  int16_t right = (int16_t)round(right_speed * 5 / 10.905);
  mc1.setSpeed(2, -left);
  mc1.setSpeed(3, right);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

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

    int initState = digitalRead(BUTTON_PIN);
    last_steady_state = initState;
    last_flickerable_state = initState;
    last_debounce_time = millis();

    analogReadResolution(12);  // Use 12-bit resolution (0-4095)
}

void loop() {
    static unsigned long lastLoopTime = 0;
    static unsigned long lastPrintTime = 0;  // Track last print time
    unsigned long currentTime = millis();

    if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
        lastLoopTime = currentTime;

        sensorLF.update();
        sensorLB.update();
        //sensorC.update();

        float distance_error = TARGET_DISTANCE - sensorLF.getMean();
        float angle_error = sensorLB.getMean() - sensorLF.getMean();
        if (angle_error < 0) {
            angle_error *= 2;
        }

        
        // float center_distance = sensorC.getMean();
        // static bool corner_turning = false;

        // if (center_distance < 5.0) {
        //     // Corner detected, initiate turn
        //     corner_turning = true;
        //     setDriveUnc(800.0, -800.0);  // Turn in place
        //     Serial.println("Turning corner...");
        // }
        // else if (corner_turning) {
        //     // Corner clearing phase, check angle_error
        //     if (abs(angle_error) < 2.0) {
        //         // Finished turning, back to wall following
        //         corner_turning = false;
        //         pid_distance.reset();
        //         pid_angle.reset();
        //         Serial.println("Corner cleared, back to straight.");
        //     } else {
        //         // Still adjusting turn
        //         setDriveUnc(800.0, -800.0);
        //         Serial.println("Aligning after turn...");
        //     }
        // }
        // else 
        
        {
            // Normal PID wall following
            float correction_distance = pid_distance.compute(distance_error);
            float correction_angle = pid_angle.compute(angle_error);

            const float baseSpeed = (MOTOR_SPEED_MIN + MOTOR_SPEED_MAX) / 2.0;
            float left_speed = baseSpeed + correction_distance + correction_angle;
            float right_speed = baseSpeed - correction_distance - correction_angle;

            if (currentTime - lastPrintTime >= 500) {
                Serial.print("cd ");
                Serial.print(correction_distance);
                Serial.print(" ca ");
                Serial.println(correction_angle);
                //Serial.println(sensorLF.getMean());
                lastPrintTime = currentTime;
            }

            setDrive(left_speed, right_speed);
        }

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

        if (robot_enabled == false) {
            setDrive(0.0, 0.0);
            mc2.setSpeed(2, 0.0);
            mc2.setSpeed(3, 0.0);
            pid_distance.reset();
            pid_angle.reset();
        }
    }
}
