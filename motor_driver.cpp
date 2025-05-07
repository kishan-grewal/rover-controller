#include "motor_driver.h"

MotorDriver::MotorDriver(int dirPin, int pwmPin) {
    dir = dirPin;
    pwm = pwmPin;
}

void MotorDriver::begin() {
    pinMode(dir, OUTPUT);
    pinMode(pwm, OUTPUT);
}

void MotorDriver::setSpeed(int speed) {
    static unsigned long lastCheckTime = 0;
    const unsigned long interval = 200; // 200 ms

    unsigned long currentTime = millis();

    if (currentTime - lastCheckTime >= interval) {
        lastCheckTime = currentTime;
        if (speed > 0) {
            digitalWrite(dir, HIGH);
            analogWrite(pwm, speed);
        }
        else if (speed < 0) {
            digitalWrite(dir, LOW);
            analogWrite(pwm, -speed);
        }
        else {
            digitalWrite(dir, LOW);
            analogWrite(pwm, 0);
        }
    }
}
