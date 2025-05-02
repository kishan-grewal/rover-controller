#include "motor_driver.h"

MotorDriver::MotorDriver(int in1Pin, int in2Pin, int enaPin) {
  in1 = in1Pin;
  in2 = in2Pin;
  ena = enaPin;
}

void MotorDriver::begin() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
}

void MotorDriver::setSpeed(int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(ena, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ena, 0);
  }
}
