#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
  public:
    MotorDriver(int dirPin, int pwmPin);
    void begin();
    void setSpeed(int speed);

  private:
    int dir;
    int pwm;
};

#endif
