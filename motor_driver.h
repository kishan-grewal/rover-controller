#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
  private:
    int in1, in2, ena;

  public:
    MotorDriver(int in1Pin, int in2Pin, int enaPin);
    void begin();
    void setSpeed(int speed);
};

#endif
