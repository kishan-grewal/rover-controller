#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
  public:
    MotorDriver(int in1Pin, int enaPin);
    void begin();
    void setSpeed(int speed);

  private:
    int in1;
    int ena;
};

#endif
