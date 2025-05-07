#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
  PIDController(float kp, float ki, float kd);
  float compute(float error);
  void reset();

private:
  float _kp;
  float _ki;
  float _kd;
  float _prevError;
  float _integral;
  unsigned long _lastTime;
  float _lastDt;  // store the previous dt
};

#endif // PID_CONTROLLER_H
