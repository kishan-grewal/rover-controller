#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd)
  : _kp(kp), _ki(ki), _kd(kd), _prevError(0), _integral(0) {}

float PIDController::compute(float error) {
  _integral += error;
  float derivative = error - _prevError;
  _prevError = error;
  return _kp * error + _ki * _integral + _kd * derivative;
}

void PIDController::reset() {
  _integral = 0;
  _prevError = 0;
}
