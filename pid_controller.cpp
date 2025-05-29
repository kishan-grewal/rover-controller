#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd)
  : _kp(kp), _ki(ki), _kd(kd), _prevError(0), _integral(0), _lastTime(0), _lastDt(0.001) {}

float PIDController::compute(float error) {
  unsigned long now = millis();
  float dt = (now - _lastTime) / 1000.0; // convert ms to seconds

  // Use last valid dt if the new dt is zero or negative
  if (dt <= 0) dt = _lastDt;
  else _lastDt = dt;

  _lastTime = now;

  _integral += error * dt;
  float derivative = (error - _prevError) / dt;
  _prevError = error;

  return _kp * error + _ki * _integral + _kd * derivative;
}

void PIDController::reset() {
  _integral = 0;
  _prevError = 0;
  _lastTime = millis();
  _lastDt = 0.001;
}
