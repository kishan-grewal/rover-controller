#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

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
};

#endif // PID_CONTROLLER_H
