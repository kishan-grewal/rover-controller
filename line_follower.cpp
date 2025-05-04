#include "line_follower.h"

LineFollower::LineFollower(QTRSensor& qtr, MotorDriver& leftMotor, MotorDriver& rightMotor,
                           float kp, float ki, float kd)
  : _qtr(qtr), _leftMotor(leftMotor), _rightMotor(rightMotor), _pid(kp, ki, kd) {}

void LineFollower::followLine() {
  uint16_t values[9];
  int pos = _qtr.readLine(values);
  float error = _center - pos;
  float correction = _pid.compute(error);

  int leftSpeed = _baseSpeed - correction;
  int rightSpeed = _baseSpeed + correction;

  _leftMotor.setSpeed(leftSpeed);
  _rightMotor.setSpeed(rightSpeed);
}
