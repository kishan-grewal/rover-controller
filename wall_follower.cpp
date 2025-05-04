#include "wall_follower.h"

WallFollower::WallFollower(DistanceSensor& sensor, MotorDriver& leftMotor, MotorDriver& rightMotor,
                           float kp, float ki, float kd)
  : _sensor(sensor), _leftMotor(leftMotor), _rightMotor(rightMotor), _pid(kp, ki, kd) {}

void WallFollower::followWall(float targetDistanceMm) {
  float currentDistance = _sensor.readDistance();  // in mm
  float error = targetDistanceMm - currentDistance;
  float correction = _pid.compute(error);

  // Limit correction to prevent wild turns
  correction = constrain(correction, -_baseSpeed, _baseSpeed);

  int leftSpeed = _baseSpeed + correction;
  int rightSpeed = _baseSpeed - correction;

  // Clamp motor outputs
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  _leftMotor.setSpeed(leftSpeed);
  _rightMotor.setSpeed(rightSpeed);
}
