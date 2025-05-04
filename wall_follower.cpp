#include "wall_follower.h"

WallFollower::WallFollower(DistanceSensor& sensor, MotorDriver& leftMotor, MotorDriver& rightMotor,
                           float kp, float ki, float kd)
  : _sensor(sensor), _leftMotor(leftMotor), _rightMotor(rightMotor), _pid(kp, ki, kd) {}

void WallFollower::followWall(float targetDistance) {
  float distance = _sensor.readDistance();
  float error = targetDistance - distance;
  float correction = _pid.compute(error);

  int leftSpeed = _baseSpeed + correction;
  int rightSpeed = _baseSpeed - correction;

  _leftMotor.setSpeed(leftSpeed);
  _rightMotor.setSpeed(rightSpeed);
}
#include "wall_follower.h"

WallFollower::WallFollower(DistanceSensor& sensor, MotorDriver& leftMotor, MotorDriver& rightMotor,
                           float kp, float ki, float kd)
  : _sensor(sensor), _leftMotor(leftMotor), _rightMotor(rightMotor), _pid(kp, ki, kd) {}

void WallFollower::followWall(float targetDistance) {
  float distance = _sensor.readDistance();
  float error = targetDistance - distance;
  float correction = _pid.compute(error);

  int leftSpeed = _baseSpeed + correction;
  int rightSpeed = _baseSpeed - correction;

  _leftMotor.setSpeed(leftSpeed);
  _rightMotor.setSpeed(rightSpeed);
}
