#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include "distance_sensor.h"
#include "motor_driver.h"
#include "pid_controller.h"

class WallFollower {
public:
  WallFollower(DistanceSensor& sensor, MotorDriver& leftMotor, MotorDriver& rightMotor,
               float kp, float ki, float kd);

  void followWall(float targetDistance);

private:
  DistanceSensor& _sensor;
  MotorDriver& _leftMotor;
  MotorDriver& _rightMotor;
  PIDController _pid;
  const int _baseSpeed = 100;
};

#endif // WALL_FOLLOWER_H
