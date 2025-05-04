#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "qtr_sensor.h"
#include "motor_driver.h"
#include "pid_controller.h"

class LineFollower {
public:
  LineFollower(QTRSensor& qtr, MotorDriver& leftMotor, MotorDriver& rightMotor,
               float kp, float ki, float kd);

  void followLine();

private:
  QTRSensor& _qtr;
  MotorDriver& _leftMotor;
  MotorDriver& _rightMotor;
  PIDController _pid;
  const int _center = 4000;
  const int _baseSpeed = 100;
};

#endif // LINE_FOLLOWER_H
