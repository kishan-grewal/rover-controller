#include "mission_logic.h"
#include <Wire.h>
#include <Motoron.h>

MissionLogic::MissionLogic(QTRSensorArray* qtr1, QTRSensorArray* qtr2, 
                           PIDController* pidLine, PIDController* pidWall, 
                           DistanceSensor* distLeft, DistanceSensor* distRight, DistanceSensor* distFront,
                           MotoronI2C* mcDrive, MotoronI2C* mcMech)
  : _qtr1(qtr1), _qtr2(qtr2), _pidLine(pidLine), _pidWall(pidWall), 
    _distLeft(distLeft), _distRight(distRight), _distFront(distFront),
    _mcDrive(mcDrive), _mcMech(mcMech),
    _maxSpeedDrive(600), _maxSpeedMech(400),
    _lastMissionTime(0) {}

void MissionLogic::lineFollowing() {
  _lastMissionTime = millis();

  static uint16_t values[9];
  uint16_t position = _qtr1->readLineBlack(values);
  int error = 4000 - position;
  float correction = _pidLine->compute(error);

  int speed = 150;
  int leftSpeed = constrain(speed - correction, -_maxSpeedDrive, _maxSpeedDrive);
  int rightSpeed = constrain(speed + correction, -_maxSpeedDrive, _maxSpeedDrive);
  _mcDrive->setSpeed(2, leftSpeed);
  _mcDrive->setSpeed(3, -rightSpeed);
}

void MissionLogic::wallFollowing() {
  _lastMissionTime = millis();

  float error = 10.0 - _distRight->getMean();
  float correction = _pidWall->compute(error);
  int leftSpeed = constrain(150 - correction, -_maxSpeedDrive, _maxSpeedDrive);
  int rightSpeed = constrain(150 + correction, -_maxSpeedDrive, _maxSpeedDrive);
  _mcDrive->setSpeed(2, leftSpeed);
  _mcDrive->setSpeed(3, -rightSpeed);
}

void MissionLogic::tunnelNavigation() {
  _lastMissionTime = millis();

  float error = 15.0 - _distFront->getMean();
  float correction = _pidWall->compute(error);
  int speed = 120;
  int leftSpeed = constrain(speed - correction, -_maxSpeedDrive, _maxSpeedDrive);
  int rightSpeed = constrain(speed + correction, -_maxSpeedDrive, _maxSpeedDrive);
  _mcDrive->setSpeed(2, leftSpeed);
  _mcDrive->setSpeed(3, -rightSpeed);
}

void MissionLogic::treadmillDrive() {
  _lastMissionTime = millis();

  int speed = 200;
  _mcDrive->setSpeed(2, speed);
  _mcDrive->setSpeed(3, -speed);
}

void MissionLogic::lunarSurfaceDrive() {
  _lastMissionTime = millis();

  int speed = 135;
  _mcDrive->setSpeed(2, speed);
  _mcDrive->setSpeed(3, -speed);
}

void MissionLogic::floorIsLava() {
  _lastMissionTime = millis();

  if (_qtr2->isLineDetected(0.2)) {
    static uint16_t values[9];
    uint16_t position = _qtr2->readLineBlack(values);
    int error = 4000 - position;
    float correction = _pidLine->compute(error);
    int leftSpeed = constrain(150 - correction, -_maxSpeedDrive, _maxSpeedDrive);
    int rightSpeed = constrain(150 + correction, -_maxSpeedDrive, _maxSpeedDrive);
    _mcDrive->setSpeed(2, leftSpeed);
    _mcDrive->setSpeed(3, -rightSpeed);
  } else {
    _mcDrive->setSpeed(2, 150);
    _mcDrive->setSpeed(3, -150);
  }
}

void MissionLogic::staircaseClimb(int stepHeight) {
  _lastMissionTime = millis();

  float factor = 1.0;
  if (stepHeight == 30) factor = 1.2;
  else if (stepHeight == 50) factor = 1.4;
  int speed = 150 * factor;
  _mcDrive->setSpeed(2, speed);
  _mcDrive->setSpeed(3, -speed);
}

void MissionLogic::ziplineEscape() {
  _lastMissionTime = millis();

  _mcDrive->setSpeed(2, 0);
  _mcDrive->setSpeed(3, 0);
  // digitalWrite(ZIPLINE_PIN, HIGH); // Placeholder for zipline trigger
}