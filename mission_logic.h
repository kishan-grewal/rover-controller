#ifndef MISSION_LOGIC_H
#define MISSION_LOGIC_H

#include <Arduino.h>
#include "QTRSensorArray.h"
#include "pid_controller.h"
#include "distance_sensor.h"
#include <Motoron.h>

class MissionLogic {
public:
    MissionLogic(QTRSensorArray* qtr1, QTRSensorArray* qtr2, 
                 PIDController* pidLine, PIDController* pidWall, 
                 DistanceSensor* distLeft, DistanceSensor* distRight, DistanceSensor* distFront,
                 MotoronI2C* mcDrive, MotoronI2C* mcMech);

    void lineFollowing();
    void wallFollowing();
    void tunnelNavigation();
    void treadmillDrive();
    void lunarSurfaceDrive();
    void floorIsLava();
    void staircaseClimb(int stepHeight);
    void ziplineEscape();

private:
    QTRSensorArray* _qtr1;
    QTRSensorArray* _qtr2;
    PIDController* _pidLine;
    PIDController* _pidWall;
    DistanceSensor* _distLeft;
    DistanceSensor* _distRight;
    DistanceSensor* _distFront;
    MotoronI2C* _mcDrive;
    MotoronI2C* _mcMech;

    const int _maxSpeedDrive;
    const int _maxSpeedMech;
    unsigned long _lastMissionTime;
};

#endif // MISSION_LOGIC_H
