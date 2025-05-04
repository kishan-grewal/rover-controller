#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

class DistanceSensor {
public:
  void begin();           // Initialise Sharp sensor
  float readDistance();   // Returns distance in mm
};

#endif
