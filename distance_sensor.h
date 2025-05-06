#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

class DistanceSensor {
public:
  // Initialise the Sharp IR distance sensor
  void begin();

  // Returns filtered distance in millimetres (uses SharpDistSensor library)
  float readDistance();
};

#endif
