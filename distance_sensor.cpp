#include "distance_sensor.h"
#include <SharpDistSensor.h>

const byte sensorPin = A0;                    // Update if needed
const byte medianFilterWindowSize = 5;
SharpDistSensor sharp(sensorPin, medianFilterWindowSize);

void DistanceSensor::begin() {
  sharp.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);  // Match your actual sensor model
}

float DistanceSensor::readDistance() {
  return sharp.getDist();  // Returns distance in mm
}
