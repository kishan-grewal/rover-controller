#include "distance_sensor.h"
#include <SharpDistSensor.h>

const byte sensorPin = A0;                    // Adjust if using a different pin
const byte medianFilterWindowSize = 5;
SharpDistSensor sharp(sensorPin, medianFilterWindowSize);

void DistanceSensor::begin() {
  // Use the official Sharp GP2Y0A51SK0F 5V model from the library
  sharp.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
}

float DistanceSensor::readDistance() {
  return sharp.getDist();  // Returns distance in mm, filtered
}
