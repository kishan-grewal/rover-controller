#include <Arduino.h>
#include <SharpDistSensor.h>
#include "distance_sensor.h"

const byte sensorPin = A0;
const byte medianFilterWindowSize = 5;

SharpDistSensor sensor(sensorPin, medianFilterWindowSize);

void initDistanceSensor() {
  sensor.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
}

unsigned int readDistance() {
  return sensor.getDist();
}
