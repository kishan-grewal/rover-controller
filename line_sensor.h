#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "qtr_sensor.h"

// Number of QTR sensors
const uint8_t NUM_SENSORS = 9;

// Sensor pins (digital)
extern const uint8_t qtrPins[NUM_SENSORS];

// Emitter control pin
extern const uint8_t emitterPin;

// QTRManual object
extern QTRSensor qtr;

// Setup and loop logic for line sensor
void setupLineSensor();
void loopLineSensor();

#endif
