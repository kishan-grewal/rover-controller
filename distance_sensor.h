#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <Arduino.h>
#include <Average.h>

class DistanceSensor {
public:
    DistanceSensor(uint8_t pin, float alpha = 0.1, size_t bufferSize = 100);

    void update();
    float getMean();

private:
    uint8_t analogPin;
    float alpha;
    float filtered;
    Average<float> ave;

    float readSmoothedVoltage();
    float calculateDistance(float voltage_mV);
};

#endif
