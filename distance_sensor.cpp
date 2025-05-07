#include "distance_sensor.h"
#include <math.h>

DistanceSensor::DistanceSensor(uint8_t pin, float alpha, size_t bufferSize)
  : analogPin(pin), alpha(alpha), filtered(0), ave(bufferSize) {}

void DistanceSensor::update() {
    static unsigned long lastCheckTime = 0;
    const unsigned long interval = 200; // 200 ms

    unsigned long currentTime = millis();

    if (currentTime - lastCheckTime >= interval) {
        float voltage_mV = readSmoothedVoltage();
        float distance = calculateDistance(voltage_mV);
        filtered = alpha * distance + (1 - alpha) * filtered;
        ave.push(filtered);
    }
}

float DistanceSensor::getMean() {
    return ave.mean();
}

float DistanceSensor::readSmoothedVoltage() {
    long sum = 0;
    const int samples = 16;
    for (int i = 0; i < samples; ++i) {
        sum += analogRead(analogPin);
        delayMicroseconds(200);
    }
    float avgADC = sum / float(samples);
    return map(avgADC, 0, 1023, 0, 5000);  // simulate 5V analog ref
}

float DistanceSensor::calculateDistance(float voltage_mV) {
    return 4600.5 * pow(voltage_mV, -0.941) + 1.0;
}
