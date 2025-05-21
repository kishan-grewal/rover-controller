#ifndef QTR_SENSOR_ARRAY_H
#define QTR_SENSOR_ARRAY_H

#include <Arduino.h>

class QTRSensorArray {
public:
    QTRSensorArray(const uint8_t sensorPins[9], uint8_t ledPin);

    void begin();

    // Calibrate for a specified duration (ms) with a sampling delay (ms)
    void calibrate(uint32_t durationMs = 10000, uint16_t delayMs = 5);

    uint16_t readLineBlack(uint16_t *raw); // returns position, fills raw[]

    void printCalibration(); // Optional debug output

    bool isLineDetected(float confidence);

private:
    uint16_t readRC(uint8_t pin);
    void readSensors(uint16_t *dest);

    static const uint8_t NUM_SENSORS = 9;
    static const uint16_t TIMEOUT_US = 3000;

    const uint8_t* _sensorPins;
    uint8_t _ledPin;

    uint16_t sensorMin[NUM_SENSORS];
    uint16_t sensorMax[NUM_SENSORS];
    uint16_t lastResult;
};

#endif
