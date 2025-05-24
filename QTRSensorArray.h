#ifndef QTR_SENSOR_ARRAY_H
#define QTR_SENSOR_ARRAY_H

#include <Arduino.h>

class QTRSensorArray {
public:
    QTRSensorArray(const uint8_t sensorPins[9], uint8_t ledPin);

    void begin();
    void calibrate();

    void updateSensors(); // Reads raw and normalised values into internal arrays
    const uint16_t* getRaw() const;         // Get pointer to latest raw values
    const uint16_t* getNormalised() const;  // Get pointer to latest normalised values

    uint16_t readLineBlack(); // Uses internal normalised readings
    void printCalibration();  // Optional debug output
    bool isLineDetected(float confidence);

    void runAveragingPhase(uint16_t* outputArray);

private:
    uint16_t readRC(uint8_t pin);
    void readSensors(uint16_t *dest);
    void normalise(const uint16_t *raw, uint16_t *normalised); // Converts raw to 0–1000

    static const uint8_t NUM_SENSORS = 9;
    static const uint16_t TIMEOUT_US = 3000;
    static const uint16_t CALIB_TIMES = 3000;

    const uint8_t* _sensorPins;
    uint8_t _ledPin;

    uint16_t sensorMin[NUM_SENSORS];
    uint16_t sensorMax[NUM_SENSORS];
    uint16_t lastResult;

    uint16_t _raw[NUM_SENSORS];
    uint16_t _normalised[NUM_SENSORS];
};

#endif
