#include "QTRSensorArray.h"

QTRSensorArray::QTRSensorArray(const uint8_t sensorPins[9], uint8_t ledPin)
    : _sensorPins(sensorPins), _ledPin(ledPin), lastResult(4000)
{
}

void QTRSensorArray::begin()
{
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, HIGH); // Turn emitters on
}

uint16_t QTRSensorArray::readRC(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    pinMode(pin, INPUT);

    uint32_t t0 = micros();
    while (digitalRead(pin)) {
        if (micros() - t0 > TIMEOUT_US) return TIMEOUT_US;
    }
    return micros() - t0;
}

void QTRSensorArray::readSensors(uint16_t *dest)
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        dest[i] = readRC(_sensorPins[i]);
    }
}

void QTRSensorArray::normalise(const uint16_t *raw, uint16_t *normalised)
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        int32_t span = (int32_t)sensorMax[i] - (int32_t)sensorMin[i];
        if (span <= 0) span = 1;

        int32_t delta = (int32_t)raw[i] - (int32_t)sensorMin[i];
        if (delta < 0) delta = 0;
        if (delta > span) delta = span;

        int32_t x = delta * 1000 / span;
        normalised[i] = constrain(x, 0, 1000);
    }
}

void QTRSensorArray::updateSensors()
{
    readSensors(_raw);
    normalise(_raw, _normalised);
}

const uint16_t* QTRSensorArray::getRaw() const
{
    return _raw;
}

const uint16_t* QTRSensorArray::getNormalised() const
{
    return _normalised;
}

uint16_t QTRSensorArray::readLineBlack()
{
    uint32_t numerator = 0, denominator = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (_normalised[i] > 100) {
            numerator += (uint32_t)_normalised[i] * (i * 1000);
            denominator += _normalised[i];
        }
    }

    if (denominator != 0) {
        lastResult = numerator / denominator;
    }

    return lastResult;
}

void QTRSensorArray::calibrate()
{
    // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //     sensorMin[i] = TIMEOUT_US;
    //     sensorMax[i] = 0;
    // }

    // for (uint16_t n = 0; n < CALIB_TIMES; n++) {
    //     uint16_t v[NUM_SENSORS];
    //     readSensors(v);
    //     for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    //         if (v[i] < sensorMin[i]) sensorMin[i] = v[i];
    //         if (v[i] > sensorMax[i]) sensorMax[i] = v[i];
    //     }
    //     delay(5);
    // }

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 45;
        sensorMax[i] = 600;
    }
}

void QTRSensorArray::calibrate_time(unsigned long duration_ms) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = TIMEOUT_US;
        sensorMax[i] = 0;
    }

    unsigned long startTime = millis();
    while (millis() - startTime < duration_ms) {
        uint16_t v[NUM_SENSORS];
        readSensors(v);
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (v[i] < sensorMin[i]) sensorMin[i] = v[i];
            if (v[i] > sensorMax[i]) sensorMax[i] = v[i];
        }
        delay(5);
    }
}

void QTRSensorArray::calibrateFast()
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 50;
        sensorMax[i] = 700;
    }
}

void QTRSensorArray::calibrateDark()
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 25;
        sensorMax[i] = 350;
    }
}

bool QTRSensorArray::isLineDetected(float confidence)
{
    // Compute average of the 9 values (each in range 0–1000)
    uint32_t total = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        total += _normalised[i];
    }
    float average = total / (1000.0f * NUM_SENSORS); // Normalised to 0–1
    return average >= confidence;
}

void QTRSensorArray::printCalibration()
{
    Serial.println("\nMIN");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorMin[i]);
        Serial.print(",");
    }

    Serial.println("\n\nMAX");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorMax[i]);
        Serial.print(",");
    }
    Serial.println();
}
