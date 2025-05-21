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

void QTRSensorArray::calibrate(uint32_t durationMs, uint16_t delayMs)
{
    // Initialise min and max arrays
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = TIMEOUT_US;
        sensorMax[i] = 0;
    }

    unsigned long start = millis();

    while (millis() - start < durationMs) {
        uint16_t v[NUM_SENSORS];
        readSensors(v);

        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (v[i] < sensorMin[i]) sensorMin[i] = v[i];
            if (v[i] > sensorMax[i]) sensorMax[i] = v[i];
        }

        delay(delayMs); // user-defined delay
    }
}

uint16_t QTRSensorArray::readLineBlack(uint16_t *raw)
{
    readSensors(raw);
    uint16_t value[NUM_SENSORS];

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        int32_t span = (int32_t)sensorMax[i] - (int32_t)sensorMin[i];
        if (span <= 0) span = 1;

        int32_t delta = (int32_t)raw[i] - (int32_t)sensorMin[i];
        if (delta < 0) delta = 0;
        if (delta > span) delta = span;

        int32_t x = delta * 1000 / span;
        value[i] = constrain(x, 0, 1000);
    }

    uint32_t numerator = 0, denominator = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (value[i] > 10) {
            numerator += (uint32_t)value[i] * (i * 1000);
            denominator += value[i];
        }
    }

    if (denominator != 0) {
        lastResult = numerator / denominator;
    }

    return lastResult;
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

bool QTRSensorArray::isLineDetected(float confidence)
{
    uint16_t raw[NUM_SENSORS];
    readSensors(raw);

    // Normalise each sensor reading using the calibration values
    uint16_t values[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        int32_t span = (int32_t)sensorMax[i] - (int32_t)sensorMin[i];
        if (span <= 0) span = 1;

        int32_t delta = (int32_t)raw[i] - (int32_t)sensorMin[i];
        if (delta < 0) delta = 0;
        if (delta > span) delta = span;

        int32_t x = delta * 1000 / span;
        values[i] = constrain(x, 0, 1000);
    }

    // Compute average of the 9 values (each in range 0–1000)
    uint32_t total = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        total += values[i];
    }
    float average = total / (1000.0f * NUM_SENSORS); // Normalised to 0–1
    Serial.println(average);
    return average >= confidence;
}
