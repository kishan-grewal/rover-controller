// qtr_sensor.cpp
#include "qtr_sensor.h"

QTRSensor::QTRSensor(const uint8_t* pins, uint8_t count,
                     uint8_t emitterPin,
                     uint16_t timeout, uint16_t chargeTime)
  : _pins(pins), _count(count),
    _emitterPin(emitterPin),
    _timeout(timeout), _chargeTime(chargeTime),
    _lastPosition(0)
{
  _min = new uint16_t[_count];
  _max = new uint16_t[_count];
}

void QTRSensor::begin() {
  if (_emitterPin != 255) {
    pinMode(_emitterPin, OUTPUT);
    digitalWrite(_emitterPin, LOW);
  }
  for (uint8_t i = 0; i < _count; i++) {
    pinMode(_pins[i], INPUT);
    _min[i] = UINT16_MAX;
    _max[i] = 0;
  }
}

void QTRSensor::enableEmitters() {
  if (_emitterPin != 255) digitalWrite(_emitterPin, HIGH);
}

void QTRSensor::disableEmitters() {
  if (_emitterPin != 255) digitalWrite(_emitterPin, LOW);
}

uint16_t QTRSensor::readRC(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(_chargeTime);  // experiment

  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);

  uint16_t count = 0;
  uint32_t start = micros();
  while (digitalRead(pin) == HIGH) {
    if ((micros() - start) > _timeout) {
      count = _timeout;  // experiment
      break;
    }
    count++;
  }
  return count;
}

void QTRSensor::calibrate(uint16_t samples) {
  uint16_t* values = new uint16_t[_count];
  for (uint16_t s = 0; s < samples; s++) {
    read(values);
    for (uint8_t i = 0; i < _count; i++) {
      _min[i] = min(_min[i], values[i]);
      _max[i] = max(_max[i], values[i]);
    }
    delay(20);  // experiment
  }
  delete[] values;
}

void QTRSensor::read(uint16_t* values) {
  enableEmitters();
  for (uint8_t i = 0; i < _count; i++) {
    values[i] = readRC(_pins[i]);
  }
  disableEmitters();
}

uint16_t QTRSensor::readLine(uint16_t* values) {
  read(values);  // assumes this fills `values[]` with raw sensor data

  uint32_t weightedSum = 0;
  uint16_t sum = 0;

  for (uint8_t i = 0; i < _count; i++) {
    uint16_t v = values[i];

    // Map sensor values so black = 1000, white = 0
    v = constrain(v, _min[i], _max[i]);
    v = map(v, _min[i], _max[i], 0, 1000);  // black = 1000, white = 0
    values[i] = v;

    if (v > 200) {  // Ignore noise; adjust based on your surface contrast
      weightedSum += (uint32_t)v * i * 1000;
      sum += v;
    }
  }

  if (sum == 0) return _lastPosition;  // No line detected
  _lastPosition = weightedSum / sum;
  return _lastPosition;
}

float QTRSensor::getLineConvexity(const uint16_t* rawValues,
                                  uint16_t minValue,
                                  float centreWeight,
                                  float neighbourWeight) {
  if (_count < 3) return 0.0;  // Need at least 3 sensors

  uint16_t* v = new uint16_t[_count];
  for (uint8_t i = 0; i < _count; i++) {
    v[i] = constrain(rawValues[i], _min[i], _max[i]);
    v[i] = map(v[i], _min[i], _max[i], 0, 1000);
  }

  float maxCurvature = 0.0;
  for (uint8_t i = 1; i < _count - 1; i++) {
    if (v[i] < minValue) continue;  // Skip weak middle readings

    float curv = centreWeight * v[i] - neighbourWeight * (v[i - 1] + v[i + 1]);
    if (curv > maxCurvature) maxCurvature = curv;
  }

  delete[] v;
  return maxCurvature / 1000.0;  // Normalise to [0.0, 1.0]
}