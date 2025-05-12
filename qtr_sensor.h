// qtr_sensor.h
#ifndef QTR_SENSOR_H
#define QTR_SENSOR_H

#include <Arduino.h>

class QTRSensor {
public:
  // pins: array of sensor pins; count: number of sensors
  // emitterPin: control pin for emitters, or 255 to disable
  // timeout: max RC count before giving up (µs)
  // chargeTime: LED charge pulse duration (µs)
  QTRSensor(const uint8_t* pins, uint8_t count,
            uint8_t emitterPin,
            uint16_t timeout = 10000, uint16_t chargeTime = 50);

  void begin();
  void calibrate(uint16_t samples = 100);
  void read(uint16_t* values);
  uint16_t readLine(uint16_t* values);

  float getLineConvexity(const uint16_t* rawValues,
                       uint16_t minValue = 600,
                       float centreWeight = 2.0,
                       float neighbourWeight = 1.0);

private:
  const uint8_t* _pins;
  uint8_t _count;
  uint8_t _emitterPin;
  uint16_t _timeout;
  uint16_t _chargeTime;

  uint16_t* _min;
  uint16_t* _max;
  uint16_t _lastPosition;

  uint16_t readRC(uint8_t pin);
  void enableEmitters();
  void disableEmitters();
};

#endif // QTR_SENSOR_H
