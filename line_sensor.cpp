#include "line_sensor.h"

const uint8_t qtrPins[NUM_SENSORS] = {
  D3, D4, D5,
  D6, D7, D8,
  D9, D10, D11
};

const uint8_t emitterPin = D2;

QTRSensor qtr(qtrPins, NUM_SENSORS, emitterPin, 10000, 50);

void setupLineSensor() {
  Serial.begin(115200);
  while (!Serial);

  qtr.begin();
  Serial.println("Calibration: place sensor over white/black surfaces...");
  delay(1000);
  qtr.calibrate(400);
  Serial.println("Calibration complete");
}

void loopLineSensor() {
  static unsigned long lastCheckTime = 0;
  const unsigned long interval = 200; // 200 ms

  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= interval) {
    uint16_t raw[NUM_SENSORS];
    uint16_t pos = qtr.readLine(raw);

    Serial.print("Raw: ");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print(raw[i]);
      Serial.print(i < NUM_SENSORS - 1 ? '\t' : '\n');
    }
    Serial.print("Pos: ");
    Serial.println(pos);
  }
}
