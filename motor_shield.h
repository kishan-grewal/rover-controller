#ifndef MOTOR_SHIELD_H
#define MOTOR_SHIELD_H

#include <Wire.h>
#include <Motoron.h>

class MotorShield {
public:
    explicit MotorShield(uint8_t i2c_address);
    void begin();  // Call in setup()
    void setMotorSpeed(uint8_t motor, int16_t speed);

private:
    MotoronI2C mc;
};

#endif
