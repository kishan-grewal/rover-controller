#include "motor_shield.h"

MotorShield::MotorShield(uint8_t i2c_address)
    : mc(i2c_address) {}

void MotorShield::begin() {
    Wire1.begin();  // Secondary I2C

    mc.reinitialize();
    mc.disableCrc();
    mc.clearResetFlag();

    // Configure motor 1
    mc.setMaxAcceleration(1, 140);
    mc.setMaxDeceleration(1, 300);

    // Configure motor 2
    mc.setMaxAcceleration(2, 140);
    mc.setMaxDeceleration(2, 300);
}

void MotorShield::setMotorSpeed(uint8_t motor, int16_t speed) {
    mc.setSpeed(motor, speed);
}
