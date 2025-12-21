#include <Joint.h>

Joint::Joint(Adafruit_PWMServoDriver* driver, JointConfig config) : _driver(driver), _cfg(config) {
}

void Joint::setAngle(float angle) {
    uint16_t pulse = degreesToPulse(constrain(angle, _cfg.minAngle, _cfg.maxAngle));
    _driver->setPWM(_cfg.id, 0, pulse);
}

uint16_t Joint::degreesToPulse(float angle) {
    angle = constrain(angle, _cfg.minAngle, _cfg.maxAngle);
    return (uint16_t)map(angle, _cfg.minAngle, _cfg.maxAngle, _cfg.minPulse, _cfg.maxPulse);
}