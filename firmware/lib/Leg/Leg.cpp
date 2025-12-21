#include <Leg.h>

Leg::Leg(LegConfig config) : _cfg(config) {
}

void Leg::moveTo(float x, float y) {
    // put ik code here
}

void Leg::home() {
    moveTo(8.0, 11.5);
}

uint16_t Leg::degreesToPulse(float angle) {
    angle = constrain(angle, _cfg.minAngle, _cfg.maxAngle);
    return (uint16_t)map(angle, _cfg.minAngle, _cfg.maxAngle, _cfg.minPulse, _cfg.maxPulse);
}