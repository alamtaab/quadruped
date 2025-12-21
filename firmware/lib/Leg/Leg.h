#pragma once
#include <Arduino.h>

struct LegConfig {
    uint8_t hipPin;
    uint8_t kneePin;  // corresponding pwm driver pins
    uint16_t minPulse;
    uint16_t maxPulse;  // maximum pulse widths in microseconds
    float minAngle;
    float maxAngle;  // mechanical limits of joint
    float offsetAngle;
};

class Leg {
public:
    Leg(LegConfig config);

    void moveTo(float x, float y);

    void home();  // return to safe position

private:
    LegConfig _cfg;  // store settings in class
    uint16_t degreesToPulse(float angle);
};
