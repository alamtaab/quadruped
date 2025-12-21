#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

struct JointConfig {
    uint8_t id;  // pwm driver channel
    uint16_t minPulse;
    uint16_t maxPulse;
    float minAngle;
    float maxAngle;  // physical limits (degrees)

    float offset;
    bool inverted;
};

class Joint {
public:

    Joint(Adafruit_PWMServoDriver* _driver, JointConfig config);

    void setAngle(float Angle);

    private:
    Adafruit_PWMServoDriver* _driver;
    JointConfig _cfg;

    uint16_t degreesToPulse(float angle);
};