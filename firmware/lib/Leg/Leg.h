#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Joint.h>

struct LegConfig {
    JointConfig hip; // joint 1
    JointConfig knee; // joint 2

    float femurLength;  // link 1
    float tibiaLength;  // link 2
};

class Leg {
public:
    Leg(Adafruit_PWMServoDriver* driver, LegConfig config);

    void moveTo(float x, float y);

    void home();  // return to safe position

private:
    LegConfig _cfg;  // store settings in class

    Joint _hip;
    Joint _knee;
};
