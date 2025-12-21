#include <Leg.h>

Leg::Leg(Adafruit_PWMServoDriver* driver, LegConfig config) : _cfg(config), _hip(driver, config.hip), _knee(driver, config.knee) {
}

void Leg::moveTo(float x, float y) {
    // put ik code here

    float L1 = _cfg.femurLength;
    float L2 = _cfg.tibiaLength;

    float r2 = x * x + y * y;  // distance to end effector from origin

    float maxReach = L1 + L2;

    float distance = sqrt(r2);
    if (distance > maxReach) {
        x *= maxReach / distance; // scale to avoid reaching for impossible points
        y *= maxReach / distance;
    }

    float innerAngle = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    innerAngle = constrain(innerAngle, -1.0f, 1.0f);

    // knee angle (IK)
    // 0 degs = leg fully extended, + = flexion
    float kneeRadians = acos(innerAngle);

    // hip angle (IK)
    // angle of femur ccw from +X axis
    float hipRadians = atan2(y, x) - atan2(L2 * sin(kneeRadians), L1 + L2 * cos(kneeRadians));

    float kneeDegrees = kneeRadians * (180.0 / M_PI);

    // hip servo is mounted such that servo 90deg = leg pointing down
    // + rotation is CW
    // ik angle is sign inverted such that CCW -> CW
    float hipDegrees = -hipRadians * (180.0f / M_PI);

    kneeDegrees = constrain(kneeDegrees, 25, 160);

    _hip.setAngle(hipDegrees);
    _knee.setAngle(kneeDegrees);

    // Serial.print("Angle1: ");
    // Serial.print(hipDegrees);
    // Serial.print(" Angle2: ");
    // Serial.println(kneeDegrees);
    // Serial.print("X: ");
    // Serial.print(x);
    // Serial.print(" Y: ");
    // Serial.println(y);
}

void Leg::home() {
    moveTo(_cfg.femurLength, _cfg.tibiaLength);
}
