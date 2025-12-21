#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Leg.h>
#include <Wire.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// servo consts
constexpr uint16_t SERVOMIN = 140;  // min pulse width microsec
constexpr uint16_t SERVOMAX = 560;  // max pulse width microsec
constexpr uint8_t FREQ = 50;        // servo frequency in Hz

// leg consts
constexpr float LINK1_LENGTH = 8.0f;
constexpr float LINK2_LENGTH = 11.5f;

const LegConfig FRONT_LEFT_CONF = {

    .hip = {.id = 0, .minPulse = SERVOMIN, .maxPulse = SERVOMAX, .minAngle = 0.0f, .maxAngle = 180.0f, .inverted = false},

    .knee = {.id = 1, .minPulse = SERVOMIN, .maxPulse = SERVOMAX, .minAngle = 25.0f, .maxAngle = 160.0f, .inverted = false},

    .femurLength = LINK1_LENGTH,
    .tibiaLength = LINK2_LENGTH};

Leg testLeg(&pwm, FRONT_LEFT_CONF);

// leg state
float footX = 0;
float footY = 0;

void setup() {
    Serial.begin(115200);

    delay(1000);
    Serial.println("Initialized. \n");

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // 25MHz internal clock but ~27MHz actual reccomended by manufacturer
    pwm.setPWMFreq(FREQ);

    Serial.println("Homing...");
    testLeg.home();
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        int separator = input.indexOf(':');
        if (separator > 0) {
            float x = input.substring(0, separator).toFloat();
            float y = input.substring(separator + 1).toFloat();

            Serial.print("Moving to: ");
            Serial.print(x);
            Serial.print(", ");
            Serial.println(y);

            testLeg.moveTo(x, y);
        }
    }
    delay(10);
}